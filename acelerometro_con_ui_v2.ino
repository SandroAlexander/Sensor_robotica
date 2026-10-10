#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <Preferences.h>

// Registros del MMA8451 / MMA8452
#define MMA8451_REG_OUT_X_MSB    0x01
#define MMA8451_REG_WHOAMI       0x0D
#define MMA8451_REG_XYZ_DATA_CFG 0x0E
#define MMA8451_REG_CTRL_REG1    0x2A

uint8_t mmaAddress = 0x1C; // Dirección I2C predeterminada (se autodetectará entre 0x1C y 0x1D)

// Nombre del Access Point (SSID) y contraseña vacía (red abierta)
const char* ssid = "el_acelerometro";
const char* password = "";

// Configuración DNS para redireccionar la URL personalizada "acelerometro.com"
const byte DNS_PORT = 53;
DNSServer dnsServer;
IPAddress apIP(192, 168, 4, 1);
IPAddress netMsk(255, 255, 255, 0);

// Servidor Web en puerto 80
WebServer server(80);

// Estructura para almacenar las lecturas procesadas
struct SensorData {
  float x;
  float y;
  float z;
  float pitch;
  float roll;
  bool isConnected;
  uint8_t rawBytes[6]; // Guardar bytes crudos para depuración
  uint8_t i2cError;    // Estado de la última transmisión I2C
} currentData;

// ======================================================================
//  CALIBRACIÓN DE ORIENTACIÓN
// ======================================================================
// Matriz de rotación 3x3 que lleva la lectura cruda del sensor al sistema
// de referencia "calibrado". Por defecto es la identidad (sin calibrar).
//
// Convención del sistema calibrado:
//   +Z apunta al cielo, -Z apunta hacia el planeta (la gravedad va hacia -Z).
//   Un acelerómetro en reposo mide la reacción a la gravedad, es decir,
//   +1 g sobre el eje Z (igual que en la versión original del programa).
float calR[3][3] = {
  {1, 0, 0},
  {0, 1, 0},
  {0, 0, 1}
};
bool calibrated = false;

enum CalState : uint8_t {
  CAL_IDLE = 0,        // Sin calibración en curso
  CAL_WAIT = 1,        // Cuenta atrás para que el usuario suelte el sensor
  CAL_SAMPLING = 2,    // Recolectando muestras con el sensor quieto
  CAL_OK = 3,          // Calibración completada
  CAL_ERR_MOTION = 4,  // Se detectó movimiento durante la medición
  CAL_ERR_MAG = 5      // La magnitud medida no se parece a 1 g
};

CalState calState = CAL_IDLE;
unsigned long calStartMs = 0;

const unsigned long CAL_WAIT_MS   = 3000;  // Tiempo de espera antes de medir
const uint16_t      CAL_SAMPLES   = 100;   // Muestras a promediar (~1 s)
const float         CAL_MAX_RANGE = 0.05f; // Variación máx. permitida por eje (g)
const float         CAL_MIN_G     = 0.85f; // Magnitud mínima aceptada (g)
const float         CAL_MAX_G     = 1.15f; // Magnitud máxima aceptada (g)

uint16_t calCount = 0;
float calSum[3], calMin[3], calMax[3];

Preferences prefs;

void resetCalAccumulators() {
  calCount = 0;
  for (int i = 0; i < 3; i++) {
    calSum[i] = 0;
    calMin[i] = 1e9f;
    calMax[i] = -1e9f;
  }
}

void setIdentityCalibration() {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      calR[i][j] = (i == j) ? 1.0f : 0.0f;
}

void saveCalibration() {
  prefs.begin("accel", false);
  prefs.putBytes("R", calR, sizeof(calR));
  prefs.putBool("cal", true);
  prefs.end();
}

void loadCalibration() {
  prefs.begin("accel", false);
  if (prefs.getBool("cal", false) && prefs.getBytesLength("R") == sizeof(calR)) {
    prefs.getBytes("R", calR, sizeof(calR));
    calibrated = true;
  }
  prefs.end();
}

void clearCalibration() {
  prefs.begin("accel", false);
  prefs.clear();
  prefs.end();
  setIdentityCalibration();
  calibrated = false;
}

// Calcula la rotación mínima (fórmula de Rodrigues) que lleva el vector
// medido en reposo (gx, gy, gz) hacia el eje +Z = (0, 0, 1).
void computeRotationToZ(float gx, float gy, float gz) {
  float n = sqrtf(gx * gx + gy * gy + gz * gz);
  float u[3] = { gx / n, gy / n, gz / n };
  float c = u[2]; // u · z

  // Sensor boca abajo (u ≈ -Z): rotación de 180° sobre el eje X
  if (c < -0.9999f) {
    setIdentityCalibration();
    calR[1][1] = -1.0f;
    calR[2][2] = -1.0f;
    return;
  }

  // v = u × z
  float v[3] = { u[1], -u[0], 0.0f };
  float v2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
  float K[3][3] = {
    {  0.0f, -v[2],  v[1] },
    {  v[2],  0.0f, -v[0] },
    { -v[1],  v[0],  0.0f }
  };
  float k = 1.0f / (1.0f + c);

  // R = I + K + k * (v·vᵀ - |v|²·I)
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      float id = (i == j) ? 1.0f : 0.0f;
      calR[i][j] = id + K[i][j] + k * (v[i] * v[j] - id * v2);
    }
  }
}

// Se llama al terminar de recolectar las muestras
void finishCalibration() {
  float mean[3];
  for (int i = 0; i < 3; i++) mean[i] = calSum[i] / calCount;

  // 1. ¿Estuvo quieto el sensor?
  for (int i = 0; i < 3; i++) {
    if ((calMax[i] - calMin[i]) > CAL_MAX_RANGE) {
      calState = CAL_ERR_MOTION;
      Serial.println("[CALIBRACIÓN] Fallida: se detectó movimiento.");
      return;
    }
  }

  // 2. ¿La magnitud es coherente con la gravedad (~1 g)?
  float mag = sqrtf(mean[0] * mean[0] + mean[1] * mean[1] + mean[2] * mean[2]);
  if (mag < CAL_MIN_G || mag > CAL_MAX_G) {
    calState = CAL_ERR_MAG;
    Serial.print("[CALIBRACIÓN] Fallida: magnitud medida = ");
    Serial.print(mag, 3);
    Serial.println(" g");
    return;
  }

  // 3. Calcular, guardar y activar la nueva orientación
  computeRotationToZ(mean[0], mean[1], mean[2]);
  saveCalibration();
  calibrated = true;
  calState = CAL_OK;

  Serial.print("[CALIBRACIÓN] Completada. Gravedad medida (sensor): ");
  Serial.print(mean[0], 3); Serial.print(", ");
  Serial.print(mean[1], 3); Serial.print(", ");
  Serial.println(mean[2], 3);
}

// Procesa una muestra cruda (en g, sin calibrar) si hay calibración en curso
void processCalibrationSample(const float g[3]) {
  if (calState == CAL_WAIT) {
    if (millis() - calStartMs >= CAL_WAIT_MS) {
      resetCalAccumulators();
      calState = CAL_SAMPLING;
    } else {
      return;
    }
  }
  if (calState != CAL_SAMPLING) return;

  for (int i = 0; i < 3; i++) {
    calSum[i] += g[i];
    if (g[i] < calMin[i]) calMin[i] = g[i];
    if (g[i] > calMax[i]) calMax[i] = g[i];
  }
  calCount++;

  if (calCount >= CAL_SAMPLES) finishCalibration();
}

// ======================================================================
//  I2C
// ======================================================================

// Función auxiliar para escribir un registro I2C con retorno de estado
uint8_t writeRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mmaAddress);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(); // Retorna 0 si tuvo éxito
}

// Función auxiliar para leer un registro I2C
uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(mmaAddress);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) return 0;
  Wire.requestFrom((uint8_t)mmaAddress, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

// Función auxiliar para leer varios registros seguidos y capturar errores I2C
uint8_t readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len) {
  Wire.beginTransmission(mmaAddress);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) return err; // Si falla el envio de direccion, retorna el error
  
  uint8_t count = Wire.requestFrom((uint8_t)mmaAddress, len);
  if (count < len) return 4; // Error: no devolvió los bytes esperados
  
  for (uint8_t i = 0; i < len; i++) {
    buffer[i] = Wire.read();
  }
  return 0; // Exito
}

// Escáner I2C para diagnosticar pines D21 y D22 en la consola
void scanI2C() {
  Serial.println("\n--- [DIAGNÓSTICO] Escaneando Bus I2C (SDA=D21, SCL=D22) ---");
  byte count = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("  [OK] Dispositivo I2C encontrado en la dirección: 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      if (address == 0x1C || address == 0x1D) {
        Serial.print(" <- Posible MMA8451/GY-45!");
      }
      Serial.println();
      count++;
    }
  }
  if (count == 0) {
    Serial.println("  [ERROR CRÍTICO] No se detectó NINGÚN dispositivo I2C.");
    Serial.println("  -> Revisa que SDA esté en D21, SCL en D22, VCC a 3.3V y GND conectado.");
  }
  Serial.println("-----------------------------------------------------------\n");
}

// Inicializar el acelerómetro mediante comandos I2C nativos
bool initMMA8451Direct() {
  // Intentar autodetectar la dirección I2C (0x1C o 0x1D)
  mmaAddress = 0x1C;
  uint8_t whoami = readRegister(MMA8451_REG_WHOAMI);
  
  if (whoami != 0x1A && whoami != 0x2A) { // 0x1A = MMA8451, 0x2A = MMA8452
    mmaAddress = 0x1D;
    whoami = readRegister(MMA8451_REG_WHOAMI);
  }

  Serial.print("Respuesta WHO_AM_I (0x0D): 0x");
  Serial.print(whoami, HEX);
  Serial.print(" en dirección 0x");
  Serial.println(mmaAddress, HEX);

  if (whoami != 0x1A && whoami != 0x2A) {
    return false; // No se encontró un chip compatible
  }

  // 1. Pasar a modo Standby para permitir configuración
  writeRegister(MMA8451_REG_CTRL_REG1, 0x00);
  delay(10);

  // 2. Configurar rango a +/- 2g
  writeRegister(MMA8451_REG_XYZ_DATA_CFG, 0x00);
  delay(10);

  // 3. Activar el sensor (Bit 0 ACTIVE = 1, ODR 800Hz = Fast Read desactivado)
  uint8_t err = writeRegister(MMA8451_REG_CTRL_REG1, 0x01);
  if (err != 0) {
    Serial.print("Error al activar modo ACTIVO. Código I2C: ");
    Serial.println(err);
    return false;
  }

  return true;
}

// ======================================================================
//  INTERFAZ WEB
// ======================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Acelerómetro ESP32</title>
    <style>
        :root {
            --bg: #0f172a;
            --card-bg: rgba(30, 41, 59, 0.7);
            --accent-x: #ef4444;
            --accent-y: #10b981;
            --accent-z: #3b82f6;
            --text-primary: #f8fafc;
            --text-secondary: #94a3b8;
        }

        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
            font-family: 'Segoe UI', system-ui, -apple-system, sans-serif;
        }

        body {
            background-color: var(--bg);
            color: var(--text-primary);
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            align-items: center;
            padding: 1.5rem;
        }

        header {
            text-align: center;
            margin-bottom: 1.5rem;
            width: 100%;
            max-width: 800px;
        }

        header h1 {
            font-size: 1.8rem;
            font-weight: 700;
            letter-spacing: -0.025em;
            color: #ffffff;
        }

        header p {
            color: var(--text-secondary);
            font-size: 0.9rem;
            margin-top: 0.25rem;
        }

        .status-badge {
            display: inline-flex;
            align-items: center;
            gap: 0.5rem;
            background: rgba(16, 185, 129, 0.1);
            color: #34d399;
            padding: 0.35rem 0.8rem;
            border-radius: 9999px;
            font-size: 0.8rem;
            font-weight: 600;
            margin-top: 0.75rem;
            border: 1px solid rgba(16, 185, 129, 0.2);
        }

        .status-dot {
            width: 8px;
            height: 8px;
            background-color: #34d399;
            border-radius: 50%;
            box-shadow: 0 0 8px #34d399;
        }

        /* ---------- Barra de controles ---------- */
        .toolbar {
            display: flex;
            flex-wrap: wrap;
            align-items: center;
            justify-content: space-between;
            gap: 1rem;
            width: 100%;
            max-width: 800px;
            margin-bottom: 1.25rem;
        }

        .segmented {
            display: inline-flex;
            background: rgba(255, 255, 255, 0.06);
            border: 1px solid rgba(255, 255, 255, 0.08);
            border-radius: 10px;
            padding: 3px;
        }

        .segmented button {
            background: transparent;
            border: none;
            color: var(--text-secondary);
            padding: 0.45rem 1rem;
            border-radius: 8px;
            font-size: 0.85rem;
            font-weight: 600;
            cursor: pointer;
            transition: background 0.15s ease, color 0.15s ease;
        }

        .segmented button.active {
            background: var(--accent-z);
            color: #fff;
        }

        .cal-group {
            display: flex;
            align-items: center;
            gap: 0.75rem;
            flex-wrap: wrap;
        }

        .cal-status {
            font-size: 0.78rem;
            color: var(--text-secondary);
        }

        .cal-status.ok { color: #34d399; }

        .btn {
            border: 1px solid rgba(255, 255, 255, 0.15);
            background: rgba(255, 255, 255, 0.08);
            color: var(--text-primary);
            padding: 0.5rem 1rem;
            border-radius: 10px;
            font-size: 0.85rem;
            font-weight: 600;
            cursor: pointer;
            transition: background 0.15s ease;
        }

        .btn:hover { background: rgba(255, 255, 255, 0.16); }

        .btn.primary {
            background: var(--accent-z);
            border-color: var(--accent-z);
            color: #fff;
        }

        .btn.primary:hover { background: #2563eb; }

        .btn.link {
            background: transparent;
            border: none;
            color: var(--text-secondary);
            font-weight: 400;
            text-decoration: underline;
            padding: 0.5rem 0.25rem;
        }

        .grid-container {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
            gap: 1.25rem;
            width: 100%;
            max-width: 800px;
        }

        .card {
            background: var(--card-bg);
            backdrop-filter: blur(12px);
            border: 1px solid rgba(255, 255, 255, 0.08);
            border-radius: 16px;
            padding: 1.5rem;
            box-shadow: 0 10px 25px -5px rgba(0, 0, 0, 0.3);
            transition: transform 0.2s ease, border-color 0.2s ease;
        }

        .card:hover {
            transform: translateY(-2px);
            border-color: rgba(255, 255, 255, 0.18);
        }

        .axis-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 0.75rem;
        }

        .axis-title {
            font-size: 0.85rem;
            text-transform: uppercase;
            letter-spacing: 0.05em;
            color: var(--text-secondary);
            font-weight: 600;
        }

        .axis-value {
            font-size: 2rem;
            font-weight: 800;
            font-variant-numeric: tabular-nums;
        }

        .unit {
            font-size: 0.9rem;
            color: var(--text-secondary);
            font-weight: 400;
            margin-left: 0.2rem;
        }

        .bar-bg {
            width: 100%;
            height: 8px;
            background: rgba(255, 255, 255, 0.1);
            border-radius: 4px;
            overflow: hidden;
            margin-top: 1rem;
            position: relative;
        }

        .bar-fill {
            height: 100%;
            width: 50%;
            position: absolute;
            left: 50%;
            transition: all 0.1s ease-out;
            border-radius: 4px;
        }

        .x-axis { color: var(--accent-x); }
        .x-axis .bar-fill { background-color: var(--accent-x); }

        .y-axis { color: var(--accent-y); }
        .y-axis .bar-fill { background-color: var(--accent-y); }

        .z-axis { color: var(--accent-z); }
        .z-axis .bar-fill { background-color: var(--accent-z); }

        .orientation-card {
            grid-column: 1 / -1;
            display: flex;
            flex-wrap: wrap;
            justify-content: space-around;
            align-items: center;
            gap: 2rem;
        }

        .cube-wrapper {
            perspective: 600px;
            width: 120px;
            height: 120px;
            display: flex;
            align-items: center;
            justify-content: center;
        }

        .cube {
            width: 80px;
            height: 80px;
            position: relative;
            transform-style: preserve-3d;
            transition: transform 0.1s linear;
        }

        .cube-face {
            position: absolute;
            width: 80px;
            height: 80px;
            background: rgba(59, 130, 246, 0.3);
            border: 2px solid #3b82f6;
            display: flex;
            align-items: center;
            justify-content: center;
            font-weight: bold;
            font-size: 0.8rem;
            color: #fff;
            border-radius: 8px;
        }

        .front  { transform: rotateY(0deg) translateZ(40px); }
        .back   { transform: rotateY(180deg) translateZ(40px); }
        .right  { transform: rotateY(90deg) translateZ(40px); }
        .left   { transform: rotateY(-90deg) translateZ(40px); }
        .top    { transform: rotateX(90deg) translateZ(40px); }
        .bottom { transform: rotateX(-90deg) translateZ(40px); }

        .angles-display {
            display: flex;
            gap: 2rem;
        }

        .angle-box {
            text-align: center;
        }

        .angle-box span {
            display: block;
        }

        .angle-label {
            font-size: 0.8rem;
            color: var(--text-secondary);
            margin-bottom: 0.2rem;
        }

        .angle-val {
            font-size: 1.5rem;
            font-weight: 700;
        }

        footer {
            margin-top: 3rem;
            color: var(--text-secondary);
            font-size: 0.8rem;
            text-align: center;
        }

        /* ---------- Ventana de calibración ---------- */
        .modal-overlay {
            position: fixed;
            inset: 0;
            background: rgba(2, 6, 23, 0.75);
            backdrop-filter: blur(4px);
            display: none;
            align-items: center;
            justify-content: center;
            padding: 1.5rem;
            z-index: 100;
        }

        .modal-overlay.open { display: flex; }

        .modal {
            background: #1e293b;
            border: 1px solid rgba(255, 255, 255, 0.12);
            border-radius: 16px;
            padding: 1.75rem;
            width: 100%;
            max-width: 420px;
            box-shadow: 0 25px 50px -12px rgba(0, 0, 0, 0.6);
        }

        .modal h2 {
            font-size: 1.25rem;
            margin-bottom: 0.75rem;
        }

        .modal p {
            color: var(--text-secondary);
            font-size: 0.9rem;
            line-height: 1.5;
            margin-bottom: 0.75rem;
        }

        .modal ul {
            color: var(--text-secondary);
            font-size: 0.88rem;
            line-height: 1.5;
            margin: 0 0 1rem 1.2rem;
        }

        .modal-actions {
            display: flex;
            flex-wrap: wrap;
            gap: 0.6rem;
            justify-content: flex-end;
            margin-top: 1rem;
        }

        .big-number {
            font-size: 3rem;
            font-weight: 800;
            text-align: center;
            margin: 0.5rem 0;
        }

        .progress {
            width: 100%;
            height: 10px;
            background: rgba(255, 255, 255, 0.1);
            border-radius: 5px;
            overflow: hidden;
            margin: 1rem 0;
        }

        .progress > div {
            height: 100%;
            width: 0%;
            background: var(--accent-z);
            transition: width 0.15s linear;
        }

        .msg-ok  { color: #34d399 !important; font-weight: 600; }
        .msg-err { color: #f87171 !important; font-weight: 600; }
    </style>
</head>
<body>

    <header>
        <h1>Acelerómetro MMA8451 / ESP32</h1>
        <p>Servidor en vivo desde el punto de acceso WiFi <strong>el_acelerometro</strong></p>
        <div class="status-badge">
            <span class="status-dot"></span>
            URL: acelerometro.com (192.168.4.1)
        </div>
    </header>

    <!-- Controles: unidades y calibración -->
    <div class="toolbar">
        <div class="segmented" role="group" aria-label="Unidades">
            <button id="btn-unit-g" onclick="setUnit('g')">g</button>
            <button id="btn-unit-ms2" onclick="setUnit('ms2')">m/s²</button>
        </div>
        <div class="cal-group">
            <span class="cal-status" id="cal-status">Sin calibrar</span>
            <button class="btn primary" onclick="openCalModal()">Recalibrar sensor</button>
        </div>
    </div>

    <div class="grid-container">
        <!-- Eje X -->
        <div class="card x-axis">
            <div class="axis-header">
                <span class="axis-title">Eje X</span>
                <span class="unit-label">g</span>
            </div>
            <div class="axis-value" id="val-x">0.00<span class="unit">g</span></div>
            <div class="bar-bg">
                <div class="bar-fill" id="bar-x"></div>
            </div>
        </div>

        <!-- Eje Y -->
        <div class="card y-axis">
            <div class="axis-header">
                <span class="axis-title">Eje Y</span>
                <span class="unit-label">g</span>
            </div>
            <div class="axis-value" id="val-y">0.00<span class="unit">g</span></div>
            <div class="bar-bg">
                <div class="bar-fill" id="bar-y"></div>
            </div>
        </div>

        <!-- Eje Z -->
        <div class="card z-axis">
            <div class="axis-header">
                <span class="axis-title">Eje Z</span>
                <span class="unit-label">g</span>
            </div>
            <div class="axis-value" id="val-z">0.00<span class="unit">g</span></div>
            <div class="bar-bg">
                <div class="bar-fill" id="bar-z"></div>
            </div>
        </div>

        <!-- Orientación 3D y Ángulos -->
        <div class="card orientation-card">
            <div class="cube-wrapper">
                <div class="cube" id="cube">
                    <div class="cube-face front">ESP32</div>
                    <div class="cube-face back">atrás</div>
                    <div class="cube-face right">der</div>
                    <div class="cube-face left">izq</div>
                    <div class="cube-face top">arriba</div>
                    <div class="cube-face bottom">abajo</div>
                </div>
            </div>
            <div class="angles-display">
                <div class="angle-box">
                    <span class="angle-label">Inclinación (Pitch)</span>
                    <span class="angle-val" id="val-pitch">0°</span>
                </div>
                <div class="angle-box">
                    <span class="angle-label">Roll (Balanceo)</span>
                    <span class="angle-val" id="val-roll">0°</span>
                </div>
            </div>
        </div>
    </div>

    <footer>
        Sistema de Monitoreo ESP32 AP Mode &bull; Puerto 80
    </footer>

    <!-- Ventana de calibración -->
    <div class="modal-overlay" id="cal-modal">
        <div class="modal" id="cal-modal-content"></div>
    </div>

    <script>
        const G_STD = 9.80665; // 1 g en m/s²

        // ---------- Unidades ----------
        let unit = 'g';
        try { unit = localStorage.getItem('unit') || 'g'; } catch (e) {}
        let lastData = null;

        function unitText() { return unit === 'g' ? 'g' : 'm/s²'; }
        function fmt(gVal) { return (unit === 'g' ? gVal : gVal * G_STD).toFixed(2); }

        function setUnit(u) {
            unit = u;
            try { localStorage.setItem('unit', u); } catch (e) {}
            refreshUnitUI();
            if (lastData) renderData(lastData);
        }

        function refreshUnitUI() {
            document.getElementById('btn-unit-g').classList.toggle('active', unit === 'g');
            document.getElementById('btn-unit-ms2').classList.toggle('active', unit === 'ms2');
            document.querySelectorAll('.unit-label').forEach(el => el.textContent = unitText());
        }

        // ---------- Datos del sensor ----------
        function renderData(data) {
            const u = '<span class="unit">' + unitText() + '</span>';
            document.getElementById('val-x').innerHTML = fmt(data.x) + u;
            document.getElementById('val-y').innerHTML = fmt(data.y) + u;
            document.getElementById('val-z').innerHTML = fmt(data.z) + u;

            document.getElementById('val-pitch').innerText = data.pitch.toFixed(1) + '°';
            document.getElementById('val-roll').innerText = data.roll.toFixed(1) + '°';

            // Las barras siempre se calculan en g (rango -2g a +2g), sin importar la unidad
            updateBar('bar-x', data.x);
            updateBar('bar-y', data.y);
            updateBar('bar-z', data.z);

            // Rotar el cubo 3D representativo
            const cube = document.getElementById('cube');
            cube.style.transform = `rotateX(${data.pitch}deg) rotateZ(${-data.roll}deg)`;

            // Estado de calibración
            const st = document.getElementById('cal-status');
            st.textContent = data.calibrated ? 'Calibrado ✓' : 'Sin calibrar';
            st.classList.toggle('ok', data.calibrated);
        }

        async function fetchSensorData() {
            try {
                const response = await fetch('/data');
                const data = await response.json();
                lastData = data;
                renderData(data);
                renderCalModal(data);
            } catch (err) {
                console.error("Error al obtener datos:", err);
            }
        }

        function updateBar(barId, val) {
            const bar = document.getElementById(barId);
            // Limitar rango de -2 a 2 g
            let percentage = (val / 2.0) * 50; 
            if (percentage > 50) percentage = 50;
            if (percentage < -50) percentage = -50;

            if (percentage >= 0) {
                bar.style.left = '50%';
                bar.style.width = percentage + '%';
            } else {
                bar.style.left = (50 + percentage) + '%';
                bar.style.width = Math.abs(percentage) + '%';
            }
        }

        // ---------- Calibración ----------
        // calView: 'intro' (instrucciones) | 'running' (siguiendo el estado del ESP32)
        let calModalOpen = false;
        let calView = 'intro';
        let calSensorError = false;

        const modalEl = document.getElementById('cal-modal');
        const modalContent = document.getElementById('cal-modal-content');
        let lastModalKey = '';

        function setModal(key, html) {
            // Evita reconstruir el DOM si nada cambió (así los botones no parpadean)
            if (key === lastModalKey) return;
            lastModalKey = key;
            modalContent.innerHTML = html;
        }

        function openCalModal() {
            calModalOpen = true;
            calView = 'intro';
            calSensorError = false;
            lastModalKey = '';
            modalEl.classList.add('open');
            renderCalModal(lastData || { cal: 0 });
        }

        async function closeCalModal() {
            // Si había una calibración en marcha, se cancela
            if (calView === 'running' && lastData && (lastData.cal === 1 || lastData.cal === 2)) {
                try { await fetch('/calibrate/cancel', { method: 'POST' }); } catch (e) {}
            }
            calModalOpen = false;
            modalEl.classList.remove('open');
        }

        async function startCalibration() {
            try {
                const r = await fetch('/calibrate/start', { method: 'POST' });
                if (!r.ok) {
                    calSensorError = true;
                    calView = 'intro';
                    lastModalKey = '';
                    renderCalModal(lastData || { cal: 0 });
                    return;
                }
                calSensorError = false;
                calView = 'running';
                lastModalKey = '';
                await fetchSensorData();
            } catch (e) {
                console.error(e);
            }
        }

        async function resetCalibration() {
            try { await fetch('/calibrate/reset', { method: 'POST' }); } catch (e) {}
            await closeCalModal();
            fetchSensorData();
        }

        function renderCalModal(data) {
            if (!calModalOpen) return;

            if (calView === 'intro') {
                const err = calSensorError
                    ? '<p class="msg-err">No se detecta el sensor. Revisa las conexiones.</p>' : '';
                setModal('intro' + calSensorError, `
                    <h2>Recalibrar sensor</h2>
                    <p>La calibración define cuál es la orientación de referencia del sensor,
                       de modo que la gravedad quede en el eje <strong>-Z</strong>
                       (<strong>-Z</strong> hacia el planeta, <strong>Z</strong> hacia el cielo).</p>
                    <ul>
                        <li>Coloca el sensor sobre una superficie estable, en la posición
                            que quieras usar como referencia (idealmente con su cara -Z hacia abajo).</li>
                        <li>Al pulsar <em>Iniciar</em> tendrás 3 segundos para soltarlo.</li>
                        <li>Déjalo <strong>totalmente inmóvil</strong> durante la medición (~1 s).</li>
                    </ul>
                    ${err}
                    <div class="modal-actions">
                        <button class="btn link" onclick="resetCalibration()">Restablecer de fábrica</button>
                        <button class="btn" onclick="closeCalModal()">Cancelar</button>
                        <button class="btn primary" onclick="startCalibration()">Iniciar calibración</button>
                    </div>`);
                return;
            }

            // Vista 'running': sigue el estado que reporta el ESP32
            switch (data.cal) {
                case 1: // cuenta atrás
                    setModal('wait' + data.wait, `
                        <h2>Suelta el sensor</h2>
                        <p>Déjalo inmóvil. La medición comienza en:</p>
                        <div class="big-number">${data.wait}</div>
                        <div class="modal-actions">
                            <button class="btn" onclick="closeCalModal()">Cancelar</button>
                        </div>`);
                    break;
                case 2: // midiendo
                    setModal('samp', `
                        <h2>Midiendo…</h2>
                        <p>No muevas el sensor.</p>
                        <div class="progress"><div id="cal-progress"></div></div>
                        <div class="modal-actions">
                            <button class="btn" onclick="closeCalModal()">Cancelar</button>
                        </div>`);
                    // La barra se actualiza aparte para que la animación sea fluida
                    document.getElementById('cal-progress').style.width = data.prog + '%';
                    break;
                case 3:
                    setModal('ok', `
                        <h2>Calibración completada</h2>
                        <p class="msg-ok">La nueva orientación ya está activa y se guardó en el dispositivo.</p>
                        <p>Ahora la gravedad se lee sobre el eje Z del sistema calibrado.</p>
                        <div class="modal-actions">
                            <button class="btn primary" onclick="closeCalModal()">Cerrar</button>
                        </div>`);
                    break;
                case 4:
                case 5:
                    setModal('err' + data.cal, `
                        <h2>Calibración fallida</h2>
                        <p class="msg-err">${data.cal === 4
                            ? 'Se detectó movimiento durante la medición.'
                            : 'La aceleración medida no corresponde a 1 g. Revisa que el sensor esté quieto.'}</p>
                        <p>Apoya el sensor en una superficie estable e inténtalo de nuevo.</p>
                        <div class="modal-actions">
                            <button class="btn" onclick="closeCalModal()">Cerrar</button>
                            <button class="btn primary" onclick="startCalibration()">Reintentar</button>
                        </div>`);
                    break;
                default:
                    // Estado 0: la calibración fue cancelada
                    break;
            }
            if (data.cal === 2) {
                const p = document.getElementById('cal-progress');
                if (p) p.style.width = data.prog + '%';
            }
        }

        refreshUnitUI();

        // Realizar peticiones cada 150 ms para movimiento fluido
        setInterval(fetchSensorData, 150);
    </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void handleData() {
  // Segundos restantes de la cuenta atrás (solo en CAL_WAIT)
  int waitSec = 0;
  if (calState == CAL_WAIT) {
    unsigned long elapsed = millis() - calStartMs;
    unsigned long remaining = (elapsed >= CAL_WAIT_MS) ? 0 : (CAL_WAIT_MS - elapsed);
    waitSec = (remaining + 999) / 1000;
  }
  int progress = (calState == CAL_SAMPLING) ? (int)((calCount * 100UL) / CAL_SAMPLES) : 0;

  // Retorna datos formateados en formato JSON (aceleración siempre en g)
  String json = "{";
  json += "\"x\":" + String(currentData.x, 3) + ",";
  json += "\"y\":" + String(currentData.y, 3) + ",";
  json += "\"z\":" + String(currentData.z, 3) + ",";
  json += "\"pitch\":" + String(currentData.pitch, 2) + ",";
  json += "\"roll\":" + String(currentData.roll, 2) + ",";
  json += "\"calibrated\":" + String(calibrated ? "true" : "false") + ",";
  json += "\"cal\":" + String((int)calState) + ",";
  json += "\"wait\":" + String(waitSec) + ",";
  json += "\"prog\":" + String(progress);
  json += "}";
  server.send(200, "application/json", json);
}

void handleCalStart() {
  if (!currentData.isConnected) {
    server.send(409, "application/json", "{\"ok\":false,\"error\":\"sensor\"}");
    return;
  }
  calState = CAL_WAIT;
  calStartMs = millis();
  resetCalAccumulators();
  Serial.println("[CALIBRACIÓN] Iniciada. Deja el sensor inmóvil...");
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleCalCancel() {
  if (calState == CAL_WAIT || calState == CAL_SAMPLING) {
    calState = CAL_IDLE;
    Serial.println("[CALIBRACIÓN] Cancelada por el usuario.");
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleCalReset() {
  clearCalibration();
  calState = CAL_IDLE;
  Serial.println("[CALIBRACIÓN] Restablecida a valores de fábrica.");
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleNotFound() {
  // Redirigir cualquier consulta no encontrada hacia el root (Captive Portal)
  server.sendHeader("Location", String("http://") + apIP.toString(), true);
  server.send(302, "text/plain", "");
}

void setup() {
  // 1. Inicialización del puerto Serie
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== INICIALIZANDO ESP32 ACELERÓMETRO (I2C Nativo) ===");

  // Cargar la calibración guardada (si existe)
  loadCalibration();
  Serial.println(calibrated ? "Calibración de orientación cargada desde memoria."
                            : "Sin calibración previa (orientación de fábrica).");

  // 2. Inicialización del bus I2C (SDA = GPIO 21, SCL = GPIO 22)
  Wire.begin(21, 22);

  // Ejecutar diagnóstico inicial de conexiones en el bus I2C
  scanI2C();

  // 3. Inicialización directa del Acelerómetro MMA8451/MMA8452 sin librerías de Adafruit
  if (!initMMA8451Direct()) {
    Serial.println("Error: No se detectó el sensor MMA8451/MMA8452 en 0x1C o 0x1D.");
    currentData.isConnected = false;
  } else {
    Serial.print("Sensor MMA detectado y activado en 0x");
    Serial.println(mmaAddress, HEX);
    currentData.isConnected = true;
  }

  // 4. Configurar el ESP32 como Access Point (AP)
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, netMsk);
  
  // SSID = "el_acelerometro", Password = "" (abierto)
  bool apSuccess = WiFi.softAP(ssid, password);

  if (apSuccess) {
    Serial.println("Red Wi-Fi AP Creada con éxito.");
    Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("Contraseña: Red abierta (sin contraseña)\n");
    Serial.print("Dirección IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Error al iniciar el punto de acceso AP.");
  }

  // 5. Iniciar Servidor DNS: "*" redirige todas las solicitudes (incluida "acelerometro.com")
  dnsServer.start(DNS_PORT, "*", apIP);

  // 6. Configuración de rutas del Servidor Web
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/calibrate/start", HTTP_POST, handleCalStart);
  server.on("/calibrate/cancel", HTTP_POST, handleCalCancel);
  server.on("/calibrate/reset", HTTP_POST, handleCalReset);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("Servidor Web iniciado en puerto 80.");
  Serial.println("Escribe 'acelerometro.com' o '192.168.4.1' en tu navegador conectado a la red.\n");
}

void loop() {
  // A. Procesar peticiones DNS y Web
  dnsServer.processNextRequest();
  server.handleClient();

  // B. Leer datos del acelerómetro directamente desde I2C
  if (currentData.isConnected) {
    currentData.i2cError = readRegisters(MMA8451_REG_OUT_X_MSB, currentData.rawBytes, 6);

    if (currentData.i2cError == 0) {
      // Los datos del acelerómetro se almacenan en 14 bits alineados a la izquierda
      int16_t xRaw = (int16_t)((currentData.rawBytes[0] << 8) | currentData.rawBytes[1]) >> 2;
      int16_t yRaw = (int16_t)((currentData.rawBytes[2] << 8) | currentData.rawBytes[3]) >> 2;
      int16_t zRaw = (int16_t)((currentData.rawBytes[4] << 8) | currentData.rawBytes[5]) >> 2;

      // Convertir de cuentas enteras a aceleración en 'g' (en rango +-2g: 1g = 4096 cuentas)
      float g[3] = { xRaw / 4096.0f, yRaw / 4096.0f, zRaw / 4096.0f };

      // Si hay una calibración en curso, usa la lectura cruda (sin rotar)
      processCalibrationSample(g);

      // Aplicar la rotación de calibración: v_cal = R * v_sensor
      currentData.x = calR[0][0] * g[0] + calR[0][1] * g[1] + calR[0][2] * g[2];
      currentData.y = calR[1][0] * g[0] + calR[1][1] * g[1] + calR[1][2] * g[2];
      currentData.z = calR[2][0] * g[0] + calR[2][1] * g[1] + calR[2][2] * g[2];

      // Calcular ángulos de Inclinación (Pitch) y Balanceo (Roll) en grados
      currentData.pitch = atan2(-currentData.x, sqrt(currentData.y * currentData.y + currentData.z * currentData.z)) * 180.0 / M_PI;
      currentData.roll  = atan2(currentData.y, currentData.z) * 180.0 / M_PI;
    }
  } else {
    currentData.x = 0; currentData.y = 0; currentData.z = 0;
    currentData.pitch = 0; currentData.roll = 0;
  }

  // C. Mostrar lectura periódica con BYTES CRUDOS en la consola de Arduino
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 250) { // Imprime cada 250 ms
    lastPrint = millis();

    if (!currentData.isConnected) {
      Serial.println("[SISTEMA] Sensor no detectado. Revisa las conexiones físicas.");
    } else if (currentData.i2cError != 0) {
      Serial.print("[ERROR I2C] Falló la lectura. Código: ");
      Serial.print(currentData.i2cError);
      Serial.println(" (1: Data muy larga, 2: NACK en dirección, 3: NACK en datos, 4: Otro)");
    } else {
      // Mostrar bytes hexadecimales que envía físicamente el chip
      Serial.print("Raw Bytes: [");
      for (int i = 0; i < 6; i++) {
        if (currentData.rawBytes[i] < 16) Serial.print("0");
        Serial.print(currentData.rawBytes[i], HEX);
        if (i < 5) Serial.print(" ");
      }
      Serial.print("] | ");

      // Mostrar valores calculados (ya calibrados)
      Serial.print("X: "); Serial.print(currentData.x, 2); Serial.print("g ");
      Serial.print("| Y: "); Serial.print(currentData.y, 2); Serial.print("g ");
      Serial.print("| Z: "); Serial.print(currentData.z, 2); Serial.print("g ");
      Serial.print("|| Pitch: "); Serial.print(currentData.pitch, 1); Serial.print("° ");
      Serial.print("| Roll: "); Serial.print(currentData.roll, 1); Serial.println("°");
    }
  }

  delay(10);
}
