#include <Arduino.h>
#include <Wire.h>
#include <Sensores.h>
#include "Adafruit_ICM20948.h"
#include "Adafruit_Sensor.h"
#include "../Debug_mode.h"

// Objeto global UNICO de la IMU (el de calibracion_imu.cpp es 'static')
Adafruit_ICM20948 icm;

// ── Parametros de calibracion (noise floor) ──────────────────
static float gyroOffsetZ  = 0.0f;   // bias estatico del giroscopio Z [rad/s]
static float accelOffsetX = 0.0f;   // bias estatico del acelerometro X [m/s^2]

// ── Buffers de media movil (ultimas 3 muestras) ──────────────
static float bufGyroZ[3]  = {0};
static float bufAccelX[3] = {0};

// ── Estado del integrador del yaw ────────────────────────────
static float          yaw_acumulado   = 0.0f;   // [grados] horario+
static unsigned long  tiempo_anterior = 0;       // para calcular dt

static constexpr int   GYRO_CALIB_SAMPLES = 500;

// ── Zona muerta para suprimir drift en reposo (anti-drift) ───
// Si |gyroZ corregido| < este valor, no se integra (zona muerta)
static constexpr float GYRO_DEADZONE_DPS  = 0.1f; // grados/s

// Normaliza un angulo a (-180, 180] (helper local de este modulo)
static float wrap180_imu(float ang) {
  while (ang >   180.0f) ang -= 360.0f;
  while (ang <= -180.0f) ang += 360.0f;
  return ang;
}


void setup_imu() {
  // Wire.begin(SDA, SCL) -> el primer argumento es SDA (pin_serial_Data = 21)
  Wire.begin(pin_serial_Data, pin_serial_clk); // SDA=21, SCL=22
  Wire.setClock(clock_sensor);
  delay(100);

  if (!icm.begin_I2C(0x68, &Wire)) {
    DEBUG_PRINTLN("[ERROR] IMU no detectado. Verifica conexion.");
    DEBUG_PRINTLN("Probaré con la direccion alternativa 0x69...");

    if (!icm.begin_I2C(0x69, &Wire)) {
      DEBUG_PRINTLN("[ERROR] IMU no detectado en ninguna direccion.");
      while (1) { delay(40); }
    }
  }

  // Rango del giroscopio: ±250 dps (maxima resolucion)
  icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
  // Rango del acelerometro: ±4 g
  icm.setAccelRange(ICM20948_ACCEL_RANGE_4_G);

  // ────────────────────────────────────────────────────────────
  // CALIBRACION DEL OFFSET ESTATICO (gyro Z y accel X)
  // El robot debe estar COMPLETAMENTE INMOVIL durante esto.
  // ────────────────────────────────────────────────────────────
  DEBUG_PRINTLN("CALIBRANDO IMU: manten el robot COMPLETAMENTE INMOVIL...");

  double acc_gyroZ  = 0.0; // double para mayor precision en la suma
  double acc_accelX = 0.0;

  for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);

    acc_gyroZ  += (double)gyro.gyro.z;          // [rad/s]
    acc_accelX += (double)accel.acceleration.x; // [m/s^2]

    delay(4); // ~250 Hz durante la calibracion
  }

  // El offset es el promedio de las muestras (noise floor / bias estatico)
  gyroOffsetZ  = (float)(acc_gyroZ  / GYRO_CALIB_SAMPLES);
  accelOffsetX = (float)(acc_accelX / GYRO_CALIB_SAMPLES);

  // Inicializar buffers de suavizado y la base de tiempo
  for (int i = 0; i < 3; i++) { bufGyroZ[i] = 0.0f; bufAccelX[i] = 0.0f; }
  yaw_acumulado   = 0.0f;
  tiempo_anterior = micros();

  DEBUG_PRINTLN("Calibracion IMU lista.");
  DEBUG_PRINTF("Offset Gyro Z: %.6f rad/s (%.4f dps) | Offset Accel X: %.4f m/s2\n",
               gyroOffsetZ, gyroOffsetZ * 180.0f / PI, accelOffsetX);
}


// Lectura NO bloqueante. Llamar periodicamente desde una tarea.
void leer_imu(DatosImu &out) {
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  // ── 1. Calcular dt ───────────────────────────────────────────
  unsigned long ahora = micros();
  float dt = (float)(ahora - tiempo_anterior) * 1e-6f; // segundos
  tiempo_anterior = ahora;

  // Guardia de seguridad: dt invalido -> devolvemos el ultimo estado sin integrar
  if (dt <= 0.0001f || dt > 0.5f) {
    out.omega_dps = 0.0f;
    out.yaw_deg   = yaw_acumulado;
    out.accel_x   = 0.0f;
    return;
  }

  // ── 2. Quitar el noise floor (calibracion) ───────────────────
  float gz = gyro.gyro.z - gyroOffsetZ;             // gyroZ corregido [rad/s]
  float ax = accel.acceleration.x - accelOffsetX;   // accelX corregido [m/s^2]

  // ── 3. Suavizado: media movil de las ultimas 3 muestras ──────
  bufGyroZ[2]  = bufGyroZ[1];  bufGyroZ[1]  = bufGyroZ[0];  bufGyroZ[0]  = gz;
  bufAccelX[2] = bufAccelX[1]; bufAccelX[1] = bufAccelX[0]; bufAccelX[0] = ax;

  gz = (bufGyroZ[0]  + bufGyroZ[1]  + bufGyroZ[2])  / 3.0f;
  ax = (bufAccelX[0] + bufAccelX[1] + bufAccelX[2]) / 3.0f;

  // ── 4. Velocidad angular en grados/s con signo de la convencion ──
  float gyroZ_dps = IMU_GYRO_SIGN * gz * 180.0f / PI; // horario+

  // ── 5. Integracion de Euler con zona muerta anti-drift ───────
  if (fabsf(gyroZ_dps) > GYRO_DEADZONE_DPS) {
    yaw_acumulado += gyroZ_dps * dt;
  }
  yaw_acumulado = wrap180_imu(yaw_acumulado);

  // ── 6. Salidas ───────────────────────────────────────────────
  out.omega_dps = gyroZ_dps;
  out.yaw_deg   = yaw_acumulado;
  out.accel_x   = ax;
}
