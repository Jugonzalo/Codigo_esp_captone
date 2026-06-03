#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_ICM20948.h"
#include "Adafruit_Sensor.h"
#include "../Debug_mode.h"

Adafruit_ICM20948 icm;

#define pin_serial_Data  21
#define pin_serial_clk   22
#define clock_sensor     100000

// ── Parámetros de calibración del giroscopio ─────────────────
// (reemplazan a los offsets del magnetómetro)
float mag_offset_x = 0; // Reutilizado como gyroOffsetZ [rad/s]
float mag_offset_y = 0; // Reservado (no usado en modo gyro-only)
float mag_offset_z = 0; // Reservado (no usado en modo gyro-only)

// ── Buffer de suavizado (últimas 3 lecturas de velocidad angular Z) ──
float mag_readings_x[3] = {0}; // Reutilizado: histórico de gyroZ
float mag_readings_y[3] = {0}; // Reservado
float mag_readings_z[3] = {0}; // Reservado

// ── Estado del integrador ─────────────────────────────────────
static float          yaw_acumulado   = 0.0f;   // Ángulo integrado [grados]
static unsigned long  tiempo_anterior = 0;       // Para calcular dt

static constexpr int   GYRO_CALIB_SAMPLES = 500;

// ── Umbral de velocidad angular para suprimir drift en reposo ─
// Si |gyroZ corregido| < este valor, no se integra (zona muerta)
static constexpr float GYRO_DEADZONE_DPS  = 0.1f; // grados/s


void setup_sensores() {
  Wire.begin(pin_serial_clk, pin_serial_Data); // Pines originales
  Wire.setClock(clock_sensor);
  delay(100);

  if (!icm.begin_I2C(0x68, &Wire)) {
    DEBUG_PRINTLN("[ERROR] IMU no detectado. Verifica conexión.");
    while (1) { delay(40); }
  }

  // ── Configurar rango del giroscopio: ±250 dps (máxima resolución) ──
  icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);

  // ────────────────────────────────────────────────────────────
  // RUTINA DE CALIBRACIÓN DEL OFFSET ESTÁTICO DEL GIROSCOPIO
  // El robot debe estar COMPLETAMENTE INMÓVIL durante esto.
  // Reemplaza la calibración del magnetómetro con movimiento.
  // ────────────────────────────────────────────────────────────
  DEBUG_PRINTLN("CALIBRANDO: Mantén el robot COMPLETAMENTE INMÓVIL...");

  // Usamos 'min/max' como acumuladores para mantener la estructura original
  float min_x = 0, max_x = 0; // No usados en este modo; se mantienen declarados
  float min_y = 0, max_y = 0;
  float min_z = 0, max_z = 0;

  // Lista de los últimos 3 datos leídos (se mantiene la estructura original)
  double acumulador_gyroZ = 0.0; // double para mayor precisión en la suma

  for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);

    // Acumular velocidad angular Z en rad/s
    acumulador_gyroZ += (double)gyro.gyro.z;

    delay(4); // ~250 Hz durante la calibración
  }

  // El offset es el promedio de las muestras (bias estático)
  // Se almacena en mag_offset_x para mantener el nombre original de la variable
  mag_offset_x = (float)(acumulador_gyroZ / GYRO_CALIB_SAMPLES); // [rad/s]
  mag_offset_y = 0.0f; // Reservado
  mag_offset_z = 0.0f; // Reservado

  // Inicializar el tiempo DESPUÉS de la calibración
  tiempo_anterior = micros();

  // Inicializar buffer de suavizado en cero
  for (int i = 0; i < 3; i++) {
    mag_readings_x[i] = 0.0f;
  }

  DEBUG_PRINTLN("Parametros de offset:");
  DEBUG_PRINTLN("Calibración lista. Offsets calculados.");
  DEBUG_PRINTF("Offset Gyro Z: %.6f rad/s (%.4f dps)\n",
               mag_offset_x, mag_offset_x * 180.0f / PI);
}


void loop_sensores() {
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  // ── 1. Calcular dt ───────────────────────────────────────────
  unsigned long ahora = micros();
  float dt = (float)(ahora - tiempo_anterior) * 1e-6f; // segundos
  tiempo_anterior = ahora;

  // Guardia de seguridad: descartar dt inválidos
  if (dt <= 0.0001f || dt > 0.5f) return;

  // ── 2. Leer giroscopio Z y eliminar offset estático ──────────
  // Reutilizamos mx/my como en el código original del magnetómetro
  float mx = gyro.gyro.z - mag_offset_x; // gyroZ corregido [rad/s]
  float my = 0.0f;                        // Reservado (mantiene estructura)

  // ── 3. Suavizado: promedio de los últimos 3 datos ─────────────
  // (mantiene exactamente la lógica de desplazamiento del código original)

  // Desplazar los datos anteriores
  mag_readings_x[2] = mag_readings_x[1];
  mag_readings_x[1] = mag_readings_x[0];
  mag_readings_x[0] = mx;

  mag_readings_y[2] = mag_readings_y[1];
  mag_readings_y[1] = mag_readings_y[0];
  mag_readings_y[0] = my;

  // Calcular el promedio
  mx = (mag_readings_x[0] + mag_readings_x[1] + mag_readings_x[2]) / 3.0f;
  my = (mag_readings_y[0] + mag_readings_y[1] + mag_readings_y[2]) / 3.0f;

  // ── 4. Zona muerta: suprimir integración si el robot está quieto ──
  // Equivale a la condición ZUPT sin encoders
  float gyroZ_dps = mx * 180.0f / PI; // Convertir a grados/s para la zona muerta
  if (fabsf(gyroZ_dps) > GYRO_DEADZONE_DPS) {
    // Integración de Euler: θ_k = θ_{k-1} + ω_z * dt
    yaw_acumulado += gyroZ_dps * dt;
  }
  // Si |gyroZ| < zona muerta → no se integra, el drift se congela

  // ── 5. Normalizar a 0–360° (igual que el código original del mag) ──
  float yaw = yaw_acumulado;
  while (yaw <   0.0f) yaw += 360.0f;
  while (yaw > 360.0f) yaw -= 360.0f;

  // Imprimir para verificar precisión
  DEBUG_PRINT("Yaw Calibrado: "); DEBUG_PRINTLN(yaw);

  delay(100);
}