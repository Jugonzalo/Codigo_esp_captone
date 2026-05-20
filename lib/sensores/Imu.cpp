#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_ICM20948.h"
#include "Adafruit_Sensor.h"
#include "../Debug_mode.h"

Adafruit_ICM20948 icm;

#define pin_serial_Data  21
#define pin_serial_clk 22
#define clock_sensor 100000

// Variables para offsets (Corrección Hard-Iron)
float mag_offset_x = 0, mag_offset_y = 0, mag_offset_z = 0;

void setup_sensores() {
  Wire.begin(pin_serial_clk, pin_serial_Data); // Pines originales
  Wire.setClock(clock_sensor);
  delay(100);

  if (!icm.begin_I2C(0x68, &Wire)) {
    while (1) { delay(40); }
  }

  // --- RUTINA DE CALIBRACIÓN ---
  DEBUG_PRINTLN("CALIBRANDO: Mueve el robot en círculos y en todas direcciones...");
  
  float min_x = 1000, max_x = -1000;
  float min_y = 1000, max_y = -1000;
  float min_z = 1000, max_z = -1000;

  unsigned long startTime = millis();
  while (millis() - startTime < 15000) { // 15 segundos de calibración
    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);

    if (mag.magnetic.x < min_x) min_x = mag.magnetic.x;
    if (mag.magnetic.x > max_x) max_x = mag.magnetic.x;
    if (mag.magnetic.y < min_y) min_y = mag.magnetic.y;
    if (mag.magnetic.y > max_y) max_y = mag.magnetic.y;
    if (mag.magnetic.z < min_z) min_z = mag.magnetic.z;
    if (mag.magnetic.z > max_z) max_z = mag.magnetic.z;
    
    delay(50);
  }

  // El centro de la elipse es el promedio de los extremos
  mag_offset_x = (min_x + max_x) / 2.0;
  mag_offset_y = (min_y + max_y) / 2.0;
  mag_offset_z = (min_z + max_z) / 2.0;

  DEBUG_PRINTLN("Calibración lista. Offsets calculados.");
}

void loop_sensores() {
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  // Aplicar offsets a la lectura actual
  float mx = mag.magnetic.x - mag_offset_x;
  float my = mag.magnetic.y - mag_offset_y;

  // Calcular Yaw normalizado a 0-360°
  float yaw = atan2(my, mx) * 180.0 / PI;
  if (yaw < 0) yaw += 360.0;

  // Imprimir para verificar precisión
  DEBUG_PRINT("Yaw Calibrado: "); DEBUG_PRINTLN(yaw);
  
  delay(100);
}