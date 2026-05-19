#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_ICM20948.h"
#include "Adafruit_Sensor.h"
#include "../Debug_mode.h"

Adafruit_ICM20948 icm;

#define pin_serial_Data  21
#define pin_serial_clk 22
#define clock_sensor 100000

void setup_sensores() {
  // defino el i2c
  Wire.begin(pin_serial_clk, pin_serial_Data);
  Wire.setClock(clock_sensor);
  DEBUG_PRINTLN("AAAAAlo");
  delay(100);

  DEBUG_PRINTLN("Inicializando ICM20948...");

  // Inicializar IMU
  if (!icm.begin_I2C(0x68, &Wire)) {

    DEBUG_PRINTLN("No se encontró ICM20948");

    while (1) {
      delay(10);
    }
  }

  DEBUG_PRINTLN("ICM20948 detectado!");

  // Configuración opcional
  icm.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);

  DEBUG_PRINTLN("Iniciando lectura...\n");
  
}

void loop_sensores() {

  // Estructuras para datos

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  // Leer IMU
  icm.getEvent(&accel, &gyro, &temp, &mag);

  // ----------- ACELERACIÓN -----------
  DEBUG_PRINTLN("===== IMU =====");

  DEBUG_PRINT("Accel X [m/s^2]: ");
  DEBUG_PRINTLN(accel.acceleration.x);

  DEBUG_PRINT("Accel Y [m/s^2]: ");
  DEBUG_PRINTLN(accel.acceleration.y);

  //DEBUG_PRINT("Accel Z [m/s^2]: ");
  //DEBUG_PRINTLN(accel.acceleration.z);

  DEBUG_PRINTLN();

  // ----------- VELOCIDAD ANGULAR -----------
  DEBUG_PRINT("Gyro X [rad/s]: ");
  DEBUG_PRINTLN(gyro.gyro.x);

  DEBUG_PRINT("Gyro Y [rad/s]: ");
  DEBUG_PRINTLN(gyro.gyro.y);

  //DEBUG_PRINT("Gyro Z [rad/s]: ");
  //DEBUG_PRINTLN(gyro.gyro.z);

  DEBUG_PRINTLN("================\n");

  delay(200); // este delay porque????
}
