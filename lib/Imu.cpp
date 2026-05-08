#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

Adafruit_ICM20948 icm;

void setup() {

  Serial.begin(115200);
  delay(2000);

  // I2C ESP32
  Wire.begin(21, 22);
  Wire.setClock(100000);

  delay(1000);

  Serial.println("Inicializando ICM20948...");

  // Inicializar IMU
  if (!icm.begin_I2C(0x68, &Wire)) {

    Serial.println("No se encontró ICM20948");

    while (1) {
      delay(10);
    }
  }

  Serial.println("ICM20948 detectado!");

  // Configuración opcional
  icm.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);

  Serial.println("Iniciando lectura...\n");
}

void loop() {

  // Estructuras para datos
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;

  // Leer IMU
  icm.getEvent(&accel, &gyro, &temp, &mag);

  // ----------- ACELERACIÓN -----------
  Serial.println("===== IMU =====");

  Serial.print("Accel X [m/s^2]: ");
  Serial.println(accel.acceleration.x);

  Serial.print("Accel Y [m/s^2]: ");
  Serial.println(accel.acceleration.y);

  //Serial.print("Accel Z [m/s^2]: ");
  //Serial.println(accel.acceleration.z);

  Serial.println();

  // ----------- VELOCIDAD ANGULAR -----------
  Serial.print("Gyro X [rad/s]: ");
  Serial.println(gyro.gyro.x);

  Serial.print("Gyro Y [rad/s]: ");
  Serial.println(gyro.gyro.y);

  //Serial.print("Gyro Z [rad/s]: ");
  //Serial.println(gyro.gyro.z);

  Serial.println("================\n");

  delay(200);
}
