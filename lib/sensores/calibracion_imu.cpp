#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_ICM20948.h"
#include "Adafruit_Sensor.h"

Adafruit_ICM20948 icm;
// este codifo es para probar unas cosas nomas, no lo pesquen 
#define pin_serial_Data  21  // SDA
#define pin_serial_clk 22    // SCL
#define clock_sensor 100000

// Variables para rastrear los extremos del campo magnético
float min_x = 10000, max_x = -10000;
float min_y = 10000, max_y = -10000;
float min_z = 10000, max_z = -10000;

void setup_calibracion_imu() {
  Serial.begin(115200);
  Wire.begin(pin_serial_Data, pin_serial_clk);
  Wire.setClock(clock_sensor);
  delay(100);

  if (!icm.begin_I2C(0x68, &Wire)) {
    Serial.println("Fallo al iniciar el ICM20948. Revisa conexiones.");
    while (1) { delay(40); }
  }

  Serial.println("ICM20948 Detectado.");
  Serial.println("==================================================");
  Serial.println("INICIANDO CALIBRACION EN 5 SEGUNDOS...");
  Serial.println("PREPARATE PARA GIRAR EL ROBOT EN TODAS DIRECCIONES");
  Serial.println("==================================================");
  delay(5000);

  Serial.println("¡CALIBRANDO! Gira el robot en 3D (pitch, roll, yaw) durante 30 segundos...");

  unsigned long startTime = millis();
  
  // Bucle de calibración de 30 segundos
  while (millis() - startTime < 30000) { 
    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);

    // Actualizar máximos y mínimos
    if (mag.magnetic.x < min_x) min_x = mag.magnetic.x;
    if (mag.magnetic.x > max_x) max_x = mag.magnetic.x;
    if (mag.magnetic.y < min_y) min_y = mag.magnetic.y;
    if (mag.magnetic.y > max_y) max_y = mag.magnetic.y;
    if (mag.magnetic.z < min_z) min_z = mag.magnetic.z;
    if (mag.magnetic.z > max_z) max_z = mag.magnetic.z;

    // Imprimir para dar retroalimentación visual de que está leyendo
    Serial.print("Leyendo... X:"); Serial.print(mag.magnetic.x); 
    Serial.print(" Y:"); Serial.print(mag.magnetic.y); 
    Serial.print(" Z:"); Serial.println(mag.magnetic.z);

    delay(20); // Tomar muestras a 50Hz
  }

  Serial.println("\n\n==================================================");
  Serial.println("CALIBRACION TERMINADA. CALCULANDO RESULTADOS...");
  Serial.println("==================================================");

  // 1. Calcular Offsets (Hard-Iron) -> El centro de la esfera
  float offset_x = (max_x + min_x) / 2.0;
  float offset_y = (max_y + min_y) / 2.0;
  float offset_z = (max_z + min_z) / 2.0;

  // 2. Calcular Escalas (Soft-Iron) -> El radio promedio para volver la elipse una esfera
  float chord_x = (max_x - min_x) / 2.0;
  float chord_y = (max_y - min_y) / 2.0;
  float chord_z = (max_z - min_z) / 2.0;

  float avg_chord = (chord_x + chord_y + chord_z) / 3.0;

  float scale_x = avg_chord / chord_x;
  float scale_y = avg_chord / chord_y;
  float scale_z = avg_chord / chord_z;

  // 3. Imprimir el bloque listo para tu código principal
  Serial.println("\nCOPIA Y PEGA ESTE BLOQUE EXACTAMENTE ASI EN TU CODIGO PRINCIPAL:");
  Serial.println("--------------------------------------------------");
  Serial.println("// 1. Offsets (Correccion Hard-Iron)");
  Serial.print("float mag_offset_x = "); Serial.print(offset_x, 4); Serial.println(";");
  Serial.print("float mag_offset_y = "); Serial.print(offset_y, 4); Serial.println(";");
  Serial.print("float mag_offset_z = "); Serial.print(offset_z, 4); Serial.println(";");
  Serial.println("");
  Serial.println("// 2. Factores de Escala (Correccion Soft-Iron)");
  Serial.print("float mag_scale_x = "); Serial.print(scale_x, 4); Serial.println(";");
  Serial.print("float mag_scale_y = "); Serial.print(scale_y, 4); Serial.println(";");
  Serial.print("float mag_scale_z = "); Serial.print(scale_z, 4); Serial.println(";");
  Serial.println("--------------------------------------------------\n");

}

void loop_calibracion_imu() {
  // Una vez que termina, el programa se queda aquí. No hace falta hacer nada más.
  delay(1000);
}