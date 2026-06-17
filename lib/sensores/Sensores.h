#ifndef Sensores_H
#define Sensores_H

#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_Sensor.h>


#define pin_serial_Data  21
#define pin_serial_clk 22

#define clock_sensor 100000


// ── Signo del giroscopio Z para la convencion teta horario+ ──
// Si la IMU integra el yaw al reves segun el montaje, cambiar a (-1.0f)
#define IMU_GYRO_SIGN (+1.0f)

void setup_sensores();
void loop_sensores();
// ── Paquete de datos crudos/procesados que entrega la IMU ────
struct DatosImu {
  float omega_dps;   // velocidad angular Z (con signo de la convencion) [grados/s]
  float yaw_deg;     // angulo integrado [grados, (-180,180], horario+]
  float accel_x;     // aceleracion lineal de avance (eje X) sin bias [cm/s^2]
};


// IMU (lectura no bloqueante para usar desde una tarea RTOS)
void setup_imu();
void leer_imu(DatosImu &out);

// Calibracion legacy del magnetometro (banco de pruebas)
void setup_calibracion_imu();
void loop_calibracion_imu();




#endif // Sensores_H

