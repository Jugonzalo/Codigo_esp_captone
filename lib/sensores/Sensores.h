#ifndef Sensores_H
#define Sensores_H

#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_Sensor.h>
#include <VL53L1X.h>



// DATOS

#define pin_serial_Data  21
#define pin_serial_clk 22
#define NUM_SENSORES 
#define clock_sensor 100000


// PARAMETROS PARA LA LECTURA 
extern float gyroOffsetZ;   // bias estatico del giroscopio Z [rad/s]
extern float accelOffsetX;   // bias estatico del acelerometro X [m/s^2]





// OBJETOS SENSORES

extern Adafruit_ICM20948 icm;


extern VL53L1X sensor_izquierda, sensor_izquierda ,sensor_derecha;




void setup_i2c();



struct DatosImu {
  float omega_dps;   // velocidad angular Z (con signo de la convencion) [grados/s]
  float yaw_deg;     // angulo integrado [grados, (-180,180], horario+]
  float accel_x;     // aceleracion lineal de avance (eje X) sin bias [cm/s^2]
};

// IMU (lectura no bloqueante para usar desde una tarea RTOS)

void leer_imu(DatosImu &out);





#endif 

