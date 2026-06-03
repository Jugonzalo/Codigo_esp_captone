#ifndef Sensores_H
#define Sensores_H

#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_Sensor.h>


#define pin_serial_Data  21
#define pin_serial_clk 22

#define clock_sensor 100000



void setup_sensores();
void loop_sensores();
void setup_calibracion_imu();
void loop_calibracion_imu();


#endif // Sensores_H


