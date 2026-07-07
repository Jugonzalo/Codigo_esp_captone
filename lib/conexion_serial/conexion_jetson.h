#ifndef CONEXION_JETSON_H
#define CONEXION_JETSON_H

#include <Arduino.h>

#define velocidad_serial 115200

extern String Modo_uso;

extern HardwareSerial Serial_elejido;

void setup_jetson();

#endif // CONEXION_JETSON_H




