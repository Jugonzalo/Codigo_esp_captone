#ifndef CONEXION_JETSON_H
#define CONEXION_JETSON_H

#include <Arduino.h>

extern String Modo_uso;

const int velocidad_serial = 115200;

extern byte byte_inicio ; 

extern HardwareSerial Serial_elejido;

void setup_jetson();

#endif // CONEXION_JETSON_H




