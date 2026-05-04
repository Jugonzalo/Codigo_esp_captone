#ifndef CONEXION_JETSON_H
#define CONEXION_JETSON_H



extern int velocidad_izquierda_global;
extern int velocidad_derecha_global;

#define velocidad_serial 115200



void setup_jetson();
void enviar_datos_jetson();
void leer_datos_jetson();

#endif // CONEXION_JETSON_H




