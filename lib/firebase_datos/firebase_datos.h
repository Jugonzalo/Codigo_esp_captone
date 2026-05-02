#ifndef FIREBASE_DATOS_H
#define FIREBASE_DATOS_H

#include <Firebase_ESP_Client.h>

// defino varaibles a leer

// ----- Rutas del proyecto

#define fijar_derecha "/Escritura/Potencia_derecha"
#define fijar_izquierda "/Escritura/Potencia_izquierda"
#define teta_leido "/Lecturas/teta"

// datos para guardar lo recibido

extern int v_izquierda;
extern int v_derecha;

// funcioones

bool actualizar_dato(String Ruta);
bool fijar_dato(String Ruta, int dato);
void datos_loop();

#endif // FIREBASE_DATOS_H