#include <Arduino.h>
#include "conexion_jetson.h"


void setup_jetson() {
  Serial.begin(velocidad_serial);
  delay(2000); // Esperamos un momento para asegurarnos de que la conexión serial esté establecida
  while (Serial.available() > 0) {
    Serial.read(); // Limpiamos el buffer al inicio
  }
  Serial.println("Serial Jetson listo");
  delay(1000); // Pequeña pausa para asegurar que el mensaje se envíe antes de continuar
  if (Serial.available() != 0){
    //llego un error, vuelvo a mandar la wea
    Serial.println("Serial Jetson listo");
  } 
}









