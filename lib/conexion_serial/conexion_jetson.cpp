#include <Arduino.h>
#include "conexion_jetson.h"

String Modo_uso = "";

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

  // Espero hasta 5 segundos a que Python mande el byte de modo
  unsigned long t0 = millis();
  while (Serial.available() == 0 && millis() - t0 < 5000) {
    delay(10);
  }

  if (Serial.available() > 0) {
    char modo_ascii = (char)Serial.read();
    Modo_uso = String(modo_ascii);
  } else {
    Modo_uso = "c";  // default si no llega nada
  }

  Serial.println("modo:" + Modo_uso);

}









