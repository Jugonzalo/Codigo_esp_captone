#include <Arduino.h>
#include "conexion_jetson.h"

String Modo_uso = "";

//Aca voy a dejar la opcion pa que serial usar





//Aca defino que voy a usar el otro serial.
//Lo ideal seria que solito cache si esta en windows o en la jetson




HardwareSerial Serial_elejido(1);




void setup_jetson() {


  //
  //Serial.begin(velocidad_Serial_elejido);   // DECOMENTA PARA USAR WINDOWS

  Serial_elejido.begin(115200, SERIAL_8N1, 21, 22);    //21=Rx   |   22=Tx     // DESCOMENTA PARA USAR JETSON


  delay(2000); // Esperamos un momento para asegurarnos de que la conexión serial esté establecida
  while (Serial.available() > 0) {
    Serial_elejido.read(); // Limpiamos el buffer al inicio
  }
  Serial_elejido.println("Serial Jetson listo");
  delay(1000); // Pequeña pausa para asegurar que el mensaje se envíe antes de continuar
  if (Serial_elejido.available() != 0){
    //llego un error, vuelvo a mandar la wea
    Serial_elejido.println("Serial Jetson listo");
  }

  // Espero hasta 5 segundos a que Python mande el byte de modo
  unsigned long t0 = millis();
  while (Serial_elejido.available() == 0 && millis() - t0 < 5000) {
    delay(10);
  }

  if (Serial_elejido.available() > 0) {
    char modo_ascii = (char)Serial_elejido.read();
    Modo_uso = String(modo_ascii);
  } else {
    Modo_uso = "c";  // default si no llega nada
  }

  Serial_elejido.println("modo:" + Modo_uso);

}









