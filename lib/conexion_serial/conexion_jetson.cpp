#include <Arduino.h>
#include "conexion_jetson.h"
#include<Debug_mode.h>

String Modo_uso = "";

//Aca voy a dejar la opcion pa que serial usar





//Aca defino que voy a usar el otro serial.
//Lo ideal seria que solito cache si esta en windows o en la jetson




HardwareSerial Serial_elejido(1);


byte byte_inicio  = 0xAE;

void setup_jetson() {


  //
  //Serial.begin(velocidad_Serial_elejido);   // DECOMENTA PARA USAR WINDOWS

  Serial_elejido.begin(velocidad_serial, SERIAL_8N1, 32, 14);    //21=Rx   |   14=Tx     // DESCOMENTA PARA USAR JETSON


  while (Serial.available() > 0) {
    Serial_elejido.read(); // Limpiamos el buffer al inicio
  }
  delay(1000);
  Serial_elejido.write(byte_inicio);
  delay(1000); // Pequeña pausa para asegurar que el mensaje se envíe antes de continuar
  while (Serial_elejido.available() == 0){
    //llego un error, vuelvo a mandar la wea
    Serial_elejido.write(byte_inicio);
    DEBUG_PRINT("sigoaqui");
    delay(500);
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









