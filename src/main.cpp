#include "../lib/Debug_mode.h"
#include "../lib/firebase_datos/firebase_datos.h"
#include "../lib/firebase_manager/firebase_manager.h"
#include "../lib/motor/motor.h"
#include "../lib/conexion_serial/conexion_jetson.h"
#include <Arduino.h>






void setup() {
  if (DEBUG) {
    Serial.begin(115200);
  }

  //motorSetup();    // Inicializa motores y canales PWM
  //firebaseSetup(); // Conecta WiFi e inicializa Firebase
  setup_jetson();   // Configura la comunicación serial con Jetson
} 

// =============================================================================
//  LOOP
// =============================================================================
void loop() {

    leer_datos_jetson(); // Lee datos de Jetson y actualiza velocidades
    delay(100); // Pequeña pausa para evitar saturar el buffer serial
    enviar_datos_jetson(); // Envía datos de velocidad actual a Jetson

    delay(1000);
  }
