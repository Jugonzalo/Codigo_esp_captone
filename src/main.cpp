#include "../lib/Debug_mode.h"
#include "../lib/firebase_datos/firebase_datos.h"
#include "../lib/firebase_manager/firebase_manager.h"
#include "../lib/motor/motor.h"
#include <Arduino.h>

#define pin 19

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  if (DEBUG) {
    Serial.begin(115200);
  }
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("=================================");
  DEBUG_PRINTLN(" ESP32 Firebase Distance Sensor");
  DEBUG_PRINTLN("=================================");

  //motorSetup();    // Inicializa motores y canales PWM
  //firebaseSetup(); // Conecta WiFi e inicializa Firebase

  DEBUG_PRINTLN("Setup completo. Enviando datos cada 1s");
  DEBUG_PRINTLN();
  pinMode(pin, OUTPUT);
  
}

// =============================================================================
//  LOOP
// =============================================================================
void loop() {
  digitalWrite(pin, HIGH);
  delay(1000);
  digitalWrite(pin, LOW);
  delay(1000);
  //firebaseLoop(); // Sincroniza LEDs y movimiento con Firebase
  //loop_encoder();
}