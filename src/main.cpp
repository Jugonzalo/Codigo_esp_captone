#include "../lib/Debug_mode.h"
#include "../lib/firebase_datos/firebase_datos.h"
#include "../lib/firebase_manager/firebase_manager.h"
#include "../lib/motor/motor.h"
#include "../lib/conexion_serial/conexion_jetson.h"
#include <Arduino.h>

struct __attribute__((packed)) Envio {
  uint8_t header = 0xAA; // Byte de inicio para sincronizar
  int32_t velocidad_izquierda;      // un por ahora dejemoslo como
  int32_t velocidad_derecha;        // un byte

  uint8_t checksum;      // Para validar integridad
};




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
    leer_datos_jetson();  // Lee datos de Jetson y actualiza velocidades globales
    enviar_datos_jetson(); // Lee datos de Jetson y actualiza velocidades globales
    delay(100); // Pequeña pausa para evitar saturar el puerto serial
  }
