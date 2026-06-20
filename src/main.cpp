#include "../lib/Debug_mode.h"
#include "../lib/motor/motor.h"
#include "../lib/conexion_serial/conexion_jetson.h"
#include "../lib/sensores/Sensores.h"
#include "../lib/tareas_loops/tareas.h"
#include <Arduino.h>


#define pinA1 26
#define pinB1 27
#define pinA2 34
#define pinB2 35


int64_t count_anterior = 0;   // acumulado anterior para calcular delta
void setup() {
  if (DEBUG) {
    Serial.begin(115200);
  }


  motorSetup();    // Inicializa motores y canales PWM
  //firebaseSetup(); // Conecta WiFi e inicializa Firebase
  //setup_jetson();   // Configura la comunicación serial con Jetson
  //setup_sensores(); // Configura los sensores


  //setup_rtos(); // inicia todas las task


  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoderIzq.attachFullQuad(pinB1, pinA1);
  encoderIzq.setFilter(1023);
  encoderIzq.clearCount();


  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoderDer.attachFullQuad(pinA2, pinB2);
  encoderDer.setFilter(1023);
  encoderDer.clearCount();

  cambiar_velocidad_derecha(60);
  cambiar_velocidad_izquierda(60);
  
} 

// =============================================================================
//  LOOP
// =============================================================================
int conteo =0 ;

void loop() {
    //leer_datos_jetson();  // Lee datos de Jetson y actualiza velocidades globales
    //enviar_datos_jetson(); // Lee datos de Jetson y actualiza velocidades globales
    //loop_sensores(); // Lee datos de los sensores
    //delay(100); // Pequeña pausa para evitar saturar el puerto serial
    //loop_sensores();
    //cambiar_velocidad_derecha(velocidad_derecha_global);
    //cambiar_velocidad_izquierda(velocidad_izquierda_global);
    //vTaskDelete(NULL);  // DEJA SOLO ESTA PRENDIDA SI QUIERES USAR EL RTOS

  // Delta atómico: sin clearCount() para no perder pulsos entre getCount y clear
  int64_t count_izq = encoderIzq.getCount();
  encoderIzq.clearCount();
  float velocidad_izq = count_izq * CM_POR_PULSO;

  int64_t count_der = encoderDer.getCount();
  encoderDer.clearCount();
  float velocidad_der = count_der * CM_POR_PULSO;

  Serial.printf("t=%lu ms | Izq: count=%lld vel=%.2f cm/s | Der: count=%lld vel=%.2f cm/s\n",
                millis(), count_izq, velocidad_izq, count_der, velocidad_der);

  delay(1000);




  }

