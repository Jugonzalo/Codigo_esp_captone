#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>

// 1. Heredamos las definiciones de pines y PWM de tu librería de motores
#include "motor.h" 

// ====== PINES DEL ENCODER ======
// Usamos #ifndef por si acaso ya los habías definido en otro lado
#ifndef pinA1
  #define pinA1 36 // Motor 1 (Rueda Izquierda)
  #define pinB1 39 
  #define pinA2 34 // Motor 2 (Rueda Derecha)
  #define pinB2 35 
#endif


// ===== Pin Analógico para leer Voltaje =====
#define PIN_VOLTAJE 32 // Pin seguro que no interfiere con los encoders


// ===== Constantes del Divisor de Tensión =====
#define R1 9740.0 // 10k Ohms
#define R2 3250.0  // 3.3k Ohms
#define FACTOR_DIVISOR ((R1 + R2) / R2)

// ===== Constantes Cinemáticas y del Motor =====
#define RELACION_ENGRANAJES 1.0 
#define PPR_TOTAL (11.0 * RELACION_ENGRANAJES) 
#define DIAMETRO_METROS 0.066 
#define PERIMETRO_METROS (PI * DIAMETRO_METROS) 
// =========================================================

// ===== Variables Globales =====
extern volatile long pulsosIzq;
extern volatile long pulsosDer;
extern long pulsosAnterioresIzq;
extern long pulsosAnterioresDer;
extern unsigned long tiempoAnterior;

extern int dutyCycleActual;
extern unsigned long tiempoEscalon;
extern bool modoPWM;

// ===== Prototipos de Funciones =====
void IRAM_ATTR leerEncoderIzq();
void IRAM_ATTR leerEncoderDer();

// FUNCIONES PRINCIPALES PARA EL MAIN
void setupEncoders();
void loopEncoders();

#endif // ENCODERS_H