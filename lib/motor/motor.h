#ifndef MOTOR_H
#define MOTOR_H

#include <QuickPID.h>
#include <ESP32Encoder.h>

// ===== Pines del puente H =====
#define PWM_izquierdo 23 // PWM izquierdo
#define PWM_derecho 16   // PWM derecho

#define L_state 19    // 1 avanza 0  
#define L_notstate 18 // 1 retrocede 0 avanza

#define D_state 25    // 1 avanza 0 retrocede
#define D_notstate 17 // 1 retrocede 0 avanza

// ===== Parámetros PWM =====
#define MOTOR_FRECUENCIA 200000 // Frecuencia PWM (200 kHz)
#define MOTOR_RESOLUCION 8
#define MOTOR_DUTYCYCLE 70 // 0-255
#define CANAL_L 0
#define CANAL_R 1

extern bool der_adelante; // true = adelante, false = atrás
extern bool izq_adelante; // true = adelante, false = atrás

// ====== PINES DEL ENCODER ======



// Motor 1 (Rueda Izquierda)
#define pinA1 26 // GPIO14 (D5)   FASE A (AMARILLO )
#define pinB1 27 // GPIO12 (D6) FASE B (VERDE)

 //Motor 2 (Rueda Derecha)
#define pinA2 34 // GPIO5 (D1)
#define pinB2 35 // GPIO4 (D2)













// ===== CARACTERIZACIÓN EN cm/s (Y = mX + b) =====
// Motor 1 (Izquierdo)
const float M1_m = 22/37;
const float M1_b = 8.91;
const int M1_MIN_DUTY = 20;

// Motor 2 (Derecho)
const float M2_m = 0.53;
const float M2_b = 77.55;
const int M2_MIN_DUTY = 20;

const int MAX_DUTY = 245;

// ===== PARÁMETROS DE CONTROL =====
const float ACCEL_MAX = 0.5; // m/s² para el Soft Start





// ===== Prototipos =====
void motorSetup();
void cambiar_velocidad_izquierda(int velocidad);
void cambiar_velocidad_derecha(int velocidad);
void cambiar_direccion(bool direc_actual, int pin_direccion, int pin_negado,
                       bool direccion_nueva);

void leerEncoder1();
void leerEncoder2();
void loop_encoder();
void aplicarRampa(float ref, float &ramp, float dt);
int calcularDuty(float vel_deseada, float correccion_pid, float m, float b, int min_duty);
#endif // MOTOR_H
