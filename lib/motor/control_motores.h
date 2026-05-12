#ifndef CONTROL_MOTORES_H
#define CONTROL_MOTORES_H

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <QuickPID.h>

// ===== ASIGNACIÓN DE PINES =====
// Encoders
#define PIN_A1 26 // Motor Izquierdo
#define PIN_B1 27
#define PIN_A2 34 // Motor Derecho
#define PIN_B2 35

// Puente H
#define PWM_L 23
#define PWM_R 16
#define L_IN1 19
#define L_IN2 18
#define D_IN1 25
#define D_IN2 17

// ===== CARACTERIZACIÓN EN m/s (Y = mX + b) =====
// Motor 1 (Izquierdo)
const float M1_m = 0.00709362;
const float M1_b = -0.4760306;
const int M1_MIN_DUTY = 130;

// Motor 2 (Derecho)
const float M2_m = 0.0085289;
const float M2_b = -0.9035745;
const int M2_MIN_DUTY = 135;

const int MAX_DUTY = 245;

// ===== PARÁMETROS MECÁNICOS =====
// METROS_POR_PULSO = (2 * PI * Radio_Rueda) / (Pulsos_Por_Revolucion_Del_Eje)
const float METROS_POR_PULSO = 0.0001; 

// ===== PARÁMETROS DE CONTROL =====
const float ACCEL_MAX = 0.5; // m/s² para el Soft Start

// Usamos float para aprovechar la FPU nativa de la ESP32
const float Kp = 200.0, Ki = 200.0, Kd = 1.0; 

// Prototipos
void setupMotores();
void moverMotores(float ref_vel_izq, float ref_vel_der);
extern float v_act_izq;
extern float v_act_der;
extern float v_ramp_izq;
extern float v_ramp_der;
extern int duty_L; 
extern int duty_R;

#endif