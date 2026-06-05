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

// Motor 2 (Rueda Derecha)
#define pinA2 34 // GPIO5 (D1)
#define pinB2 35 // GPIO4 (D2)

extern ESP32Encoder encoderIzq;
extern ESP32Encoder encoderDer;


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





// ====== VARIABLES VOLÁTILES ======
extern volatile long contadorPulsos1;
extern volatile long contadorPulsos2;

extern long pulsosAnteriores1;
extern long pulsosAnteriores2;

// ====== CONTROL DE TIEMPO ======
extern unsigned long tiempoAnterior;
extern const unsigned long intervalo; // 1000 ms = 1 segundo

// ====== CONSTANTES FÍSICAS ======
extern const float PPR;
extern const float RELACION_ENGRANAJES;
extern const float PPR_TOTAL;

extern const float DIAMETRO_RUEDA_CM;
extern const float PERIMETRO_RUEDA_CM;

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
