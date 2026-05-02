#ifndef MOTOR_H
#define MOTOR_H

// ===== Pines del puente H =====
#define PWM_izquierdo 23 // PWM izquierdo
#define PWM_derecho 16   // PWM derecho

#define L_state 19    // 1 avanza 0 retrocede
#define L_notstate 18 // 1 retrocede 0 avanza

#define D_state 25    // 1 avanza 0 retrocede
#define D_notstate 17 // 1 retrocede 0 avanza

// ===== Parámetros PWM =====
#define MOTOR_FRECUENCIA 5000
#define MOTOR_RESOLUCION 8
#define MOTOR_DUTYCYCLE 70 // 0-255
#define CANAL_L 0
#define CANAL_R 1

extern bool der_adelante; // true = adelante, false = atrás
extern bool izq_adelante; // true = adelante, false = atrás

// ====== PINES DEL ENCODER ======
// Motor 1 (Rueda Izquierda)
#define pinA1 36 // GPIO14 (D5)
#define pinB1 39 // GPIO12 (D6)

// Motor 2 (Rueda Derecha)
#define pinA2 34 // GPIO5 (D1)
#define pinB2 35 // GPIO4 (D2)

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
#endif // MOTOR_H
