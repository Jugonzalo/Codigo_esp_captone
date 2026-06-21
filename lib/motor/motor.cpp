#include "motor.h"
#include "../Debug_mode.h"
#include "esp32-hal-gpio.h"
#include <Arduino.h>

// Definición real de los booleanos de dirección (declarados en motor.h con
// extern)
bool der_adelante = true; // true = adelante, false = atrás
bool izq_adelante = true; // true = adelante, false = atrás


// INICIO EL PID





//VOY A INTENTAR PONER LAS FUNCIONES DEL PID ACA
void aplicarRampa(float ref, float &ramp, float dt) {
    float max_step = (ACCEL_MAX) * dt;
    if (abs(ref - ramp) < max_step) ramp = ref;
    else ramp += (ref > ramp) ? max_step : -max_step;
}

int calcularDuty(float vel_deseada, float correccion_pid, float m, float b, int min_duty) {
    if (vel_deseada <= 0.01) return 0;
    
    float duty_base = (vel_deseada - b) / m;
    int duty_final = round(duty_base + correccion_pid);
    
    if (duty_final < min_duty) duty_final = 0;
    if (duty_final > MAX_DUTY) duty_final = MAX_DUTY;
    
    return duty_final;
}



// funcion para que no se solapen las direcciones
void cambiar_direccion(bool direc_actual, int pin_direccion, int pin_negado,
                       bool direccion_nueva) {
  if (direccion_nueva != direc_actual) {
    digitalWrite(pin_direccion, LOW);
    digitalWrite(pin_negado, LOW);
  }
}

// ===== Cambiar velocidad motor izquierdo =====
void cambiar_velocidad_izquierda(int velocidad) {
  if (velocidad >= 255) {
    ledcWrite(CANAL_L, 255);
    cambiar_direccion(izq_adelante, L_state, L_notstate, true);
    izq_adelante = true;
    digitalWrite(L_state, HIGH);
    digitalWrite(L_notstate, LOW);
  }
  if (velocidad >= 0 && velocidad <= 255) {
    ledcWrite(CANAL_L, velocidad);
    cambiar_direccion(izq_adelante, L_state, L_notstate, true);
    izq_adelante = true;
    digitalWrite(L_state, HIGH);
    digitalWrite(L_notstate, LOW);
  }
  if (velocidad == 0) {
    digitalWrite(L_state, LOW);
    digitalWrite(L_notstate, LOW);
    ledcWrite(CANAL_L, 0);
  }
  if (velocidad < 0 && velocidad > -255) {
    ledcWrite(CANAL_L, -velocidad);
    cambiar_direccion(izq_adelante, L_state, L_notstate, false);
    izq_adelante = false;
    digitalWrite(L_state, LOW);
    digitalWrite(L_notstate, HIGH);
  }
  if (velocidad < -255) {
    ledcWrite(CANAL_L, 255);
    cambiar_direccion(izq_adelante, L_state, L_notstate, false);
    izq_adelante = false;
    digitalWrite(L_state, LOW);
    digitalWrite(L_notstate, HIGH);
    ledcWrite(CANAL_L, 255);
  }
}

// ===== Cambiar velocidad motor derecho =====
void cambiar_velocidad_derecha(int velocidad) {
  if (velocidad >= 255) {
    ledcWrite(CANAL_R, 255);
    cambiar_direccion(der_adelante, D_state, D_notstate, true);
    der_adelante = true;
    digitalWrite(D_state, HIGH);
    digitalWrite(D_notstate, LOW);
  }
  if (velocidad >= 0 && velocidad <= 255) {
    ledcWrite(CANAL_R, velocidad);
    cambiar_direccion(der_adelante, D_state, D_notstate, true);
    der_adelante = true;
    digitalWrite(D_state, HIGH);
    digitalWrite(D_notstate, LOW);
  }
  if (velocidad == 0) {
    digitalWrite(D_state, LOW);
    digitalWrite(D_notstate, LOW);
    ledcWrite(CANAL_R, 0);
  }
  if (velocidad < 0 && velocidad > -255) {
    ledcWrite(CANAL_R, -velocidad);
    cambiar_direccion(der_adelante, D_state, D_notstate, false);
    der_adelante = false;
    digitalWrite(D_state, LOW);
    digitalWrite(D_notstate, HIGH);
  }
  if (velocidad < -255) {
    ledcWrite(CANAL_R, 255);
    cambiar_direccion(der_adelante, D_state, D_notstate, false);
    der_adelante = false;
    digitalWrite(D_state, LOW);
    digitalWrite(D_notstate, HIGH);
  }
}


// ===== Configuración de motores =====
void motorSetup() {
  // Configurar canal PWM: ledcSetup(canal, frecuencia, resolucion)
  ledcSetup(CANAL_L, MOTOR_FRECUENCIA, MOTOR_RESOLUCION);
  ledcSetup(CANAL_R, MOTOR_FRECUENCIA, MOTOR_RESOLUCION);

  // Vincular pines físicos a sus canales PWM
  ledcAttachPin(PWM_izquierdo, CANAL_L);
  ledcAttachPin(PWM_derecho, CANAL_R);

  // Configurar pines de dirección como salida
  pinMode(L_state, OUTPUT);
  pinMode(D_state, OUTPUT);
  pinMode(L_notstate, OUTPUT);
  pinMode(D_notstate, OUTPUT);

  // Arrancar detenidos
  cambiar_velocidad_izquierda(0);
  cambiar_velocidad_derecha(0);

 
  // Encoder Izquierdo


  // Encoder Derecho
  // encoderDer.attachFullQuad(pinA2, pinB2);
  //encoderDer.setFilter(1023);   // idem
  //encoderDer.clearCount();
}
