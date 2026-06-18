#include "motor.h"
#include "../Debug_mode.h"
#include "esp32-hal-gpio.h"
#include <Arduino.h>
#include <ESP32Encoder.h>

// Definición real de los booleanos de dirección (declarados en motor.h con
// extern)
bool der_adelante = true; // true = adelante, false = atrás
bool izq_adelante = true; // true = adelante, false = atrás

// variables de encoder

// ====== VARIABLES VOLÁTILES ======
volatile long contadorPulsos1 = 0;
volatile long contadorPulsos2 = 0;

long pulsosAnteriores1 = 0;
long pulsosAnteriores2 = 0;

// ====== CONTROL DE TIEMPO ======
unsigned long tiempoAnterior = 0;
const unsigned long intervalo = 1000; // 1000 ms = 1 segundo

// ====== CONSTANTES FÍSICAS ======
const float PPR = 11.0;
const float RELACION_ENGRANAJES = 1.0;
const float PPR_TOTAL = PPR * RELACION_ENGRANAJES;

const float DIAMETRO_RUEDA_CM = 6.67;
const float PERIMETRO_RUEDA_CM = 3.14159 * DIAMETRO_RUEDA_CM;


float v_act_izq = 0, v_out_izq = 0, v_ramp_izq = 0; 
// Variables PID Motor Derecho
float v_act_der = 0, v_out_der = 0, v_ramp_der = 0;       
int duty_L = 0;
int duty_R = 0;

// INICIO EL PID
QuickPID pidIzq(
  &v_act_izq, &v_out_izq, &v_ramp_izq, 
  Kp, Ki, Kd, 
  QuickPID::pMode::pOnError, 
  QuickPID::dMode::dOnMeas, 
  QuickPID::iAwMode::iAwCondition, // <-- CORRECCIÓN: Modo Anti-Windup añadido
  QuickPID::Action::direct
);

QuickPID pidDer(
  &v_act_der, &v_out_der, &v_ramp_der, 
  Kp, Ki, Kd, 
  QuickPID::pMode::pOnError, 
  QuickPID::dMode::dOnMeas, 
  QuickPID::iAwMode::iAwCondition, // <-- CORRECCIÓN: Modo Anti-Windup añadido
  QuickPID::Action::direct
);




//VOY A INTENTAR PONER LAS FUNCIONES DEL PID ACA
void aplicarRampa(float ref, float &ramp, float dt) {
    float max_step = (ACCEL_MAX) * dt;
    if (abs(ref - ramp) < max_step) ramp = ref;
    else ramp += (ref > ramp) ? max_step : -max_step;
}

int calcularDuty(float vel_deseada, float correccion_pid, float m, float b, int min_duty) {
    if (vel_deseada <= 1.0) return 0; // cm/s (antes 0.01 m/s)
    
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

// funcion para que no solapen

/// funcion encoder

// El uso de ICACHE_RAM_ATTR asegura la velocidad necesaria para procesar pulsos
// sin reiniciar el ESP8266
void ICACHE_RAM_ATTR leerEncoder1() {
  if (digitalRead(pinB1) == LOW) {
    contadorPulsos1++;
  } else {
    contadorPulsos1--;
  }
}

void ICACHE_RAM_ATTR leerEncoder2() {
  if (digitalRead(pinB2) == LOW) {
    contadorPulsos2++;
  } else {
    contadorPulsos2--;
  }
}

void loop_encoder() {
  // Usamos millis() en lugar de delay() para no pausar el procesador
  unsigned long tiempoActual = millis();

  if (tiempoActual - tiempoAnterior >= intervalo) {

    // Lectura segura de variables bloqueando interrupciones por microsegundos
    noInterrupts();
    long pulsosActuales1 = contadorPulsos1;
    long pulsosActuales2 = contadorPulsos2;
    interrupts();

    // 1. Calcular deltas (pulsos en este segundo)
    long deltaPulsos1 = pulsosActuales1 - pulsosAnteriores1;
    long deltaPulsos2 = pulsosActuales2 - pulsosAnteriores2;

    pulsosAnteriores1 = pulsosActuales1;
    pulsosAnteriores2 = pulsosActuales2;

    // 2. Determinar los 5 Estados de Movimiento
    String estadoMovimiento = "Reposo";

    // Tolerancia pequeña por si un motor vibra y marca 1 pulso fantasma
    if (abs(deltaPulsos1) < 2 && abs(deltaPulsos2) < 2) {
      estadoMovimiento = "Reposo";
    } else if (deltaPulsos1 > 0 && deltaPulsos2 > 0) {
      estadoMovimiento = "Adelante";
    } else if (deltaPulsos1 < 0 && deltaPulsos2 < 0) {
      estadoMovimiento = "Atras";
    } else if (deltaPulsos1 > 0 && deltaPulsos2 < 0) {
      // Izquierda avanza, derecha retrocede
      estadoMovimiento = "Giro Horario";
    } else if (deltaPulsos1 < 0 && deltaPulsos2 > 0) {
      // Izquierda retrocede, derecha avanza
      estadoMovimiento = "Giro Antihorario";
    }

    // 3. Cálculos matemáticos (Promedio de ambas ruedas para velocidad del
    // chasis)
    float revsPorSegundo1 = deltaPulsos1 / PPR_TOTAL;
    float revsPorSegundo2 = deltaPulsos2 / PPR_TOTAL;

    float velocidadLineal1 = revsPorSegundo1 * PERIMETRO_RUEDA_CM;
    float velocidadLineal2 = revsPorSegundo2 * PERIMETRO_RUEDA_CM;

    // Velocidad y RPM promedio del robot
    float velocidadLinealPromedio = (velocidadLineal1 + velocidadLineal2) / 2.0;
    float rpmPromedio = ((revsPorSegundo1 + revsPorSegundo2) / 2.0) * 60.0;

    // Distancia total basada en el promedio de avance
    float revolucionesTotales =
        ((pulsosActuales1 + pulsosActuales2) / 2.0) / PPR_TOTAL;
    float distanciaRecorrida = revolucionesTotales * PERIMETRO_RUEDA_CM;

    float tiempoSegundos = tiempoActual / 1000.0;

    // 4. Imprimir en el formato solicitado
    DEBUG_PRINTLN("=========");
    DEBUG_PRINTF("tiempo = %.2f s\n", tiempoSegundos);
    DEBUG_PRINTF("RPM (promedio) = %.2f\n", rpmPromedio);
    DEBUG_PRINTF("Velocidad lineal = %.2f cm/s\n", velocidadLinealPromedio);
    DEBUG_PRINTF("distancia recorrida = %.2f cm\n", distanciaRecorrida);
    DEBUG_PRINTF("Estado = %s\n", estadoMovimiento.c_str());
    DEBUG_PRINTLN("=============");

    tiempoAnterior = tiempoActual;
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




 



}
