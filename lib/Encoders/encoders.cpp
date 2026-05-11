#include "encoders.h"

// Definición de variables globales
volatile long pulsosIzq = 0;
volatile long pulsosDer = 0;
long pulsosAnterioresIzq = 0;
long pulsosAnterioresDer = 0;
unsigned long tiempoAnterior = 0;

int dutyCycleActual = 0;
unsigned long tiempoEscalon = 0;
bool modoPWM = false;

// Interrupciones nativas en RAM para los Encoders
void IRAM_ATTR leerEncoderIzq() {
  if (digitalRead(pinB1) == LOW) {
    pulsosIzq++;
  } else {
    pulsosIzq--;
  }
}

void IRAM_ATTR leerEncoderDer() {
  if (digitalRead(pinB2) == LOW) {
    pulsosDer++;
  } else {
    pulsosDer--;
  }
}

// ==========================================
// CAMBIO 1: Renombrado de setup() a setupEncoders()
// ==========================================
void setupEncoders() {
  Serial.begin(115200);

  // 1. Configurar pines de dirección L298N (Ambos Motores)
  pinMode(L_state, OUTPUT);
  pinMode(L_notstate, OUTPUT);
  pinMode(D_state, OUTPUT);
  pinMode(D_notstate, OUTPUT);

  digitalWrite(L_state, HIGH); // Izquierdo avanza
  digitalWrite(L_notstate, LOW);
  digitalWrite(D_state, HIGH); // Derecho avanza
  digitalWrite(D_notstate, LOW);

  // 2. Configurar canales PWM para ambos motores
  ledcSetup(CANAL_L, MOTOR_FRECUENCIA, MOTOR_RESOLUCION);
  ledcAttachPin(PWM_izquierdo, CANAL_L);
  
  ledcSetup(CANAL_R, MOTOR_FRECUENCIA, MOTOR_RESOLUCION);
  ledcAttachPin(PWM_derecho, CANAL_R);

  // 3. Configurar pines de Encoders
  pinMode(pinA1, INPUT);
  pinMode(pinB1, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinA1), leerEncoderIzq, RISING);

  pinMode(pinA2, INPUT);
  pinMode(pinB2, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinA2), leerEncoderDer, RISING);

  // 4. Configurar ADC para lectura de voltaje
  analogReadResolution(12); // Resolución de 0 a 4095

  // 5. Menú de inicio
  Serial.println("Selecciona la prueba enviando '1' o '2' por el Monitor Serie:");
  Serial.println("1: Prueba de Voltaje (Sube la fuente manualmente)");
  Serial.println("2: Prueba de Duty Cycle (Fija la fuente a 12V)");
  
  while (!Serial.available()) {
    delay(10); 
  }
  char comando = Serial.read();
  
  if (comando == '2') {
    modoPWM = true;
    Serial.println("Iniciando Prueba de Duty Cycle...");
  } else {
    // Si es por voltaje manual, ambos duty cycles van al máximo
    ledcWrite(CANAL_L, 255); 
    ledcWrite(CANAL_R, 255); 
    Serial.println("Iniciando Prueba de Voltaje...");
  }

  // Encabezado del CSV adaptado para 2 motores
  Serial.println("Voltaje_V, DutyCycle, RPM_Izq, Vel_Izq_m_s, RPM_Der, Vel_Der_m_s");
  delay(2000); 
}

// ==========================================
// CAMBIO 2: Renombrado de loop() a loopEncoders()
// ==========================================
void loopEncoders() {
  unsigned long tiempoActual = millis();

  // --- Lógica para la Prueba de Duty Cycle Automática ---
  if (modoPWM && (tiempoActual - tiempoEscalon >= 6000)) {
    if (dutyCycleActual <= 265) {
      // Aplicar PWM a ambos motores
      ledcWrite(CANAL_L, dutyCycleActual);
      ledcWrite(CANAL_R, dutyCycleActual);
      
      dutyCycleActual += 5; 
      tiempoEscalon = tiempoActual;
    } else {
      // Apagar ambos motores al finalizar
      ledcWrite(CANAL_L, 0);
      ledcWrite(CANAL_R, 0);
      Serial.println("0, 0, 0, 0, 0, 0");
      Serial.println("--- FIN DE LA PRUEBA ---");
      while(true); 
    }
  }

  // --- Toma de Datos (Cada 1 segundo) ---
  if (tiempoActual - tiempoAnterior >= 1000) {
    
    // 1. Lectura de Voltaje
    int lecturaADC = analogRead(PIN_VOLTAJE);
    float voltajePin = lecturaADC * (3.3 / 4095.0);
    float voltajeReal = voltajePin * FACTOR_DIVISOR;

    // 2. Lectura Segura de Encoders (Bloqueando interrupciones brevemente)
    noInterrupts();
    long pulsosActualesIzq = pulsosIzq;
    long pulsosActualesDer = pulsosDer;
    interrupts();

    // 3. Diferencial de pulsos
    long deltaPulsosIzq = pulsosActualesIzq - pulsosAnterioresIzq;
    long deltaPulsosDer = pulsosActualesDer - pulsosAnterioresDer;
    
    pulsosAnterioresIzq = pulsosActualesIzq;
    pulsosAnterioresDer = pulsosActualesDer;

    // 4. Matemáticas Motor Izquierdo
    float revsSegIzq = deltaPulsosIzq / PPR_TOTAL;
    float rpmIzq = revsSegIzq * 60.0;
    float velIzq = revsSegIzq * PERIMETRO_METROS;

    // 5. Matemáticas Motor Derecho
    float revsSegDer = deltaPulsosDer / PPR_TOTAL;
    float rpmDer = revsSegDer * 60.0;
    float velDer = revsSegDer * PERIMETRO_METROS;

    // 6. Imprimir en formato CSV (Ambos motores)
    int dutyImpreso = modoPWM ? (dutyCycleActual - 15) : 255;
    if (dutyImpreso < 0) dutyImpreso = 0;

    Serial.printf("%.2f, %d, %.2f, %.4f, %.2f, %.4f\n", 
                  voltajeReal, dutyImpreso, 
                  rpmIzq, velIzq, 
                  rpmDer, velDer);

    tiempoAnterior = tiempoActual;
  }
}