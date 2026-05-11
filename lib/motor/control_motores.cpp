#include "control_motores.h"

// Objetos globales
ESP32Encoder encoderIzq, encoderDer;

// Variables PID Motor Izquierdo (QuickPID usa float nativamente)
float v_act_izq = 0, v_out_izq = 0, v_ramp_izq = 0; 
// Variables PID Motor Derecho
float v_act_der = 0, v_out_der = 0, v_ramp_der = 0;       

// Instancias de QuickPID con los 10 parámetros obligatorios
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

unsigned long lastTimePID = 0;

void setupMotores() {
    // 1. Configurar Encoders por Hardware (PCNT)
    ESP32Encoder::useInternalWeakPullResistors = puType::up; // <-- CORRECCIÓN: puType::up en minúsculas
    encoderIzq.attachFullQuad(PIN_A1, PIN_B1);
    encoderDer.attachFullQuad(PIN_A2, PIN_B2);
    encoderIzq.clearCount();
    encoderDer.clearCount();

    // 2. Configurar Puentes H
    pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
    pinMode(D_IN1, OUTPUT); pinMode(D_IN2, OUTPUT);
    
    // 3. Configurar PWM ESP32
    ledcSetup(0, 5000, 8); ledcAttachPin(PWM_L, 0);
    ledcSetup(1, 5000, 8); ledcAttachPin(PWM_R, 1);

    // 4. Iniciar PIDs
    pidIzq.SetOutputLimits(-30.0, 30.0); 
    pidIzq.SetSampleTimeUs(50000); // 50ms = 20Hz
    pidIzq.SetMode(QuickPID::Control::timer); 

    pidDer.SetOutputLimits(-30.0, 30.0);
    pidDer.SetSampleTimeUs(50000); 
    pidDer.SetMode(QuickPID::Control::timer); 
}

void moverMotores(float ref_izq, float ref_der) {
    unsigned long now = millis();
    float dt = (now - lastTimePID) / 1000.0;
    
    if (dt < 0.05) return; // Forzar frecuencia de control a 20Hz (50ms)

    // 1. APLICAR SOFT START (Aceleración controlada en m/s)
    auto aplicarRampa = [&](float ref, float &ramp) {
        float max_step = ACCEL_MAX * dt;
        if (abs(ref - ramp) < max_step) ramp = ref;
        else ramp += (ref > ramp) ? max_step : -max_step;
    };
    aplicarRampa(ref_izq, v_ramp_izq);
    aplicarRampa(ref_der, v_ramp_der);

    // 2. LEER VELOCIDAD REAL EN m/s
    v_act_izq = (encoderIzq.getCount() * METROS_POR_PULSO) / dt; 
    v_act_der = (encoderDer.getCount() * METROS_POR_PULSO) / dt;
    
    encoderIzq.clearCount();
    encoderDer.clearCount();

    // 3. CALCULAR PID
    v_act_izq = abs(v_act_izq);
    v_act_der = abs(v_act_der);
    v_ramp_izq = abs(v_ramp_izq);
    v_ramp_der = abs(v_ramp_der);
    
    pidIzq.Compute();
    pidDer.Compute();

    // 4. CALCULAR DUTY CYCLE FINAL (FeedForward + PID)
    auto calcularDuty = [&](float vel_deseada, float correccion_pid, float m, float b, int min_duty) {
        if (vel_deseada <= 0.01) return 0; // Banda muerta para detener el motor
        
        // Ecuación invertida: Duty = (Vel - b) / m
        float duty_base = (vel_deseada - b) / m;
        int duty_final = round(duty_base + correccion_pid);
        
        // Aplicar límites caracterizados
        if (duty_final < min_duty) duty_final = 0;
        if (duty_final > MAX_DUTY) duty_final = MAX_DUTY;
        
        return duty_final;
    };

    int duty_L = calcularDuty(v_ramp_izq, v_out_izq, M1_m, M1_b, M1_MIN_DUTY);
    int duty_R = calcularDuty(v_ramp_der, v_out_der, M2_m, M2_b, M2_MIN_DUTY);

    // 5. APLICAR DIRECCIÓN Y PWM A LOS MOTORES
    digitalWrite(L_IN1, ref_izq >= 0); 
    digitalWrite(L_IN2, ref_izq < 0);
    ledcWrite(0, (ref_izq == 0) ? 0 : duty_L);
    
    digitalWrite(D_IN1, ref_der >= 0); 
    digitalWrite(D_IN2, ref_der < 0);
    ledcWrite(1, (ref_der == 0) ? 0 : duty_R);

    lastTimePID = now;
}