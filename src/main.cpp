#include <Arduino.h>
#include "control_motores.h"

float velocidad_objetivo = 0.0;

void setup() {
    Serial.begin(115200);
    setupMotores();
    
    delay(2000); 
    Serial.println("Sistema Listo. Ingresa velocidad en m/s:");
    
    // Imprimimos el encabezado para Excel (una sola vez)
    Serial.println("Tiempo_ms,Vel_Act_Der,Duty_Der,Vel_Act_izq,Duty_izq,Ref_Objetivo");
}

void loop() {
    // 1. LECTURA DE COMANDOS
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        float val = input.toFloat();
        velocidad_objetivo = constrain(val, -1.1, 1.1);
    }

    // 2. EJECUCIÓN DEL CONTROL
    moverMotores(velocidad_objetivo, velocidad_objetivo);

// 3. TELEMETRÍA (Formato CSV para exportar a Excel)
    static unsigned long lastPlot = 0;
    
    // Puedes aumentar este valor a 50 o 100 si Excel se llena de demasiados datos muy rápido
    if (millis() - lastPlot > 500) { 
        
        Serial.print(millis());             Serial.print(","); // Tiempo en milisegundos
        Serial.print(v_act_izq);            Serial.print(","); // Velocidad real izquierda
        Serial.print( duty_L );             Serial.print(","); //dutycicle izquierdo)
        Serial.print(v_act_der);            Serial.print(","); // Velocidad real derecha
        Serial.print( duty_R);              Serial.print(",")            ; //dutycicle izquierdo)
        Serial.println(velocidad_objetivo);                    // Referencia final ingresada (lleva println)
        
        lastPlot = millis();
    }}