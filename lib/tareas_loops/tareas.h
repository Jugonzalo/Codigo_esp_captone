#ifndef TAREAS_H
#define TAREAS_H

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <QuickPID.h>
#include <VL53L1X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_Sensor.h>

#include <Sensores.h>
 



// ---------Frecuencias-----------

#define FRECUENCIA_LECTURA 50   //
#define FRECUENCIA_ENVIO 50
#define FRECUENCIA_IMU 10
#define FRECUENCIA_ENCODER 5 // en MiliSeg, 1 es 1kz y 1000 es 1hz 
#define FRECUENCIA_CONTROL_ANGULO 10
#define FRECUENCIA_ESTIMADOR 10 

#define LIMITE_POSITIVO_PID_MOTOR 200
#define LIMITE_NEGATIVO_PID_MOTOR -200

// ====== PINES DEL ENCODER ======
// Motor 1 (Rueda Izquierda)
#define pinA1 26 // GPIO14 (D5)   FASE A (AMARILLO )
#define pinB1 27 // GPIO12 (D6) FASE B (VERDE)
// Motor 2 (Rueda Derecha)
#define pinA2 34 // GPIO5 (D1)
#define pinB2 35 // GPIO4 (D2)





// ------------------------CONSTANTES PID PID------------------------------------
// MOTOR DERECHO
constexpr float Kp_motor_derecho = 2.0f; 
constexpr float Ki_motor_derecho  = 1.5f; 
constexpr float Kd_motor_derecho  = 0.1f; 
// MOTOR izquierdo (Hay que tunearlo )
constexpr float Kp_motor_izquierdo = 2.0f; 
constexpr float Ki_motor_izquierdo  = 1.5f; 
constexpr float Kd_motor_izquierdo  = 0.1f; 
// Controlador de angulo (Hay que tunearlo)
constexpr float Kp_theta = 0.5f; 
constexpr float Ki_theta  = 0.1f; 
constexpr float Kd_theta  = 0.05f; 
// Controldador de poscicion
constexpr float Kp_posicion = 0.009f; 
constexpr float Ki_posicion  = 0.005f; 
constexpr float Kd_posicion  = 0.0f; 


const float VEL_GIRO_MAX = 2.0f; // cm/s: diferencial maximo que pide el control de angulo


const float V_TOTAL_MAX = 15.0f; // cm/s
const float UMBRAL_LLEGADA_POS = 2.5f; // cm: radio de aceptacion del target




//  ============= variables IMU ================

extern float gyroOffsetZ;
extern float accelOffsetX;

// ============= Constantes IMU ================  
const float Umbral_deslizamiento = 0.1f ; // Umbral de deslizamiento para la IMU (grados/segundo)
const float sentido_giro = 1.0f; // Convención de sentido de giro para la IMU (1.0 o -1.0)



// ===== Ganancias del controlador de angulo (PLACEHOLDERS - sintonizar) =====

// Constantes 

constexpr float RADIO_DE_RUEDA = 3.5f; // en cm
constexpr float LARGO_ENTRE_RUEDAS = 18.2f;
constexpr float PERIMETRO = 7 * 3.14;
constexpr float CM_POR_PULSO = PERIMETRO / 897;   // EN PROMEDIO LEI 8978 cm por vuelta







// Declaracion de objetos


// ---------ESTRUCTURAS DE TELEMETRIA-----------
struct __attribute__((packed)) Envio {
  uint8_t header = 0xAA;
  int32_t duty_izq;
  int32_t duty_der;
  float teta;
  float teta_ref;
  float v_der;
  float v_izq;
  float v_der_ref;
  float v_izq_ref;
  float v_total;
  float v_total_ref;
  float x_pos;
  float y_pos;
  float x_ref;
  float y_ref;
};

struct __attribute__((packed)) Lectura {
  uint8_t header = 0xAA;
  int32_t duty_izq;
  int32_t duty_der;
  float teta_ref;
  float v_der_ref;
  float v_izq_ref;
  float v_total_ref;
  float x_ref;
  float y_ref;
};

struct __attribute__((packed)) Coordenadas {
    float x;
    float y;
};


struct __attribute__((packed)) Sensores_distancia {
  float izquierda;
  float derecha;
  float adelante;
};


// IMU


extern DatosImu datosImu;


// --------ENCODERS ----------

extern ESP32Encoder encoderDer;
extern ESP32Encoder encoderIzq;


// ---------------------COLAS---------------------
extern QueueHandle_t ColaLectorDutyIzq, ColaUsoDutyIzq, ColaLectorDutyDer, ColaUsoDutyDer;
extern QueueHandle_t ColaLectorVelIzq, ColaUsoVelIzq, ColaLectorVelDer, ColaUsoVelDer;
extern QueueHandle_t ColaLecturaRampaIzq, ColaUsoRampaIzq, ColaLecturaRampaDer, ColaUsoRampaDer;
extern QueueHandle_t ColaUsoVREFIzq, ColaLecturaVREFIzq, ColaUsoVREFDer, ColaLecturaVREFDer;
extern QueueHandle_t ColaUsoVREFTotal, ColaLecturaVREFTotal;
extern QueueHandle_t ColaUsoTeta, ColaLecturaTeta, ColaUsoTetaRef, ColaLecturaTetaRef;
extern QueueHandle_t ColaUsoVelAng, ColaLecturaVelAng;
extern QueueHandle_t ColaUsoPosicion, ColaLecturaPosicion, ColaUsoPosicionRef, ColaLecturaPosicionRef;
extern QueueHandle_t ColaLecturaSensores;

// ---------------------PID VELOCIDAD---------------------
extern float abs_velocidad_actual_izq, v_out_izq, abs_velocidad_ref_izq;
extern QuickPID pidIzq;
extern float abs_velocidad_actual_der, v_out_der, abs_velocidad_ref_der;
extern QuickPID pidDer;
extern float teta_actual, teta_ref, v_diff;
extern QuickPID pidAngulo;
extern QuickPID pidPosicion;
extern float v_total_out , delta_d;

//funciones

float wrap180(float ang);



void setup_rtos();



// Esta funciones las dejare definidas por mientras como ejemplo
void taskSystemMonitor(void *pvParameters);
void motorderechoSwitchTask(void *pvParameters);
void motorizquierdoSwitchTask(void *pvParameters);
void enviarJetsonTask(void *pvParameters);
void leerJetsonTaks(void *pvParameters);
void lecturaImuTask(void *pvparameters);
void pidPosiciontask(void *pvparameters);

// ESTIMADOR, CASCADA Y RUTINA
void estimadorDePoscicionTask(void *pvParameters);
void pidControlDireccionAngularTask(void *pvParameters);
void asignacionVelocidadRuedasTask(void *pvParameters);
void rutinaCuadradoTask(void *pvParameters);

// Punto de entrada para resetear la pose estimada (Jetson / ArUco)
void solicitarResetPose(float x, float y, float teta);

#endif