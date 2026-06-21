#ifndef TAREAS_H
#define TAREAS_H





#define FRECUENCIA_LECTURA 200   //
#define FRECUENCIA_ENVIO 200
#define FRECUENCIA_IMU 10
#define FRECUENCIA_PID_MOTORES 5 // en MiliSeg, 1 es 1kz y 1000 es 1hz
#define FRECUENCIA_ENCODER 10   // en MiliSeg, 1 es 1kz y 1000 es 1hz
#define FRECUENCIA_CONTROL_ANGULO 10

#define LIMITE_POSITIVO_PID_MOTOR 200
#define LIMITE_NEGATIVO_PID_MOTOR -200


// ------------------------CONSTANTES PID PID------------------------------------
// MOTOR DERECHO
constexpr float Kp_motor_derecho = 200.0f; 
constexpr float Ki_motor_derecho  = 200.5f; 
constexpr float Kd_motor_derecho  = 1.0f; 
// MOTOR izquierdo
constexpr float Kp_motor_izquierdo = 200.0f; 
constexpr float Ki_motor_izquierdo  = 200.5f; 
constexpr float Kd_motor_izquierdo  = 1.0f; 
// Controlador de angulo
constexpr float Kp_teta = 1.0f; 
constexpr float Ki_teta  = 0.5f; 
constexpr float Kd_teta  = 0.1f; 
// Controldador de poscicion
constexpr float Kp_posicion = 1.0f; 
constexpr float Ki_posicion  = 0.5f; 
constexpr float Kd_posicion  = 0.1f; 





// ====== PINES DEL ENCODER ======
// Motor 1 (Rueda Izquierda)
// ====== PINES DEL ENCODER ======
// Motor 1 (Rueda Izquierda)
#define pinA1 26 // GPIO14 (D5)   FASE A (AMARILLO )
#define pinB1 27 // GPIO12 (D6) FASE B (VERDE)

// Motor 2 (Rueda Derecha)
#define pinA2 34 // GPIO5 (D1)
#define pinB2 35 // GPIO4 (D2)


// ========KP, KI, KD========

// ===== Ganancias del controlador de angulo (PLACEHOLDERS - sintonizar) =====
const float Kp_ang = 0.01f, Ki_ang = 0.0f, Kd_ang = 0.0f;
// Diferencial maximo de velocidad de rueda que puede pedir el control [m/s]
const float VEL_GIRO_MAX = 0.1f;




// Constantes 

constexpr float RADIO_DE_RUEDA = 3.5f; // en cm
constexpr float LARGO_ENTRE_RUEDAS = 18.2f;
constexpr float PERIMETRO = 7 * 3.14;
constexpr float CM_POR_PULSO = PERIMETRO / 897;   // EN PROMEDIO LEI 8978 cm por vuelta

//funciones

void setup_rtos();



// Esta funciones las dejare definidas por mientras como ejemplo
void taskSystemMonitor(void *pvParameters);
void motorderechoSwitchTask(void *pvParameters);
void motorizquierdoSwitchTask(void *pvParameters);
void enviarJetsonTask(void *pvParameters);
void leerJetsonTaks(void *pvParameters);
void lecturaImuTask(void *pvparameters);

#endif 