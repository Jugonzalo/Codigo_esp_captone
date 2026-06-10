#ifndef TAREAS_H
#define TAREAS_H



//  -----------------------FRECUENCIAS DE LAS TAREAS -----------------------
//MOTORES
constexpr int FRECUENCIA_MOTORES = 1; // en MiliSeg, segun el andres hay cuello de botella
//Frecuencia jetson
constexpr int FRECUENCIA_LECTURA = 100;    //
constexpr int FRECUENCIA_ENVIO = 100;
//frecuencias sensores
constexpr int FRECUENCIA_IMU = 10;
constexpr int FRECUENCIA_ENCODER = 10;




// --------------------------------PIDS --------------------------------
// LIMITES PWM
constexpr int LIMITE_POSITIVO_PID_MOTOR = 200;
constexpr int LIMITE_NEGATIVO_PID_MOTOR = -200;


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
#define pinA1 26 // GPIO14 (D5) PARECE QUE LOS COLORES ESTAN INVERTIDOS POR LO QUE LOS DARE VUELTA EN SOFTWERE
#define pinB1 27 // GPIO12 (D6)

// Motor 2 (Rueda Derecha)
#define pinA2 34 // GPIO5 (D1)
#define pinB2 35 // GPIO4 (D2)


// Constantes 

constexpr float RADIO_DE_RUEDA = 3.5f; // en cm
constexpr float LARGO_ENTRE_RUEDAS = 18.2f;



//funciones

void setup_rtos();



// Esta funciones las dejare definidas por mientras como ejemplo
void taskControl(void *pvParameters);
void taskMotors(void *pvParameters);
void taskSerial(void *pvParameters);
void taskSystemMonitor(void *pvParameters);
void motorderechoSwitchTask(void *pvParameters);
void motorizquierdoSwitchTask(void *pvParameters);
void enviarJetsonTask(void *pvParameters);
void leerJetsonTaks(void *pvParameters);
void lecturaImuTask(void *pvparameters);

#endif