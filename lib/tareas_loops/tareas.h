#ifndef TAREAS_H
#define TAREAS_H




#define FRECUENCIA_MOTORES 1 // en MiliSeg, segun el andres hay cuello de botella
#define FRECUENCIA_LECTURA 500    //
#define FRECUENCIA_ENVIO 500
#define FRECUENCIA_IMU 10
#define FRECUENCIA_ENCODER 10
#define FRECUENCIA_CONTROL_ANGULO 20  // en MiliSeg (50 Hz)

#define LIMITE_POSITIVO_PID_MOTOR 200
#define LIMITE_NEGATIVO_PID_MOTOR -200

// ===== Ganancias del controlador de angulo (PLACEHOLDERS - sintonizar) =====
const float Kp_ang = 0.01f, Ki_ang = 0.0f, Kd_ang = 0.0f;
// Diferencial maximo de velocidad de rueda que puede pedir el control [m/s]
const float VEL_GIRO_MAX = 0.1f;

// ===== Rutina de prueba: cuadrado (4 lados de 50 cm, giro 90 a la derecha) =====
#define FRECUENCIA_RUTINA 50           // ms (20 Hz)
const float RUTINA_VEL_AVANCE = 0.20f; // velocidad de avance leve [m/s]
const float RUTINA_LADO_M     = 0.50f; // longitud de cada lado [m]
const float RUTINA_GIRO_DEG   = 90.0f; // giro a la derecha (horario, +) [grados]
const int   RUTINA_NUM_LADOS  = 4;     // numero de lados/giros
const float RUTINA_TOL_ANG    = 3.0f;  // tolerancia para dar el giro por terminado [grados]
const int   RUTINA_SETTLE_N   = 8;     // ciclos seguidos dentro de tolerancia (anti-rebote)




// ====== PINES DEL ENCODER ======
// Motor 1 (Rueda Izquierda)
#define pinA1 26 // GPIO14 (D5) PARECE QUE LOS COLORES ESTAN INVERTIDOS POR LO QUE LOS DARE VUELTA EN SOFTWERE
#define pinB1 27 // GPIO12 (D6)

// Motor 2 (Rueda Derecha)
#define pinA2 34 // GPIO5 (D1)
#define pinB2 35 // GPIO4 (D2)





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

// ESTIMADOR DE POSICION Y CONTROL DE ANGULO
void estimadorPosicionTask(void *pvParameters);
void controladorAnguloTask(void *pvParameters);

// RUTINA DE PRUEBA (cuadrado)
void rutinaCuadradoTask(void *pvParameters);

// Punto de entrada para resetear la pose estimada (Jetson / ArUco)
void solicitarResetPose(float x, float y, float teta);

#endif