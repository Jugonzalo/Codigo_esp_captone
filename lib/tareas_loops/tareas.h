#ifndef TAREAS_H
#define TAREAS_H




#define FRECUENCIA_MOTORES 1 // en MiliSeg, segun el andres hay cuello de botella
#define FRECUENCIA_LECTURA 100    //
#define FRECUENCIA_ENVIO 100
#define FRECUENCIA_IMU 10
#define FRECUENCIA_ENCODER 1

#define LIMITE_POSITIVO_PID_MOTOR 200
#define LIMITE_NEGATIVO_PID_MOTOR -200










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