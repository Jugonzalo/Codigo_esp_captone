#include <Arduino.h>
#include <motor.h>
#include <conexion_jetson.h>
#include <tareas.h>
#include <Sensores.h>
#include <estimador.h>
#include "../Debug_mode.h"
#include <string.h>

ESP32Encoder encoderDer;
ESP32Encoder encoderIzq;

// ------------------COLAS------------------------

// TAL PARECE QUE CADA VARIABLE NECESITARA COLA DE ENVIO Y COLA DE LECTURA.

// ---------------------COLAS DUTY---------------------
// IZQUIERDO
QueueHandle_t ColaLectorDutyIzq;
QueueHandle_t ColaUsoDutyIzq;
//DERECHO
QueueHandle_t ColaLectorDutyDer;
QueueHandle_t ColaUsoDutyDer;

//---------------------COLAS VELOCIDAD ENCODER---------------------
//IZQUIERDO
QueueHandle_t ColaLectorVelIzq;
QueueHandle_t ColaUsoVelIzq;
//DERECHO
QueueHandle_t ColaLectorVelDer;
QueueHandle_t ColaUsoVelDer;

//---------------------COLAS RAMPAS---------------------
//IZQUIERDO
QueueHandle_t ColaLecturaRampaIzq;
QueueHandle_t ColaUsoRampaIzq;
//DERCHO
QueueHandle_t ColaLecturaRampaDer;
QueueHandle_t ColaUsoRampaDer;

//---------------------COLAS VREF---------------------
//IZQUIERDO
QueueHandle_t ColaUsoVREFIzq;
QueueHandle_t ColaLecturaVREFIzq;
//DERECHO
QueueHandle_t ColaUsoVREFDer;
QueueHandle_t ColaLecturaVREFDer;
//V_TOTAL_REF
QueueHandle_t ColaUsoVREFTotal;
QueueHandle_t ColaLecturaVREFTotal;
//---------------------COLAS Teta---------------------
// teta medido
QueueHandle_t ColaUsoTeta;
QueueHandle_t ColaLecturaTeta;
//teta ref
QueueHandle_t ColaUsoTetaRef;
QueueHandle_t ColaLecturaTetaRef;
//velocidad angular
QueueHandle_t ColaUsoVelAng;
QueueHandle_t ColaLecturaVelAng;
//---------------------COLAS POSICION---------------------
//posicion medida  (Puedo ponerle 2 objetos a la queue)
QueueHandle_t ColaUsoPosicion;
QueueHandle_t ColaLecturaPosicion;
//posicion ref
QueueHandle_t ColaUsoPosicionRef;
QueueHandle_t ColaLecturaPosicionRef;

//---------------------COLA IMU (la publica lecturaImuTask, la consume el estimador)---------------------
QueueHandle_t ColaUsoImu;

//---------------------COLA RESET DE POSE (Jetson / ArUco)---------------------
QueueHandle_t ColaResetPose;


// wrap180() ahora vive en el modulo estimador (lib/Estimador). Se usa desde aca
// via #include <estimador.h> para no duplicar el simbolo al enlazar.

// Estructura de posicion estimada [cm]
struct Posicion { float x; float y; };
// Estructura de reset de pose absoluta (x,y en cm, teta en grados)
struct PoseReset { float x; float y; float teta; };

// Punto de entrada publico para corregir la pose estimada con una referencia
// absoluta (ej. al recibir un ArUco desde la Jetson).
void solicitarResetPose(float x, float y, float teta) {
  if (ColaResetPose == NULL) return;
  PoseReset p = { x, y, teta };
  xQueueOverwrite(ColaResetPose, &p);
}

// ---------ESTRUCTURA DE TELEMETRIA -----------

struct __attribute__((packed)) Envio {
  uint8_t header = 0xAA; // Byte de inicio para sincronizar
  int32_t duty_izq;      // un por ahora dejemoslo como
  int32_t duty_der;        // un byte
  float teta;
  float teta_ref; //4
  float v_der; //5
  float v_izq; //6
  float v_der_ref;
  float v_izq_ref;
  float v_total;
  float v_total_ref;
  float x_pos;
  float y_pos;
  float x_ref;
  float y_ref;  // hasta aca parecen haber 57 bytes

  //uint8_t checksum;      // Para validar integridad
};
struct __attribute__((packed)) Lectura {
  uint8_t header = 0xAA; // Byte de inicio para sincronizar
  int32_t duty_izq;      // un por ahora dejemoslo como
  int32_t duty_der;        // un byte
  float teta_ref;
  float v_der_ref;
  float v_izq_ref;
  int32_t v_total_ref;
  int32_t x_ref;
  int32_t y_ref;  // hasta aca parecen haber 57 bytes

};
/// ---------------------MIS FUNCIONES ------------------------

// ------------- SWITCH MOTORES ----------------

void motorDerechoSwitchTask(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_MOTORES);
    int velocidad_nueva = 0;
    bool subiendo = true;
    
    for(;;){
        //if (xQueueReceive(colaDutyDer, &velocidad_nueva, 0) == pdTRUE) {
          //  cambiar_velocidad_derecha(velocidad_nueva);
        //}
        //if (subiendo && velocidad_nueva < 255){velocidad_nueva += 1; }else if(subiendo && velocidad_nueva == 255){subiendo = false;}
        //if (!subiendo && velocidad_nueva > -255){velocidad_nueva -= 1;}else if (!subiendo  && velocidad_nueva == -255){subiendo = true;}cambiar_velocidad_derecha(velocidad_nueva);
        if (xQueueReceive(ColaUsoDutyDer, &velocidad_nueva, portMAX_DELAY) == pdTRUE){
            cambiar_velocidad_derecha(velocidad_nueva);
        }
    }
}
void motorizquierdoSwitchTask(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_MOTORES);
    int velocidad_nueva = 0;
    
    for(;;){
        if (xQueueReceive(ColaUsoDutyIzq, &velocidad_nueva, portMAX_DELAY) == pdTRUE) {
            cambiar_velocidad_izquierda(velocidad_nueva);
        }
    }
}
//---------------- CONTROL VELOCIDAD----------------
//Rampas
void pasar_rampa_izq_task(void *pvParameters){




    float vref = 0;
    float vRampa = 0;
    float dt = FRECUENCIA_ENCODER / 1000.0f;
    for (;;){
        if (xQueueReceive(ColaUsoVREFIzq, &vref,portMAX_DELAY) == pdTRUE){
            aplicarRampa(vref, vRampa, dt);

            xQueueOverwrite(ColaLecturaRampaIzq, &vref);
        }
    }
}
void pasar_rampa_der_task(void *pvParameters){




    float vref = 0;
    float vRampa = 0;
    float dt = FRECUENCIA_ENCODER / 1000.0f;
    for (;;){
        if (xQueueReceive(ColaUsoVREFDer, &vref,portMAX_DELAY) == pdTRUE){
            aplicarRampa(vref, vRampa, dt);

            xQueueOverwrite(ColaLecturaRampaDer, &vref);
        }
    }
}
//PID
void pidMotorIzqTask(void *pvparameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_MOTORES);

    // VARIABLES INTERNAS
    float velocidad_actual;
    float v_out= 0;
    float v_ramp_ref= 0;
    //ABSOLUTOS
    float abs_velocidad_actual;
    float abs_velocidad_ref;
    //VALOR FINAL
    int duty;

    //SETEO EL PID
    QuickPID pid(
    &abs_velocidad_actual, &v_out, &abs_velocidad_ref,   // les puse el absoluto de una
    Kp, Ki, Kd, 
    QuickPID::pMode::pOnError, 
    QuickPID::dMode::dOnMeas, 
    QuickPID::iAwMode::iAwCondition, // <-- CORRECCIÓN: Modo Anti-Windup añadido
    QuickPID::Action::direct
    );
    
    //PARAMETROS
    pid.SetOutputLimits(LIMITE_NEGATIVO_PID_MOTOR, LIMITE_POSITIVO_PID_MOTOR);
    pid.SetSampleTimeUs(FRECUENCIA_MOTORES * 1000);  // la frecuencia pal pid      
    pid.SetMode(QuickPID::Control::timer); 

    //LOOP
    for (;;) {
        // LEO VELOCIDAD REF Y ACTUAL
        xQueueReceive(ColaUsoVelIzq, &velocidad_actual, 0);
        xQueueReceive(ColaUsoVREFIzq, &v_ramp_ref,0); //AHORA MISMO BYPASEO LA RAMPA

        // CALCULO SEGUN VALORES ABSOLUTOS
        abs_velocidad_actual = abs(velocidad_actual);
        abs_velocidad_ref = abs(v_ramp_ref);
        
        pid.Compute();
        
        // Obtenemos la magnitud bruta del PWM, tiene que ser positiva para el PID
        int duty_magnitud = calcularDuty(abs_velocidad_ref, v_out, M1_m, M1_b, M1_MIN_DUTY);

        if (v_ramp_ref < 0) {
            duty = -duty_magnitud;
        } else {
            duty = duty_magnitud; 
        }

        // ENVIO
        xQueueSend(ColaUsoDutyIzq, &duty, 0); //este va aca porque se activa con la cola el otro 
        xQueueOverwrite(ColaLectorDutyIzq, &duty); // para que el envio a jetson tenga la ultima wea


        //ENVIO
   
    // DELAY
    vTaskDelayUntil(&xLastWakeTime, xfrec); 
    }
}
void pidMotorDerTask(void *pvparameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_MOTORES);

    // VARIABLES INTERNAS
    float velocidad_actual;
    float v_out= 0;
    float v_ramp_ref= 0;
    //ABSOLUTOS
    float abs_velocidad_actual;
    float abs_velocidad_ref;
    //VALOR FINAL
    int duty;

    //SETEO EL PID
    QuickPID pid(
    &abs_velocidad_actual, &v_out, &abs_velocidad_ref,   // les puse el absoluto de una
    Kp, Ki, Kd, 
    QuickPID::pMode::pOnError, 
    QuickPID::dMode::dOnMeas, 
    QuickPID::iAwMode::iAwCondition, // <-- CORRECCIÓN: Modo Anti-Windup añadido
    QuickPID::Action::direct
    );
    
    //PARAMETROS
    pid.SetOutputLimits(LIMITE_NEGATIVO_PID_MOTOR, LIMITE_POSITIVO_PID_MOTOR);
    pid.SetSampleTimeUs(FRECUENCIA_MOTORES * 1000);  // la frecuencia pal pid      
    pid.SetMode(QuickPID::Control::timer); 

    //LOOP
    for (;;) {
        // LEO VELOCIDAD REF Y ACTUAL
        xQueueReceive(ColaUsoVelDer, &velocidad_actual, 0);
        xQueueReceive(ColaUsoVREFDer, &v_ramp_ref,0); //AHORA MISMO BYPASEO LA RAMPA 

        // CALCULO SEGUN VALORES ABSOLUTOS
        abs_velocidad_actual = abs(velocidad_actual);
        abs_velocidad_ref = abs(v_ramp_ref);
        
        pid.Compute();
        
        // Obtenemos la magnitud bruta del PWM (tiene que ser positiva)
        int duty_magnitud = calcularDuty(abs_velocidad_ref, v_out, M1_m, M1_b, M1_MIN_DUTY);

        if (v_ramp_ref < 0) {
            duty = -duty_magnitud; 
        } else {
            duty = duty_magnitud;  
        }

        // ENVIO
        xQueueSend(ColaUsoDutyDer, &duty, 0); // este va aca porque se activa con la cola el otro
        xQueueOverwrite(ColaLectorDutyDer, &duty); // para que el envio a jetson tenga la ultima wea

    // DELAY
    vTaskDelayUntil(&xLastWakeTime, xfrec); 
    }
}

// --------------- CONTROL DE ANGULO ------------------
// PID
void pidControlDireccionAngularTask(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_CONTROL_ANGULO);

    // VARIABLES PID
    float teta_actual = 0.0f;  // entrada del PID (medida "desenrollada")
    float teta_ref    = 0.0f;  // referencia
    float v_diff      = 0.0f;  // salida: diferencial de velocidad [m/s]

    // SETEO EL PID
    QuickPID pidAngulo(
        &teta_actual, &v_diff, &teta_ref,
        Kp_theta, Ki_theta, Kd_theta,
        QuickPID::pMode::pOnError,
        QuickPID::dMode::dOnMeas,
        QuickPID::iAwMode::iAwCondition,
        QuickPID::Action::direct
    );

    // PARAMETROS
    pidAngulo.SetOutputLimits(-VEL_GIRO_MAX, VEL_GIRO_MAX);
    pidAngulo.SetSampleTimeUs(FRECUENCIA_CONTROL_ANGULO * 1000);
    pidAngulo.SetMode(QuickPID::Control::timer);

    float teta_med = 0.0f;
    float v_avance = 0.0f;   // velocidad de avance comun (rutina/secuenciador)
    float v_angular = 0.0f;
    float error = 0.0f;
    // LOOP
    for (;;){
        // LEO REFERENCIA, MEDIDA Y VELOCIDAD DE AVANCE
        xQueuePeek(ColaUsoTetaRef,   &teta_ref, 0);
        xQueuePeek(ColaUsoTeta,      &teta_med, 0);

        // "Desenrollamos" la medida alrededor de la referencia para que el PID
        // vea el error mas corto en (-180,180] sin saltos en el wrap.
        error = wrap180(teta_ref - teta_med);
        teta_actual = teta_ref - error;  // => (teta_ref - teta_actual) = error

        pidAngulo.Compute();

        // Salida del control de angulo: diferencial de velocidad [cm/s].
        // El avance (v_total) se suma despues en asignacionVelocidadRuedasTask.
        // Convencion horario+: para girar horario (error>0) la rueda izquierda
        // va mas rapido que la derecha.
        v_angular = v_avance + v_diff;   // v_avance=0 aqui; el avance entra por v_total

        // ENVIO al conversor (usa peek -> overwrite para no perder el ultimo valor)
        xQueueOverwrite(ColaUsoVelAng, &v_angular);
        xQueueOverwrite(ColaLecturaVelAng, &v_angular);
        // DELAY
        vTaskDelayUntil(&xLastWakeTime, xfrec);
    }
}

//Conversor V_total y V_angular a V_der V_izq
void asignacionVelocidadRuedasTask(void *pvParameters) {
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_CONTROL_ANGULO);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float velocidad_total = 0.0f;
    float velocidad_angular_nueva = 0.0f;
    float v_der_out = 0.0f;
    float v_izq_out = 0.0f;

    for (;;) {
        // Peek: tomamos el ultimo valor sin consumirlo (lo refrescan los productores)
        xQueuePeek(ColaUsoVelAng, &velocidad_angular_nueva, 0);
        xQueuePeek(ColaUsoVREFTotal, &velocidad_total, 0);

        // Modelo diferencial: avance comun + diferencial de giro [cm/s]
        v_izq_out = velocidad_total + velocidad_angular_nueva;
        v_der_out = velocidad_total - velocidad_angular_nueva;

        xQueueSend(ColaUsoVREFIzq, &v_izq_out, 0);
        xQueueSend(ColaUsoVREFDer, &v_der_out, 0);
        xQueueOverwrite(ColaLecturaVREFIzq, &v_izq_out);
        xQueueOverwrite(ColaLecturaVREFDer, &v_der_out);

        vTaskDelayUntil(&xLastWakeTime, xfrec);
        }
    }




//--------------- CONTROL DE POSCICION -----------------
void pidControlPosicionTask(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float velocidad_nueva = 0;
    for (;;){
    // -- LLENAR --
    }
}

// -----------------LECTURA DE SENSORES ----------------
//ENCODERS
void lecturaEncoderIzqTask(void *pvparaameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_ENCODER);      

    float velocidad_leida = 0; 
    int ticks;
    // CREO EL OBJETO
    encoderIzq.clearCount();
    // LOOP
    for (;;){
        ticks = encoderIzq.getCount();
        encoderIzq.clearCount();
        velocidad_leida = ticks * CM_POR_PULSO * 1000.0 / FRECUENCIA_ENCODER; // cm/s
        
        //ENVIAR
        xQueueOverwrite(ColaLectorVelIzq, &velocidad_leida);  // para la lectura
        xQueueSend(ColaUsoVelIzq, &velocidad_leida, 0);   // para activar el pid

        // DELAY
        vTaskDelayUntil(&xLastWakeTime, xfrec);
    }
}
void lecturaEncoderDerTask(void *pvparaameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_ENCODER);      

    float velocidad_leida = 0; 
    int ticks;
    // CREO EL OBJETO
    encoderDer.clearCount();
    // LOOP
    for (;;){
        ticks = encoderDer.getCount();
        encoderDer.clearCount();
        velocidad_leida = ticks * CM_POR_PULSO * 1000.0 / FRECUENCIA_ENCODER; // cm/s
        
        //ENVIAR
        xQueueOverwrite(ColaLectorVelDer, &velocidad_leida);  // para la lectura
        xQueueSend(ColaUsoVelDer, &velocidad_leida, 0);   // para activar el pid

        // DELAY
        vTaskDelayUntil(&xLastWakeTime, xfrec);
    }
}

//IMU: lee la IMU y publica DatosImu (omega, yaw, accel cm/s^2) para el estimador
void lecturaImuTask(void *pvparameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_IMU);

    DatosImu d;
    for (;;){
        leer_imu(d);
        xQueueOverwrite(ColaUsoImu, &d); // el estimador la consume con peek

        vTaskDelayUntil(&xLastWakeTime, xfrec);
    }
}

//Sensores de poscicion
void lecturaPosicionTask(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_LECTURA);
}



// ---------------CALCULO DE POSCICION---------------
//Estimador de poscicion: consolida velocidad de rueda (encoders) + IMU
void estimadorDePoscicionTask(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_IMU);

    EstadoEstimador est;
    estimador_init(est);

    DatosImu imu = {0};
    float v_izq = 0.0f, v_der = 0.0f;   // cm/s (los publican los encoders)
    PoseReset reset;
    TickType_t t_prev = xTaskGetTickCount();
    float dt = 0.0f;
    float teta = 0.0f;

    for (;;){
        // 0. Reset de pose si la Jetson/ArUco lo solicito
        if (xQueueReceive(ColaResetPose, &reset, 0) == pdTRUE){
            estimador_set_pose(est, reset.x, reset.y, reset.teta);
        }

        // 1. Datos ya publicados por otros modulos (no releemos sensores)
        xQueuePeek(ColaUsoImu,       &imu,   0); // IMU (lecturaImuTask)
        xQueuePeek(ColaLectorVelIzq, &v_izq, 0); // velocidad rueda izq [cm/s]
        xQueuePeek(ColaLectorVelDer, &v_der, 0); // velocidad rueda der [cm/s]

        // 2. dt real de la tarea
        TickType_t ahora = xTaskGetTickCount();
        dt = (float)(ahora - t_prev) * portTICK_PERIOD_MS / 1000.0f;
        t_prev = ahora;

        // 3. Estimacion consolidada (cm, cm/s, grados)
        estimador_update(est, v_izq, v_der, imu.yaw_deg, imu.accel_x, dt);

        // 4. Publicar heading y posicion
        teta = est.teta;
        xQueueOverwrite(ColaUsoTeta,     &teta);   // para el control de angulo
        xQueueOverwrite(ColaLecturaTeta, &teta);   // telemetria

        Posicion pos = { est.x, est.y };
        xQueueOverwrite(ColaUsoPosicion,     &pos);
        xQueueOverwrite(ColaLecturaPosicion, &pos);

        vTaskDelayUntil(&xLastWakeTime, xfrec);
    }
}
// -----------------RUTINA DE PRUEBA: CUADRADO ----------------
// Maquina de estados: avanza RUTINA_LADO_CM, gira RUTINA_GIRO_DEG a la derecha
// (en el sitio), repetido RUTINA_NUM_LADOS veces, y se detiene. Comanda la
// cascada escribiendo v_total (ColaUsoVREFTotal) y teta_ref (ColaUsoTetaRef).
void rutinaCuadradoTask(void *pvParameters){
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_RUTINA);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Espera inicial para estabilizar tras la calibracion de la IMU
    vTaskDelay(pdMS_TO_TICKS(2000));

    enum Fase { AVANZAR, GIRAR, HECHO };
    Fase fase = AVANZAR;

    int   lado     = 0;       // tramos completados
    float teta_obj = 0.0f;    // heading objetivo [grados]
    int   settle   = 0;       // anti-rebote del giro
    float v_total  = 0.0f;    // velocidad de avance comandada [cm/s]

    Posicion pos = {0, 0};
    Posicion pos0 = {0, 0};   // origen del tramo actual [cm]
    float teta = 0.0f;

    // Pose inicial = origen y rumbo del primer tramo
    xQueuePeek(ColaUsoTeta,         &teta, 0); teta_obj = teta;
    xQueuePeek(ColaLecturaPosicion, &pos0, 0);

    for (;;){
        // Pose actual
        xQueuePeek(ColaUsoTeta,         &teta, 0);
        xQueuePeek(ColaLecturaPosicion, &pos,  0);

        switch (fase){
            case AVANZAR: {
                v_total = RUTINA_VEL_AVANCE;
                xQueueOverwrite(ColaUsoTetaRef,     &teta_obj);
                xQueueOverwrite(ColaLecturaTetaRef, &teta_obj);

                float dx = pos.x - pos0.x, dy = pos.y - pos0.y;
                float dist = sqrtf(dx * dx + dy * dy);
                if (dist >= RUTINA_LADO_CM){
                    v_total = 0.0f;
                    teta_obj = wrap180(teta_obj + RUTINA_GIRO_DEG); // derecha = horario +
                    settle = 0;
                    fase = GIRAR;
                }
                break;
            }
            case GIRAR: {
                v_total = 0.0f; // giro en el sitio
                xQueueOverwrite(ColaUsoTetaRef,     &teta_obj);
                xQueueOverwrite(ColaLecturaTetaRef, &teta_obj);

                float err = fabsf(wrap180(teta_obj - teta));
                if (err < RUTINA_TOL_ANG){
                    if (++settle >= RUTINA_SETTLE_N){
                        lado++;
                        if (lado >= RUTINA_NUM_LADOS){
                            fase = HECHO;
                        } else {
                            pos0 = pos;        // origen del siguiente tramo
                            fase = AVANZAR;
                        }
                    }
                } else {
                    settle = 0;
                }
                break;
            }
            case HECHO:
            default: {
                v_total = 0.0f;
                xQueueOverwrite(ColaUsoTetaRef,     &teta); // sin error -> ruedas en 0
                xQueueOverwrite(ColaLecturaTetaRef, &teta);
                break;
            }
        }

        // Comando de avance a la cascada
        xQueueOverwrite(ColaUsoVREFTotal,     &v_total);
        xQueueOverwrite(ColaLecturaVREFTotal, &v_total);

#if TEST_ESTIMADOR
        static int _decr = 0;
        if (++_decr >= 4){
            _decr = 0;
            const char *nf = (fase == AVANZAR) ? "AVANZA" :
                             (fase == GIRAR)   ? "GIRA"   : "HECHO";
            DEBUG_PRINTF("[RUTINA] lado=%d %s teta=%6.1f obj=%6.1f x=%6.1f y=%6.1f\n",
                         lado, nf, teta, teta_obj, pos.x, pos.y);
        }
#endif
        vTaskDelayUntil(&xLastWakeTime, xfrec);
    }
}

//  ----------------ENVIO DE DATOS A JETSON ----------------

void enviarJetsonTask(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_LECTURA);
    Envio data;
    memset(&data, 0, sizeof(data)); //inicio todos en 0
    data.header = 0xAA;  // asigno el header
    Posicion _pos = {0, 0};
 
    for (;;){

        //-----------------------DUTY-----------------------
        //IZQ
        xQueuePeek(ColaLectorDutyIzq, &data.duty_izq, 0);
        xQueuePeek(ColaLectorDutyDer, &data.duty_der, 0);
        //-----------------------TETA-----------------------
        xQueuePeek(ColaLecturaTeta,    &data.teta, 0);
        xQueuePeek(ColaLecturaTetaRef, &data.teta_ref, 0);

        //-----------------------V_IZQ V_DER ACTUAL-----------------------
        //IZQ
        xQueuePeek(ColaLectorVelIzq, &data.v_izq, 0);
        //der
        xQueuePeek(ColaLectorVelDer, &data.v_der, 0);
        //-----------------------V_IZQ V_DER REF-----------------------

        xQueuePeek(ColaLecturaVREFIzq, &data.v_izq_ref, 0);
        xQueuePeek(ColaLecturaVREFDer, &data.v_der_ref, 0);

        //-----------------------V_IZQ V_DER REF  (RAMPA O REF REAL)-----------------------
        //IZQ
        xQueuePeek(ColaLecturaRampaIzq, &data.v_izq_ref, 0);
        xQueuePeek(ColaLecturaRampaDer, &data.v_der_ref, 0);
        //-----------------------VTOTAL-----------------------

        //-----------------------VTOTAL REF-----------------------

        //-----------------------X Y ESTIMADO-----------------------
        if (xQueuePeek(ColaLecturaPosicion, &_pos, 0) == pdTRUE){
            data.x_pos = _pos.x;
            data.y_pos = _pos.y;
        }

        //-----------------------X Y REF-----------------------


        // Cálculo simple de checksum (XOR de los datos)
        //data.checksum = (uint8_t)(data.velocidad_izquierda ^ data.velocidad_derecha);

        // Enviamos el bloque de memoria completo (binario puro)
        Serial.write((uint8_t*)&data, sizeof(data));

        //DELAY
        vTaskDelayUntil(&xLastWakeTime, xfrec);
    }
}
void leerJetsonTaks(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_LECTURA);
    Lectura data_leida;
    
    for (;;) {
        // Verificamos si hay al menos el tamaño de la estructura en el buffer
        if (Serial.available() >= sizeof(Lectura)) {
            // Leemos los bytes y los "volcamos" en la dirección de memoria de 'data_leida'
            Serial.readBytes((uint8_t*)&data_leida, sizeof(data_leida));
            // Validamos el header para no procesar basura
            if (data_leida.header == 0xAA) {


            // ------------------------------MANEJO DE COLAS ------------------------------

            //------------------DUTY----------------
            if (false) { // FALSE SI NO QUIERES USARLO
            xQueueSend(ColaUsoDutyIzq, &data_leida.duty_izq ,0);   
            xQueueSend(ColaUsoDutyDer, &data_leida.duty_der ,0); 
            }

            //------------------TETA----------------
            if (true) {  // FALSE SI NO QUIERES USARLO
                xQueueSend(ColaUsoTetaRef, &data_leida.teta_ref, 0);
            }


            // ------------------VREF_IZQ_DER----------------
            if (false) { // FALSE SI NO QUIERES USARLO
            xQueueSend(ColaUsoVREFIzq, &data_leida.v_izq_ref, 0);
            xQueueSend(ColaUsoVREFDer, &data_leida.v_der_ref, 0);
            }

            // ------------------V_TOTAL_REF----------------
            if (true) { // FALSE SI NO QUIERES USARLO
                xQueueSend(ColaUsoVREFTotal, &data_leida.v_total_ref, 0);
            }
            

            // ------------------X_REF_Y_REF----------------


            // LIMPIEZA
            while (Serial.available() > 0) {Serial.read();}}// Descartamos el byte
        else {// Si el header está mal, el buffer está desincronizado // Limpiamos todo
            while (Serial.available() > 0) {Serial.read();}} // Descartamos el byte   
        
        }
    vTaskDelayUntil(&xLastWakeTime, xfrec);
    }
}
/// ------------------------------------SETUP ------------------------------------
 
void setup_rtos() {
    // ---------------------COLAS DUTY---------------------
    // IZQUIERDO
     ColaLectorDutyIzq = xQueueCreate(1, sizeof(int));
     ColaUsoDutyIzq = xQueueCreate(1, sizeof(int));
    //DERECHO
     ColaLectorDutyDer = xQueueCreate(1, sizeof(int));
     ColaUsoDutyDer = xQueueCreate(1, sizeof(int));


    //---------------------COLAS VELOCIDAD ENCODER---------------------
    //IZQUIERDO
     ColaLectorVelIzq  = xQueueCreate(1, sizeof(float));
     ColaUsoVelIzq  = xQueueCreate(1, sizeof(float));
    //DERECHO
     ColaLectorVelDer  = xQueueCreate(1, sizeof(float));
     ColaUsoVelDer = xQueueCreate(1, sizeof(float));

    //---------------------COLAS RAMPAS---------------------
    //IZQUIERDO
     ColaLecturaRampaIzq  = xQueueCreate(1, sizeof(float));
     ColaUsoRampaIzq  = xQueueCreate(1, sizeof(float));
    //DERCHO
     ColaLecturaRampaDer  = xQueueCreate(1, sizeof(float));
     ColaUsoRampaDer  = xQueueCreate(1, sizeof(float));

    //---------------------COLAS VREF---------------------
    //IZQUIERDO
     ColaUsoVREFIzq  = xQueueCreate(1, sizeof(float));
     ColaLecturaVREFIzq  = xQueueCreate(1, sizeof(float));
    //DERECHO
     ColaUsoVREFDer  = xQueueCreate(1, sizeof(float));
     ColaLecturaVREFDer = xQueueCreate(1, sizeof(float));
    //---------------------COLAS V_total_Ref y Teta_ref---------------------
    //V_total
    ColaUsoVREFTotal = xQueueCreate(1, sizeof(float));
    ColaLecturaVREFTotal= xQueueCreate(1, sizeof(float));
    //Teta_ref
    ColaUsoTetaRef = xQueueCreate(1, sizeof(float));
    ColaLecturaTetaRef = xQueueCreate(1, sizeof(float));
    // ------------------ Colas medidas angulares ------------
    // teta medido
    ColaUsoTeta = xQueueCreate(1, sizeof(float));
    ColaLecturaTeta= xQueueCreate(1, sizeof(float));
    //vel angular
    ColaUsoVelAng = xQueueCreate(1, sizeof(float));
    ColaLecturaVelAng= xQueueCreate(1, sizeof(float));

    // ------------------ Colas POSICION ------------
    ColaUsoPosicion     = xQueueCreate(1, sizeof(Posicion));
    ColaLecturaPosicion = xQueueCreate(1, sizeof(Posicion));
    ColaUsoPosicionRef     = xQueueCreate(1, sizeof(Posicion));
    ColaLecturaPosicionRef = xQueueCreate(1, sizeof(Posicion));

    // ------------------ Cola IMU y reset de pose ------------
    ColaUsoImu    = xQueueCreate(1, sizeof(DatosImu));
    ColaResetPose = xQueueCreate(1, sizeof(PoseReset));



    // Inicializamos v_total en 0 para que los peek tengan valor valido
    { float _v0 = 0.0f; xQueueOverwrite(ColaUsoVREFTotal, &_v0); }



   // ENCODERS
    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    encoderIzq.attachFullQuad(pinB1, pinA1);
    encoderIzq.clearCount();
    encoderDer.attachFullQuad(pinA2, pinB2);
    encoderDer.clearCount();


    // USO MOTORES
    xTaskCreatePinnedToCore(
        motorDerechoSwitchTask, //funcion
        "motordtask", //nombre
        2048, //habra que analizarlo
        NULL, //parametros
        10, // Prioridad analizar
        NULL, //handel
        1 //core
    );
    xTaskCreatePinnedToCore(
        motorizquierdoSwitchTask, //funcion
        "motoritask", //nombre
        2048, //habra que analizarlo
        NULL, //parametros
        10, // Prioridadanalizar
        NULL, //handel
        1 //core
    );

     // JETOSN Y ESP
     // En modo banco de pruebas NO se crean: el envio es binario y se
     // mezclaria con el log de texto de la rutina en el mismo puerto serial.
#if !TEST_ESTIMADOR
    xTaskCreatePinnedToCore(
      enviarJetsonTask, //funcion
       "enviodatos", //nombre
        4096, //habra que analizarlo Stack size (Bytes) //creo que esta es la cantidad maxima de data que puede sacar
        NULL, //parametros
    5, // Prioridadanalizar
       NULL, //handel
     0 //core
    );
    xTaskCreatePinnedToCore(leerJetsonTaks,  "leerdatos", 2048,  NULL,4 , NULL, 0);
#endif

    // CONTROLADOR
    //xTaskCreatePinnedToCore(pasar_rampa_izq_task,  "rampaizq",4096,  NULL, 3, NULL, 1);   ESTA DANDO MUCHOS PROBELMAS LA RAMPA, LA bypaseare
    //MOTOR
    xTaskCreatePinnedToCore(pidMotorIzqTask,  "pidizq",4096,  NULL, 8, NULL, 1);
    xTaskCreatePinnedToCore(pidMotorDerTask,  "pidder",4096,  NULL, 8, NULL, 1);

    //ANGULO
    //Pid
    xTaskCreatePinnedToCore(pidControlDireccionAngularTask,  "pid_ang",4096,  NULL, 6, NULL, 1);
    //conversor
    xTaskCreatePinnedToCore(asignacionVelocidadRuedasTask,  "Conversor",4096,  NULL, 5, NULL, 1);

    //ENCODER
    xTaskCreatePinnedToCore(lecturaEncoderIzqTask,  "encizq",3072,  NULL, 7, NULL, 1);
    xTaskCreatePinnedToCore(lecturaEncoderDerTask,  "encder",3072,  NULL, 7, NULL, 1);

    //IMU + ESTIMADOR DE POSICION
    xTaskCreatePinnedToCore(lecturaImuTask,          "imu",       4096, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(estimadorDePoscicionTask,"estimador", 4096, NULL, 6, NULL, 1);

    //RUTINA DE PRUEBA (cuadrado), solo si esta habilitada
#if RUTINA_CUADRADO
    xTaskCreatePinnedToCore(rutinaCuadradoTask, "rutina", 4096, NULL, 4, NULL, 1);
#endif

}
