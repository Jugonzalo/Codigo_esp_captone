#include <Arduino.h>
#include <motor.h>
#include <conexion_jetson.h>
#include <tareas.h>
#include <Sensores.h>
#include <estimador.h>
#include "../Debug_mode.h"
#include <string.h>


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

//---------------------COLAS ESTIMADOR (HEADING Y POSICION)---------------------
QueueHandle_t ColaLectorTeta;   // teta fusionado, para telemetria
QueueHandle_t ColaUsoTeta;      // teta fusionado, para el control de angulo
QueueHandle_t ColaLectorXpos;   // posicion X estimada [mm]
QueueHandle_t ColaLectorYpos;   // posicion Y estimada [mm]

//---------------------COLAS REFERENCIA DE ANGULO---------------------
QueueHandle_t ColaUsoTetaRef;       // teta_ref desde Jetson -> control de angulo
QueueHandle_t ColaLecturaTetaRef;   // teta_ref para telemetria

//---------------------COLA VELOCIDAD DE AVANCE---------------------
QueueHandle_t ColaUsoVelAvance;     // velocidad de avance [m/s] -> control de angulo

//---------------------COLA RESET DE POSE (Jetson / ArUco)---------------------
struct PoseReset { float x; float y; float teta; };
QueueHandle_t ColaResetPose;

// Punto de entrada publico para corregir la pose estimada con una referencia
// absoluta. Puede llamarse desde cualquier tarea (ej. al recibir un ArUco).
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
  int32_t v_der_ref;
  int32_t v_izq_ref;
  int32_t v_total;
  int32_t v_total_ref;
  int32_t x_pos;
  int32_t y_pos;
  int32_t x_ref;
  int32_t y_ref;  // hasta aca parecen haber 57 bytes

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
//---------------- CONTROL MOTORES----------------
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


// -----------------LECTURA DE SENSORES ----------------
//ENCODERS
void lecturaEncoderIzqTask(void *pvparaameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_ENCODER);      

    float velocidad_leida = 0; 
    int ticks;
    // CREO EL OBJETO

    ESP32Encoder encoder;
    ESP32Encoder::useInternalWeakPullResistors = puType::up; // <-- CORRECCIÓN: puType::up en minúsculas
    encoder.attachFullQuad(pinB1, pinA1);  // cambia los pines pal derecho
    encoder.clearCount();
    // LOOP
    for (;;){
        ticks = encoder.getCount();
        encoder.clearCount();
        velocidad_leida = ticks * METROS_POR_PULSO * 1000.0 / FRECUENCIA_ENCODER;
        
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

    ESP32Encoder encoder;
    ESP32Encoder::useInternalWeakPullResistors = puType::up; // <-- CORRECCIÓN: puType::up en minúsculas
    encoder.attachFullQuad(pinB2, pinA2); 
    encoder.clearCount();
    // LOOP
    for (;;){
        ticks = encoder.getCount();
        encoder.clearCount();
        velocidad_leida = ticks * METROS_POR_PULSO * 1000.0 / FRECUENCIA_ENCODER;
        
        //ENVIAR
        xQueueOverwrite(ColaLectorVelDer, &velocidad_leida);  // para la lectura
        xQueueSend(ColaUsoVelDer, &velocidad_leida, 0);   // para activar el pid

        // DELAY
        vTaskDelayUntil(&xLastWakeTime, xfrec);
    }
}

//IMU (placeholder historico, el estimador consolida la lectura de la IMU)
void lecturaImuTask(void *pvparameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_LECTURA);



}

// -----------------ESTIMADOR DE POSICION ----------------
// Consolida IMU + encoders + filtro complementario en una pose (x, y, teta).
void estimadorPosicionTask(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_IMU); // 100 Hz

    // ESTADO DEL ESTIMADOR
    EstadoEstimador est;
    estimador_init(est);

    // VARIABLES INTERNAS
    DatosImu imu;
    float v_izq = 0, v_der = 0;
    PoseReset reset;

    TickType_t t_prev = xTaskGetTickCount();

    for (;;){
        // 0. Reset de pose si la Jetson/ArUco lo solicito
        if (xQueueReceive(ColaResetPose, &reset, 0) == pdTRUE){
            estimador_set_pose(est, reset.x, reset.y, reset.teta);
        }

        // 1. Leer IMU (no bloqueante)
        leer_imu(imu);

        // 2. Velocidades de rueda (peek: NO le robamos el dato al PID)
        xQueuePeek(ColaLectorVelIzq, &v_izq, 0);
        xQueuePeek(ColaLectorVelDer, &v_der, 0);

        // 3. dt real de la tarea
        TickType_t ahora = xTaskGetTickCount();
        float dt = (float)(ahora - t_prev) * portTICK_PERIOD_MS / 1000.0f;
        t_prev = ahora;

        // 4. Estimacion consolidada
        estimador_update(est, v_izq, v_der, imu.yaw_deg, imu.accel_x, dt);

        // 5. Publicar resultados
        float teta = est.teta;
        xQueueOverwrite(ColaLectorTeta, &teta);  // telemetria
        xQueueOverwrite(ColaUsoTeta,    &teta);  // control de angulo

        int32_t x_mm = (int32_t)(est.x * 1000.0f);
        int32_t y_mm = (int32_t)(est.y * 1000.0f);
        xQueueOverwrite(ColaLectorXpos, &x_mm);
        xQueueOverwrite(ColaLectorYpos, &y_mm);

        // 6. MODO BANCO DE PRUEBAS: log en texto + referencia fija opcional
#if TEST_ESTIMADOR
#ifdef TEST_TETA_REF_FIJO
        float _ref = TEST_TETA_REF_FIJO;
        xQueueOverwrite(ColaUsoTetaRef, &_ref);     // inyecta referencia al control de angulo
        xQueueOverwrite(ColaLecturaTetaRef, &_ref);
#endif
        static int _dec = 0;
        if (++_dec >= 10) {                          // 100 Hz -> imprime a 10 Hz
            _dec = 0;
            DEBUG_PRINTF("IMU:%7.1f ENC:%7.1f FUS:%7.1f | x:%6.3f y:%6.3f | vL:%5.2f vR:%5.2f | %s\n",
                         est.teta_imu, est.teta_enc, est.teta,
                         est.x, est.y, v_izq, v_der,
                         est.deslizando ? "SLIP" : "ok");
        }
#endif

        // DELAY
        vTaskDelayUntil(&xLastWakeTime, xfrec);
    }
}

// -----------------CONTROLADOR DE ANGULO ----------------
// Recibe teta_ref, lo compara con el heading fusionado y genera un
// diferencial de velocidad de rueda (cascada hacia los PID de velocidad).
void controladorAnguloTask(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_CONTROL_ANGULO);

    // VARIABLES PID
    float teta_actual = 0.0f;  // entrada del PID (medida "desenrollada")
    float teta_ref    = 0.0f;  // referencia
    float v_diff      = 0.0f;  // salida: diferencial de velocidad [m/s]

    // SETEO EL PID
    QuickPID pidAngulo(
        &teta_actual, &v_diff, &teta_ref,
        Kp_ang, Ki_ang, Kd_ang,
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

    // LOOP
    for (;;){
        // LEO REFERENCIA, MEDIDA Y VELOCIDAD DE AVANCE
        xQueuePeek(ColaUsoTetaRef,   &teta_ref, 0);
        xQueuePeek(ColaUsoTeta,      &teta_med, 0);
        xQueuePeek(ColaUsoVelAvance, &v_avance, 0); // 0 si nadie la setea -> giro en el sitio

        // "Desenrollamos" la medida alrededor de la referencia para que el PID
        // vea el error mas corto en (-180,180] sin saltos en el wrap.
        float error = wrap180(teta_ref - teta_med);
        teta_actual = teta_ref - error;  // => (teta_ref - teta_actual) = error

        pidAngulo.Compute();

        // Modelo uniciclo: velocidad de avance comun + correccion diferencial.
        // Convencion horario+: para girar horario (error>0) la rueda izquierda
        // va mas rapido que la derecha.
        float v_izq_ref = v_avance + v_diff;
        float v_der_ref = v_avance - v_diff;

        // ENVIO a los PID de velocidad (lazo interno de la cascada)
        xQueueSend(ColaUsoVREFIzq, &v_izq_ref, 0);
        xQueueSend(ColaUsoVREFDer, &v_der_ref, 0);

        // DELAY
        vTaskDelayUntil(&xLastWakeTime, xfrec);
    }
}

// -----------------RUTINA DE PRUEBA: CUADRADO ----------------
// Maquina de estados que dibuja un cuadrado: avanza RUTINA_LADO_M, gira
// RUTINA_GIRO_DEG a la derecha (en el sitio), repetido RUTINA_NUM_LADOS veces,
// y se detiene. Mide distancia/angulo con la pose del estimador y comanda el
// control de angulo via teta_ref + velocidad de avance.
void rutinaCuadradoTask(void *pvParameters){
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_RUTINA);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Espera inicial para estabilizar tras la calibracion de la IMU
    vTaskDelay(pdMS_TO_TICKS(2000));

    enum Fase { AVANZAR, GIRAR, HECHO };
    Fase fase = AVANZAR;

    int   lado     = 0;     // tramos completados (0..RUTINA_NUM_LADOS)
    float x0 = 0, y0 = 0;   // origen del tramo actual [m]
    float teta_obj = 0.0f;  // heading objetivo [grados]
    int   settle   = 0;     // contador anti-rebote del giro
    float v_avance = 0.0f;

    // Pose actual
    float teta = 0.0f, xm = 0.0f, ym = 0.0f;
    int32_t x_mm = 0, y_mm = 0;

    // Capturamos la pose inicial como origen y rumbo del primer tramo
    xQueuePeek(ColaUsoTeta,    &teta,  0); teta_obj = teta;
    xQueuePeek(ColaLectorXpos, &x_mm,  0); x0 = x_mm / 1000.0f;
    xQueuePeek(ColaLectorYpos, &y_mm,  0); y0 = y_mm / 1000.0f;

    for (;;){
        // ── Leer pose actual del estimador ──
        xQueuePeek(ColaUsoTeta,    &teta,  0);
        xQueuePeek(ColaLectorXpos, &x_mm,  0); xm = x_mm / 1000.0f;
        xQueuePeek(ColaLectorYpos, &y_mm,  0); ym = y_mm / 1000.0f;

        switch (fase) {
            case AVANZAR: {
                v_avance = RUTINA_VEL_AVANCE;
                // mantener el rumbo del tramo
                xQueueOverwrite(ColaUsoTetaRef,     &teta_obj);
                xQueueOverwrite(ColaLecturaTetaRef, &teta_obj);

                float dx = xm - x0, dy = ym - y0;
                float dist = sqrtf(dx * dx + dy * dy);
                if (dist >= RUTINA_LADO_M) {
                    // fin del tramo -> preparar giro a la derecha (horario +)
                    v_avance = 0.0f;
                    teta_obj = wrap180(teta_obj + RUTINA_GIRO_DEG);
                    settle = 0;
                    fase = GIRAR;
                }
                break;
            }
            case GIRAR: {
                v_avance = 0.0f; // giro en el sitio
                xQueueOverwrite(ColaUsoTetaRef,     &teta_obj);
                xQueueOverwrite(ColaLecturaTetaRef, &teta_obj);

                float err = fabsf(wrap180(teta_obj - teta));
                if (err < RUTINA_TOL_ANG) {
                    if (++settle >= RUTINA_SETTLE_N) {
                        lado++;
                        if (lado >= RUTINA_NUM_LADOS) {
                            fase = HECHO;
                        } else {
                            // siguiente tramo: origen = pose actual
                            x0 = xm; y0 = ym;
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
                v_avance = 0.0f;
                // sin error de angulo -> el control deja las ruedas en 0
                xQueueOverwrite(ColaUsoTetaRef,     &teta);
                xQueueOverwrite(ColaLecturaTetaRef, &teta);
                break;
            }
        }

        // Publicar la velocidad de avance para el control de angulo
        xQueueOverwrite(ColaUsoVelAvance, &v_avance);

#if TEST_ESTIMADOR
        static int _decr = 0;
        if (++_decr >= 4) { // 20 Hz -> ~5 Hz
            _decr = 0;
            const char *nf = (fase == AVANZAR) ? "AVANZA" :
                             (fase == GIRAR)   ? "GIRA"   : "HECHO";
            DEBUG_PRINTF("[RUTINA] lado=%d %s teta=%6.1f obj=%6.1f x=%5.2f y=%5.2f\n",
                         lado, nf, teta, teta_obj, xm, ym);
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
 
    for (;;){

        //-----------------------DUTY-----------------------
        //IZQ
        xQueuePeek(ColaLectorDutyIzq, &data.duty_izq, 0);
        xQueuePeek(ColaLectorDutyDer, &data.duty_der, 0);
        //-----------------------TETA-----------------------
        xQueuePeek(ColaLectorTeta,     &data.teta, 0);
        xQueuePeek(ColaLecturaTetaRef, &data.teta_ref, 0);

        //-----------------------V_IZQ V_DER ACTUAL-----------------------
        //IZQ
        xQueuePeek(ColaLectorVelIzq, &data.v_izq, 0);
        xQueuePeek(ColaLectorVelDer, &data.v_der, 0);

        //-----------------------V_IZQ V_DER REF  (RAMPA O REF REAL)-----------------------
        //IZQ
        xQueuePeek(ColaLecturaRampaIzq, &data.v_izq_ref, 0);
        xQueuePeek(ColaLecturaRampaDer, &data.v_der_ref, 0);
        //-----------------------VTOTAL-----------------------

        //-----------------------VTOTAL REF-----------------------

        //-----------------------X Y ESTIMADO-----------------------
        xQueuePeek(ColaLectorXpos, &data.x_pos, 0);
        xQueuePeek(ColaLectorYpos, &data.y_pos, 0);

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
            //xQueueSend(ColaUsoDutyIzq, &data_leida.duty_izq ,0);   //COMENTA PARA DESACTIVAR Y QUE SOLO FUNCIONE LA OTRA
            //XqueueSend(ColaUsoDutyIzq, &data_leida.duty_der ,0); 

            //------------------TETA (referencia para el control de angulo)----------------
            xQueueSend(ColaUsoTetaRef,      &data_leida.teta_ref, 0);
            xQueueOverwrite(ColaLecturaTetaRef, &data_leida.teta_ref); // telemetria

            // ------------------VREF_IZQ_DER----------------
            // CONTROL EN CASCADA: la referencia es ahora el ANGULO; el
            // controlador de angulo es quien escribe ColaUsoVREFIzq/Der.
            // Para pruebas de velocidad directa, descomentar estas 2 lineas
            // (y comentar el envio de teta_ref de arriba) para no competir.
            //xQueueSend(ColaUsoVREFIzq, &data_leida.v_izq_ref, 0);
            //xQueueSend(ColaUsoVREFDer, &data_leida.v_der_ref, 0);

            // ------------------RESET DE POSE (punto de entrada preparado)----------------
            // Cuando la Jetson envie una pose absoluta (ej. desde un ArUco),
            // llamar a solicitarResetPose(...). Hoy x_ref/y_ref se usan como
            // objetivo, no como reset, por lo que se deja documentado:
            //solicitarResetPose(data_leida.x_ref, data_leida.y_ref, data_leida.teta_ref);

            // ------------------V_TOTAL_REF----------------


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

    //---------------------COLAS ESTIMADOR---------------------
     ColaLectorTeta = xQueueCreate(1, sizeof(float));
     ColaUsoTeta    = xQueueCreate(1, sizeof(float));
     ColaLectorXpos = xQueueCreate(1, sizeof(int32_t));
     ColaLectorYpos = xQueueCreate(1, sizeof(int32_t));

    //---------------------COLAS REFERENCIA DE ANGULO---------------------
     ColaUsoTetaRef     = xQueueCreate(1, sizeof(float));
     ColaLecturaTetaRef = xQueueCreate(1, sizeof(float));

    //---------------------COLA VELOCIDAD DE AVANCE---------------------
     ColaUsoVelAvance = xQueueCreate(1, sizeof(float));
     { float _v0 = 0.0f; xQueueOverwrite(ColaUsoVelAvance, &_v0); } // arranca en 0

    //---------------------COLA RESET DE POSE---------------------
     ColaResetPose = xQueueCreate(1, sizeof(PoseReset));


    // USO MOTORES
    xTaskCreatePinnedToCore(
        motorDerechoSwitchTask, //funcion
        "motordtask", //nombre
        4096, //habra que analizarlo
        NULL, //parametros
        5, // Prioridad analizar
        NULL, //handel
        1 //core
    );
    xTaskCreatePinnedToCore(
        motorizquierdoSwitchTask, //funcion
        "motoritask", //nombre
        4096, //habra que analizarlo
        NULL, //parametros
        5, // Prioridadanalizar
        NULL, //handel
        1 //core
    );

     // JETOSN Y ESP
     // En modo banco de pruebas NO se crean: el envio es binario y se
     // mezclaria con el log de texto del estimador en el mismo puerto serial.
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
    xTaskCreatePinnedToCore(leerJetsonTaks,  "leerdatos", 4096,  NULL, 5, NULL, 0);
#endif

    // CONTROLADOR
    //xTaskCreatePinnedToCore(pasar_rampa_izq_task,  "rampaizq",4096,  NULL, 3, NULL, 1);   ESTA DANDO MUCHOS PROBELMAS LA RAMPA, LA bypaseare
    xTaskCreatePinnedToCore(pidMotorIzqTask,  "pidizq",4096,  NULL, 8, NULL, 1);
    xTaskCreatePinnedToCore(pidMotorDerTask,  "pidder",4096,  NULL, 8, NULL, 1);

    //ENCODER
    xTaskCreatePinnedToCore(lecturaEncoderIzqTask,  "encizq",4096,  NULL, 7, NULL, 1);
    xTaskCreatePinnedToCore(lecturaEncoderDerTask,  "encder",4096,  NULL, 7, NULL, 1);

    //ESTIMADOR DE POSICION Y CONTROL DE ANGULO
    xTaskCreatePinnedToCore(estimadorPosicionTask, "estimador",  4096, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(controladorAnguloTask, "ctrlangulo", 4096, NULL, 6, NULL, 1);

    //RUTINA DE PRUEBA (cuadrado), solo si esta habilitada
#if RUTINA_CUADRADO
    xTaskCreatePinnedToCore(rutinaCuadradoTask, "rutina", 4096, NULL, 4, NULL, 1);
#endif

}