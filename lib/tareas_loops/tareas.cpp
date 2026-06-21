#include <Arduino.h>
#include <motor.h>
#include <conexion_jetson.h>
#include <tareas.h>
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


ESP32Encoder encoderDer;
ESP32Encoder encoderIzq;

float wrap180(float ang) {
  while (ang >   180.0f) ang -= 360.0f;
  while (ang <= -180.0f) ang += 360.0f;
  return ang;
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
  float v_total_ref;
  float x_ref;
  float y_ref;  // hasta aca parecen haber 57 bytes

};
/// ---------------------MIS FUNCIONES ------------------------

// ------------- SWITCH MOTORES ----------------

void motorDerechoSwitchTask(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();

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
            xQueueSend(ColaLectorDutyDer, &velocidad_nueva,0);
        }
    }
}
void motorizquierdoSwitchTask(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int velocidad_nueva = 0;
    
    for(;;){
        if (xQueueReceive(ColaUsoDutyIzq, &velocidad_nueva, portMAX_DELAY) == pdTRUE) {
            cambiar_velocidad_izquierda(velocidad_nueva);
        }
    }
}
//---------------- CONTROL VELOCIDAD----------------
//PID
void pidMotorIzqTask(void *pvparameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_PID_MOTORES);

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
    Kp_motor_izquierdo, Ki_motor_izquierdo, Kd_motor_izquierdo, 
    QuickPID::pMode::pOnError, 
    QuickPID::dMode::dOnMeas, 
    QuickPID::iAwMode::iAwCondition, // <-- CORRECCIÓN: Modo Anti-Windup añadido
    QuickPID::Action::direct
    );
    
    //PARAMETROS
    pid.SetOutputLimits(LIMITE_NEGATIVO_PID_MOTOR, LIMITE_POSITIVO_PID_MOTOR);
    pid.SetSampleTimeUs(FRECUENCIA_PID_MOTORES * 1000);  // la frecuencia pal pid      
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
        xQueueOverwrite(ColaLectorDutyIzq, &v_out); // para que el envio a jetson tenga la ultima wea

   
    // DELAY
        vTaskDelayUntil(&xLastWakeTime, xfrec); 
    }
}
void pidMotorDerTask(void *pvparameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_PID_MOTORES);

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
    Kp_motor_derecho, Ki_motor_derecho, Kd_motor_derecho,
    QuickPID::pMode::pOnError, 
    QuickPID::dMode::dOnMeas, 
    QuickPID::iAwMode::iAwCondition, // <-- CORRECCIÓN: Modo Anti-Windup añadido
    QuickPID::Action::direct
    );
    
    //PARAMETROS
    pid.SetOutputLimits(LIMITE_NEGATIVO_PID_MOTOR, LIMITE_POSITIVO_PID_MOTOR);
    pid.SetSampleTimeUs(FRECUENCIA_PID_MOTORES * 1000);  // la frecuencia pal pid      
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
        int duty_magnitud = calcularDuty(abs_velocidad_ref, v_out, M2_m, M2_b, M2_MIN_DUTY);

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

        // Modelo uniciclo: velocidad de avance comun + correccion diferencial.
        // Convencion horario+: para girar horario (error>0) la rueda izquierda
        // va mas rapido que la derecha.
        v_angular = v_avance + v_diff;   // Aca lo cambie a v_angular, en otro lado se transforma a velocidad de cada rueda.;

        // ENVIO a los PID de velocidad (lazo interno de la cascada)
        xQueueSend(ColaUsoVelAng, &v_angular, 0);   //  envia a v angular
        xQueueSend(ColaLecturaVelAng, &v_angular,0);
        // DELAY
        vTaskDelayUntil(&xLastWakeTime, xfrec);
    }
}

//Conversor V_total y V_angular a V_der V_izq
void asignacionVelocidadRuedasTask(void *pvParameters) {
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_CONTROL_ANGULO); // define tu frecuencia
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float velocidad_total = 0.0f;
    float velocidad_angular_nueva = 0.0f;
    float v_der_out = 0.0f;
    float v_izq_out = 0.0f;

    for (;;) {

        xQueueReceive(ColaUsoVelAng, &velocidad_angular_nueva, 0);
        xQueueReceive(ColaUsoVREFTotal, &velocidad_total, 0);

        v_izq_out = (2* velocidad_total - 0 * LARGO_ENTRE_RUEDAS) / (2 * RADIO_DE_RUEDA);
        v_der_out =  velocidad_total * (2/RADIO_DE_RUEDA) - v_izq_out;

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

// -----------------LECTURA DE SENSORES----------------
//ENCODERS
void lecturaEncoders(void *pvparaameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_ENCODER);      
    //izquierda
    float velocidad_leida_izq = 0;
    float velocidad_leida_izq_anterior = 0;
    float distancia_del_ciclo_izq_filtrada = 0; 
    float distancia_ciclo_izq_anterior = 0;
    float distancia_del_ciclo_izq = 0;
    float aceleracion_izq = 0;
    //parametro
    float alpha = 0.2;
    //derecha
    float velocidad_leida_der = 0;
    float velocidad_leida_der_anterior = 0;
    float distancia_del_ciclo_der_filtrada = 0; 
    float distancia_ciclo_der_anterior = 0;
    float distancia_del_ciclo_der = 0;
    float aceleracion_der = 0;
    // LOOP
    for (;;){
        // Se cuentan los 2
        int64_t count_actual_izq = encoderIzq.getCount();      
        encoderIzq.clearCount();
        int64_t count_actual_der = encoderDer.getCount();      
        encoderDer.clearCount();
        distancia_del_ciclo_izq = count_actual_izq * CM_POR_PULSO;
        distancia_del_ciclo_izq_filtrada = alpha * distancia_del_ciclo_izq + (1-alpha) * distancia_ciclo_izq_anterior;
        distancia_ciclo_izq_anterior = distancia_del_ciclo_izq_filtrada;

        velocidad_leida_izq =  distancia_del_ciclo_izq_filtrada * 1000.0f / FRECUENCIA_ENCODER;  //   cm/s
        aceleracion_izq = velocidad_leida_izq - velocidad_leida_izq_anterior * 1000.0f / FRECUENCIA_ENCODER;
        velocidad_leida_izq_anterior = velocidad_leida_izq;


        distancia_del_ciclo_der = count_actual_der * CM_POR_PULSO;
        distancia_del_ciclo_der_filtrada = alpha * distancia_del_ciclo_der + (1-alpha) * distancia_ciclo_der_anterior;
        distancia_ciclo_der_anterior = distancia_del_ciclo_der_filtrada;

        velocidad_leida_der =  distancia_del_ciclo_der_filtrada * 1000.0f / FRECUENCIA_ENCODER;  //   cm/s
        aceleracion_der = (velocidad_leida_der - velocidad_leida_der_anterior)* 1000.0f / FRECUENCIA_ENCODER;
        velocidad_leida_der_anterior  =velocidad_leida_der;

        if (abs(distancia_del_ciclo_der_filtrada) < 0.001){distancia_ciclo_der_anterior = 0;}

        if (abs(distancia_del_ciclo_izq_filtrada) < 0.001){distancia_ciclo_izq_anterior = 0;}



        // Se lee la cuenta

        
        //ENVIAR
        xQueueOverwrite(ColaLectorVelIzq, &velocidad_leida_izq);  // para la lectura
        xQueueSend(ColaUsoVelIzq, &velocidad_leida_izq, 0);   // para activar el pid

        xQueueOverwrite(ColaLectorVelDer, &velocidad_leida_der);  // para la lectura
        xQueueSend(ColaUsoVelDer, &velocidad_leida_der, 0);   // para activar el pid

        // DELAY
        vTaskDelayUntil(&xLastWakeTime, xfrec);
    }
}

//IMU
void lecturaImuTask(void *pvparameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_LECTURA);



}

//Sensores de poscicion
void lecturaPosicionTask(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_LECTURA);
}
// ---------------CALCULO DE POSCICION---------------
//Estimador de poscicion
void estimadorDePoscicionTask(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;){
        // -- LENAR --
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
            if (false) {  // FALSE SI NO QUIERES USARLO
                xQueueSend(ColaUsoTetaRef, &data_leida.teta_ref, 0);
            }


            // ------------------VREF_IZQ_DER----------------
            if (true) { // FALSE SI NO QUIERES USARLO
            xQueueSend(ColaUsoVREFIzq, &data_leida.v_izq_ref, 0);
            xQueueSend(ColaUsoVREFDer, &data_leida.v_der_ref, 0);
            }

            // ------------------V_TOTAL_REF----------------
            if (false) { // FALSE SI NO QUIERES USARLO
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




    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoderIzq.attachFullQuad(pinB1, pinA1);
    encoderIzq.setFilter(1023);
    encoderIzq.clearCount();


    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoderDer.attachFullQuad(pinA2, pinB2);
    encoderDer.setFilter(1023);
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
        9, // Prioridadanalizar
        NULL, //handel
        1 //core
    );

     // JETOSN Y ESP
    xTaskCreatePinnedToCore(
      enviarJetsonTask, //funcion
       "enviodatos", //nombre
        4096, //habra que analizarlo Stack size (Bytes) //creo que esta es la cantidad maxima de data que puede sacar
        NULL, //parametros
    9, // Prioridadanalizar
       NULL, //handel
     0 //core
    );

    
    xTaskCreatePinnedToCore(leerJetsonTaks,  "leerdatos", 2048,  NULL, 4, NULL, 0);

    // CONTROLADOR
    //xTaskCreatePinnedToCore(pasar_rampa_izq_task,  "rampaizq",4096,  NULL, 3, NULL, 1);   ESTA DANDO MUCHOS PROBELMAS LA RAMPA, LA bypaseare
    //MOTOR
    xTaskCreatePinnedToCore(pidMotorIzqTask,  "pidizq",4096,  NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(pidMotorDerTask,  "pidder",4096,  NULL, 4, NULL, 1);

    //ANGULO
    //Pid
    //xTaskCreatePinnedToCore(pidControlDireccionAngularTask,  "pid_ang",4096,  NULL, 6, NULL, 1);
    //conversor
    //xTaskCreatePinnedToCore(asignacionVelocidadRuedasTask,  "Conversor",4096,  NULL, 5, NULL, 1);

    //ENCODER
    xTaskCreatePinnedToCore(lecturaEncoders,  "encizq",3072,  NULL, 7, NULL, 1);
    ///xTaskCreatePinnedToCore(lecturaEncoderIzqTask,  "encizq",3072,  NULL, 7, NULL, 1);
    //xTaskCreatePinnedToCore(lecturaEncoderDerTask,  "encder",3072,  NULL, 8, NULL, 1);


}










































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