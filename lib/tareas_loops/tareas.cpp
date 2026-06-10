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
//---------------------v_angular Teta---------------------
QueueHandle_t ColaUsoVAngular;
QueueHandle_t ColaLecturaVAngular;

//---------------------COLAS POSICION---------------------
//posicion medida  (Puedo ponerle 2 objetos a la queue)
QueueHandle_t ColaUsoPosicion;
QueueHandle_t ColaLecturaPosicion;
//posicion ref
QueueHandle_t ColaUsoPosicionRef;
QueueHandle_t ColaLecturaPosicionRef;
struct PosicionRef {
    float x;
    float y;
};



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
  float w_ref;
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
    float kp = Kp_motor_izquierdo;
    float ki = Ki_motor_izquierdo;
    float kd = Kd_motor_izquierdo;


    //SETEO EL PID
    QuickPID pid(
    &abs_velocidad_actual, &v_out, &abs_velocidad_ref,   // les puse el absoluto de una
    kp, ki, kd, 
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
    float kp = Kp_motor_derecho;
    float ki = Ki_motor_derecho;
    float kd = Kd_motor_derecho;

    //SETEO EL PID
    QuickPID pid(
    &abs_velocidad_actual, &v_out, &abs_velocidad_ref,   // les puse el absoluto de una
    kp, ki, kd, 
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
void pidControlDireccionAngularTask(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_MOTORES);

    for(;;){


    }
}

void velocidadAngularVtotaltask(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    float velocidad_total = 0;
    float velocidad_angular_nueva = 0;
    float v_der_out = 0;
    float v_izq_out = 0;
    for (;;){
        if (xQueueReceive(ColaUsoVAngular, &velocidad_angular_nueva, portMAX_DELAY) == pdTRUE || xQueueReceive( ColaUsoVREFTotal, &velocidad_total, portMAX_DELAY) == pdTRUE){
        //uso la formula
        v_izq_out = (2* velocidad_total - velocidad_angular_nueva * LARGO_ENTRE_RUEDAS) / RADIO_DE_RUEDA;
        v_der_out = velocidad_total * (2/RADIO_DE_RUEDA) - v_izq_out;
        //ENVIO
        xQueueSend(ColaUsoVREFIzq, &v_izq_out, 0);
        xQueueSend(ColaUsoVREFDer, &v_der_out, 0);
        }
    }
}

//--------------- CONTROL DE POSCICION -----------------
void pidControlPosicionTask(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;){
    // 
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

//IMU
void lecturaImuTask(void *pvparameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_LECTURA);



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
        xQueuePeek(ColaLectorVelDer, &data.v_der, 0);

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
            if (true) { // FALSE SI NO QUIERES USARLO
            xQueueSend(ColaUsoDutyIzq, &data_leida.duty_izq ,0);   //COMENTA PARA DESACTIVAR Y QUE SOLO FUNCIONE LA OTRA
            xQueueSend(ColaUsoDutyDer, &data_leida.duty_der ,0); 
            }
            //------------------TETA----------------
            if (true) {  // FALSE SI NO QUIERES USARLO
                xQueueSend(ColaUsoTetaRef, &data_leida.teta_ref, 0);
            }


            // ------------------VREF_IZQ_DER----------------
            if (true) { // FALSE SI NO QUIERES USARLO
            xQueueSend(ColaUsoVREFIzq, &data_leida.v_izq_ref, 0);
            xQueueSend(ColaUsoVREFDer, &data_leida.v_der_ref, 0);
            }
            // ------------------V_TOTAL_REF----------------
            if (true) { // FALSE SI NO QUIERES USARLO
                xQueueSend(ColaUsoVREFTotal, &data_leida.v_total_ref, 0);
            }

            // ------------------X_REF_Y_REF----------------
            if (true) { // FALSE SI NO QUIERES USARLO
                PosicionRef pos = {data_leida.x_ref, data_leida.y_ref};
                xQueueSend(ColaUsoPosicionRef, &pos, portMAX_DELAY);

            }

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

    // CONTROLADOR
    //xTaskCreatePinnedToCore(pasar_rampa_izq_task,  "rampaizq",4096,  NULL, 3, NULL, 1);   ESTA DANDO MUCHOS PROBELMAS LA RAMPA, LA bypaseare
    xTaskCreatePinnedToCore(pidMotorIzqTask,  "pidizq",4096,  NULL, 8, NULL, 1);
    xTaskCreatePinnedToCore(pidMotorDerTask,  "pidder",4096,  NULL, 8, NULL, 1);

    //ENCODER
    xTaskCreatePinnedToCore(lecturaEncoderIzqTask,  "encizq",4096,  NULL, 7, NULL, 1);
    xTaskCreatePinnedToCore(lecturaEncoderDerTask,  "encder",4096,  NULL, 7, NULL, 1);

}
