#include <Arduino.h>
#include <motor.h>
#include <conexion_jetson.h>
#include <tareas.h>
#include <string.h>

// --- Configuración de FreeRTOS ---
// Handles para las tareas
TaskHandle_t TaskControlHandle;  // que es un handler???
TaskHandle_t TaskSerialHandle; //???

// los handlers son como admins, con ellos puedes hacer vainas
// en otras tareas llamandolo desde otra tarea.

// Queues para comunicación entre núcleos (Thread-safe)
QueueHandle_t sensorQueue; /// esta wea es el mensajero de entre datos
// cada wea que pase de un nucleo a otro tiene que llevar este
// de ahi defini una funcion pa darle el mensaje al cartero
QueueHandle_t commandQueue;


// ------------------ACA VAN LOS MIOS ------------------------

QueueHandle_t colaDutyDer;
QueueHandle_t colaDutyIzq;



//---------------- VARAIABLES CORE 0 --------------------

volatile int32_t velocidad_izquierda_core0 = 0;
volatile int32_t velocidad_derecha_core0 = 0;


// ----------------VARIABLES CORE 1 -----------------






// Estructuras de datos para las colas
struct TelemetryData {
    float angle; // aca demas que deberia meter la wea en el de conexion serial.
    int encoderCount;
};


// ---------MIOS -----------

struct __attribute__((packed)) Envio {
  uint8_t header = 0xAA; // Byte de inicio para sincronizar
  int32_t duty_izq;      // un por ahora dejemoslo como
  int32_t duty_der;        // un byte
  float teta;
  float teta_ref;
  int32_t v_der;
  int32_t v_izq;
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
  int32_t v_der_ref;
  int32_t v_izq_ref;
  int32_t v_total_ref;
  int32_t x_ref;
  int32_t y_ref;  // hasta aca parecen haber 57 bytes

};




/// -------------MIS FUNCIONES ----------------

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
        if (xQueueReceive(colaDutyDer, &velocidad_nueva, portMAX_DELAY) == pdTRUE){
            cambiar_velocidad_derecha(velocidad_nueva);
        }
    }
}

void motorizquierdoSwitchTask(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_MOTORES);
    int velocidad_nueva = 0;
    
    for(;;){
        if (xQueueReceive(colaDutyIzq, &velocidad_nueva, portMAX_DELAY) == pdTRUE) {
            cambiar_velocidad_izquierda(velocidad_nueva);
        }
    }
}


void enviarJetsonTask(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_LECTURA);
    Envio data;
    memset(&data, 0, sizeof(data)); //inicio todos en 0
    data.header = 0xAA;  // asigno el header
 
    for (;;){
        data.duty_der = velocidad_izquierda_core0;
        data.duty_izq = velocidad_derecha_core0 ;

        // Cálculo simple de checksum (XOR de los datos)
        //data.checksum = (uint8_t)(data.velocidad_izquierda ^ data.velocidad_derecha);

        // Enviamos el bloque de memoria completo (binario puro)
        Serial.write((uint8_t*)&data, sizeof(data));

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
        //Aca envio a la cola de comandos

        xQueueSend(colaDutyDer, &data_leida.duty_der ,0);
        xQueueSend(colaDutyIzq, &data_leida.duty_izq ,0);
        velocidad_derecha_core0 = data_leida.duty_der;
        velocidad_izquierda_core0 = data_leida.duty_izq;

        
        while (Serial.available() > 0) {
                Serial.read(); // Descartamos el byte
            }
        }

        else {
            // Si el header está mal, el buffer está desincronizado
            // Limpiamos todo
            while (Serial.available() > 0) {
                Serial.read(); // Descartamos el byte
                }
            }   
        
        }
    vTaskDelayUntil(&xLastWakeTime, xfrec);
    }
}


void lecturaImuTask(void *pvparameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_LECTURA);
}



/// ------------------------------------EJEMPLOS ------------------------------------
// --- TAREA 1: CONTROL (Core 1) ---
// Alta prioridad: Lectura de sensores y cálculo matemático
void taskControl(void *pvParameters) {
    TelemetryData data;      // crea el objeto
    TickType_t xLastWakeTime = xTaskGetTickCount();  // esta funcion te devuelve cuantos ticks ha estado prendida la esp
    const TickType_t xFrequency = pdMS_TO_TICKS(10); //milisegundos a ticks, 10 miliseg son 100hz

    for (;;) {
        // Simulación: Lectura de IMU ICM20948
        data.angle = 45.0 + random(-5, 5);  //picio
        data.encoderCount++;

        // Enviamos datos a la cola de telemetría para que el Core 0 los despache
        // xQueueSend no bloquea esta tarea si la cola está llena (0 wait time)
        xQueueSend(sensorQueue, &data, 0); 
        // el primer dato que recibe la funcion es el nombre de la cola que definiste antes
        // despues recibe los datos a enviar
        // el tercero es cuanto tiempo hay que esperar si es que cacha que el otro nucleo esta usando la wea
        // en este caso con un 0 si la wea la esta usando el datos a python se pierde nomas.

        // vTaskDelayUntil garantiza que el periodo sea constante (sin jitter)
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        //  esta funcion toma de parametros el ultimo tiempo  se prendio
        // y el segundo parametro es "cuantos ticks hay que esperar"
        // como ya definimos los ticks a frecuencia, con ese dato podemos forzar a que trabaje siempre a la 
        // misma frecuencia
    }
}

// --- TAREA 2: MOTORES (Core 1) ---
// Prioridad Media: Aplicar PWM a los drivers
void taskMotors(void *pvParameters) {
    float targetSpeed = 0; // esta variable queda interna para la tarea
    for (;;) {
        // Revisar si hay un nuevo comando de velocidad desde la Jetson
        if (xQueueReceive(commandQueue, &targetSpeed, 0) == pdTRUE) {
            // el pdTRUE te dice cuando llego un dato, util para las weas que 
            // solo se ejecutan si hay un cambio
        }
       // vTaskDelay(pdMS_TO_TICKS(FRECUENCIA_MOTORES)); // 50Hz es suficiente para motores
        // este delay le vale pico las interrupciones asi que se puede demorar demas
        // por lo mismo en vola si quiero cambiar las velocidades al mismo tiempo
        // o lo hago todo en una o les pongo un delay until
    }
}



// --- TAREA 3: COMUNICACIÓN SERIAL (Core 0) ---
// Prioridad Media: Puente entre ESP32 y Jetson Nano
void taskSerial(void *pvParameters) {
    TelemetryData incomingData; // creas el objeto que recibira data
    for (;;) {
        // Si hay datos en la cola de sensores, los mandamos por Serial
        if (xQueueReceive(sensorQueue, &incomingData, pdMS_TO_TICKS(10)) == pdTRUE) {
            // recives la data y la pones en el incoming
            // timepo maximo 10 Ms
            //el and es para que el incoming data se vaya actualizando sobre si mismo.
            Serial.printf("DATA:%.2f,%d\n", incomingData.angle, incomingData.encoderCount);
        }

        // Leer comandos desde la Jetson
        if (Serial.available() > 0) {
            float cmdSpeed = Serial.parseFloat(); // parse float??
            xQueueSend(commandQueue, &cmdSpeed, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Respiro para el procesador
    }
}


 
void setup_rtos() {
    // Crear Colas: (Tamaño de la cola, tamaño de cada elemento)
    sensorQueue = xQueueCreate(10, sizeof(TelemetryData));
    commandQueue = xQueueCreate(5, sizeof(float));

    colaDutyDer = xQueueCreate(1, sizeof(int));
    colaDutyIzq = xQueueCreate(1, sizeof(int));



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
    xTaskCreatePinnedToCore(
      enviarJetsonTask, //funcion
       "enviodatos", //nombre
        4096, //habra que analizarlo Stack size (Bytes) //creo que esta es la cantidad maxima de data que puede sacar
        NULL, //parametros
    5, // Prioridadanalizar
       NULL, //handel
     0 //core
    );
    xTaskCreatePinnedToCore(leerJetsonTaks,  "leerdatos",4096,  NULL, 5, NULL, 0);
    

}
