#include <Arduino.h>
#include <motor.h>
#include <conexion_jetson.h>
#include <tareas.h>
#include <string.h>
#include <math.h>


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
void pidMotorIzqTask(void *pvparameters){

    float velocidad_actual = 0.0f;
    float v_out = 0.0f;
    float v_ref_suavizada = 0.0f;
    const float alpha = 0.1f;
    float v_ref_nueva;

    int   duty   = 0;

    pidIzq.SetOutputLimits(-V_CADA_RUEDA_MAX, V_CADA_RUEDA_MAX);
    pidIzq.SetSampleTimeUs(FRECUENCIA_ENCODER * 1000.0f); // dt real del dato
    pidIzq.SetMode(QuickPID::Control::timer);

    for (;;) {
        // Bloquea hasta que hay una lectura NUEVA del encoder
        xQueueReceive(ColaUsoVelIzq, &velocidad_actual, portMAX_DELAY);
        // v_ref es asincrono: tomo el ultimo valor sin bloquear
        xQueuePeek(ColaUsoVREFIzq, &v_ref_nueva, 0);
        
        //v_ref_suavizada = alpha * v_ref_nueva + (1.0f - alpha) * v_ref_suavizada;


        abs_velocidad_actual_izq = abs(velocidad_actual);
        abs_velocidad_ref_izq    = abs(v_ref_nueva);

        pidIzq.Compute();

        int duty_magnitud = calcularDuty(abs_velocidad_ref_izq, v_out_izq, M1_m, M1_b, M1_MIN_DUTY);
        duty = (v_ref_nueva < 0) ? -duty_magnitud : duty_magnitud;

        xQueueSend(ColaUsoDutyIzq, &duty, 0);
        xQueueOverwrite(ColaLectorDutyIzq, &duty); // antes mandaba v_out, corregido
    }
}
void pidMotorDerTask(void *pvparameters){

    float velocidad_actual = 0.0f;
    float v_ramp_ref       = 0.0f;
    int   duty              = 0;


    pidDer.SetOutputLimits(-V_CADA_RUEDA_MAX, V_CADA_RUEDA_MAX);
    pidDer.SetSampleTimeUs(FRECUENCIA_ENCODER * 1000);
    pidDer.SetMode(QuickPID::Control::timer);

    for (;;) {
        xQueueReceive(ColaUsoVelDer, &velocidad_actual, portMAX_DELAY);
        xQueuePeek(ColaUsoVREFDer, &v_ramp_ref, 0);

        //Podria intentar escalonar el ref
        // casos raros   20 + 15 = 10  //   -40 - 35 
        abs_velocidad_actual_der = abs(velocidad_actual);
        abs_velocidad_ref_der    = abs(v_ramp_ref);

        pidDer.Compute();

        int duty_magnitud = calcularDuty(abs_velocidad_ref_der, v_out_der, M2_m, M2_b, M2_MIN_DUTY);
        duty = (v_ramp_ref < 0) ? -duty_magnitud : duty_magnitud;

        xQueueSend(ColaUsoDutyDer, &duty, 0);
        xQueueOverwrite(ColaLectorDutyDer, &duty);
    }
}

// --------------- CONTROL DE ANGULO ------------------
// PID
void pidControlDireccionAngularTask(void *pvParameters){

    pidAngulo.SetOutputLimits(-VEL_GIRO_MAX, VEL_GIRO_MAX);
    pidAngulo.SetSampleTimeUs(FRECUENCIA_ENCODER * 1000);
    pidAngulo.SetMode(QuickPID::Control::timer);


    float teta_med = 0.0f;
    float v_avance = 0.0f;   // velocidad de avance comun (rutina/secuenciador)
    float v_angular = 0.0f;
    float error = 0.0f;
    float v_der_out = 0.0f;
    float v_izq_out = 0.0f;
    float velocidad_total = 0.0f;
    // LOOP
    for (;;){
        // LEO REFERENCIA, MEDIDA Y VELOCIDAD DE AVANCE
        xQueueReceive(ColaUsoTeta,    &teta_med, portMAX_DELAY);
        xQueuePeek(ColaUsoTetaRef,   &teta_ref ,0);
        xQueuePeek(ColaUsoVREFTotal, &velocidad_total,0);
        

        // "Desenrollamos" la medida alrededor de la referencia para que el PID
        // vea el error mas corto en (-180,180] sin saltos en el wrap.


        //Voy a dejar la parte del wrap en comentado por ahora
        error = wrap180(teta_ref - teta_med);
        teta_actual = teta_ref - error;  // => (teta_ref - teta_actual) = error

        pidAngulo.Compute();

        v_angular = v_diff;   // Aca lo cambie a v_angular

        if (fabs(v_angular) < 0.1){v_angular = 0.0f;} //Para que no oscile por tonteras

        //Los transformo a velocidad de cada rueda
        v_izq_out =  velocidad_total  -( v_angular * LARGO_ENTRE_RUEDAS)/ 2.0f ;
        
        v_der_out = 2.0 * velocidad_total - v_izq_out;

        
        
        xQueueOverwrite(ColaUsoVREFIzq, &v_izq_out);
        xQueueOverwrite(ColaUsoVREFDer, &v_der_out);
        xQueueOverwrite(ColaLecturaVelAng, &v_angular);
        xQueueOverwrite(ColaLecturaVREFIzq, &v_izq_out);
        xQueueOverwrite(ColaLecturaVREFDer, &v_der_out);
    }
}

// --------------- CONTROL DE POSCICION ------------------
void pidPosiciontask(void *pvparameters){
    pidPosicion.SetOutputLimits(-V_TOTAL_MAX, V_TOTAL_MAX);
    pidPosicion.SetSampleTimeUs(FRECUENCIA_ENCODER * 1000);    // LA FRECUENCIA A LA QUE LE DEBERIAN LLEGAR POSCICIONES NUEVAS
    pidPosicion.SetMode(QuickPID::Control::timer);

    Coordenadas pos_actual;
    Coordenadas pos_ref = {0,0};
    float x_actual = 0.0f;
    float y_actual = 0.0f;
    float x_ref = 0.0f; 
    float y_ref = 0.0f;
    float delta_x = 0.0f;
    float delta_y = 0.0f;
    float theta_out = 0.0f;
    float theta_actual = 0.0f;


    for (;;){
    // Se actualiza por cada pos nueva
    xQueueReceive(ColaUsoPosicion, &pos_actual, portMAX_DELAY);
    xQueuePeek(ColaUsoPosicionRef, &pos_ref, 0);
    xQueuePeek(ColaLecturaTeta, &theta_actual,0);


    x_actual = pos_actual.x;
    y_actual = pos_actual.y;
    x_ref = pos_ref.x;
    y_ref = pos_ref.y;


    //calculo deltas para trigonometria
    delta_x = x_ref - x_actual;
    delta_y = y_ref - y_actual;

    theta_out = atan2f(delta_y, delta_x); // Retorna radiantes entre    PI y -PI
    theta_out = (theta_out * 180.0f) / (M_PI); //se convierte a grados 
    
    if (theta_out < 0){theta_out = 360 + theta_out;} // Se pasan a full positivo

    // delta D
    delta_d = abs(delta_x) + abs(delta_y);




        // Sin el delta umbral oscila mucho antes de llegar a la poscicion
    if (delta_d < UMBRAL_LLEGADA_POS) {
        // Llegó al target: detener y resetear integral
        v_total_out = 0.0f;
        pidPosicion.Reset();
    } else {
        pidPosicion.Compute();
    }


    // obligo a que ruede sobre si mismo si es que tiene un angulo muy grande.
    if (abs(wrap180(theta_out - theta_actual)) > 10) {
        v_total_out = 0.0f;
    } else {
    }



    // El modo freno de emergencia lanza valores negativos de referencia.
    if (x_ref < -1 || y_ref < -1){
        theta_out = theta_actual;
        pidPosicion.Reset();


    }

    xQueueOverwrite(ColaUsoVREFTotal, &v_total_out);
    xQueueOverwrite(ColaLecturaVREFTotal, &v_total_out);

    xQueueOverwrite(ColaUsoTetaRef, &theta_out);
    xQueueOverwrite(ColaLecturaTetaRef, &theta_out);


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
    //supuesto delta teta
    float vel_angular = 0;
    float delta_theta = 0;
    float theta_calculado = 0;
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

        if (abs(distancia_del_ciclo_der_filtrada) < 0.001){distancia_ciclo_der_anterior = 0.0f;}

        if (abs(distancia_del_ciclo_izq_filtrada) < 0.001){distancia_ciclo_izq_anterior = 0.0f;}

        vel_angular = (velocidad_leida_der - velocidad_leida_izq)/ LARGO_ENTRE_RUEDAS;
        delta_theta = (vel_angular * (FRECUENCIA_ENCODER / 1000.0f)) * (180.0f / 3.14f);
        theta_calculado = theta_calculado + delta_theta;
        
        while (theta_calculado >= 360.0f) {
            theta_calculado -= 360.0f;
        }
        while (theta_calculado < 0.0f) {
            theta_calculado += 360.0f;
}

        



        // Se lee la cuenta

        
        //ENVIAR
        xQueueOverwrite(ColaLectorVelIzq, &velocidad_leida_izq);  // para la lectura
        xQueueSend(ColaUsoVelIzq, &velocidad_leida_izq,0);   // para activar el pid

        xQueueOverwrite(ColaLectorVelDer, &velocidad_leida_der);  // para la lectura
        xQueueOverwrite(ColaUsoVelDer, &velocidad_leida_der);   // para activar el pid

        xQueueSend(ColaUsoTeta, &theta_calculado,0);   // DEMAS QUE AHI  tengo que añadir theta encoder y theta datosimu
        xQueueOverwrite(ColaLecturaTeta, &theta_calculado);

        // DELAY
        vTaskDelayUntil(&xLastWakeTime, xfrec);
    }
}

//IMU
void lecturaImuTask(void *pvparameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_LECTURA);
    
    Adafruit_ICM20948 icm;
    DatosImu datosimu;
    float yaw_acumulado = 0.0f;
    float tiempo_anterior = 0.0f;
    float bufGyroZ[3] = {0.0f, 0.0f, 0.0f};
    float bufAccelX[3] = {0.0f, 0.0f, 0.0f};
    float sentido_giro = 1.0f;




    for(;;){
        sensors_event_t accel, gyro, mag, temp;
        icm.getEvent(&accel, &gyro, &temp, &mag);

        // ── 1. Calcular dt ───────────────────────────────────────────
        unsigned long ahora = micros();
        float dt = (float)(ahora - tiempo_anterior) * 1e-6f; // segundos
        tiempo_anterior = ahora;

        // Guardia de seguridad: dt invalido -> devolvemos el ultimo estado sin integrar
        if (dt <= 0.0001f || dt > 0.5f) {
            datosimu.vel_angular = 0.0f;
            datosimu.pos_angular   = yaw_acumulado;
            datosimu.aceleracion_lineal   = 0.0f;
            return;
        }

        // Quitar el noise floor (calibracion) ───────────────────
        float gz = gyro.gyro.z - gyroOffsetZ;             // gyroZ corregido [rad/s]
        float ax = accel.acceleration.x - accelOffsetX;   // accelX corregido [m/s^2]

        // Media movil de las ultimas 3 muestras ──────
        bufGyroZ[2]  = bufGyroZ[1];  bufGyroZ[1]  = bufGyroZ[0];  bufGyroZ[0]  = gz;
        bufAccelX[2] = bufAccelX[1]; bufAccelX[1] = bufAccelX[0]; bufAccelX[0] = ax;

        gz = (bufGyroZ[0]  + bufGyroZ[1]  + bufGyroZ[2])  / 3.0f;
        ax = (bufAccelX[0] + bufAccelX[1] + bufAccelX[2]) / 3.0f;

        // ── 4. Velocidad angular en grados/s con signo de la convencion ──
        float gyroZ_dps = sentido_giro * gz * 180.0f / PI; // horario+

        // ── 5. Integracion de Euler con zona muerta anti-drift ───────
        if (fabsf(gyroZ_dps) > Umbral_deslizamiento) {
            yaw_acumulado += gyroZ_dps * dt;
        }
        yaw_acumulado = wrap180(yaw_acumulado);

        // ── 6. Salidas ───────────────────────────────────────────────
        datosimu.vel_angular = gyroZ_dps;
        datosimu.pos_angular   = yaw_acumulado;
        datosimu.aceleracion_lineal   = ax * 100.0f; //  cm/s^2 

        //xQueueOverwrite(ColaLecturaVelAng, &datosimu.omega_dps);
}

}

//Sensores de poscicion
void SensoresPosicionTask(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_LECTURA);

    float distancia_izquierda = 0.0f;
    float distancia_derecha = 0.0f;
    float distancia_adelante = 0.0f;

    VL53L1X sensor_adelante;
    VL53L1X sensor_izquierda;
    VL53L1X sensor_derecha;

    Sensores_distancia sensores; //   VL53L1X sensores[3];


    for(;;){

    sensores.izquierda = sensor_adelante.read()/10.0; //distancia en cm
    sensores.derecha = sensor_izquierda.read()/10.0;
    sensores.adelante = sensor_derecha.read()/10;

    xQueueOverwrite(ColaLecturaSensores, &sensores);
    }
}
// ---------------CALCULO DE POSCICION---------------
//Estimador de poscicion
void estimadorDePoscicionTask(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xfrec = pdMS_TO_TICKS(FRECUENCIA_ENCODER); 
    float v_der;
    float v_izq;
    float v_total;
    float theta = 0.0f;
    float delta_x;
    float delta_y;
    float x_out = 0.0f;
    float y_out = 0.0f;

    int reset_flag = 0;

    Coordenadas pos;

    Coordenadas pos_ref;

    for (;;){
        // Se leen las velocidades 
        // === Falta pensar bien si el filtro hacerlo aca, en la datosimu o donde ====
        xQueuePeek(ColaLectorVelDer, &v_der, 0);
        xQueuePeek(ColaLectorVelIzq, &v_izq, 0);
        xQueuePeek(ColaLecturaTeta, &theta, 0);
        xQueuePeek(ColaUsoPosicion, &pos_ref, 0);

        v_total = (v_der + v_izq) / 2.0f ;
        theta = theta * (M_PI / 180.0f); // Lo paso a rad/s
        delta_x = (v_total) * cosf(theta) * (FRECUENCIA_ENCODER / 1000.0f); //ACA HAY QUE PONER LA FRECUENCIA A LA QUE SE CALCULO LA VELOCIDAD, HAY QUE ANDAR FIJANDOSE.
        delta_y = (v_total) * sinf(theta) * (FRECUENCIA_ENCODER/ 1000.0f);

        x_out = x_out + delta_x;
        y_out = y_out + delta_y;



        //VOY A DEJAR EL RESET SUJETO AL V_IZQ_REF,   ==================== RECUERDA PULIRLO=================
        xQueuePeek(ColaUsoResetPos, &reset_flag,0);
        xQueuePeek(ColaLecturaPosicionRef, &pos_ref, 0);

        

        if (reset_flag){
            x_out = pos_ref.x;
            y_out = pos_ref.y;
        }


        pos.x = x_out;
        pos.y = y_out;





        xQueueSend(ColaUsoPosicion, &pos, 0);
        xQueueOverwrite(ColaLecturaPosicion, &pos);

        

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
    Coordenadas pos;
 
    for (;;){

        //-----------------------DUTY-----------------------
        //IZQ
        xQueuePeek(ColaLectorDutyIzq, &data.duty_izq, 0);
        xQueuePeek(ColaLectorDutyDer, &data.duty_der, 0);
        //-----------------------TETA-----------------------
        xQueuePeek(ColaLecturaTeta, &data.teta, 0);

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
        xQueuePeek(ColaLecturaVREFTotal, &data.v_total_ref,0);
        xQueuePeek(ColaLecturaTetaRef, &data.teta_ref,0);

        //-----------------------VTOTAL REF-----------------------

        //-----------------------X Y ESTIMADO-----------------------
        xQueuePeek(ColaLecturaPosicion, &pos, 0);
        data.x_pos = pos.x;
        data.y_pos = pos.y;
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
    Coordenadas pos;
    
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
            xQueueOverwrite(ColaUsoDutyIzq, &data_leida.duty_izq);   
            xQueueOverwrite(ColaUsoDutyDer, &data_leida.duty_der); 
            }


            // ------------------VREF_IZQ_DER----------------
            if (false) { // FALSE SI NO QUIERES USARLO
            xQueueOverwrite(ColaUsoVREFIzq, &data_leida.v_izq_ref);
            xQueueOverwrite(ColaUsoVREFDer, &data_leida.v_der_ref);
            }

            xQueueOverwrite(ColaUsoVREFIzq, &data_leida.v_izq_ref);

            // ------------------V_TOTAL_REF Teta_REF----------------
            if (false) { // FALSE SI NO QUIERES USARLO
                xQueueOverwrite(ColaUsoVREFTotal, &data_leida.v_total_ref);
                xQueueOverwrite(ColaUsoTetaRef, &data_leida.teta_ref);
            }
            

            // ------------------X_REF_Y_REF----------------
            if (true){
                pos.x = data_leida.x_ref;
                pos.y =  data_leida.y_ref;
                xQueueOverwrite(ColaUsoPosicionRef, &pos);

                xQueueOverwrite(ColaLecturaPosicionRef, &pos);
            }


            xQueueOverwrite(ColaUsoResetPos, &data_leida.reset_pos);
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

    // ------------------ Cola Lectura sensores de posicion ------------
    ColaLecturaSensores = xQueueCreate(1, sizeof(Sensores_distancia));

    // ------------------ Colas Poscicion Cartesiana ------------
    ColaUsoPosicion = xQueueCreate(1, sizeof(Coordenadas));
    ColaLecturaPosicion = xQueueCreate(1, sizeof(Coordenadas));
    //pos ref
    ColaUsoPosicionRef = xQueueCreate(1, sizeof(Coordenadas));
    ColaLecturaPosicionRef = xQueueCreate(1, sizeof(Coordenadas));
    //reset pos
    ColaUsoResetPos = xQueueCreate(1, sizeof(float));




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
        1536, //habra que analizarlo
        NULL, //parametros
        10, // Prioridad analizar
        NULL, //handel
        1 //core
    );
    xTaskCreatePinnedToCore(
        motorizquierdoSwitchTask, //funcion
        "motoritask", //nombre
        1536, //habra que analizarlo
        NULL, //parametros
        9, // Prioridadanalizar
        NULL, //handel
        1 //core
    );

     // JETOSN Y ESP
    xTaskCreatePinnedToCore(
      enviarJetsonTask, //funcion
       "enviodatos", //nombre
        3072, //habra que analizarlo Stack size (Bytes) //creo que esta es la cantidad maxima de data que puede sacar
        NULL, //parametros
    9, // Prioridadanalizar
       NULL, //handel
     0 //core
    );

    
    xTaskCreatePinnedToCore(leerJetsonTaks,  "leerdatos", 2048,  NULL, 4, NULL, 0);

    //===== CONTROLADORES=====
    //MOTOR
    xTaskCreatePinnedToCore(pidMotorIzqTask,  "pidizq",2048,  NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(pidMotorDerTask,  "pidder",2048,  NULL, 4, NULL, 1);

    //ANGULO
    xTaskCreatePinnedToCore(pidControlDireccionAngularTask,  "pid_ang",2048,  NULL, 6, NULL, 1);

    //Poscicion
    xTaskCreatePinnedToCore(pidPosiciontask, "pidpos", 2048, NULL, 1, NULL, 1);  


    //===== LECTURAS =====
    //ENCODER
    xTaskCreatePinnedToCore(lecturaEncoders,  "encizq",3072,  NULL, 7, NULL, 1);

    //ESTIMACION POSCICION
    xTaskCreatePinnedToCore(estimadorDePoscicionTask, "est_pos", 3072,NULL, 6, NULL , 1);

}




















