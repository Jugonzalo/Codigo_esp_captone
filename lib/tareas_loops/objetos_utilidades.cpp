#include <tareas.h>
#include <string.h>
#include <Arduino.h>
#include <motor.h>




// ---------------------COLAS---------------------


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


// ---------------------PID VELOCIDAD---------------------
float abs_velocidad_actual_izq = 0.0f, v_out_izq = 0.0f, abs_velocidad_ref_izq = 0.0f;
QuickPID pidIzq(
    &abs_velocidad_actual_izq, &v_out_izq, &abs_velocidad_ref_izq,
    Kp_motor_izquierdo, Ki_motor_izquierdo, Kd_motor_izquierdo,
    QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas,
    QuickPID::iAwMode::iAwCondition, QuickPID::Action::direct
);

float abs_velocidad_actual_der = 0.0f, v_out_der = 0.0f, abs_velocidad_ref_der = 0.0f;
QuickPID pidDer(
    &abs_velocidad_actual_der, &v_out_der, &abs_velocidad_ref_der,
    Kp_motor_derecho, Ki_motor_derecho, Kd_motor_derecho,
    QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas,
    QuickPID::iAwMode::iAwCondition, QuickPID::Action::direct
);


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

float wrap180(float ang) {
  while (ang >   180.0f) ang -= 360.0f;
  while (ang <= -180.0f) ang += 360.0f;
  return ang;
}

