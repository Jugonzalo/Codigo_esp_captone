#include <Arduino.h>
#include <Wire.h>
#include <Sensores.h>
#include "Adafruit_ICM20948.h"
//#include "Adafruit_Sensor.h"
#include "../Debug_mode.h"
#include <VL53L1X.h>

//const uint8_t NUM_SENSORES = 3 ;
const uint8_t pines_xshut[3] = {25, 26, 27};
const uint8_t direcciones[3] = {0x30, 0x31, 0x32};

// DEFINO LOS SENSORES DE DISTANCIA 

VL53L1X sensor_adelante;
VL53L1X sensor_izquierda;
VL53L1X sensor_derecha;
Adafruit_ICM20948 icm;

//DatosImu datosImu;





// CREO EL OBJETO QUE 


// ── Parametros de calibracion (noise floor) ──────────────────
float gyroOffsetZ  = 0.0f;   // bias estatico del giroscopio Z [rad/s]
float accelOffsetX = 0.0f;   // bias estatico del acelerometro X [m/s^2]

// ── Buffers de media movil (ultimas 3 muestras) ──────────────
float bufGyroZ[3]  = {0};
float bufAccelX[3] = {0};

// ── Estado del integrador del yaw ────────────────────────────
float          yaw_acumulado   = 0.0f;   // [grados] horario+
unsigned long  tiempo_anterior = 0;       // para calcular dt

constexpr int   GYRO_CALIB_SAMPLES = 500;


constexpr float GYRO_DEADZONE_DPS  = 0.1f; // grados/s



//DEFINICION ORIENTACION POSITIVA IMU

 static float IMU_GYRO_SIGN = -1.0f;

// Normaliza un angulo a (-180, 180] (helper local de este modulo)
static float wrap180_imu(float ang) {
  while (ang >   180.0f) ang -= 360.0f;
  while (ang <= -180.0f) ang += 360.0f;
  return ang;
}






bool esta_sensores_dist = false;

void setup_i2c() {

  // Wire.begin(SDA, SCL) -> el primer argumento es SDA (pin_serial_Data = 21)
  Wire.begin(pin_serial_Data, pin_serial_clk); // SDA=21, SCL=22
  Wire.setClock(clock_sensor);
  delay(100);

{DEBUG_PRINTLN("Inicializando IMU");
  if (!icm.begin_I2C(0x68, &Wire)) {
    DEBUG_PRINTLN("[ERROR] IMU no detectado. Probaré con la direccion alternativa 0x69....");

    if (!icm.begin_I2C(0x69, &Wire)) {
      DEBUG_PRINTLN("[ERROR] IMU no detectado en ninguna direccion.");
      while (1) { delay(40); }
    }
  }

  // Rango del giroscopio: ±250 dps (maxima resolucion)
  icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
  // Rango del acelerometro: ±4 g
  icm.setAccelRange(ICM20948_ACCEL_RANGE_4_G);

  DEBUG_PRINTLN("CALIBRANDO IMU: manten el robot INMOVIL...");


  //
  float acc_gyroZ  = 0.0; // double para mayor precision en la suma
  float acc_accelX = 0.0;

  for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);

    acc_gyroZ  += (float)gyro.gyro.z;          // [rad/s]   // REVUSAR ESTI
    acc_accelX += (float)(accel.acceleration.x / 100); // [cm/s^2]   //Aceleracion lineal

    delay(4); // ~250 Hz durante la calibracion
  }







  // El offset es el promedio de las muestras (noise floor / bias estatico)
  accelOffsetX = (float)(acc_accelX / GYRO_CALIB_SAMPLES);

  // Inicializar buffers de suavizado y la base de tiempo en 0
  for (int i = 0; i < 3; i++) { bufGyroZ[i] = 0.0f; bufAccelX[i] = 0.0f; }
  yaw_acumulado   = 0.0f;




  DEBUG_PRINTLN("Calibracion IMU lista.");
  DEBUG_PRINTF("Offset Gyro Z: %.6f rad/s (%.4f grados/s) | Offset Accel X: %.4f m/s2\n",
               gyroOffsetZ, gyroOffsetZ * 180.0f / PI, accelOffsetX);}
              
{DEBUG_PRINTLN("Inicializando sensores");

     //defino los modos de pines
     

      if (esta_sensores_dist) {
      pinMode(pines_xshut[0], OUTPUT);
      digitalWrite(pines_xshut[0], LOW);

      pinMode(pines_xshut[1], OUTPUT);
      digitalWrite(pines_xshut[1], LOW);

      pinMode(pines_xshut[2], OUTPUT);
      digitalWrite(pines_xshut[2], LOW);

      delay(10);

    // Encender e inicializar los sensores de a uno, asignando direcciones unicas
      digitalWrite(pines_xshut[0], HIGH); // Encender solo este sensor
      delay(10);

      sensor_adelante.setTimeout(500);


      // SI NO LO PILLA DE AHI VEO QUE HACEMOS
      if (!sensor_adelante.init()){
        DEBUG_PRINT("Fallo inicializar sensor adelante");
      };


      sensor_adelante.setAddress(direcciones[0]); // Direccion unica para este sensor

      // Modo de corto alcance y medicion continua cada 50 ms
      sensor_adelante.setDistanceMode(VL53L1X::Short);
      sensor_adelante.setMeasurementTimingBudget(50000);
      sensor_adelante.startContinuous(50);



      // ===================== SENSOR DERECHA========================== 
    // Encender e inicializar los sensores de a uno, asignando direcciones unicas
      digitalWrite(pines_xshut[1], HIGH); // Encender solo este sensor
      delay(10);

      sensor_derecha.setTimeout(500);


      // SI NO LO PILLA DE AHI VEO QUE HACEMOS
      if (!sensor_derecha.init()){

        DEBUG_PRINT("Fallo al inicializar sensor derecha ");

      };    


      sensor_derecha.setAddress(direcciones[1]); // Direccion unica para este sensor

      // Modo de corto alcance y medicion continua cada 50 ms
      sensor_derecha.setDistanceMode(VL53L1X::Short);
      sensor_derecha.setMeasurementTimingBudget(50000);
      sensor_derecha.startContinuous(50);



      // =========================== SENSOR IZQUIERDA =====================

            

    // Encender e inicializar los sensores de a uno, asignando direcciones unicas
      digitalWrite(pines_xshut[2], HIGH); // Encender solo este sensor
      delay(10);

      sensor_izquierda.setTimeout(500);


      // SI NO LO PILLA DE AHI VEO QUE HACEMOS
      if (!sensor_izquierda.init()){
        DEBUG_PRINT("Fallo al inicializar sensor izquierdo");
      };


      sensor_izquierda.setAddress(direcciones[2]); // Direccion unica para este sensor

      // Modo de corto alcance y medicion continua cada 50 ms
      sensor_izquierda.setDistanceMode(VL53L1X::Short);
      sensor_izquierda.setMeasurementTimingBudget(50000);
      sensor_izquierda.startContinuous(50);
    } }
  }





















// ======================== DISTANCIA =========================


// Lectura NO bloqueante. Llamar periodicamente desde una tarea.
void leer_imu(DatosImu &out) {



  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  // ── 1. Calcular dt ───────────────────────────────────────────
  unsigned long ahora = micros();
  float dt = (float)(ahora - tiempo_anterior) * 1e-6f; // segundos
  tiempo_anterior = ahora;

  // Guardia de seguridad: dt invalido -> devolvemos el ultimo estado sin integrar
  if (dt <= 0.0001f || dt > 0.5f) {
    out.vel_angular = 0.0f;
    out.pos_angular   = yaw_acumulado;
    out.aceleracion_lineal   = 0.0f;
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
  float gyroZ_dps = IMU_GYRO_SIGN * gz * 180.0f / PI; // horario+

  // ── 5. Integracion de Euler con zona muerta anti-drift ───────
  if (fabsf(gyroZ_dps) > GYRO_DEADZONE_DPS) {
    yaw_acumulado += gyroZ_dps * dt;
  }
  yaw_acumulado = wrap180_imu(yaw_acumulado);

  // ── 6. Salidas ───────────────────────────────────────────────
  out.vel_angular = gyroZ_dps;
  out.pos_angular   = yaw_acumulado;
  out.aceleracion_lineal   = ax * 100.0f; //  cm/s^2 
}
