#ifndef ESTIMADOR_H
#define ESTIMADOR_H

#include <Arduino.h>

// ============================================================
//  PARAMETROS FISICOS  (editar facilmente aca)
// ============================================================
// Distancia entre las dos ruedas (eje a eje) en cm
#define DISTANCIA_RUEDAS 18.2f

// ============================================================
//  CONVENCION DE ANGULOS
//  teta en (-180, 180], POSITIVO en sentido HORARIO
// ============================================================
// Signo de la odometria del encoder: (v_izq - v_der).
// Para horario+ con la rueda izquierda mas rapida -> +1.0f
// Si tras la prueba fisica gira al reves, cambiar a (-1.0f)
#define ENC_SIGN (+1.0f)

// ============================================================
//  PESOS DEL FILTRO COMPLEMENTARIO  (alpha + beta = 1)

// ============================================================
#define FILTRO_ALPHA_IMU   0.98f   // peso IMU
#define FILTRO_BETA_ENC    0.02f   // peso angular del encoder (nos sirve para el deslizamiento)
#define FILTRO_GAMMA_CALC  0.00f   // peso modelo 

// ============================================================
//  UMBRALES DE DETECCION DE DESLIZAMIENTO  (placeholders x ahora)
// ============================================================
// |omega_encoder - omega_imu| sobre el que se asume patinaje [grados/s]
#define SLIP_OMEGA_THRESH_DPS  15.0f
// |accel_encoder - accel_imu_x| sobre el que se asume patinaje [cm/s^2]
#define SLIP_ACCEL_THRESH       150.0f
// Velocidad minima de rueda para considerar que "hay movimiento" [cm/s]
#define SLIP_VEL_MIN            2.0f

// ============================================================
//  ESTADO DEL ESTIMADOR
// ============================================================
struct EstadoEstimador {
  // Salidas principales
  float x;            // posicion X [cm]
  float y;            // posicion Y [cm]
  float teta;         // heading fusionado [grados, (-180,180], horario+]

  // Sub-estimaciones (debug / telemetria)
  float teta_imu;     // heading segun IMU [grados]
  float teta_enc;     // heading segun encoder (integrado) [grados]
  float omega_enc;    // velocidad angular encoder [grados/s]
  float V;            // velocidad lineal del chasis [cm/s]
  bool  deslizando;   // flag de deslizamiento

  // Internos
  float V_prev;       // V del ciclo anterior (para aceleracion encoder)
  bool  init_listo;   // se completo el primer ciclo
};

// ── API ──────────────────────────────────────────────────────
void  estimador_init(EstadoEstimador &e);

// Punto de entrada para corregir la pose con una referencia absoluta
// (marcadores ArUco / vision desde la Jetson).
void  estimador_set_pose(EstadoEstimador &e, float x, float y, float teta);

// Paso de estimacion (llamar periodicamente). Consolida encoder + IMU.
void  estimador_update(EstadoEstimador &e,
                       float v_izq, float v_der,
                       float teta_imu, float accel_imu_x,
                       float dt);

// Utilidad: normaliza un angulo a (-180, 180]
float wrap180(float ang);

#endif // ESTIMADOR_H
