#include "estimador.h"

// Normaliza un angulo a (-180, 180]
float wrap180(float ang) {
  while (ang >   180.0f) ang -= 360.0f;
  while (ang <= -180.0f) ang += 360.0f;
  return ang;
}

void estimador_init(EstadoEstimador &e) {
  e.x = 0.0f; e.y = 0.0f; e.teta = 0.0f;
  e.teta_imu = 0.0f; e.teta_enc = 0.0f;
  e.omega_enc = 0.0f; e.V = 0.0f;
  e.deslizando = false;
  e.V_prev = 0.0f;
  e.init_listo = false;
}

// Corrige la pose con una referencia absoluta (ArUco / Jetson).
// Re-alinea las sub-estimaciones de heading para evitar saltos en el filtro.
void estimador_set_pose(EstadoEstimador &e, float x, float y, float teta) {
  e.x = x;
  e.y = y;
  e.teta     = wrap180(teta);
  e.teta_enc = e.teta;   // re-alinea la integral del encoder
  e.teta_imu = e.teta;   // re-alinea la referencia IMU
}

// Deslizamiento: las ruedas se mueven (encoder) pero la IMU no confirma el
// movimiento coherente (ni en giro ni en aceleracion lineal de avance).
static bool detectar_deslizamiento(float v_izq, float v_der,
                                   float omega_enc_dps, float omega_imu_dps,
                                   float accel_enc, float accel_imu_x) {
  bool hay_mov_ruedas = (fabsf(v_izq) > SLIP_VEL_MIN) ||
                        (fabsf(v_der) > SLIP_VEL_MIN);
  if (!hay_mov_ruedas) return false;

  bool incoherencia_giro  = fabsf(omega_enc_dps - omega_imu_dps) > SLIP_OMEGA_THRESH_DPS;
  bool incoherencia_accel = fabsf(accel_enc - accel_imu_x)       > SLIP_ACCEL_THRESH;

  return incoherencia_giro || incoherencia_accel;
}

void estimador_update(EstadoEstimador &e,
                      float v_izq, float v_der,
                      float teta_imu, float accel_imu_x,
                      float dt) {
  if (dt <= 0.0f) return;

  // 1. Cinematica diferencial (informe seccion 4)
  e.V = (v_izq + v_der) * 0.5f;                                       // m/s
  float omega_rad = ENC_SIGN * (v_izq - v_der) / DISTANCIA_RUEDAS;    // rad/s
  e.omega_enc = omega_rad * 180.0f / PI;                             // grados/s

  // 2. Heading IMU (ya viene integrado y con signo aplicado en leer_imu)
  e.teta_imu = wrap180(teta_imu);

  // omega de la IMU (derivada del yaw) para el chequeo de deslizamiento
  static float teta_imu_prev = 0.0f;
  float omega_imu_dps = wrap180(e.teta_imu - teta_imu_prev) / dt;
  teta_imu_prev = e.teta_imu;

  // 3. Heading encoder: integramos omega y envolvemos
  e.teta_enc = wrap180(e.teta_enc + e.omega_enc * dt);

  // 4. Aceleracion lineal segun encoder (para cruce con la IMU)
  float accel_enc = (e.V - e.V_prev) / dt;
  e.V_prev = e.V;

  // 5. Deteccion de deslizamiento
  e.deslizando = detectar_deslizamiento(v_izq, v_der,
                                        e.omega_enc, omega_imu_dps,
                                        accel_enc, accel_imu_x);

  // 6. Filtro complementario del heading (forma robusta al wrap).
  //    Durante deslizamiento confiamos 100% en la IMU (beta = 0).
  float diff = wrap180(e.teta_enc - e.teta_imu);
  float beta = e.deslizando ? 0.0f : FILTRO_BETA_ENC;
  e.teta = wrap180(e.teta_imu + beta * diff);

  // 7. Odometria (x, y). Si hay deslizamiento NO acumulamos distancia:
  //    las ruedas patinan y el encoder miente sobre el avance real.
  if (!e.deslizando) {
    float teta_rad = e.teta * PI / 180.0f;
    e.x += e.V * cosf(teta_rad) * dt;
    e.y += e.V * sinf(teta_rad) * dt;
  }

  e.init_listo = true;
}
