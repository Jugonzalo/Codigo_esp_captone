#include "../firebase_datos/firebase_datos.h"
#include "../firebase_manager/firebase_manager.h"
#include "../motor/motor.h"
#include "../Debug_mode.h"

// ===== Variables  =====
int v_derecha = 0;
int v_izquierda = 0;

// ===== Funciones =====
bool actualizar_dato(String Ruta) {
  if (Firebase.RTDB.get(&fbdo, Ruta)) {
    if (Ruta == teta_leido) {
      teta_leido_firebase = fbdo.floatData(); // Asignamos a la variable
      DEBUG_PRINTF("teta_leido: %f\n", teta_leido_firebase);
    }

    if (Ruta == fijar_derecha) {
      v_derecha = fbdo.intData();
      DEBUG_PRINTF("motor derecho: %d\n", v_derecha);
    }

    if (Ruta == fijar_izquierda) {
      v_izquierda = fbdo.intData();
      DEBUG_PRINTF("motor izquierdo: %d\n", v_izquierda);
    }

    // ----------------------------------------------------------------------

    // aqui  tengo que ir agragando los datos que quiera ir poniendo

    // ----------------------------------------------------------------------

    return true;

  } else {
    DEBUG_PRINTF("  -> Firebase ERROR al leer '%s': %s\n", Ruta.c_str(),
                fbdo.errorReason().c_str());
    return false;
  }
}

bool fijar_dato(String Ruta, int dato) {
  if (Ruta == fijar_derecha) {
    Firebase.RTDB.setInt(&fbdo, Ruta, dato);
    DEBUG_PRINTF("motor derecho fijado en : %d\n", dato);
  }

  if (Ruta == fijar_izquierda) {
    Firebase.RTDB.setInt(&fbdo, Ruta, dato);
    DEBUG_PRINTF("motor izquierdo fijado en : %d\n", dato);
  }

  // ----------------------------------------------------------------------

  // aqui  tengo que ir agragando los datos que quiera ir poniendo

  // ----------------------------------------------------------------------

  return true;
}

void datos_loop() {
  actualizar_dato(fijar_derecha);
  actualizar_dato(fijar_izquierda);
  cambiar_velocidad_izquierda(v_izquierda);
  cambiar_velocidad_derecha(v_derecha);
}
