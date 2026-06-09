// defino las cosas para el macro debug
#pragma once

#define DEBUG 1

// ===== MODO BANCO DE PRUEBAS DEL ESTIMADOR =====
// En 1: estimadorPosicionTask imprime teta/x/y/deslizando en TEXTO por el
// monitor serial y se DESACTIVA el envio binario a la Jetson (no se pueden
// mezclar binario y texto en el mismo puerto). En 0: operacion normal.
#define TEST_ESTIMADOR 1

// Referencia de angulo FIJA para probar el control de angulo en banco [grados].
// Descomentar para que el robot intente apuntar a ese angulo (ruedas al aire).
// Dejar comentado para probar solo los sensores sin mover motores.
// OJO: no usar junto con RUTINA_CUADRADO (ambos escriben la referencia de angulo).
//#define TEST_TETA_REF_FIJO 90.0f

// ===== RUTINA DE PRUEBA: CUADRADO =====
// En 1: al encender (tras calibrar la IMU) el robot ejecuta automaticamente
// la rutina cuadrada (avanza 50 cm, gira 90 a la derecha, x4) y se detiene.
// En 0: no se crea la tarea y el robot opera normal.
#define RUTINA_CUADRADO 1

#if DEBUG
// Para strings simples o valores: DEBUG_PRINTLN("hola") / DEBUG_PRINTLN(var)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
// Para texto sin salto de linea: DEBUG_PRINT("Conectando")
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
// Para formato estilo printf: DEBUG_PRINTF("valor: %d\n", x)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINTLN(...)
#define DEBUG_PRINT(...)
#define DEBUG_PRINTF(...)
#endif