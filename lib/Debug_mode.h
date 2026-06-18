// defino las cosas para el macro debug
#pragma once

// ===== Banco de pruebas =====
// DEBUG 1 + TEST_ESTIMADOR 1: log de texto por serial y SIN envio binario a la
// Jetson (no se pueden mezclar). RUTINA_CUADRADO 1: ejecuta el cuadrado al
// encender. Para produccion con la Jetson: DEBUG 0, TEST_ESTIMADOR 0.
#define DEBUG 0
#define TEST_ESTIMADOR 0
#define RUTINA_CUADRADO 0

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