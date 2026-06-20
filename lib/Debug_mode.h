// defino las cosas para el macro debug
#pragma once

#define DEBUG 1

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