#include <Arduino.h>
#include "conexion_jetson.h"

int velocidad_izquierda_global = 0;
int velocidad_derecha_global = 0;


struct __attribute__((packed)) Envio {
  uint8_t header = 0xAA; // Byte de inicio para sincronizar
  int32_t velocidad_izquierda;      // un por ahora dejemoslo como
  int32_t velocidad_derecha;        // un byte

  uint8_t checksum;      // Para validar integridad
};
struct __attribute__((packed)) Lectura {
  uint8_t header = 0xAA; // Byte de inicio para sincronizar
  int32_t velocidad_izquierda;      // un por ahora dejemoslo como
  int32_t velocidad_derecha;        // un byte

  uint8_t checksum;      // Para validar integridad
};


void setup_jetson() {
  Serial.begin(velocidad_serial);
  delay(2000); // Espera para asegurar que el puerto serial esté listo
}



void enviar_datos_jetson() {
  Envio data;
  data.velocidad_izquierda = velocidad_izquierda_global;
  data.velocidad_derecha = velocidad_derecha_global ;

  // Cálculo simple de checksum (XOR de los datos)
  data.checksum = (uint8_t)(data.velocidad_izquierda ^ data.velocidad_derecha);

  // Enviamos el bloque de memoria completo (binario puro)
  Serial.write((uint8_t*)&data, sizeof(data));

}

void leer_datos_jetson() {
  // Verificamos si hay al menos el tamaño de la estructura en el buffer
  if (Serial.available() >= sizeof(Lectura)) {
    Lectura data_leida;
    
    // Leemos los bytes y los "volcamos" en la dirección de memoria de 'data_leida'
    Serial.readBytes((uint8_t*)&data_leida, sizeof(data_leida));

    // Validamos el header para no procesar basura
    if (data_leida.header == 0xAA) {
      // Aquí aplicas el data_leida.velocidad a tu motor
      velocidad_izquierda_global = data_leida.velocidad_izquierda;
      velocidad_derecha_global = data_leida.velocidad_derecha;

      Serial.println("Datos recibidos de Jetson:");
      Serial.print("Velocidad Derecha: ");
      Serial.println(velocidad_derecha_global);
      Serial.print("Velocidad Izquierda: ");
      Serial.println(velocidad_izquierda_global);
    } else {
      // Si el header está mal, el buffer está desincronizado
      // Limpiamos un byte para intentar buscar el header en el siguiente loop
      Serial.read(); 
    }
  }
}






