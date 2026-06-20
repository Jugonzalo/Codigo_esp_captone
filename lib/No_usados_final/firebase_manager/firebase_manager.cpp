#include "../firebase_manager/firebase_manager.h"
#include "../firebase_datos/firebase_datos.h"
#include "../Debug_mode.h"
#include <Arduino.h>
#include <WiFi.h>
#include <addons/RTDBHelper.h>
#include <addons/TokenHelper.h>

#include "../motor/motor.h"

// ===== Definición de objetos Firebase =====
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// ===== Variables internas =====
static unsigned long lastSendTime = 0;
static bool firebaseReady = false;

float teta_leido_firebase;

// ---------------------------------------------------------------------------
// Conecta al WiFi y configura Firebase (llamar desde setup())
// ---------------------------------------------------------------------------
void firebaseSetup() {
  // ----- WiFi -----
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  DEBUG_PRINT("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    DEBUG_PRINT(".");
    delay(300);
  }
  DEBUG_PRINTLN();
  DEBUG_PRINT("Conectado! IP: ");
  DEBUG_PRINTLN(WiFi.localIP());

  // ----- Firebase -----
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  // Autenticación anónima
  if (Firebase.signUp(&config, &auth, "", "")) {
    DEBUG_PRINTLN("Firebase sign-up OK (anónimo)");
    firebaseReady = true;
  } else {
    DEBUG_PRINTF("Error sign-up: %s\n",
                config.signer.signupError.message.c_str());
  }

  config.token_status_callback = tokenStatusCallback;

  Firebase.begin(&config, &auth);
  Firebase.reconnectNetwork(true);
}

// ---------------------------------------------------------------------------
// Sincroniza LEDs y movimiento del robot con Firebase (llamar desde loop())
// ---------------------------------------------------------------------------

void firebaseLoop() {
  unsigned long now = millis();

  if (!Firebase.ready() || (now - lastSendTime < SEND_INTERVAL_MS)) {
    return;
  }
  lastSendTime = now;

  datos_loop();
}
