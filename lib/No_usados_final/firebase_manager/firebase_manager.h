#ifndef FIREBASE_MANAGER_H
#define FIREBASE_MANAGER_H

#include <Firebase_ESP_Client.h>

// ===== Credenciales WiFi =====
#define WIFI_SSID "Mauricio"
#define WIFI_PASSWORD "12345678a"

// ===== Credenciales Firebase =====
#define API_KEY "AIzaSyCutgJmyEmB8EyjYv6VuqT62HgJogDQ-b4"
#define DATABASE_URL "https://project-yalent-default-rtdb.firebaseio.com"

// ===== Intervalo de sincronización con Firebase =====

#define SEND_INTERVAL_MS 10 // ms

// ===== Objetos Firebase (accesibles desde otros módulos) =====
extern FirebaseData fbdo;
extern FirebaseAuth auth;
extern FirebaseConfig config;

extern float teta_leido_firebase;

// ===== Prototipos =====
void firebaseSetup();
void firebaseLoop();

#endif // FIREBASE_MANAGER_H
