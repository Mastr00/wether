#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <DHT.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <WiFiClientSecure.h> // Ajouté pour HTTPS
#include <HTTPClient.h>      // Ajouté pour les requêtes HTTP

// ===================== CONFIGURATION =====================
#define DHTPIN 15
#define DHTTYPE DHT11
#define LDR_PIN 4
#define GAS_PIN 5
#define BUZZER_PIN 9
#define GPS_RX 16
#define GPS_TX 17
#define PPS_PIN 7

// Configuration LEDC pour le buzzer
#define BUZZER_LEDC_CHANNEL 0
#define BUZZER_FREQ 2500 // Fréquence légèrement plus aiguë
#define BUZZER_RESOLUTION 8

// --- Identifiants WiFi du téléphone (Mode Station) ---
const char* STA_SSID = "Mstro"; // REMPLACEZ par le nom exact de votre partage
const char* STA_PASS = "[(1234567890)]<M@str0><M@md0uh>[(1234567890)]"; // REMPLACEZ par le mot de passe

// --- Configuration Pushover ---
#define PUSHOVER_API_TOKEN "aw2rs4e8hbk11kytn8tejteedkchep" // <<< VOTRE API TOKEN EST ICI
#define PUSHOVER_USER_KEY "u7nmss8b31ouvjm7iid4tvek7q9en1" // <<< VOTRE USER KEY EST ICI
const char* pushoverApiUrl = "https://api.pushover.net/1/messages.json";
// Note: setInsecure() est utilisé ci-dessous pour simplifier. Voir les commentaires dans sendPushoverNotification.

// Identifiants pour le login web
const char* LOGIN_USER = "Mastr00";
const char* LOGIN_PASS = "1234";

bool alarmEnabled = true;
bool isAlarmActive = false;
int gasThresholdPct = 60;
bool notificationSent = false; // Pour n'envoyer qu'une seule notification par alarme

// ===================== OBJETS =====================
DHT dht(DHTPIN, DHTTYPE);
AsyncWebServer server(80);
HardwareSerial GPS_Serial(1);
TinyGPSPlus gps;

bool ppsPulse = false;
void IRAM_ATTR ppsISR() { ppsPulse = true; }

// ===================== VARIABLES =====================
float temperature = 0, humidity = 0;
int ldrValue = 0, gasValue = 0;
float gpsLat = 0, gpsLng = 0, gpsSpeed = 0;
char gpsTime[32] = "--:--:--";
char gpsDate[32] = "--/--/----";
int gpsSatellites = 0; // Nouveau: Nombre de satellites
float gpsHDOP = 99.0;   // Nouveau: Précision HDOP (plus bas = mieux)

// Conversion LDR -> Lux (Inversée)
float ldrToLux(int raw) {
    raw = constrain(raw, 0, 4095);
    float mappedLux = ( (float)(4095 - raw) / 4095.0 ) * (50000.0 - 10.0) + 10.0;
    if (mappedLux < 10.0) mappedLux = 10.0;
    if (mappedLux > 50000.0) mappedLux = 50000.0;
    return mappedLux;
}

// Fonctions Buzzer
void startBuzzer() { ledcWrite(BUZZER_LEDC_CHANNEL, 128); }
void stopBuzzer() { ledcWrite(BUZZER_LEDC_CHANNEL, 0); }

// --- Fonction pour envoyer notification Pushover ---
void sendPushoverNotification(String message) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Pushover Error: Not connected to WiFi.");
        return;
    }
     // Vérifie si les clés sont toujours les placeholders (double vérification)
     if (String(PUSHOVER_API_TOKEN) == "YOUR_APP_API_TOKEN" || String(PUSHOVER_API_TOKEN) == "" || String(PUSHOVER_USER_KEY) == "YOUR_USER_KEY" || String(PUSHOVER_USER_KEY) == "") {
        Serial.println("Pushover Error: API Token or User Key not configured in the code.");
        return;
    }

    WiFiClientSecure client;
    HTTPClient http;

    // ATTENTION : Moins sécurisé. Ignore la validation du certificat du serveur.
    client.setInsecure();

    Serial.println("[HTTP] Début de la requête Pushover...");
    // --- AJOUT DEBOGAGE ---
    Serial.printf("  Utilisation Token: %s\n", PUSHOVER_API_TOKEN);
    Serial.printf("  Utilisation User Key: %s\n", PUSHOVER_USER_KEY);
    // --- FIN AJOUT DEBOGAGE ---

    if (http.begin(client, pushoverApiUrl)) {
        http.addHeader("Content-Type", "application/x-www-form-urlencoded");

        String postData = "token=" + String(PUSHOVER_API_TOKEN) +
                          "&user=" + String(PUSHOVER_USER_KEY) +
                          "&message=" + message;

        int httpCode = http.POST(postData);

        if (httpCode > 0) {
            Serial.printf("[HTTP] Code retour Pushover : %d\n", httpCode);
            String payload = http.getString(); // Lire la réponse même en cas d'erreur
            if (httpCode == HTTP_CODE_OK) {
                Serial.println("[HTTP] Réponse Pushover OK : " + payload);
                notificationSent = true;
            } else {
                 Serial.printf("[HTTP] Erreur Pushover (%d): %s\n", httpCode, payload.c_str()); // Afficher la réponse d'erreur
                 Serial.printf("  -> Vérifiez API Token et User Key!\n");
            }
        } else {
            Serial.printf("[HTTP] Échec de la requête Pushover (connexion?): %s\n", http.errorToString(httpCode).c_str());
        }
        http.end();
    } else {
        Serial.printf("[HTTP] Impossible de se connecter à l'API Pushover (URL? DNS?)\n");
    }
}


void setup() {
  Serial.begin(115200);
  Serial.println("\n\n--- Initialisation ---");
  dht.begin();
  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  pinMode(PPS_PIN, INPUT_PULLDOWN);
  attachInterrupt(PPS_PIN, ppsISR, RISING);

  ledcSetup(BUZZER_LEDC_CHANNEL, BUZZER_FREQ, BUZZER_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CHANNEL);
  stopBuzzer();
  Serial.println("Test du buzzer...");
  startBuzzer(); delay(200); stopBuzzer();
  Serial.println("Test du buzzer terminé.");

  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID, STA_PASS);
   Serial.print("Connexion au WiFi '");
  Serial.print(STA_SSID);
  Serial.print("'...");
  int wifi_retries = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_retries < 20) {
    delay(500);
    Serial.print(".");
    wifi_retries++;
  }
   if(WiFi.status() == WL_CONNECTED) {
    Serial.println(" Connecté!");
    Serial.print("Adresse IP de l'ESP32 : ");
    Serial.println(WiFi.localIP());
  } else {
      Serial.println(" Échec de la connexion WiFi.");
  }


  if (!SPIFFS.begin(true)) { Serial.println("ERREUR SPIFFS ! Échec de l'initialisation."); return;} else { Serial.println("SPIFFS Initialisé avec succès.");}

  // --- ROUTES ---
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){ req->redirect("/login"); });
  server.on("/login", HTTP_GET, [](AsyncWebServerRequest *request){
       if (request->hasParam("user") && request->hasParam("pass")) {
            String u = request->getParam("user")->value();
            String p = request->getParam("pass")->value();
            if (u == LOGIN_USER && p == LOGIN_PASS) request->send(200, "text/plain", "OK");
            else request->send(401, "text/plain", "FAIL");
            return;
        }
        request->send(SPIFFS, "/login.html", "text/html");
  });
  server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(SPIFFS, "/index.html", "text/html"); });
  server.on("/settings.html", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(SPIFFS, "/settings.html", "text/html"); });
  server.on("/chart.js", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(SPIFFS, "/chart.js", "text/javascript"); });
  server.on("/leaflet.js", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(SPIFFS, "/leaflet.js", "text/javascript"); });
  server.on("/leaflet.css", HTTP_GET, [](AsyncWebServerRequest *req){ req->send(SPIFFS, "/leaflet.css", "text/css"); });
  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request){
       if (request->hasParam("threshold")) {
            String thresholdStr = request->getParam("threshold")->value();
            gasThresholdPct = thresholdStr.toInt();
            Serial.printf("Nouveau seuil de gaz : %d%%\n", gasThresholdPct);
            request->send(200, "text/plain", "OK");
        } else {
            request->send(400, "text/plain", "Paramètre 'threshold' manquant");
        }
  });
  server.on("/alarm", HTTP_GET, [](AsyncWebServerRequest *request){
     if (request->hasParam("state")) {
          String state = request->getParam("state")->value();
          if (state == "on") {
              alarmEnabled = true;
              notificationSent = false;
              Serial.println("Alarme activée (prête pour notification)");
              request->send(200, "text/plain", "ALARM_ON");
          } else if (state == "off") {
              alarmEnabled = false;
              isAlarmActive = false;
              notificationSent = false;
              stopBuzzer();
              Serial.println("Alarme désactivée et réinitialisée");
              request->send(200, "text/plain", "ALARM_OFF_RESET");
          } else { request->send(400, "text/plain", "Paramètre 'state' invalide"); }
      } else { request->send(400, "text/plain", "Paramètre 'state' manquant"); }
  });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *req){
    int gasPct = map(gasValue, 0, 4095, 0, 100);
    bool wifiConnected = (WiFi.status() == WL_CONNECTED);

    String json = "{";
    json += "\"temp\":" + String(temperature, 1) + ",";
    json += "\"hum\":" + String(humidity, 1) + ",";
    json += "\"lux\":" + String(ldrToLux(ldrValue), 1) + ",";
    json += "\"gas\":" + String(gasValue) + ",";
    json += "\"gasPct\":" + String(gasPct) + ",";
    json += "\"gpsLat\":" + String(gpsLat, 6) + ",";
    json += "\"gpsLng\":" + String(gpsLng, 6) + ",";
    json += "\"gpsSpeed\":" + String(gpsSpeed, 2) + ",";
    json += "\"gpsSatellites\":" + String(gpsSatellites) + ",";
    json += "\"gpsHDOP\":" + String(gpsHDOP, 1) + ",";
    json += "\"pps\":" + String(ppsPulse ? "true" : "false") + ",";
    json += "\"wifiRSSI\":" + String(WiFi.RSSI()) + ",";
    json += "\"wifiConnected\":" + String(wifiConnected ? "true" : "false") + ",";
    json += "\"time\":\"" + String(gpsTime) + "\",";
    json += "\"date\":\"" + String(gpsDate) + "\",";
    json += "\"alarmEnabled\":" + String(alarmEnabled ? "true" : "false") + ","; // Virgule ajoutée ici
    json += "\"isAlarmActive\":" + String(isAlarmActive ? "true" : "false");      // La dernière ligne n'a pas de virgule
    json += "}";
    req->send(200, "application/json", json);
    ppsPulse = false;
  });

  server.begin();
  Serial.println("Serveur web démarré.");
}

// --- loop() reste inchangé ---
void loop() {
  // --- Loop ---
  float newTemp = dht.readTemperature();
  float newHum = dht.readHumidity();
  if (!isnan(newTemp)) { temperature = newTemp; }
  if (!isnan(newHum)) { humidity = newHum; }

  ldrValue = analogRead(LDR_PIN);
  gasValue = analogRead(GAS_PIN);

  while (GPS_Serial.available()) {
    gps.encode(GPS_Serial.read());
  }
  // Mise à jour variables GPS
  if (gps.location.isValid() && gps.location.isUpdated()) { gpsLat = gps.location.lat(); gpsLng = gps.location.lng(); }
  if (gps.speed.isValid() && gps.speed.isUpdated()) { gpsSpeed = gps.speed.kmph(); }
  if (gps.time.isValid() && gps.time.isUpdated()) { sprintf(gpsTime, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second()); }
  if (gps.date.isValid() && gps.date.isUpdated()) { sprintf(gpsDate, "%02d/%02d/%04d", gps.date.day(), gps.date.month(), gps.date.year()); }
  if (gps.satellites.isValid() && gps.satellites.isUpdated()) { gpsSatellites = gps.satellites.value(); } else if (!gps.satellites.isValid()) { gpsSatellites = 0;}
  if (gps.hdop.isValid() && gps.hdop.isUpdated()) { gpsHDOP = gps.hdop.hdop(); } else if (!gps.hdop.isValid()) { gpsHDOP = 99.0;}


  // LOGIQUE D'ALARME
  int currentGasPct = map(gasValue, 0, 4095, 0, 100);
  if (alarmEnabled && currentGasPct > gasThresholdPct && !isAlarmActive) {
    isAlarmActive = true;
    Serial.println("ALARM TRIGGERED!");
    if (!notificationSent) {
      sendPushoverNotification("⚠️ Alerte Gaz! Niveau: " + String(currentGasPct) + "%");
    }
  }

  // Contrôle du buzzer
  if (isAlarmActive && alarmEnabled) {
    startBuzzer();
  } else {
    stopBuzzer();
     if (!alarmEnabled && isAlarmActive) {
         isAlarmActive = false;
         notificationSent = false;
     }
  }

  delay(1000);
}

