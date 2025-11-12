#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <DHT.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===================== CONFIGURATION =====================
#define DHTPIN 15
#define DHTTYPE DHT11
#define LDR_PIN 4
#define GAS_PIN 5
#define BUZZER_PIN 9

// ✅ GPS corrigé → pins 13/14 + baud 9600
#define GPS_RX 13
#define GPS_TX 14
#define GPS_DEFAULT_BAUD_RATE 9600
#define PPS_PIN 7

#define OLED_SDA 8
#define OLED_SCL 10
#define PIR_PIN 6

// KY-037 (capteur sonore)
#define KY037_A0_PIN 11
#define KY037_D0_PIN 12

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

#define BUZZER_LEDC_CHANNEL 0
#define BUZZER_FREQ 2500
#define BUZZER_RESOLUTION 8

// --- Identifiants WiFi ---
const char* STA_SSID = "SFR_6B0F";
const char* STA_PASS = "jaa9ij6d14vps48c7e3b";

#define PUSHOVER_API_TOKEN "aw2rs4e8hbk11kytn8tejteedkchep"
#define PUSHOVER_USER_KEY "u7nmss8b31ouvjm7iid4tvek7q9en1"
const char* pushoverApiUrl = "https://api.pushover.net/1/messages.json";

// Identifiants Login Web
const char* LOGIN_USER = "Mastr00";
const char* LOGIN_PASS = "1234";

bool alarmEnabled = true;
bool isAlarmActive = false;
int gasThresholdPct = 60;
int dbThreshold = 65;
int dbCorrection = -50;
bool notificationSent = false;

// ===================== OBJETS =====================
DHT dht(DHTPIN, DHTTYPE);
AsyncWebServer server(80);
HardwareSerial GPS_Serial(1);
TinyGPSPlus gps;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===================== VARIABLES =====================
float temperature = 0, humidity = 0;
int ldrValue = 0, gasValue = 0;
float gpsLat = 0, gpsLng = 0, gpsSpeed = 0;
char gpsTime[32] = "--:--:--";
char gpsDate[32] = "--/--/----";
int gpsSatellites = 0;
float gpsHDOP = 99.0;

bool ppsPulse = false;
void IRAM_ATTR ppsISR() { ppsPulse = true; }

int pirState = 0;
int soundAnalogValue = 0;
float soundDecibel = 0.0;

// Conversion LDR -> Lux
float ldrToLux(int raw) {
    raw = constrain(raw, 0, 4095);
    float mappedLux = ((float)(4095 - raw) / 4095.0) * (50000.0 - 10.0) + 10.0;
    return constrain(mappedLux, 10.0, 50000.0);
}
// ===================== KY-037 corrigé =====================
// Lecture amplitude crête-à-crête stabilisée
int readSoundSensor() {
    const int sampleWindow = 30; // ms pour échantillonnage
    unsigned long start = millis();

    int signalMax = 0;
    int signalMin = 4095;

    while (millis() - start < sampleWindow) {
        int sample = analogRead(KY037_A0_PIN);

        if (sample > signalMax) signalMax = sample;
        if (sample < signalMin) signalMin = sample;
    }

    int peakToPeak = signalMax - signalMin;

    // Anti-bruit : si variation très faible → silence
    if (peakToPeak < 10) peakToPeak = 0;

    return peakToPeak;
}

// Conversion en pseudo-dB stabilisée
float analogToDecibel(int peak) {
    if (peak <= 1) return 0.0;

    float db = 20.0 * log10((float)peak);
    db += dbCorrection;

    return constrain(db, 0.0, 100.0);
}

// ===================== BUZZER =====================
void startBuzzer() {
    ledcWrite(BUZZER_LEDC_CHANNEL, 128);
}
void stopBuzzer() {
    ledcWrite(BUZZER_LEDC_CHANNEL, 0);
}

// ===================== PUSHOVER =====================
void sendPushoverNotification(String message) {
    if (WiFi.status() != WL_CONNECTED) return;

    WiFiClientSecure client;
    HTTPClient http;
    client.setInsecure();

    if (http.begin(client, pushoverApiUrl)) {
        http.addHeader("Content-Type", "application/x-www-form-urlencoded");

        String postData =
            "token=" + String(PUSHOVER_API_TOKEN) +
            "&user=" + String(PUSHOVER_USER_KEY) +
            "&message=" + message;

        int httpCode = http.POST(postData);

        if (httpCode == 200) {
            notificationSent = true;
        }

        http.end();
    }
}

// ===================== OLED =====================
void updateOLED() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.printf("T:%.1fC | H:%.1f%%", temperature, humidity);

    display.setCursor(0, 10);
    display.printf("Gas:%d%% | Lux:%.0f", map(gasValue, 0, 4095, 0, 100), ldrToLux(ldrValue));

    display.setCursor(0, 20);
    display.printf("Bruit:%.1f dB", soundDecibel);

    display.setCursor(0, 30);
    display.printf("Time:%s", gpsTime);

    display.setCursor(0, 45);
    if (isAlarmActive && alarmEnabled) {
        display.printf("!!! ALERTE !!!");
    } else if (alarmEnabled) {
        display.printf("Alarme: ARMEE");
    } else {
        display.printf("Alarme: OFF");
    }

    display.setCursor(0, 55);
    display.printf("Mvt:%s", pirState ? "OUI" : "NON");

    display.display();
}
// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n--- Initialisation de l'ESP32 ---");

  dht.begin();

  pinMode(PIR_PIN, INPUT);
  pinMode(KY037_D0_PIN, INPUT);
  Serial.println("HARDWARE: Capteurs PIR/KY-037 configurés.");

  // Stabilisation PIR
  Serial.println("ATTENTION: Stabilisation PIR (10s)... NE PAS BOUGER!");
  delay(10000);
  Serial.println("Stabilisation terminée.");

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("ERREUR: Echec OLED !"));
  } else {
    delay(100);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("ESP32-S3 Pret!");
    display.display();
    Serial.println("HARDWARE: OLED initialisé.");
  }

  // ✅ GPS CORRIGÉ
  GPS_Serial.begin(GPS_DEFAULT_BAUD_RATE, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS: UART initialisé @9600 baud (pins 13/14)");

  pinMode(PPS_PIN, INPUT_PULLDOWN);
  attachInterrupt(PPS_PIN, ppsISR, RISING);

  // Buzzer
  ledcSetup(BUZZER_LEDC_CHANNEL, BUZZER_FREQ, BUZZER_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CHANNEL);
  stopBuzzer();

  // SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("ERREUR: SPIFFS !"); 
    return;
  }

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID, STA_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWIFI: Connecté! IP: " + WiFi.localIP().toString());

  // ===================== ROUTES WEB =====================
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){
      req->redirect("/login");
  });

  server.on("/login", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("user") && request->hasParam("pass")) {
            String u = request->getParam("user")->value();
            String p = request->getParam("pass")->value();
            if (u == LOGIN_USER && p == LOGIN_PASS)
                request->send(200, "text/plain", "OK");
            else
                request->send(401, "text/plain", "FAIL");
            return;
        }
        request->send(SPIFFS, "/login.html", "text/html");
  });

  server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest *req){
      req->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/settings.html", HTTP_GET, [](AsyncWebServerRequest *req){
      req->send(SPIFFS, "/settings.html", "text/html");
  });

  server.on("/chart.js", HTTP_GET, [](AsyncWebServerRequest *req){
      req->send(SPIFFS, "/chart.js", "text/javascript");
  });

  server.on("/leaflet.js", HTTP_GET, [](AsyncWebServerRequest *req){
      req->send(SPIFFS, "/leaflet.js", "text/javascript");
  });

  server.on("/leaflet.css", HTTP_GET, [](AsyncWebServerRequest *req){
      req->send(SPIFFS, "/leaflet.css", "text/css");
  });

  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request){
       if (request->hasParam("threshold")) {
            gasThresholdPct = request->getParam("threshold")->value().toInt();
            request->send(200, "text/plain", "OK");
       }
       else if (request->hasParam("dbThreshold")) {
            dbThreshold = request->getParam("dbThreshold")->value().toInt();
            request->send(200, "text/plain", "OK");
       }
       else if (request->hasParam("dbCorrection")) {
            dbCorrection = request->getParam("dbCorrection")->value().toInt();
            request->send(200, "text/plain", "OK");
       }
       else {
            request->send(400, "text/plain", "Paramètre manquant");
       }
  });

  server.on("/alarm", HTTP_GET, [](AsyncWebServerRequest *request){
      if (!request->hasParam("state")) {
          request->send(400, "text/plain", "Paramètre manquant");
          return;
      }
      String state = request->getParam("state")->value();
      if (state == "on") {
          alarmEnabled = true;
          notificationSent = false;
          request->send(200, "text/plain", "ALARM_ON");
      }
      else if (state == "off") {
          alarmEnabled = false;
          isAlarmActive = false;
          notificationSent = false;
          stopBuzzer();
          request->send(200, "text/plain", "ALARM_OFF_RESET");
      }
      else {
          request->send(400, "text/plain", "Etat invalide");
      }
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
    json += "\"pirState\":" + String(pirState ? "true" : "false") + ",";
    json += "\"soundDecibel\":" + String(soundDecibel, 1) + ",";
    json += "\"dbThreshold\":" + String(dbThreshold) + ",";
    json += "\"dbCorrection\":" + String(dbCorrection) + ",";
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
    json += "\"alarmEnabled\":" + String(alarmEnabled ? "true" : "false") + ",";
    json += "\"isAlarmActive\":" + String(isAlarmActive ? "true" : "false");
    json += "}";

    req->send(200, "application/json", json);
    ppsPulse = false;
  });

  server.begin();
  Serial.println("SERVEUR: Serveur web démarré.");
}
// ===================== LOOP =====================
void loop() {
  // --- Lecture des capteurs ---
  float newTemp = dht.readTemperature();
  float newHum = dht.readHumidity();
  if (!isnan(newTemp)) temperature = newTemp;
  if (!isnan(newHum)) humidity = newHum;

  ldrValue = analogRead(LDR_PIN);
  gasValue = analogRead(GAS_PIN);

  // PIR
  pirState = digitalRead(PIR_PIN);

  // Son KY-037 corrigé
  int peakToPeak = readSoundSensor();
  soundDecibel = analogToDecibel(peakToPeak);

  // ===================== GPS (corrigé) =====================
  // Lecture brute GPS
  while (GPS_Serial.available()) gps.encode(GPS_Serial.read());

  // Position
  if (gps.location.isUpdated()) {
      gpsLat = gps.location.lat();
      gpsLng = gps.location.lng();
  }

  // Vitesse
  gpsSpeed = gps.speed.isValid() ? gps.speed.kmph() : 0.0;

  // Heure
  if (gps.time.isUpdated()) {
      sprintf(gpsTime, "%02d:%02d:%02d",
              gps.time.hour(), gps.time.minute(), gps.time.second());
  }

  // Date
  if (gps.date.isUpdated()) {
      sprintf(gpsDate, "%02d/%02d/%04d",
              gps.date.day(), gps.date.month(), gps.date.year());
  }

  // Satellites / HDOP
  gpsSatellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
  gpsHDOP       = gps.hdop.isValid()       ? gps.hdop.hdop()       : 99.0;

  // ===================== LOGIQUE D'ALARME =====================
  int currentGasPct = map(gasValue, 0, 4095, 0, 100);
  bool gasAlarm = currentGasPct > gasThresholdPct;
  bool dbAlarm  = soundDecibel > dbThreshold;

  if (alarmEnabled && (gasAlarm || dbAlarm) && !isAlarmActive) {
      isAlarmActive = true;

      if (!notificationSent) {
          String msg =
              "Alerte! Gaz (" + String(currentGasPct) +
              "%) OU Bruit (" + String(soundDecibel, 1) + " dB)!";
          sendPushoverNotification(msg);
      }
  }

  // Buzzer
  if (isAlarmActive && alarmEnabled) {
      startBuzzer();
  } else {
      stopBuzzer();
      if (!alarmEnabled && isAlarmActive) {
          isAlarmActive = false;
          notificationSent = false;
      }
  }

  // ===================== OLED =====================
  updateOLED();

  delay(100);
}
