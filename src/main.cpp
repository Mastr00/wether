#include <Arduino.h>
#include <ArduinoOTA.h>
#include <DHT.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <HardwareSerial.h>
#include <NTPClient.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <SPIFFS.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <ld2410.h>
#include <math.h>
#include <time.h>

// ===================== PINS =====================
#define DHTPIN          15
#define DHTTYPE         DHT11
#define LDR_PIN         4
#define GAS_PIN         5
#define UV_PIN          6    // GUVA-S12SD (sortie analogique)
#define BUZZER_PIN      9
#define KY037_A0_PIN    10   // Brancher A0 du KY-037 ici (pas 11, evite conflit ADC2/WiFi)
#define LD2410_RX       13
#define LD2410_TX       14
#define TOUCH_PIN       7    // Bouton tactile TTP223 (I/O)

// ===================== BUZZER =====================
#define BUZZER_LEDC_CHANNEL 0
#define BUZZER_FREQ         2500
#define BUZZER_RESOLUTION   8

// ===================== NTP / FUSEAU =====================
#define NTP_SERVER  "pool.ntp.org"
#define NTP_OFFSET  7200  // (legacy, conserve pour compatibilite)
// Fuseau POSIX avec gestion automatique heure d'ete/hiver pour la France
#define POSIX_TZ    "CET-1CEST,M3.5.0,M10.5.0/3"

// ===================== SECRETS =====================
// Les identifiants sensibles (WiFi, OTA, web, MQTT) sont definis dans
// include/secrets.h qui est gitignore. Copier secrets.h.example -> secrets.h
// puis remplir avec vos valeurs reelles.
#include "secrets.h"

// ===================== HOSTNAME / OTA =====================
#define DEVICE_HOSTNAME  "station-meteo"   // accessible via http://station-meteo.local
#define OTA_PASSWORD     SECRET_OTA_PASSWORD

// ===================== AUTH WEB =====================
#define WEB_USER         SECRET_WEB_USER
#define WEB_PASS         SECRET_WEB_PASS

// ===================== TIMING =====================
#define GAS_WARMUP_SEC       60      // ignorer alarme gaz les 60s apres boot (chauffe MQ)
#define WIFI_DEAD_TIMEOUT_MS 300000  // 5 min sans WiFi → reboot
#define ALARM_CONFIRM_COUNT  3       // 3 lectures successives au-dessus du seuil pour declencher
#define DATA_CACHE_MS        1000    // cache JSON /data 1s

// ===================== WIFI =====================
const char* STA_SSID = SECRET_STA_SSID;
const char* STA_PASS = SECRET_STA_PASS;

// ===================== MQTT =====================
#define MQTT_BROKER     "192.168.1.50"  // Adresse IP de votre broker Mosquitto
#define MQTT_PORT       1883
#define MQTT_CLIENT_ID  "esp32_station"
// Si le broker est en mode authentifie, definir SECRET_MQTT_USER / SECRET_MQTT_PASS
// dans include/secrets.h ; le bloc ci-dessous prend alors le relais automatiquement.
#ifdef SECRET_MQTT_USER
  #define MQTT_USER    SECRET_MQTT_USER
  #define MQTT_PASS    SECRET_MQTT_PASS
#endif

#define TOPIC_BASE      "esp32/station"
#define TOPIC_STATUS    TOPIC_BASE "/status"
#define TOPIC_ALARM_CMD TOPIC_BASE "/alarm/set"

// ===================== WATCHDOG =====================
#define WDT_TIMEOUT_S   30   // redemarre si la loop est bloquee plus de 30s
                              // (laisse le temps aux reconnexions MQTT/WiFi de timeout)

// ===================== HISTORIQUE GRAPHIQUES =====================
#define HIST_SIZE       60   // 60 echantillons * 10s = 10 minutes de trend

// ===================== OBJETS =====================
DHT              dht(DHTPIN, DHTTYPE);
AsyncWebServer   server(80);
TFT_eSPI         tft;
WiFiClient       wifiClient;
PubSubClient     mqtt(wifiClient);
HardwareSerial   LD2410_Serial(2);
ld2410           radar;
WiFiUDP          ntpUDP;
NTPClient        timeClient(ntpUDP, NTP_SERVER, NTP_OFFSET, 60000);
Preferences      prefs;

// ===================== VARIABLES CAPTEURS =====================
float temperature  = 0;
float humidity     = 0;
int   ldrValue     = 0;
int   gasValue     = 0;
float soundDecibel = 0.0;
float uvIndex      = 0.0;
float heatIdx      = 0.0;   // Indice de chaleur (ressenti) calcule

bool     presenceDetected    = false;
bool     movingDetected      = false;
bool     stationaryDetected  = false;
uint16_t movingDistance      = 0;
uint16_t stationaryDistance  = 0;

// ===================== MOYENNES GLISSANTES =====================
// Index separes pour eviter de polluer la moyenne (bug fixe : gaz/son ecrits 20x avant)
#define AVG_SIZE 5
float   tempBuf[AVG_SIZE]   = {0};
float   humBuf[AVG_SIZE]    = {0};
int     gasBuf[AVG_SIZE]    = {0};
float   soundBuf[AVG_SIZE]  = {0};
uint8_t dhtAvgIdx           = 0;   // pour temp + humidite (2s)
uint8_t fastAvgIdx          = 0;   // pour gaz + son (toutes les 500ms)
bool    dhtAvgFilled        = false;
bool    fastAvgFilled       = false;
unsigned long lastFastSample = 0;

// ===================== HISTORIQUE TFT (page graphiques) =====================
float tempHist[HIST_SIZE] = {0};
float humHist[HIST_SIZE]  = {0};
int   luxHist[HIST_SIZE]  = {0};
uint8_t histIdx           = 0;
bool    histFilled        = false;
unsigned long lastHistSample = 0;

// ===================== VARIABLES ALARME =====================
bool alarmEnabled    = true;
bool isAlarmActive   = false;
int  gasThresholdPct = 60;
int  dbThreshold     = 65;
int  dbCorrection    = 20;  // Calibration: ajuster via /settings (ancien -50 trop bas → toujours 0dB)

// ===================== VARIABLES SYSTEME =====================
char currentTime[32] = "--:--:--";
char currentDate[32] = "--/--/----";

bool          mqttConnected   = false;
unsigned long lastMqttPublish = 0;
unsigned long lastTftUpdate   = 0;
uint8_t       tftPage         = 0;     // 0 = Capteurs, 1 = Systeme, 2 = Graphiques
#define       TFT_PAGE_COUNT  3
bool          tftNeedsRedraw  = true;
bool          lastTouchState  = false;
unsigned long lastTouchTime   = 0;
uint8_t       animFrame       = 0;
unsigned long lastAnimUpdate  = 0;
bool          alarmFlashState = false;  // toggle pour le clignotement
unsigned long lastAlarmFlash  = 0;

// --- Veille ecran ---
#define TFT_SLEEP_TIMEOUT_MS  60000   // 60s d'inactivite avant mise en veille
unsigned long lastActivity     = 0;
bool          tftSleeping      = false;
bool          prevMovingDetect = false;

// --- Detection WiFi mort ---
unsigned long wifiLostSince    = 0;     // 0 = WiFi OK
uint32_t      bootEpoch        = 0;     // epoch UTC au demarrage
unsigned long bootMillis       = 0;

// --- Anti-rebond alarme ---
uint8_t alarmConfirmCount      = 0;

// --- Cache JSON /data ---
String        dataJsonCache;
unsigned long dataJsonCacheTime = 0;

// --- Pulse animation (highlight valeur changee) ---
float prevTemp        = -999, prevHum = -999;
float prevHeatIdx     = -999;
unsigned long pulseTempUntil = 0, pulseHumUntil = 0;

// --- Sirene non-bloquante ---
bool          sirenHigh        = false;
unsigned long lastSirenToggle  = 0;

// ===================== HELPERS CAPTEURS =====================
float ldrToLux(int raw) {
  raw = constrain(raw, 0, 4095);
  return constrain(((float)(4095 - raw) / 4095.0f) * 49990.0f + 10.0f, 10.0f, 50000.0f);
}

float readUvIndex() {
  int raw = analogRead(UV_PIN);
  float voltage = raw * (3.3f / 4095.0f);
  return constrain(voltage / 0.1f, 0.0f, 15.0f);
}

int readSoundSensor() {
  unsigned long start = millis();
  int signalMax = 0, signalMin = 4095;
  while (millis() - start < 50) {   // 50ms au lieu de 30ms → capture mieux les basses frequences
    int s = analogRead(KY037_A0_PIN);
    if (s > signalMax) signalMax = s;
    if (s < signalMin) signalMin = s;
  }
  int p2p = signalMax - signalMin;
  return (p2p < 3) ? 0 : p2p;   // Seuil abaisse de 10 a 3 pour capter les sons faibles
}

float analogToDecibel(int peak) {
  if (peak <= 1) return 0.0f;
  return constrain(20.0f * log10f((float)peak) + dbCorrection, 0.0f, 100.0f);
}

// ===================== HEAT INDEX (ressenti) =====================
// Formule simplifiee Rothfusz, T en °C, RH en %
float computeHeatIndex(float T, float RH) {
  if (T < 26.0f) return T;  // formule pertinente seulement au-dessus de ~27°C
  float Tf = T * 9.0f / 5.0f + 32.0f;
  float HI = -42.379f + 2.04901523f * Tf + 10.14333127f * RH
           - 0.22475541f * Tf * RH - 0.00683783f * Tf * Tf
           - 0.05481717f * RH * RH + 0.00122874f * Tf * Tf * RH
           + 0.00085282f * Tf * RH * RH - 0.00000199f * Tf * Tf * RH * RH;
  return (HI - 32.0f) * 5.0f / 9.0f;
}

// ===================== MOYENNES GLISSANTES =====================
template<typename T> T avgOf(const T* buf, uint8_t size, bool filled, uint8_t idx) {
  uint8_t n = filled ? size : idx;
  if (n == 0) return 0;
  float s = 0;
  for (uint8_t i = 0; i < n; i++) s += buf[i];
  return (T)(s / n);
}

// ===================== PREFERENCES (NVS) =====================
void loadSettings() {
  prefs.begin("station", true);  // read-only
  gasThresholdPct = prefs.getInt("gasTh",  60);
  dbThreshold     = prefs.getInt("dbTh",   65);
  dbCorrection    = prefs.getInt("dbCorr", 20);
  alarmEnabled    = prefs.getBool("alarmOn", true);
  prefs.end();
  Serial.printf("Prefs chargees : gas=%d, dB=%d, corr=%d, alarm=%d\n",
                gasThresholdPct, dbThreshold, dbCorrection, alarmEnabled);
}

void saveSettings() {
  prefs.begin("station", false);
  prefs.putInt("gasTh",  gasThresholdPct);
  prefs.putInt("dbTh",   dbThreshold);
  prefs.putInt("dbCorr", dbCorrection);
  prefs.putBool("alarmOn", alarmEnabled);
  prefs.end();
}

// ===================== WIFI BARS =====================
// rssi typique : -30 (excellent) -> -90 (mauvais)
uint8_t rssiToBars(int rssi) {
  if (rssi >= -55) return 4;
  if (rssi >= -65) return 3;
  if (rssi >= -75) return 2;
  if (rssi >= -85) return 1;
  return 0;
}

void drawWiFiBars(int x, int y, int rssi) {
  uint8_t bars = rssiToBars(rssi);
  uint16_t color = (bars >= 3) ? TFT_GREEN : (bars >= 2) ? TFT_YELLOW
                                            : (bars >= 1) ? TFT_ORANGE : TFT_RED;
  // 4 barres de hauteurs croissantes
  for (uint8_t i = 0; i < 4; i++) {
    int h = 4 + i * 4;            // 4, 8, 12, 16 px
    int bx = x + i * 7;
    int by = y + 18 - h;
    if (i < bars) {
      tft.fillRect(bx, by, 5, h, color);
    } else {
      tft.drawRect(bx, by, 5, h, TFT_DARKGREY);
    }
  }
}

// ===================== RESET REASON / CRASH LOG =====================
const char* resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT_RESET";
    case ESP_RST_SW:        return "SOFTWARE";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "OTHER_WDT";
    case ESP_RST_DEEPSLEEP: return "DEEP_SLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

void logResetReason() {
  esp_reset_reason_t r = esp_reset_reason();
  Serial.printf("Reset reason: %s\n", resetReasonStr(r));
  if (r == ESP_RST_PANIC || r == ESP_RST_TASK_WDT || r == ESP_RST_INT_WDT
   || r == ESP_RST_BROWNOUT) {
    // Log uniquement les reboots anormaux dans /crashlog.txt
    if (SPIFFS.begin(true)) {
      File f = SPIFFS.open("/crashlog.txt", FILE_APPEND);
      if (f) {
        // Au boot l'heure NTP n'est pas encore synchronisee → on note juste boot+raison
        f.printf("[boot+%lums] %s\n", millis(), resetReasonStr(r));
        f.close();
        Serial.println("Crash logge dans /crashlog.txt");
      }
    }
  }
}

// ===================== INDICATEUR DE PAGES (3 dots en haut) =====================
void drawPageIndicator(uint8_t activePage) {
  // 3 dots a (140, 4), (160, 4), (180, 4) — taille 5px, espacement 14px
  for (uint8_t i = 0; i < TFT_PAGE_COUNT; i++) {
    int cx = 144 + i * 16;
    if (i == activePage) {
      tft.fillCircle(cx, 4, 4, TFT_CYAN);   // dot actif
    } else {
      tft.drawCircle(cx, 4, 3, TFT_DARKGREY); // dot inactif
    }
  }
}

// ===================== SIRENE NON-BLOQUANTE =====================
// Alterne entre 2 frequences pour simuler une sirene
void sirenUpdate(unsigned long now) {
  if (!isAlarmActive || !alarmEnabled) return;
  if (now - lastSirenToggle > 250) {
    lastSirenToggle = now;
    sirenHigh = !sirenHigh;
    ledcWriteTone(BUZZER_LEDC_CHANNEL, sirenHigh ? 3000 : 1500);
  }
}

// ===================== TFT SLEEP / WAKE =====================
// Commandes ST7789 : 0x28 = display off, 0x29 = display on
//                    0x10 = sleep in,    0x11 = sleep out
void tftSleep() {
  if (tftSleeping) return;
  tft.writecommand(0x28);     // Display OFF
  tft.writecommand(0x10);     // Sleep IN
  tftSleeping = true;
  Serial.println("TFT: veille");
}

void tftWake() {
  if (!tftSleeping) return;
  tft.writecommand(0x11);     // Sleep OUT
  delay(120);                 // Datasheet ST7789 : attendre 120ms apres sleep out
  tft.writecommand(0x29);     // Display ON
  tftSleeping = false;
  tftNeedsRedraw = true;      // Force le redessin de la page actuelle
  Serial.println("TFT: reveil");
}

// ===================== BUZZER =====================
void startBuzzer() { ledcWrite(BUZZER_LEDC_CHANNEL, 128); }
void stopBuzzer()  { ledcWrite(BUZZER_LEDC_CHANNEL, 0);   }

// ===================== MQTT =====================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  if (String(topic) == TOPIC_ALARM_CMD) {
    if (msg == "ON") {
      alarmEnabled = true;
      saveSettings();
    } else if (msg == "OFF") {
      alarmEnabled   = false;
      isAlarmActive  = false;
      stopBuzzer();
      saveSettings();
    }
  }
}

void publishDiscovery() {
  const String dev =
    ",\"dev\":{\"ids\":[\"esp32_station\"],\"name\":\"Station Meteo\","
    "\"model\":\"ESP32-S3\",\"mf\":\"DIY\"},"
    "\"avty_t\":\"" TOPIC_STATUS "\","
    "\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\"";

  struct { const char* topic; String payload; } sensors[] = {
    { "homeassistant/sensor/esp32_station_temp/config",
      "{\"name\":\"Temperature\",\"stat_t\":\"" TOPIC_BASE "/temperature\","
      "\"unit_of_meas\":\"\\u00b0C\",\"dev_cla\":\"temperature\","
      "\"uniq_id\":\"esp32_station_temp\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_hum/config",
      "{\"name\":\"Humidite\",\"stat_t\":\"" TOPIC_BASE "/humidity\","
      "\"unit_of_meas\":\"%\",\"dev_cla\":\"humidity\","
      "\"uniq_id\":\"esp32_station_hum\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_gas/config",
      "{\"name\":\"Gaz Fumee\",\"stat_t\":\"" TOPIC_BASE "/gas\","
      "\"unit_of_meas\":\"%\",\"icon\":\"mdi:fire\","
      "\"uniq_id\":\"esp32_station_gas\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_lux/config",
      "{\"name\":\"Luminosite\",\"stat_t\":\"" TOPIC_BASE "/lux\","
      "\"unit_of_meas\":\"lx\",\"dev_cla\":\"illuminance\","
      "\"uniq_id\":\"esp32_station_lux\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_uv/config",
      "{\"name\":\"Indice UV\",\"stat_t\":\"" TOPIC_BASE "/uv\","
      "\"unit_of_meas\":\"idx\",\"icon\":\"mdi:sun-wireless\","
      "\"uniq_id\":\"esp32_station_uv\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_sound/config",
      "{\"name\":\"Son\",\"stat_t\":\"" TOPIC_BASE "/sound\","
      "\"unit_of_meas\":\"dB\",\"icon\":\"mdi:volume-high\","
      "\"uniq_id\":\"esp32_station_sound\"" + dev + "}" },

    { "homeassistant/binary_sensor/esp32_station_presence/config",
      "{\"name\":\"Presence\",\"stat_t\":\"" TOPIC_BASE "/presence\","
      "\"dev_cla\":\"presence\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\","
      "\"uniq_id\":\"esp32_station_presence\"" + dev + "}" },

    { "homeassistant/binary_sensor/esp32_station_alarm/config",
      "{\"name\":\"Alarme\",\"stat_t\":\"" TOPIC_BASE "/alarm\","
      "\"dev_cla\":\"safety\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\","
      "\"uniq_id\":\"esp32_station_alarm\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_heat/config",
      "{\"name\":\"Ressenti\",\"stat_t\":\"" TOPIC_BASE "/heat_index\","
      "\"unit_of_meas\":\"\\u00b0C\",\"dev_cla\":\"temperature\","
      "\"uniq_id\":\"esp32_station_heat\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_movdist/config",
      "{\"name\":\"Distance Mouvement\",\"stat_t\":\"" TOPIC_BASE "/moving_dist\","
      "\"unit_of_meas\":\"cm\",\"icon\":\"mdi:run\","
      "\"uniq_id\":\"esp32_station_movdist\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_statdist/config",
      "{\"name\":\"Distance Statique\",\"stat_t\":\"" TOPIC_BASE "/static_dist\","
      "\"unit_of_meas\":\"cm\",\"icon\":\"mdi:human-male\","
      "\"uniq_id\":\"esp32_station_statdist\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_rssi/config",
      "{\"name\":\"WiFi RSSI\",\"stat_t\":\"" TOPIC_BASE "/rssi\","
      "\"unit_of_meas\":\"dBm\",\"dev_cla\":\"signal_strength\","
      "\"entity_category\":\"diagnostic\","
      "\"uniq_id\":\"esp32_station_rssi\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_lastseen/config",
      "{\"name\":\"Derniere maj\",\"stat_t\":\"" TOPIC_BASE "/last_seen\","
      "\"dev_cla\":\"timestamp\",\"entity_category\":\"diagnostic\","
      "\"uniq_id\":\"esp32_station_lastseen\"" + dev + "}" },
  };

  for (auto& s : sensors) {
    mqtt.publish(s.topic, s.payload.c_str(), true);
  }
}

void mqttReconnect() {
  if (mqtt.connected() || WiFi.status() != WL_CONNECTED) return;

  bool ok;
#ifdef MQTT_USER
  ok = mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS,
                    TOPIC_STATUS, 0, true, "offline");
#else
  ok = mqtt.connect(MQTT_CLIENT_ID, nullptr, nullptr,
                    TOPIC_STATUS, 0, true, "offline");
#endif

  if (ok) {
    mqttConnected = true;
    mqtt.publish(TOPIC_STATUS, "online", true);
    mqtt.subscribe(TOPIC_ALARM_CMD);
    publishDiscovery();
    Serial.println("MQTT: connecte");
  } else {
    mqttConnected = false;
    Serial.printf("MQTT: echec rc=%d\n", mqtt.state());
  }
}

void publishSensorData() {
  if (!mqtt.connected()) return;
  mqtt.publish(TOPIC_BASE "/temperature", String(temperature, 1).c_str(), true);
  mqtt.publish(TOPIC_BASE "/humidity",    String(humidity, 1).c_str(),    true);
  mqtt.publish(TOPIC_BASE "/gas",         String(map(gasValue, 0, 4095, 0, 100)).c_str(), true);
  mqtt.publish(TOPIC_BASE "/lux",         String(ldrToLux(ldrValue), 0).c_str(), true);
  mqtt.publish(TOPIC_BASE "/uv",          String(uvIndex, 1).c_str(),    true);
  mqtt.publish(TOPIC_BASE "/sound",       String(soundDecibel, 1).c_str(), true);
  mqtt.publish(TOPIC_BASE "/presence",    presenceDetected ? "ON" : "OFF", true);
  mqtt.publish(TOPIC_BASE "/alarm",       isAlarmActive    ? "ON" : "OFF", true);
  mqtt.publish(TOPIC_BASE "/heat_index",  String(heatIdx, 1).c_str(),    true);
  mqtt.publish(TOPIC_BASE "/moving_dist", String(movingDistance).c_str(),    true);
  mqtt.publish(TOPIC_BASE "/static_dist", String(stationaryDistance).c_str(), true);
  mqtt.publish(TOPIC_BASE "/rssi",        String(WiFi.RSSI()).c_str(),    true);

  // Horodatage ISO 8601 (UTC) pour Home Assistant
  time_t epoch = (time_t)timeClient.getEpochTime();
  struct tm* ti = gmtime(&epoch);
  char isoTs[32];
  strftime(isoTs, sizeof(isoTs), "%Y-%m-%dT%H:%M:%S+00:00", ti);
  mqtt.publish(TOPIC_BASE "/last_seen", isoTs, true);
}

// ===================== TFT — ICONES ANIMEES =====================

// Soleil anime : corps + rayons rotatifs
void drawAnimSun(int cx, int cy, int r, uint8_t frame, uint16_t color) {
  tft.fillCircle(cx, cy, r, color);
  for (int i = 0; i < 8; i++) {
    float a = frame * 0.3927f + i * 0.7854f;   // 22.5° par frame, 45° entre rayons
    int x1 = cx + (int)(cosf(a) * (r + 2));
    int y1 = cy + (int)(sinf(a) * (r + 2));
    int x2 = cx + (int)(cosf(a) * (r + 6));
    int y2 = cy + (int)(sinf(a) * (r + 6));
    tft.drawLine(x1, y1, x2, y2, color);
  }
}

// Nuage (formes arrondies superposees)
void drawAnimCloud(int cx, int cy, uint16_t color) {
  tft.fillCircle(cx - 7, cy + 2, 6, color);
  tft.fillCircle(cx + 7, cy + 2, 6, color);
  tft.fillCircle(cx,     cy - 3, 7, color);
  tft.fillCircle(cx + 4, cy - 1, 5, color);
  tft.fillRect(cx - 13, cy + 2, 26, 6, color);
}

// Flocon de neige rotatif (6 branches + sous-branches)
void drawAnimSnowflake(int cx, int cy, uint8_t frame, uint16_t color) {
  float off = frame * 0.2618f;  // ~15° par frame
  for (int i = 0; i < 6; i++) {
    float a = off + i * 1.0472f;  // 60° entre branches
    int xe = cx + (int)(cosf(a) * 10);
    int ye = cy + (int)(sinf(a) * 10);
    tft.drawLine(cx, cy, xe, ye, color);
    // Petites branches laterales
    int xm = cx + (int)(cosf(a) * 6);
    int ym = cy + (int)(sinf(a) * 6);
    tft.drawLine(xm, ym, xm + (int)(cosf(a + 0.7854f) * 3),
                          ym + (int)(sinf(a + 0.7854f) * 3), color);
    tft.drawLine(xm, ym, xm + (int)(cosf(a - 0.7854f) * 3),
                          ym + (int)(sinf(a - 0.7854f) * 3), color);
  }
  tft.fillCircle(cx, cy, 2, color);  // Centre
}

// Flamme dansante (decalage aleatoire par frame)
void drawAnimFlame(int cx, int cy, uint8_t frame) {
  int dx = (frame % 3) - 1;  // oscille -1, 0, +1
  tft.fillTriangle(cx + dx, cy - 12, cx - 6, cy + 6, cx + 6, cy + 6, TFT_RED);
  tft.fillTriangle(cx - dx, cy - 7,  cx - 3, cy + 6, cx + 3, cy + 6, TFT_YELLOW);
  tft.fillCircle(cx, cy + 3, 2, TFT_ORANGE);
}

// Gouttes de pluie animees sous un nuage
void drawAnimRain(int cx, int cy, uint8_t frame, uint16_t color) {
  drawAnimCloud(cx, cy - 6, 0x7BEF);  // nuage gris clair
  for (int i = 0; i < 3; i++) {
    int rx = cx - 8 + i * 8;
    int ry = cy + 4 + ((frame + i * 3) % 6);  // gouttes qui tombent
    tft.drawLine(rx, ry, rx - 1, ry + 3, color);
  }
}

// Dessine les 2 icones meteo dans le bandeau bas de la page 1
void drawWeatherIcons(uint8_t frame) {
  // --- Icone temperature : zone x=4..38, y=198..234 ---
  tft.fillRect(4, 198, 36, 36, TFT_BLACK);

  if (temperature < 5.0f) {
    drawAnimSnowflake(22, 216, frame, TFT_CYAN);
  } else if (temperature < 15.0f) {
    drawAnimRain(22, 216, frame, 0x4A7F);   // frais → pluie bleue
  } else if (temperature < 22.0f) {
    drawAnimCloud(22, 214, 0x7BEF);         // doux → nuage
  } else if (temperature < 30.0f) {
    drawAnimSun(22, 214, 7, frame, TFT_YELLOW);  // chaud → soleil
  } else {
    drawAnimFlame(22, 216, frame);           // tres chaud → flamme
  }

  // --- Icone UV : zone x=44..80, y=198..234 ---
  tft.fillRect(44, 198, 38, 36, TFT_BLACK);

  if (uvIndex < 1.0f) {
    drawAnimCloud(64, 214, TFT_DARKGREY);        // pas d'UV → nuage sombre
  } else if (uvIndex < 3.0f) {
    drawAnimSun(68, 210, 5, frame, TFT_YELLOW);  // soleil voile
    drawAnimCloud(58, 216, 0x7BEF);
  } else if (uvIndex < 6.0f) {
    drawAnimSun(64, 214, 8, frame, TFT_YELLOW);  // UV modere → soleil jaune
  } else if (uvIndex < 8.0f) {
    drawAnimSun(64, 214, 8, frame, TFT_ORANGE);  // UV fort → soleil orange
  } else {
    int pulse = 8 + (frame % 3);                  // UV extreme → soleil rouge pulsant
    drawAnimSun(64, 214, pulse, frame, TFT_RED);
  }
}

// ===================== TFT — ANIMATION DE DEMARRAGE =====================
// Position de la barre de progression
#define BOOT_BAR_X      40
#define BOOT_BAR_Y      188
#define BOOT_BAR_W      240
#define BOOT_BAR_H      14
#define BOOT_TOTAL_STEPS 5    // TFT, LD2410, SPIFFS, WiFi, MQTT

uint8_t bootStepDone = 0;

// Logo soleil + nuage anime au centre
void drawBootLogo(int cx, int cy, uint8_t frame, uint8_t scale) {
  // Cercle principal du soleil (jaune) — pulse legerement
  uint8_t pulse = (frame / 2) % 4;
  int rSun = scale + pulse;
  tft.fillCircle(cx, cy, rSun, TFT_YELLOW);
  // Reflet plus clair en haut a gauche (effet 3D)
  tft.fillCircle(cx - rSun/3, cy - rSun/3, rSun/3, 0xFFE8);

  // Rayons rotatifs — 12 rayons longs et fins
  for (int i = 0; i < 12; i++) {
    float a = frame * 0.08f + i * 0.5236f;  // 30 degres entre rayons
    int r1 = rSun + 4;
    int r2 = rSun + 14;
    int x1 = cx + (int)(cosf(a) * r1);
    int y1 = cy + (int)(sinf(a) * r1);
    int x2 = cx + (int)(cosf(a) * r2);
    int y2 = cy + (int)(sinf(a) * r2);
    tft.drawLine(x1, y1, x2, y2, TFT_ORANGE);
    // Rayon plus epais
    tft.drawLine(x1+1, y1, x2+1, y2, TFT_ORANGE);
  }
}

// Sequence d'animation initiale (avant les inits)
void bootAnimation() {
  tft.fillScreen(TFT_BLACK);

  int cx = 160, cy = 70;

  // Phase 1 : le soleil grandit progressivement (zoom-in)
  for (uint8_t s = 4; s <= 22; s += 2) {
    drawBootLogo(cx, cy, 0, s);
    delay(35);
  }

  // Phase 2 : rotation des rayons pendant ~1s
  for (uint8_t f = 0; f < 25; f++) {
    drawBootLogo(cx, cy, f, 22);
    delay(35);
  }

  // Phase 3 : titre "Station Meteo" apparait par effet machine a ecrire
  const char* title = "Station Meteo";
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextSize(3);
  // size 3 → 18px par char ; "Station Meteo" = 13 chars = 234px → centre = (320-234)/2 = 43
  int titleX = 43, titleY = 120;
  for (uint8_t i = 0; i <= strlen(title); i++) {
    tft.setCursor(titleX, titleY);
    for (uint8_t j = 0; j < i; j++) tft.print(title[j]);
    delay(60);
  }

  // Phase 4 : sous-titre
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(118, 152);    // "ESP32-S3 v1.0" = 13 chars * 6 = 78px ; centre = 121
  tft.print("ESP32-S3 v1.0");

  // Phase 5 : ligne separatrice elegante (s'etend du centre vers les bords)
  for (int w = 0; w < 140; w += 8) {
    tft.drawFastHLine(160 - w, 170, w * 2, TFT_DARKGREY);
    delay(15);
  }

  // Phase 6 : cadre de la barre de progression
  tft.drawRect(BOOT_BAR_X - 1, BOOT_BAR_Y - 1, BOOT_BAR_W + 2, BOOT_BAR_H + 2, TFT_DARKGREY);
}

// Met a jour la barre + affiche un message (appele apres chaque init)
void bootStep(const char* label, bool ok) {
  bootStepDone++;
  if (bootStepDone > BOOT_TOTAL_STEPS) bootStepDone = BOOT_TOTAL_STEPS;

  // Remplit progressivement (effet d'animation rapide)
  int targetW = (BOOT_BAR_W * bootStepDone) / BOOT_TOTAL_STEPS;
  static int currentW = 0;
  uint16_t color = ok ? TFT_GREEN : TFT_RED;
  for (int w = currentW; w <= targetW; w += 4) {
    tft.fillRect(BOOT_BAR_X, BOOT_BAR_Y, w, BOOT_BAR_H, color);
    delay(8);
  }
  currentW = targetW;

  // Texte status (efface puis ecrit, centre)
  tft.fillRect(0, BOOT_BAR_Y + BOOT_BAR_H + 8, 320, 12, TFT_BLACK);
  tft.setTextColor(ok ? TFT_GREEN : TFT_RED, TFT_BLACK);
  tft.setTextSize(1);
  int textW = strlen(label) * 6;
  tft.setCursor((320 - textW) / 2, BOOT_BAR_Y + BOOT_BAR_H + 8);
  tft.print(label);
}

// ===================== TFT — Page 1 : Capteurs =====================
void tftDrawStaticPage1() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 8);
  tft.print("Station Meteo");
  tft.drawFastHLine(0, 30, 320, TFT_DARKGREY);
  drawPageIndicator(tftPage);

  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(10,  36);  tft.print("TEMPERATURE");
  tft.setCursor(170, 36);  tft.print("HUMIDITE");
  
  tft.setCursor(10,  90);  tft.print("GAZ / FUMEE");
  tft.setCursor(170, 90);  tft.print("SON");
  
  tft.setCursor(10,  144); tft.print("LUX");
  tft.setCursor(170, 144); tft.print("UV");

  tft.drawFastHLine(0, 196, 320, TFT_DARKGREY);
  // Labels sous les icones dans le bandeau bas
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(8, 234);   tft.print("TEMP");
  tft.setCursor(52, 234);  tft.print("UV");
  // (indicateur de pages = dots dans le titre)
}

void tftUpdateValues1() {
  unsigned long nowMs = millis();

  // --- Detection de changement significatif → pulse 800ms ---
  if (prevTemp != -999 && fabsf(temperature - prevTemp) >= 0.5f) pulseTempUntil = nowMs + 800;
  if (prevHum  != -999 && fabsf(humidity    - prevHum)  >= 1.0f) pulseHumUntil  = nowMs + 800;
  prevTemp = temperature;
  prevHum  = humidity;

  // Temperature (size 3) — pulse blanc si changement
  uint16_t cT;
  if (nowMs < pulseTempUntil) {
    cT = TFT_WHITE;
  } else {
    cT = (temperature > 30) ? TFT_RED : (temperature > 25) ? TFT_ORANGE : TFT_GREEN;
  }
  tft.setTextColor(cT, TFT_BLACK);
  tft.setTextSize(3);
  tft.setCursor(10, 48);
  tft.printf("%-5.1f C ", temperature);

  // Humidite (size 3) — pulse blanc si changement
  uint16_t cH;
  if (nowMs < pulseHumUntil) {
    cH = TFT_WHITE;
  } else {
    cH = (humidity > 75) ? TFT_CYAN : (humidity < 30) ? TFT_ORANGE : TFT_GREEN;
  }
  tft.setTextColor(cH, TFT_BLACK);
  tft.setCursor(170, 48);
  tft.printf("%-5.1f %% ", humidity);

  // Gaz (size 2)
  int gasPct = map(gasValue, 0, 4095, 0, 100);
  uint16_t cG = (gasPct > gasThresholdPct) ? TFT_RED
              : (gasPct > (int)(gasThresholdPct * 0.7)) ? TFT_ORANGE : TFT_GREEN;
  tft.setTextColor(cG, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 102);
  tft.printf("%-5d %%  ", gasPct);

  // Son (size 2)
  uint16_t cS = (soundDecibel > dbThreshold) ? TFT_RED
              : (soundDecibel > dbThreshold * 0.8f) ? TFT_ORANGE : TFT_GREEN;
  tft.setTextColor(cS, TFT_BLACK);
  tft.setCursor(170, 102);
  tft.printf("%-5.1f dB ", soundDecibel);

  // Lux (size 2)
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(10, 156);
  tft.printf("%-6.0f ", ldrToLux(ldrValue));

  // UV (size 2)
  uint16_t cU = (uvIndex >= 8) ? TFT_RED : (uvIndex >= 6) ? TFT_ORANGE
              : (uvIndex >= 3) ? TFT_YELLOW : TFT_GREEN;
  tft.setTextColor(cU, TFT_BLACK);
  tft.setCursor(170, 156);
  tft.printf("%-4.1f   ", uvIndex);

  // Heure / date — decale a droite pour laisser place aux icones animees
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(100, 200);
  tft.printf("%-8s", currentTime);
  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setCursor(100, 220);
  tft.printf("%-10s", currentDate);
}

// ===================== TFT — Page 2 : Systeme =====================
void tftDrawStaticPage2() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 8);
  tft.print("Systeme");
  tft.drawFastHLine(0, 30, 320, TFT_DARKGREY);
  drawPageIndicator(tftPage);

  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(10,  36);  tft.print("PRESENCE (LD2410C)");
  tft.setCursor(170, 36);  tft.print("WIFI RSSI");
  
  tft.setCursor(10,  90);  tft.print("IP");
  tft.setCursor(170, 90);  tft.print("MQTT");
  
  tft.setCursor(10,  144); tft.print("ALARME");
  tft.setCursor(170, 144); tft.print("UPTIME / RAM");

  tft.drawFastHLine(0, 196, 320, TFT_DARKGREY);
}

void tftUpdateValues2() {
  // Presence
  tft.setTextSize(2);
  tft.setCursor(10, 48);
  if (movingDetected) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.printf("Mvt  %3dcm  ", movingDistance);
  } else if (stationaryDetected) {
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.printf("Stat %3dcm  ", stationaryDistance);
  } else {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.print("Aucune      ");
  }

  // WiFi RSSI : valeur + barres animees
  int rssi = WiFi.RSSI();
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(170, 48);
  tft.printf("%-5d dBm", rssi);
  // Efface zone barres puis redessine
  tft.fillRect(280, 46, 35, 22, TFT_BLACK);
  drawWiFiBars(280, 46, rssi);

  // IP
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 102);
  tft.printf("%-20s", WiFi.localIP().toString().c_str());

  // MQTT
  tft.setTextSize(2);
  tft.setCursor(170, 102);
  if (mqttConnected) {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.print("OK  ");
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print("ERR ");
  }

  // Alarme
  tft.setTextSize(2);
  tft.setCursor(10, 156);
  if (isAlarmActive && alarmEnabled) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print("ALERTE  ");
  } else if (alarmEnabled) {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.print("ARMEE   ");
  } else {
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.print("OFF     ");
  }

  // Uptime + Free heap
  unsigned long up = millis() / 1000;
  uint32_t days  = up / 86400;
  uint32_t hours = (up % 86400) / 3600;
  uint32_t mins  = (up % 3600) / 60;
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(170, 156);
  tft.printf("%lud %02luh %02lum   ", days, hours, mins);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setCursor(170, 168);
  tft.printf("RAM: %u o   ", ESP.getFreeHeap());

  // Heure / date
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 210);
  tft.printf("%-8s", currentTime);
  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setCursor(120, 215);
  tft.printf("%-10s", currentDate);
}

// ===================== TFT — Page 3 : Graphiques (10 min) =====================
// Dessine une mini-courbe dans une zone donnee
template<typename T>
void drawMiniChart(int x, int y, int w, int h, T* data, uint8_t size, bool filled,
                   uint8_t cur, uint16_t color, T minOverride = (T)0, T maxOverride = (T)0) {
  // Cadre
  tft.drawRect(x, y, w, h, TFT_DARKGREY);

  uint8_t n = filled ? size : cur;
  if (n < 2) return;

  // Determine min/max auto (si overrides == 0)
  T mn = data[0], mx = data[0];
  for (uint8_t i = 0; i < n; i++) {
    uint8_t idx = filled ? (cur + i) % size : i;
    if (data[idx] < mn) mn = data[idx];
    if (data[idx] > mx) mx = data[idx];
  }
  if (minOverride != maxOverride) { mn = minOverride; mx = maxOverride; }
  if (mx - mn < 1) mx = mn + 1;  // evite division par 0

  // Fix : initialiser prevX/prevY a partir du PREMIER point reel (plus de ligne fantome du coin)
  int prevX = 0, prevY = 0;
  for (uint8_t i = 0; i < n; i++) {
    uint8_t idx = filled ? (cur + i) % size : i;
    int px = x + 1 + (i * (w - 2)) / (n - 1);
    int py = y + h - 2 - (int)(((float)(data[idx] - mn) / (float)(mx - mn)) * (h - 3));
    if (i == 0) {
      prevX = px; prevY = py;          // premier point : pas de trace
    } else {
      tft.drawLine(prevX, prevY, px, py, color);
      prevX = px; prevY = py;
    }
  }
}

void tftDrawStaticPage3() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 8);
  tft.print("Tendances 10 min");
  tft.drawFastHLine(0, 30, 320, TFT_DARKGREY);
  drawPageIndicator(tftPage);

  // 3 zones : Temperature, Humidite, Lux
  tft.setTextSize(1);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setCursor(10, 36); tft.print("TEMPERATURE  (°C)");
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(10, 92); tft.print("HUMIDITE  (%)");
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(10, 148); tft.print("LUMINOSITE  (lux)");

  tft.drawFastHLine(0, 196, 320, TFT_DARKGREY);
}

void tftUpdateValues3() {
  // Efface les zones de tracage avant redessin
  tft.fillRect(10, 46, 300, 42, TFT_BLACK);   // zone temp
  tft.fillRect(10, 102, 300, 42, TFT_BLACK);  // zone hum
  tft.fillRect(10, 158, 300, 38, TFT_BLACK);  // zone lux

  // Trace les 3 courbes
  drawMiniChart<float>(10, 46,  300, 42, tempHist, HIST_SIZE, histFilled, histIdx, TFT_RED);
  drawMiniChart<float>(10, 102, 300, 42, humHist,  HIST_SIZE, histFilled, histIdx, TFT_CYAN);
  drawMiniChart<int>  (10, 158, 300, 38, luxHist,  HIST_SIZE, histFilled, histIdx, TFT_YELLOW);

  // Valeurs courantes a droite des labels
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(220, 36); tft.printf("Cur:%5.1f", temperature);
  tft.setCursor(220, 92); tft.printf("Cur:%5.1f", humidity);
  tft.setCursor(220, 148); tft.printf("Cur:%5.0f", ldrToLux(ldrValue));

  // Heure / date
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 210);
  tft.printf("%-8s", currentTime);
  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.setCursor(120, 215);
  tft.printf("%-10s", currentDate);
}

// ===================== WEB =====================
// Helper : verifie l'auth Basic, renvoie true si OK, sinon envoie 401
bool checkAuth(AsyncWebServerRequest* req) {
  if (!req->authenticate(WEB_USER, WEB_PASS)) {
    req->requestAuthentication();
    return false;
  }
  return true;
}

void setupWebRoutes() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    req->redirect("/index.html");
  });

  server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    req->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/settings.html", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    req->send(SPIFFS, "/settings.html", "text/html");
  });

  server.on("/chart.js", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    req->send(SPIFFS, "/chart.js", "text/javascript");
  });

  // Endpoint pour telecharger le crash log
  server.on("/crashlog", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    if (SPIFFS.exists("/crashlog.txt")) {
      req->send(SPIFFS, "/crashlog.txt", "text/plain");
    } else {
      req->send(200, "text/plain", "Aucun crash enregistre");
    }
  });

  // Endpoint pour effacer le crash log
  server.on("/crashlog/clear", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    SPIFFS.remove("/crashlog.txt");
    req->send(200, "text/plain", "Crash log efface");
  });

  // Endpoint pour redemarrer manuellement
  server.on("/restart", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    req->send(200, "text/plain", "Redemarrage...");
    delay(200);
    ESP.restart();
  });

  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    bool updated = false;
    if (req->hasParam("threshold"))    { gasThresholdPct = req->getParam("threshold")->value().toInt();    updated = true; }
    if (req->hasParam("dbThreshold"))  { dbThreshold     = req->getParam("dbThreshold")->value().toInt();  updated = true; }
    if (req->hasParam("dbCorrection")) { dbCorrection    = req->getParam("dbCorrection")->value().toInt(); updated = true; }
    if (updated) saveSettings();   // persistance NVS
    req->send(updated ? 200 : 400, "text/plain", updated ? "OK" : "Parametre manquant");
  });

  server.on("/alarm", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    if (!req->hasParam("state")) { req->send(400, "text/plain", "Parametre manquant"); return; }
    String state = req->getParam("state")->value();
    if (state == "on") {
      alarmEnabled = true;
      saveSettings();
      req->send(200, "text/plain", "ALARM_ON");
    } else if (state == "off") {
      alarmEnabled  = false;
      isAlarmActive = false;
      stopBuzzer();
      saveSettings();
      req->send(200, "text/plain", "ALARM_OFF");
    } else {
      req->send(400, "text/plain", "Etat invalide");
    }
  });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    unsigned long now = millis();
    // Cache : ne reconstruit le JSON qu'au max toutes les 1s
    if (dataJsonCache.length() > 0 && (now - dataJsonCacheTime) < DATA_CACHE_MS) {
      req->send(200, "application/json", dataJsonCache);
      return;
    }
    dataJsonCacheTime = now;

    String json;
    json.reserve(640);
    json = "{";
    json += "\"temp\":"           + String(temperature, 1)          + ",";
    json += "\"hum\":"            + String(humidity, 1)             + ",";
    json += "\"heatIndex\":"      + String(heatIdx, 1)              + ",";
    json += "\"lux\":"            + String(ldrToLux(ldrValue), 0)   + ",";
    json += "\"gasPct\":"         + String(map(gasValue, 0, 4095, 0, 100)) + ",";
    json += "\"soundDecibel\":"   + String(soundDecibel, 1)         + ",";
    json += "\"uvIndex\":"        + String(uvIndex, 1)              + ",";
    json += "\"dbThreshold\":"    + String(dbThreshold)             + ",";
    json += "\"dbCorrection\":"   + String(dbCorrection)            + ",";
    json += "\"gasThreshold\":"   + String(gasThresholdPct)         + ",";
    json += "\"presence\":"       + String(presenceDetected   ? "true" : "false") + ",";
    json += "\"presenceMoving\":" + String(movingDetected     ? "true" : "false") + ",";
    json += "\"presenceStatic\":" + String(stationaryDetected ? "true" : "false") + ",";
    json += "\"movingDist\":"     + String(movingDistance)           + ",";
    json += "\"staticDist\":"     + String(stationaryDistance)       + ",";
    json += "\"alarmEnabled\":"   + String(alarmEnabled  ? "true" : "false") + ",";
    json += "\"isAlarmActive\":"  + String(isAlarmActive ? "true" : "false") + ",";
    json += "\"wifiRSSI\":"       + String(WiFi.RSSI())              + ",";
    json += "\"wifiBars\":"       + String(rssiToBars(WiFi.RSSI()))  + ",";
    json += "\"wifiConnected\":"  + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
    json += "\"mqttConnected\":"  + String(mqttConnected ? "true" : "false") + ",";
    json += "\"uptime\":"         + String(millis() / 1000)          + ",";
    json += "\"freeHeap\":"       + String(ESP.getFreeHeap())        + ",";
    json += "\"time\":\""         + String(currentTime)              + "\",";
    json += "\"date\":\""         + String(currentDate)              + "\",";
    json += "\"ip\":\""           + WiFi.localIP().toString()        + "\"";
    json += "}";
    dataJsonCache = json;   // memorise pour les prochains appels
    req->send(200, "application/json", json);
  });
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  Serial.println("\n--- Station Meteo ESP32-S3 ---");

  // Affiche raison du dernier redemarrage (et log si crash)
  logResetReason();

  // Desactive Bluetooth (economie d'energie ~25mA — on ne l'utilise pas)
  btStop();

  // Watchdog : redemarre si la loop bloque > 30s
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
  Serial.printf("Watchdog active (%ds)\n", WDT_TIMEOUT_S);

  // Charger les reglages persistes
  loadSettings();

  dht.begin();
  pinMode(TOUCH_PIN, INPUT_PULLDOWN);   // pull-down : etat 0 si fil debranche

  // Buzzer (LEDC channel 0)
  ledcSetup(BUZZER_LEDC_CHANNEL, BUZZER_FREQ, BUZZER_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CHANNEL);
  stopBuzzer();

  // TFT + animation de demarrage
  tft.init();
  tft.setRotation(1); // Passage en mode horizontal
  bootAnimation();
  bootStep("TFT initialise", true);
  esp_task_wdt_reset();

  // LD2410C — Cablage : GPIO 13 (RX) ← TX radar | GPIO 14 (TX) → RX radar
  LD2410_Serial.begin(256000, SERIAL_8N1, LD2410_RX, LD2410_TX);
  delay(1500); // Le radar met ~1s a demarrer apres mise sous tension
  bool ld2410ok = radar.begin(LD2410_Serial);
  if (ld2410ok) {
    Serial.println("LD2410C: OK");
    Serial.printf("  RX=GPIO%d (← radar TX)  TX=GPIO%d (→ radar RX)\n", LD2410_RX, LD2410_TX);
  } else {
    Serial.println("LD2410C: erreur init — verifier cablage RX/TX");
  }
  bootStep(ld2410ok ? "Radar LD2410C: OK" : "Radar LD2410C: ERR", ld2410ok);
  esp_task_wdt_reset();

  // SPIFFS
  bool spiffsOk = SPIFFS.begin(true);
  if (!spiffsOk) Serial.println("ERREUR SPIFFS");
  bootStep(spiffsOk ? "Stockage SPIFFS: OK" : "Stockage SPIFFS: ERR", spiffsOk);
  esp_task_wdt_reset();

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(STA_SSID, STA_PASS);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500); retry++; Serial.print(".");
    esp_task_wdt_reset();
  }
  bool wifiOk = (WiFi.status() == WL_CONNECTED);
  if (wifiOk) Serial.println("\nWiFi: " + WiFi.localIP().toString());
  else        Serial.println("\nWiFi: timeout");
  bootStep(wifiOk ? "WiFi connecte" : "WiFi: timeout", wifiOk);
  esp_task_wdt_reset();

  // NTP + Fuseau POSIX (gestion automatique heure d'ete/hiver pour la France)
  timeClient.begin();
  timeClient.update();
  configTime(0, 0, NTP_SERVER);                 // sync brute UTC
  setenv("TZ", POSIX_TZ, 1);                    // applique le fuseau POSIX
  tzset();
  Serial.println("Fuseau horaire applique (DST auto)");

  // mDNS : accessible via http://station-meteo.local
  if (MDNS.begin(DEVICE_HOSTNAME)) {
    Serial.printf("mDNS: http://%s.local/\n", DEVICE_HOSTNAME);
    MDNS.addService("http", "tcp", 80);
  }

  // OTA (Over-The-Air firmware update)
  ArduinoOTA.setHostname(DEVICE_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    Serial.println("OTA: debut update");
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(40, 100);
    tft.print("OTA Update...");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("OTA: termine");
    tft.setCursor(60, 130);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.print("OK - Reboot");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    int pct = (progress * 100) / total;
    tft.fillRect(40, 140, 240, 14, TFT_BLACK);
    tft.drawRect(40, 140, 240, 14, TFT_DARKGREY);
    tft.fillRect(41, 141, (238 * pct) / 100, 12, TFT_GREEN);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA: erreur %u\n", error);
  });
  ArduinoOTA.begin();
  Serial.println("OTA pret");

  // MQTT
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(1024);  // augmente pour les payloads de discovery
  mqttReconnect();
  bootStep(mqttConnected ? "MQTT connecte" : "MQTT non joignable", mqttConnected);
  esp_task_wdt_reset();

  // Web
  setupWebRoutes();
  server.begin();
  Serial.println("Serveur web: OK");

  // Affiche l'IP en bas de la barre pendant 1s
  tft.fillRect(0, BOOT_BAR_Y + BOOT_BAR_H + 8, 320, 12, TFT_BLACK);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextSize(1);
  String ipMsg = "IP: " + WiFi.localIP().toString();
  int textW = ipMsg.length() * 6;
  tft.setCursor((320 - textW) / 2, BOOT_BAR_Y + BOOT_BAR_H + 8);
  tft.print(ipMsg);
  delay(1200);

  // Transition : fade-out par bandes horizontales (effet rideau)
  for (int y = 0; y < 240; y += 6) {
    tft.fillRect(0, y, 320, 6, TFT_BLACK);
    delay(8);
  }

  // Memorise l'epoch de boot (pour calculer uptime UTC robuste)
  bootEpoch  = (uint32_t)timeClient.getEpochTime();
  bootMillis = millis();

  tftNeedsRedraw = true;
  lastActivity   = millis();   // l'ecran reste allume au demarrage
  Serial.println("Init terminee");
}

// ===================== LOOP =====================
void loop() {
  unsigned long now = millis();

  // --- OTA ---
  ArduinoOTA.handle();

  // --- Capteurs DHT (toutes les 2s, throttle imperatif) ---
  static unsigned long lastDhtRead = 0;
  if (lastDhtRead == 0) lastDhtRead = now;   // FIX : evite NaN au boot
  if (now - lastDhtRead > 2000) {
    lastDhtRead = now;
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t)) {
      tempBuf[dhtAvgIdx] = t;
      temperature = avgOf(tempBuf, AVG_SIZE, dhtAvgFilled, dhtAvgIdx + 1);
    }
    if (!isnan(h)) {
      humBuf[dhtAvgIdx] = h;
      humidity = avgOf(humBuf, AVG_SIZE, dhtAvgFilled, dhtAvgIdx + 1);
    }
    heatIdx = computeHeatIndex(temperature, humidity);
    dhtAvgIdx = (dhtAvgIdx + 1) % AVG_SIZE;
    if (dhtAvgIdx == 0) dhtAvgFilled = true;
  }

  ldrValue = analogRead(LDR_PIN);
  uvIndex  = readUvIndex();

  // --- Gaz et son : echantillonnage rapide independant (toutes les 500ms) ---
  if (lastFastSample == 0) lastFastSample = now;
  if (now - lastFastSample > 500) {
    lastFastSample = now;
    int gasRaw = analogRead(GAS_PIN);
    gasBuf[fastAvgIdx] = gasRaw;
    gasValue = avgOf(gasBuf, AVG_SIZE, fastAvgFilled, fastAvgIdx + 1);

    float sndRaw = analogToDecibel(readSoundSensor());
    soundBuf[fastAvgIdx] = sndRaw;
    soundDecibel = avgOf(soundBuf, AVG_SIZE, fastAvgFilled, fastAvgIdx + 1);

    fastAvgIdx = (fastAvgIdx + 1) % AVG_SIZE;
    if (fastAvgIdx == 0) fastAvgFilled = true;
  }

  // --- LD2410C ---
  if (radar.read()) {
    presenceDetected   = radar.presenceDetected();
    movingDetected     = radar.movingTargetDetected();
    stationaryDetected = radar.stationaryTargetDetected();
    movingDistance      = movingDetected     ? radar.movingTargetDistance()     : 0;
    stationaryDistance  = stationaryDetected ? radar.stationaryTargetDistance() : 0;
  }

  // --- NTP + heure locale (DST automatique via POSIX TZ) ---
  timeClient.update();
  time_t epochUTC = time(nullptr);     // utilise le fuseau POSIX configure
  struct tm tinfo;
  localtime_r(&epochUTC, &tinfo);
  sprintf(currentTime, "%02d:%02d:%02d", tinfo.tm_hour, tinfo.tm_min, tinfo.tm_sec);
  sprintf(currentDate, "%02d/%02d/%04d", tinfo.tm_mday, tinfo.tm_mon + 1, tinfo.tm_year + 1900);

  // --- Alarme avec anti-rebond et warmup gaz ---
  int  gasPct       = map(gasValue, 0, 4095, 0, 100);
  bool gasWarmupOk  = (millis() / 1000) >= GAS_WARMUP_SEC;   // ignore gaz les 60 premieres secondes
  bool gasAlarm     = gasWarmupOk && (gasPct > gasThresholdPct);
  bool dbAlarm      = soundDecibel > dbThreshold;
  bool seuilDepasse = gasAlarm || dbAlarm;

  if (alarmEnabled && seuilDepasse) {
    if (alarmConfirmCount < ALARM_CONFIRM_COUNT) alarmConfirmCount++;
    if (alarmConfirmCount >= ALARM_CONFIRM_COUNT && !isAlarmActive) {
      isAlarmActive = true;
      Serial.println("ALARME DECLENCHEE !");
    }
  } else {
    if (alarmConfirmCount > 0) alarmConfirmCount--;
  }

  if (isAlarmActive && alarmEnabled) {
    sirenUpdate(now);              // sirene non-bloquante (1.5kHz / 3kHz)
  } else {
    stopBuzzer();
    if (!alarmEnabled) isAlarmActive = false;
  }

  // --- Surveillance WiFi : si plus de 5min sans connexion, reboot ---
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiLostSince == 0) {
      wifiLostSince = now;
      Serial.println("WiFi: deconnecte, attente de reconnexion...");
    } else if (now - wifiLostSince > WIFI_DEAD_TIMEOUT_MS) {
      Serial.println("WiFi mort depuis 5 min, reboot !");
      delay(200);
      ESP.restart();
    }
  } else {
    if (wifiLostSince != 0) {
      Serial.printf("WiFi: reconnecte (off pendant %lus)\n", (now - wifiLostSince) / 1000);
      wifiLostSince = 0;
    }
  }

  // --- MQTT ---
  if (!mqtt.connected()) {
    static unsigned long lastReconn = 0;
    if (now - lastReconn > 5000) { lastReconn = now; mqttReconnect(); }
    mqttConnected = false;
  } else {
    mqttConnected = true;
  }
  mqtt.loop();

  if (now - lastMqttPublish > 5000) {
    lastMqttPublish = now;
    publishSensorData();
  }

  // --- Bouton tactile TTP223 ---
  // 1er appui = reveil (sans changer de page) ; appuis suivants = cycle de pages
  bool touchState = digitalRead(TOUCH_PIN);
  if (touchState && !lastTouchState && (now - lastTouchTime > 300)) {
    lastTouchTime = now;
    lastActivity  = now;
    if (tftSleeping) {
      tftWake();
    } else {
      tftPage        = (tftPage + 1) % TFT_PAGE_COUNT;
      tftNeedsRedraw = true;
    }
  }
  lastTouchState = touchState;

  // --- Reveil sur detection de mouvement (front montant) ---
  if (movingDetected && !prevMovingDetect) {
    lastActivity = now;
    if (tftSleeping) tftWake();
  }
  prevMovingDetect = movingDetected;

  // --- Reveil force si alarme active ---
  if (isAlarmActive && alarmEnabled && tftSleeping) {
    lastActivity = now;
    tftWake();
  }

  // --- Mise en veille apres inactivite ---
  if (!tftSleeping && (now - lastActivity > TFT_SLEEP_TIMEOUT_MS)) {
    tftSleep();
  }

  // --- Mises a jour TFT (uniquement si l'ecran est allume) ---
  if (!tftSleeping) {
    if (tftNeedsRedraw) {
      if      (tftPage == 0) tftDrawStaticPage1();
      else if (tftPage == 1) tftDrawStaticPage2();
      else                   tftDrawStaticPage3();
      tftNeedsRedraw = false;
    }
    if (now - lastTftUpdate > 2000) {
      lastTftUpdate = now;
      if      (tftPage == 0) tftUpdateValues1();
      else if (tftPage == 1) tftUpdateValues2();
      else                   tftUpdateValues3();

      // --- Debug capteurs problematiques ---
      int rawSound = readSoundSensor();
      int rawUV    = analogRead(UV_PIN);
      Serial.printf("[DEBUG] Son: raw_p2p=%d  dB=%.1f  (correction=%d)\n",
                    rawSound, analogToDecibel(rawSound), dbCorrection);
      Serial.printf("[DEBUG] UV:  raw=%d  voltage=%.3fV  index=%.1f\n",
                    rawUV, rawUV * 3.3f / 4095.0f, uvIndex);
      Serial.printf("[DEBUG] LD2410: presence=%d  moving=%d(%dcm)  static=%d(%dcm)\n",
                    presenceDetected, movingDetected, movingDistance,
                    stationaryDetected, stationaryDistance);
    }

    // --- Animation des icones meteo (page 0 uniquement, toutes les 400ms) ---
    if (tftPage == 0 && (now - lastAnimUpdate > 400)) {
      lastAnimUpdate = now;
      animFrame++;
      drawWeatherIcons(animFrame);
    }
  }

  // --- Clignotement d'alarme (bordure rouge sur tout l'ecran) ---
  if (!tftSleeping && isAlarmActive && alarmEnabled && (now - lastAlarmFlash > 500)) {
    lastAlarmFlash  = now;
    alarmFlashState = !alarmFlashState;
    uint16_t c = alarmFlashState ? TFT_RED : TFT_BLACK;
    // 4 bordures de 4px d'epaisseur
    tft.fillRect(0,   0,   320, 4,   c);
    tft.fillRect(0,   236, 320, 4,   c);
    tft.fillRect(0,   0,   4,   240, c);
    tft.fillRect(316, 0,   4,   240, c);
  } else if (!tftSleeping && !isAlarmActive && alarmFlashState) {
    // Effacer les bordures quand l'alarme s'arrete
    alarmFlashState = false;
    tft.fillRect(0,   0,   320, 4,   TFT_BLACK);
    tft.fillRect(0,   236, 320, 4,   TFT_BLACK);
    tft.fillRect(0,   0,   4,   240, TFT_BLACK);
    tft.fillRect(316, 0,   4,   240, TFT_BLACK);
    tftNeedsRedraw = true;  // redessiner la page proprement
  }

  // --- Echantillonnage historique pour la page graphiques (toutes les 10s) ---
  if (now - lastHistSample > 10000) {
    lastHistSample = now;
    tempHist[histIdx] = temperature;
    humHist[histIdx]  = humidity;
    luxHist[histIdx]  = (int)ldrToLux(ldrValue);
    histIdx = (histIdx + 1) % HIST_SIZE;
    if (histIdx == 0) histFilled = true;
  }

  // --- Watchdog : reset a chaque tour de loop ---
  esp_task_wdt_reset();

  delay(100);
}
