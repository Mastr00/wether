#include <Arduino.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <ArduinoOTA.h>
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
#include <Wire.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <math.h>
#include <time.h>

// ===================== PINS =====================
#define I2C_SDA_PIN     15   // AHT20 + BMP280
#define I2C_SCL_PIN     16   // AHT20 + BMP280
#define LIGHT_PIN       4    // TEMT6000 analog output
#define GAS_PIN         5
#define UV_PIN          6    // GUVA-S12SD (sortie analogique)
#define BUZZER_PIN      9
#define LD2450_RX       13
#define LD2450_TX       14
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
#define TOPIC_ALARM_STATE TOPIC_BASE "/alarm_enabled"
#define TOPIC_AWAY_CMD  TOPIC_BASE "/away/set"

// ===================== WATCHDOG =====================
#define WDT_TIMEOUT_S   30   // redemarre si la loop est bloquee plus de 30s
                              // (laisse le temps aux reconnexions MQTT/WiFi de timeout)

// ===================== HISTORIQUE GRAPHIQUES =====================
#define HIST_SIZE       60   // 60 echantillons * 10s = 10 minutes de trend
#define LD2450_TARGETS  3
#define LD2450_FRAME_SIZE 30
#define LD2450_DEFAULT_BAUD 256000
#define LD2450_BOOT_SCAN_MS 300
#define LD2450_STALE_MS 1500
#define LD2450_TARGET_HOLD_MS 500
#define LD2450_TRACK_MATCH_MM 1200
#define LD2450_COMMAND_TIMEOUT_MS 400
#define LD2450_RX_BUFFER_SIZE 1024
#define LD2450_MQTT_SNAPSHOT_MS 1000

// ===================== OBJETS =====================
Adafruit_AHTX0   aht;
Adafruit_BMP280  bmp;
AsyncWebServer   server(80);
TFT_eSPI         tft;
WiFiClient       wifiClient;
PubSubClient     mqtt(wifiClient);
HardwareSerial   LD2450_Serial(2);
WiFiUDP          ntpUDP;
NTPClient        timeClient(ntpUDP, NTP_SERVER, NTP_OFFSET, 60000);
Preferences      prefs;

// ===================== VARIABLES CAPTEURS =====================
float temperature  = 0;
float humidity     = 0;
float pressureHpa  = 0;
int   lightRaw     = 0;
float luxValue     = 0;
int   gasValue     = 0;
float uvIndex      = 0.0;
float heatIdx      = 0.0;   // Indice de chaleur (ressenti) calcule

bool     ahtOk               = false;
bool     bmpOk               = false;
bool     presenceDetected    = false;
bool     movingDetected      = false;
bool     stationaryDetected  = false;
uint16_t movingDistance      = 0;
uint16_t stationaryDistance  = 0;

struct RadarTarget {
  bool valid;
  bool fresh;
  uint8_t sourceSlot;
  int16_t xMm;
  int16_t yMm;
  int16_t speedCms;
  uint16_t resolutionMm;
  uint16_t distanceMm;
  float angleDeg;
};

RadarTarget   radarTargets[LD2450_TARGETS] = {};
unsigned long radarTargetLastSeen[LD2450_TARGETS] = {};
uint8_t       radarTargetCount             = 0;
uint8_t       radarMovingCount             = 0;
uint8_t       radarStillCount              = 0;
unsigned long lastRadarFrame               = 0;
uint32_t      ld2450Baud                   = LD2450_DEFAULT_BAUD;
uint32_t      ld2450BytesRx                = 0;
uint32_t      ld2450FramesValid            = 0;
uint32_t      ld2450FramesInvalid          = 0;
unsigned long ld2450LastByteMs             = 0;
uint8_t       ld2450LastFrame[LD2450_FRAME_SIZE] = {};
bool          ld2450HasLastFrame           = false;
int8_t        ld2450RxPin                  = LD2450_RX;
int8_t        ld2450TxPin                  = -1;
bool          ld2450StreamStale            = false;
bool          ld2450ConfigOk               = false;
bool          ld2450DeferredConfigAttempted = false;
uint8_t       ld2450TrackingMode           = 0;  // 0 inconnu, 1 mono, 2 multi
bool          radarSnapshotDirty           = false;
unsigned long lastRadarMqttPublish         = 0;

// ===================== MOYENNES GLISSANTES =====================
// Index separes pour eviter de polluer les moyennes de capteurs differents
#define AVG_SIZE 5
float   tempBuf[AVG_SIZE]   = {0};
float   humBuf[AVG_SIZE]    = {0};
float   pressureBuf[AVG_SIZE] = {0};
int     gasBuf[AVG_SIZE]    = {0};
uint8_t envAvgIdx           = 0;   // pour temp + humidite + pression (2s)
uint8_t gasAvgIdx           = 0;   // pour gaz (toutes les 500ms)
bool    envAvgFilled        = false;
bool    gasAvgFilled        = false;
unsigned long lastGasSample = 0;

// ===================== HISTORIQUE TFT (page graphiques) =====================
float tempHist[HIST_SIZE] = {0};
float humHist[HIST_SIZE]  = {0};
int   luxHist[HIST_SIZE]  = {0};
uint8_t histIdx           = 0;
bool    histFilled        = false;
unsigned long lastHistSample = 0;

// ===================== VARIABLES ALARME =====================
bool alarmEnabled    = true;
bool awayMode        = false;
bool isAlarmActive   = false;
int  gasThresholdPct = 60;

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
float temt6000ToLux(int raw) {
  raw = constrain(raw, 0, 4095);
  float voltage = raw * (3.3f / 4095.0f);
  return constrain(voltage * 200.0f, 0.0f, 1000.0f);
}

uint16_t readLe16(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

int16_t ld2450DecodeSigned(uint16_t raw) {
  int16_t magnitude = raw & 0x7FFF;
  return (raw & 0x8000) ? magnitude : -magnitude;
}

bool isLd2450DataFrame(const uint8_t* frame) {
  return frame[0] == 0xAA && frame[1] == 0xFF && frame[2] == 0x03 && frame[3] == 0x00
      && frame[28] == 0x55 && frame[29] == 0xCC;
}

void diagnoseLd2450Line(int8_t pin) {
  pinMode(pin, INPUT_PULLDOWN);
  delay(20);
  int idleWithPulldown = digitalRead(pin);
  int analogRaw = analogRead(pin);

  pinMode(pin, INPUT_PULLDOWN);
  delay(2);
  int previous = digitalRead(pin);
  uint32_t edges = 0;
  uint32_t highSamples = 0;
  uint32_t samples = 0;
  uint32_t startUs = micros();

  while ((uint32_t)(micros() - startUs) < 500000UL) {
    int current = digitalRead(pin);
    if (current != previous) {
      edges++;
      previous = current;
    }
    highSamples += current != 0;
    samples++;
  }

  float highPct = samples > 0 ? (100.0f * highSamples / samples) : 0.0f;
  Serial.printf("LD2450-LINE: GPIO%d idle_pd=%d adc=%d edges=%lu high=%.1f%%/500ms\n",
                pin, idleWithPulldown, analogRaw, (unsigned long)edges, highPct);
}

void markRadarStateDirty() {
  radarSnapshotDirty = true;
  dataJsonCacheTime = millis() - DATA_CACHE_MS;
}

void clearRadarTargets() {
  bool stateChanged = presenceDetected || movingDetected || stationaryDetected
                   || radarTargetCount > 0 || radarMovingCount > 0 || radarStillCount > 0;
  for (uint8_t i = 0; i < LD2450_TARGETS; i++) {
    stateChanged = stateChanged || radarTargets[i].valid;
    radarTargets[i] = {};
    radarTargetLastSeen[i] = 0;
  }
  radarTargetCount = 0;
  radarMovingCount = 0;
  radarStillCount = 0;
  presenceDetected = false;
  movingDetected = false;
  stationaryDetected = false;
  movingDistance = 0;
  stationaryDistance = 0;
  if (stateChanged) markRadarStateDirty();
}

void updatePresenceFromTargets() {
  bool previousPresence = presenceDetected;
  uint8_t previousTargetCount = radarTargetCount;

  radarTargetCount = 0;
  radarMovingCount = 0;
  radarStillCount = 0;
  movingDistance = 0;
  stationaryDistance = 0;
  uint16_t nearestMoving = 0xFFFF;
  uint16_t nearestStill = 0xFFFF;

  for (uint8_t i = 0; i < LD2450_TARGETS; i++) {
    if (!radarTargets[i].valid) continue;
    radarTargetCount++;
    uint16_t distanceCm = radarTargets[i].distanceMm / 10;
    if (abs(radarTargets[i].speedCms) > 3) {
      radarMovingCount++;
      if (distanceCm < nearestMoving) {
        nearestMoving = distanceCm;
        movingDistance = distanceCm;
      }
    } else {
      radarStillCount++;
      if (distanceCm < nearestStill) {
        nearestStill = distanceCm;
        stationaryDistance = distanceCm;
      }
    }
  }

  presenceDetected = radarTargetCount > 0;
  movingDetected = radarMovingCount > 0;
  stationaryDetected = radarStillCount > 0;

  if (previousPresence != presenceDetected || previousTargetCount != radarTargetCount) {
    markRadarStateDirty();
  }
}

void updateRadarTrack(uint8_t trackIndex, const RadarTarget& measurement,
                      unsigned long frameTime, bool smooth) {
  RadarTarget& track = radarTargets[trackIndex];
  if (smooth && track.valid) {
    constexpr float newWeight = 0.45f;
    track.xMm = (int16_t)lroundf(track.xMm * (1.0f - newWeight)
                               + measurement.xMm * newWeight);
    track.yMm = (int16_t)lroundf(track.yMm * (1.0f - newWeight)
                               + measurement.yMm * newWeight);
    track.speedCms = measurement.speedCms;
    track.resolutionMm = measurement.resolutionMm;
    track.sourceSlot = measurement.sourceSlot;
  } else {
    track = measurement;
  }

  track.valid = true;
  track.fresh = true;
  track.distanceMm = (uint16_t)sqrtf(
    (float)track.xMm * track.xMm + (float)track.yMm * track.yMm
  );
  track.angleDeg = atan2f((float)track.xMm, (float)track.yMm) * 180.0f / PI;
  radarTargetLastSeen[trackIndex] = frameTime;
}

bool parseLd2450Frame(const uint8_t* frame, unsigned long frameTime) {
  if (!isLd2450DataFrame(frame)) return false;

  RadarTarget measurements[LD2450_TARGETS] = {};
  bool measurementUsed[LD2450_TARGETS] = {};
  bool trackMatched[LD2450_TARGETS] = {};

  // Les emplacements T1/T2/T3 du protocole ne sont pas des identifiants de
  // personnes permanents. On decode d'abord les mesures brutes, puis on les
  // associe aux pistes existantes par proximite.
  for (uint8_t i = 0; i < LD2450_TARGETS; i++) {
    uint8_t off = 4 + i * 8;
    uint16_t rawX = readLe16(frame + off);
    uint16_t rawY = readLe16(frame + off + 2);
    uint16_t rawSpeed = readLe16(frame + off + 4);
    uint16_t resolution = readLe16(frame + off + 6);
    if (rawX == 0 && rawY == 0 && rawSpeed == 0 && resolution == 0) continue;

    RadarTarget& measurement = measurements[i];
    measurement.valid = true;
    measurement.fresh = true;
    measurement.sourceSlot = i + 1;
    measurement.xMm = ld2450DecodeSigned(rawX);
    measurement.yMm = ld2450DecodeSigned(rawY);
    measurement.speedCms = ld2450DecodeSigned(rawSpeed);
    measurement.resolutionMm = resolution;
    measurement.distanceMm = (uint16_t)sqrtf(
      (float)measurement.xMm * measurement.xMm
      + (float)measurement.yMm * measurement.yMm
    );
    measurement.angleDeg = atan2f((float)measurement.xMm,
                                  (float)measurement.yMm) * 180.0f / PI;
  }

  for (uint8_t i = 0; i < LD2450_TARGETS; i++) {
    radarTargets[i].fresh = false;
    if (radarTargets[i].valid
        && frameTime - radarTargetLastSeen[i] >= LD2450_TARGET_HOLD_MS) {
      radarTargets[i] = {};
      radarTargetLastSeen[i] = 0;
    }
  }

  // Association globale gloutonne du couple piste/mesure le plus proche.
  // A 10 Hz, un saut superieur a 1,2 m correspond presque toujours a une
  // reattribution d'emplacement par le radar et non au mouvement d'une personne.
  const int64_t maxMatchSq = (int64_t)LD2450_TRACK_MATCH_MM * LD2450_TRACK_MATCH_MM;
  while (true) {
    int8_t bestTrack = -1;
    int8_t bestMeasurement = -1;
    int64_t bestDistanceSq = maxMatchSq + 1;

    for (uint8_t track = 0; track < LD2450_TARGETS; track++) {
      if (!radarTargets[track].valid || trackMatched[track]) continue;
      for (uint8_t measurement = 0; measurement < LD2450_TARGETS; measurement++) {
        if (!measurements[measurement].valid || measurementUsed[measurement]) continue;
        int32_t dx = (int32_t)measurements[measurement].xMm - radarTargets[track].xMm;
        int32_t dy = (int32_t)measurements[measurement].yMm - radarTargets[track].yMm;
        int64_t distanceSq = (int64_t)dx * dx + (int64_t)dy * dy;
        if (distanceSq <= maxMatchSq && distanceSq < bestDistanceSq) {
          bestDistanceSq = distanceSq;
          bestTrack = track;
          bestMeasurement = measurement;
        }
      }
    }

    if (bestTrack < 0) break;
    updateRadarTrack(bestTrack, measurements[bestMeasurement], frameTime, true);
    trackMatched[bestTrack] = true;
    measurementUsed[bestMeasurement] = true;
  }

  // Une mesure sans correspondance est une nouvelle personne. Utiliser d'abord
  // une piste libre evite qu'une deuxieme personne ou un point aberrant fasse
  // teleporter une piste encore active.
  for (uint8_t measurement = 0; measurement < LD2450_TARGETS; measurement++) {
    if (!measurements[measurement].valid || measurementUsed[measurement]) continue;

    int8_t destination = -1;
    for (uint8_t track = 0; track < LD2450_TARGETS; track++) {
      if (!radarTargets[track].valid) {
        destination = track;
        break;
      }
    }

    // Si les trois pistes sont occupees, ignorer ce point isole. Une piste non
    // revue sera liberee apres LD2450_TARGET_HOLD_MS, puis la nouvelle mesure
    // pourra l'utiliser. Ainsi un seul outlier ne deplace jamais un point actif.
    if (destination >= 0) {
      updateRadarTrack(destination, measurements[measurement], frameTime, false);
      trackMatched[destination] = true;
      measurementUsed[measurement] = true;
    }
  }

  bool recoveredFromStale = ld2450StreamStale;
  lastRadarFrame = frameTime;
  ld2450StreamStale = false;
  updatePresenceFromTargets();
  radarSnapshotDirty = true;
  if (recoveredFromStale) markRadarStateDirty();
  return true;
}

void rememberLd2450Frame(const uint8_t* frame) {
  memcpy(ld2450LastFrame, frame, LD2450_FRAME_SIZE);
  ld2450HasLastFrame = true;
}

int64_t ld2450LastFrameAgeMs() {
  if (ld2450FramesValid == 0) return -1;
  return (uint32_t)(millis() - lastRadarFrame);
}

String ld2450StatusText() {
  int64_t age = ld2450LastFrameAgeMs();
  if (ld2450FramesValid > 0 && ld2450StreamStale) return "STALE";
  if (ld2450FramesValid > 0 && age >= 0 && age <= LD2450_STALE_MS) return "OK";
  if (ld2450FramesValid > 0) return "STALE";
  if (ld2450BytesRx == 0) return "NO_BYTES";
  if (ld2450FramesInvalid > 0) return "BAD_FRAME";
  return "NO_FRAME";
}

const char* ld2450TrackingModeText() {
  if (ld2450TrackingMode == 2) return "MULTI";
  if (ld2450TrackingMode == 1) return "MONO";
  return "UNKNOWN";
}

String ld2450LastFrameHex() {
  if (!ld2450HasLastFrame) return "";
  const char hex[] = "0123456789ABCDEF";
  String out;
  out.reserve(LD2450_FRAME_SIZE * 2);
  for (uint8_t i = 0; i < LD2450_FRAME_SIZE; i++) {
    out += hex[(ld2450LastFrame[i] >> 4) & 0x0F];
    out += hex[ld2450LastFrame[i] & 0x0F];
  }
  return out;
}

void flushLd2450Input() {
  while (LD2450_Serial.available()) LD2450_Serial.read();
}

bool waitLd2450Ack(uint8_t expectedCommand, uint8_t* response,
                   uint16_t& responseLength, uint32_t timeoutMs) {
  static const uint8_t header[4] = {0xFD, 0xFC, 0xFB, 0xFA};
  uint8_t frame[48] = {};
  uint8_t idx = 0;
  uint8_t totalLength = 0;
  unsigned long start = millis();
  responseLength = 0;

  while (millis() - start < timeoutMs) {
    while (LD2450_Serial.available()) {
      uint8_t b = LD2450_Serial.read();

      if (idx < 4) {
        if (b == header[idx]) {
          frame[idx++] = b;
        } else {
          idx = (b == header[0]) ? 1 : 0;
          if (idx == 1) frame[0] = b;
        }
        continue;
      }

      if (idx >= sizeof(frame)) {
        idx = 0;
        totalLength = 0;
        continue;
      }
      frame[idx++] = b;

      if (idx == 6) {
        uint16_t payloadLength = readLe16(frame + 4);
        uint16_t fullLength = 4 + 2 + payloadLength + 4;
        if (payloadLength < 4 || fullLength > sizeof(frame)) {
          idx = 0;
          totalLength = 0;
          continue;
        }
        totalLength = (uint8_t)fullLength;
      }

      if (totalLength > 0 && idx == totalLength) {
        uint16_t payloadLength = readLe16(frame + 4);
        bool footerOk = frame[totalLength - 4] == 0x04
                     && frame[totalLength - 3] == 0x03
                     && frame[totalLength - 2] == 0x02
                     && frame[totalLength - 1] == 0x01;
        bool commandMatches = frame[6] == expectedCommand && frame[7] == 0x01;
        bool statusOk = frame[8] == 0x00 && frame[9] == 0x00;
        if (footerOk && commandMatches) {
          responseLength = payloadLength;
          if (response != nullptr) memcpy(response, frame + 6, payloadLength);
          return statusOk;
        }
        idx = 0;
        totalLength = 0;
      }
    }
    delay(1);
    esp_task_wdt_reset();
  }
  return false;
}

bool sendLd2450Command(uint8_t command, const uint8_t* value, uint8_t valueLength,
                       uint8_t* response, uint16_t& responseLength) {
  static const uint8_t header[4] = {0xFD, 0xFC, 0xFB, 0xFA};
  static const uint8_t footer[4] = {0x04, 0x03, 0x02, 0x01};
  if (ld2450TxPin < 0) return false;

  flushLd2450Input();
  uint16_t payloadLength = 2 + valueLength;
  LD2450_Serial.write(header, sizeof(header));
  LD2450_Serial.write((uint8_t)(payloadLength & 0xFF));
  LD2450_Serial.write((uint8_t)(payloadLength >> 8));
  LD2450_Serial.write(command);
  LD2450_Serial.write((uint8_t)0x00);
  if (value != nullptr && valueLength > 0) LD2450_Serial.write(value, valueLength);
  LD2450_Serial.write(footer, sizeof(footer));
  LD2450_Serial.flush();

  return waitLd2450Ack(command, response, responseLength,
                       LD2450_COMMAND_TIMEOUT_MS);
}

bool configureLd2450MultiTarget() {
  ld2450ConfigOk = false;
  ld2450TrackingMode = 0;
  if (ld2450TxPin < 0) {
    Serial.println("LD2450-CONFIG: TX indisponible, mode multi non verifiable");
    return false;
  }

  uint8_t response[32] = {};
  uint16_t responseLength = 0;
  const uint8_t enableValue[2] = {0x01, 0x00};
  bool entered = sendLd2450Command(0xFF, enableValue, sizeof(enableValue),
                                   response, responseLength);
  bool queryOk = false;
  bool setOk = true;
  bool ended = false;

  if (entered) {
    queryOk = sendLd2450Command(0x91, nullptr, 0, response, responseLength);
    if (queryOk && responseLength >= 6) {
      ld2450TrackingMode = (uint8_t)readLe16(response + 4);
    }

    // Un ancien reglage Bluetooth peut avoir laisse le module en mono-cible.
    // Le mode est persistant dans le radar, donc on n'ecrit que si necessaire.
    if (ld2450TrackingMode != 2) {
      setOk = sendLd2450Command(0x90, nullptr, 0, response, responseLength);
      if (setOk) {
        ld2450TrackingMode = 2;  // confirme au minimum par l'ACK 0x90
        queryOk = sendLd2450Command(0x91, nullptr, 0, response, responseLength);
        if (queryOk && responseLength >= 6) {
          ld2450TrackingMode = (uint8_t)readLe16(response + 4);
        }
      }
    }

    ended = sendLd2450Command(0xFE, nullptr, 0, response, responseLength);
  }

  ld2450ConfigOk = entered && setOk && ended && ld2450TrackingMode == 2;
  Serial.printf("LD2450-CONFIG: enter=%s query=%s set=%s exit=%s mode=%s\n",
                entered ? "OK" : "ERR", queryOk ? "OK" : "ERR",
                setOk ? "OK" : "ERR", ended ? "OK" : "ERR",
                ld2450TrackingMode == 2 ? "MULTI" :
                (ld2450TrackingMode == 1 ? "MONO" : "INCONNU"));

  delay(80);
  flushLd2450Input();
  return ld2450ConfigOk;
}

bool collectLd2450Frame(uint32_t timeoutMs, uint8_t* outFrame,
                        uint32_t& bytesSeen, uint32_t& invalidSeen) {
  static const uint8_t header[4] = {0xAA, 0xFF, 0x03, 0x00};
  uint8_t frame[LD2450_FRAME_SIZE];
  uint8_t idx = 0;
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    while (LD2450_Serial.available()) {
      uint8_t b = LD2450_Serial.read();
      bytesSeen++;
      ld2450BytesRx++;
      ld2450LastByteMs = millis();

      if (idx < 4) {
        if (b == header[idx]) {
          frame[idx++] = b;
        } else {
          idx = (b == header[0]) ? 1 : 0;
          if (idx == 1) frame[0] = b;
        }
        continue;
      }

      frame[idx++] = b;
      if (idx == LD2450_FRAME_SIZE) {
        rememberLd2450Frame(frame);
        if (isLd2450DataFrame(frame)) {
          memcpy(outFrame, frame, LD2450_FRAME_SIZE);
          return true;
        }
        invalidSeen++;
        ld2450FramesInvalid++;
        idx = 0;
      }
    }
    delay(1);
    esp_task_wdt_reset();
  }

  return false;
}

bool autoDetectLd2450Baud() {
  const uint32_t candidates[] = {256000, 115200, 230400, 460800, 9600, 19200, 38400, 57600};
  uint8_t frame[LD2450_FRAME_SIZE];

  clearRadarTargets();
  ld2450BytesRx = 0;
  ld2450FramesValid = 0;
  ld2450FramesInvalid = 0;
  ld2450LastByteMs = 0;
  ld2450HasLastFrame = false;
  lastRadarFrame = 0;
  ld2450StreamStale = false;

  Serial.printf("LD2450: scan UART RX=GPIO%d TX=GPIO%d\n", LD2450_RX, LD2450_TX);

  // Les broches sont fixes pour eviter qu'un scan de production ne prenne le
  // controle du TTP223 ou d'un autre peripherique. Seul le debit est detecte.
  for (uint8_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
    uint32_t baud = candidates[i];
    uint32_t bytesSeen = 0;
    uint32_t invalidSeen = 0;

    LD2450_Serial.end();
    delay(60);
    ld2450RxPin = LD2450_RX;
    ld2450TxPin = LD2450_TX;
    LD2450_Serial.begin(baud, SERIAL_8N1, ld2450RxPin, ld2450TxPin);
    delay(120);
    flushLd2450Input();

    bool found = collectLd2450Frame(LD2450_BOOT_SCAN_MS, frame, bytesSeen, invalidSeen);
    Serial.printf("LD2450: RX=GPIO%d TX=GPIO%d %lu bauds -> bytes=%lu invalid=%lu valid=%s\n",
                  ld2450RxPin, ld2450TxPin, (unsigned long)baud,
                  (unsigned long)bytesSeen, (unsigned long)invalidSeen,
                  found ? "oui" : "non");

    if (found && parseLd2450Frame(frame, millis())) {
      ld2450Baud = baud;
      // Les compteurs de diagnostic commencent sur la liaison retenue,
      // sans inclure le bruit des essais precedents.
      ld2450BytesRx = bytesSeen;
      ld2450FramesInvalid = invalidSeen;
      ld2450FramesValid = 1;
      Serial.printf("LD2450: UART detecte RX=GPIO%d TX=GPIO%d baud=%lu targets=%u\n",
                    ld2450RxPin, ld2450TxPin,
                    (unsigned long)ld2450Baud, radarTargetCount);
      return true;
    }
  }

  LD2450_Serial.end();
  delay(60);
  // Sans trame valide, ne jamais verrouiller une entree bruitee: revenir au cablage documente.
  ld2450RxPin = LD2450_RX;
  ld2450TxPin = LD2450_TX;
  ld2450Baud = LD2450_DEFAULT_BAUD;
  LD2450_Serial.begin(ld2450Baud, SERIAL_8N1, ld2450RxPin, ld2450TxPin);
  clearRadarTargets();
  Serial.printf("LD2450: aucune trame valide. status=%s RX=GPIO%d TX=GPIO%d bytes=%lu invalid=%lu\n",
                ld2450StatusText().c_str(),
                ld2450RxPin, ld2450TxPin,
                (unsigned long)ld2450BytesRx,
                (unsigned long)ld2450FramesInvalid);
  return false;
}

void readLd2450() {
  static const uint8_t header[4] = {0xAA, 0xFF, 0x03, 0x00};
  static uint8_t frame[LD2450_FRAME_SIZE];
  static uint8_t idx = 0;

  while (LD2450_Serial.available()) {
    uint8_t b = LD2450_Serial.read();
    ld2450BytesRx++;
    ld2450LastByteMs = millis();
    if (idx < 4) {
      if (b == header[idx]) {
        frame[idx++] = b;
      } else {
        idx = (b == header[0]) ? 1 : 0;
        if (idx == 1) frame[0] = b;
      }
      continue;
    }

    frame[idx++] = b;
    if (idx == LD2450_FRAME_SIZE) {
      rememberLd2450Frame(frame);
      if (parseLd2450Frame(frame, millis())) {
        ld2450FramesValid++;
      } else {
        ld2450FramesInvalid++;
      }
      idx = 0;
    }
  }

  // Certains modules mettent plus de temps a emettre leur premiere trame que
  // la fenetre de detection du demarrage. Des qu'une liaison valide apparait,
  // retenter une fois la configuration MULTI sans attendre un redemarrage.
  if (!ld2450ConfigOk && !ld2450DeferredConfigAttempted && ld2450FramesValid > 0) {
    ld2450DeferredConfigAttempted = true;
    idx = 0;  // configureLd2450MultiTarget vide aussi tout fragment UART restant.
    Serial.println("LD2450-CONFIG: tentative differee apres premiere trame valide");
    configureLd2450MultiTarget();
    markRadarStateDirty();
  }

  const unsigned long radarNow = millis();
  if (ld2450FramesValid > 0 && (radarNow - lastRadarFrame > LD2450_STALE_MS)) {
    if (!ld2450StreamStale) {
      ld2450StreamStale = true;
      markRadarStateDirty();
    }
    if (presenceDetected || movingDetected || stationaryDetected || radarTargetCount > 0) {
      clearRadarTargets();
    }
  }
}

String ld2450TargetsJson() {
  String json = "[";
  unsigned long now = millis();
  for (uint8_t i = 0; i < LD2450_TARGETS; i++) {
    if (i > 0) json += ",";
    json += "{";
    json += "\"id\":"       + String(i + 1) + ",";
    json += "\"valid\":"    + String(radarTargets[i].valid ? "true" : "false") + ",";
    json += "\"fresh\":"    + String(radarTargets[i].fresh ? "true" : "false") + ",";
    json += "\"sourceSlot\":" + String(radarTargets[i].sourceSlot) + ",";
    json += "\"ageMs\":"    + String(radarTargets[i].valid
                                          ? (long)(now - radarTargetLastSeen[i]) : -1L) + ",";
    json += "\"x\":"        + String(radarTargets[i].xMm) + ",";
    json += "\"y\":"        + String(radarTargets[i].yMm) + ",";
    json += "\"speed\":"    + String(radarTargets[i].speedCms) + ",";
    json += "\"distance\":" + String(radarTargets[i].distanceMm / 10.0f, 1) + ",";
    json += "\"angle\":"    + String(radarTargets[i].angleDeg, 1) + ",";
    json += "\"resolution\":" + String(radarTargets[i].resolutionMm);
    json += "}";
  }
  json += "]";
  return json;
}

String ld2450RadarJson() {
  String json;
  json.reserve(1200);
  json = "{";
  json += "\"presence\":" + String(presenceDetected ? "true" : "false") + ",";
  json += "\"presenceMoving\":" + String(movingDetected ? "true" : "false") + ",";
  json += "\"presenceStatic\":" + String(stationaryDetected ? "true" : "false") + ",";
  json += "\"movingDist\":" + String(movingDistance) + ",";
  json += "\"staticDist\":" + String(stationaryDistance) + ",";
  json += "\"ld2450TargetCount\":" + String(radarTargetCount) + ",";
  json += "\"ld2450MovingCount\":" + String(radarMovingCount) + ",";
  json += "\"ld2450StillCount\":" + String(radarStillCount) + ",";
  json += "\"ld2450Targets\":" + ld2450TargetsJson() + ",";
  json += "\"ld2450Status\":\"" + ld2450StatusText() + "\",";
  json += "\"ld2450TrackingMode\":\"" + String(ld2450TrackingModeText()) + "\",";
  json += "\"ld2450ConfigOk\":" + String(ld2450ConfigOk ? "true" : "false") + ",";
  json += "\"ld2450Baud\":" + String(ld2450Baud) + ",";
  json += "\"ld2450RxPin\":" + String(ld2450RxPin) + ",";
  json += "\"ld2450TxPin\":" + String(ld2450TxPin) + ",";
  json += "\"ld2450BytesRx\":" + String(ld2450BytesRx) + ",";
  json += "\"ld2450FramesValid\":" + String(ld2450FramesValid) + ",";
  json += "\"ld2450FramesInvalid\":" + String(ld2450FramesInvalid) + ",";
  json += "\"ld2450LastFrameAge\":" + String(ld2450LastFrameAgeMs());
  json += "}";
  return json;
}

float readUvIndex() {
  int raw = analogRead(UV_PIN);
  float voltage = raw * (3.3f / 4095.0f);
  return constrain(voltage / 0.1f, 0.0f, 15.0f);
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
  alarmEnabled    = prefs.getBool("alarmOn", true);
  awayMode        = prefs.getBool("awayOn", false);
  prefs.end();
  Serial.printf("Prefs chargees : gas=%d, alarm=%d, absent=%d\n",
                gasThresholdPct, alarmEnabled, awayMode);
}

void saveSettings() {
  prefs.begin("station", false);
  prefs.putInt("gasTh",  gasThresholdPct);
  prefs.putBool("alarmOn", alarmEnabled);
  prefs.putBool("awayOn", awayMode);
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
void publishAlarmControlStates() {
  if (!mqtt.connected()) return;
  mqtt.publish(TOPIC_ALARM_STATE, alarmEnabled ? "ON" : "OFF", true);
  mqtt.publish(TOPIC_BASE "/away", awayMode ? "ON" : "OFF", true);
  mqtt.publish(TOPIC_BASE "/alarm", isAlarmActive ? "ON" : "OFF", true);
}

void markControlStateDirty() {
  dataJsonCache = "";
  dataJsonCacheTime = 0;
  tftNeedsRedraw = true;
}

void setAlarmEnabled(bool enabled) {
  bool changed = alarmEnabled != enabled;
  alarmEnabled = enabled;
  if (!enabled) {
    changed = changed || awayMode || isAlarmActive || alarmConfirmCount > 0;
    awayMode = false;
    isAlarmActive = false;
    alarmConfirmCount = 0;
    stopBuzzer();
  }
  if (changed) {
    saveSettings();
    markControlStateDirty();
  }
  publishAlarmControlStates();
}

void setAwayMode(bool enabled) {
  bool changed = awayMode != enabled;
  awayMode = enabled;
  if (enabled && !alarmEnabled) {
    alarmEnabled = true;
    changed = true;
  }
  if (changed) {
    saveSettings();
    markControlStateDirty();
  }
  publishAlarmControlStates();
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();
  msg.toUpperCase();

  if (String(topic) == TOPIC_ALARM_CMD) {
    if (msg == "ON") setAlarmEnabled(true);
    else if (msg == "OFF") setAlarmEnabled(false);
  } else if (String(topic) == TOPIC_AWAY_CMD) {
    if (msg == "ON") setAwayMode(true);
    else if (msg == "OFF") setAwayMode(false);
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

    { "homeassistant/sensor/esp32_station_pressure/config",
      "{\"name\":\"Pression\",\"stat_t\":\"" TOPIC_BASE "/pressure\","
      "\"unit_of_meas\":\"hPa\",\"dev_cla\":\"pressure\",\"stat_cla\":\"measurement\","
      "\"uniq_id\":\"esp32_station_pressure\"" + dev + "}" },

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

    { "homeassistant/binary_sensor/esp32_station_presence/config",
      "{\"name\":\"Presence\",\"stat_t\":\"" TOPIC_BASE "/presence\","
      "\"dev_cla\":\"presence\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\","
      "\"uniq_id\":\"esp32_station_presence\"" + dev + "}" },

    { "homeassistant/switch/esp32_station_alarm_enabled/config",
      "{\"name\":\"Activation alarme\",\"stat_t\":\"" TOPIC_ALARM_STATE "\","
      "\"cmd_t\":\"" TOPIC_ALARM_CMD "\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\","
      "\"optimistic\":false,\"icon\":\"mdi:shield-lock\","
      "\"uniq_id\":\"esp32_station_alarm_enabled\"" + dev + "}" },

    { "homeassistant/switch/esp32_station_away/config",
      "{\"name\":\"Mode absent\",\"stat_t\":\"" TOPIC_BASE "/away\","
      "\"cmd_t\":\"" TOPIC_AWAY_CMD "\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\","
      "\"uniq_id\":\"esp32_station_away\"" + dev + "}" },

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
      "{\"name\":\"Distance Cible lente\",\"stat_t\":\"" TOPIC_BASE "/static_dist\","
      "\"unit_of_meas\":\"cm\",\"icon\":\"mdi:human-male\","
      "\"uniq_id\":\"esp32_station_statdist\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_ld2450_targets/config",
      "{\"name\":\"LD2450 Cibles\",\"stat_t\":\"" TOPIC_BASE "/ld2450/target_count\","
      "\"icon\":\"mdi:radar\",\"uniq_id\":\"esp32_station_ld2450_targets\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_ld2450_moving/config",
      "{\"name\":\"LD2450 Cibles mouvement\",\"stat_t\":\"" TOPIC_BASE "/ld2450/moving_count\","
      "\"icon\":\"mdi:run\",\"uniq_id\":\"esp32_station_ld2450_moving\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_ld2450_still/config",
      "{\"name\":\"LD2450 Cibles lentes/fixes\",\"stat_t\":\"" TOPIC_BASE "/ld2450/still_count\","
      "\"icon\":\"mdi:human-male\",\"uniq_id\":\"esp32_station_ld2450_still\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_ld2450_status/config",
      "{\"name\":\"LD2450 Diagnostic\",\"stat_t\":\"" TOPIC_BASE "/ld2450/status\","
      "\"icon\":\"mdi:radar\",\"entity_category\":\"diagnostic\","
      "\"uniq_id\":\"esp32_station_ld2450_status\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_ld2450_tracking_mode/config",
      "{\"name\":\"LD2450 Mode suivi\",\"stat_t\":\"" TOPIC_BASE "/ld2450/tracking_mode\","
      "\"icon\":\"mdi:account-multiple\",\"entity_category\":\"diagnostic\","
      "\"uniq_id\":\"esp32_station_ld2450_tracking_mode\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_ld2450_baud/config",
      "{\"name\":\"LD2450 Baudrate\",\"stat_t\":\"" TOPIC_BASE "/ld2450/baud\","
      "\"unit_of_meas\":\"baud\",\"icon\":\"mdi:serial-port\",\"entity_category\":\"diagnostic\","
      "\"uniq_id\":\"esp32_station_ld2450_baud\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_ld2450_rx_pin/config",
      "{\"name\":\"LD2450 Pin RX ESP32\",\"stat_t\":\"" TOPIC_BASE "/ld2450/rx_pin\","
      "\"icon\":\"mdi:serial-port\",\"entity_category\":\"diagnostic\","
      "\"uniq_id\":\"esp32_station_ld2450_rx_pin\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_ld2450_tx_pin/config",
      "{\"name\":\"LD2450 Pin TX ESP32\",\"stat_t\":\"" TOPIC_BASE "/ld2450/tx_pin\","
      "\"icon\":\"mdi:serial-port\",\"entity_category\":\"diagnostic\","
      "\"uniq_id\":\"esp32_station_ld2450_tx_pin\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_ld2450_bytes/config",
      "{\"name\":\"LD2450 Octets recus\",\"stat_t\":\"" TOPIC_BASE "/ld2450/bytes_rx\","
      "\"unit_of_meas\":\"B\",\"stat_cla\":\"total_increasing\",\"icon\":\"mdi:counter\","
      "\"entity_category\":\"diagnostic\",\"uniq_id\":\"esp32_station_ld2450_bytes\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_ld2450_valid/config",
      "{\"name\":\"LD2450 Trames valides\",\"stat_t\":\"" TOPIC_BASE "/ld2450/frames_valid\","
      "\"stat_cla\":\"total_increasing\",\"icon\":\"mdi:check-network\","
      "\"entity_category\":\"diagnostic\",\"uniq_id\":\"esp32_station_ld2450_valid\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_ld2450_invalid/config",
      "{\"name\":\"LD2450 Trames invalides\",\"stat_t\":\"" TOPIC_BASE "/ld2450/frames_invalid\","
      "\"stat_cla\":\"total_increasing\",\"icon\":\"mdi:alert-circle\","
      "\"entity_category\":\"diagnostic\",\"uniq_id\":\"esp32_station_ld2450_invalid\"" + dev + "}" },

    { "homeassistant/sensor/esp32_station_ld2450_age/config",
      "{\"name\":\"LD2450 Age derniere trame\",\"stat_t\":\"" TOPIC_BASE "/ld2450/last_frame_age\","
      "\"unit_of_meas\":\"ms\",\"icon\":\"mdi:timer-outline\",\"entity_category\":\"diagnostic\","
      "\"uniq_id\":\"esp32_station_ld2450_age\"" + dev + "}" },

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

  const char* fields[] = {"x", "y", "speed", "distance", "angle", "resolution"};
  const char* names[] = {"X", "Y", "Vitesse", "Distance", "Angle", "Resolution"};
  const char* units[] = {"mm", "mm", "cm/s", "cm", "\\u00b0", "mm"};
  const char* icons[] = {"mdi:axis-x-arrow", "mdi:axis-y-arrow", "mdi:speedometer",
                         "mdi:map-marker-distance", "mdi:angle-acute", "mdi:radar"};

  for (uint8_t target = 1; target <= LD2450_TARGETS; target++) {
    String validTopic = "homeassistant/binary_sensor/esp32_station_ld2450_t"
                      + String(target) + "_valid/config";
    String validPayload = "{\"name\":\"LD2450 T" + String(target)
                        + " active\",\"stat_t\":\"" TOPIC_BASE "/ld2450/target"
                        + String(target) + "/valid\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\""
                        + ",\"dev_cla\":\"presence\",\"uniq_id\":\"esp32_station_ld2450_t"
                        + String(target) + "_valid\"" + dev + "}";
    mqtt.publish(validTopic.c_str(), validPayload.c_str(), true);

    for (uint8_t field = 0; field < 6; field++) {
      String topic = "homeassistant/sensor/esp32_station_ld2450_t" + String(target)
                   + "_" + fields[field] + "/config";
      String payload = "{\"name\":\"LD2450 T" + String(target) + " " + names[field]
                     + "\",\"stat_t\":\"" TOPIC_BASE "/ld2450/target" + String(target)
                     + "/" + fields[field] + "\",\"unit_of_meas\":\"" + units[field]
                     + "\",\"icon\":\"" + icons[field]
                     + "\",\"uniq_id\":\"esp32_station_ld2450_t" + String(target)
                     + "_" + fields[field] + "\"" + dev + "}";
      mqtt.publish(topic.c_str(), payload.c_str(), true);
    }
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
    mqtt.subscribe(TOPIC_AWAY_CMD);
    publishDiscovery();
    publishAlarmControlStates();
    markRadarStateDirty();
    Serial.println("MQTT: connecte");
  } else {
    mqttConnected = false;
    Serial.printf("MQTT: echec rc=%d\n", mqtt.state());
  }
}

void publishRadarData() {
  if (!mqtt.connected()) return;

  mqtt.publish(TOPIC_BASE "/presence",    presenceDetected ? "ON" : "OFF", true);
  mqtt.publish(TOPIC_BASE "/moving_dist", String(movingDistance).c_str(), true);
  mqtt.publish(TOPIC_BASE "/static_dist", String(stationaryDistance).c_str(), true);
  mqtt.publish(TOPIC_BASE "/ld2450/target_count", String(radarTargetCount).c_str(), true);
  mqtt.publish(TOPIC_BASE "/ld2450/moving_count", String(radarMovingCount).c_str(), true);
  mqtt.publish(TOPIC_BASE "/ld2450/still_count",  String(radarStillCount).c_str(), true);
  mqtt.publish(TOPIC_BASE "/ld2450/targets", ld2450TargetsJson().c_str(), true);
  String radarStatus = ld2450StatusText();
  mqtt.publish(TOPIC_BASE "/ld2450/status", radarStatus.c_str(), true);
  mqtt.publish(TOPIC_BASE "/ld2450/tracking_mode", ld2450TrackingModeText(), true);
  mqtt.publish(TOPIC_BASE "/ld2450/baud", String(ld2450Baud).c_str(), true);
  mqtt.publish(TOPIC_BASE "/ld2450/rx_pin", String(ld2450RxPin).c_str(), true);
  mqtt.publish(TOPIC_BASE "/ld2450/tx_pin", String(ld2450TxPin).c_str(), true);
  mqtt.publish(TOPIC_BASE "/ld2450/bytes_rx", String(ld2450BytesRx).c_str(), true);
  mqtt.publish(TOPIC_BASE "/ld2450/frames_valid", String(ld2450FramesValid).c_str(), true);
  mqtt.publish(TOPIC_BASE "/ld2450/frames_invalid", String(ld2450FramesInvalid).c_str(), true);
  mqtt.publish(TOPIC_BASE "/ld2450/last_frame_age", String(ld2450LastFrameAgeMs()).c_str(), true);
  mqtt.publish(TOPIC_BASE "/ld2450/last_frame_hex", ld2450LastFrameHex().c_str(), true);

  for (uint8_t i = 0; i < LD2450_TARGETS; i++) {
    String base = String(TOPIC_BASE) + "/ld2450/target" + String(i + 1);
    mqtt.publish((base + "/valid").c_str(),      radarTargets[i].valid ? "ON" : "OFF", true);
    mqtt.publish((base + "/x").c_str(),          String(radarTargets[i].valid ? radarTargets[i].xMm : 0).c_str(), true);
    mqtt.publish((base + "/y").c_str(),          String(radarTargets[i].valid ? radarTargets[i].yMm : 0).c_str(), true);
    mqtt.publish((base + "/speed").c_str(),      String(radarTargets[i].valid ? radarTargets[i].speedCms : 0).c_str(), true);
    mqtt.publish((base + "/distance").c_str(),   String(radarTargets[i].valid ? radarTargets[i].distanceMm / 10.0f : 0.0f, 1).c_str(), true);
    mqtt.publish((base + "/angle").c_str(),      String(radarTargets[i].valid ? radarTargets[i].angleDeg : 0.0f, 1).c_str(), true);
    mqtt.publish((base + "/resolution").c_str(), String(radarTargets[i].valid ? radarTargets[i].resolutionMm : 0).c_str(), true);
  }

  radarSnapshotDirty = false;
  lastRadarMqttPublish = millis();
}

void publishSensorData() {
  if (!mqtt.connected()) return;
  mqtt.publish(TOPIC_BASE "/temperature", String(temperature, 1).c_str(), true);
  mqtt.publish(TOPIC_BASE "/humidity",    String(humidity, 1).c_str(),    true);
  mqtt.publish(TOPIC_BASE "/pressure",    String(pressureHpa, 1).c_str(), true);
  mqtt.publish(TOPIC_BASE "/gas",         String(map(gasValue, 0, 4095, 0, 100)).c_str(), true);
  mqtt.publish(TOPIC_BASE "/lux",         String(luxValue, 0).c_str(), true);
  mqtt.publish(TOPIC_BASE "/uv",          String(uvIndex, 1).c_str(),    true);
  publishAlarmControlStates();
  mqtt.publish(TOPIC_BASE "/heat_index",  String(heatIdx, 1).c_str(),    true);
  mqtt.publish(TOPIC_BASE "/rssi",        String(WiFi.RSSI()).c_str(),    true);
  publishRadarData();

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
#define BOOT_TOTAL_STEPS 6    // TFT, AHT/BMP, LD2450, SPIFFS, WiFi, MQTT

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
  tft.setCursor(170, 90);  tft.print("PRESSION");
  
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

  // Pression (size 2)
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(170, 102);
  tft.printf("%-6.1f hPa", pressureHpa);

  // Lux (size 2)
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(10, 156);
  tft.printf("%-6.0f ", luxValue);

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
  tft.setCursor(10,  36);  tft.print("RADAR LD2450");
  tft.setCursor(170, 36);  tft.print("WIFI RSSI");
  
  tft.setCursor(10,  90);  tft.print("IP");
  tft.setCursor(170, 90);  tft.print("MQTT");
  
  tft.setCursor(10,  144); tft.print("ALARME");
  tft.setCursor(170, 144); tft.print("UPTIME / RAM");

  tft.drawFastHLine(0, 196, 320, TFT_DARKGREY);
}

void tftUpdateValues2() {
  // Radar LD2450
  String radarStatus = ld2450StatusText();
  tft.setTextSize(2);
  tft.setCursor(10, 48);
  if (radarStatus != "OK") {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.printf("%-12s", radarStatus.c_str());
  } else if (movingDetected) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.printf("%u cible     ", radarTargetCount);
  } else if (stationaryDetected) {
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.printf("%u lente     ", radarStillCount);
  } else {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.print("Aucune       ");
  }
  tft.setTextSize(1);
  tft.setCursor(10, 72);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  int8_t displayTarget = -1;
  if (radarStatus == "OK") {
    uint8_t displayStart = (millis() / 1500) % LD2450_TARGETS;
    for (uint8_t offset = 0; offset < LD2450_TARGETS; offset++) {
      uint8_t i = (displayStart + offset) % LD2450_TARGETS;
      if (radarTargets[i].valid) {
        displayTarget = i;
        break;
      }
    }
  }
  if (displayTarget >= 0) {
    const RadarTarget& target = radarTargets[displayTarget];
    tft.printf("T%u %4.0fcm %4.0fdeg %4dcm/s ",
               displayTarget + 1,
               target.distanceMm / 10.0f,
               target.angleDeg,
               target.speedCms);
  } else if (radarStatus == "OK") {
    tft.print("Flux OK - aucune cible          ");
  } else {
    tft.printf("Diag %-8s RX%d %lu             ",
               radarStatus.c_str(), ld2450RxPin, (unsigned long)ld2450Baud);
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
  tft.setCursor(220, 148); tft.printf("Cur:%5.0f", luxValue);

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
    if (updated) saveSettings();   // persistance NVS
    req->send(updated ? 200 : 400, "text/plain", updated ? "OK" : "Parametre manquant");
  });

  server.on("/alarm", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    if (!req->hasParam("state")) { req->send(400, "text/plain", "Parametre manquant"); return; }
    String state = req->getParam("state")->value();
    state.toLowerCase();
    if (state == "on") {
      setAlarmEnabled(true);
      req->send(200, "text/plain", "ALARM_ON");
    } else if (state == "off") {
      setAlarmEnabled(false);
      req->send(200, "text/plain", "ALARM_OFF");
    } else {
      req->send(400, "text/plain", "Etat invalide");
    }
  });

  server.on("/away", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    if (!req->hasParam("state")) { req->send(400, "text/plain", "Parametre manquant"); return; }
    String state = req->getParam("state")->value();
    state.toLowerCase();
    if (state == "on") {
      setAwayMode(true);
      req->send(200, "text/plain", "AWAY_ON");
    } else if (state == "off") {
      setAwayMode(false);
      req->send(200, "text/plain", "AWAY_OFF");
    } else {
      req->send(400, "text/plain", "Etat invalide");
    }
  });

  server.on("/radar", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!checkAuth(req)) return;
    AsyncWebServerResponse* response = req->beginResponse(200, "application/json",
                                                          ld2450RadarJson());
    response->addHeader("Cache-Control", "no-store");
    req->send(response);
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

    String radarStatus = ld2450StatusText();
    String radarFrameHex = ld2450LastFrameHex();
    int64_t radarAge = ld2450LastFrameAgeMs();

    String json;
    json.reserve(2000);
    json = "{";
    json += "\"temp\":"           + String(temperature, 1)          + ",";
    json += "\"hum\":"            + String(humidity, 1)             + ",";
    json += "\"pressure\":"       + String(pressureHpa, 1)          + ",";
    json += "\"heatIndex\":"      + String(heatIdx, 1)              + ",";
    json += "\"lux\":"            + String(luxValue, 0)             + ",";
    json += "\"gasPct\":"         + String(map(gasValue, 0, 4095, 0, 100)) + ",";
    json += "\"uvIndex\":"        + String(uvIndex, 1)              + ",";
    json += "\"gasThreshold\":"   + String(gasThresholdPct)         + ",";
    json += "\"presence\":"       + String(presenceDetected   ? "true" : "false") + ",";
    json += "\"presenceMoving\":" + String(movingDetected     ? "true" : "false") + ",";
    json += "\"presenceStatic\":" + String(stationaryDetected ? "true" : "false") + ",";
    json += "\"movingDist\":"     + String(movingDistance)           + ",";
    json += "\"staticDist\":"     + String(stationaryDistance)       + ",";
    json += "\"ld2450TargetCount\":" + String(radarTargetCount)       + ",";
    json += "\"ld2450MovingCount\":" + String(radarMovingCount)       + ",";
    json += "\"ld2450StillCount\":"  + String(radarStillCount)        + ",";
    json += "\"ld2450Targets\":"     + ld2450TargetsJson()            + ",";
    json += "\"ld2450Status\":\""    + radarStatus                    + "\",";
    json += "\"ld2450TrackingMode\":\"" + String(ld2450TrackingModeText()) + "\",";
    json += "\"ld2450ConfigOk\":"   + String(ld2450ConfigOk ? "true" : "false") + ",";
    json += "\"ld2450Baud\":"        + String(ld2450Baud)             + ",";
    json += "\"ld2450RxPin\":"       + String(ld2450RxPin)            + ",";
    json += "\"ld2450TxPin\":"       + String(ld2450TxPin)            + ",";
    json += "\"ld2450BytesRx\":"     + String(ld2450BytesRx)          + ",";
    json += "\"ld2450FramesValid\":" + String(ld2450FramesValid)      + ",";
    json += "\"ld2450FramesInvalid\":" + String(ld2450FramesInvalid)  + ",";
    json += "\"ld2450LastFrameAge\":" + String(radarAge)              + ",";
    json += "\"ld2450LastFrameHex\":\"" + radarFrameHex               + "\",";
    json += "\"alarmEnabled\":"   + String(alarmEnabled  ? "true" : "false") + ",";
    json += "\"awayMode\":"       + String(awayMode      ? "true" : "false") + ",";
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

  // I2C climate module: AHT20 + BMP280
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  ahtOk = aht.begin(&Wire);
  bmpOk = bmp.begin(0x76);
  if (!bmpOk) bmpOk = bmp.begin(0x77);
  if (bmpOk) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  }
  Serial.printf("AHT20: %s  BMP280: %s  SDA=GPIO%d SCL=GPIO%d\n",
                ahtOk ? "OK" : "ERR", bmpOk ? "OK" : "ERR",
                I2C_SDA_PIN, I2C_SCL_PIN);
  bootStep((ahtOk || bmpOk) ? "AHT20/BMP280: OK" : "AHT20/BMP280: ERR", ahtOk || bmpOk);
  esp_task_wdt_reset();

  // HLK-LD2450 : GPIO13 recoit TX radar, GPIO14 pilote RX radar.
  diagnoseLd2450Line(LD2450_RX);
  diagnoseLd2450Line(LD2450_TX);
  size_t radarRxBufferSize = LD2450_Serial.setRxBufferSize(LD2450_RX_BUFFER_SIZE);
  Serial.printf("LD2450: buffer RX=%u octets\n", (unsigned)radarRxBufferSize);
  bool ld2450Ok = autoDetectLd2450Baud();
  bool ld2450MultiOk = ld2450Ok && configureLd2450MultiTarget();
  bootStep(ld2450Ok
           ? (ld2450MultiOk ? "Radar LD2450: MULTI" : "Radar RX OK / verifier TX")
           : "Radar LD2450: DIAG",
           ld2450Ok && ld2450MultiOk);
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
    readLd2450();
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
  readLd2450();
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
  wifiClient.setTimeout(1);
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(1024);  // augmente pour les payloads de discovery
  mqtt.setSocketTimeout(2);
  mqttReconnect();
  readLd2450();
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

  // --- AHT20 + BMP280 (toutes les 2s) ---
  static unsigned long lastEnvRead = 0;
  if (lastEnvRead == 0) lastEnvRead = now;
  if (now - lastEnvRead > 2000) {
    lastEnvRead = now;
    if (ahtOk) {
      sensors_event_t humEvent, tempEvent;
      aht.getEvent(&humEvent, &tempEvent);
      if (!isnan(tempEvent.temperature)) {
        tempBuf[envAvgIdx] = tempEvent.temperature;
        temperature = avgOf(tempBuf, AVG_SIZE, envAvgFilled, envAvgIdx + 1);
      }
      if (!isnan(humEvent.relative_humidity)) {
        humBuf[envAvgIdx] = humEvent.relative_humidity;
        humidity = avgOf(humBuf, AVG_SIZE, envAvgFilled, envAvgIdx + 1);
      }
    } else if (bmpOk) {
      float bmpTemp = bmp.readTemperature();
      if (!isnan(bmpTemp)) {
        tempBuf[envAvgIdx] = bmpTemp;
        temperature = avgOf(tempBuf, AVG_SIZE, envAvgFilled, envAvgIdx + 1);
      }
    }

    if (bmpOk) {
      float p = bmp.readPressure() / 100.0f;
      if (!isnan(p) && p > 300.0f && p < 1200.0f) {
        pressureBuf[envAvgIdx] = p;
        pressureHpa = avgOf(pressureBuf, AVG_SIZE, envAvgFilled, envAvgIdx + 1);
      }
    }

    heatIdx = computeHeatIndex(temperature, humidity);
    envAvgIdx = (envAvgIdx + 1) % AVG_SIZE;
    if (envAvgIdx == 0) envAvgFilled = true;
  }

  lightRaw = analogRead(LIGHT_PIN);
  float luxRaw = temt6000ToLux(lightRaw);
  luxValue = (luxValue <= 0.0f) ? luxRaw : (luxValue * 0.75f + luxRaw * 0.25f);
  uvIndex  = readUvIndex();

  // --- Gaz : echantillonnage rapide independant (toutes les 500ms) ---
  if (lastGasSample == 0) lastGasSample = now;
  if (now - lastGasSample > 500) {
    lastGasSample = now;
    int gasRaw = analogRead(GAS_PIN);
    gasBuf[gasAvgIdx] = gasRaw;
    gasValue = avgOf(gasBuf, AVG_SIZE, gasAvgFilled, gasAvgIdx + 1);

    gasAvgIdx = (gasAvgIdx + 1) % AVG_SIZE;
    if (gasAvgIdx == 0) gasAvgFilled = true;
  }

  // --- HLK-LD2450 ---
  readLd2450();

  // --- NTP + heure locale (DST automatique via POSIX TZ) ---
  timeClient.update();
  time_t epochUTC = time(nullptr);
  struct tm tinfo;
  localtime_r(&epochUTC, &tinfo);
  sprintf(currentTime, "%02d:%02d:%02d", tinfo.tm_hour, tinfo.tm_min, tinfo.tm_sec);
  sprintf(currentDate, "%02d/%02d/%04d", tinfo.tm_mday, tinfo.tm_mon + 1, tinfo.tm_year + 1900);

  // --- Alarme avec anti-rebond et warmup gaz ---
  int  gasPct       = map(gasValue, 0, 4095, 0, 100);
  bool gasWarmupOk  = (millis() / 1000) >= GAS_WARMUP_SEC;
  bool gasAlarm     = gasWarmupOk && (gasPct > gasThresholdPct);
  bool awayAlarm    = awayMode && presenceDetected;
  bool seuilDepasse = gasAlarm || awayAlarm;

  if (alarmEnabled && seuilDepasse) {
    if (alarmConfirmCount < ALARM_CONFIRM_COUNT) alarmConfirmCount++;
    if (alarmConfirmCount >= ALARM_CONFIRM_COUNT && !isAlarmActive) {
      isAlarmActive = true;
      markControlStateDirty();
      publishAlarmControlStates();
      Serial.println("ALARME DECLENCHEE !");
    }
  } else {
    alarmConfirmCount = 0;
  }

  if (isAlarmActive && alarmEnabled) {
    sirenUpdate(now);
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
  }
  mqttConnected = mqtt.connected();
  mqtt.loop();

  // Une reconnexion/publication MQTT peut bloquer: vider a nouveau l'UART ensuite.
  readLd2450();
  now = millis();

  if (now - lastMqttPublish > 5000) {
    lastMqttPublish = now;
    publishSensorData();
  } else if (radarSnapshotDirty && mqtt.connected()
             && (now - lastRadarMqttPublish >= LD2450_MQTT_SNAPSHOT_MS)) {
    publishRadarData();
  }

  // Les publications MQTT sont synchrones: recuperer aussitot les trames accumulees.
  readLd2450();
  now = millis();

  // --- Bouton tactile TTP223 ---
  bool touchState = (ld2450RxPin != TOUCH_PIN) && digitalRead(TOUCH_PIN);
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

  // La presence et l'alarme ne pilotent plus l'ecran. Seul un appui volontaire
  // sur le TTP223 le reveille ou prolonge son temps d'activite.

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
    unsigned long tftUpdateInterval = (tftPage == 1) ? 500 : 2000;
    if (now - lastTftUpdate > tftUpdateInterval) {
      lastTftUpdate = now;
      if      (tftPage == 0) tftUpdateValues1();
      else if (tftPage == 1) tftUpdateValues2();
      else                   tftUpdateValues3();

      int rawUV    = analogRead(UV_PIN);
      Serial.printf("[DEBUG] UV:  raw=%d  voltage=%.3fV  index=%.1f\n",
                    rawUV, rawUV * 3.3f / 4095.0f, uvIndex);
      Serial.printf("[DEBUG] LD2450: status=%s mode=%s config=%s RX=GPIO%d TX=GPIO%d baud=%lu bytes=%lu valid=%lu invalid=%lu targets=%u moving=%u slow=%u\n",
                    ld2450StatusText().c_str(),
                    ld2450TrackingModeText(), ld2450ConfigOk ? "OK" : "ERR",
                    ld2450RxPin, ld2450TxPin,
                    (unsigned long)ld2450Baud,
                    (unsigned long)ld2450BytesRx,
                    (unsigned long)ld2450FramesValid,
                    (unsigned long)ld2450FramesInvalid,
                    radarTargetCount, radarMovingCount, radarStillCount);
      for (uint8_t i = 0; i < LD2450_TARGETS; i++) {
        if (!radarTargets[i].valid) continue;
        Serial.printf("  T%u(slot%u age=%lums): x=%d y=%d speed=%d\n",
                      i + 1, radarTargets[i].sourceSlot,
                      (unsigned long)(now - radarTargetLastSeen[i]),
                      radarTargets[i].xMm, radarTargets[i].yMm,
                      radarTargets[i].speedCms);
      }
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
    tft.fillRect(0,   0,   320, 4,   c);
    tft.fillRect(0,   236, 320, 4,   c);
    tft.fillRect(0,   0,   4,   240, c);
    tft.fillRect(316, 0,   4,   240, c);
  } else if (!tftSleeping && !isAlarmActive && alarmFlashState) {
    alarmFlashState = false;
    tft.fillRect(0,   0,   320, 4,   TFT_BLACK);
    tft.fillRect(0,   236, 320, 4,   TFT_BLACK);
    tft.fillRect(0,   0,   4,   240, TFT_BLACK);
    tft.fillRect(316, 0,   4,   240, TFT_BLACK);
    tftNeedsRedraw = true;
  }

  // --- Echantillonnage historique pour la page graphiques (toutes les 10s) ---
  if (now - lastHistSample > 10000) {
    lastHistSample = now;
    tempHist[histIdx] = temperature;
    humHist[histIdx]  = humidity;
    luxHist[histIdx]  = (int)luxValue;
    histIdx = (histIdx + 1) % HIST_SIZE;
    if (histIdx == 0) histFilled = true;
  }

  // --- Watchdog : reset a chaque tour de loop ---
  esp_task_wdt_reset();

  delay(100);
}
