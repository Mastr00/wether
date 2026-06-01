<h1 align="center">ESP32-S3 Environmental Station</h1>

<p align="center">
  Connected indoor environmental monitor with on-device TFT UI, web dashboard,
  and Home Assistant integration over MQTT.
</p>

<p align="center">
  <img alt="Platform"   src="https://img.shields.io/badge/platform-ESP32--S3-E7352C?logo=espressif&logoColor=white">
  <img alt="Framework"  src="https://img.shields.io/badge/framework-Arduino-00979D?logo=arduino&logoColor=white">
  <img alt="Build tool" src="https://img.shields.io/badge/build-PlatformIO-F5822A?logo=platformio&logoColor=white">
  <img alt="Language"   src="https://img.shields.io/badge/language-C%2B%2B17-00599C?logo=cplusplus&logoColor=white">
  <img alt="MQTT"       src="https://img.shields.io/badge/protocol-MQTT-660066?logo=mqtt&logoColor=white">
  <img alt="License"    src="https://img.shields.io/badge/license-TBD-lightgrey">
</p>

## Overview

This project is a single-firmware environmental monitor built on the [Espressif ESP32-S3](https://www.espressif.com/en/products/socs/esp32-s3) using the [Arduino core for ESP32](https://github.com/espressif/arduino-esp32) and managed through [PlatformIO](https://platformio.org/). It reads ambient temperature, humidity, air-quality (gas), illuminance, ultraviolet index, sound pressure, and human presence, then exposes that data through three channels:

1. A local touch-driven [ST7789V](https://www.displayfuture.com/Display/datasheet/controller/ST7789V.pdf) TFT user interface with three pages and animated transitions,
2. An authenticated web dashboard served from on-chip [SPIFFS](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/storage/spiffs.html),
3. An [MQTT](https://mqtt.org/) feed implementing the [Home Assistant MQTT Discovery](https://www.home-assistant.io/integrations/mqtt/#mqtt-discovery) protocol so every sensor is auto-registered as a Home Assistant entity.

The firmware is designed for permanent 24/7 operation: it includes a task watchdog, a NVS-backed configuration store, a non-blocking siren alarm with hysteresis, automatic Wi-Fi recovery, OTA firmware updates, and a persistent crash log.

## Features

- **Sensors** — DHT11 (temperature, humidity), LDR (illuminance, mapped to lux), MQ gas sensor (% of full scale), GUVA-S12SD (UV index), KY-037 (sound pressure, dB-equivalent), LD2410C 24 GHz FMCW radar (moving / stationary presence with per-target distance).
- **Computed metrics** — Rothfusz heat index, RSSI signal bars, smoothed running averages, derived alarm state.
- **Display** — Three-page TFT (sensors / system / 10 min trend charts), animated weather glyphs reflecting current temperature and UV index, capacitive page cycling (TTP223), automatic sleep after 60 s with motion/touch/alarm wake.
- **Web dashboard** — Chart.js gauges and history graphs, real-time JSON polling, threshold tuning, alarm arm/disarm, crash log viewer, remote restart.
- **MQTT / Home Assistant** — 13 auto-discovered entities, last-will-and-testament availability topic, ISO-8601 `last_seen` timestamp, remote alarm control via `esp32/station/alarm/set`.
- **Reliability** — 30 s task watchdog, brown-out / panic / WDT reset logging to `/crashlog.txt`, 5 min Wi-Fi-dead reboot guard, debounced (3-sample) alarm trigger, 60 s MQ sensor warm-up window.
- **Operations** — OTA firmware updates (ArduinoOTA, port 3232), mDNS hostname (`station-meteo.local`), HTTP Basic Auth, POSIX time zone with automatic DST.

## Tech stack

| Layer            | Technology                                                                                                              |
|------------------|-------------------------------------------------------------------------------------------------------------------------|
| MCU              | [Espressif ESP32-S3-DevKitC-1](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/hw-reference/esp32s3/user-guide-devkitc-1.html) |
| Toolchain        | [PlatformIO Core](https://platformio.org/) with [`espressif32`](https://github.com/platformio/platform-espressif32) platform |
| Framework        | [Arduino core for ESP32](https://github.com/espressif/arduino-esp32)                                                    |
| Language         | C++17                                                                                                                   |
| Display driver   | [`bodmer/TFT_eSPI`](https://github.com/Bodmer/TFT_eSPI) `2.5.43`                                                        |
| Radar driver     | [`ncmreynolds/ld2410`](https://github.com/ncmreynolds/ld2410) `0.1.4`                                                  |
| Climate sensor   | [`adafruit/DHT sensor library`](https://github.com/adafruit/DHT-sensor-library) `1.4.6` + [`Adafruit Unified Sensor`](https://github.com/adafruit/Adafruit_Sensor) `1.1.15` |
| HTTP server      | [`ESPAsyncWebServer`](https://github.com/ESP32Async/ESPAsyncWebServer) `3.6.0` over [`AsyncTCP`](https://github.com/me-no-dev/AsyncTCP) `3.4.9` |
| MQTT client      | [`knolleary/PubSubClient`](https://github.com/knolleary/pubsubclient) `2.8`                                              |
| NTP client       | [`arduino-libraries/NTPClient`](https://github.com/arduino-libraries/NTPClient) `3.2.1`                                  |
| Persistence      | [`Preferences`](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/preferences.html) (NVS), [`SPIFFS`](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/storage/spiffs.html) |
| Front-end        | Vanilla HTML/CSS/JS, [Chart.js](https://www.chartjs.org/), served from SPIFFS                                            |
| Integration      | [Mosquitto](https://mosquitto.org/) broker, [Home Assistant](https://www.home-assistant.io/) MQTT Discovery              |

## Architecture

```
.
├── data/                  # SPIFFS image — uploaded with `pio run -t uploadfs`
│   ├── chart.js           # Chart.js bundle (offline copy)
│   ├── index.html         # Live dashboard (gauges, history, controls)
│   └── settings.html      # Threshold / calibration / maintenance UI
├── include/
│   └── User_Setup.h       # TFT_eSPI pin map and panel config (ST7789V, 240x320)
├── src/
│   └── main.cpp           # Firmware entry point (~1500 LOC, single TU)
├── lib/                   # Reserved for local libraries (empty)
├── test/                  # Reserved for PlatformIO unit tests (empty)
├── platformio.ini         # Two environments: USB flash + OTA flash
└── .gitignore
```

### Runtime data flow

```
   ┌───────────────────────┐
   │  Physical sensors     │
   │  DHT11 / LDR / MQ /   │
   │  GUVA / KY-037 /      │
   │  LD2410C / TTP223     │
   └──────────┬────────────┘
              │ ADC · GPIO · UART2 (256 kbaud)
              ▼
   ┌───────────────────────────────────────────────────────────────┐
   │                ESP32-S3 firmware (FreeRTOS)                   │
   │  ┌──────────────┐   ┌───────────────────┐   ┌──────────────┐  │
   │  │ Sampling +   │──►│ Alarm state       │──►│ Buzzer siren │  │
   │  │ smoothing    │   │ (debounced)       │   │ 1.5/3.0 kHz  │  │
   │  └──────┬───────┘   └─────────┬─────────┘   └──────────────┘  │
   │         │                     │                               │
   │  ┌──────▼───────┐   ┌─────────▼─────────┐   ┌──────────────┐  │
   │  │ TFT renderer │   │ MQTT publisher    │   │ HTTP server  │  │
   │  │ 3 pages +    │   │ Discovery + LWT   │   │ Basic Auth + │  │
   │  │ animations   │   │ + retained values │   │ JSON cache   │  │
   │  └──────────────┘   └─────────┬─────────┘   └──────┬───────┘  │
   └──────────────────────────────│────────────────────│───────────┘
                                  │ Wi-Fi STA          │
                                  ▼                    ▼
                       ┌──────────────────┐   ┌──────────────────┐
                       │ Mosquitto broker │   │  Web browser /   │
                       │ tcp/1883         │   │  Home Assistant  │
                       └────────┬─────────┘   │  Lovelace        │
                                ▼             └──────────────────┘
                       ┌──────────────────┐
                       │  Home Assistant  │
                       │  MQTT Discovery  │
                       └──────────────────┘
```

## Getting started

### Prerequisites

| Tool                  | Minimum version | Notes                                                |
|-----------------------|----------------:|------------------------------------------------------|
| [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation.html)       | `6.1`         | CLI; the VS Code extension also works                |
| Python                | `3.9`           | Bundled with PlatformIO installer                    |
| USB-Serial driver     | latest          | CP210x or CH343 depending on the DevKit revision     |
| MQTT broker           | any             | [Mosquitto](https://mosquitto.org/) `2.x` recommended |
| Home Assistant        | `2023.6+`       | MQTT integration with auto-discovery enabled         |

### Hardware wiring

| Peripheral             | ESP32-S3 GPIO                | Notes                                               |
|------------------------|------------------------------|-----------------------------------------------------|
| DHT11                  | `15`                         | Single-wire                                         |
| LDR (photoresistor)    | `4`                          | ADC1                                                |
| MQ gas sensor (analog) | `5`                          | ADC1, requires 5 V supply                           |
| GUVA-S12SD UV          | `6`                          | ADC1                                                |
| TTP223 capacitive pad  | `7`                          | `INPUT_PULLDOWN`                                    |
| Buzzer                 | `9`                          | LEDC channel 0                                      |
| KY-037 sound (A0)      | `10`                         | ADC1; ADC2 channels are reserved by Wi-Fi           |
| LD2410C radar          | `13` (RX), `14` (TX)         | UART2, 256 000 baud, **5 V supply required**        |
| ST7789V TFT (SPI)      | MOSI `35`, SCLK `36`, CS `37`, DC `38`, RST `39` | No backlight pin on the module used        |

### Installation

```bash
git clone https://github.com/Mastr00/wether.git
cd wether

# Initial flash over USB
pio run -t uploadfs        # writes the web dashboard to SPIFFS
pio run -t upload          # writes the firmware
pio device monitor         # 115200 baud
```

Subsequent builds can be flashed wirelessly:

```bash
pio run -e esp32-s3-ota -t upload
```

### Secrets

Sensitive credentials are isolated from the source tree. Copy the template, fill it in, and rebuild:

```bash
cp include/secrets.h.example include/secrets.h
# edit include/secrets.h with your Wi-Fi, OTA, and dashboard credentials
```

`include/secrets.h` is listed in [`.gitignore`](./.gitignore) and never reaches the repository. The build will fail to compile until the file exists.

### Build-time configuration

Non-sensitive configuration lives in the upper section of [`src/main.cpp`](src/main.cpp); sensitive values come from [`include/secrets.h`](include/secrets.h.example).

| Constant                  | Location                | Scope    | Description                                                                   |
|---------------------------|-------------------------|----------|-------------------------------------------------------------------------------|
| `SECRET_STA_SSID`         | `include/secrets.h`     | Wi-Fi    | SSID of the station network.                                                  |
| `SECRET_STA_PASS`         | `include/secrets.h`     | Wi-Fi    | WPA2 passphrase.                                                              |
| `SECRET_OTA_PASSWORD`     | `include/secrets.h`     | OTA      | Password required by `espota` to push new firmware.                           |
| `SECRET_WEB_USER`         | `include/secrets.h`     | Web      | HTTP Basic Auth username for the dashboard.                                   |
| `SECRET_WEB_PASS`         | `include/secrets.h`     | Web      | HTTP Basic Auth password for the dashboard.                                   |
| `SECRET_MQTT_USER` / `SECRET_MQTT_PASS` | `include/secrets.h` | MQTT | Optional broker credentials; define both to enable authenticated MQTT. |
| `MQTT_BROKER`             | `src/main.cpp`          | MQTT     | IPv4 address or hostname of the Mosquitto broker.                             |
| `MQTT_PORT`               | `src/main.cpp`          | MQTT     | TCP port (default `1883`).                                                    |
| `MQTT_CLIENT_ID`          | `src/main.cpp`          | MQTT     | Unique client identifier published to the broker.                             |
| `TOPIC_BASE`              | `src/main.cpp`          | MQTT     | Root topic for sensor publications (default `esp32/station`).                 |
| `DEVICE_HOSTNAME`         | `src/main.cpp`          | Network  | mDNS hostname (`station-meteo.local`).                                        |
| `POSIX_TZ`                | `src/main.cpp`          | Time     | POSIX time-zone string with DST rules.                                        |
| `NTP_SERVER`              | `src/main.cpp`          | Time     | NTP host queried by `NTPClient` and `configTime`.                             |
| `TFT_SLEEP_TIMEOUT_MS`    | `src/main.cpp`          | Display  | Inactivity delay before the TFT enters sleep mode.                            |
| `WIFI_DEAD_TIMEOUT_MS`    | `src/main.cpp`          | Recovery | Maximum Wi-Fi outage before the device reboots itself.                        |
| `WDT_TIMEOUT_S`           | `src/main.cpp`          | Recovery | Task watchdog timeout in seconds.                                             |
| `ALARM_CONFIRM_COUNT`     | `src/main.cpp`          | Alarm    | Consecutive over-threshold samples required to arm the buzzer.                |
| `GAS_WARMUP_SEC`          | `src/main.cpp`          | Alarm    | Grace period during which gas alarms are suppressed after boot.               |

## Available commands

The project follows the PlatformIO command convention.

| Command                                       | Purpose                                                            |
|-----------------------------------------------|--------------------------------------------------------------------|
| `pio run`                                     | Compile the default `esp32-s3` environment.                         |
| `pio run -t upload`                           | Compile and flash the firmware over USB.                            |
| `pio run -t uploadfs`                         | Upload the contents of `data/` to SPIFFS.                           |
| `pio run -e esp32-s3-ota -t upload`           | Flash a new firmware image over Wi-Fi via ArduinoOTA.               |
| `pio device monitor`                          | Open the serial monitor at 115 200 baud.                            |
| `pio run -t clean`                            | Remove build artefacts.                                             |
| `pio check`                                   | Run the static-analysis pass configured by PlatformIO.              |

## Testing

A `test/` directory is reserved for the [PlatformIO Unity-based test framework](https://docs.platformio.org/en/latest/advanced/unit-testing/index.html). No tests are currently bundled.

```bash
pio test -e esp32-s3
```

## Deployment

1. **Initial USB flash.** Plug the board in, install the USB-Serial driver if required, then run `pio run -t uploadfs && pio run -t upload`.
2. **Provision Wi-Fi.** Copy `include/secrets.h.example` to `include/secrets.h` and set `SECRET_STA_SSID` / `SECRET_STA_PASS` before flashing. After the first boot the device announces itself on the local network at `station-meteo.local`.
3. **Configure the broker.** Point `MQTT_BROKER` at a reachable Mosquitto instance (default port `1883`). Anonymous access is supported; enable `MQTT_USER` / `MQTT_PASS` for authenticated brokers.
4. **Pair with Home Assistant.** Install the MQTT integration in Home Assistant and target the same broker. Entities appear automatically under the device *ESP32-S3 Environmental Station* thanks to the discovery payloads emitted on connect.
5. **Subsequent updates.** Use the `esp32-s3-ota` environment to push firmware updates over the air; the device displays a progress bar on the TFT and reboots on completion.
6. **Remote operations.** The dashboard exposes `/restart`, `/crashlog`, and `/crashlog/clear` for in-field maintenance without re-flashing.

## Security

- **HTTP Basic Auth** guards every dashboard endpoint, including the JSON data feed, settings mutation, alarm control, and maintenance routes (`/restart`, `/crashlog`, `/crashlog/clear`).
- **OTA updates are password-protected** through `ArduinoOTA.setPassword(OTA_PASSWORD)`; firmware uploads are rejected if the secret is missing or wrong.
- **Last Will and Testament** publishes `offline` on the status topic when the device drops off the broker, preventing stale `online` reads in Home Assistant.
- **Alarm debouncing.** Three consecutive over-threshold samples are required before the siren is armed, and the gas channel is gated by a 60-second post-boot warm-up window to absorb MQ sensor transients.
- **Watchdog and recovery.** The hardware task watchdog reboots the device after 30 seconds of unresponsive code, and a Wi-Fi-dead guard resets it after five minutes without an associated AP.
- **Crash log.** Abnormal reboot reasons (`PANIC`, `TASK_WDT`, `INT_WDT`, `BROWNOUT`) are appended to `/crashlog.txt` on SPIFFS and exposed read-only through the authenticated `/crashlog` endpoint.
- **Secret handling.** Wi-Fi passphrase, OTA password, dashboard credentials, and optional MQTT credentials live in [`include/secrets.h`](include/secrets.h.example), which is gitignored. Only the placeholder template ([`include/secrets.h.example`](include/secrets.h.example)) is committed. Rotating credentials therefore only requires editing the local file and re-flashing.
- **Bluetooth radio disabled** at boot via `btStop()` to reduce the attack surface and power draw.

## License

No `LICENSE` file is currently provided. All rights are reserved by the repository owner until a license is added. To open the work to third parties, drop a `LICENSE` file at the repository root (for example [MIT](https://choosealicense.com/licenses/mit/) or [Apache-2.0](https://choosealicense.com/licenses/apache-2.0/)) and update the badge above.

## Contact

Maintainer: [`@Mastr00`](https://github.com/Mastr00). Issues and pull requests are tracked on the [GitHub repository](https://github.com/Mastr00/wether).
