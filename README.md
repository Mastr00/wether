# 🌦️ Station Météo & Alarme Connectée (ESP32)

Ce projet est une station météo intelligente basée sur un **ESP32-S3**. Il surveille la température, l'humidité, la qualité de l'air (gaz), la luminosité et le niveau sonore. Il intègre également une fonction **alarme** (détection de gaz ou de bruit excessif) avec notifications via **Pushover**.

## 🚀 Fonctionnalités

*   **Météo** : Affichage Température, Humidité (DHT11).
*   **Qualité de l'Air** : Détection de gaz (MQ-2 ou similaire).
*   **Luminosité** : Mesure via photorésistance (LDR).
*   **Niveau Sonore** : Mesure dB via KY-037 (avec calibration).
*   **GPS** : Position, Vitesse, Heure et Date précises via module GPS.
*   **Détection de Mouvement** : Via capteur PIR.
*   **Interface Web** : Tableau de bord complet (Chart.js, Leaflet map) hébergé sur l'ESP32 (SPIFFS).
*   **Alarme** : Sirène (Buzzer) et Notification push (Pushover) en cas de seuil critique (Gaz ou Bruit).
*   **Écran OLED** : Affichage local des données principales en temps réel.

## 🛠️ Matériel Requis

*   **ESP32-S3** (DevKitC-1 ou compatible)
*   **DHT11** (Température/Humidité) - Pin 15
*   **MQ-2 / MQ-135** (Gaz) - Pin 5
*   **LDR** (Photorésistance) - Pin 4
*   **KY-037** (Capteur Son) - Pin 11 (Analog), Pin 12 (Digital)
*   **HC-SR501** (PIR Mouvement) - Pin 6
*   **GPS Module** (NEO-6M ou compatible) - RX: 13, TX: 14, PPS: 7
*   **OLED Display** (SSD1306 I2C) - SDA: 8, SCL: 10
*   **Buzzer** - Pin 9

## ⚙️ Configuration Avant Flash

Avant de téléverser le code, vous devez configurer vos identifiants dans `src/main.cpp`.

1.  Ouvrez `src/main.cpp`.
2.  Cherchez la section **Identifiants WiFi** et modifiez :
    ```cpp
    const char* STA_SSID = "VOTRE_SSID_ICI";
    const char* STA_PASS = "VOTRE_MOT_DE_PASSE_ICI";
    ```
3.  Cherchez la section **Pushover** (pour les notifications) et modifiez :
    ```cpp
    #define PUSHOVER_API_TOKEN "VOTRE_API_TOKEN_ICI"
    #define PUSHOVER_USER_KEY "VOTRE_USER_KEY_ICI"
    ```
4.  Cherchez la section **Identifiants Login Web** pour sécuriser l'accès au tableau de bord :
    ```cpp
    const char* LOGIN_USER = "admin"; // Changez-le !
    const char* LOGIN_PASS = "admin"; // Changez-le !
    ```

## 📦 Installation

1.  Installez [PlatformIO](https://platformio.org/) sur VS Code.
2.  Clonez ce dépôt.
3.  Ouvrez le dossier du projet dans PlatformIO.
4.  Connectez votre ESP32-S3.
5.  **Important** : Uploadez d'abord les fichiers du système de fichiers (SPIFFS) :
    *   Dans PlatformIO > Project Tasks > esp32-s3 > Platform > **Upload Filesystem Image**.
6.  Téléversez le firmware :
    *   Cliquez sur la flèche **Upload** (ou `pio run -t upload`).

## 🖥️ Utilisation

1.  Une fois démarré, l'adresse IP de l'ESP32 s'affiche sur le port Série et sur l'écran OLED (si configuré pour).
2.  Accédez à `http://<IP_ESP32>/` depuis votre navigateur.
3.  Connectez-vous avec vos identifiants.
4.  Visualisez les données en temps réel, les graphiques et la carte GPS.
5.  Allez dans "Réglages" pour ajuster les seuils d'alarme.

## ⚠️ Avertissement

Ce projet est à but éducatif. Ne l'utilisez pas comme seul système de sécurité pour des situations critiques (incendie, intrusion).
