// Configuration TFT_eSPI pour ST7789V 2" (240x320) sur ESP32-S3
//
// Cablage ecran → ESP32-S3 :
//   SDA  → GPIO 35   (= MOSI, donnees)
//   SCL  → GPIO 36   (= SCLK, horloge)
//   CS   → GPIO 37
//   DC   → GPIO 38
//   RST  → GPIO 39
//   VCC  → 3.3V
//   GND  → GND
// (pas de pin BL sur ce module → retroeclairage toujours allume)

#define ST7789_DRIVER
#define TFT_WIDTH  240
#define TFT_HEIGHT 320

// Le tactile est gere par un TTP223 separe: aucune puce tactile TFT/TOUCH_CS.
#define DISABLE_ALL_LIBRARY_WARNINGS

#define USE_FSPI_PORT // Add this to fix the ESP32-S3 StoreProhibited crash


#define TFT_MOSI  35   // SDA sur l'ecran
#define TFT_SCLK  36   // SCL sur l'ecran
#define TFT_CS    37
#define TFT_DC    38
#define TFT_RST   39
// Pas de broche backlight sur ce module → on ne definit PAS TFT_BL
// (definir TFT_BL a -1 provoque une erreur GPIO dans certaines versions de TFT_eSPI)

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF
#define SMOOTH_FONT

#define SPI_FREQUENCY       27000000   // 27 MHz : plus stable, fluidite identique
#define SPI_READ_FREQUENCY  20000000
