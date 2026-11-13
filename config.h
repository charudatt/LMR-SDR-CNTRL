/*
 * config.h - Parameters & pin definitions for ESP8266 LMR-SDR CNTRL
 *
 * Pin style matches the working Si4732 SSB Receiver project:
 * GPIO numbers (not only D1/D2 labels) for reliability across boards.
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
// Compile-time options
// ============================================================

// Si5351 CLK0 multiplier for Tayloe / QSD detector
// Set to 4 for x4 LO (typical Tayloe), or 1 for x1
#define SI5351_CLK0_MULT        4

// Seconds of frequency stability before auto-save to EEPROM
#define FREQ_SAVE_DELAY_SEC     5

// Serial (CAT + boot debug) baud rate
#define CAT_BAUD                115200

// Default crystal frequency (Hz)
#define SI5351_XTAL_DEFAULT     25000000UL

// Default correction in parts-per-billion (ppb). Positive = crystal is slow.
#define SI5351_CORRECTION_DEFAULT  0

// EEPROM magic ("LMRD") + version
#define EEPROM_MAGIC            0x4C4D5244UL
#define EEPROM_VERSION          3

#define FAV_CHANNELS            10
#define DEFAULT_FREQ_HZ         7100000UL
#define DEFAULT_MODE            0

static const uint32_t STEP_SIZES[] = {
  10, 100, 1000, 5000, 10000, 100000, 1000000
};
#define NUM_STEP_SIZES          (sizeof(STEP_SIZES)/sizeof(STEP_SIZES[0]))
#define DEFAULT_STEP_INDEX      2   // 1 kHz

#define FREQ_MIN_HZ             100000UL
#define FREQ_MAX_HZ             30000000UL

// ============================================================
// Pin definitions — GPIO numbers (NodeMCU / Wemos D1 mini)
// Same map as working ESP8266-Si4732-SSB-Receiver
// ============================================================

// Shared I2C bus: OLED + Si5351
#define PIN_I2C_SCL             5    // GPIO5  / D1
#define PIN_I2C_SDA             4    // GPIO4  / D2

// Rotary encoder
#define PIN_ENC_A              14    // GPIO14 / D5
#define PIN_ENC_B              12    // GPIO12 / D6
#define PIN_ENC_SW             13    // GPIO13 / D7

// Optional status LED (NodeMCU onboard often GPIO2 / D4, active low)
#define PIN_LED                 2

// ============================================================
// WiFi / AP defaults (AP-first like working Si4732 project)
// ============================================================
#define WIFI_AP_SSID            "LMR-SDR-CNTRL"
#define WIFI_AP_PASS            "lmrsdr123"
#define WIFI_HOSTNAME           "lmr-sdr-cntrl"
#define STA_CONNECT_TIMEOUT_S   12

#define OLED_I2C_ADDR           0x3C
#define SI5351_I2C_ADDR         0x60
#define SI5351_CLK              SI5351_CLK0

// Favourites: { freq_Hz, mode }  mode 0=LSB 1=USB 2=CW 3=AM 4=FM
struct FavEntry {
  uint32_t freq;
  uint8_t  mode;
};

static const FavEntry DEFAULT_FAVOURITES[FAV_CHANNELS] = {
  {  1840000UL, 0 },
  {  3650000UL, 0 },
  {  7100000UL, 0 },
  { 10120000UL, 1 },
  { 14200000UL, 1 },
  { 18100000UL, 1 },
  { 21200000UL, 1 },
  { 24920000UL, 1 },
  { 28300000UL, 1 },
  {  7030000UL, 2 },
};

static const char* MODE_NAMES[] = { "LSB", "USB", "CW", "AM", "FM" };
#define NUM_MODES 5

#endif
