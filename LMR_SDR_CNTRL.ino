/*
 * LMR_SDR_CNTRL.ino
 * ESP8266 + Si5351 Controller for LMR SDR (Tayloe / QSD)
 *
 * Boot / OLED / WiFi pattern matched to the known-working
 * ESP8266-Si4732-SSB-Receiver project on the same hardware.
 *
 * ---------------------------------------------------------------------------
 * REQUIRED LIBRARIES
 * ---------------------------------------------------------------------------
 * 1) Etherkit Si5351
 *    https://github.com/etherkit/Si5351Arduino
 * 2) U8g2 (U8x8)
 *    https://github.com/olikraus/u8g2
 * 3) WiFiManager (tzapu) — only used on-demand from Web UI, NOT at boot
 *    https://github.com/tzapu/WiFiManager
 * 4) ESP8266 core
 *    http://arduino.esp8266.com/stable/package_esp8266com_index.json
 *
 * BOARD (critical):
 *   Use "NodeMCU 1.0 (ESP-12E Module)" or "LOLIN(WEMOS) D1 R2 & mini"
 *   Do NOT use "Generic ESP8266 Module" — wrong board = no Serial / no OLED.
 * ---------------------------------------------------------------------------
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <si5351.h>
#include <U8x8lib.h>
#include "config.h"

// ------------------------------------------------------------------
// Hardware objects
// ------------------------------------------------------------------
Si5351 si5351;

// Same OLED constructor as working Si4732 project
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE, /*clock=*/ PIN_I2C_SCL, /*data=*/ PIN_I2C_SDA);
// Fallback if blank: comment HW line above, uncomment SW line below
// U8X8_SH1106_128X64_NONAME_SW_I2C u8x8(/*clock=*/ PIN_I2C_SCL, /*data=*/ PIN_I2C_SDA, /*reset=*/ U8X8_PIN_NONE);

ESP8266WebServer server(80);

// ------------------------------------------------------------------
// Runtime state
// ------------------------------------------------------------------
uint32_t currentFreq   = DEFAULT_FREQ_HZ;
uint8_t  currentMode   = DEFAULT_MODE;
uint8_t  stepIndex     = DEFAULT_STEP_INDEX;
int32_t  si5351Corr    = SI5351_CORRECTION_DEFAULT;
uint8_t  si5351Addr    = 0;
bool     si5351Ok      = false;
uint8_t  loMult        = SI5351_CLK0_MULT;  // runtime 1 or 4 (EEPROM)
uint32_t lastCatMs     = 0;
bool     catSeen       = false;

bool     freqDirty     = false;
uint32_t lastFreqChangeMs = 0;
bool     eepromDirty   = false;

FavEntry favourites[FAV_CHANNELS];

volatile int8_t encDelta = 0;
uint8_t lastEncA = 1, lastEncB = 1;
bool    encBtnPressed = false;
uint32_t encBtnPressMs = 0;
bool    encBtnLongDone = false;

bool    displayNeedsUpdate = true;
uint32_t lastDisplayMs = 0;

String  catBuffer;

bool    webWiFiIsAP = true;
IPAddress webIPAddr(192, 168, 4, 1);

// EEPROM layout
#define EE_ADDR_MAGIC     0
#define EE_ADDR_VERSION   4
#define EE_ADDR_FREQ      5
#define EE_ADDR_MODE      9
#define EE_ADDR_STEP      10
#define EE_ADDR_CORR      11
#define EE_ADDR_LOMULT    15
#define EE_ADDR_FAVS      16
#define EE_ADDR_WIFIMGR   (EE_ADDR_FAVS + FAV_CHANNELS * 5)
#define EE_SIZE           (EE_ADDR_WIFIMGR + 8)
#define EE_WIFIMGR_MAGIC  0xA5

// ------------------------------------------------------------------
// Forward declarations
// ------------------------------------------------------------------
void applyFrequency();
void saveToEEPROM(bool force = false);
void loadFromEEPROM();
void updateDisplay(bool force = false);
void handleEncoder();
void processCAT();
void setupWebServer();
void handleRoot();
void handleSet();
void handleCal();
void handleFav();
void handleStatus();
void handleFactory();
void handleWifiManager();
void handleWifiPage();
void handleWifiSave();
void handleWifiScan();
uint8_t scanSi5351();
void setSi5351Correction(int32_t corr);
String modeName(uint8_t m);
String formatFreq(uint32_t f);
void startSoftAP();

// ------------------------------------------------------------------
// Encoder ISR
// ------------------------------------------------------------------
void IRAM_ATTR encoderISR() {
  uint8_t a = digitalRead(PIN_ENC_A);
  uint8_t b = digitalRead(PIN_ENC_B);
  if (a != lastEncA) {
    if (a == b) encDelta++;
    else        encDelta--;
  }
  lastEncA = a;
  lastEncB = b;
}

// ------------------------------------------------------------------
// Setup — keep this path as close as possible to the working Si4732 sketch
// ------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(F("\nLMR-SDR-CNTRL starting..."));

  pinMode(PIN_ENC_A,  INPUT_PULLUP);
  pinMode(PIN_ENC_B,  INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);

  // ---- I2C + OLED first (identical order to working project) ----
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);   // GPIO4=SDA, GPIO5=SCL
  Wire.setClock(100000);

  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.clear();
  u8x8.drawString(0, 0, "LMR-SDR-CNTRL");
  u8x8.drawString(0, 1, "Starting...");
  Serial.println(F("OLED OK"));

  // ---- EEPROM ----
  EEPROM.begin(EE_SIZE);
  loadFromEEPROM();
  u8x8.drawString(0, 2, "EEPROM OK");

  // ---- Si5351 (non-fatal if missing) ----
  u8x8.drawString(0, 3, "Si5351...");
  Serial.println(F("Scanning Si5351..."));
  si5351Addr = scanSi5351();
  char buf[22];
  if (si5351Addr == 0) {
    u8x8.drawString(0, 4, "Si NOT FOUND");
    Serial.println(F("Si5351 not found"));
  } else {
    snprintf(buf, sizeof(buf), "Si 0x%02X", si5351Addr);
    u8x8.drawString(0, 4, buf);
    Serial.printf("Si5351 at 0x%02X\n", si5351Addr);
  }

  si5351Ok = si5351.init(SI5351_CRYSTAL_LOAD_8PF, SI5351_XTAL_DEFAULT, si5351Corr);
  if (si5351Ok) {
    si5351.set_correction(si5351Corr, SI5351_PLL_INPUT_XO);
    si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
    si5351.output_enable(SI5351_CLK0, 1);
    applyFrequency();
    u8x8.drawString(0, 5, "Si5351 OK");
    Serial.println(F("Si5351 init OK"));
  } else {
    u8x8.drawString(0, 5, "Si5351 FAIL");
    Serial.println(F("Si5351 init FAIL (continuing)"));
  }

  // Encoder after display is alive
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);

  // ---- Wi-Fi ----
  // If Web UI requested WiFiManager, run captive portal once at boot (reliable).
  if (EEPROM.read(EE_ADDR_WIFIMGR) == EE_WIFIMGR_MAGIC) {
    EEPROM.write(EE_ADDR_WIFIMGR, 0);
    EEPROM.commit();
    Serial.println(F("Boot WiFiManager portal (flag set)..."));
    u8x8.clear();
    u8x8.drawString(0, 0, "WiFi Manager");
    u8x8.drawString(0, 2, "Join AP:");
    u8x8.drawString(0, 3, "LMR-SDR-Setup");
    u8x8.drawString(0, 5, "Pass: lmrsdr123");
    u8x8.drawString(0, 7, "Open 192.168.4.1");

    WiFi.softAPdisconnect(true);
    WiFi.disconnect(true);
    delay(300);
    WiFi.mode(WIFI_STA);
    delay(200);

    WiFiManager wm;
    wm.setDebugOutput(true);
    wm.setHostname(WIFI_HOSTNAME);
    wm.setConfigPortalTimeout(180);
    wm.setConnectTimeout(25);
    bool ok = wm.startConfigPortal("LMR-SDR-Setup", WIFI_AP_PASS);
    if (ok) {
      Serial.println(F("Credentials saved, rebooting"));
      delay(500);
      ESP.restart();
    }
    Serial.println(F("Portal timeout — continuing to Soft-AP"));
  }

  // Normal Soft-AP for control UI (always available if not in STA)
  // Try saved STA briefly, else AP
  u8x8.drawString(0, 6, "WiFi...");
  Serial.println(F("WiFi start..."));
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin();  // use saved credentials if any
  {
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000UL) {
      delay(250);
      Serial.print('.');
    }
    Serial.println();
  }
  if (WiFi.status() == WL_CONNECTED) {
    webWiFiIsAP = false;
    webIPAddr = WiFi.localIP();
    Serial.print(F("STA IP: "));
    Serial.println(webIPAddr);
  } else {
    Serial.println(F("Starting Soft-AP..."));
    startSoftAP();
  }

  u8x8.clear();
  u8x8.drawString(0, 0, "LMR-SDR-CNTRL");
  if (si5351Addr) {
    snprintf(buf, sizeof(buf), "Si 0x%02X", si5351Addr);
    u8x8.drawString(0, 1, buf);
  } else {
    u8x8.drawString(0, 1, "Si --");
  }
  if (webWiFiIsAP) {
    u8x8.drawString(0, 2, "AP mode");
    u8x8.drawString(0, 3, "192.168.4.1");
    u8x8.drawString(0, 4, WIFI_AP_SSID);
    u8x8.drawString(0, 5, WIFI_AP_PASS);
  } else {
    u8x8.drawString(0, 2, "STA mode");
    u8x8.drawString(0, 3, webIPAddr.toString().c_str());
  }
  delay(1500);

  setupWebServer();
  server.begin();
  Serial.println(F("HTTP server on http://192.168.4.1"));
  Serial.println(F("Ready."));

  displayNeedsUpdate = true;
  updateDisplay(true);
}

void startSoftAP() {
  webWiFiIsAP = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  delay(150);
  webIPAddr = WiFi.softAPIP();
  Serial.print(F("Soft-AP SSID: ")); Serial.println(WIFI_AP_SSID);
  Serial.print(F("Soft-AP PASS: ")); Serial.println(WIFI_AP_PASS);
  Serial.print(F("Soft-AP IP  : ")); Serial.println(webIPAddr);
}

// ------------------------------------------------------------------
// Loop
// ------------------------------------------------------------------
void loop() {
  server.handleClient();
  processCAT();
  handleEncoder();

  if (freqDirty && (millis() - lastFreqChangeMs >= (uint32_t)FREQ_SAVE_DELAY_SEC * 1000UL)) {
    saveToEEPROM(false);
    freqDirty = false;
  }

  if (displayNeedsUpdate || (millis() - lastDisplayMs > 2000)) {
    if (millis() - lastDisplayMs > 2000) displayNeedsUpdate = true;
    updateDisplay(false);
  }

  yield();
}

// ------------------------------------------------------------------
// Frequency
// ------------------------------------------------------------------
void applyFrequency() {
  if (currentFreq < FREQ_MIN_HZ) currentFreq = FREQ_MIN_HZ;
  if (currentFreq > FREQ_MAX_HZ) currentFreq = FREQ_MAX_HZ;

  if (si5351Ok) {
    uint8_t m = (loMult == 1) ? 1 : 4;
    uint64_t lo = (uint64_t)currentFreq * (uint64_t)m;
    si5351.set_freq(lo * 100ULL, SI5351_CLK0);
  }

  lastFreqChangeMs = millis();
  freqDirty = true;
  displayNeedsUpdate = true;
}

void setFrequency(uint32_t f) {
  currentFreq = f;
  applyFrequency();
}

void changeFrequency(int32_t delta) {
  int64_t nf = (int64_t)currentFreq + delta;
  if (nf < (int64_t)FREQ_MIN_HZ) nf = FREQ_MIN_HZ;
  if (nf > (int64_t)FREQ_MAX_HZ) nf = FREQ_MAX_HZ;
  currentFreq = (uint32_t)nf;
  applyFrequency();
}

// ------------------------------------------------------------------
// Encoder
// ------------------------------------------------------------------
void handleEncoder() {
  noInterrupts();
  int8_t d = encDelta;
  encDelta = 0;
  interrupts();

  if (d != 0) {
    changeFrequency(d * (int32_t)STEP_SIZES[stepIndex]);
  }

  bool btn = (digitalRead(PIN_ENC_SW) == LOW);
  uint32_t now = millis();

  if (btn && !encBtnPressed) {
    encBtnPressed = true;
    encBtnPressMs = now;
    encBtnLongDone = false;
  } else if (btn && encBtnPressed && !encBtnLongDone && (now - encBtnPressMs > 800)) {
    currentMode = (currentMode + 1) % NUM_MODES;
    encBtnLongDone = true;
    displayNeedsUpdate = true;
    eepromDirty = true;
  } else if (!btn && encBtnPressed) {
    if (!encBtnLongDone && (now - encBtnPressMs < 800)) {
      stepIndex = (stepIndex + 1) % NUM_STEP_SIZES;
      displayNeedsUpdate = true;
      eepromDirty = true;
    }
    encBtnPressed = false;
  }
}

// ------------------------------------------------------------------
// OLED
// ------------------------------------------------------------------
void updateDisplay(bool force) {
  if (!force && !displayNeedsUpdate) return;
  if (!force && (millis() - lastDisplayMs < 80)) return;
  lastDisplayMs = millis();
  displayNeedsUpdate = false;

  char buf[22];
  bool catActive = catSeen && (millis() - lastCatMs < 15000UL);

  if (catActive) snprintf(buf, sizeof(buf), "%-3s CAT        ", modeName(currentMode).c_str());
  else           snprintf(buf, sizeof(buf), "%-3s            ", modeName(currentMode).c_str());
  u8x8.drawString(0, 0, buf);

  u8x8.setFont(u8x8_font_8x13B_1x2_f);
  snprintf(buf, sizeof(buf), "%-11s", formatFreq(currentFreq).c_str());
  u8x8.drawString(0, 1, buf);
  u8x8.setFont(u8x8_font_chroma48medium8_r);

  uint32_t st = STEP_SIZES[stepIndex];
  if (st >= 1000000) snprintf(buf, sizeof(buf), "Step %lu MHz  ", st / 1000000UL);
  else if (st >= 1000) snprintf(buf, sizeof(buf), "Step %lu kHz  ", st / 1000UL);
  else snprintf(buf, sizeof(buf), "Step %lu Hz   ", st);
  u8x8.drawString(0, 3, buf);

  snprintf(buf, sizeof(buf), "LO x%u %dppb  ", (loMult == 1) ? 1 : 4, (int)si5351Corr);
  u8x8.drawString(0, 4, buf);

  if (webWiFiIsAP) snprintf(buf, sizeof(buf), "AP 192.168.4.1 ");
  else {
    String ip = webIPAddr.toString();
    snprintf(buf, sizeof(buf), "%-16s", ip.c_str());
  }
  u8x8.drawString(0, 5, buf);

  if (catActive) u8x8.drawString(0, 6, "CAT active     ");
  else           u8x8.drawString(0, 6, "CAT idle       ");

  u8x8.drawString(0, 7, "Enc:tune SW:stp");
}

String formatFreq(uint32_t f) {
  char buf[16];
  uint32_t mhz = f / 1000000UL;
  uint32_t khz = (f % 1000000UL) / 1000UL;
  uint32_t hz  = f % 1000UL;
  snprintf(buf, sizeof(buf), "%2lu.%03lu.%03lu", mhz, khz, hz);
  return String(buf);
}

String modeName(uint8_t m) {
  if (m < NUM_MODES) return String(MODE_NAMES[m]);
  return "???";
}

// ------------------------------------------------------------------
// Si5351 helpers
// ------------------------------------------------------------------
uint8_t scanSi5351() {
  const uint8_t addrs[] = { 0x60, 0x61, 0x62, 0x6F };
  for (uint8_t i = 0; i < sizeof(addrs); i++) {
    Wire.beginTransmission(addrs[i]);
    if (Wire.endTransmission() == 0) return addrs[i];
  }
  for (uint8_t a = 1; a < 127; a++) {
    if (a == 0x3C || a == 0x3D) continue; // OLED
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) return a;
  }
  return 0;
}

void setSi5351Correction(int32_t corr) {
  si5351Corr = corr;
  if (si5351Ok) {
    si5351.set_correction(si5351Corr, SI5351_PLL_INPUT_XO);
    applyFrequency();
  }
  eepromDirty = true;
  displayNeedsUpdate = true;
}

// ------------------------------------------------------------------
// EEPROM
// ------------------------------------------------------------------
void loadFromEEPROM() {
  uint32_t magic = 0;
  EEPROM.get(EE_ADDR_MAGIC, magic);
  uint8_t ver = EEPROM.read(EE_ADDR_VERSION);

  if (magic != EEPROM_MAGIC || ver != EEPROM_VERSION) {
    Serial.println(F("EEPROM defaults"));
    currentFreq = DEFAULT_FREQ_HZ;
    currentMode = DEFAULT_MODE;
    stepIndex   = DEFAULT_STEP_INDEX;
    si5351Corr  = SI5351_CORRECTION_DEFAULT;
    loMult      = SI5351_CLK0_MULT;
    for (int i = 0; i < FAV_CHANNELS; i++) favourites[i] = DEFAULT_FAVOURITES[i];
    saveToEEPROM(true);
    return;
  }

  EEPROM.get(EE_ADDR_FREQ, currentFreq);
  currentMode = EEPROM.read(EE_ADDR_MODE);
  stepIndex   = EEPROM.read(EE_ADDR_STEP);
  EEPROM.get(EE_ADDR_CORR, si5351Corr);
  loMult = EEPROM.read(EE_ADDR_LOMULT);
  if (loMult != 1 && loMult != 4) loMult = SI5351_CLK0_MULT;

  if (currentMode >= NUM_MODES) currentMode = DEFAULT_MODE;
  if (stepIndex >= NUM_STEP_SIZES) stepIndex = DEFAULT_STEP_INDEX;
  if (currentFreq < FREQ_MIN_HZ || currentFreq > FREQ_MAX_HZ) currentFreq = DEFAULT_FREQ_HZ;

  for (int i = 0; i < FAV_CHANNELS; i++) {
    int addr = EE_ADDR_FAVS + i * 5;
    EEPROM.get(addr, favourites[i].freq);
    favourites[i].mode = EEPROM.read(addr + 4);
    if (favourites[i].mode >= NUM_MODES) favourites[i].mode = 0;
  }
  Serial.printf("EEPROM: %lu Hz mode %u corr %d\n", currentFreq, currentMode, (int)si5351Corr);
}

void saveToEEPROM(bool force) {
  if (!force && !freqDirty && !eepromDirty) return;

  EEPROM.put(EE_ADDR_MAGIC, (uint32_t)EEPROM_MAGIC);
  EEPROM.write(EE_ADDR_VERSION, EEPROM_VERSION);
  EEPROM.put(EE_ADDR_FREQ, currentFreq);
  EEPROM.write(EE_ADDR_MODE, currentMode);
  EEPROM.write(EE_ADDR_STEP, stepIndex);
  EEPROM.put(EE_ADDR_CORR, si5351Corr);
  EEPROM.write(EE_ADDR_LOMULT, loMult);

  for (int i = 0; i < FAV_CHANNELS; i++) {
    int addr = EE_ADDR_FAVS + i * 5;
    EEPROM.put(addr, favourites[i].freq);
    EEPROM.write(addr + 4, favourites[i].mode);
  }
  EEPROM.commit();
  eepromDirty = false;
  Serial.println(F("EEPROM saved"));
}

// ------------------------------------------------------------------
// CAT (Kenwood TS-570 style, 115200)
// ------------------------------------------------------------------
void processCAT() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == ';') {
      catBuffer.trim();
      if (catBuffer.length() > 0) handleCATCommand(catBuffer);
      catBuffer = "";
    } else if (c >= 32 && c < 127) {
      catBuffer += c;
      if (catBuffer.length() > 64) catBuffer = "";
    }
  }
}

void handleCATCommand(const String& cmd) {
  catSeen = true;
  lastCatMs = millis();
  displayNeedsUpdate = true;

  String c = cmd;
  c.toUpperCase();

  if (c.startsWith("FA")) {
    if (c.length() == 2) {
      char buf[16];
      snprintf(buf, sizeof(buf), "FA%011lu;", currentFreq);
      Serial.print(buf);
    } else if (c.length() >= 13) {
      uint32_t f = c.substring(2, 13).toInt();
      if (f >= FREQ_MIN_HZ && f <= FREQ_MAX_HZ) setFrequency(f);
      char buf[16];
      snprintf(buf, sizeof(buf), "FA%011lu;", currentFreq);
      Serial.print(buf);
    }
    return;
  }

  if (c.startsWith("FB")) {
    if (c.length() == 2) {
      char buf[16];
      snprintf(buf, sizeof(buf), "FB%011lu;", currentFreq);
      Serial.print(buf);
    } else if (c.length() >= 13) {
      uint32_t f = c.substring(2, 13).toInt();
      if (f >= FREQ_MIN_HZ && f <= FREQ_MAX_HZ) setFrequency(f);
      char buf[16];
      snprintf(buf, sizeof(buf), "FB%011lu;", currentFreq);
      Serial.print(buf);
    }
    return;
  }

  if (c.startsWith("MD")) {
    if (c.length() == 2) {
      uint8_t km = 1;
      switch (currentMode) {
        case 0: km = 1; break;
        case 1: km = 2; break;
        case 2: km = 3; break;
        case 3: km = 5; break;
        case 4: km = 4; break;
      }
      Serial.printf("MD%d;", km);
    } else if (c.length() >= 3) {
      int km = c.substring(2).toInt();
      switch (km) {
        case 1: currentMode = 0; break;
        case 2: currentMode = 1; break;
        case 3: case 7: currentMode = 2; break;
        case 4: currentMode = 4; break;
        case 5: currentMode = 3; break;
        default: break;
      }
      eepromDirty = true;
      displayNeedsUpdate = true;
      Serial.printf("MD%d;", km);
    }
    return;
  }

  if (c == "IF") {
    char buf[64];
    snprintf(buf, sizeof(buf), "IF%011lu     +00000000003000000 ;", currentFreq);
    Serial.print(buf);
    return;
  }

  if (c == "ID") {
    Serial.print(F("ID019;"));
    return;
  }

  if (c.startsWith("AI")) {
    Serial.print(c);
    Serial.print(';');
    return;
  }
}

// ------------------------------------------------------------------
// Web UI
// ------------------------------------------------------------------
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>LMR-SDR-CNTRL</title>
<style>
body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:12px}
h1{font-size:1.3rem;margin:0 0 8px}
.card{background:#1e1e1e;border-radius:10px;padding:14px;margin-bottom:12px}
label{display:block;margin:6px 0 2px;font-size:.85rem;color:#aaa}
input,select,button{font-size:1rem;padding:8px;border-radius:6px;border:1px solid #444;background:#2a2a2a;color:#fff;width:100%;box-sizing:border-box}
button{background:#0a7;border:none;margin-top:8px;cursor:pointer}
button.sec{background:#444}
button.danger{background:#a33}
.row{display:flex;gap:8px}
.row>*{flex:1}
#freq{font-size:1.6rem;font-weight:700}
.small{font-size:.8rem;color:#888}
</style>
</head>
<body>
<h1>LMR-SDR-CNTRL</h1>
<div class="card">
  <div id="freq">--</div>
  <div class="small" id="info">Mode / Step / LO</div>
</div>
<div class="card">
  <label>Frequency (Hz)</label>
  <input type="number" id="fIn" min="100000" max="30000000" step="1">
  <button onclick="setFreq()">Set Frequency</button>
  <p class="small">Change frequency by the selected tuning step:</p>
  <div class="row">
    <button class="sec" onclick="nudge(-1)">Freq - (one step)</button>
    <button class="sec" onclick="nudge(1)">Freq + (one step)</button>
  </div>
  <label>Mode</label>
  <select id="modeSel" onchange="setMode()">
    <option value="0">LSB</option><option value="1">USB</option>
    <option value="2">CW</option><option value="3">AM</option><option value="4">FM</option>
  </select>
  <label>Tuning step size</label>
  <select id="stepSel" onchange="setStep()">
    <option value="0">10 Hz</option><option value="1">100 Hz</option>
    <option value="2">1 kHz</option><option value="3">5 kHz</option>
    <option value="4">10 kHz</option><option value="5">100 kHz</option>
    <option value="6">1 MHz</option>
  </select>
  <label>Si5351 LO multiplier (Tayloe QSD)</label>
  <select id="multSel" onchange="setMult()">
    <option value="4">x4 (LO = 4 x RF)</option>
    <option value="1">x1 (LO = 1 x RF)</option>
  </select>
  <p class="small">Saved to EEPROM. Use x4 for a standard Tayloe detector.</p>
</div>
<div class="card">
  <label>Si5351 calibration (ppb)</label>
  <input type="number" id="calIn" step="1">
  <button onclick="setCal()">Apply Calibration</button>
  <p class="small">Positive if crystal is slow. Saved to EEPROM.</p>
</div>
<div class="card">
  <label>Favourites</label>
  <select id="favSel"></select>
  <div class="row">
    <button onclick="recallFav()">Recall</button>
    <button class="sec" onclick="storeFav()">Store current</button>
  </div>
</div>
<div class="card">
  <a href="/wifi" style="display:block;text-align:center;padding:10px;background:#246;color:#fff;border-radius:6px;text-decoration:none;margin-top:8px">WiFi Setup (join home network)</a>
  <button class="danger" onclick="factory()">Factory Reset</button>
</div>
<script>
let st={};
function fmt(f){let m=Math.floor(f/1e6),k=Math.floor((f%1e6)/1e3),h=f%1e3;return m+'.'+String(k).padStart(3,'0')+'.'+String(h).padStart(3,'0');}
async function refresh(){
  try{
    let r=await fetch('/status'); st=await r.json();
    document.getElementById('freq').textContent=fmt(st.freq)+' Hz';
    document.getElementById('info').textContent=st.modeName+' | step '+st.step+' Hz | LO x'+st.mult+' | '+st.corr+' ppb | '+st.ip+(st.cat ? ' | CAT active' : ' | CAT idle');
    document.getElementById('fIn').value=st.freq;
    document.getElementById('modeSel').value=st.mode;
    document.getElementById('stepSel').value=st.stepIdx;
    if(document.getElementById('multSel')) document.getElementById('multSel').value=st.mult;
    document.getElementById('calIn').value=st.corr;
    let fs=document.getElementById('favSel');
    if(fs.options.length===0 && st.favs){
      st.favs.forEach((f,i)=>{let o=document.createElement('option');o.value=i;o.textContent='CH'+(i+1)+': '+fmt(f.freq)+' '+f.modeName;fs.appendChild(o);});
    }
  }catch(e){}
}
async function setFreq(){await fetch('/set?freq='+document.getElementById('fIn').value);refresh();}
async function nudge(d){await fetch('/set?nudge='+d);refresh();}
async function setMode(){await fetch('/set?mode='+document.getElementById('modeSel').value);refresh();}
async function setStep(){await fetch('/set?step='+document.getElementById('stepSel').value);refresh();}
async function setMult(){await fetch('/set?mult='+document.getElementById('multSel').value);refresh();}
async function setCal(){await fetch('/cal?ppb='+document.getElementById('calIn').value);refresh();}
async function recallFav(){await fetch('/fav?recall='+document.getElementById('favSel').value);refresh();}
async function storeFav(){await fetch('/fav?store='+document.getElementById('favSel').value);refresh();location.reload();}
async function factory(){if(confirm('Factory reset?')){await fetch('/factory');location.reload();}}
refresh(); setInterval(refresh,2000);
</script>
</body>
</html>
)rawliteral";

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/set", handleSet);
  server.on("/cal", handleCal);
  server.on("/fav", handleFav);
  server.on("/factory", handleFactory);
  server.on("/wifi", handleWifiPage);
  server.on("/wifi-save", HTTP_POST, handleWifiSave);
  server.on("/wifi-scan", handleWifiScan);
  server.on("/wifi-manager", handleWifiManager);  // reboot into captive portal
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
}

void handleRoot() {
  server.send_P(200, "text/html", HTML_PAGE);
}

void handleStatus() {
  String json = "{";
  json += "\"freq\":" + String(currentFreq) + ",";
  json += "\"mode\":" + String(currentMode) + ",";
  json += "\"modeName\":\"" + modeName(currentMode) + "\",";
  json += "\"stepIdx\":" + String(stepIndex) + ",";
  json += "\"step\":" + String(STEP_SIZES[stepIndex]) + ",";
  json += "\"mult\":" + String((loMult == 1) ? 1 : 4) + ",";
  json += "\"corr\":" + String(si5351Corr) + ",";
  json += "\"cat\":" + String((catSeen && (millis() - lastCatMs < 15000UL)) ? "true" : "false") + ",";
  if (webWiFiIsAP) json += "\"ip\":\"192.168.4.1 (AP)\",";
  else json += "\"ip\":\"" + webIPAddr.toString() + "\",";
  json += "\"favs\":[";
  for (int i = 0; i < FAV_CHANNELS; i++) {
    if (i) json += ",";
    json += "{\"freq\":" + String(favourites[i].freq) +
            ",\"mode\":" + String(favourites[i].mode) +
            ",\"modeName\":\"" + modeName(favourites[i].mode) + "\"}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

void handleSet() {
  if (server.hasArg("freq")) {
    uint32_t f = server.arg("freq").toInt();
    if (f >= FREQ_MIN_HZ && f <= FREQ_MAX_HZ) setFrequency(f);
  }
  if (server.hasArg("nudge")) {
    int d = server.arg("nudge").toInt();
    changeFrequency(d * (int32_t)STEP_SIZES[stepIndex]);
  }
  if (server.hasArg("mode")) {
    int m = server.arg("mode").toInt();
    if (m >= 0 && m < NUM_MODES) { currentMode = m; eepromDirty = true; displayNeedsUpdate = true; }
  }
  if (server.hasArg("step")) {
    int s = server.arg("step").toInt();
    if (s >= 0 && s < (int)NUM_STEP_SIZES) { stepIndex = s; eepromDirty = true; displayNeedsUpdate = true; }
  }
  if (server.hasArg("mult")) {
    int m = server.arg("mult").toInt();
    if (m == 1 || m == 4) {
      loMult = (uint8_t)m;
      applyFrequency();
      eepromDirty = true;
      saveToEEPROM(true);
      displayNeedsUpdate = true;
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleCal() {
  if (server.hasArg("ppb")) {
    setSi5351Correction(server.arg("ppb").toInt());
    saveToEEPROM(true);
  }
  server.send(200, "text/plain", "OK");
}

void handleFav() {
  if (server.hasArg("recall")) {
    int i = server.arg("recall").toInt();
    if (i >= 0 && i < FAV_CHANNELS) {
      currentFreq = favourites[i].freq;
      currentMode = favourites[i].mode;
      applyFrequency();
      eepromDirty = true;
    }
  }
  if (server.hasArg("store")) {
    int i = server.arg("store").toInt();
    if (i >= 0 && i < FAV_CHANNELS) {
      favourites[i].freq = currentFreq;
      favourites[i].mode = currentMode;
      eepromDirty = true;
      saveToEEPROM(true);
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleFactory() {
  EEPROM.put(EE_ADDR_MAGIC, (uint32_t)0);
  EEPROM.commit();
  currentFreq = DEFAULT_FREQ_HZ;
  currentMode = DEFAULT_MODE;
  stepIndex   = DEFAULT_STEP_INDEX;
  si5351Corr  = SI5351_CORRECTION_DEFAULT;
  loMult      = SI5351_CLK0_MULT;
  for (int i = 0; i < FAV_CHANNELS; i++) favourites[i] = DEFAULT_FAVOURITES[i];
  setSi5351Correction(si5351Corr);
  saveToEEPROM(true);
  displayNeedsUpdate = true;
  server.send(200, "text/plain", "Factory reset done");
}

// Set flag and reboot — WiFiManager captive portal runs at next boot (reliable on ESP8266)
void handleWifiManager() {
  EEPROM.write(EE_ADDR_WIFIMGR, EE_WIFIMGR_MAGIC);
  EEPROM.commit();
  server.send(200, "text/plain", "Rebooting into WiFi Manager portal (AP LMR-SDR-Setup)...");
  delay(400);
  ESP.restart();
}

// Built-in WiFi setup page (works while Soft-AP is up — no portal switch required)
void handleWifiPage() {
  String html = F(
    "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>WiFi Setup</title>"
    "<style>"
    "body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:16px}"
    "input,select,button{width:100%;padding:10px;margin:6px 0;border-radius:6px;border:1px solid #444;"
    "background:#2a2a2a;color:#fff;font-size:1rem;box-sizing:border-box}"
    "button{background:#0a7;border:none}"
    "a{color:#6af}"
    ".card{background:#1e1e1e;padding:14px;border-radius:10px}"
    "</style></head><body>"
    "<h2>WiFi Setup</h2>"
    "<div class='card'>"
    "<form method='POST' action='/wifi-save'>"
    "<label>Network (SSID)</label>"
    "<input name='ssid' id='ssid' required placeholder='Home WiFi name'>"
    "<label>Password</label>"
    "<input name='pass' type='password' placeholder='WiFi password'>"
    "<button type='submit'>Connect &amp; Save</button>"
    "</form>"
    "<p style='font-size:.85rem;color:#aaa'>Board will reboot and try to join the network. "
    "If join fails it returns to AP mode (LMR-SDR-CNTRL).</p>"
    "<p><a href='/wifi-scan'>Scan networks</a> · <a href='/'>Back</a></p>"
    "<hr style='border-color:#333'>"
    "<p style='font-size:.85rem'>Or use full captive portal:</p>"
    "<form method='GET' action='/wifi-manager'><button type='submit' style='background:#246'>"
    "Reboot into WiFiManager portal</button></form>"
    "</div>"
    "<script>"
    "fetch('/wifi-scan').then(r=>r.json()).then(list=>{"
    " if(!list.length)return;"
    " let s=document.getElementById('ssid');"
    " let d=document.createElement('div');"
    " list.forEach(n=>{let b=document.createElement('button');b.type='button';"
    " b.textContent=n;b.style.background='#333';b.onclick=()=>{s.value=n};d.appendChild(b)});"
    " s.parentNode.insertBefore(d,s.nextSibling);"
    "}).catch(()=>{});"
    "</script>"
    "</body></html>"
  );
  server.send(200, "text/html", html);
}

void handleWifiScan() {
  // Scanning briefly drops STA traffic; OK in AP mode
  int n = WiFi.scanNetworks();
  String json = "[";
  for (int i = 0; i < n; i++) {
    if (i) json += ",";
    String ssid = WiFi.SSID(i);
    ssid.replace("\"", "\\\"");
    json += "\"" + ssid + "\"";
  }
  json += "]";
  WiFi.scanDelete();
  server.send(200, "application/json", json);
}

void handleWifiSave() {
  if (!server.hasArg("ssid")) {
    server.send(400, "text/plain", "ssid required");
    return;
  }
  String ssid = server.arg("ssid");
  String pass = server.hasArg("pass") ? server.arg("pass") : "";
  ssid.trim();

  server.send(200, "text/html",
    F("<html><body style='font-family:sans-serif;background:#111;color:#eee;padding:20px'>"
      "<h2>Saving...</h2><p>Connecting and rebooting. Rejoin AP if STA fails.</p></body></html>"));
  delay(300);

  Serial.print(F("WiFi save SSID="));
  Serial.println(ssid);

  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000UL) {
    delay(400);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("Connected, IP "));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("Connect failed — will boot AP if still down after restart"));
  }
  delay(400);
  ESP.restart();
}

