# LMR-SDR-CNTRL

**ESP8266 + Si5351 controller for LMR / Tayloe (QSD) SDR**

Local oscillator controller with rotary encoder, SH1106 OLED, Web UI, Wi‑Fi setup, and Kenwood-style CAT. Designed for a ×4 (or ×1) LO into a Tayloe quadrature sampling detector.

---

## Features

| Feature | Description |
|--------|-------------|
| **Si5351 LO** | CLK0 output, runtime **×1 / ×4** (EEPROM) |
| **Encoder** | Tune · short press = step · long press = mode |
| **OLED** | 1.3" SH1106 128×64 (U8x8) — freq, mode, step, LO, IP, CAT |
| **Web UI** | Frequency, mode, step, LO mult, calibration, favourites |
| **Wi‑Fi** | Soft-AP + built-in setup page; optional WiFiManager portal |
| **CAT** | 115200 baud, `FA` / `FB` / `MD` / `IF` / `ID` |
| **EEPROM** | Last frequency (after idle delay), cal ppb, LO mult, 10 memories |
| **Calibration** | Si5351 crystal correction in **ppb** via Web UI |

---

## Hardware

| Function | NodeMCU | GPIO |
|----------|---------|------|
| I2C SCL (Si5351 + OLED) | **D1** | 5 |
| I2C SDA (Si5351 + OLED) | **D2** | 4 |
| Encoder A | **D5** | 14 |
| Encoder B | **D6** | 12 |
| Encoder SW | **D7** | 13 |

- ESP8266: **NodeMCU 1.0** or **Wemos D1 mini**
- Si5351A (25 MHz crystal)
- 1.3" **SH1106** OLED I2C
- Rotary encoder with switch

> **Arduino board setting:** use *NodeMCU 1.0 (ESP-12E Module)* or *LOLIN(WEMOS) D1 R2 & mini*.  
> Do **not** use *Generic ESP8266 Module* — that often boots with no Serial and a blank display.

---

## Libraries

1. [Etherkit Si5351](https://github.com/etherkit/Si5351Arduino)  
2. [U8g2](https://github.com/olikraus/u8g2) (U8x8)  
3. [WiFiManager](https://github.com/tzapu/WiFiManager) (tzapu)  
4. ESP8266 Arduino core  

---

## Quick start

1. Install the libraries and ESP8266 board package.  
2. Open `LMR_SDR_CNTRL.ino` (keep `config.h` in the same folder).  
3. Edit **`config.h`** if you need different pins, defaults, or favourites.  
4. Select the correct board + port → Upload.  
5. Serial Monitor **115200**.  
6. Join Wi‑Fi **`LMR-SDR-CNTRL`** / password **`lmrsdr123`** → open **http://192.168.4.1**

---

## Configuration (`config.h`)

Important compile-time options:

| Define | Default | Meaning |
|--------|---------|---------|
| `SI5351_CLK0_MULT` | `4` | Default LO mult (×4 Tayloe / ×1); Web UI can change & save |
| `FREQ_SAVE_DELAY_SEC` | `5` | Idle seconds before frequency auto-save |
| `CAT_BAUD` | `115200` | CAT + debug Serial |
| `SI5351_XTAL_DEFAULT` | `25000000` | Crystal Hz |
| `SI5351_CORRECTION_DEFAULT` | `0` | Default ppb |
| `DEFAULT_FREQ_HZ` | `7100000` | First-boot frequency |
| `FREQ_MIN_HZ` / `FREQ_MAX_HZ` | 100 kHz / 30 MHz | Tune limits |
| `WIFI_AP_SSID` / `WIFI_AP_PASS` | `LMR-SDR-CNTRL` / `lmrsdr123` | Soft-AP |
| `STEP_SIZES[]` | 10 Hz … 1 MHz | Encoder / Web step table |
| `DEFAULT_FAVOURITES[]` | 10 HF spots | Factory memories |

Full parameter tables: see **[docs/LMR_SDR_CNTRL_Complete_User_Guide.docx](docs/LMR_SDR_CNTRL_Complete_User_Guide.docx)**.

---

## Web UI & Wi‑Fi

- **Control:** frequency, **Freq ± (one step)**, mode, step size, **LO ×1/×4**, calibration ppb, favourites  
- **Wi‑Fi Setup** page (`/wifi`): enter home SSID/password → reboot into STA  
- Optional: reboot into **WiFiManager** portal (`LMR-SDR-Setup`)

---

## CAT (USB serial)

Baud **115200**, commands end with `;`

| Command | Action |
|---------|--------|
| `FA;` / `FA###########;` | Get / set frequency (Hz) |
| `FB…` | Same (mirrored) |
| `MD;` / `MD#;` | Mode |
| `IF;` `ID;` | Status / `ID019;` |

OLED and Web show **CAT active** for ~15 s after a command.

---

## Si5351 calibration

1. Warm up; set LO mult to match your detector (usually ×4).  
2. Set a known RF; measure CLK0 or zero-beat a reference on the SDR.  
3. Enter **ppb** on the Web UI → **Apply Calibration** (saved to EEPROM).  

**Sign:** positive = crystal slow; negative = crystal fast.

```text
ppb ≈ 1e9 × (commanded_Hz − measured_Hz) / commanded_Hz
```

(Use commanded LO = RF × multiplier when measuring CLK0.)

---

## Documentation

| File | Description |
|------|-------------|
| [docs/LMR_SDR_CNTRL_Complete_User_Guide.docx](docs/LMR_SDR_CNTRL_Complete_User_Guide.docx) | Full user guide + complete `config.h` reference + calibration |
| [docs/LMR_SDR_CNTRL_Calibration_Manual.docx](docs/LMR_SDR_CNTRL_Calibration_Manual.docx) | Calibration-focused manual (if included) |

---

## Repository layout

```text
LMR_SDR_CNTRL/
├── LMR_SDR_CNTRL.ino   # Firmware
├── config.h            # Pins & parameters
├── README.md
├── LICENSE
└── docs/
    └── LMR_SDR_CNTRL_Complete_User_Guide.docx
```

---

## License

MIT — see [LICENSE](LICENSE).

## Acknowledgements

OLED / Soft-AP boot patterns aligned with common ESP8266 + U8x8 practice (including community Si4732 receiver layouts). Si5351 control via the Etherkit library.
