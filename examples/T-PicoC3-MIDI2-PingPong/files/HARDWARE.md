# T-PicoC3 — Hardware Reference

## Overview

LilyGO T-PicoC3 is a dual-chip development board combining a **Raspberry Pi RP2040**
and an **Espressif ESP32-C3** on a single PCB, sharing one USB Type-C connector.

- **RP2040** — ARM Cortex-M0+, 133 MHz, 264 KB SRAM, 2 MB Flash
  → handles USB MIDI 2.0 device (TinyUSB native)

- **ESP32-C3** — RISC-V 32-bit, 160 MHz, Wi-Fi 2.4 GHz, BLE 5
  → available for debug Serial or wireless features

---

## Display

| Parameter | Value |
|-----------|-------|
| Driver IC | ST7789V (rev. of ST7789, same TFT_eSPI driver) |
| Resolution | 240 × 135 px |
| Interface | SPI |
| Backlight pin (RP2040) | GPIO 4 (TFT_BL) |

> The T-Display-S3 uses ST7789 at 320×170. The T-PicoC3 is smaller (240×135).
> The `UMPDisplay.h` driver will need adaptation for the lower resolution.
> **The display is the primary debug interface** — it shows alt setting, received UMP
> packets, and counters, exactly as on the T-Display-S3. No USB flip needed for debug.

---

## USB Behavior

The USB-C connector is **physically shared** between both chips via orientation switch:

| USB-C orientation | Connected chip | Use |
|-------------------|----------------|-----|
| Normal | RP2040 | USB MIDI 2.0 device → ping-pong with T-Display-S3 |
| Flipped 180° | ESP32-C3 | Serial debug fallback (if display is insufficient) |

LED indicators on the board show which chip is currently connected.

**In normal operation the display (ST7789V) is used for debug** — it shows alt setting,
UMP message type, and packet counters. The USB-C flip to ESP32-C3 is available as a
fallback for deeper Serial debugging only.

---

## RP2040 GPIO Pinout

| Signal | GPIO |
|--------|------|
| TFT_MOSI | 3 |
| TFT_SCLK | 2 |
| TFT_CS | 5 |
| TFT_DC | 1 |
| TFT_RST | 0 |
| TFT_BL | 4 |
| PWR_ON | 22 |
| BUTTON1 | 6 |
| BUTTON2 | 7 |
| Red LED | 25 |

> `PWR_ON` (GPIO 22) must be driven HIGH to power the display and peripherals.

---

## Images

- [pinout.png](pinout.png) — GPIO pinout diagram
- [specs.png](specs.png) — board specifications
- [switch.png](switch.png) — USB switch / connector orientation diagram

---

## References

- **Board repository:** https://github.com/Xinyuan-LilyGO/T-PicoC3
- **Schematic (PDF):** https://github.com/Xinyuan-LilyGO/T-PicoC3/blob/main/Schematic/T-PicoC3.pdf
- **RP2040 datasheet:** https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf
- **ESP32-C3 TRM (CN):** https://github.com/Xinyuan-LilyGO/T-PicoC3/blob/main/doc/esp32-c3_technical_reference_manual_cn.pdf
- **arduino-pico (Earle Philhower):** https://github.com/earlephilhower/arduino-pico
- **TinyUSB:** https://github.com/hathach/tinyusb

---

## Development Notes

- Framework target: **arduino-pico** (not arduino-esp32)
- The RP2040 TinyUSB in arduino-pico is the **upstream** version — no IDF quirks
- `__attribute__((used))` workaround from T-Display-S3 may **not** be needed
- `build_unflags` for `--gc-sections` should be reviewed against arduino-pico defaults
- Display library: **TFT_eSPI** (already in T-PicoC3 repo `/lib/`) configured for ST7789V
- `UMPDisplay.h` adaptation needed: change `TFT_WIDTH`/`TFT_HEIGHT` from 320×170 → 240×135
- **Ping-pong target:** T-Display-S3 (ESP32-S3) — same VID/PID `0x1209:0x0001`, same UMP
  protocol contract, same Endpoint Discovery. Any two boards can exchange UMP packets.
