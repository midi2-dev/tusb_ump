# tusb_ump — Roadmap

## Board compatibility matrix

| Example | MCU | Display | Debug | USB Stack | Status |
|---------|-----|---------|-------|-----------|--------|
| T-Display-S3-MIDI2-PingPong | ESP32-S3 | ST7789 320x170 | Serial (second USB) | TinyUSB / IDF fork | ✅ Done |
| T-PicoC3-MIDI2-PingPong | RP2040 + ESP32-C3 | ST7789 240×135 | C3 USB (flip USB-C) | Adafruit TinyUSB 0.18 | ✅ Done |
| RP-Pico-2-RP2350-MIDI2-PingPong | RP2350 | none | Serial (built-in) | Adafruit TinyUSB 0.18 | ✅ Done |
| RP-Pico-RP2040-MIDI2-PingPong | RP2040 | none | Serial (built-in) | Adafruit TinyUSB 0.18 | ✅ Done |
| Teensy-4.1-MIDI2-PingPong | iMXRT1062 | none | Serial (dedicated USB) | TinyUSB (non-standard) | 🔬 Research |
| DaisySeed-MIDI2-PingPong | STM32H750 | none | Serial (dedicated USB) | STM32 HAL / LibDaisy | 🔬 Research |

All examples implement the same **UMP protocol contract** — any two boards can
ping-pong with each other, or with any third-party MIDI 2.0 device.

---

## Protocol contract (shared across all examples)

Every example must implement:

1. **Same USB descriptors** — Alt 0 (MIDI 1.0) + Alt 1 (UMP), same GTB structure
2. **Same Endpoint Discovery** — full MT=0xF Stream message responses via `ump_stream_handler.h`
3. **Same echo-back** — all non-Stream UMP messages echoed to host (loopback for testing)
4. **Same VID/PID** — `0x1209:0x0001` across all examples (consistent ping-pong identity)
5. **MIDI 2.0 Compliance Checklist** — each example must pass all items before merge

---

## Near term

- [ ] **Unified example folder structure** — currently each example is split into
  `platformio/` and `arduino/` subfolders. The ideal is a single folder compatible
  with Arduino IDE, PlatformIO, and ESP-IDF simultaneously (following the official
  Arduino library spec and Arduino CLI conventions), similar to how the ESP32_Host_MIDI
  project handles it. The main obstacle is that RP2040 (Adafruit TinyUSB API) and
  ESP32-S3 (strong symbol override) require different USB descriptor implementations,
  but the folder structure could still be unified using `#ifdef` or separate source
  files selected at build time.

- [x] **T-PicoC3-MIDI2-PingPong** — RP2040 + ESP32-C3 (LilyGO T-PicoC3). Display:
  ST7789V 240×135 via SPI — used as the debug interface (same role as T-Display-S3).
  RP2040 USB handles MIDI; display shows alt setting, received UMP, packet counters.
  Designed to ping-pong directly with the T-Display-S3 example. Framework: arduino-pico
  (Earle Philhower). Adafruit TinyUSB 0.18. Validates tusb_ump on Cortex-M0+.

- [x] **RP-Pico-2-RP2350-MIDI2-PingPong** — bare board, no display. Serial monitor
  shows alt setting, RX/TX events, auto-sends NoteOn every 2 s. RP2350 (Cortex-M33).
  Framework: arduino-pico. Adafruit TinyUSB 0.18. Built-in LED blinks on USB mount.

- [x] **RP-Pico-RP2040-MIDI2-PingPong** — same as above for the original Pico.
  Shares all code with RP2350 example — only the board target differs in platformio.ini.

- [ ] **Native MIDI 2.0 Channel Voice (MT=0x4)** — currently all examples send MT=0x2
  (MIDI 1.0 CV wrapped in UMP). When Alt Setting 1 is active, upgrade to native 64-bit
  MIDI 2.0 messages with 32-bit velocity, per-note pitch, per-note CC.

- [ ] **Protocol Negotiation** — automatic switch from MT=0x2 to MT=0x4 when Alt 1
  is negotiated, based on Endpoint Info capabilities exchange.

- [ ] **SysEx demonstration** — button-triggered SysEx Identity Request/Reply to
  actively exercise the SysEx 7-bit (MT=0x3) path.

---

## Medium term

- [ ] **Teensy 4.1** — iMXRT1062 has dedicated USB for Serial (debug is easy) and
  a second USB for device/host. Challenge: Paul Stoffregen's USB stack is not standard
  TinyUSB. Requires either: (a) porting tusb_ump to Teensy's USBHost_t36 MIDI2 layer,
  or (b) configuring Teensy to use TinyUSB via alternate build path. Research needed.

- [ ] **Daisy Seed** — STM32H750, uses LibDaisy / DaisyDuino. Has dedicated USB for
  Serial debug and a second USB for audio/MIDI device. Challenge: LibDaisy wraps STM32
  HAL USB, not TinyUSB. Options: (a) adapt tusb_ump to STM32 HAL app driver model,
  (b) add TinyUSB as a component alongside LibDaisy, (c) use DaisyDuino's existing
  USB MIDI class as a base.

- [ ] **Multi-group / multi-Function Block** — expand beyond a single group to
  demonstrate multiple Function Blocks (e.g., synth on Group 0, drums on Group 1).

- [ ] **Linux ALSA validation** — when ALSA userspace tools support Alt Setting 1
  and UMP Endpoint Discovery, add a Linux test script alongside the Windows ones.

- [ ] **macOS CoreMIDI 2.0** — test when Apple ships public USB MIDI 2.0 host support.

---

## Long term

- [ ] **MIDI-CI Property Exchange** — query and set device properties over UMP.
- [ ] **MIDI-CI Profile Configuration** — advertise and respond to standard profiles
  (General MIDI, MPE, Default Control Change Mapping).
- [ ] **Integration with TinyUSB upstream** — contribute the UMP class driver directly
  into TinyUSB's `class/` directory, eliminating the external library dependency.
- [ ] **ESP-IDF component** — publish to ESP Component Registry with `idf_component.yml`
  for native ESP-IDF projects (no Arduino dependency).
