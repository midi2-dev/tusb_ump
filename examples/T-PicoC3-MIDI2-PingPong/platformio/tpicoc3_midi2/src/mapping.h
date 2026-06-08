#ifndef MAPPING_H
#define MAPPING_H

// ── Board selection ───────────────────────────────────────────────────────────
//   -DBOARD_T_PICOC3    → LilyGO T-PicoC3 (RP2040 + ESP32-C3, ST7789V 240×135)
//   -DBOARD_RP_PICO     → Raspberry Pi Pico (RP2040, no display)
//   -DBOARD_RP_PICO2    → Raspberry Pi Pico 2 (RP2350, no display)

// ── Color palette ─────────────────────────────────────────────────────────────
#define C_BG        0x0000   // Black
#define C_HEADER    0x0821   // Very dark blue  — header/status bg
#define C_DIV       0x7BEF   // Light gray  (~48%) — dividers, dim text, inactive
#define C_WHITE     0xFFFF
#define C_GREEN     0x07E0   // Bright green — MIDI 2.0 active, NoteOn
#define C_CYAN      0x07FF   // Cyan — CC, stream messages, hex byte 0
#define C_ORANGE    0xFD20   // Orange — MIDI 1.0 active, disconnected
#define C_YELLOW    0xFFE0   // Yellow — SysEx
#define C_GRAY      0x8C71   // Medium gray (~55%) — NoteOff (was 0x4208, too dark)
#define C_MAGENTA   0xF81F   // Magenta — PitchBend

#ifdef BOARD_T_PICOC3

// ── T-PicoC3 hardware ────────────────────────────────────────────────────────
#define TFT_BL_PIN    4
#define PIN_POWER_ON  22
#define BTN1           6
#define BTN2           7
#define LED_PIN       25

// ── Screen (landscape 240 × 135) ─────────────────────────────────────────────
#define SCR_W    240
#define SCR_H    135
#define MARGIN     4

// ── Layout — T-Display-S3 style, scaled for 240×135 ──────────────────────────
//
//  Y=  0  ┌──────────────────────────────────────────┐
//         │  USB MIDI 2.0  •  T-PicoC3            ●  │  20  HEADER
//  Y= 20  ├─────────────────────┬────────────────────┤
//         │   MIDI 1.0          │   MIDI 2.0 / UMP   │  18  PROTO (color tabs)
//  Y= 38  ├─────────────────────┴────────────────────┤
//         │  20 90 3C 60   NoteOn G0 Ch0 C#4 V96     │  30  HEX  (2 lines)
//  Y= 68  ├──────────────────────────────────────────┤
//         │  ████████████████░░░░░  V: 96            │  14  VEL bar
//  Y= 82  ├──────────────────────────────────────────┤
//         │  R:NoteOn  G0 Ch0 N60 V96                │
//         │  T:EndpointReply                         │  28  LOG (2 × 14px, Font2)
//  Y=110  ├──────────────────────────────────────────┤
//         │  RX: 1234    TX: 5678         MIDI 2.0   │  25  STATUS
//  Y=135  └──────────────────────────────────────────┘
//
//  Totals: 20+18+30+14+28+25 = 135 ✓

#define Y_HEADER   0
#define H_HEADER  20

#define Y_PROTO   20
#define H_PROTO   18

#define Y_HEX     38
#define H_HEX     30

#define Y_VEL     68
#define H_VEL     14

#define Y_LOG     82
#define H_LOG     28
#define N_LOG      2
#define H_LLINE   14

#define Y_STATUS  110
#define H_STATUS   25

#else  // BOARD_RP_PICO or BOARD_RP_PICO2

#define BTN1    15
#define BTN2    16
#define LED_PIN 25

#endif  // board selection

#endif // MAPPING_H
