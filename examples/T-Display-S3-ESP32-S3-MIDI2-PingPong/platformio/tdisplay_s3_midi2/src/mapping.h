#ifndef MAPPING_H
#define MAPPING_H

// ── T-Display-S3 hardware ────────────────────────────────────────────────────
#define TFT_BL_PIN    38
#define PIN_POWER_ON  15
#define BTN1           0   // GPIO0  — toggle NoteOn/Off
#define BTN2          14   // GPIO14 — cycle velocity

// ── Screen (landscape 320 × 170) ────────────────────────────────────────────
#define SCR_W    320
#define SCR_H    170
#define MARGIN     4

// ── Color palette ────────────────────────────────────────────────────────────
#define C_BG        0x0000   // Black
#define C_HEADER    0x0821   // Very dark blue — header bg
#define C_DIV       0x2945   // Dark slate     — dividers, dim labels
#define C_PANEL     0x1082   // Dark navy      — section panels
#define C_WHITE     0xFFFF   // White
#define C_GREEN     0x07E0   // Green
#define C_CYAN      0x07FF   // Cyan
#define C_ORANGE    0xFD20   // Orange
#define C_YELLOW    0xFFE0   // Yellow
#define C_GRAY      0x4208   // Gray
#define C_BLUE      0x001F   // Blue
#define C_MAGENTA   0xF81F   // Magenta

// ── Layout rows ──────────────────────────────────────────────────────────────
//
//  Y=  0  ┌───────────────────────────────────┐
//         │   USB MIDI 2.0  •  T-Display-S3  ● │  24 HEADER
//  Y= 24  ├────────────────┬──────────────────┤
//         │   MIDI 1.0     │  MIDI 2.0 / UMP  │  18 PROTO
//  Y= 42  ├────────────────┴──────────────────┤
//         │  20 90 48 60  ── NoteOn info       │  30 HEX
//  Y= 72  ├───────────────────────────────────┤
//         │  ████████████░░░░░░░░  Vel: 96    │  14 VEL
//  Y= 86  ├───────────────────────────────────┤
//         │  NoteOn  G0 Ch0  N72 V96          │
//         │  NoteOff G0 Ch0  N72              │  56 LOG (4×14)
//         │  CC      G0 Ch0  CC7 V64          │
//         │  NoteOn  G0 Ch0  N60 V80          │
//  Y=142  ├───────────────────────────────────┤
//         │  RX:0     TX:0       MIDI 1.0     │  28 STATUS
//  Y=170  └───────────────────────────────────┘

#define Y_HEADER  0
#define H_HEADER  24

#define Y_PROTO   24
#define H_PROTO   18

#define Y_HEX     42
#define H_HEX     30

#define Y_VEL     72
#define H_VEL     14

#define Y_LOG     86
#define H_LOG     56
#define N_LOG      4
#define H_LLINE   14

#define Y_STATUS  142
#define H_STATUS   28

#endif // MAPPING_H
