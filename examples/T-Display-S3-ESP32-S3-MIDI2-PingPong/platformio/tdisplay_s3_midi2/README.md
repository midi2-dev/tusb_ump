# USB MIDI 2.0 Device — PlatformIO

ESP32-S3 USB MIDI 2.0 device example using tusb_ump.
Tested on LilyGO T-Display-S3.

## Build

```bash
pio run -e T-Display-S3-MIDI2            # compile
pio run -e T-Display-S3-MIDI2 -t upload  # flash
pio device monitor                       # serial output
```

All build flags are in `platformio.ini` — no extra configuration needed.

## Files

```
tdisplay_s3_midi2/
├── platformio.ini
└── src/
    ├── main.cpp              — setup/loop, buttons, UMP RX/TX
    ├── usb_descriptors.cpp   — USB MIDI 2.0 config descriptor (dual alt settings)
    ├── UMPDisplay.h          — LovyanGFX display handler
    └── mapping.h             — Pin assignments, colours, layout
```

## Controls

| Button | Action |
|--------|--------|
| BTN1 (GPIO0) | Toggle NoteOn / NoteOff (C5) |
| BTN2 (GPIO14) | Cycle velocity: 32 > 64 > 96 > 127 |

## Build flags

| Flag | Purpose |
|------|---------|
| `ARDUINO_USB_MODE=0` | TinyUSB device mode |
| `ARDUINO_USB_CDC_ON_BOOT=0` | MIDI 2.0 owns the USB port |
| `CFG_TUD_UMP=1` | Enable tusb_ump class driver |
| `CFG_TUD_MIDI=0` | Disable built-in MIDI 1.0 driver |

See the [Arduino IDE version](../../arduino/tdisplay_s3_midi2/) if you prefer
Arduino IDE over PlatformIO.
