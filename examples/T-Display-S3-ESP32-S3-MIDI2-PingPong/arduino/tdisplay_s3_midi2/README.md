# USB MIDI 2.0 Device — Arduino IDE

ESP32-S3 USB MIDI 2.0 device example using tusb_ump.
Tested on LilyGO T-Display-S3.

## Setup

1. **Board:** ESP32S3 Dev Module
2. **Tools > USB Mode:** USB-OTG (TinyUSB)
3. **Tools > USB CDC On Boot:** Disabled
4. **LovyanGFX:** install via Library Manager
5. **tusb_ump:** copy repo into `~/Arduino/libraries/tusb_ump/`
6. **Build flags:** copy `platform.local.txt.example` as `platform.local.txt` to:
   - Linux: `~/.arduino15/packages/esp32/hardware/esp32/<version>/`
   - macOS: `~/Library/Arduino15/packages/esp32/hardware/esp32/<version>/`
   - Windows: `%LOCALAPPDATA%\Arduino15\packages\esp32\hardware\esp32\<version>\`
7. **Restart Arduino IDE**, then open `tdisplay_s3_midi2.ino` and upload.

## Files

| File | Description |
|------|-------------|
| `tdisplay_s3_midi2.ino` | Main sketch (setup/loop, buttons, UMP RX/TX) |
| `usb_descriptors.cpp` | USB MIDI 2.0 config descriptor (dual alt settings) |
| `UMPDisplay.h` | LovyanGFX display handler |
| `mapping.h` | Pin assignments, colours, layout |
| `platform.local.txt.example` | Build flags template for Arduino IDE |

## Controls

| Button | Action |
|--------|--------|
| BTN1 (GPIO0) | Toggle NoteOn / NoteOff (C5) |
| BTN2 (GPIO14) | Cycle velocity: 32 > 64 > 96 > 127 |

See the [PlatformIO version](../../platformio/tdisplay_s3_midi2/) for an
alternative build setup that doesn't require `platform.local.txt`.
