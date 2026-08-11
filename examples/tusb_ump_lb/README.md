# tusb_ump_lb - USB MIDI 2.0 (UMP) Loopback Example

A minimal canned example built on `tusb_ump`'s `ump_device` TinyUSB class
driver. It enumerates as a single USB MIDI 2.0 device with one bidirectional
[Group Terminal Block](../../ump.h) spanning all 16 groups, and echoes back
any UMP word(s) it receives as soon as they arrive - on whichever
MIDIStreaming alternate setting the host has selected:

- **Alt 0** - legacy USB-MIDI 1.0 byte stream (backward compatible fallback)
- **Alt 1** - UMP / USB MIDI 2.0

It's meant as a small, self-contained reference for the descriptor layout
and API (`tud_ump_read_ntoh` / `tud_ump_write_hton`) needed to build a real
UMP-capable USB MIDI device with this driver - not a product.

## Hardware

Default target is a Raspberry Pi Pico (RP2040), using the on-board LED as a
mount indicator (blinks while unmounted, solid once the host opens the UMP
interface). No other wiring is required - loop MIDI back to itself purely
over USB.

### Porting to other targets

Nothing in `src/main.c` or `src/usb_descriptors.c` is RP2040-specific except
board bring-up in `main()` (`stdio_init_all()` / on-board LED GPIO). The
driver itself - [`../../ump_device.h`](../../ump_device.h) /
[`../../ump_device.cpp`](../../ump_device.cpp) - is a generic TinyUSB device
class driver with no RP2040 dependencies, registered via TinyUSB's
`usbd_app_driver_get_cb()` hook. To port to another pico-sdk board, change
`PICO_BOARD` in `CMakeLists.txt` (or pass `-DPICO_BOARD=...` on the command
line). To port to a non pico-sdk MCU, swap this project's
`CMakeLists.txt`/`pico_sdk_import.cmake` for that target's TinyUSB build
integration and carry `ump_device.h`/`ump_device.cpp`/`ump.h` over unchanged.

## Building

Requires the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
(2.0.0 or later) and its ARM toolchain, either installed via the [Pico VS
Code extension](https://github.com/raspberrypi/pico-vscode) or with
`PICO_SDK_PATH` set manually.

```sh
cmake -S . -B build -G Ninja
cmake --build build
```

This produces `build/tusb_ump_lb.uf2`. Hold BOOTSEL while plugging in the
Pico, then copy the UF2 onto the mass-storage drive that appears.

## Trying it out

Any UMP-capable MIDI 2.0 host (e.g. a DAW using Windows MIDI Services or
macOS/CoreMIDI's MIDI 2.0 support) should enumerate the device as
"tusb_ump Loopback" with a single "Loopback" function block. Anything sent
to it - notes, CCs, SysEx7 packed as UMP, etc. - is sent straight back.
