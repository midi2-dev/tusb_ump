# tusb_ump_host_demo - USB Host UMP Reference App

A small reference app built on `tusb_ump`'s `ump_host` TinyUSB class driver.
Plug a USB MIDI device (or a hub with one attached) into the board and it:

- Prints basic identification for **any** attached USB device (VID/PID,
  manufacturer/product/serial strings) as soon as it's mounted, MIDI or not
  - useful during bring-up to confirm enumeration is happening at all.
- Dumps the attached MIDI device's parsed (or synthesized, for
  alt-setting-0-only devices) [Group Terminal Block](../../ump.h) info once
  its UMP interface mounts.
- Decodes and prints every incoming UMP word, on whichever MIDIStreaming
  alternate setting the device presents:
  - **Alt 0** - legacy USB-MIDI 1.0 byte stream (translated to UMP by the
    driver)
  - **Alt 1** - UMP / USB MIDI 2.0 native
- Injects a test Note On/Off once a second (press SPACE to toggle) so you
  can exercise the write path without needing a second app sending MIDI.

It's meant as a small, self-contained reference for the API
(`tuh_ump_read_ntoh` / `tuh_ump_write_hton`, mount/unmount callbacks) needed
to build a real USB-MIDI-host application with this driver - not a product.

## Hardware

Default target is a Raspberry Pi Pico (RP2040). A Pico's USB port normally
*senses* VBUS supplied by whatever it's plugged into; running it in **host**
role flips that - the board must instead *supply* 5V onto its own VBUS pin
so a downstream USB device can power up and enumerate. Wire the Pico's own
5V rail (VSYS, or an external 5V supply) onto its USB connector's VBUS pin
(directly, or via whatever power-switch/jumper your carrier board provides)
before attaching a device - without it, nothing will enumerate.

Since the Pico's only USB port is committed to host role, this example logs
over the board's default UART (115200 8N1 - GP0/GP1 on a Pico) instead.

## Logging

```
tusb_ump_host_demo -- USB Host UMP reference app
Waiting for a USB MIDI device...
Press SPACE to toggle the periodic test Note On/Off write (starts OFF).

[USB] device mounted daddr=1 VID:PID=1234:5678 bcdUSB=0x0200 bcdDevice=0x0100 class=0/0/0
  Manufacturer: Example Corp
  Product: Example MIDI Keyboard
  Serial Number: (no string descriptor)

[UMP] mounted daddr=1 itf_num=1
  alt_setting = 1 (native UMP)
  USB MIDI Streaming class spec version (bcdMSC) = 0x0200
  Group Terminal Blocks: 1
    [0] ID=1 type=0x00 firstGroup=0 numGroups=1 protocol=0x02 maxInBW=0 maxOutBW=0
[UMP daddr=1 itf=1] read word: 0x40903c64 (MT=0x4 group=0)
[UMP]            0x00000000
```

## Runtime Controls

Press **SPACE** at any time to toggle a periodic test Note On/Off (middle
C, alternating every second) sent to every currently-mounted UMP device -
useful for confirming the write path works even with nothing else sending
MIDI to the device.

## Building

Requires the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
(2.0.0 or later) and its ARM toolchain, either installed via the [Pico VS
Code extension](https://github.com/raspberrypi/pico-vscode) or with
`PICO_SDK_PATH` set manually.

```sh
cmake -S . -B build -G Ninja
cmake --build build
```

This produces `build/tusb_ump_host_demo.uf2`. Hold BOOTSEL while plugging in
the Pico, then copy the UF2 onto the mass-storage drive that appears.

## Trying it out

Plug a USB MIDI 2.0 or USB-MIDI 1.0 device (or a hub with one attached) into
the board's USB port. Watch the UART log for enumeration, then play a note
on the device (or send MIDI to it from a DAW/controller) and watch it appear
as decoded UMP words. Press SPACE to also see a test note make it back out
to the device.

### Porting to other targets

Nothing in `src/main.cpp` is RP2040-specific except board bring-up
(`stdio_init_all()`) and the `tinyusb_overrides/hcd_rp2040.c` swap in
`CMakeLists.txt` (an RP2040 host-controller-driver bugfix - see that file's
own header comment; drop it if your target's controller doesn't need it).
The driver itself - [`../../ump_host.h`](../../ump_host.h) /
[`../../ump_host.cpp`](../../ump_host.cpp) - is a generic TinyUSB host class
driver with no RP2040 dependencies. To port to another pico-sdk board with a
native USB host controller, change `PICO_BOARD` in `CMakeLists.txt` (or pass
`-DPICO_BOARD=...` on the command line). To port to a non pico-sdk MCU, swap
this project's `CMakeLists.txt`/`pico_sdk_import.cmake` for that target's
TinyUSB host build integration and carry `ump_host.h`/`ump_host.cpp`/`ump.h`
over unchanged.
