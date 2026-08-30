# tusb_ump_uart_din_bridge - DIN MIDI 1.0 <-> USB MIDI 2.0 (UMP) bridge over hardware UART

A canned example built on `tusb_ump`'s `ump_device` TinyUSB class driver,
bridging a physical DIN MIDI 1.0 port to a USB MIDI 2.0 (UMP) interface over
the RP2040's real hardware UART peripheral (not a PIO bit-banger). It
enumerates as a single USB MIDI 2.0 device with one bidirectional
[Group Terminal Block](../../ump.h) covering the single UMP group bridged
to/from the DIN port:

- **Alt 0** - legacy USB-MIDI 1.0 byte stream (backward compatible fallback)
- **Alt 1** - UMP / USB MIDI 2.0

Byte<->UMP conversion is handled by this example's own dependency-free
[`src/midi1_bytestream.h`](src/midi1_bytestream.h) /
[`src/midi1_bytestream.c`](src/midi1_bytestream.c) - no external library, in
keeping with `tusb_ump`'s own "no dependence on external libraries" design.
It's scoped to plain MIDI 1.0 UMP words (Channel Voice / System / Sysex7),
since that's all a physical DIN port can carry - no MIDI 2.0 upscaling.

This example does not implement UMP Endpoint/Function Block Discovery (the
UMP Stream messages a fully protocol-compliant MIDI 2.0 endpoint answers) -
like [`tusb_ump_lb`](../tusb_ump_lb), it's scoped to demonstrating the
transport and descriptor layout, not a complete implementation. For a
Discovery-compliant DIN bridge reference, see `UUT/UART_DIN_Bridge` in the
[AmeNote_Protozoa](https://github.com/midi2-dev/AmeNote_Protozoa) repo,
which this example was ported from.

## Hardware

Default target is a Raspberry Pi Pico (RP2040). The DIN port defaults to
**UART1 on GPIO4 (TX) / GPIO5 (RX)** (see `DIN_UART`/`DIN_TX_PIN`/
`DIN_RX_PIN` in [`src/main.c`](src/main.c)) - deliberately *not* UART0,
since UART0 is also this board's default console/stdio UART (GPIO0/1).
Sharing one hardware UART peripheral between two independent pin pairs
doesn't work: initializing it for the DIN port reprograms the shared
baud-rate divisor the console depends on, and can also block the console's
own RX routing entirely (confirmed by hands-on testing on a board with this
exact constraint - see `UUT/UART_DIN_Bridge`'s README in AmeNote_Protozoa
for the full story). Change `DIN_UART`/pins in `main.c` to whichever UART
instance your board's console doesn't already use; wire a MIDI 1.0 DIN
transceiver circuit to those pins (opto-isolated input on RX, current-loop
driver on TX, standard 31.25kbaud).

## Logging

Boot and alt-setting changes are logged over the board's default UART
(115200 8N1 - GP0/GP1 on a Pico), same as `tusb_ump_lb`. The USB MIDI
interface itself carries only MIDI data.

### Porting to other targets

Only `src/main.c`'s board bring-up (which UART/pins, `stdio_init_all()`) is
RP2040-specific. `src/midi1_bytestream.h`/`.c` (the byte<->UMP converter)
and [`../../ump_device.h`](../../ump_device.h) /
[`../../ump_device.cpp`](../../ump_device.cpp) (the generic TinyUSB class
driver) have no RP2040 dependencies and carry over unchanged. To port to
another pico-sdk board, change `PICO_BOARD` in `CMakeLists.txt` (or pass
`-DPICO_BOARD=...`); to port to a non pico-sdk MCU, swap this project's
`CMakeLists.txt`/`pico_sdk_import.cmake` for that target's TinyUSB build
integration.

See [`tusb_ump_loopback_din_bridge`](../tusb_ump_loopback_din_bridge) for
this same converter and DIN-bridging logic reused a second time, on a
second `tud_ump` interface, inside a single two-interface USB device.

## Building

Requires the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
(2.0.0 or later) and its ARM toolchain, either installed via the [Pico VS
Code extension](https://github.com/raspberrypi/pico-vscode) or with
`PICO_SDK_PATH` set manually.

```sh
cmake -S . -B build -G Ninja
cmake --build build
```

Flash the resulting `tusb_ump_uart_din_bridge.uf2` to your board.
