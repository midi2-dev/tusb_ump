# tusb_ump_loopback_din_bridge - two independent USB MIDI 2.0 (UMP) interfaces on one device

Demonstrates `tusb_ump`'s `ump_device` driver enumerating **two**
simultaneous, independently-addressable USB MIDI 2.0 (UMP) interfaces on a
single device (`CFG_TUD_UMP=2` in [`src/tusb_config.h`](src/tusb_config.h)):

- **`tud_ump` itf 0** - a raw UMP loopback, echoing back whatever it
  receives. Same behavior as [`../tusb_ump_lb`](../tusb_ump_lb).
- **`tud_ump` itf 1** - a DIN MIDI 1.0 bridge over hardware UART1 (GPIO4=TX,
  GPIO5=RX by default), using the same
  [`midi1_bytestream.h`](src/midi1_bytestream.h) /
  [`midi1_bytestream.c`](src/midi1_bytestream.c) converter and bridging
  approach as [`../tusb_ump_uart_din_bridge`](../tusb_ump_uart_din_bridge),
  just pointed at itf 1 instead of itf 0.

The two interfaces are fully separate USB MIDIStreaming functions (separate
Audio-Control+MIDIStreaming interface pairs, separate bulk endpoints,
separate Group Terminal Blocks) -- traffic on one never appears on the
other. A MIDI 2.0-aware host sees this as one device exposing two distinct
ports/function blocks: "Loopback" and "DIN Bridge".

## Why this exists

`ump_device.cpp`'s class driver has always supported multiple UMP interfaces
(every `tud_ump_*` call takes an `itf` index), but no example in this repo
exercised it until now. This is the concrete proof, and doubles as the
reference for the USB descriptor layout needed to declare a second
independent MIDIStreaming interface pair -- see the comments in
[`src/usb_descriptors.c`](src/usb_descriptors.c). This exact byte layout was
first validated on real hardware as `UUT/LoopbackDIN_Bridge` in the
[AmeNote_Protozoa](https://github.com/midi2-dev/AmeNote_Protozoa) repo
before being ported here.

Like `tusb_ump_lb` and `tusb_ump_uart_din_bridge`, this example does not
implement UMP Endpoint/Function Block Discovery - it's scoped to
demonstrating the transport and descriptor layout.

## Hardware

Default target is a Raspberry Pi Pico (RP2040). See
[`../tusb_ump_uart_din_bridge`'s README](../tusb_ump_uart_din_bridge/README.md#hardware)
for why the DIN port defaults to UART1/GPIO4-5 rather than UART0.

## Building

```sh
cmake -S . -B build -G Ninja
cmake --build build
```

Flash the resulting `tusb_ump_loopback_din_bridge.uf2` to your board.
