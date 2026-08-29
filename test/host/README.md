# Host-side unit tests

Regression tests for `tud_ump_read_impl()`'s alt-0 (legacy USB-MIDI1 byte
stream) read path, covering issues [#15](https://github.com/midi2-dev/tusb_ump/issues/15)
and [#18](https://github.com/midi2-dev/tusb_ump/issues/18):

1. **No buffer overflow** -- a dense stream of SysEx7 messages never yields
   more UMP words than requested, regardless of how many raw USB-MIDI1
   words each one takes to complete.
2. **No unnecessary stalling** -- a stream of 1-word messages (Channel
   Voice, System Common) fully drains a request rather than stopping 1
   word short "just in case" the next message needs 2.
3. **Split-call continuity** -- draining the same input across several
   small reads gives the same words, in the same order, as one large read.

No hardware required. Links the real `../../ump_device.cpp` and TinyUSB's
real (unmodified) `common/tusb_fifo.c` against a handful of stubs
(`stubs.c`) standing in for the rest of the USB device stack, plus two
test-only hooks in `ump_device.cpp`/`.h` guarded by `UMP_DEVICE_UNIT_TEST`
(`tud_ump_test_set_ep_out`, `tud_ump_test_rx_write`) that let a test mark an
interface "open" and inject raw bytes without a real USB enumeration.

## Running

```sh
make check
```

By default this builds against the TinyUSB copy vendored by the Pico SDK
(`~/.pico-sdk/sdk/<version>/lib/tinyusb`). Point at a different TinyUSB
checkout with:

```sh
make check TUSB_ROOT=/path/to/tinyusb
```

Any TinyUSB checkout works — only its portable `common/tusb_fifo.c` is
compiled, nothing MCU-specific.
