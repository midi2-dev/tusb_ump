/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2026 MIDI2.dev
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

#ifndef CFG_TUSB_MCU
  #error CFG_TUSB_MCU must be defined
#endif

#ifndef CFG_TUSB_OS
  #define CFG_TUSB_OS               OPT_OS_NONE
#endif

// NOTE: this define does nothing on its own -- the SDK's own
// hw/bsp/rp2040/family.cmake force-injects -DCFG_TUSB_DEBUG=<N> on the
// compiler command line (via tinyusb_common_base's target_compile_definitions),
// which wins over this header's #ifndef guard regardless of what's written
// here. To actually enable verbose logging, configure with `cmake -DLOG=1`
// (see CFG_TUH_LOG_LEVEL below for why 1, not 2).
#ifndef CFG_TUSB_DEBUG
  #define CFG_TUSB_DEBUG            0
#endif

// Deliberately 1, not the TinyUSB default of 2: hcd_rp2040.c's host-controller
// driver has TWO hardcoded TU_LOG(2, ...) calls ("Buffer complete"/"Transfer
// complete") INSIDE the actual USB interrupt handler (hcd_rp2040_irq()), fired
// on every single low-level USB buffer/packet event -- not just once per
// enumeration step like everything else. Blocking UART printf from inside
// that ISR is slow enough to disrupt time-critical USB control-transfer
// sequencing and can break enumeration (devices stalling silently at Set
// Configuration's status stage) if CFG_TUSB_DEBUG>=2 is used with
// `cmake -DLOG=2`. Since TU_LOG(2,...) requires CFG_TUSB_DEBUG>=2 to be
// anything other than a no-op, capping CFG_TUH_LOG_LEVEL at 1 lets this
// demo's own app/driver-level messages (TU_LOG_USBH in ump_host.cpp and
// usbh.c's own enumeration trace -- "Device configured", "opened", mount
// info, etc.) print via the always-available TU_LOG1 at CFG_TUSB_DEBUG=1,
// while those two ISR-context calls stay silent (they need level 2, which
// `-DLOG=1` never reaches). Don't raise CFG_TUSB_DEBUG to 2 without
// re-verifying enumeration reliability against real hardware first.
#ifndef CFG_TUH_LOG_LEVEL
  #define CFG_TUH_LOG_LEVEL         1
#endif

#ifndef CFG_TUSB_MEM_SECTION
  #define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
  #define CFG_TUSB_MEM_ALIGN        __attribute__ ((aligned(4)))
#endif

#ifndef CFG_TUH_MEM_SECTION
  #define CFG_TUH_MEM_SECTION
#endif

#ifndef CFG_TUH_MEM_ALIGN
  #define CFG_TUH_MEM_ALIGN         __attribute__ ((aligned(4)))
#endif

//--------------------------------------------------------------------
// HOST CONFIGURATION
//--------------------------------------------------------------------

#define CFG_TUH_ENABLED             1

// RP2040/RP2350 native USB controller; roothub port 0 for both device and
// host role (single controller, dual-role). VBUS 5V must be supplied to the
// downstream port for host mode to work -- see README.md's Hardware section.
#ifndef BOARD_TUH_RHPORT
  #define BOARD_TUH_RHPORT          0
#endif

#ifndef BOARD_TUH_MAX_SPEED
  #define BOARD_TUH_MAX_SPEED       OPT_MODE_FULL_SPEED
#endif

#define CFG_TUH_MAX_SPEED           BOARD_TUH_MAX_SPEED

// Size of buffer to hold descriptors and other data used during enumeration.
// Must be >= the largest attached device's full Configuration Descriptor
// wTotalLength -- some composite USB MIDI devices (e.g. a keyboard with
// extra HID controls) report 300+ bytes across several interfaces, which
// silently fails enumeration (a TU_ASSERT in usbh.c's
// ENUM_GET_FULL_CONFIG_DESC state) against a smaller buffer.
#define CFG_TUH_ENUMERATION_BUFSIZE 512

// Hub support: a device may be attached through a hub rather than directly
// to the root port. With CFG_TUH_HUB == 0, TinyUSB core reserves zero
// address slots for hub-class devices, so enumeration always fails for any
// device reporting bDeviceClass == Hub, trips a TU_ASSERT, and leaves dev0
// stuck "enumerating" -- silently wedging all future attach events until
// that hub is physically unplugged. RP2040's HCD (hcd_rp2040.c) needs no
// hub-specific changes for this: it addresses devices flatly by
// dev_addr/endpoint regardless of topology.
//
// Caveat: RP2040's native USB controller is Full-Speed-only hardware (no
// High-Speed PHY) -- BOARD_TUH_MAX_SPEED above isn't a preference, it's a
// hardware limit. A High-Speed-capable hub/device falls back to Full Speed
// against this host per standard USB behavior, and some devices present a
// materially reduced (or MIDI-less) descriptor set at Full Speed vs. their
// High-Speed configuration.
//
// 3 covers stacked/chained hubs (a hub plugged into another hub's
// downstream port to fan out beyond one hub's own port count), not just a
// single hub directly on the root port.
#define CFG_TUH_HUB                 3

// Up to 10 simultaneously-attached devices -- covers a hub plus headroom
// for stacked/chained hubs (see CFG_TUH_HUB above), to exercise multiple
// USB MIDI devices attached at once, not just hub traversal to one device.
// Hubs themselves draw addresses from the separate CFG_TUH_HUB-sized pool,
// so they don't count against this.
#define CFG_TUH_DEVICE_MAX          10

//--------------------------------------------------------------------
// UMP HOST CLASS DRIVER CONFIGURATION
//--------------------------------------------------------------------

// NOTE: CFG_TUH_MIDI is intentionally left undefined here. Enabling it
// would merge the AudioControl + MIDIStreaming interfaces into a single
// open()/set_config() call during enumeration (see ump_host.cpp's design
// notes), but TinyUSB core's own usbh.c references AUDIO_SUBCLASS_CONTROL /
// AUDIO_FUNC_PROTOCOL_CODE_UNDEF under that guard without including
// class/audio/audio.h itself, which fails to build (confirmed against
// pico-sdk 2.3.0's vendored TinyUSB -- a gap in TinyUSB core, not this
// driver). ump_host.cpp's umph_open()/umph_set_config() already handle the
// unmerged (AC-only-call then MS-call) enumeration shape correctly, so this
// is not a functional loss, just a simpler/safer config.
// #define CFG_TUH_MIDI             1

// 2 slots per device (see CFG_TUH_DEVICE_MAX above for the device count
// this is sized against): some composite devices expose an unrelated
// AudioControl interface (audio in/out) alongside their MIDI AC+MS pair --
// ump_host.cpp gives each distinct AC interface its own slot (see
// alloc_new_itf()/find_pending_ac_itf()) so the unrelated one doesn't
// clobber the real MIDI interface's state. Sized as 2 * CFG_TUH_DEVICE_MAX
// so every attached device can hit that worst case simultaneously.
#define CFG_TUH_UMP                 20
#define CFG_TUH_UMP_MAX_GTB         8

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_CONFIG_H_ */
