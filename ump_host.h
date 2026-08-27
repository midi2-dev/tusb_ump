/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2026 Michael Loh (AmeNote.com)
 *
 * NOTE: TinyUSB HOST class driver for USB MIDI 2.0 UMP. Supports both
 * Alternate Interface 0 (legacy USB MIDI 1.0 byte stream, translated
 * to/from UMP) and Alternate Interface 1 (native UMP passthrough). See
 * USB Device Class Definition for MIDI Devices, Version 2.0 - May 2, 2020.
 *
 * v1 scope is transport + Group Terminal Block descriptor discovery only
 * (mirrors the architecture of Microsoft's USBMIDI2 Windows driver): this
 * driver does not implement the UMP Stream-message handshake (Endpoint
 * Discovery / Function Block Discovery / Stream Configuration). That
 * negotiation, if needed, is expected to be layered on top as ordinary
 * UMP messages sent/received through tuh_ump_read()/tuh_ump_write().
 *
 * This driver targets the usbh_class_driver_t ABI introduced in current
 * TinyUSB (bool init/deinit, unconditional name field -- see
 * src/host/usbh_pvt.h). If building against a pre-0.17-era TinyUSB with the
 * older host class-driver ABI (void init(void), no deinit), a compatibility
 * shim will be needed -- see ump_device.cpp's USBD version-compat shim for
 * the pattern to follow.
 *
 * UMP Host Driver Version 0.2 - 2026-08-27
 *  - Enumeration, alt-setting selection, Group Terminal Block descriptor
 *    fetch/synthesis, and a full FIFO-backed UMP data pump for both
 *    alt-setting-1 (native UMP passthrough) and alt-setting-0 (legacy USB
 *    MIDI 1.0, translated to/from UMP including SysEx7 reassembly). See
 *    ump_host.cpp's version comment for the milestone-by-milestone history.
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

#ifndef _TUSB_UMP_HOST_H_
#define _TUSB_UMP_HOST_H_

#include "class/audio/audio.h"
#include "ump.h"

//--------------------------------------------------------------------+
// Class Driver Configuration
//--------------------------------------------------------------------+

// Number of concurrent UMP host interfaces (instances) supported.
#ifndef CFG_TUH_UMP
  #define CFG_TUH_UMP 1
#endif

#ifndef CFG_TUH_UMP_EP_BUFSIZE
  #define CFG_TUH_UMP_EP_BUFSIZE     (TUH_OPT_HIGH_SPEED ? 512 : 64)
#endif

#ifndef CFG_TUH_UMP_RX_BUFSIZE
  #define CFG_TUH_UMP_RX_BUFSIZE     CFG_TUH_UMP_EP_BUFSIZE
#endif

#ifndef CFG_TUH_UMP_TX_BUFSIZE
  #define CFG_TUH_UMP_TX_BUFSIZE     CFG_TUH_UMP_EP_BUFSIZE
#endif

// Maximum number of Group Terminal Block descriptor entries parsed/synthesized
// per interface. Bounds memory regardless of what a device claims in wTotalLength.
#ifndef CFG_TUH_UMP_MAX_GTB
  #define CFG_TUH_UMP_MAX_GTB        8
#endif

#ifdef __cplusplus
 extern "C" {
#endif

/** \addtogroup UMP
 *  @{
 *  \defgroup   UMP Host
 *  @{ */

//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+

// Check if a UMP interface is mounted (enumerated and ready for I/O)
bool     tuh_ump_mounted     (uint8_t daddr, uint8_t itf_num);

// Get the number of words (32 bits) available for reading
uint32_t tuh_ump_available   (uint8_t daddr, uint8_t itf_num);

// Get the number of words (32 bits) available for writing
uint32_t tuh_ump_writeable   (uint8_t daddr, uint8_t itf_num);

// Write UMP words. Legacy raw interface: words[] is reinterpreted onto the
// wire with no endian conversion (host-endian dependent).
uint16_t tuh_ump_write       (uint8_t daddr, uint8_t itf_num, uint32_t *words, uint16_t numWords);

// Write UMP words. Portable interface: words[] must be the host-native
// uint32_t numeric value of each UMP word (bits 31:28 = message type).
// Recommended for new code.
uint16_t tuh_ump_write_hton  (uint8_t daddr, uint8_t itf_num, uint32_t *words, uint16_t numWords);

// Read UMP words. Legacy raw interface: words are reinterpreted from the
// wire byte buffer with no endian conversion (host-endian dependent).
uint16_t tuh_ump_read        (uint8_t daddr, uint8_t itf_num, uint32_t *words, uint16_t numAvail);

// Read UMP words. Portable interface: each returned word is the host-native
// uint32_t whose arithmetic value matches the UMP wire word (bits 31:28 =
// message type). Recommended for new code.
uint16_t tuh_ump_read_ntoh   (uint8_t daddr, uint8_t itf_num, uint32_t *words, uint16_t numAvail);

// Get the currently active alternate setting (0 = legacy MIDI 1.0, 1 = native UMP)
uint8_t  tuh_ump_alt_setting (uint8_t daddr, uint8_t itf_num);

// Get the MIDIStreaming class-specific interface header's bcdMSC (class spec
// version, e.g. 0x0100) for the currently active alt setting. Returns 0 if
// the device had no CS interface header descriptor for that alt setting.
uint16_t tuh_ump_get_bcd_msc (uint8_t daddr, uint8_t itf_num);

// Get the parsed (or, for alt-setting-0-only devices, synthesized) Group
// Terminal Block entries for an interface. Returns the number of entries
// and, if gtb_array_out is non-NULL, points it at the internal array
// (valid until the interface is unmounted).
uint8_t  tuh_ump_get_group_terminal_blocks(uint8_t daddr, uint8_t itf_num,
                                            midi2_desc_group_terminal_block_t const **gtb_array_out);

//--------------------------------------------------------------------+
// Application Callback API (weak is optional)
//--------------------------------------------------------------------+

// Invoked when a UMP interface is mounted (enumeration complete)
TU_ATTR_WEAK void tuh_ump_mount_cb  (uint8_t daddr, uint8_t itf_num);

// Invoked when a UMP interface is unmounted (device disconnected)
TU_ATTR_WEAK void tuh_ump_umount_cb (uint8_t daddr, uint8_t itf_num);

// Invoked when new UMP data has been received
TU_ATTR_WEAK void tuh_ump_rx_cb     (uint8_t daddr, uint8_t itf_num);

// DIAGNOSTIC ONLY (milestone 1/2 bring-up): invoked with the raw bytes read off
// the IN endpoint, before any UMP/MIDI1 translation -- for alt-setting-0 devices
// this is raw 4-byte USB-MIDI1.0 CIN packets; for alt-setting-1 this is raw
// little-endian UMP words. `data` is only valid for the duration of the call.
// Superseded by tuh_ump_read()/tuh_ump_read_ntoh() once the real FIFO-backed
// data pump lands (see project plan milestones 3/4).
TU_ATTR_WEAK void tuh_ump_raw_rx_cb (uint8_t daddr, uint8_t itf_num, uint8_t const* data, uint16_t len);

//--------------------------------------------------------------------+
// Internal Class Driver API
//--------------------------------------------------------------------+
bool umph_init       (void);
bool umph_deinit     (void);
bool umph_open       (uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const * itf_desc, uint16_t max_len);
bool umph_set_config (uint8_t dev_addr, uint8_t itf_num);
bool umph_xfer_cb    (uint8_t dev_addr, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes);
void umph_close      (uint8_t dev_addr);

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_UMP_HOST_H_ */

/** @} */
/** @} */
