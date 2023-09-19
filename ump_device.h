/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2022 Michael Loh (AmeNote.com)
 * Copyright (c) 2022 Franz Detro (native-instruments.de)
 *
 * NOTE: Code adjustments made to support USB MIDI 2.0 UMP Packet format as
 * Alternate Interface 1. See USB Device Class Definition for MIDI Devices,
 * Version 2.0 - May 2, 2020.
 *
 * UMP Driver Version 0.2 - July 13, 2022
 * UMP Driver Version 0.2 - Dec. 13, 2022
 *  - Splitting UMP Driver base from tud_midi
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

#ifndef _TUSB_UMP_DEVICE_H_
#define _TUSB_UMP_DEVICE_H_

#include "class/audio/audio.h"
#include "ump.h"

//--------------------------------------------------------------------+
// Class Driver Configuration
//--------------------------------------------------------------------+

#ifndef CFG_TUD_UMP_EP_BUFSIZE
  #define CFG_TUD_UMP_EP_BUFSIZE     (TUD_OPT_HIGH_SPEED ? 512 : 64)
#endif

#ifdef __cplusplus
 extern "C" {
#endif

/** \addtogroup UMP
 *  @{
 *  \defgroup   UMP Device
 *  @{ */

//--------------------------------------------------------------------+
// Application API (Multiple Interfaces)
// CFG_TUD_UMP > 1
//--------------------------------------------------------------------+

// Check if UMP interface is mounted
bool     tud_ump_n_mounted      (uint8_t itf);

// Get the number of words (32 bits) available for reading
uint32_t tud_ump_n_available    (uint8_t itf );

// Get the number of words (32 bits) available for writing
uint32_t tud_ump_n_writeable    (uint8_t itf );

// Write UMP words
uint16_t tud_ump_write       ( uint8_t itf, uint32_t *words, uint16_t numWords );

// Read UMP words
uint16_t tud_ump_read        ( uint8_t itf, uint32_t *words, uint16_t numAvail );

//Get Alternate Setting
uint8_t tud_alt_setting( uint8_t itf);

//--------------------------------------------------------------------+
// Application Callback API (weak is optional)
//--------------------------------------------------------------------+
TU_ATTR_WEAK void tud_ump_rx_cb(uint8_t itf);

// Invoked when midi set interface request received
TU_ATTR_WEAK void tud_ump_set_itf_cb(uint8_t itf, uint8_t alt);

// Invoked when midi class specific get request received for an interface
TU_ATTR_WEAK bool tud_ump_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request);

//--------------------------------------------------------------------+
// Internal Class Driver API
//--------------------------------------------------------------------+
void     umpd_init            (void);
void     umpd_reset           (uint8_t rhport);
uint16_t umpd_open            (uint8_t rhport, tusb_desc_interface_t const * itf_desc, uint16_t max_len);
bool     umpd_control_xfer_cb (uint8_t rhport, uint8_t stage, tusb_control_request_t const * request);
bool     umpd_xfer_cb         (uint8_t rhport, uint8_t edpt_addr, xfer_result_t result, uint32_t xferred_bytes);

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_UMP_DEVICE_H_ */

/** @} */
/** @} */
