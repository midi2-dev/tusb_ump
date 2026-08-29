/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2023-2026 Michael Loh (AmeNote.com)
 *
 * NOTE: Code adjustments made to support USB MIDI 2.0 UMP Packet format as
 * Alternate Interface 1. See USB Device Class Definition for MIDI Devices,
 * Version 2.0 - May 2, 2020.
 *
 * UMP Driver Version 0.1 - June 28, 2022
 * UMP Driver Version 0.2 - Dec. 13, 2022
 *  - Splitting UMP Driver base from tud_midi
 * UMP Driver Version 0.3 - June 10, 2023
 *  - fixes issue with virtual cable ID and group IDs translation between
 *    USB MIDI 1.0 and USB MIDI 2.0
 * UMP Driver Version 0.4 - Sept. 4, 2023
 * - further fixes for multiple virtual cables when translating between
 *   USB MIDI 1.0 and USB MIDI 2.0. Remove dependance on external libraries.
 * UMP Driver Version 0.5 - Sept. 18, 2023
 * - bug fixes, USB MIDI 1.0 SYSEX on USB IN and USB OUT.
 * UMP Driver Version 1.0 - Sept. 26, 2024
 * - handling of USB MIDI 1.0 SYSEX translation
 * - Update of driver to latest tinyUSB implementation requirements
 *
 * The driver is backwards compatible with USB MIDI 1.0 if connected to an
 * operating system or other USB Hosting that does not support USB MIDI 2.0.
 * The driver does not currently support CIN 0xF, Single Byte as no known
 * operating system hosting is expected to send to device as CIN 0xf. If
 * CIN of 0xf is required, the implementer is welcome to contribute this
 * added processing.
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

#include "tusb_option.h"

#if (TUSB_OPT_DEVICE_ENABLED && CFG_TUD_UMP)

//--------------------------------------------------------------------+
// INCLUDE
//--------------------------------------------------------------------+
#include "device/usbd.h"
#include "device/usbd_pvt.h"

#include "tusb.h"
#include "ump_device.h"

//--------------------------------------------------------------------+
// TinyUSB API compatibility
//
// The ESP-IDF TinyUSB fork (arduino-esp32 3.x) reports version 0.18 but
// keeps pre-0.16 signatures (usbd_edpt_xfer: 4 args, no is_isr;
// tu_fifo_config: 5 args, with item_size) rather than the 0.17+ upstream
// ones. sdkconfig.h exists only in ESP-IDF builds, so it's used to detect
// the fork and pick the matching call shape below.
//--------------------------------------------------------------------+
#if defined(TUSB_VERSION_NUMBER) && TUSB_VERSION_NUMBER >= 1600 && \
    TUSB_VERSION_NUMBER < 1700 && !__has_include(<sdkconfig.h>)
  // Upstream TinyUSB 0.16 only (transient: added is_isr, removed item_size)
  #define _usbd_edpt_xfer(rh, ep, buf, len) \
      usbd_edpt_xfer((rh), (ep), (buf), (len), false)
  #define _tu_fifo_cfg(f, buf, depth, item_sz, ow) \
      tu_fifo_config((f), (buf), (depth), (ow))
#else
  // ESP-IDF TinyUSB fork OR upstream TinyUSB < 0.16 / >= 0.17
  #define _usbd_edpt_xfer(rh, ep, buf, len) \
      usbd_edpt_xfer((rh), (ep), (buf), (len))
  #define _tu_fifo_cfg(f, buf, depth, item_sz, ow) \
      tu_fifo_config((f), (buf), (depth), (item_sz), (ow))
#endif

// TUD_OPT_RHPORT: defined by the old CFG_TUSB_RHPORT*_MODE config style.
// TinyUSB 0.18+ on platforms like RP2040 use CFG_TUD_ENABLED instead,
// leaving TUD_OPT_RHPORT undefined. Single-port USB hardware is always port 0.
#ifndef TUD_OPT_RHPORT
  #define TUD_OPT_RHPORT 0
#endif

//--------------------------------------------------------------------+
// ENDIAN HELPERS
//--------------------------------------------------------------------+
// Two distinct byte orders are in play, don't conflate them:
//
// 1. INTERNAL (UMP_PACKET.umpData.umpBytes[]): this driver's own CIN
//    translation logic builds/reads messages with umpBytes[0] = the MT/group
//    byte, matching the UMP spec's logical (most-significant-byte-first)
//    view. UMP_HOST_BSWAP32 converts a host-native arithmetic uint32_t (MT in
//    bits 31:28) into/out of that layout, portably regardless of host
//    endianness.
//
// 2. WIRE (native alt-setting-1 passthrough, raw endpoint FIFO bytes): per
//    USB Device Class Definition for MIDI Devices v2.0 section 3.2.2, each
//    32-bit UMP word is sent least-significant-byte-first on the wire (byte 0
//    = LSB, byte 3 = MT/group). UMP_WIRE_BSWAP32 converts a host-native
//    arithmetic uint32_t into/out of that little-endian wire layout -- a
//    no-op on a little-endian host, a swap on a big-endian one.
//
// tud_ump_read()/tud_ump_write() keep this driver's original raw behavior
// (no conversion) for existing callers. Use tud_ump_read_ntoh()/
// tud_ump_write_hton() for portable, spec-correct behavior in new code.
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
  #define UMP_HOST_BSWAP32(x) (x)
  #define UMP_WIRE_BSWAP32(x) __builtin_bswap32(x)
#else
  #define UMP_HOST_BSWAP32(x) __builtin_bswap32(x)
  #define UMP_WIRE_BSWAP32(x) (x)
#endif

//--------------------------------------------------------------------+
// APP SPECIFIC DRIVERS
//--------------------------------------------------------------------+
#define TUSB_NUM_APP_DRIVERS 1  // defines number of app drivers
usbd_class_driver_t const tusb_app_drivers[TUSB_NUM_APP_DRIVERS] = {
  {
#if TUSB_VERSION_MAJOR == 0 && TUSB_VERSION_MINOR > 16
    "UMP",
#endif
    umpd_init,            // Driver init function
#if TUSB_VERSION_MAJOR == 0 && TUSB_VERSION_MINOR > 16
    umpd_deinit,          // Driver deinit function
#endif
    umpd_reset,           // Driver reset function
    umpd_open,            // Driver open function
    umpd_control_xfer_cb, // Driver control transfer callback function
    umpd_xfer_cb,         // Driver transfer callback function
    NULL                  // Driver sof function
  }
};

/**
 * @brief Routine to load app specific drivers
 * This routine is used for tinyUSB to associate to external application specific
 * drivers. This will be used until ump_device is included in the standard set of
 * tinyUSB drivers.
 * 
 * @param driver_count  Set with number of app specific drivers 
 * @return usbd_class_driver_t const* return pointer to structure
 */
usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t* driver_count)
{
  *driver_count = TUSB_NUM_APP_DRIVERS;

  return tusb_app_drivers;
}

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+
typedef struct
{
  uint8_t buffer[4];
  uint8_t index;
}midid_stream_t;

typedef struct
{
  uint8_t   wordCount;
  union ump_device
  {
    uint32_t  umpWords[4];
    uint8_t   umpBytes[sizeof(uint32_t)*4];
  } umpData;
} UMP_PACKET, *PUMP_PACKET;

//
// Structure to aid in UMP SYSEX to USB MIDI 1.0
//
#define SYSEX_BS_RB_SIZE 16
typedef struct UMP_TO_MIDI1_SYSEX_t
{
    bool    inSysex;
    uint8_t sysexBS[SYSEX_BS_RB_SIZE];
    uint8_t usbMIDI1Tail;
    uint8_t usbMIDI1Head;
} UMP_TO_MIDI1_SYSEX;

#define MAX_NUM_GROUPS_CABLES 16

typedef struct
{
  uint8_t itf_num;
  uint8_t ep_in;
  uint8_t ep_out;

  bool midi1IsInSysex[MAX_NUM_GROUPS_CABLES];
  UMP_TO_MIDI1_SYSEX midi1OutSysex[MAX_NUM_GROUPS_CABLES];

  /*------------- From this point, data is not cleared by bus reset -------------*/
  // FIFO
  tu_fifo_t rx_ff;                            // reference to rx fifo
  tu_fifo_t tx_ff;                            // reference to tx fifo
  uint8_t rx_ff_buf[CFG_TUD_UMP_RX_BUFSIZE];  // storage buffer for rx fifo
  uint8_t tx_ff_buf[CFG_TUD_UMP_TX_BUFSIZE];  // storage buffer for tx fifo

  #if CFG_FIFO_MUTEX
  osal_mutex_def_t rx_ff_mutex;               // mutex for rx fifo if needed
  osal_mutex_def_t tx_ff_mutex;               // mutex for tx fifo if needed
  #endif

  // Endpoint Transfer buffer
  CFG_TUSB_MEM_ALIGN uint8_t epout_buf[CFG_TUD_UMP_EP_BUFSIZE]; // temp endpoint storage buffer
  CFG_TUSB_MEM_ALIGN uint8_t epin_buf[CFG_TUD_UMP_EP_BUFSIZE];  // temp endpoint storage buffer

  // Selected Interface
  uint8_t ump_interface_selected;             // for interface seclection - needed for USB MIDI 2.0 (UMP)

} umpd_interface_t;

// Default Group Terminal Block Descriptor
static uint8_t default_ump_group_terminal_blk_desc[] =
{
  // header
  5, // bLength
  MIDI_1_CS_INTERFACE_GR_TRM_BLOCK,
  MIDI_GR_TRM_BLOCK_HEADER,
  U16_TO_U8S_LE(sizeof(midi2_cs_interface_desc_group_terminal_blocks_t)), // wTotalLength

  // block
  13, // bLength
  MIDI_1_CS_INTERFACE_GR_TRM_BLOCK,
  MIDI_GR_TRM_BLOCK,
  1,          // bGrpTrmBlkID
  0x00,       // bGrpTrmBlkType: bi-directional
  0x00,       // nGroupTrm
  1,          // nNumGroupTrm
  0,          // iBlockItem: no string
  0x00,       // bMIDIProtocol: Unknown (Use MIDI-CI)
  0x00, 0x00, // wMaxInputBandwidth: Unknown or Not Fixed
  0x00, 0x00  // wMaxOutputBandwidth: Unknown or Not Fixed
};

#define ITF_MEM_RESET_SIZE   offsetof(umpd_interface_t, rx_ff)

//--------------------------------------------------------------------+
// INTERNAL OBJECT & FUNCTION DECLARATION
//--------------------------------------------------------------------+
CFG_TUSB_MEM_SECTION umpd_interface_t _umpd_itf[CFG_TUD_UMP];

extern "C"
{
bool     tud_USBMIDI1ToUMP    (uint32_t usbMidi1Pkt, bool* pbIsInSysex, PUMP_PACKET umpPkt);

bool tud_ump_n_mounted (uint8_t itf)
{
  umpd_interface_t* ump = &_umpd_itf[itf];
  return ump->ep_in && ump->ep_out;
}

static void _prep_out_transaction (umpd_interface_t* p_ump)
{
  uint8_t const rhport = TUD_OPT_RHPORT;
  uint16_t available = tu_fifo_remaining(&p_ump->rx_ff);

  // Prepare for incoming data but only allow what we can store in the ring buffer.
  // TODO Actually we can still carry out the transfer, keeping count of received bytes
  // and slowly move it to the FIFO when read().
  // This pre-check reduces endpoint claiming
  TU_VERIFY(available >= sizeof(p_ump->epout_buf), );

  // claim endpoint
  TU_VERIFY(usbd_edpt_claim(rhport, p_ump->ep_out), );

  // fifo can be changed before endpoint is claimed
  available = tu_fifo_remaining(&p_ump->rx_ff);

  if ( available >= sizeof(p_ump->epout_buf) )  {
    _usbd_edpt_xfer(rhport, p_ump->ep_out, p_ump->epout_buf, sizeof(p_ump->epout_buf));
  }else
  {
    // Release endpoint since we don't make any transfer
    usbd_edpt_release(rhport, p_ump->ep_out);
  }
}

//--------------------------------------------------------------------+
// READ API
//--------------------------------------------------------------------+
uint32_t tud_ump_n_available(uint8_t itf)
{
  umpd_interface_t* ump = &_umpd_itf[itf];

  // is amount in fifo / 4 for 32 bit word count
  return tu_fifo_count(&ump->rx_ff) / 4;
}

/**
 * @brief return if MIDI UMP is enabled.
*
 * @param itf       interface number
 * @return bool true if enabled
 */
uint8_t tud_alt_setting( uint8_t itf) { 
    umpd_interface_t* ump = &_umpd_itf[itf];
    return ump->ump_interface_selected;
}

/**
 * @brief Process data read from USB OUT Stream and process into UMP data.
 * Read 32bit data words from USB stream, convert if necessary from USB MIDI 1.0
 * to UMP or pass UMP packets.
 *
 * @param itf       interface number
 * @param pkts      Array of 32 bit formatted UMP packet data.
 * @param numAvail  Number of 32 bit UMP words that are available in handle
 *                  to populate. A pending SYSEX-producing word (64-bit UMP
 *                  packet) is only converted once 2 words of space remain;
 *                  it is otherwise deferred to the next call rather than
 *                  overflowing the buffer.
 * @param hostOrder If true, each returned word is the host-native uint32_t
 *                  whose arithmetic value matches the UMP wire word (bits
 *                  31:28 = message type, etc, per the MIDI 2.0 UMP spec) --
 *                  safe to consume with bit-shifts/masks regardless of host
 *                  endianness. If false, words are returned via raw
 *                  byte-buffer reinterpretation with no endian conversion
 *                  (this driver's original, host-endian-dependent behavior).
 * @return uint16_t Number of 32 bit UMP words populated in handle
 */
static uint16_t tud_ump_read_impl( uint8_t itf, uint32_t *pkts, uint16_t numAvail, bool hostOrder )
{
  umpd_interface_t* ump = &_umpd_itf[itf];
  uint16_t numRead = 0;

  TU_VERIFY(ump->ep_out);

  // Determine if MIDI 1
  if (!ump->ump_interface_selected)
  {
    // Loop while there's at least 1 free output slot. A raw USB-MIDI1 word
    // converts to either 1 UMP word (Channel Voice, System Common) or 2 (a
    // 64-bit SYSEX7 packet), so peek the next word's CIN first and only
    // require 2 free slots when it's SYSEX-producing -- this bounds output
    // against numAvail without needlessly stalling on 1-word messages when
    // only 1 slot remains.
    while ((numAvail - numRead) >= 1)
    {
      uint8_t peekBuf[2];
      if (tu_fifo_peek_n(&ump->rx_ff, peekBuf, sizeof(peekBuf)) != sizeof(peekBuf))
      {
        goto END_READ;
      }

      // Mirror tud_USBMIDI1ToUMP()'s reassignment of a real-time System
      // message (CIN 15, data byte's high bit set) to CIN 5 (SYSEX_END_1BYTE)
      // so the word-count estimate here matches what conversion will do.
      // That reassigned case converts to a single 1-word UMP System message,
      // not the 2-word SysEx7 completion a literal CIN 5 produces, so it's
      // excluded from needsTwoWords below.
      uint8_t code_index = peekBuf[0] & 0x0f;
      bool isReassignedRealtime = false;
      if (code_index == MIDI_1_CIN_1BYTE_DATA && (peekBuf[1] & 0x80))
      {
        code_index = MIDI_1_CIN_SYSEX_END_1BYTE;
        isReassignedRealtime = true;
      }
      bool needsTwoWords = (code_index == MIDI_1_CIN_SYSEX_START)
                         || (code_index == MIDI_1_CIN_SYSEX_END_1BYTE && !isReassignedRealtime)
                         || (code_index == MIDI_1_CIN_SYSEX_END_2BYTE)
                         || (code_index == MIDI_1_CIN_SYSEX_END_3BYTE);
      if (needsTwoWords && (numAvail - numRead) < 2)
      {
        break;
      }

      // Get next word from USB
      uint32_t readWord;
      if (tu_fifo_read_n(&ump->rx_ff, (void *)&readWord, sizeof(uint32_t)) != sizeof(uint32_t) )
      {
        goto END_READ;
      }
      //readWord = RtlUlongByteSwap(readWord);

      if (readWord)
      {
        uint8_t *pBuffer = (uint8_t *)&readWord;
        uint8_t cbl_num = (pBuffer[0] & 0xf0) >> 4;
        UMP_PACKET pkt;
        if (tud_USBMIDI1ToUMP(readWord, &ump->midi1IsInSysex[cbl_num], &pkt))
        {
          for (uint8_t count = 0; count < pkt.wordCount; count++)
          {
            // pkt.umpData.umpBytes[] was built byte-by-byte (byte 0 = MT
            // nibble, matching wire order). hostOrder callers get the
            // arithmetic-correct uint32_t; legacy callers get the raw union
            // reinterpretation (host-endian dependent).
            uint32_t word = pkt.umpData.umpWords[count];
            pkts[numRead++] = hostOrder ? UMP_HOST_BSWAP32(word) : word;
          }
        }
      }
    }
  }
  else
  {
    uint8_t umpBuffer[4];
    // Read in as much data as possible
    while (numRead < numAvail &&
      tu_fifo_read_n(&ump->rx_ff, umpBuffer, 4) == 4)
    {
      if (hostOrder)
      {
        // Wire format is little-endian (byte 0 = LSB, byte 3 = MT nibble)
        // per USB MIDI 2.0 spec section 3.2.2; UMP_WIRE_BSWAP32 reconstructs
        // the host-native arithmetic value (MT in bits 31:28) regardless of
        // host endianness.
        pkts[numRead++] = UMP_WIRE_BSWAP32(*(uint32_t *)umpBuffer);
      }
      else
      {
        // Legacy raw reinterpretation (host-endian dependent).
        pkts[numRead++] = *(uint32_t *)umpBuffer;
      }
    }
  }

END_READ:
  _prep_out_transaction(ump);

  return numRead;
}

// Legacy raw interface: words are reinterpreted from the wire byte buffer
// with no endian conversion (host-endian dependent). Preserved for
// applications already built against this driver's existing behavior.
uint16_t tud_ump_read( uint8_t itf, uint32_t *pkts, uint16_t numAvail )
{
  return tud_ump_read_impl(itf, pkts, numAvail, false);
}

// Portable interface: each returned word is the host-native uint32_t whose
// arithmetic value matches the UMP wire word (bits 31:28 = message type),
// safe to consume with bit-shifts/masks regardless of host endianness.
// Recommended for new code.
uint16_t tud_ump_read_ntoh( uint8_t itf, uint32_t *pkts, uint16_t numAvail )
{
  return tud_ump_read_impl(itf, pkts, numAvail, true);
}

//--------------------------------------------------------------------+
// WRITE API
//--------------------------------------------------------------------+

uint32_t tud_ump_n_writeable(uint8_t itf)
{
  umpd_interface_t* ump = &_umpd_itf[itf];

  // is amount in fifo / 4 for 32 bit word count
  return tu_fifo_remaining(&ump->tx_ff) / 4;
}

static uint32_t write_flush(umpd_interface_t* ump)
{
  // No data to send
  if ( !tu_fifo_count(&ump->tx_ff) ) return 0;

  uint8_t const rhport = TUD_OPT_RHPORT;

  // skip if previous transfer not complete
  TU_VERIFY( usbd_edpt_claim(rhport, ump->ep_in), 0 );

  uint16_t count = tu_fifo_read_n(&ump->tx_ff, ump->epin_buf, CFG_TUD_UMP_EP_BUFSIZE);

  if (count)
  {
    TU_ASSERT( _usbd_edpt_xfer(rhport, ump->ep_in, ump->epin_buf, count), 0 );
    return count;
  }else
  {
    // Release endpoint since we don't make any transfer
    usbd_edpt_release(rhport, ump->ep_in);
    return 0;
  }
}

/**
 * @brief Process data write for USB IN Stream to host device.
 * Will write up to the number of UMP packets provided to the USB data stream.
 * If required, will convert to USB MIDI 1.0 stream format.
 * NOTE: This routine will translate to a single USB endpoint data message. Therefore
 * for optimization, it is suggested to group 32 bit UMP Words as much as possible.
 * Depending on if Full Speed or High Speed, the single transfer will be 64 bytes
 * or 512 bytes respectively - meaning 16 or 128 UMP words per transfer.
 *
 * @param itf       interface number
 * @param words     Pointer to 32 bit formatted UMP data array.
 * @param numWords   number of 32 bit UMP packets to try to write
 * @param hostOrder If true, each word in words[] must be the host-native
 *                  uint32_t numeric value of the UMP word (bits 31:28 =
 *                  message type, etc, per the MIDI 2.0 UMP spec) -- i.e.
 *                  built with bit-shifts/masks, not by casting a raw byte
 *                  buffer to uint32_t*. The driver converts to the correct
 *                  wire byte order internally, so callers do not need to
 *                  worry about host endianness. If false, words[] is written
 *                  via raw byte-buffer reinterpretation with no endian
 *                  conversion (this driver's original, host-endian-dependent
 *                  behavior).
 * @return uint16_t number of packets written
 */
static uint16_t tud_ump_write_impl( uint8_t itf, uint32_t *words, uint16_t numWords, bool hostOrder )
{
  umpd_interface_t* ump = &_umpd_itf[itf];
  TU_VERIFY(ump->ep_out);

  uint16_t    numProcessed = 0;
  bool        bEnterSysex;
  bool        bEndSysex;
  uint8_t     numberBytes;
  uint8_t     sysexStatus;
  static UMP_PACKET  umpPacket;
  static UMP_PACKET  umpWritePacket; // used as storage to translate to USB MIDI 1.0

  // As long as there is data to process and room to write into fifo
  while (numProcessed < numWords)
  {
        // Process UMP Packet
    // Convert to USB MIDI 1.0?
    if ( ump->ump_interface_selected != 1 )
    {
      umpPacket.wordCount = 0;

      // Determine size of UMP packet based on message type.
      // umpBytes[] below is accessed via this driver's internal layout
      // (byte 0 = MT nibble, see the ENDIAN HELPERS comment above); for
      // hostOrder callers words[] is numeric (MT nibble in the arithmetic
      // top bits) and needs UMP_HOST_BSWAP32 to convert into that internal
      // layout, legacy callers get the raw union reinterpretation
      // (host-endian dependent).
      umpPacket.umpData.umpWords[0] = hostOrder
        ? UMP_HOST_BSWAP32(words[numProcessed]) : words[numProcessed];

      switch (umpPacket.umpData.umpBytes[0] & UMP_MT_MASK)
      {
        case UMP_MT_UTILITY:
        case UMP_MT_SYSTEM:
        case UMP_MT_MIDI1_CV:
        case UMP_MT_RESERVED_6:
        case UMP_MT_RESERVED_7:
          umpPacket.wordCount = 1;
          break;

        case UMP_MT_DATA_64:
        case UMP_MT_MIDI2_CV:
        case UMP_MT_RESERVED_8:
        case UMP_MT_RESERVED_9:
        case UMP_MT_RESERVED_A:
          umpPacket.wordCount = 2;
          break;

        case UMP_MT_RESERVED_B:
        case UMP_MT_RESERVED_C:
          umpPacket.wordCount = 3;
          break;

        case UMP_MT_DATA_128:
        case UMP_MT_FLEX_128:
        case UMP_MT_STREAM_128:
        case UMP_MT_RESERVED_E:
          umpPacket.wordCount = 4;
          break;

        default:
          // Unhandled or corrupt data, force to move on
          numProcessed++;
          continue;
      }

      // Confirm have enough data for full packet
      if ((numWords - numProcessed) < umpPacket.wordCount)
      {
        // If not, let system populate more
        goto exitWrite;
      }
      // Get rest of words if needed for UMP Packet
      for (int count = 1; count < umpPacket.wordCount; count++)
      {
        umpPacket.umpData.umpWords[count] = hostOrder
          ? UMP_HOST_BSWAP32(words[numProcessed + count]) : words[numProcessed + count];
      }

      // Now that we have full UMP packet, need to convert to USB MIDI 1.0 format
      uint8_t cbl_num = umpPacket.umpData.umpBytes[0] & UMP_GROUP_MASK; // if used, cable num is group block num

      uint8_t mtVal = umpPacket.umpData.umpBytes[0] & UMP_MT_MASK;
      switch (mtVal)
      {
        case UMP_MT_SYSTEM:  // System Common messages
          umpWritePacket.wordCount = 1; // All types are single USB UMP 1.0 message
          // Now need to determine number of bytes for CIN
          switch (umpPacket.umpData.umpBytes[1])
          {
            case UMP_SYSTEM_TUNE_REQ:
            case UMP_SYSTEM_TIMING_CLK:
            case UMP_SYSTEM_START:
            case UMP_SYSTEM_CONTINUE:
            case UMP_SYSTEM_STOP:
            case UMP_SYSTEM_ACTIVE_SENSE:
            case UMP_SYSTEM_RESET:
            case UMP_SYSTEM_UNDEFINED_F4:
            case UMP_SYSTEM_UNDEFINED_F5:
            case UMP_SYSTEM_UNDEFINED_F9:
            case UMP_SYSTEM_UNDEFINED_FD:
              umpWritePacket.umpData.umpBytes[0] = (cbl_num << 4) | MIDI_1_CIN_SYSEX_END_1BYTE;
              break;

            case UMP_SYSTEM_MTC:
            case UMP_SYSTEM_SONG_SELECT:
              umpWritePacket.umpData.umpBytes[0] = (cbl_num << 4) | MIDI_1_CIN_SYSCOM_2BYTE;
              break;

            case UMP_SYSTEM_SONG_POS_PTR:
              umpWritePacket.umpData.umpBytes[0] = (cbl_num << 4) | MIDI_1_CIN_SYSCOM_3BYTE;
              break;

            default:
              umpWritePacket.wordCount = 0;
              break;
          }

          // Copy over actual data
          for (int count = 1; count < 4; count++)
          {
            umpWritePacket.umpData.umpBytes[count] = umpPacket.umpData.umpBytes[count];
          }
          break;

        case UMP_MT_MIDI1_CV:
          umpWritePacket.wordCount = 1;
          umpWritePacket.umpData.umpBytes[0] = (cbl_num << 4) | ((umpPacket.umpData.umpBytes[1] & 0xf0) >> 4);
          for (int count = 1; count < 4; count++)
          {
            umpWritePacket.umpData.umpBytes[count] = umpPacket.umpData.umpBytes[count];
          }
          break;

        // A UMP SysEx7 (64-bit data) packet carries up to 6 payload bytes, but a
        // USB-MIDI1 SysEx report is 4 bytes carrying at most 3 payload bytes, and
        // the two packet boundaries don't line up. sysexBS is a per-cable ring
        // buffer that decouples "bytes queued from this UMP packet" from "bytes
        // emittable in the next 4-byte report": bytes are pushed in below in
        // whatever count the UMP packet delivers, then drained 1-3 at a time
        // into successive USB-MIDI1 reports until empty.
        case UMP_MT_DATA_64:
          bEnterSysex = false;
          bEndSysex = false;

          umpWritePacket.wordCount = 0;

          // Determine if sysex will end after this message
          switch (umpPacket.umpData.umpBytes[1] & UMP_SYSEX7_STATUS_MASK)
          {
            case UMP_SYSEX7_COMPLETE:
              bEnterSysex = true;
            case UMP_SYSEX7_END:
              bEndSysex = true;
              break;

            case UMP_SYSEX7_START:
              bEnterSysex = true;
            default:
              bEndSysex = false;
              break;
          }

          if (bEnterSysex)
          {
            // Determine if believed already in Sysex and if so, reset converter
            if (ump->midi1OutSysex[cbl_num].inSysex)
            {
              ump->midi1OutSysex[cbl_num].inSysex = false;
            }
          }

          if (bEnterSysex && !ump->midi1OutSysex[cbl_num].inSysex)
          {
            ump->midi1OutSysex[cbl_num].usbMIDI1Head = 0;
            ump->midi1OutSysex[cbl_num].usbMIDI1Tail = 0;
            ump->midi1OutSysex[cbl_num].inSysex = true;
          }

          uint8_t byteStream[SYSEX_BS_RB_SIZE];
          sysexStatus = (umpPacket.umpData.umpBytes[1] >> 4);
          numberBytes = 0;

          if (sysexStatus <= 1 && numberBytes < SYSEX_BS_RB_SIZE)
          {
            byteStream[numberBytes++] = MIDI_1_STATUS_SYSEX_START;
          }
          for (uint8_t count = 0; count < (umpPacket.umpData.umpBytes[1] & 0xf); count++)
          {
            if (numberBytes < SYSEX_BS_RB_SIZE)
            {
              byteStream[numberBytes++] = umpPacket.umpData.umpBytes[2 + count];
            }
          }
          if ((sysexStatus == 0 || sysexStatus == 3) && numberBytes < SYSEX_BS_RB_SIZE)
          {
            byteStream[numberBytes++] = MIDI_1_STATUS_SYSEX_END;
          }

          // Move into sysex circular buffer queue
          for (uint8_t count = 0; count < numberBytes; count++)
          {
            ump->midi1OutSysex[cbl_num].sysexBS[ump->midi1OutSysex[cbl_num].usbMIDI1Head++]
                = byteStream[count];
            ump->midi1OutSysex[cbl_num].usbMIDI1Head %= SYSEX_BS_RB_SIZE;
          }

          // How many bytes available in BS
          numberBytes = (ump->midi1OutSysex[cbl_num].usbMIDI1Head > ump->midi1OutSysex[cbl_num].usbMIDI1Tail)
              ? ump->midi1OutSysex[cbl_num].usbMIDI1Head - ump->midi1OutSysex[cbl_num].usbMIDI1Tail
              : (SYSEX_BS_RB_SIZE - ump->midi1OutSysex[cbl_num].usbMIDI1Tail) + ump->midi1OutSysex[cbl_num].usbMIDI1Head;

          while (numberBytes && umpWritePacket.wordCount < 4)
          {
            umpWritePacket.umpData.umpWords[umpWritePacket.wordCount] = 0;

            if (numberBytes > 2)
            {
              uint8_t *pumpBytes = (uint8_t*) & umpWritePacket.umpData.umpWords[umpWritePacket.wordCount];
              for (uint8_t count = 0; count < 3; count++)
              {
                pumpBytes[count + 1] =
                  ump->midi1OutSysex[cbl_num].sysexBS[ump->midi1OutSysex[cbl_num].usbMIDI1Tail++];
                ump->midi1OutSysex[cbl_num].usbMIDI1Tail %= SYSEX_BS_RB_SIZE;
                numberBytes--;
              }
              // Mark cable number and CIN for start / continue SYSEX in USB MIDI 1.0 format
              if (bEndSysex && !numberBytes)
              {
                pumpBytes[0] = (uint8_t)(cbl_num << 4) | MIDI_1_CIN_SYSEX_END_3BYTE;
              }
              else
              {
                pumpBytes[0] = (uint8_t)(cbl_num << 4) | MIDI_1_CIN_SYSEX_START;
              }
            }
            else
            {
              // If less than two and we have a word to populate, check if the end of sysex
              if (bEndSysex)
              {
                // Process bytes
                uint8_t* pumpBytes = (uint8_t*)&umpWritePacket.umpData.umpWords[umpWritePacket.wordCount];
                uint8_t count;
                for (count = 0; numberBytes; count++)
                {
                  pumpBytes[count + 1] =
                    ump->midi1OutSysex[cbl_num].sysexBS[ump->midi1OutSysex[cbl_num].usbMIDI1Tail++];
                  ump->midi1OutSysex[cbl_num].usbMIDI1Tail %= SYSEX_BS_RB_SIZE;
                  numberBytes--;
                }
                // Mark cable number and CIN for start / continue SYSEX in USB MIDI 1.0 format
                switch (count)
                {
                  case 1:
                    pumpBytes[0] = (uint8_t)(cbl_num << 4) | MIDI_1_CIN_SYSEX_END_1BYTE;
                    break;

                  case 2:
                  default:
                    pumpBytes[0] = (uint8_t)(cbl_num << 4) | MIDI_1_CIN_SYSEX_END_2BYTE;
                    break;
                }
              }
              else
              {
                break;
              }
            }
            umpWritePacket.wordCount++;
          }
          break;

        default:
          // Not handled so ignore
          numProcessed += umpPacket.wordCount;    // ignore this UMP packet as corrupted
          umpWritePacket.wordCount = 0;
      }

      if (umpWritePacket.wordCount)
      {
        numProcessed += umpPacket.wordCount;
      
        tu_fifo_write_n(&ump->tx_ff, (void*)&umpWritePacket.umpData.umpBytes[0],
          umpWritePacket.wordCount*4);
      }
    }
    else
    {
      // Should already be UMP formatted, so just pass along.
      uint16_t numAvailable = tu_fifo_remaining(&ump->tx_ff) / 4;
      numProcessed = (numAvailable < numWords) ? numAvailable : numWords;
      if (hostOrder)
      {
        // words[] is numeric (host-native order, MT in bits 31:28); the
        // wire needs little-endian byte order (byte 0 = LSB, byte 3 = MT
        // nibble) per USB MIDI 2.0 spec section 3.2.2. Swap in fixed-size
        // batches and write each batch in one call rather than one
        // tu_fifo_write_n() per word.
        uint32_t wireWords[16];
        uint16_t offset = 0;
        uint16_t remaining = numProcessed;
        while (remaining)
        {
          uint16_t chunk = (remaining < TU_ARRAY_SIZE(wireWords)) ? remaining : TU_ARRAY_SIZE(wireWords);
          for (uint16_t count = 0; count < chunk; count++)
          {
            wireWords[count] = UMP_WIRE_BSWAP32(words[offset + count]);
          }
          tu_fifo_write_n(&ump->tx_ff, (void*)wireWords, chunk*4);
          offset += chunk;
          remaining -= chunk;
        }
      }
      else
      {
        // Legacy raw reinterpretation (host-endian dependent).
        tu_fifo_write_n(&ump->tx_ff, (void*)words, numProcessed*4);
      }
    }
  }

exitWrite :

  // Make sure fifo is pushed to endpoint
  write_flush(ump);

  // Let calling routine know how many words processed
  return numProcessed;
}

// Legacy raw interface: words[] is reinterpreted onto the wire with no
// endian conversion (host-endian dependent). Preserved for applications
// already built against this driver's existing behavior.
uint16_t tud_ump_write( uint8_t itf, uint32_t *words, uint16_t numWords )
{
  return tud_ump_write_impl(itf, words, numWords, false);
}

// Portable interface: words[] must be the host-native uint32_t numeric
// value of each UMP word (bits 31:28 = message type), built with
// bit-shifts/masks. The driver converts to correct wire byte order
// internally, so callers do not need to worry about host endianness.
// Recommended for new code.
uint16_t tud_ump_write_hton( uint8_t itf, uint32_t *words, uint16_t numWords )
{
  return tud_ump_write_impl(itf, words, numWords, true);
}

//--------------------------------------------------------------------+
// USBD Driver API
//--------------------------------------------------------------------+
void umpd_init(void)
{
  tu_memclr((void *)_umpd_itf, sizeof(_umpd_itf));

  for(uint8_t i=0; i<CFG_TUD_UMP; i++)
  {
    umpd_interface_t* ump = &_umpd_itf[i];

    // config fifo
    _tu_fifo_cfg(&ump->rx_ff, ump->rx_ff_buf, CFG_TUD_UMP_RX_BUFSIZE, 1, false);
    _tu_fifo_cfg(&ump->tx_ff, ump->tx_ff_buf, CFG_TUD_UMP_TX_BUFSIZE, 1, false);

    // Default select the first interface
    ump->ump_interface_selected = 0;

    #if CFG_FIFO_MUTEX
    tu_fifo_config_mutex(&ump->rx_ff, NULL, osal_mutex_create(&ump->rx_ff_mutex));
    tu_fifo_config_mutex(&ump->tx_ff, osal_mutex_create(&ump->tx_ff_mutex), NULL);
    #endif
  }
}

bool umpd_deinit(void)
{
  // Need to add to handle cleanup of multiple instances

  return true;
}

void umpd_reset(uint8_t rhport)
{
  (void) rhport;

  for(uint8_t i=0; i<CFG_TUD_UMP; i++)
  {
    umpd_interface_t* ump = &_umpd_itf[i];
    tu_memclr((void *)ump, ITF_MEM_RESET_SIZE);
    tu_fifo_clear(&ump->rx_ff);
    tu_fifo_clear(&ump->tx_ff);

    // Reset any current processing condition
    for(uint8_t grp=0; grp<MAX_NUM_GROUPS_CABLES; grp++)
    {
      ump->midi1IsInSysex[grp] = false;
      ump->midi1OutSysex[grp].inSysex = 0;
      ump->midi1OutSysex[grp].usbMIDI1Head = 0;
      ump->midi1OutSysex[grp].usbMIDI1Tail = 0;
    }

    ump->ump_interface_selected = 0;
  }
}

uint16_t umpd_open(uint8_t rhport, tusb_desc_interface_t const * desc_itf, uint16_t max_len)
{
  // 1st Interface is Audio Control v1
  TU_VERIFY(TUSB_CLASS_AUDIO               == desc_itf->bInterfaceClass    &&
            AUDIO_SUBCLASS_CONTROL         == desc_itf->bInterfaceSubClass &&
            AUDIO_FUNC_PROTOCOL_CODE_UNDEF == desc_itf->bInterfaceProtocol, 0);

  uint16_t drv_len = tu_desc_len(desc_itf);
  uint8_t const * p_desc = tu_desc_next(desc_itf);

  // Skip Class Specific descriptors
  while ( TUSB_DESC_CS_INTERFACE == tu_desc_type(p_desc) && drv_len <= max_len )
  {
    drv_len += tu_desc_len(p_desc);
    p_desc   = tu_desc_next(p_desc);
  }

  // 2nd Interface is MIDI Streaming
  TU_VERIFY(TUSB_DESC_INTERFACE == tu_desc_type(p_desc), 0);
  tusb_desc_interface_t const * desc_ump = (tusb_desc_interface_t const *) p_desc;

  TU_VERIFY(TUSB_CLASS_AUDIO               == desc_ump->bInterfaceClass    &&
            AUDIO_SUBCLASS_MIDI_STREAMING  == desc_ump->bInterfaceSubClass &&
            AUDIO_FUNC_PROTOCOL_CODE_UNDEF == desc_ump->bInterfaceProtocol, 0);

  // Find available interface
  umpd_interface_t * p_ump = NULL;
  for(uint8_t i=0; i<CFG_TUD_UMP; i++)
  {
    if ( _umpd_itf[i].ep_in == 0 && _umpd_itf[i].ep_out == 0 )
    {
      p_ump = &_umpd_itf[i];
      break;
    }
  }
  TU_ASSERT(p_ump);

  p_ump->itf_num = desc_ump->bInterfaceNumber;
  (void) p_ump->itf_num;

  // next descriptor
  drv_len += tu_desc_len(p_desc);
  p_desc   = tu_desc_next(p_desc);

  // Find and open endpoint descriptors
  uint8_t found_endpoints = 0;
  while ( (found_endpoints < desc_ump->bNumEndpoints) && (drv_len <= max_len)  )
  {
    if ( TUSB_DESC_ENDPOINT == tu_desc_type(p_desc) )
    {
      TU_ASSERT(usbd_edpt_open(rhport, (tusb_desc_endpoint_t const *) p_desc), 0);
      uint8_t ep_addr = ((tusb_desc_endpoint_t const *) p_desc)->bEndpointAddress;

      if (tu_edpt_dir(ep_addr) == TUSB_DIR_IN)
      {
        p_ump->ep_in = ep_addr;
      } else {
        p_ump->ep_out = ep_addr;
      }

      // Class Specific MIDI Stream endpoint descriptor
      drv_len += tu_desc_len(p_desc);
      p_desc   = tu_desc_next(p_desc);

      found_endpoints += 1;
    }

    drv_len += tu_desc_len(p_desc);
    p_desc   = tu_desc_next(p_desc);
  }

  // Finish off any further class specific definitions for interface
  while ( TUSB_DESC_CS_INTERFACE == tu_desc_type(p_desc) && drv_len <= max_len )
  {
    drv_len += tu_desc_len(p_desc);
    p_desc   = tu_desc_next(p_desc);
  }

  // See if there is an alternate interface for UMP USB MIDI 2.0 (bAlternateSetting 1,
  // same bInterfaceNumber) immediately following. A config descriptor can hold more
  // than one USB Function (e.g. this one plus an unrelated CDC/vendor Function), so
  // drv_len must reflect only what this interface's own descriptor block consumes --
  // walk it explicitly (same pattern as Alternate Setting 0 above) rather than
  // assuming nothing else follows, or TinyUSB's driver-dispatch loop will believe
  // umpd_open() consumed the whole descriptor and skip any Function after this one.
  // The alt setting's endpoint descriptors reuse Alt 0's addresses (USB MIDI 2.0
  // Alt-Setting model), so measure their length without calling usbd_edpt_open() again.
  while ( TUSB_DESC_INTERFACE == tu_desc_type(p_desc) && drv_len <= max_len )
  {
    tusb_desc_interface_t const * desc_alt = (tusb_desc_interface_t const *) p_desc;
    if ( desc_alt->bInterfaceNumber != desc_ump->bInterfaceNumber ) break; // next Function's interface, not an alt setting of this one

    drv_len += tu_desc_len(p_desc);
    p_desc   = tu_desc_next(p_desc);

    // Skip class-specific descriptors (MS Header etc)
    while ( TUSB_DESC_CS_INTERFACE == tu_desc_type(p_desc) && drv_len <= max_len )
    {
      drv_len += tu_desc_len(p_desc);
      p_desc   = tu_desc_next(p_desc);
    }

    // Skip this alt setting's endpoint descriptors (+ their class-specific descriptors)
    uint8_t alt_found_endpoints = 0;
    while ( (alt_found_endpoints < desc_alt->bNumEndpoints) && (drv_len <= max_len) )
    {
      if ( TUSB_DESC_ENDPOINT == tu_desc_type(p_desc) )
      {
        // Same endpoint address as Alternate Setting 0 -- already opened, do not reopen
        drv_len += tu_desc_len(p_desc);
        p_desc   = tu_desc_next(p_desc);
        alt_found_endpoints += 1;
      }

      drv_len += tu_desc_len(p_desc);
      p_desc   = tu_desc_next(p_desc);
    }

    // Finish off any further class specific definitions for this alt setting
    while ( TUSB_DESC_CS_INTERFACE == tu_desc_type(p_desc) && drv_len <= max_len )
    {
      drv_len += tu_desc_len(p_desc);
      p_desc   = tu_desc_next(p_desc);
    }
  }

  // Prepare for incoming data
  _prep_out_transaction(p_ump);

  return drv_len;
}

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool umpd_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
  // nothing to with DATA & ACK stage
  if (stage != CONTROL_STAGE_SETUP) return true;

  // Interface-addressed requests (SET_INTERFACE, GTB GET_DESCRIPTOR) carry the
  // target interface number in wIndex, not the controller/root-hub-port index --
  // resolve the matching UMP instance by itf_num, same pattern umpd_xfer_cb()
  // already uses (matched by endpoint address there instead). This only matters
  // once CFG_TUD_UMP > 1: with a single instance any indexing scheme happens to
  // resolve to it.
  uint8_t itf_num = tu_u16_low(request->wIndex);
  umpd_interface_t* ump = NULL;
  for (uint8_t i = 0; i < CFG_TUD_UMP; i++)
  {
    if (_umpd_itf[i].itf_num == itf_num)
    {
      ump = &_umpd_itf[i];
      break;
    }
  }
  TU_VERIFY(ump);

  switch ( request->bRequest )
  {
    case TUSB_REQ_SET_INTERFACE :
      // Set the interface type for driver operaiton
      ump->ump_interface_selected = tu_u16_low(request->wValue);

      // As we are using still bulk transfer, no reason to close and open endpoints, however we should clear
      // fifos to start from scratch
      tu_fifo_clear(&ump->rx_ff);
      tu_fifo_clear(&ump->tx_ff);

      // invoke set interface callback if available
      if (tud_ump_set_itf_cb) tud_ump_set_itf_cb(tu_u16_low(request->wIndex), ump->ump_interface_selected);

      tud_control_status(rhport, request); // send a status zero length packet

      return true;

    case TUSB_REQ_GET_DESCRIPTOR :
      if ( request->wValue == 0x2601 ) //0x26 - CS_GR_TRM_BLOCK 0x01 - alternate interface setting
      {
        // invoke midi class specific get request callback if available
        if (tud_ump_get_req_itf_cb && tud_ump_get_req_itf_cb(rhport, request)) return true;

        // return default group block descriptor if not handled by client code
        uint16_t length = request->wLength;
        if ( length > sizeof( default_ump_group_terminal_blk_desc ) )
        {
          length = sizeof( default_ump_group_terminal_blk_desc );
        }
        tud_control_xfer(rhport, request, (void *)default_ump_group_terminal_blk_desc, length );
        return true;
      }
      else
        return false;

    default :
      return false;
  }
}

bool umpd_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
  (void) result;
  (void) rhport;

  uint8_t itf;
  umpd_interface_t* p_ump;

  // Identify which interface to use
  for (itf = 0; itf < CFG_TUD_UMP; itf++)
  {
    p_ump = &_umpd_itf[itf];
    if ( ( ep_addr == p_ump->ep_out ) || ( ep_addr == p_ump->ep_in ) ) break;
  }
  TU_ASSERT(itf < CFG_TUD_UMP);

  // receive new data
  if ( ep_addr == p_ump->ep_out )
  {
    tu_fifo_write_n(&p_ump->rx_ff, p_ump->epout_buf, xferred_bytes);

    // invoke receive callback if available
    if (tud_ump_rx_cb) tud_ump_rx_cb(itf);

    // prepare for next
    // TODO for now ep_out is not used by public API therefore there is no race condition,
    // and does not need to claim like ep_in
    _prep_out_transaction(p_ump);
  }
  else if ( ep_addr == p_ump->ep_in )
  {
    if (0 == write_flush(p_ump))
    {
      // If there is no data left, a ZLP should be sent if
      // xferred_bytes is multiple of EP size and not zero
      if ( !tu_fifo_count(&p_ump->tx_ff) && xferred_bytes && (0 == (xferred_bytes % CFG_TUD_UMP_EP_BUFSIZE)) )
      {
        if ( usbd_edpt_claim(rhport, p_ump->ep_in) )
        {
          _usbd_edpt_xfer(rhport, p_ump->ep_in, NULL, 0);
        }
      }
    }
  }

  return true;
}

/**
 * Convert one USB-MIDI1 32-bit word packet to a UMP packet. Populates only
 * UMP message types 1 (System), 2 (MIDI 1.0 Channel Voice), and 3 (64-bit
 * data / SysEx7); callers needing MIDI 2.0 Channel Voice must convert
 * separately.
 *
 * Refined from the USB MIDI 2.0 Host Driver contributed to Windows by the
 * Association of Musical Electronics Industry and Microsoft; driver source
 * further developed by AmeNote.
 *
 * @param usbMidi1Pkt  The USB MIDI 1.0 packet as a 32-bit word.
 * @param pbIsInSysex  In/out SysEx state for this USB-MIDI1 data stream;
 *                     caller owns and persists it across calls, starting false.
 * @param umpPkt       Output UMP packet. Data beyond wordCount is left
 *                     unwritten -- callers must not read past wordCount.
 * @return bool        true if umpPkt was populated, false otherwise.
 */
bool
tud_USBMIDI1ToUMP(
    uint32_t        usbMidi1Pkt,
    bool*           pbIsInSysex,
    PUMP_PACKET     umpPkt
)
{
    // Checked passed parameters
    if (!usbMidi1Pkt || !pbIsInSysex || !umpPkt)
    {
        return false;
    }

    uint8_t* pBuffer = (uint8_t*)&usbMidi1Pkt;
    umpPkt->wordCount = 0;

    // Determine packet cable number from group
    uint8_t cbl_num = (pBuffer[0] & 0xf0) >> 4;

    // USB MIDI 1.0 uses a CIN as an identifier for packet, grab CIN.
    uint8_t code_index = pBuffer[0] & 0x0f;

    // Handle special case of single byte data
    if (code_index == MIDI_1_CIN_1BYTE_DATA && (pBuffer[1] & 0x80))
    {
        switch (pBuffer[1])
        {
        case UMP_SYSTEM_TUNE_REQ:
        case UMP_SYSTEM_TIMING_CLK:
        case UMP_SYSTEM_START:
        case UMP_SYSTEM_CONTINUE:
        case UMP_SYSTEM_STOP:
        case UMP_SYSTEM_ACTIVE_SENSE:
        case UMP_SYSTEM_RESET:
        case UMP_SYSTEM_UNDEFINED_F4:
        case UMP_SYSTEM_UNDEFINED_F5:
        case UMP_SYSTEM_UNDEFINED_F9:
        case UMP_SYSTEM_UNDEFINED_FD:
            code_index = MIDI_1_CIN_SYSEX_END_1BYTE;
            break;

        default:
            break;
        }
    }

    uint8_t firstByte = 1;
    uint8_t lastByte = 4;
    uint8_t copyPos;

    switch (code_index)
    {
    case MIDI_1_CIN_SYSEX_START: // or continue
 
        if (!*pbIsInSysex)
        {
            // SYSEX Start means first byte should be SYSEX start
            if (pBuffer[1] != MIDI_1_STATUS_SYSEX_START) return false;
            firstByte = 2;
            lastByte = 4;

            // As this is start of SYSEX, need to set status to indicate so and copy 2 bytes of data
            // as first byte of MIDI_1_STATUS_SYSEX_START
            umpPkt->umpData.umpBytes[1] = UMP_SYSEX7_START | 2;

            // Set that in SYSEX
            *pbIsInSysex = true;
        }
        else
        {
            firstByte = 1;
            lastByte = 4;

            // As this is in SYSEX, then need to indicate continue
            umpPkt->umpData.umpBytes[1] = UMP_SYSEX7_CONTINUE | 3;
        }

        // Capture Cable number
        umpPkt->umpData.umpBytes[0] = UMP_MT_DATA_64 | cbl_num;   // Message Type and group

        umpPkt->wordCount = 2;
        // Transfer in bytes
        copyPos = firstByte;
        for (uint8_t count = 2; count < 8; count++)
        {
            umpPkt->umpData.umpBytes[count] = (copyPos < lastByte)
                ? pBuffer[copyPos++] : 0x00;
        }
        break;

    case MIDI_1_CIN_SYSEX_END_1BYTE: // or single byte System Common
        // Determine if a system common
        if ( (pBuffer[1] & 0x80) // most significant bit set and not sysex ending
            && (pBuffer[1] != MIDI_1_STATUS_SYSEX_END))
        {
            // A single-byte System message is one 32-bit UMP word (MT=1),
            // not a 64-bit SysEx7 packet -- fill and pad it directly rather
            // than falling into the 2-word SysEx completion path below.
            umpPkt->umpData.umpBytes[0] = UMP_MT_SYSTEM | cbl_num;
            umpPkt->umpData.umpBytes[1] = pBuffer[1];
            umpPkt->umpData.umpBytes[2] = 0x00;
            umpPkt->umpData.umpBytes[3] = 0x00;
            umpPkt->wordCount = 1;
            break;
        }

        umpPkt->umpData.umpBytes[0] = UMP_MT_DATA_64 | cbl_num;

        // Determine if complete based on if currently in SYSEX
        if (*pbIsInSysex)
        {
            if (pBuffer[1] != MIDI_1_STATUS_SYSEX_END) return false;
            umpPkt->umpData.umpBytes[1] = UMP_SYSEX7_END | 0;
            *pbIsInSysex = false; // we are done with SYSEX
            firstByte = 1;
            lastByte = 1;
        }
        else
        {
            // should not get here
            return false;
        }

        umpPkt->wordCount = 2;
        // Transfer in bytes
        copyPos = firstByte;
        for (uint8_t count = 2; count < 8; count++)
        {
            umpPkt->umpData.umpBytes[count] = (copyPos < lastByte)
                ? pBuffer[copyPos++] : 0x00;
        }
        break;

    case MIDI_1_CIN_SYSEX_END_2BYTE:
        umpPkt->umpData.umpBytes[0] = UMP_MT_DATA_64 | cbl_num;

        // Determine if complete based on if currently in SYSEX
        if (*pbIsInSysex)
        {
            if (pBuffer[2] != MIDI_1_STATUS_SYSEX_END) return false;
            umpPkt->umpData.umpBytes[1] = UMP_SYSEX7_END | 1;
            *pbIsInSysex = false; // we are done with SYSEX
            firstByte = 1;
            lastByte = 2;
        }
        else
        {
            umpPkt->umpData.umpBytes[1] = UMP_SYSEX7_COMPLETE | 0;
            *pbIsInSysex = false; // we are done with SYSEX
            firstByte = 1;
            lastByte = 1;
        }

        umpPkt->wordCount = 2;
        // Transfer in bytes
        copyPos = firstByte;
        for (uint8_t count = 2; count < 8; count++)
        {
            umpPkt->umpData.umpBytes[count] = (copyPos < lastByte)
                ? pBuffer[copyPos++] : 0x00;
        }
        break;

    case MIDI_1_CIN_SYSEX_END_3BYTE:
        umpPkt->umpData.umpBytes[0] = UMP_MT_DATA_64 | cbl_num;

        // Determine if complete based on if currently in SYSEX
        if (*pbIsInSysex)
        {
            if (pBuffer[3] != MIDI_1_STATUS_SYSEX_END) return false;
            umpPkt->umpData.umpBytes[1] = UMP_SYSEX7_END | 2;
            *pbIsInSysex = false; // we are done with SYSEX
            firstByte = 1;
            lastByte = 3;
        }
        else
        {
            if (pBuffer[1] != MIDI_1_STATUS_SYSEX_START || pBuffer[3] != MIDI_1_STATUS_SYSEX_END) return false;
            umpPkt->umpData.umpBytes[1] = UMP_SYSEX7_COMPLETE | 1;
            *pbIsInSysex = false; // we are done with SYSEX
            firstByte = 2;
            lastByte = 3;
        }

        umpPkt->wordCount = 2;
        // Transfer in bytes
        copyPos = firstByte;
        for (uint8_t count = 2; count < 8; count++)
        {
            umpPkt->umpData.umpBytes[count] = (copyPos < lastByte)
                ? pBuffer[copyPos++] : 0x00;
        }
        break;

        // MIDI1 Channel Voice Messages
    case MIDI_1_CIN_NOTE_ON:
    case MIDI_1_CIN_NOTE_OFF:
    case MIDI_1_CIN_POLY_KEYPRESS:
    case MIDI_1_CIN_CONTROL_CHANGE:
    case MIDI_1_CIN_PROGRAM_CHANGE:
    case MIDI_1_CIN_CHANNEL_PRESSURE:
    case MIDI_1_CIN_PITCH_BEND_CHANGE:
        umpPkt->umpData.umpBytes[0] = UMP_MT_MIDI1_CV | cbl_num; // message type 2
        *pbIsInSysex = false; // ensure we end any current sysex packets, other layers need to handle error

        // Copy in rest of data
        for (int count = 1; count < 4; count++)
        {
            umpPkt->umpData.umpBytes[count] = pBuffer[count];
        }

        umpPkt->wordCount = 1;
        break;

    case MIDI_1_CIN_SYSCOM_2BYTE:
    case MIDI_1_CIN_SYSCOM_3BYTE:
        umpPkt->umpData.umpBytes[0] = UMP_MT_SYSTEM | cbl_num;
        for (int count = 1; count < 4; count++)
        {
            umpPkt->umpData.umpBytes[count] = pBuffer[count];
        }
        umpPkt->wordCount = 1;
        break;

    case MIDI_1_CIN_MISC:
    case MIDI_1_CIN_CABLE_EVENT:
        // These are reserved for future use and will not be translated, drop data with no processing
    default:
        // Not valid USB MIDI 1.0 transfer or NULL, skip
        return false;
    }

    return true;
}

#ifdef UMP_DEVICE_UNIT_TEST
void tud_ump_test_set_ep_out(uint8_t itf, uint8_t ep_out)
{
    _umpd_itf[itf].ep_out = ep_out;
}

uint16_t tud_ump_test_rx_write(uint8_t itf, const uint8_t* data, uint16_t n)
{
    return tu_fifo_write_n(&_umpd_itf[itf].rx_ff, data, n);
}
#endif

} // extern "C"
#endif
