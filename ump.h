/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2023-2026 Michael Loh (AmeNote.com)
 * Copyright (c) 2022 Franz Detro (native-instruments.de)
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
 *
 * This file is part of the TinyUSB stack.
 */

/** \ingroup group_class
 *  \defgroup ClassDriver_UMP MIDI/UMP Device Class
 *  @{ */

#ifndef _TUSB_UMP_H__
#define _TUSB_UMP_H__

//#include "common/tusb_common.h"

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------+
// UMP Protocol definitions
//--------------------------------------------------------------------+
// Message Types
#define UMP_MT_MASK       0xf0
#define UMP_MT_UTILITY    0x00
#define UMP_MT_SYSTEM     0x10
#define UMP_MT_MIDI1_CV   0x20
#define UMP_MT_DATA_64    0x30
#define UMP_MT_MIDI2_CV   0x40
#define UMP_MT_DATA_128   0x50
#define UMP_MT_RESERVED_6 0x60 // 32bits reserved future
#define UMP_MT_RESERVED_7 0x70 // 32bits reserved future
#define UMP_MT_RESERVED_8 0x80 // 64bits reserved future
#define UMP_MT_RESERVED_9 0x90 // 64bits reserved future
#define UMP_MT_RESERVED_A 0xA0 // 64bits reserved future
#define UMP_MT_RESERVED_B 0xB0 // 96bits reserved future
#define UMP_MT_RESERVED_C 0xC0 // 96bits reserved future
#define UMP_MT_FLEX_128   0xd0
#define UMP_MT_RESERVED_E 0xE0 // 128bits reserved future
#define UMP_MT_STREAM_128 0xf0

// Group Number
#define UMP_GROUP_MASK    0x0f

// System Exclusive 7-Bit Status
#define UMP_SYSEX7_STATUS_MASK  0xf0
#define UMP_SYSEX7_COMPLETE     0x00
#define UMP_SYSEX7_START        0x10
#define UMP_SYSEX7_CONTINUE     0x20
#define UMP_SYSEX7_END          0x30
#define UMP_SYSEX7_SIZE_MASK    0x0f

// System Common Status
#define UMP_SYSTEM_MTC          0xf1  // 2 bytes incl status
#define UMP_SYSTEM_SONG_POS_PTR 0xf2  // 3 bytes incl status
#define UMP_SYSTEM_SONG_SELECT  0xf3  // 2 bytes incl status
#define UMP_SYSTEM_UNDEFINED_F4 0xf4  // undefined
#define UMP_SYSTEM_UNDEFINED_F5 0xf5  // undefined
#define UMP_SYSTEM_TUNE_REQ     0xf6  // status byte only
#define UMP_SYSTEM_TIMING_CLK   0xf8  // status byte only
#define UMP_SYSTEM_UNDEFINED_F9 0xf9  // undefined
#define UMP_SYSTEM_START        0xfa  // status byte only
#define UMP_SYSTEM_CONTINUE     0xfb  // status byte only
#define UMP_SYSTEM_STOP         0xfc  // status byte only
#define UMP_SYSTEM_UNDEFINED_FD 0xfd  // undefined
#define UMP_SYSTEM_ACTIVE_SENSE 0xfe  // status byte only
#define UMP_SYSTEM_RESET        0xff  // status byte only

//--------------------------------------------------------------------+
// ENDIAN HELPERS
//--------------------------------------------------------------------+
// Two distinct byte-order concerns apply to UMP words -- do not conflate them.
// Shared here so both the device (ump_device.cpp) and host (ump_host.cpp)
// drivers use one definition instead of risking drift.
//
// 1. INTERNAL representation: driver code that extracts/builds message
//    fields (e.g. the alt-setting-0 <-> alt-setting-1 MIDI 1.0 CIN
//    translation switch/case logic) reads/builds messages assuming byte 0
//    holds the MT/group byte, matching the UMP spec's logical/diagram view
//    (most-significant byte first). UMP_HOST_BSWAP32 converts a host-native
//    arithmetic uint32_t (MT in bits 31:28) into/out of that internal
//    layout, portably regardless of host endianness.
//
// 2. WIRE byte order (raw bytes read from or written to the USB endpoint
//    FIFOs for native alt-setting-1 passthrough): per the USB Device Class
//    Definition for MIDI Devices v2.0, section 3.2.2 "UMP Messages in a USB
//    Packet: Byte Ordering" -- "Each 32 bit word of a Universal MIDI Packet
//    is sent with the least significant byte first" -- confirmed against
//    real USB captures of a spec-compliant host (byte 0 on the wire is the
//    word's LSB, byte 3 is the MT/group byte). UMP_WIRE_BSWAP32 converts a
//    host-native arithmetic uint32_t into/out of that little-endian wire
//    layout: a no-op on a little-endian host (its native memory layout
//    already matches), a swap on a big-endian host.
//
// UMP_WIRE_BSWAP32 must be applied only at the FIFO/endpoint transfer
// boundary -- never inside MIDI1<->UMP translation math, which stays in the
// UMP_HOST_BSWAP32 internal logical layout throughout. Conflating the two
// was the root cause of a real bug (see commit c090bbe, "Fix UMP wire byte
// order for native alt-setting-1 passthrough").
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
  #define UMP_HOST_BSWAP32(x) (x)
  #define UMP_WIRE_BSWAP32(x) __builtin_bswap32(x)
#else
  #define UMP_HOST_BSWAP32(x) __builtin_bswap32(x)
  #define UMP_WIRE_BSWAP32(x) (x)
#endif

//--------------------------------------------------------------------+
// MIDI 2.0 Channel Voice -> USB MIDI 1.0 (alternate setting 0)
//--------------------------------------------------------------------+
// With the USB MIDI 1.0 alternate selected, both drivers reformat MIDI 1.0
// UMP (MT 1, 2, 3) into USB-MIDI 1.0 event packets. A MIDI 2.0 Channel Voice
// message (MT 4) has no USB-MIDI 1.0 form, and used to be discarded -- so a
// MIDI 1.0 device received nothing at all from a sender using the MIDI 2.0
// Protocol. With this enabled (the default) the drivers apply the MIDI 2.0
// Specification's default translation (M2-104-UM, "Translation of MIDI 2.0
// Channel Voice Messages to MIDI 1.0") before reformatting:
//
//   Note On/Off, Poly Pressure, Control Change, Channel Pressure: values
//     scaled down by taking their most significant bits; a Note On whose
//     velocity scales to 0 is sent with velocity 1, since 0 means Note Off.
//   Pitch Bend: 32 -> 14 bits.
//   Program Change: preceded by Bank Select MSB/LSB (CC 0 / CC 32) when the
//     Bank Valid flag is set.
//   Registered / Assignable Controller (RPN / NRPN): CC 101/100 or 99/98 to
//     select, then Data Entry MSB/LSB (CC 6 / CC 38), 14-bit value.
//   Per-note controllers, per-note pitch bend, per-note management and the
//     relative RPN/NRPN forms have no MIDI 1.0 equivalent and are dropped.
//
// Define CFG_UMP_MIDI2_TO_MIDI1 to 0 to restore the old behaviour (MT 4
// discarded on the MIDI 1.0 alternate).
#ifndef CFG_UMP_MIDI2_TO_MIDI1
  #define CFG_UMP_MIDI2_TO_MIDI1 1
#endif

/** Translate one MIDI 2.0 Channel Voice UMP into USB-MIDI 1.0 event packets.
 *
 * @param ump   the 8-byte UMP in the drivers' internal layout: byte 0 holds
 *              the MT/group nibbles, byte 1 opcode/channel, bytes 4-7 the data
 *              word most significant byte first
 * @param cable USB-MIDI cable number to put in each packet
 * @param out   16 bytes: up to four 4-byte USB-MIDI 1.0 event packets
 * @return      the number of packets written, 0 if the message has no MIDI 1.0
 *              equivalent (the caller consumes and drops it)
 */
static inline uint8_t ump_midi2cv_to_usbmidi1(const uint8_t ump[8], uint8_t cable, uint8_t out[16])
{
  const uint8_t op  = (uint8_t)(ump[1] >> 4);
  const uint8_t ch  = (uint8_t)(ump[1] & 0x0f);
  const uint8_t hi7 = (uint8_t)(ump[4] >> 1);                         // top 7 bits of the data word
  const uint16_t v14 = (uint16_t)(((uint16_t)ump[4] << 6) | (ump[5] >> 2)); // top 14 bits
  uint8_t n = 0;

  #define UMP_M1_EMIT(status, d1, d2) do {                              \
      out[n * 4 + 0] = (uint8_t)((cable << 4) | ((status) >> 4));       \
      out[n * 4 + 1] = (uint8_t)(status);                               \
      out[n * 4 + 2] = (uint8_t)((d1) & 0x7f);                          \
      out[n * 4 + 3] = (uint8_t)((d2) & 0x7f);                          \
      n++;                                                              \
    } while (0)

  switch (op)
  {
    case 0x9: UMP_M1_EMIT(0x90 | ch, ump[2], hi7 ? hi7 : 1); break;   // Note On (velocity 0 -> 1)
    case 0x8: UMP_M1_EMIT(0x80 | ch, ump[2], hi7);          break;   // Note Off
    case 0xA: UMP_M1_EMIT(0xA0 | ch, ump[2], hi7);          break;   // Poly Pressure
    case 0xB: UMP_M1_EMIT(0xB0 | ch, ump[2], hi7);          break;   // Control Change
    case 0xD: UMP_M1_EMIT(0xD0 | ch, hi7, 0);               break;   // Channel Pressure
    case 0xE: UMP_M1_EMIT(0xE0 | ch, v14 & 0x7f, v14 >> 7); break;   // Pitch Bend
    case 0xC:                                                          // Program Change
      if (ump[3] & 0x01)                                               // Bank Valid
      {
        UMP_M1_EMIT(0xB0 | ch, 0,  ump[6]);                            // Bank Select MSB
        UMP_M1_EMIT(0xB0 | ch, 32, ump[7]);                            // Bank Select LSB
      }
      UMP_M1_EMIT(0xC0 | ch, ump[4], 0);
      break;
    case 0x2:                                                          // Registered Controller (RPN)
    case 0x3:                                                          // Assignable Controller (NRPN)
      UMP_M1_EMIT(0xB0 | ch, op == 0x2 ? 101 : 99, ump[2]);            // bank
      UMP_M1_EMIT(0xB0 | ch, op == 0x2 ? 100 : 98, ump[3]);            // index
      UMP_M1_EMIT(0xB0 | ch, 6,  v14 >> 7);                            // Data Entry MSB
      UMP_M1_EMIT(0xB0 | ch, 38, v14 & 0x7f);                          // Data Entry LSB
      break;
    default:                                                           // no MIDI 1.0 equivalent
      break;
  }
  #undef UMP_M1_EMIT
  return n;
}

//--------------------------------------------------------------------+
// Class Specific Descriptor
//--------------------------------------------------------------------+

typedef enum
{
  MIDI_1_CS_INTERFACE_HEADER    = 0x01,
  MIDI_1_CS_INTERFACE_IN_JACK   = 0x02,
  MIDI_1_CS_INTERFACE_OUT_JACK  = 0x03,
  MIDI_1_CS_INTERFACE_ELEMENT   = 0x04,
  MIDI_1_CS_INTERFACE_GR_TRM_BLOCK = 0x26,
} midi_1_cs_interface_subtype_t;

typedef enum
{
  MIDI_1_CS_ENDPOINT_GENERAL = 0x01,
  MIDI20_CS_ENDPOINT_GENERAL = 0x02,
} midi_1_cs_endpoint_subtype_t;

typedef enum
{
  MIDI_1_JACK_EMBEDDED = 0x01,
  MIDI_1_JACK_EXTERNAL = 0x02
} midi_1_jack_type_t;

typedef enum
{
  MIDI_GR_TRM_BLOCK_HEADER = 0x01,
  MIDI_GR_TRM_BLOCK = 0x02
} midi_group_terminal_block_type_t;

typedef enum
{
  MIDI_1_CIN_MISC              = 0,
  MIDI_1_CIN_CABLE_EVENT       = 1,
  MIDI_1_CIN_SYSCOM_2BYTE      = 2, // 2 byte system common message e.g MTC, SongSelect
  MIDI_1_CIN_SYSCOM_3BYTE      = 3, // 3 byte system common message e.g SPP
  MIDI_1_CIN_SYSEX_START       = 4, // SysEx starts or continue
  MIDI_1_CIN_SYSEX_END_1BYTE   = 5, // SysEx ends with 1 data, or 1 byte system common message
  MIDI_1_CIN_SYSEX_END_2BYTE   = 6, // SysEx ends with 2 data
  MIDI_1_CIN_SYSEX_END_3BYTE   = 7, // SysEx ends with 3 data
  MIDI_1_CIN_NOTE_ON           = 8,
  MIDI_1_CIN_NOTE_OFF          = 9,
  MIDI_1_CIN_POLY_KEYPRESS     = 10,
  MIDI_1_CIN_CONTROL_CHANGE    = 11,
  MIDI_1_CIN_PROGRAM_CHANGE    = 12,
  MIDI_1_CIN_CHANNEL_PRESSURE  = 13,
  MIDI_1_CIN_PITCH_BEND_CHANGE = 14,
  MIDI_1_CIN_1BYTE_DATA = 15
} midi_1_code_index_number_t;

// MIDI 1.0 status byte
enum
{
  //------------- System Exclusive -------------//
  MIDI_1_STATUS_SYSEX_START                    = 0xF0,
  MIDI_1_STATUS_SYSEX_END                      = 0xF7,

  //------------- System Common -------------//
  MIDI_1_STATUS_SYSCOM_TIME_CODE_QUARTER_FRAME = 0xF1,
  MIDI_1_STATUS_SYSCOM_SONG_POSITION_POINTER   = 0xF2,
  MIDI_1_STATUS_SYSCOM_SONG_SELECT             = 0xF3,
  // F4, F5 is undefined
  MIDI_1_STATUS_SYSCOM_TUNE_REQUEST            = 0xF6,

  //------------- System RealTime  -------------//
  MIDI_1_STATUS_SYSREAL_TIMING_CLOCK           = 0xF8,
  // 0xF9 is undefined
  MIDI_1_STATUS_SYSREAL_START                  = 0xFA,
  MIDI_1_STATUS_SYSREAL_CONTINUE               = 0xFB,
  MIDI_1_STATUS_SYSREAL_STOP                   = 0xFC,
  // 0xFD is undefined
  MIDI_1_STATUS_SYSREAL_ACTIVE_SENSING         = 0xFE,
  MIDI_1_STATUS_SYSREAL_SYSTEM_RESET           = 0xFF,
};

/// MIDI Interface Header Descriptor
typedef struct TU_ATTR_PACKED
{
  uint8_t bLength            ; ///< Size of this descriptor in bytes.
  uint8_t bDescriptorType    ; ///< Descriptor Type, must be Class-Specific
  uint8_t bDescriptorSubType ; ///< Descriptor SubType
  uint16_t bcdMSC            ; ///< MidiStreaming SubClass release number in Binary-Coded Decimal
  uint16_t wTotalLength      ;
} midi_1_desc_header_t;

/// MIDI In Jack Descriptor
typedef struct TU_ATTR_PACKED
{
  uint8_t bLength            ; ///< Size of this descriptor in bytes.
  uint8_t bDescriptorType    ; ///< Descriptor Type, must be Class-Specific
  uint8_t bDescriptorSubType ; ///< Descriptor SubType
  uint8_t bJackType          ; ///< Embedded or External
  uint8_t bJackID            ; ///< Unique ID for MIDI IN Jack
  uint8_t iJack              ; ///< string descriptor
} midi_1_desc_in_jack_t;


/// MIDI Out Jack Descriptor with single pin
typedef struct TU_ATTR_PACKED
{
  uint8_t bLength            ; ///< Size of this descriptor in bytes.
  uint8_t bDescriptorType    ; ///< Descriptor Type, must be Class-Specific
  uint8_t bDescriptorSubType ; ///< Descriptor SubType
  uint8_t bJackType          ; ///< Embedded or External
  uint8_t bJackID            ; ///< Unique ID for MIDI IN Jack
  uint8_t bNrInputPins;

  uint8_t baSourceID;
  uint8_t baSourcePin;

  uint8_t iJack              ; ///< string descriptor
} midi_1_desc_out_jack_t ;

/// MIDI Out Jack Descriptor with multiple pins
#define midi_desc_out_jack_n_t(input_num) \
  struct TU_ATTR_PACKED { \
    uint8_t bLength            ; \
    uint8_t bDescriptorType    ; \
    uint8_t bDescriptorSubType ; \
    uint8_t bJackType          ; \
    uint8_t bJackID            ; \
    uint8_t bNrInputPins       ; \
    struct TU_ATTR_PACKED {      \
        uint8_t baSourceID;      \
        uint8_t baSourcePin;     \
    } pins[input_num];           \
   uint8_t iJack              ;  \
  }

/// MIDI Element Descriptor
typedef struct TU_ATTR_PACKED
{
  uint8_t bLength            ; ///< Size of this descriptor in bytes.
  uint8_t bDescriptorType    ; ///< Descriptor Type, must be Class-Specific
  uint8_t bDescriptorSubType ; ///< Descriptor SubType
  uint8_t bElementID;

  uint8_t bNrInputPins;
  uint8_t baSourceID;
  uint8_t baSourcePin;

  uint8_t bNrOutputPins;
  uint8_t bInTerminalLink;
  uint8_t bOutTerminalLink;
  uint8_t bElCapsSize;

  uint16_t bmElementCaps;
  uint8_t  iElement;
} midi_1_desc_element_t;

/// MIDI Element Descriptor with multiple pins
#define midi_desc_element_n_t(input_num) \
  struct TU_ATTR_PACKED {       \
    uint8_t bLength;            \
    uint8_t bDescriptorType;    \
    uint8_t bDescriptorSubType; \
    uint8_t bElementID;         \
    uint8_t bNrInputPins;       \
    struct TU_ATTR_PACKED {     \
        uint8_t baSourceID;     \
        uint8_t baSourcePin;    \
    } pins[input_num];          \
    uint8_t bNrOutputPins;      \
    uint8_t bInTerminalLink;    \
    uint8_t bOutTerminalLink;   \
    uint8_t bElCapsSize;        \
    uint16_t bmElementCaps;     \
    uint8_t  iElement;          \
 }

/// MIDI 2 Streaming Data Endpoint Descriptor with one group terminal block
typedef struct TU_ATTR_PACKED
{
  uint8_t bLength            ; ///< Size of this descriptor in bytes: 4+n
  uint8_t bDescriptorType    ; ///< Descriptor Type: CS_ENDPOINT
  uint8_t bDescriptorSubType ; ///< Descriptor SubType: MIDI20_CS_ENDPOINT_GENERAL
  uint8_t bNumGrpTrmBlock    ; ///< Number of Group Terminal Blocks: 1
  uint8_t bAssoGrpTrmBlkID   ; ///< ID of the Group Terminal Block that is associated with this endpoint
} midi2_desc_streaming_data_endpoint_t;

/// MIDI 2 Streaming Data Endpoint Descriptor with multiple group terminal blocks
#define midi2_desc_streaming_data_endpoint_n_t(group_terminal_block_num) \
  struct TU_ATTR_PACKED {       \
    uint8_t bLength;            \
    uint8_t bDescriptorType;    \
    uint8_t bDescriptorSubType; \
    uint8_t bNumGrpTrmBlock;    \
    uint8_t bNrInputPins;       \
    uint8_t baAssoGrpTrmBlkID[group_terminal_block_num]; \
  }

/// MIDI 2 Group Terminal Block Header Descriptor
typedef struct TU_ATTR_PACKED
{
  uint8_t  bLength            ; ///< Size of this descriptor in bytes: 5
  uint8_t  bDescriptorType    ; ///< Descriptor Type: MIDI_1_CS_INTERFACE_GR_TRM_BLOCK
  uint8_t  bDescriptorSubType ; ///< Descriptor SubType: MIDI_GR_TRM_BLOCK_HEADER
  uint16_t wTotalLength       ; ///< Total number of bytes returned for the class-specific Group Terminal Block descriptors. Includes the combined length of this header descriptor and all Group Terminal Block descriptors.
} midi2_desc_group_terminal_block_header_t;

/// MIDI 2 Group Terminal Block Descriptor
typedef struct TU_ATTR_PACKED
{
  uint8_t  bLength            ; ///< Size of this descriptor in bytes: 13
  uint8_t  bDescriptorType    ; ///< Descriptor Type: MIDI_1_CS_INTERFACE_GR_TRM_BLOCK
  uint8_t  bDescriptorSubType ; ///< Descriptor SubType: MIDI_GR_TRM_BLOCK
  uint8_t  bGrpTrmBlkID       ; ///< ID of this Group Terminal Block
  uint8_t  bGrpTrmBlkType     ; ///< Group Terminal Block Type
  uint8_t  nGroupTrm          ; ///< The first member Group Terminal in this Block
  uint8_t  nNumGroupTrm       ; ///< Number of member Group Terminals spanned
  uint8_t  iBlockItem         ; ///< ID of STRING descriptor for UI representation of Block item
  uint8_t  bMIDIProtocol      ; ///< Default MIDI protocol
  uint16_t wMaxInputBandwidth ; ///< Maximum Input Bandwidth Capability in 4KB/second
  uint16_t wMaxOutputBandwidth; ///< Maximum Output Bandwidth Capability in 4KB/second
} midi2_desc_group_terminal_block_t;

/// MIDI 2 Group Terminal Blocks Descriptor with one group terminal block
typedef struct TU_ATTR_PACKED
{
  midi2_desc_group_terminal_block_header_t header;
  midi2_desc_group_terminal_block_t block;
} midi2_cs_interface_desc_group_terminal_blocks_t;

/// MIDI 2 Group Terminal Blocks Descriptor with one group terminal block
#define midi2_cs_interface_desc_group_terminal_blocks_n_t(group_terminal_block_num) \
  struct TU_ATTR_PACKED {       \
    midi2_desc_group_terminal_block_header_t header;            \
    midi2_desc_group_terminal_block_t aBlock[group_terminal_block_num]; \
  }

/** @} */

#ifdef __cplusplus
 }
#endif

#endif

/** @} */
