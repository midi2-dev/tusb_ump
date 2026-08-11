/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2022 Michael Loh (AmeNote.com)
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
 */

// USB descriptors for the tusb_ump_lb loopback example.
//
// Single USB MIDI 2.0 device, one MIDIStreaming interface with two
// alternate settings (Alt 0: legacy USB-MIDI 1.0 byte stream, Alt 1: UMP /
// USB MIDI 2.0 - see USB Device Class Definition for MIDI Devices,
// Version 2.0), and one Group Terminal Block spanning all 16 groups so the
// host sees a single bidirectional "Loopback" function block.

#include "tusb.h"
#include "pico/unique_id.h"
#include "ump_device.h"

#define USB_VID   0xCafe  // NOTE: TinyUSB's default/example VID - not valid for commercial use
#define USB_PID   0x4004
#define USB_BCD   0x0200

//--------------------------------------------------------------------+
// Device Descriptor
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device =
{
  .bLength            = sizeof(tusb_desc_device_t),
  .bDescriptorType    = TUSB_DESC_DEVICE,
  .bcdUSB             = USB_BCD,

  // Single function (Audio/MIDIStreaming) device - class is defined at the
  // interface level, no Interface Association Descriptor needed.
  .bDeviceClass       = 0x00,
  .bDeviceSubClass    = 0x00,
  .bDeviceProtocol    = 0x00,

  .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

  .idVendor           = USB_VID,
  .idProduct          = USB_PID,
  .bcdDevice          = 0x0100,

  .iManufacturer      = 0x01,
  .iProduct           = 0x02,
  .iSerialNumber      = 0x03,

  .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

enum
{
  ITF_NUM_AUDIO_CONTROL = 0,
  ITF_NUM_MIDI_STREAMING,
  ITF_NUM_TOTAL
};

// Jack IDs used on the Alt 0 (legacy MIDI 1.0) MIDIStreaming setting
enum
{
  JACKID_EMB_IN  = 1, // Embedded IN Jack  - receives bytes arriving from the host (Bulk OUT ep)
  JACKID_EMB_OUT = 2, // Embedded OUT Jack - sends bytes back to the host (Bulk IN ep), sourced from JACKID_EMB_IN
};

// Group Terminal Block ID used on the Alt 1 (UMP / MIDI 2.0) MIDIStreaming setting
#define GRPTRMBLKID_LOOPBACK  1

#define EPNUM_MIDI  0x01

#define TOTAL_DESC_LEN  (9 /*Config*/ + 9 /*AC ITF*/ + 9 /*AC CS Header*/ \
                          + 9 + 7 + 6 + 9 + 7 + 5 + 7 + 5 /*Alt 0*/ \
                          + 9 + 7 + 7 + 5 + 7 + 5 /*Alt 1*/ )

uint8_t const desc_fs_configuration[] =
{
    // ----- Configuration Descriptor
    0x09,                    // bLength
    TUSB_DESC_CONFIGURATION, // bDescriptorType
    U16_TO_U8S_LE(TOTAL_DESC_LEN),
    ITF_NUM_TOTAL,           // bNumInterfaces
    0x01,                    // bConfigurationValue
    0x00,                    // iConfiguration
    0x80,                    // bmAttributes (bus powered)
    0x32,                    // bMaxPower (100mA)

    // ----- Standard Interface - Audio Control (no endpoints)
    0x09, TUSB_DESC_INTERFACE, ITF_NUM_AUDIO_CONTROL, 0x00, 0x00,
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_CONTROL, AUDIO_FUNC_PROTOCOL_CODE_UNDEF, 0x00,

    // ----- Class-Specific AC Interface Header
    0x09, TUSB_DESC_CS_INTERFACE, 0x01 /*HEADER*/, U16_TO_U8S_LE(0x0100) /*bcdADC*/,
    U16_TO_U8S_LE(0x09) /*wTotalLength*/, 0x01 /*bInCollection*/, ITF_NUM_MIDI_STREAMING,

    // ===== Alt 0: legacy USB-MIDI 1.0 byte stream =====

    // ----- Standard Interface - MIDIStreaming, Alt 0
    0x09, TUSB_DESC_INTERFACE, ITF_NUM_MIDI_STREAMING, 0x00 /*bAlternateSetting*/, 0x02 /*bNumEndpoints*/,
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_MIDI_STREAMING, AUDIO_INT_PROTOCOL_CODE_UNDEF, 0x04 /*iInterface*/,

    // ----- Class-Specific MS Interface Header (MIDI 1.0)
    0x07, TUSB_DESC_CS_INTERFACE, MIDI_1_CS_INTERFACE_HEADER, U16_TO_U8S_LE(0x0100) /*bcdMSC*/,
    U16_TO_U8S_LE(7 + 6 + 9) /*wTotalLength: header+INjack+OUTjack*/,

    // ----- Embedded MIDI IN Jack (receives from host)
    0x06, TUSB_DESC_CS_INTERFACE, MIDI_1_CS_INTERFACE_IN_JACK, MIDI_1_JACK_EMBEDDED, JACKID_EMB_IN, 0x05 /*iJack*/,

    // ----- Embedded MIDI OUT Jack (sends to host), sourced from the embedded IN jack -
    //       this is the loopback wired directly into the descriptor topology.
    0x09, TUSB_DESC_CS_INTERFACE, MIDI_1_CS_INTERFACE_OUT_JACK, MIDI_1_JACK_EMBEDDED, JACKID_EMB_OUT,
    0x01 /*bNrInputPins*/, JACKID_EMB_IN /*baSourceID*/, 0x01 /*baSourcePin*/, 0x06 /*iJack*/,

    // ----- Standard Bulk OUT Endpoint
    0x07, TUSB_DESC_ENDPOINT, EPNUM_MIDI, TUSB_XFER_BULK, U16_TO_U8S_LE(64), 0x00,

    // ----- Class-Specific MS Bulk OUT Endpoint -> Embedded IN Jack
    0x05, TUSB_DESC_CS_ENDPOINT, MIDI_1_CS_ENDPOINT_GENERAL, 0x01 /*bNumEmbMIDIJack*/, JACKID_EMB_IN,

    // ----- Standard Bulk IN Endpoint
    0x07, TUSB_DESC_ENDPOINT, (0x80 | EPNUM_MIDI), TUSB_XFER_BULK, U16_TO_U8S_LE(64), 0x00,

    // ----- Class-Specific MS Bulk IN Endpoint -> Embedded OUT Jack
    0x05, TUSB_DESC_CS_ENDPOINT, MIDI_1_CS_ENDPOINT_GENERAL, 0x01 /*bNumEmbMIDIJack*/, JACKID_EMB_OUT,

    // ===== Alt 1: UMP / USB MIDI 2.0 =====

    // ----- Standard Interface - MIDIStreaming, Alt 1
    0x09, TUSB_DESC_INTERFACE, ITF_NUM_MIDI_STREAMING, 0x01 /*bAlternateSetting*/, 0x02 /*bNumEndpoints*/,
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_MIDI_STREAMING, AUDIO_INT_PROTOCOL_CODE_UNDEF, 0x07 /*iInterface*/,

    // ----- Class-Specific MS Interface Header (MIDI 2.0 / UMP)
    0x07, TUSB_DESC_CS_INTERFACE, MIDI_1_CS_INTERFACE_HEADER, U16_TO_U8S_LE(0x0200) /*bcdMSC*/,
    U16_TO_U8S_LE(0x07) /*wTotalLength: header only, no jacks in UMP mode*/,

    // ----- Standard Bulk OUT Endpoint
    0x07, TUSB_DESC_ENDPOINT, EPNUM_MIDI, TUSB_XFER_BULK, U16_TO_U8S_LE(64), 0x00,

    // ----- Class-Specific MS Bulk OUT Endpoint -> Group Terminal Block
    0x05, TUSB_DESC_CS_ENDPOINT, MIDI20_CS_ENDPOINT_GENERAL, 0x01 /*bNumGrpTrmBlock*/, GRPTRMBLKID_LOOPBACK,

    // ----- Standard Bulk IN Endpoint
    0x07, TUSB_DESC_ENDPOINT, (0x80 | EPNUM_MIDI), TUSB_XFER_BULK, U16_TO_U8S_LE(64), 0x00,

    // ----- Class-Specific MS Bulk IN Endpoint -> Group Terminal Block
    0x05, TUSB_DESC_CS_ENDPOINT, MIDI20_CS_ENDPOINT_GENERAL, 0x01 /*bNumGrpTrmBlock*/, GRPTRMBLKID_LOOPBACK,
};

TU_VERIFY_STATIC(sizeof(desc_fs_configuration) == TOTAL_DESC_LEN, "size mismatch");

// device qualifier is mostly similar to device descriptor since we don't change configuration based on speed
tusb_desc_device_qualifier_t const desc_device_qualifier =
{
  .bLength            = sizeof(tusb_desc_device_qualifier_t),
  .bDescriptorType    = TUSB_DESC_DEVICE_QUALIFIER,
  .bcdUSB             = USB_BCD,

  .bDeviceClass       = 0x00,
  .bDeviceSubClass    = 0x00,
  .bDeviceProtocol    = 0x00,

  .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
  .bNumConfigurations = 0x01,
  .bReserved          = 0x00
};

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index; // only one configuration
  return desc_fs_configuration;
}

// Invoked when received GET DEVICE QUALIFIER DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete.
uint8_t const* tud_descriptor_device_qualifier_cb(void)
{
  return (uint8_t const*) &desc_device_qualifier;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;

  int len = 2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1;
  char serialId[len];
  pico_get_unique_board_id_string(serialId, len);

  // array of pointer to string descriptors
  char const* string_desc_arr [] =
  {
    (const char[]) { 0x09, 0x04 },     // 0: supported language is English (0x0409)
    "AmeNote",                         // 1: Manufacturer
    "tusb_ump Loopback",               // 2: Product
    serialId,                          // 3: Serial, from board unique ID
    "MIDI 1.0 Loopback",               // 4: MIDIStreaming Alt 0 interface
    "Loopback In",                     // 5: Embedded IN Jack (Alt 0)
    "Loopback Out",                    // 6: Embedded OUT Jack (Alt 0)
    "MIDI 2.0 Loopback (UMP)",         // 7: MIDIStreaming Alt 1 interface
    "Loopback",                        // 8: Group Terminal Block label
  };

  uint8_t chr_count;

  if ( index == 0)
  {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  }
  else
  {
    if ( !(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) ) return NULL;

    const char* str = string_desc_arr[index];

    // Cap at max char
    chr_count = strlen(str);
    if ( chr_count > 31 ) chr_count = 31;

    // Convert ASCII string into UTF-16
    for(uint8_t i=0; i<chr_count; i++)
    {
      _desc_str[1+i] = str[i];
    }
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (TUSB_DESC_STRING << 8 ) | (2*chr_count + 2);

  return _desc_str;
}

//--------------------------------------------------------------------+
// Group Terminal Block Descriptor
//--------------------------------------------------------------------+
// One bidirectional Group Terminal Block spanning all 16 groups - the whole
// device is a single "Loopback" function block, matching the single UMP
// endpoint driven by ump_device.c/h.

static midi2_cs_interface_desc_group_terminal_blocks_n_t(1) group_terminal_blocks_desc =
{
  .header = {
    .bLength             = 5,
    .bDescriptorType     = MIDI_1_CS_INTERFACE_GR_TRM_BLOCK,
    .bDescriptorSubType  = MIDI_GR_TRM_BLOCK_HEADER,
    .wTotalLength        = sizeof(group_terminal_blocks_desc)
  },
  .aBlock = {
    {
      .bLength             = 13,
      .bDescriptorType     = MIDI_1_CS_INTERFACE_GR_TRM_BLOCK,
      .bDescriptorSubType  = MIDI_GR_TRM_BLOCK,
      .bGrpTrmBlkID        = GRPTRMBLKID_LOOPBACK,
      .bGrpTrmBlkType      = 0x00,   // bi-directional
      .nGroupTrm           = 0x00,   // starting Group Terminal (Group 1 / index 0)
      .nNumGroupTrm        = 16,     // spans all 16 groups
      .iBlockItem          = 0x08,   // "Loopback"
      .bMIDIProtocol       = 0x03,   // MIDI 1.0, UMP up to 128 bits
      .wMaxInputBandwidth  = 0x0000, // Unknown or Not Fixed
      .wMaxOutputBandwidth = 0x0000  // Unknown or Not Fixed
    }
  }
};

// Invoked on class-specific GET request for the Group Terminal Block descriptor
// (wValue 0x2601: 0x26 = CS_GR_TRM_BLOCK, 0x01 = MIDIStreaming alt setting 1 / UMP)
bool tud_ump_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const * request)
{
  if ( request->wValue == 0x2601 )
  {
    uint16_t length = request->wLength;
    if ( length > sizeof(group_terminal_blocks_desc) )
    {
      length = sizeof(group_terminal_blocks_desc);
    }
    tud_control_xfer(rhport, request, (void *) &group_terminal_blocks_desc, length);
    return true;
  }

  return false;
}
