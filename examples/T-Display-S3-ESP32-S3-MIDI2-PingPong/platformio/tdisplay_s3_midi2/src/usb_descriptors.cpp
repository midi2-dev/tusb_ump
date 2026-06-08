// usb_descriptors.cpp — USB MIDI 2.0 descriptor callbacks for tusb_ump
//
// Overrides the WEAK symbols from arduino-esp32's esp32-hal-tinyusb.c:
//   tud_descriptor_device_cb()
//   tud_descriptor_configuration_cb()
//   tud_descriptor_string_cb()
//
// USB MIDI 2.0 requires DUAL ALTERNATE SETTINGS on Interface 1:
//   Alt 0x00 — MIDI Streaming 1.0  (legacy hosts)
//   Alt 0x01 — MIDI Streaming 2.0  (UMP, modern hosts)
//
// Host selects alt setting via SET_INTERFACE. tusb_ump handles this
// automatically via umpd_control_xfer_cb() and exposes tud_alt_setting().
//
// Group Terminal Block (GTB) descriptor is served by tusb_ump via a
// class-specific GET_DESCRIPTOR request (0x2601), not via the config
// descriptor — so we don't include it in the config blob below.

#include <Arduino.h>
#include "tusb.h"
#include "ump.h"

// ─────────────────────────────────────────────────────────────
// Interface and endpoint numbering
// ─────────────────────────────────────────────────────────────
#define ITF_NUM_AUDIO_CONTROL   0
#define ITF_NUM_MIDI_STREAMING  1
#define ITF_NUM_TOTAL           2

#define EPNUM_MIDI_OUT  0x01
#define EPNUM_MIDI_IN   0x81

#define EP_SIZE_FS   64

// ─────────────────────────────────────────────────────────────
// String descriptor indices
// ─────────────────────────────────────────────────────────────
enum {
  STR_IDX_LANGID          = 0,
  STR_IDX_MANUFACTURER    = 1,
  STR_IDX_PRODUCT         = 2,
  STR_IDX_SERIAL          = 3,
  STR_IDX_MIDI_STREAMING  = 4,
  STR_IDX_BLOCK_1         = 5,
};

// ─────────────────────────────────────────────────────────────
// Device descriptor
// ─────────────────────────────────────────────────────────────
static const tusb_desc_device_t desc_device = {
  .bLength            = sizeof(tusb_desc_device_t),
  .bDescriptorType    = TUSB_DESC_DEVICE,
  .bcdUSB             = 0x0200,
  .bDeviceClass       = TUSB_CLASS_MISC,  // 0xEF — required for IAD on Windows
  .bDeviceSubClass    = 0x02,
  .bDeviceProtocol    = 0x01,
  .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
  .idVendor           = 0x1209,   // pid.codes (open-source VID)
  .idProduct          = 0x0001,   // change for production
  .bcdDevice          = 0x0100,
  .iManufacturer      = STR_IDX_MANUFACTURER,
  .iProduct           = STR_IDX_PRODUCT,
  .iSerialNumber      = STR_IDX_SERIAL,
  .bNumConfigurations = 1,
};

// ─────────────────────────────────────────────────────────────
// Configuration descriptor (153 bytes)
//
//  Config(9) + IAD(8) + ITF0_AC(9) + CS_AC(9)
//  + ITF1_Alt0(9) + CS_MS(7) + 2xJackIN(12) + 2xJackOUT(18) + 2xEP(18) + 2xCS_EP(10)
//  + ITF1_Alt1(9) + CS_MS(7) + 2xEP(18) + 2xCS_EP(10)
//  = 153
// ─────────────────────────────────────────────────────────────

#define CONFIG_TOTAL_LEN  153
#define AC_CS_HEADER_LEN  9

static const uint8_t desc_configuration[CONFIG_TOTAL_LEN] = {

  // ── Configuration ─────────────────────────────────────────
  9, TUSB_DESC_CONFIGURATION,
  U16_TO_U8S_LE(CONFIG_TOTAL_LEN),
  ITF_NUM_TOTAL, 1, 0,
  TU_BIT(7) | TUSB_DESC_CONFIG_ATT_SELF_POWERED, 50,

  // ── IAD ───────────────────────────────────────────────────
  8, TUSB_DESC_INTERFACE_ASSOCIATION,
  ITF_NUM_AUDIO_CONTROL, 2,
  TUSB_CLASS_AUDIO, 0x03, 0x00, 0,

  // ── Interface 0: AudioControl ─────────────────────────────
  9, TUSB_DESC_INTERFACE,
  ITF_NUM_AUDIO_CONTROL, 0, 0,
  TUSB_CLASS_AUDIO, 0x01, 0x00, 0,  // protocol must be 0x00 (umpd_open checks AUDIO_FUNC_PROTOCOL_CODE_UNDEF)

  // ── CS AC Header (UAC2) ───────────────────────────────────
  AC_CS_HEADER_LEN, TUSB_DESC_CS_INTERFACE, 0x01,
  U16_TO_U8S_LE(0x0200), 0x03,
  U16_TO_U8S_LE(AC_CS_HEADER_LEN), 0x00,

  // ─── Interface 1, Alt 0: MIDI Streaming 1.0 ───────────────
  9, TUSB_DESC_INTERFACE,
  ITF_NUM_MIDI_STREAMING, 0, 2,
  TUSB_CLASS_AUDIO, 0x03, 0x00, STR_IDX_MIDI_STREAMING,

  // CS MS Header (bcdMSC=1.00, wTotalLength=37)
  7, TUSB_DESC_CS_INTERFACE, MIDI_1_CS_INTERFACE_HEADER,
  U16_TO_U8S_LE(0x0100), U16_TO_U8S_LE(37),

  // Embedded IN Jack (ID=1)
  6, TUSB_DESC_CS_INTERFACE, MIDI_1_CS_INTERFACE_IN_JACK,
  MIDI_1_JACK_EMBEDDED, 1, 0,

  // External IN Jack (ID=3)
  6, TUSB_DESC_CS_INTERFACE, MIDI_1_CS_INTERFACE_IN_JACK,
  MIDI_1_JACK_EXTERNAL, 3, 0,

  // Embedded OUT Jack (ID=2, src=ExtIN 3)
  9, TUSB_DESC_CS_INTERFACE, MIDI_1_CS_INTERFACE_OUT_JACK,
  MIDI_1_JACK_EMBEDDED, 2, 1, 3, 1, 0,

  // External OUT Jack (ID=4, src=EmbIN 1)
  9, TUSB_DESC_CS_INTERFACE, MIDI_1_CS_INTERFACE_OUT_JACK,
  MIDI_1_JACK_EXTERNAL, 4, 1, 1, 1, 0,

  // Bulk OUT endpoint Alt0
  9, TUSB_DESC_ENDPOINT, EPNUM_MIDI_OUT, TUSB_XFER_BULK,
  U16_TO_U8S_LE(EP_SIZE_FS), 0, 0, 0,

  // CS Endpoint (assoc Embedded OUT jack 2)
  5, TUSB_DESC_CS_ENDPOINT, MIDI_1_CS_ENDPOINT_GENERAL, 1, 2,

  // Bulk IN endpoint Alt0
  9, TUSB_DESC_ENDPOINT, EPNUM_MIDI_IN, TUSB_XFER_BULK,
  U16_TO_U8S_LE(EP_SIZE_FS), 0, 0, 0,

  // CS Endpoint (assoc Embedded IN jack 1)
  5, TUSB_DESC_CS_ENDPOINT, MIDI_1_CS_ENDPOINT_GENERAL, 1, 1,

  // ─── Interface 1, Alt 1: MIDI Streaming 2.0 (UMP) ─────────
  9, TUSB_DESC_INTERFACE,
  ITF_NUM_MIDI_STREAMING, 1, 2,
  TUSB_CLASS_AUDIO, 0x03, 0x00, STR_IDX_MIDI_STREAMING,

  // CS MS Header (bcdMSC=2.00, wTotalLength=7)
  7, TUSB_DESC_CS_INTERFACE, MIDI_1_CS_INTERFACE_HEADER,
  U16_TO_U8S_LE(0x0200), U16_TO_U8S_LE(7),

  // Bulk OUT endpoint Alt1
  9, TUSB_DESC_ENDPOINT, EPNUM_MIDI_OUT, TUSB_XFER_BULK,
  U16_TO_U8S_LE(EP_SIZE_FS), 0, 0, 0,

  // CS Endpoint MIDI 2.0 (assoc GTB ID=1)
  5, TUSB_DESC_CS_ENDPOINT, MIDI20_CS_ENDPOINT_GENERAL, 1, 1,

  // Bulk IN endpoint Alt1
  9, TUSB_DESC_ENDPOINT, EPNUM_MIDI_IN, TUSB_XFER_BULK,
  U16_TO_U8S_LE(EP_SIZE_FS), 0, 0, 0,

  // CS Endpoint MIDI 2.0 (assoc GTB ID=1)
  5, TUSB_DESC_CS_ENDPOINT, MIDI20_CS_ENDPOINT_GENERAL, 1, 1,
};

static_assert(sizeof(desc_configuration) == CONFIG_TOTAL_LEN,
              "CONFIG_TOTAL_LEN mismatch");

// ─────────────────────────────────────────────────────────────
// String descriptors
// ─────────────────────────────────────────────────────────────
static const char *const string_desc_arr[] = {
  (const char[]){ 0x09, 0x04 },
  "sauloverissimo",
  "ESP32-S3 MIDI 2.0 Device",
  "ESP32S3-MIDI2-001",
  "MIDI Streaming",
  "Group 0",
};

static uint16_t _string_desc_buf[32];

// ─────────────────────────────────────────────────────────────
// Callbacks (override WEAK symbols from arduino-esp32)
//
// __attribute__((used)) prevents --gc-sections from eliminating
// these functions. arduino-esp32 links with -Wl,--gc-sections
// and -ffunction-sections, so any function not reachable from a
// link root (setup/loop/app_main) would otherwise be discarded.
// The weak definitions in esp32-hal-tinyusb.c would then be used
// instead, causing VID=0x303A and a MIDI 1.0-only descriptor.
// ─────────────────────────────────────────────────────────────

extern "C" {

__attribute__((used))
uint8_t const *tud_descriptor_device_cb(void) {
  return (uint8_t const *)&desc_device;
}

__attribute__((used))
uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
  (void)index;
  return desc_configuration;
}

__attribute__((used))
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void)langid;
  uint8_t chr_count;
  const uint8_t max_str = sizeof(string_desc_arr) / sizeof(string_desc_arr[0]);

  if (index == 0) {
    memcpy(&_string_desc_buf[1], string_desc_arr[0], 2);
    chr_count = 1;
  } else if (index < max_str) {
    const char *str = string_desc_arr[index];
    chr_count = (uint8_t)strlen(str);
    if (chr_count > 31) chr_count = 31;
    for (uint8_t i = 0; i < chr_count; i++) {
      _string_desc_buf[1 + i] = str[i];
    }
  } else {
    return NULL;
  }

  _string_desc_buf[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
  return _string_desc_buf;
}

} // extern "C"
