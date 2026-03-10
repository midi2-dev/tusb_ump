// usb_descriptors.cpp — USB MIDI 2.0 descriptors for tusb_ump (RP2040)
//
// On RP2040 + arduino-pico, tud_descriptor_*_cb() are already defined
// as strong symbols in Adafruit_USBD_Device.cpp (from Adafruit TinyUSB
// Arduino). We therefore do NOT define those callbacks here.
//
// Instead, usb_descriptors_begin() injects our raw 153-byte MIDI 2.0
// configuration descriptor directly into TinyUSBDevice after it has been
// initialised by the framework (before setup()), and configures the device
// identity strings via the Adafruit API.
//
// Call from setup() BEFORE Serial or any other TinyUSB initialisation.
//
// USB MIDI 2.0 descriptor structure — same as T-Display-S3:
//   VID=0x1209, PID=0x0001
//   Alt 0 — MIDI Streaming 1.0  (legacy hosts)
//   Alt 1 — MIDI Streaming 2.0  (UMP, modern hosts)
// Same VID/PID and GTB ensure ping-pong compatibility with T-Display-S3.

#include <Arduino.h>
#include "tusb.h"
#include "ump.h"
#include <Adafruit_TinyUSB.h>

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
// Adafruit TinyUSBDevice always reserves 0-3:
//   0 = Language ID, 1 = Manufacturer, 2 = Product, 3 = Serial
// addStringDescriptor() appends from index 4 onwards.
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
// Configuration descriptor (153 bytes)
//
//  Config(9) + IAD(8) + ITF0_AC(9) + CS_AC(9)
//  + ITF1_Alt0(9) + CS_MS(7) + 2xJackIN(12) + 2xJackOUT(18)
//  + 2xEP(18) + 2xCS_EP(10)
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
  TUSB_CLASS_AUDIO, 0x01, 0x00, 0,

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
// RP2040 descriptor setup
//
// TinyUSBDevice.begin() is called by the framework before setup().
// We call this function at the top of setup() to:
//   1. Set VID/PID and identity strings via the Adafruit API
//   2. Inject our raw 153-byte MIDI 2.0 config descriptor
//
// The persistent _cfg_buf holds the config descriptor for the lifetime
// of the firmware — it must NOT be a local variable.
//
// Note: Adafruit begin() sets bDeviceClass=0xEF/SubClass=2/Protocol=1
// (IAD) automatically, which is exactly what USB MIDI 2.0 requires.
// ─────────────────────────────────────────────────────────────
static uint8_t _cfg_buf[256];

void usb_descriptors_begin(void) {
  TinyUSBDevice.setID(0x1209, 0x0001);
  TinyUSBDevice.setManufacturerDescriptor("sauloverissimo");
  TinyUSBDevice.setProductDescriptor("RP2040 MIDI 2.0 Device");
  TinyUSBDevice.setSerialDescriptor("RP2040-MIDI2-001");
  TinyUSBDevice.addStringDescriptor("MIDI Streaming");  // index 4
  TinyUSBDevice.addStringDescriptor("Group 0");          // index 5

  // Replace Adafruit's default config descriptor with our raw MIDI 2.0 one.
  // setConfigurationBuffer() points _desc_cfg at _cfg_buf.
  // The subsequent memcpy overwrites it with our full 153-byte descriptor.
  TinyUSBDevice.setConfigurationBuffer(_cfg_buf, sizeof(_cfg_buf));
  memcpy(_cfg_buf, desc_configuration, CONFIG_TOTAL_LEN);
}
