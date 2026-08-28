/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 * Copyright (c) 2026 Michael Loh (AmeNote.com)
 *
 * NOTE: TinyUSB HOST class driver for USB MIDI 2.0 UMP. See ump_host.h for
 * scope notes (transport + Group Terminal Block descriptors only, no UMP
 * Stream-message handshake in this driver).
 *
 * UMP Host Driver Version 0.1 - 2026-08-26
 *  - Milestone 1+2: enumeration (AC-only and AC+MS-merged call shapes),
 *    alt-setting selection (prefers native UMP alt-setting-1, falls back to
 *    legacy alt-setting-0), Group Terminal Block descriptor fetch (alt-1) or
 *    synthesis from MIDI 1.0 jack descriptors (alt-0-only devices), all with
 *    verbose logging for bring-up against real hardware.
 *  - Raw diagnostic RX pump added: arms the initial IN read and re-arms on
 *    every completion, surfacing raw bytes via the new (diagnostic-only)
 *    tuh_ump_raw_rx_cb().
 *  - Milestone 3: real FIFO-backed tuh_ump_read()/read_ntoh()/write()/
 *    write_hton()/available()/writeable() for alt-setting-1 (native UMP),
 *    mirroring ump_device.cpp's raw-wire-bytes-in-the-fifo /
 *    swap-at-read-or-write-boundary approach (umph_prep_in_read() /
 *    umph_write_flush(), analogous to _prep_out_transaction()/write_flush()).
 *    Validated against real USB MIDI 2.0 UMP traffic on ProtoZOA hardware.
 *  - Milestone 4: alt-setting-0 (legacy USB MIDI 1.0) <-> UMP translation,
 *    ported from ump_device.cpp's tud_USBMIDI1ToUMP() (RX) and
 *    tud_ump_write_impl()'s alt-0 branch (TX, including the per-cable
 *    SysEx7 ring-buffer reassembly). rx_ff/tx_ff now always hold raw wire
 *    bytes regardless of alt setting; translation happens lazily at
 *    read()/write() call time for both alt settings, matching the device
 *    driver's design.
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

#if (CFG_TUH_ENABLED && CFG_TUH_UMP)

//--------------------------------------------------------------------+
// INCLUDE
//--------------------------------------------------------------------+
#include "host/usbh.h"
#include "host/usbh_pvt.h"

#include "tusb.h"
#include "ump_host.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+

// Scratch byte buffer for the two-phase Group Terminal Block descriptor
// fetch. Sized generously; the parse loop below still bounds itself by
// CFG_TUH_UMP_MAX_GTB regardless of what a device claims in wTotalLength.
#define UMPH_GTB_FETCH_BUFSIZE  256

// Alt-setting-0 (legacy MIDI1) <-> UMP translation types, ported verbatim
// from ump_device.cpp -- pure byte-level conversion, no device/host role
// dependence. See umph_USBMIDI1ToUMP() below for attribution.
typedef struct
{
  uint8_t   wordCount;
  union ump_host
  {
    uint32_t  umpWords[4];
    uint8_t   umpBytes[sizeof(uint32_t) * 4];
  } umpData;
} UMPH_PACKET, *PUMPH_PACKET;

// Structure to aid in UMP SYSEX to USB MIDI 1.0 (per-cable ring buffer of
// pending SysEx7 bytes awaiting repacking into 3-byte USB MIDI1 CIN packets)
#define UMPH_SYSEX_BS_RB_SIZE 16
typedef struct
{
  bool    inSysex;
  uint8_t sysexBS[UMPH_SYSEX_BS_RB_SIZE];
  uint8_t usbMIDI1Tail;
  uint8_t usbMIDI1Head;
} UMPH_TO_MIDI1_SYSEX;

#define UMPH_MAX_NUM_GROUPS_CABLES 16

typedef struct
{
  uint8_t  daddr;
  uint8_t  itf_num;          // MIDIStreaming interface number (0xFF = unassigned slot)
  uint8_t  itf_num_ac;       // AudioControl interface number, if seen separately (0xFF = none)

  // Full endpoint descriptors captured during open(), per alt setting -- kept
  // verbatim (not just address/mps) so set_config() can call tuh_edpt_open()
  // with the real descriptor once the active alt setting is chosen.
  tusb_desc_endpoint_t desc_ep_in0,  desc_ep_out0; // alt-setting-0 (legacy MIDI1), bLength=0 if none
  tusb_desc_endpoint_t desc_ep_in1,  desc_ep_out1; // alt-setting-1 (native UMP), bLength=0 if none

  uint8_t  ep_in,  ep_out;   // endpoints of the *active* alt setting (post set_config)
  uint8_t  alt_setting;      // active alt setting: 0 or 1
  bool     alt1_available;

  uint8_t  num_in_jacks, num_out_jacks; // alt-0 jack descriptor tally, for GTB synthesis
  uint16_t bcdMSC0, bcdMSC1;             // MIDIStreaming class spec version per alt setting, 0 if not seen

  // Alt-setting-0 (legacy MIDI1) <-> UMP translation state, per virtual cable/group
  bool                 midi1_rx_is_in_sysex[UMPH_MAX_NUM_GROUPS_CABLES];
  UMPH_TO_MIDI1_SYSEX  midi1_tx_sysex[UMPH_MAX_NUM_GROUPS_CABLES];

  uint8_t  num_gtb;
  midi2_desc_group_terminal_block_t gtb[CFG_TUH_UMP_MAX_GTB];

  uint8_t  gtb_fetch_buf[UMPH_GTB_FETCH_BUFSIZE];

  // Raw IN-endpoint transfer buffer, and (alt-1 native UMP only, for now --
  // see umph_xfer_cb) the app-facing FIFO pump. Alt-0 MIDI1<->UMP
  // translation into these same FIFOs is still milestone 4.
  uint8_t  ep_in_buf[CFG_TUH_UMP_EP_BUFSIZE];
  uint8_t  ep_out_buf[CFG_TUH_UMP_EP_BUFSIZE];
  tu_fifo_t rx_ff, tx_ff;
  uint8_t  rx_ff_buf[CFG_TUH_UMP_RX_BUFSIZE];
  uint8_t  tx_ff_buf[CFG_TUH_UMP_TX_BUFSIZE];

  bool     mounted;
} umph_interface_t;

static umph_interface_t _umph_itf[CFG_TUH_UMP];

//--------------------------------------------------------------------+
// INSTANCE POOL HELPERS
//--------------------------------------------------------------------+

// A composite device can expose more than one AudioControl interface (e.g.
// a real audio-streaming function alongside an unrelated MIDI function, as
// on the iRig Keys 2 PRO -- 5 interfaces: AC(audio)+AS+HID+AC(midi)+MS).
// Always allocate a *fresh* slot for a newly-seen AC interface rather than
// reusing whatever slot already exists for this daddr -- otherwise a second,
// unrelated AC interface silently clobbers the first one's itf_num_ac,
// orphaning it (find_itf() can no longer find it), which stalls TinyUSB's
// enumeration state machine forever waiting on a set_config completion that
// never comes.
static umph_interface_t* alloc_new_itf(uint8_t daddr)
{
  for (uint8_t i = 0; i < CFG_TUH_UMP; i++)
  {
    if (_umph_itf[i].daddr == 0)
    {
      tu_memclr(&_umph_itf[i], sizeof(umph_interface_t));
      _umph_itf[i].daddr     = daddr;
      _umph_itf[i].itf_num    = 0xFF;
      _umph_itf[i].itf_num_ac = 0xFF;
      return &_umph_itf[i];
    }
  }
  return NULL;
}

// Unmerged-shape enumeration (separate open() calls for the AC and MS
// interfaces) needs to pair a just-seen MIDIStreaming interface with the
// *correct* previously-opened, still-unpaired AudioControl interface when a
// device has more than one candidate AC slot pending. USB interface
// numbering convention places an MS interface immediately after its
// associated AC interface, so match on itf_num_ac + 1 == ms_itf_num rather
// than just "any pending AC slot for this daddr".
static umph_interface_t* find_pending_ac_itf(uint8_t daddr, uint8_t ms_itf_num)
{
  for (uint8_t i = 0; i < CFG_TUH_UMP; i++)
  {
    umph_interface_t* p = &_umph_itf[i];
    if (p->daddr == daddr && p->itf_num == 0xFF && p->itf_num_ac != 0xFF &&
        (uint8_t) (p->itf_num_ac + 1) == ms_itf_num)
      return p;
  }
  return NULL;
}

// A device may have multiple slots (one per AC/MS interface grouping) --
// search all of them rather than assuming the first daddr match is the
// right one.
static umph_interface_t* find_itf(uint8_t daddr, uint8_t itf_num)
{
  for (uint8_t i = 0; i < CFG_TUH_UMP; i++)
  {
    umph_interface_t* p = &_umph_itf[i];
    if (p->daddr == daddr && (p->itf_num == itf_num || p->itf_num_ac == itf_num)) return p;
  }
  return NULL;
}

// Same rationale as find_itf() -- a device's dangling (unpaired) AC-only
// slot has ep_in == ep_out == 0, so a first-daddr-match lookup could return
// the wrong slot and silently drop the mounted interface's traffic.
static umph_interface_t* find_itf_by_ep(uint8_t daddr, uint8_t ep_addr)
{
  for (uint8_t i = 0; i < CFG_TUH_UMP; i++)
  {
    umph_interface_t* p = &_umph_itf[i];
    if (p->daddr == daddr && (p->ep_in == ep_addr || p->ep_out == ep_addr)) return p;
  }
  return NULL;
}

//--------------------------------------------------------------------+
// GROUP TERMINAL BLOCK: FETCH (alt-1) / SYNTHESIZE (alt-0 only)
//--------------------------------------------------------------------+

static void umph_parse_gtb_buffer(umph_interface_t* p_ump, uint8_t const* buf, uint16_t len)
{
  p_ump->num_gtb = 0;

  if (len < sizeof(midi2_desc_group_terminal_block_header_t)) return;
  midi2_desc_group_terminal_block_header_t const* hdr = (midi2_desc_group_terminal_block_header_t const*) buf;

  uint16_t total = tu_le16toh(hdr->wTotalLength);
  if (total > len) total = len; // defensive: don't trust device-claimed length past what we actually fetched

  uint16_t offset = hdr->bLength;
  while ( offset + sizeof(midi2_desc_group_terminal_block_t) <= total &&
          p_ump->num_gtb < CFG_TUH_UMP_MAX_GTB )
  {
    midi2_desc_group_terminal_block_t const* blk = (midi2_desc_group_terminal_block_t const*) (buf + offset);

    if ( blk->bLength < sizeof(midi2_desc_group_terminal_block_t) ||
         blk->bDescriptorType != MIDI_1_CS_INTERFACE_GR_TRM_BLOCK ||
         blk->bDescriptorSubType != MIDI_GR_TRM_BLOCK )
    {
      // Malformed / unrecognized entry -- stop rather than mis-parse further bytes
      break;
    }

    p_ump->gtb[p_ump->num_gtb++] = *blk;
    offset += blk->bLength;
  }

  TU_LOG_USBH("UMPH: parsed %u Group Terminal Block(s) from device descriptor\r\n", p_ump->num_gtb);
}

static void umph_synthesize_gtb_from_jacks(umph_interface_t* p_ump)
{
  p_ump->num_gtb = 0;
  if ( (p_ump->num_in_jacks == 0 && p_ump->num_out_jacks == 0) || CFG_TUH_UMP_MAX_GTB == 0 ) return;

  midi2_desc_group_terminal_block_t* blk = &p_ump->gtb[0];
  tu_memclr(blk, sizeof(*blk));
  blk->bLength            = sizeof(midi2_desc_group_terminal_block_t);
  blk->bDescriptorType    = MIDI_1_CS_INTERFACE_GR_TRM_BLOCK;
  blk->bDescriptorSubType = MIDI_GR_TRM_BLOCK;
  blk->bGrpTrmBlkID       = 1;
  blk->bGrpTrmBlkType     = 0x00; // bidirectional (best-effort default; a legacy MIDI1 device's jack
                                   // graph doesn't map 1:1 onto GTB direction semantics)
  blk->nGroupTrm          = 0;
  blk->nNumGroupTrm        = 1;
  blk->iBlockItem         = 0;
  blk->bMIDIProtocol      = 0x01; // USB MIDI 1.0 up to 64 bits
  blk->wMaxInputBandwidth  = 0;   // unknown/not fixed
  blk->wMaxOutputBandwidth = 0;
  p_ump->num_gtb = 1;

  TU_LOG_USBH("UMPH: synthesized 1 Group Terminal Block from %u in / %u out MIDI1 jack(s)\r\n",
             p_ump->num_in_jacks, p_ump->num_out_jacks);
}

//--------------------------------------------------------------------+
// DESCRIPTOR PARSING (umph_open)
//--------------------------------------------------------------------+

// Walk one alternate-setting's endpoint (+ trailing CS endpoint) descriptors,
// recording endpoint addresses/max-packet-sizes into the alt-setting-specific
// fields of p_ump. Does NOT call tuh_edpt_open() -- deferred to set_config()
// once the active alt setting is chosen.
static void umph_parse_endpoints(umph_interface_t* p_ump, uint8_t alt_setting,
                                  uint8_t const** pp_desc, uint16_t* p_drv_len, uint16_t max_len,
                                  uint8_t num_endpoints)
{
  uint8_t const* p_desc = *pp_desc;
  uint16_t drv_len = *p_drv_len;
  uint8_t found = 0;

  while ( found < num_endpoints && drv_len < max_len )
  {
    if ( TUSB_DESC_ENDPOINT == tu_desc_type(p_desc) )
    {
      tusb_desc_endpoint_t const* desc_ep = (tusb_desc_endpoint_t const*) p_desc;
      uint8_t const ep_addr = desc_ep->bEndpointAddress;

      TU_LOG_USBH("UMPH:   alt%u endpoint 0x%02x mps=%u\r\n", alt_setting, ep_addr,
                 tu_le16toh(desc_ep->wMaxPacketSize) & 0x07FF);

      if (alt_setting == 0)
      {
        if (tu_edpt_dir(ep_addr) == TUSB_DIR_IN) p_ump->desc_ep_in0  = *desc_ep;
        else                                     p_ump->desc_ep_out0 = *desc_ep;
      }
      else
      {
        if (tu_edpt_dir(ep_addr) == TUSB_DIR_IN) p_ump->desc_ep_in1  = *desc_ep;
        else                                     p_ump->desc_ep_out1 = *desc_ep;
      }

      drv_len += tu_desc_len(p_desc);
      p_desc   = tu_desc_next(p_desc);

      // trailing class-specific MIDI streaming endpoint descriptor (associates GTB via bAssoGrpTrmBlkID)
      if ( drv_len < max_len && TUSB_DESC_CS_ENDPOINT == tu_desc_type(p_desc) )
      {
        drv_len += tu_desc_len(p_desc);
        p_desc   = tu_desc_next(p_desc);
      }

      found++;
    }
    else
    {
      drv_len += tu_desc_len(p_desc);
      p_desc   = tu_desc_next(p_desc);
    }
  }

  *pp_desc  = p_desc;
  *p_drv_len = drv_len;
}

// Walk the MIDIStreaming interface descriptor block (CS interface header,
// jack descriptors, alt-setting-0 endpoints, then -- if present -- the
// alt-setting-1 interface descriptor and its endpoints). Handles both the
// "AC + MS merged into one open() call" and "MS-only open() call" shapes.
static bool umph_parse_midistreaming(umph_interface_t* p_ump, tusb_desc_interface_t const* desc_ms,
                                      uint8_t const** pp_desc, uint16_t* p_drv_len, uint16_t max_len)
{
  p_ump->itf_num = desc_ms->bInterfaceNumber;

  uint16_t drv_len = *p_drv_len;
  uint8_t const* p_desc = *pp_desc;

  TU_LOG_USBH("UMPH: MIDIStreaming itf_num=%u alt=%u num_ep=%u\r\n",
             desc_ms->bInterfaceNumber, desc_ms->bAlternateSetting, desc_ms->bNumEndpoints);

  // CS interface header + jack/element descriptors for this alt setting
  while ( drv_len < max_len && TUSB_DESC_CS_INTERFACE == tu_desc_type(p_desc) )
  {
    uint8_t const subtype = p_desc[2];
    if      (subtype == MIDI_1_CS_INTERFACE_IN_JACK)  p_ump->num_in_jacks++;
    else if (subtype == MIDI_1_CS_INTERFACE_OUT_JACK) p_ump->num_out_jacks++;
    else if (subtype == MIDI_1_CS_INTERFACE_HEADER)
    {
      p_ump->bcdMSC0 = tu_le16toh(((midi_1_desc_header_t const*) p_desc)->bcdMSC);
    }

    drv_len += tu_desc_len(p_desc);
    p_desc   = tu_desc_next(p_desc);
  }

  // Endpoints for this (alt-setting-0) block
  umph_parse_endpoints(p_ump, 0, &p_desc, &drv_len, max_len, desc_ms->bNumEndpoints);

  // Any trailing CS interface descriptors after the endpoints
  while ( drv_len < max_len && TUSB_DESC_CS_INTERFACE == tu_desc_type(p_desc) )
  {
    drv_len += tu_desc_len(p_desc);
    p_desc   = tu_desc_next(p_desc);
  }

  // Is there a second (alt-setting-1, native UMP) interface descriptor for the same itf_num?
  if ( drv_len < max_len && TUSB_DESC_INTERFACE == tu_desc_type(p_desc) )
  {
    tusb_desc_interface_t const* desc_alt1 = (tusb_desc_interface_t const*) p_desc;

    if ( desc_alt1->bInterfaceNumber == desc_ms->bInterfaceNumber && desc_alt1->bAlternateSetting != 0 )
    {
      p_ump->alt1_available = true;

      drv_len += tu_desc_len(p_desc);
      p_desc   = tu_desc_next(p_desc);

      TU_LOG_USBH("UMPH: MIDIStreaming itf_num=%u alt=%u (native UMP) num_ep=%u\r\n",
                 desc_alt1->bInterfaceNumber, desc_alt1->bAlternateSetting, desc_alt1->bNumEndpoints);

      // CS interface descriptors for alt-1 (GTB itself is fetched separately via control
      // request, but the CS interface header with bcdMSC is typically still present inline)
      while ( drv_len < max_len && TUSB_DESC_CS_INTERFACE == tu_desc_type(p_desc) )
      {
        if (p_desc[2] == MIDI_1_CS_INTERFACE_HEADER)
        {
          p_ump->bcdMSC1 = tu_le16toh(((midi_1_desc_header_t const*) p_desc)->bcdMSC);
        }

        drv_len += tu_desc_len(p_desc);
        p_desc   = tu_desc_next(p_desc);
      }

      umph_parse_endpoints(p_ump, 1, &p_desc, &drv_len, max_len, desc_alt1->bNumEndpoints);

      while ( drv_len < max_len && TUSB_DESC_CS_INTERFACE == tu_desc_type(p_desc) )
      {
        drv_len += tu_desc_len(p_desc);
        p_desc   = tu_desc_next(p_desc);
      }
    }
  }

  *pp_desc   = p_desc;
  *p_drv_len = drv_len;
  return true;
}

bool umph_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const* itf_desc, uint16_t max_len)
{
  (void) rhport;

  TU_VERIFY(TUSB_CLASS_AUDIO == itf_desc->bInterfaceClass);

  TU_LOG_USBH("UMPH: open daddr=%u itf_num=%u class=%u/%u/%u max_len=%u\r\n",
             dev_addr, itf_desc->bInterfaceNumber, itf_desc->bInterfaceClass,
             itf_desc->bInterfaceSubClass, itf_desc->bInterfaceProtocol, max_len);

  if ( AUDIO_SUBCLASS_CONTROL == itf_desc->bInterfaceSubClass )
  {
    uint16_t drv_len = tu_desc_len(itf_desc);
    uint8_t const* p_desc = tu_desc_next(itf_desc);

    // Skip AudioControl class-specific descriptors
    while ( drv_len < max_len && TUSB_DESC_CS_INTERFACE == tu_desc_type(p_desc) )
    {
      drv_len += tu_desc_len(p_desc);
      p_desc   = tu_desc_next(p_desc);
    }

    umph_interface_t* p_ump = alloc_new_itf(dev_addr);
    TU_ASSERT(p_ump);
    p_ump->itf_num_ac = itf_desc->bInterfaceNumber;

    // Merged shape (CFG_TUH_MIDI grouping): MIDIStreaming interface follows within max_len
    if ( drv_len < max_len && TUSB_DESC_INTERFACE == tu_desc_type(p_desc) )
    {
      tusb_desc_interface_t const* desc_ms = (tusb_desc_interface_t const*) p_desc;
      if ( TUSB_CLASS_AUDIO == desc_ms->bInterfaceClass && AUDIO_SUBCLASS_MIDI_STREAMING == desc_ms->bInterfaceSubClass )
      {
        drv_len += tu_desc_len(p_desc);
        p_desc   = tu_desc_next(p_desc);
        umph_parse_midistreaming(p_ump, desc_ms, &p_desc, &drv_len, max_len);
      }
    }

    return true;
  }

  if ( AUDIO_SUBCLASS_MIDI_STREAMING == itf_desc->bInterfaceSubClass )
  {
    umph_interface_t* p_ump = find_pending_ac_itf(dev_addr, itf_desc->bInterfaceNumber);
    if (!p_ump) p_ump = alloc_new_itf(dev_addr);
    TU_ASSERT(p_ump);

    uint16_t drv_len = tu_desc_len(itf_desc);
    uint8_t const* p_desc = tu_desc_next(itf_desc);
    umph_parse_midistreaming(p_ump, itf_desc, &p_desc, &drv_len, max_len);

    return true;
  }

  TU_LOG_USBH("UMPH: open() ignoring unrecognized AUDIO subclass %u\r\n", itf_desc->bInterfaceSubClass);
  return false;
}

//--------------------------------------------------------------------+
// SET CONFIG (control-transfer state machine)
//--------------------------------------------------------------------+

enum
{
  UMPH_CFG_SELECT_ALT = 0,
  UMPH_CFG_GTB_HEADER,
  UMPH_CFG_GTB_FULL,
  UMPH_CFG_COMPLETE,
};

static void umph_process_set_config(tuh_xfer_t* xfer);

// NOTE: struct fields are set individually (no designated initializers) to
// match this repo's C++ struct-init style (see ump_device.cpp's tusb_app_drivers[]).
static bool umph_ctrl_get_gtb_descriptor(uint8_t daddr, uint8_t itf_num, uint8_t* buffer, uint16_t len,
                                          tuh_xfer_cb_t complete_cb, uintptr_t user_data)
{
  tusb_control_request_t request;
  tu_memclr(&request, sizeof(request));
  request.bmRequestType_bit.recipient = TUSB_REQ_RCPT_INTERFACE;
  request.bmRequestType_bit.type      = TUSB_REQ_TYPE_STANDARD;
  request.bmRequestType_bit.direction = TUSB_DIR_IN;
  request.bRequest = TUSB_REQ_GET_DESCRIPTOR;
  request.wValue   = tu_htole16(0x2601); // 0x26 = CS_GR_TRM_BLOCK, 0x01 = alt-setting-1 index
  request.wIndex   = tu_htole16((uint16_t) itf_num);
  request.wLength  = tu_htole16(len);

  tuh_xfer_t xfer;
  tu_memclr(&xfer, sizeof(xfer));
  xfer.daddr       = daddr;
  xfer.ep_addr     = 0;
  xfer.setup       = &request;
  xfer.buffer      = buffer;
  xfer.complete_cb = complete_cb;
  xfer.user_data   = user_data;

  return tuh_control_xfer(&xfer);
}

static bool umph_ctrl_set_alt_interface(uint8_t daddr, uint8_t itf_num, uint8_t alt_setting,
                                         tuh_xfer_cb_t complete_cb, uintptr_t user_data)
{
  tusb_control_request_t request;
  tu_memclr(&request, sizeof(request));
  request.bmRequestType_bit.recipient = TUSB_REQ_RCPT_INTERFACE;
  request.bmRequestType_bit.type      = TUSB_REQ_TYPE_STANDARD;
  request.bmRequestType_bit.direction = TUSB_DIR_OUT;
  request.bRequest = TUSB_REQ_SET_INTERFACE;
  request.wValue   = tu_htole16((uint16_t) alt_setting);
  request.wIndex   = tu_htole16((uint16_t) itf_num);
  request.wLength  = 0;

  tuh_xfer_t xfer;
  tu_memclr(&xfer, sizeof(xfer));
  xfer.daddr       = daddr;
  xfer.ep_addr     = 0;
  xfer.setup       = &request;
  xfer.buffer      = NULL;
  xfer.complete_cb = complete_cb;
  xfer.user_data   = user_data;

  return tuh_control_xfer(&xfer);
}

// Arm the next IN read, but only if there's room in rx_ff for a full
// EP_BUFSIZE chunk -- provides natural backpressure against a fast device
// outrunning a slow reader. Mirrors ump_device.cpp's _prep_out_transaction(),
// reversed (host reads FROM the device on ep_in, device reads FROM the host
// on ep_out). Called after every IN completion and after every app read().
static void umph_prep_in_read(umph_interface_t* p_ump)
{
  if (!p_ump->ep_in) return;

  TU_VERIFY(tu_fifo_remaining(&p_ump->rx_ff) >= sizeof(p_ump->ep_in_buf), );
  TU_VERIFY(usbh_edpt_claim(p_ump->daddr, p_ump->ep_in), );

  if (tu_fifo_remaining(&p_ump->rx_ff) >= sizeof(p_ump->ep_in_buf))
  {
    usbh_edpt_xfer(p_ump->daddr, p_ump->ep_in, p_ump->ep_in_buf, sizeof(p_ump->ep_in_buf));
  }
  else
  {
    usbh_edpt_release(p_ump->daddr, p_ump->ep_in);
  }
}

// Pull queued tx_ff data out to the OUT endpoint. Mirrors ump_device.cpp's
// write_flush(). Called after every app write() and after every OUT
// completion (to keep draining a backlog one EP_BUFSIZE chunk at a time).
static uint32_t umph_write_flush(umph_interface_t* p_ump)
{
  if (!p_ump->ep_out || !tu_fifo_count(&p_ump->tx_ff)) return 0;

  TU_VERIFY(usbh_edpt_claim(p_ump->daddr, p_ump->ep_out), 0);

  uint16_t count = tu_fifo_read_n(&p_ump->tx_ff, p_ump->ep_out_buf, sizeof(p_ump->ep_out_buf));
  if (count)
  {
    TU_ASSERT(usbh_edpt_xfer(p_ump->daddr, p_ump->ep_out, p_ump->ep_out_buf, count), 0);
    return count;
  }

  usbh_edpt_release(p_ump->daddr, p_ump->ep_out);
  return 0;
}

static void umph_finish_mount(umph_interface_t* p_ump)
{
  tusb_desc_endpoint_t const* desc_ep_in  = (p_ump->alt_setting == 1) ? &p_ump->desc_ep_in1  : &p_ump->desc_ep_in0;
  tusb_desc_endpoint_t const* desc_ep_out = (p_ump->alt_setting == 1) ? &p_ump->desc_ep_out1 : &p_ump->desc_ep_out0;

  if (desc_ep_in->bLength)
  {
    TU_ASSERT(tuh_edpt_open(p_ump->daddr, desc_ep_in), );
    p_ump->ep_in = desc_ep_in->bEndpointAddress;
  }
  if (desc_ep_out->bLength)
  {
    TU_ASSERT(tuh_edpt_open(p_ump->daddr, desc_ep_out), );
    p_ump->ep_out = desc_ep_out->bEndpointAddress;
  }

  tu_fifo_config(&p_ump->rx_ff, p_ump->rx_ff_buf, CFG_TUH_UMP_RX_BUFSIZE, 1, false);
  tu_fifo_config(&p_ump->tx_ff, p_ump->tx_ff_buf, CFG_TUH_UMP_TX_BUFSIZE, 1, false);

  // Arm the initial IN read so umph_xfer_cb() starts seeing incoming MIDI data.
  umph_prep_in_read(p_ump);

  p_ump->mounted = true;
  TU_LOG_USBH("UMPH: daddr=%u itf_num=%u mounted, alt=%u ep_in=0x%02x ep_out=0x%02x gtb_count=%u\r\n",
             p_ump->daddr, p_ump->itf_num, p_ump->alt_setting, p_ump->ep_in, p_ump->ep_out, p_ump->num_gtb);

  if (tuh_ump_mount_cb) tuh_ump_mount_cb(p_ump->daddr, p_ump->itf_num);

  usbh_driver_set_config_complete(p_ump->daddr, p_ump->itf_num);
}

static void umph_process_set_config(tuh_xfer_t* xfer)
{
  uintptr_t const state = xfer->user_data;
  uint8_t const itf_num = (uint8_t) tu_le16toh(xfer->setup->wIndex);
  uint8_t const daddr    = xfer->daddr;

  umph_interface_t* p_ump = find_itf(daddr, itf_num);
  TU_VERIFY(p_ump, );

  switch (state)
  {
    case UMPH_CFG_SELECT_ALT:
    {
      if (p_ump->alt1_available)
      {
        p_ump->alt_setting = 1;
        umph_ctrl_set_alt_interface(daddr, p_ump->itf_num, 1, umph_process_set_config, UMPH_CFG_GTB_HEADER);
      }
      else
      {
        p_ump->alt_setting = 0;
        // No SET_INTERFACE needed -- alt-setting-0 is already active post-configuration.
        umph_synthesize_gtb_from_jacks(p_ump);
        umph_finish_mount(p_ump);
      }
      break;
    }

    case UMPH_CFG_GTB_HEADER:
    {
      // xfer->result may be a stall if the device claims alt-1 but has no GTB descriptor --
      // fall back to synthesis rather than failing enumeration.
      if (xfer->result != XFER_RESULT_SUCCESS)
      {
        TU_LOG_USBH("UMPH: GTB header fetch failed (result=%d), synthesizing from jacks\r\n", xfer->result);
        umph_synthesize_gtb_from_jacks(p_ump);
        umph_finish_mount(p_ump);
        break;
      }

      midi2_desc_group_terminal_block_header_t const* hdr =
          (midi2_desc_group_terminal_block_header_t const*) p_ump->gtb_fetch_buf;
      uint16_t total = tu_le16toh(hdr->wTotalLength);
      if (total > UMPH_GTB_FETCH_BUFSIZE) total = UMPH_GTB_FETCH_BUFSIZE;
      if (total < sizeof(*hdr)) total = sizeof(*hdr);

      umph_ctrl_get_gtb_descriptor(daddr, p_ump->itf_num, p_ump->gtb_fetch_buf, total,
                                    umph_process_set_config, UMPH_CFG_GTB_FULL);
      break;
    }

    case UMPH_CFG_GTB_FULL:
    {
      if (xfer->result == XFER_RESULT_SUCCESS)
      {
        umph_parse_gtb_buffer(p_ump, p_ump->gtb_fetch_buf, xfer->actual_len);
      }
      else
      {
        TU_LOG_USBH("UMPH: GTB full fetch failed (result=%d), synthesizing from jacks\r\n", xfer->result);
        umph_synthesize_gtb_from_jacks(p_ump);
      }
      umph_finish_mount(p_ump);
      break;
    }

    default:
      break;
  }
}

bool umph_set_config(uint8_t dev_addr, uint8_t itf_num)
{
  umph_interface_t* p_ump = find_itf(dev_addr, itf_num);
  TU_VERIFY(p_ump);

  // AudioControl-only itf_num (unmerged enumeration shape): nothing to configure, just ack.
  if (itf_num == p_ump->itf_num_ac && itf_num != p_ump->itf_num)
  {
    usbh_driver_set_config_complete(dev_addr, itf_num);
    return true;
  }

  tusb_control_request_t request;
  request.wIndex = tu_htole16((uint16_t) itf_num);

  tuh_xfer_t xfer;
  xfer.daddr     = dev_addr;
  xfer.result    = XFER_RESULT_SUCCESS;
  xfer.setup     = &request;
  xfer.user_data = UMPH_CFG_SELECT_ALT;

  // fake xfer to kick off the state machine (matches cdc_host.c / hid_host.c pattern)
  umph_process_set_config(&xfer);

  return true;
}

//--------------------------------------------------------------------+
// TRANSFER CALLBACK
//--------------------------------------------------------------------+
// rx_ff always holds raw wire bytes regardless of alt setting -- exactly
// like ump_device.cpp's umpd_xfer_cb(). Both native UMP passthrough
// (UMP_WIRE_BSWAP32) and alt-0 MIDI1->UMP translation (umph_USBMIDI1ToUMP())
// happen lazily at read() time, not here.

bool umph_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
  umph_interface_t* p_ump = find_itf_by_ep(dev_addr, ep_addr);
  TU_VERIFY(p_ump);

  TU_LOG_USBH("UMPH: xfer_cb daddr=%u ep=0x%02x result=%d bytes=%lu\r\n",
             dev_addr, ep_addr, result, (unsigned long) xferred_bytes);

  if (ep_addr == p_ump->ep_in)
  {
    if (result == XFER_RESULT_SUCCESS && xferred_bytes > 0)
    {
      if (tuh_ump_raw_rx_cb) tuh_ump_raw_rx_cb(dev_addr, p_ump->itf_num, p_ump->ep_in_buf, (uint16_t) xferred_bytes);

      tu_fifo_write_n(&p_ump->rx_ff, p_ump->ep_in_buf, (uint16_t) xferred_bytes);

      if (tuh_ump_rx_cb) tuh_ump_rx_cb(dev_addr, p_ump->itf_num);
    }

    // re-arm the next read (a stall/error just means try again)
    umph_prep_in_read(p_ump);
  }
  else if (ep_addr == p_ump->ep_out)
  {
    // keep draining any queued tx_ff backlog, one EP_BUFSIZE chunk at a time
    umph_write_flush(p_ump);
  }

  return true;
}

//--------------------------------------------------------------------+
// INIT / DEINIT / CLOSE
//--------------------------------------------------------------------+

bool umph_init(void)
{
  tu_memclr(_umph_itf, sizeof(_umph_itf));
  return true;
}

bool umph_deinit(void)
{
  return true;
}

void umph_close(uint8_t dev_addr)
{
  // A device may hold more than one slot (one per AC/MS interface grouping)
  // -- clear all of them, not just the first match.
  for (uint8_t i = 0; i < CFG_TUH_UMP; i++)
  {
    umph_interface_t* p_ump = &_umph_itf[i];
    if (p_ump->daddr != dev_addr) continue;

    uint8_t const itf_num = p_ump->itf_num;
    bool const was_mounted = p_ump->mounted;

    tu_memclr(p_ump, sizeof(*p_ump));

    if (was_mounted && tuh_ump_umount_cb) tuh_ump_umount_cb(dev_addr, itf_num);
  }
}

//--------------------------------------------------------------------+
// ALT-SETTING-0 (LEGACY MIDI1) <-> UMP TRANSLATION
//--------------------------------------------------------------------+

// Helper routine to handle conversion of a USB MIDI 1.0 32-bit word packet
// to UMP formatted packet. Only populates UMP message types 1 (System), 2
// (MIDI 1.0 Channel Voice), and 3 (64-bit data / SysEx7) -- ported verbatim
// (pure byte-level conversion, no device/host role dependence) from
// ump_device.cpp's tud_USBMIDI1ToUMP(), which itself was refined from the
// USB MIDI 2.0 Host Driver developed as open source to be included in
// Windows by the Association of Musical Electronics Industry.
//
// Copyright 2023 Association of Musical Electronics Industry
// Copyright 2023 Microsoft
// Driver source code developed by AmeNote. Some components Copyright 2023 AmeNote Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
static bool umph_USBMIDI1ToUMP(uint32_t usbMidi1Pkt, bool* pbIsInSysex, PUMPH_PACKET umpPkt)
{
  if (!usbMidi1Pkt || !pbIsInSysex || !umpPkt) return false;

  uint8_t* pBuffer = (uint8_t*) &usbMidi1Pkt;
  umpPkt->wordCount = 0;

  uint8_t cbl_num = (pBuffer[0] & 0xf0) >> 4;
  uint8_t code_index = pBuffer[0] & 0x0f;

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
        if (pBuffer[1] != MIDI_1_STATUS_SYSEX_START) return false;
        firstByte = 2;
        lastByte = 4;
        umpPkt->umpData.umpBytes[1] = UMP_SYSEX7_START | 2;
        *pbIsInSysex = true;
      }
      else
      {
        firstByte = 1;
        lastByte = 4;
        umpPkt->umpData.umpBytes[1] = UMP_SYSEX7_CONTINUE | 3;
      }

      umpPkt->umpData.umpBytes[0] = UMP_MT_DATA_64 | cbl_num;
      umpPkt->wordCount = 2;
      copyPos = firstByte;
      for (uint8_t count = 2; count < 8; count++)
      {
        umpPkt->umpData.umpBytes[count] = (copyPos < lastByte) ? pBuffer[copyPos++] : 0x00;
      }
      break;

    case MIDI_1_CIN_SYSEX_END_1BYTE: // or single byte System Common
      if ((pBuffer[1] & 0x80) && (pBuffer[1] != MIDI_1_STATUS_SYSEX_END))
      {
        umpPkt->umpData.umpBytes[0] = UMP_MT_SYSTEM | cbl_num;
        umpPkt->umpData.umpBytes[1] = pBuffer[1];
        firstByte = 1;
        lastByte = 1;
        goto COMPLETE_1BYTE;
      }

      umpPkt->umpData.umpBytes[0] = UMP_MT_DATA_64 | cbl_num;

      if (*pbIsInSysex)
      {
        if (pBuffer[1] != MIDI_1_STATUS_SYSEX_END) return false;
        umpPkt->umpData.umpBytes[1] = UMP_SYSEX7_END | 0;
        *pbIsInSysex = false;
        firstByte = 1;
        lastByte = 1;
      }
      else
      {
        return false; // should not get here
      }

COMPLETE_1BYTE:
      umpPkt->wordCount = 2;
      copyPos = firstByte;
      for (uint8_t count = 2; count < 8; count++)
      {
        umpPkt->umpData.umpBytes[count] = (copyPos < lastByte) ? pBuffer[copyPos++] : 0x00;
      }
      break;

    case MIDI_1_CIN_SYSEX_END_2BYTE:
      umpPkt->umpData.umpBytes[0] = UMP_MT_DATA_64 | cbl_num;

      if (*pbIsInSysex)
      {
        if (pBuffer[2] != MIDI_1_STATUS_SYSEX_END) return false;
        umpPkt->umpData.umpBytes[1] = UMP_SYSEX7_END | 1;
        *pbIsInSysex = false;
        firstByte = 1;
        lastByte = 2;
      }
      else
      {
        umpPkt->umpData.umpBytes[1] = UMP_SYSEX7_COMPLETE | 0;
        *pbIsInSysex = false;
        firstByte = 1;
        lastByte = 1;
      }

      umpPkt->wordCount = 2;
      copyPos = firstByte;
      for (uint8_t count = 2; count < 8; count++)
      {
        umpPkt->umpData.umpBytes[count] = (copyPos < lastByte) ? pBuffer[copyPos++] : 0x00;
      }
      break;

    case MIDI_1_CIN_SYSEX_END_3BYTE:
      umpPkt->umpData.umpBytes[0] = UMP_MT_DATA_64 | cbl_num;

      if (*pbIsInSysex)
      {
        if (pBuffer[3] != MIDI_1_STATUS_SYSEX_END) return false;
        umpPkt->umpData.umpBytes[1] = UMP_SYSEX7_END | 2;
        *pbIsInSysex = false;
        firstByte = 1;
        lastByte = 3;
      }
      else
      {
        if (pBuffer[1] != MIDI_1_STATUS_SYSEX_START || pBuffer[3] != MIDI_1_STATUS_SYSEX_END) return false;
        umpPkt->umpData.umpBytes[1] = UMP_SYSEX7_COMPLETE | 1;
        *pbIsInSysex = false;
        firstByte = 2;
        lastByte = 3;
      }

      umpPkt->wordCount = 2;
      copyPos = firstByte;
      for (uint8_t count = 2; count < 8; count++)
      {
        umpPkt->umpData.umpBytes[count] = (copyPos < lastByte) ? pBuffer[copyPos++] : 0x00;
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
      umpPkt->umpData.umpBytes[0] = UMP_MT_MIDI1_CV | cbl_num;
      *pbIsInSysex = false; // ensure we end any current sysex packets, other layers need to handle error
      for (int count = 1; count < 4; count++) umpPkt->umpData.umpBytes[count] = pBuffer[count];
      umpPkt->wordCount = 1;
      break;

    case MIDI_1_CIN_SYSCOM_2BYTE:
    case MIDI_1_CIN_SYSCOM_3BYTE:
      umpPkt->umpData.umpBytes[0] = UMP_MT_SYSTEM | cbl_num;
      for (int count = 1; count < 4; count++) umpPkt->umpData.umpBytes[count] = pBuffer[count];
      umpPkt->wordCount = 1;
      break;

    case MIDI_1_CIN_MISC:
    case MIDI_1_CIN_CABLE_EVENT:
      // Reserved for future use, not translated
    default:
      return false;
  }

  return true;
}

//--------------------------------------------------------------------+
// APPLICATION API
//--------------------------------------------------------------------+

bool tuh_ump_mounted(uint8_t daddr, uint8_t itf_num)
{
  umph_interface_t* p_ump = find_itf(daddr, itf_num);
  return p_ump && p_ump->mounted;
}

uint8_t tuh_ump_alt_setting(uint8_t daddr, uint8_t itf_num)
{
  umph_interface_t* p_ump = find_itf(daddr, itf_num);
  return p_ump ? p_ump->alt_setting : 0;
}

uint16_t tuh_ump_get_bcd_msc(uint8_t daddr, uint8_t itf_num)
{
  umph_interface_t* p_ump = find_itf(daddr, itf_num);
  if (!p_ump) return 0;
  return (p_ump->alt_setting == 1) ? p_ump->bcdMSC1 : p_ump->bcdMSC0;
}

uint8_t tuh_ump_get_group_terminal_blocks(uint8_t daddr, uint8_t itf_num,
                                           midi2_desc_group_terminal_block_t const** gtb_array_out)
{
  umph_interface_t* p_ump = find_itf(daddr, itf_num);
  if (!p_ump)
  {
    if (gtb_array_out) *gtb_array_out = NULL;
    return 0;
  }

  if (gtb_array_out) *gtb_array_out = p_ump->gtb;
  return p_ump->num_gtb;
}

// NOTE: like ump_device.cpp's tud_ump_n_available()/n_writeable(), these
// report raw fifo occupancy / 4, which is only an approximation of true UMP
// word count for alt-setting-0 (a single 4-byte fifo entry can translate to
// 0, 1, or 2 UMP words) -- an accepted imprecision inherited from the
// device driver's existing behavior, not something introduced here.

uint32_t tuh_ump_available(uint8_t daddr, uint8_t itf_num)
{
  umph_interface_t* p_ump = find_itf(daddr, itf_num);
  return p_ump ? tu_fifo_count(&p_ump->rx_ff) / 4 : 0;
}

uint32_t tuh_ump_writeable(uint8_t daddr, uint8_t itf_num)
{
  umph_interface_t* p_ump = find_itf(daddr, itf_num);
  return p_ump ? tu_fifo_remaining(&p_ump->tx_ff) / 4 : 0;
}

static uint16_t umph_read_impl(uint8_t daddr, uint8_t itf_num, uint32_t* pkts, uint16_t numAvail, bool hostOrder)
{
  umph_interface_t* p_ump = find_itf(daddr, itf_num);
  if (!p_ump) return 0;

  uint16_t numRead = 0;

  if (p_ump->alt_setting == 1)
  {
    uint8_t umpBuffer[4];
    while (numRead < numAvail && tu_fifo_read_n(&p_ump->rx_ff, umpBuffer, 4) == 4)
    {
      // Wire format is little-endian (byte 0 = LSB, byte 3 = MT nibble) per
      // USB MIDI 2.0 spec section 3.2.2; UMP_WIRE_BSWAP32 reconstructs the
      // host-native arithmetic value (MT in bits 31:28) regardless of host
      // endianness. Legacy raw callers get the byte-buffer reinterpretation
      // (host-endian dependent), matching tud_ump_read()'s existing behavior.
      pkts[numRead++] = hostOrder ? UMP_WIRE_BSWAP32(*(uint32_t*) umpBuffer) : *(uint32_t*) umpBuffer;
    }
  }
  else
  {
    // Legacy MIDI1: rx_ff holds raw 4-byte USB-MIDI1 CIN packets; translate
    // to UMP here (lazily, at read time), mirroring ump_device.cpp's
    // tud_ump_read_impl() alt-0 branch exactly, including the 2-word
    // headroom (a single CIN packet can expand to a 2-word SysEx7 UMP msg).
    uint16_t numProcessed = 0;
    while ((numAvail - numProcessed) >= 2)
    {
      uint32_t readWord;
      if (tu_fifo_read_n(&p_ump->rx_ff, (void*) &readWord, sizeof(uint32_t)) != sizeof(uint32_t)) break;
      numProcessed++;

      if (readWord)
      {
        uint8_t cbl_num = (((uint8_t*) &readWord)[0] & 0xf0) >> 4;
        UMPH_PACKET pkt;
        if (umph_USBMIDI1ToUMP(readWord, &p_ump->midi1_rx_is_in_sysex[cbl_num], &pkt))
        {
          for (uint8_t count = 0; count < pkt.wordCount; count++)
          {
            uint32_t word = pkt.umpData.umpWords[count];
            pkts[numRead++] = hostOrder ? UMP_HOST_BSWAP32(word) : word;
          }
        }
      }
    }
  }

  umph_prep_in_read(p_ump); // fifo has room again -- re-arm if we'd stalled on backpressure
  return numRead;
}

// Legacy raw interface: words are reinterpreted from the wire byte buffer
// with no endian conversion (host-endian dependent).
uint16_t tuh_ump_read(uint8_t daddr, uint8_t itf_num, uint32_t* words, uint16_t numAvail)
{
  return umph_read_impl(daddr, itf_num, words, numAvail, false);
}

// Portable interface: each returned word is the host-native uint32_t whose
// arithmetic value matches the UMP wire word (bits 31:28 = message type).
// Recommended for new code.
uint16_t tuh_ump_read_ntoh(uint8_t daddr, uint8_t itf_num, uint32_t* words, uint16_t numAvail)
{
  return umph_read_impl(daddr, itf_num, words, numAvail, true);
}

static uint16_t umph_write_impl(uint8_t daddr, uint8_t itf_num, uint32_t* words, uint16_t numWords, bool hostOrder)
{
  umph_interface_t* p_ump = find_itf(daddr, itf_num);
  if (!p_ump) return 0;

  uint16_t numProcessed = 0;

  if (p_ump->alt_setting == 1)
  {
    uint16_t const numAvailable = tu_fifo_remaining(&p_ump->tx_ff) / 4;
    numProcessed = (numAvailable < numWords) ? numAvailable : numWords;

    if (hostOrder)
    {
      // words[] is numeric (host-native order, MT in bits 31:28); the wire
      // needs little-endian byte order (byte 0 = LSB, byte 3 = MT nibble) per
      // USB MIDI 2.0 spec section 3.2.2. Swap in fixed-size batches, matching
      // ump_device.cpp's tud_ump_write_impl() native-passthrough branch.
      uint32_t wireWords[16];
      uint16_t offset = 0, remaining = numProcessed;
      while (remaining)
      {
        uint16_t chunk = (remaining < TU_ARRAY_SIZE(wireWords)) ? remaining : TU_ARRAY_SIZE(wireWords);
        for (uint16_t i = 0; i < chunk; i++) wireWords[i] = UMP_WIRE_BSWAP32(words[offset + i]);
        tu_fifo_write_n(&p_ump->tx_ff, wireWords, chunk * 4);
        offset += chunk;
        remaining -= chunk;
      }
    }
    else
    {
      // Legacy raw reinterpretation (host-endian dependent).
      tu_fifo_write_n(&p_ump->tx_ff, words, numProcessed * 4);
    }
  }
  else
  {
    // Legacy MIDI1: translate UMP -> USB-MIDI1 CIN packets (with per-cable
    // SysEx7 ring-buffer reassembly), mirroring ump_device.cpp's
    // tud_ump_write_impl() alt-0 branch.
    UMPH_PACKET umpPacket;
    UMPH_PACKET umpWritePacket;

    while (numProcessed < numWords)
    {
      umpPacket.wordCount = 0;
      umpPacket.umpData.umpWords[0] = hostOrder ? UMP_HOST_BSWAP32(words[numProcessed]) : words[numProcessed];

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
          numProcessed++;
          continue;
      }

      if ((numWords - numProcessed) < umpPacket.wordCount) break; // not enough data this call

      for (int count = 1; count < umpPacket.wordCount; count++)
      {
        umpPacket.umpData.umpWords[count] =
          hostOrder ? UMP_HOST_BSWAP32(words[numProcessed + count]) : words[numProcessed + count];
      }

      uint8_t cbl_num = umpPacket.umpData.umpBytes[0] & UMP_GROUP_MASK;
      uint8_t mtVal = umpPacket.umpData.umpBytes[0] & UMP_MT_MASK;
      umpWritePacket.wordCount = 0;

      switch (mtVal)
      {
        case UMP_MT_SYSTEM:
          umpWritePacket.wordCount = 1;
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

        case UMP_MT_DATA_64:
        {
          bool bEnterSysex = false;
          bool bEndSysex = false;

          switch (umpPacket.umpData.umpBytes[1] & UMP_SYSEX7_STATUS_MASK)
          {
            case UMP_SYSEX7_COMPLETE:
              bEnterSysex = true;
              // fallthrough
            case UMP_SYSEX7_END:
              bEndSysex = true;
              break;
            case UMP_SYSEX7_START:
              bEnterSysex = true;
              break;
            default:
              break;
          }

          UMPH_TO_MIDI1_SYSEX* sx = &p_ump->midi1_tx_sysex[cbl_num];

          if (bEnterSysex && sx->inSysex) sx->inSysex = false;
          if (bEnterSysex && !sx->inSysex)
          {
            sx->usbMIDI1Head = 0;
            sx->usbMIDI1Tail = 0;
            sx->inSysex = true;
          }

          uint8_t byteStream[UMPH_SYSEX_BS_RB_SIZE];
          uint8_t sysexStatus = (umpPacket.umpData.umpBytes[1] >> 4);
          uint8_t numberBytes = 0;

          if (sysexStatus <= 1 && numberBytes < UMPH_SYSEX_BS_RB_SIZE)
          {
            byteStream[numberBytes++] = MIDI_1_STATUS_SYSEX_START;
          }
          for (uint8_t count = 0; count < (umpPacket.umpData.umpBytes[1] & 0xf); count++)
          {
            if (numberBytes < UMPH_SYSEX_BS_RB_SIZE) byteStream[numberBytes++] = umpPacket.umpData.umpBytes[2 + count];
          }
          if ((sysexStatus == 0 || sysexStatus == 3) && numberBytes < UMPH_SYSEX_BS_RB_SIZE)
          {
            byteStream[numberBytes++] = MIDI_1_STATUS_SYSEX_END;
          }

          for (uint8_t count = 0; count < numberBytes; count++)
          {
            sx->sysexBS[sx->usbMIDI1Head++] = byteStream[count];
            sx->usbMIDI1Head %= UMPH_SYSEX_BS_RB_SIZE;
          }

          numberBytes = (sx->usbMIDI1Head > sx->usbMIDI1Tail)
              ? sx->usbMIDI1Head - sx->usbMIDI1Tail
              : (UMPH_SYSEX_BS_RB_SIZE - sx->usbMIDI1Tail) + sx->usbMIDI1Head;

          umpWritePacket.wordCount = 0;
          while (numberBytes && umpWritePacket.wordCount < 4)
          {
            umpWritePacket.umpData.umpWords[umpWritePacket.wordCount] = 0;

            if (numberBytes > 2)
            {
              uint8_t* pumpBytes = (uint8_t*) &umpWritePacket.umpData.umpWords[umpWritePacket.wordCount];
              for (uint8_t count = 0; count < 3; count++)
              {
                pumpBytes[count + 1] = sx->sysexBS[sx->usbMIDI1Tail++];
                sx->usbMIDI1Tail %= UMPH_SYSEX_BS_RB_SIZE;
                numberBytes--;
              }
              pumpBytes[0] = (bEndSysex && !numberBytes)
                  ? (uint8_t) (cbl_num << 4) | MIDI_1_CIN_SYSEX_END_3BYTE
                  : (uint8_t) (cbl_num << 4) | MIDI_1_CIN_SYSEX_START;
            }
            else if (bEndSysex)
            {
              uint8_t* pumpBytes = (uint8_t*) &umpWritePacket.umpData.umpWords[umpWritePacket.wordCount];
              uint8_t count;
              for (count = 0; numberBytes; count++)
              {
                pumpBytes[count + 1] = sx->sysexBS[sx->usbMIDI1Tail++];
                sx->usbMIDI1Tail %= UMPH_SYSEX_BS_RB_SIZE;
                numberBytes--;
              }
              pumpBytes[0] = (count == 1)
                  ? (uint8_t) (cbl_num << 4) | MIDI_1_CIN_SYSEX_END_1BYTE
                  : (uint8_t) (cbl_num << 4) | MIDI_1_CIN_SYSEX_END_2BYTE;
            }
            else
            {
              break;
            }
            umpWritePacket.wordCount++;
          }
          break;
        }

        default:
          numProcessed += umpPacket.wordCount; // ignore this UMP packet as corrupted
          umpWritePacket.wordCount = 0;
      }

      if (umpWritePacket.wordCount)
      {
        numProcessed += umpPacket.wordCount;
        tu_fifo_write_n(&p_ump->tx_ff, (void*) &umpWritePacket.umpData.umpBytes[0], umpWritePacket.wordCount * 4);
      }
    }
  }

  umph_write_flush(p_ump);
  return numProcessed;
}

// Legacy raw interface: words[] is reinterpreted onto the wire with no
// endian conversion (host-endian dependent).
uint16_t tuh_ump_write(uint8_t daddr, uint8_t itf_num, uint32_t* words, uint16_t numWords)
{
  return umph_write_impl(daddr, itf_num, words, numWords, false);
}

// Portable interface: words[] must be the host-native uint32_t numeric value
// of each UMP word (bits 31:28 = message type). Recommended for new code.
uint16_t tuh_ump_write_hton(uint8_t daddr, uint8_t itf_num, uint32_t* words, uint16_t numWords)
{
  return umph_write_impl(daddr, itf_num, words, numWords, true);
}

//--------------------------------------------------------------------+
// DRIVER REGISTRATION (out-of-tree, mirrors ump_device.cpp's usbd_app_driver_get_cb)
//--------------------------------------------------------------------+

// Field order must match usbh_class_driver_t exactly (src/host/usbh_pvt.h):
// name, init, deinit, open, set_config, xfer_cb, close.
static usbh_class_driver_t const umph_driver =
{
  "UMP-HOST",
  umph_init,
  umph_deinit,
  umph_open,
  umph_set_config,
  umph_xfer_cb,
  umph_close,
};

usbh_class_driver_t const* usbh_app_driver_get_cb(uint8_t* driver_count)
{
  *driver_count = 1;
  return &umph_driver;
}

#endif // CFG_TUH_ENABLED && CFG_TUH_UMP
