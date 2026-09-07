/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2026 Michael Loh (AmeNote.com)
 *
 * tusb_ump_host_demo -- a small reference app for `ump_host`, the USB Host
 * UMP TinyUSB class driver: enumerates whatever USB MIDI device is plugged
 * in (native UMP or legacy USB-MIDI 1.0), dumps its parsed (or synthesized,
 * for alt-setting-0-only devices) Group Terminal Block info over UART,
 * reports basic device identification for any attached USB device (not
 * just MIDI ones), decodes and prints every incoming UMP word via the real
 * read API (both alt settings), and injects a test Note On/Off once a
 * second to exercise the write path (toggle with SPACE -- see README.md's
 * Runtime Controls).
 *
 * Not a product -- meant as a self-contained reference for the API
 * (`tuh_ump_read_ntoh` / `tuh_ump_write_hton`, mount/unmount callbacks)
 * needed to build a real USB-MIDI-host application with this driver.
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

#include <cstdio>

#include "pico/stdlib.h"
#include "tusb.h"
#include "host/hcd.h"
#include "ump_host.h"

#define LANGUAGE_ID 0x0409 // English (US)

//--------------------------------------------------------------------+
// UTF-16LE -> UTF-8 string descriptor helper (same approach as TinyUSB's
// own host/device_info example)
//--------------------------------------------------------------------+

static void print_utf16(uint16_t* temp_buf, size_t buf_len)
{
  if ((temp_buf[0] & 0xff) == 0) { printf("(none)"); return; }

  size_t utf16_len = ((temp_buf[0] & 0xff) - 2) / sizeof(uint16_t);
  uint8_t* utf8 = (uint8_t*) temp_buf;
  size_t out_pos = 0;

  for (size_t i = 0; i < utf16_len && out_pos + 3 < buf_len * sizeof(uint16_t); i++)
  {
    uint16_t chr = temp_buf[1 + i];
    if (chr < 0x80)
    {
      utf8[out_pos++] = (uint8_t) chr;
    }
    else if (chr < 0x800)
    {
      utf8[out_pos++] = (uint8_t) (0xC0 | (chr >> 6 & 0x1F));
      utf8[out_pos++] = (uint8_t) (0x80 | (chr >> 0 & 0x3F));
    }
    else
    {
      utf8[out_pos++] = (uint8_t) (0xE0 | (chr >> 12 & 0x0F));
      utf8[out_pos++] = (uint8_t) (0x80 | (chr >> 6 & 0x3F));
      utf8[out_pos++] = (uint8_t) (0x80 | (chr >> 0 & 0x3F));
    }
  }
  utf8[out_pos] = '\0';

  printf("%s", (char*) utf8);
}

static void print_string_desc(uint8_t daddr, uint8_t index, char const* label)
{
  if (index == 0)
  {
    printf("  %s: (no string descriptor)\r\n", label);
    return;
  }

  uint16_t buf[64];
  printf("  %s: ", label);
  if (XFER_RESULT_SUCCESS == tuh_descriptor_get_string_sync(daddr, index, LANGUAGE_ID, buf, sizeof(buf)))
  {
    print_utf16(buf, sizeof(buf) / 2);
  }
  else
  {
    printf("(failed to read)");
  }
  printf("\r\n");
}

//--------------------------------------------------------------------+
// UMP class driver diagnostics
//--------------------------------------------------------------------+

static void print_gtb_info(uint8_t daddr, uint8_t itf_num)
{
  midi2_desc_group_terminal_block_t const* gtb = NULL;
  uint8_t const count = tuh_ump_get_group_terminal_blocks(daddr, itf_num, &gtb);
  uint8_t const alt = tuh_ump_alt_setting(daddr, itf_num);

  printf("  alt_setting = %u (%s)\r\n", alt, alt == 1 ? "native UMP" : "legacy MIDI 1.0");
  printf("  USB MIDI Streaming class spec version (bcdMSC) = 0x%04x\r\n", tuh_ump_get_bcd_msc(daddr, itf_num));
  printf("  Group Terminal Blocks: %u\r\n", count);

  for (uint8_t i = 0; i < count; i++)
  {
    midi2_desc_group_terminal_block_t const* blk = &gtb[i];
    printf("    [%u] ID=%u type=0x%02x firstGroup=%u numGroups=%u protocol=0x%02x "
           "maxInBW=%u maxOutBW=%u\r\n",
           i, blk->bGrpTrmBlkID, blk->bGrpTrmBlkType, blk->nGroupTrm, blk->nNumGroupTrm,
           blk->bMIDIProtocol, blk->wMaxInputBandwidth, blk->wMaxOutputBandwidth);
  }
}

// Tracks every currently-mounted UMP interface -- multiple devices can be
// mounted at once (e.g. several USB MIDI devices behind a hub, see
// CFG_TUH_UMP/CFG_TUH_DEVICE_MAX in tusb_config.h), so poll_ump_read() and
// send_test_note() below iterate this table rather than a single "the
// mounted device" global.
#define MAX_MOUNTED_UMP CFG_TUH_UMP
typedef struct { uint8_t daddr; uint8_t itf_num; bool active; } mounted_ump_t;
static mounted_ump_t s_mounted[MAX_MOUNTED_UMP];

static mounted_ump_t* find_mounted(uint8_t daddr, uint8_t itf_num)
{
  for (uint8_t i = 0; i < MAX_MOUNTED_UMP; i++)
  {
    if (s_mounted[i].active && s_mounted[i].daddr == daddr && s_mounted[i].itf_num == itf_num) return &s_mounted[i];
  }
  return NULL;
}

// Invoked when a UMP interface finishes enumeration (ump_host.cpp)
void tuh_ump_mount_cb(uint8_t daddr, uint8_t itf_num)
{
  printf("\r\n[UMP] mounted daddr=%u itf_num=%u\r\n", daddr, itf_num);
  print_gtb_info(daddr, itf_num);

  for (uint8_t i = 0; i < MAX_MOUNTED_UMP; i++)
  {
    if (!s_mounted[i].active)
    {
      s_mounted[i].daddr    = daddr;
      s_mounted[i].itf_num  = itf_num;
      s_mounted[i].active   = true;
      return;
    }
  }

  printf("[UMP] WARNING: mounted-device table full (MAX_MOUNTED_UMP=%u) -- daddr=%u itf_num=%u won't be polled/exercised\r\n",
         MAX_MOUNTED_UMP, daddr, itf_num);
}

void tuh_ump_umount_cb(uint8_t daddr, uint8_t itf_num)
{
  printf("[UMP] unmounted daddr=%u itf_num=%u\r\n", daddr, itf_num);

  mounted_ump_t* m = find_mounted(daddr, itf_num);
  if (m) m->active = false;
}

void tuh_ump_rx_cb(uint8_t daddr, uint8_t itf_num)
{
  (void) daddr; (void) itf_num;
}

//--------------------------------------------------------------------+
// Generic USB device diagnostics (any attached device, not just MIDI)
//--------------------------------------------------------------------+

// Invoked when any USB device is mounted (TinyUSB host stack, not UMP-specific) --
// prints basic identification even for non-MIDI devices, useful to confirm
// enumeration is happening at all during bring-up.
void tuh_mount_cb(uint8_t daddr)
{
  tusb_desc_device_t desc_device;
  if (XFER_RESULT_SUCCESS != tuh_descriptor_get_device_sync(daddr, &desc_device, sizeof(desc_device)))
  {
    printf("\r\n[USB] device mounted daddr=%u (failed to read device descriptor)\r\n", daddr);
    return;
  }

  printf("\r\n[USB] device mounted daddr=%u VID:PID=%04x:%04x bcdUSB=0x%04x bcdDevice=0x%04x "
         "class=%u/%u/%u\r\n",
         daddr, desc_device.idVendor, desc_device.idProduct, desc_device.bcdUSB, desc_device.bcdDevice,
         desc_device.bDeviceClass, desc_device.bDeviceSubClass, desc_device.bDeviceProtocol);

  print_string_desc(daddr, desc_device.iManufacturer, "Manufacturer");
  print_string_desc(daddr, desc_device.iProduct, "Product");
  print_string_desc(daddr, desc_device.iSerialNumber, "Serial Number");
}

void tuh_umount_cb(uint8_t daddr)
{
  printf("[USB] device removed daddr=%u\r\n", daddr);
}

//--------------------------------------------------------------------+
// Pull decoded UMP words via the real FIFO-backed tuh_ump_read_ntoh() API,
// and inject a test Note On/Off every second via tuh_ump_write_hton() to
// exercise the write/TX path too -- both alt settings.
//--------------------------------------------------------------------+

// Number of 32-bit words in a UMP message of the given Message Type (bits
// 31:28 of its first word), per the UMP spec -- e.g. MIDI 2.0 Channel Voice
// (type 4) is always 2 words. Mirrors ump_host.cpp's umph_write_impl() word-
// count switch.
static uint8_t ump_mt_word_count(uint32_t mt)
{
  switch (mt)
  {
    case 0x0: case 0x1: case 0x2: case 0x6: case 0x7: return 1;
    case 0x3: case 0x4: case 0x8: case 0x9: case 0xA: return 2;
    case 0xB: case 0xC:                                return 3;
    case 0x5: case 0xD: case 0xE: case 0xF:            return 4;
    default:                                            return 1;
  }
}

static void poll_ump_read_one(uint8_t daddr, uint8_t itf_num)
{
  uint32_t words[16];
  uint16_t count = tuh_ump_read_ntoh(daddr, itf_num, words, TU_ARRAY_SIZE(words));
  uint16_t i = 0;
  while (i < count)
  {
    unsigned long const mt = (unsigned long) (words[i] >> 28);
    printf("[UMP daddr=%u itf=%u] read word: 0x%08lx (MT=0x%lx group=%lu)\r\n",
           daddr, itf_num, (unsigned long) words[i], mt, (unsigned long) ((words[i] >> 24) & 0xF));
    i++;

    // Remaining words of this same multi-word message are payload/data, not
    // independent UMP words -- their top nibble isn't a Message Type, so
    // don't mislabel them with a bogus (MT=... group=...) decode.
    uint8_t remaining = (uint8_t) (ump_mt_word_count((uint32_t) mt) - 1);
    while (remaining && i < count)
    {
      printf("[UMP]            0x%08lx\r\n", (unsigned long) words[i]);
      i++;
      remaining--;
    }
  }
}

// Multiple devices can be mounted at once -- poll every one of them each
// iteration rather than just "the" (most recently mounted) device.
static void poll_ump_read(void)
{
  for (uint8_t i = 0; i < MAX_MOUNTED_UMP; i++)
  {
    if (s_mounted[i].active) poll_ump_read_one(s_mounted[i].daddr, s_mounted[i].itf_num);
  }
}

static void send_test_note_one(uint8_t daddr, uint8_t itf_num, bool note_on)
{
  uint16_t written;
  if (tuh_ump_alt_setting(daddr, itf_num) == 1)
  {
    // UMP MIDI 2.0 Channel Voice Note On/Off, group 0, channel 0, note 60 (middle C)
    uint32_t ump[2];
    ump[0] = (0x4u << 28) | (0x0u << 24) | ((note_on ? 0x9u : 0x8u) << 20) | (0x0u << 16) | (60u << 8) | 0x00u;
    ump[1] = (note_on ? 0x7FFFu : 0x0000u) << 16;
    written = tuh_ump_write_hton(daddr, itf_num, ump, 2);
  }
  else
  {
    // UMP MIDI 1.0 Channel Voice (message type 2) Note On/Off, group 0,
    // channel 0, note 60, velocity 100 -- translated to a legacy USB-MIDI1
    // CIN packet by the driver's alt-0 write path.
    uint32_t ump[1];
    ump[0] = (0x2u << 28) | (0x0u << 24) | ((note_on ? 0x9u : 0x8u) << 20) | (0x0u << 16) | (60u << 8) |
             (note_on ? 100u : 0u);
    written = tuh_ump_write_hton(daddr, itf_num, ump, 1);
  }

  printf("[UMP daddr=%u itf=%u] sent test Note %s (wrote %u words)\r\n",
         daddr, itf_num, note_on ? "On" : "Off", written);
}

// Exercise the TX path on every mounted device at once, so a multi-device
// hub test actually validates write, not just read, on each of them.
static void send_test_note(bool note_on)
{
  for (uint8_t i = 0; i < MAX_MOUNTED_UMP; i++)
  {
    if (s_mounted[i].active) send_test_note_one(s_mounted[i].daddr, s_mounted[i].itf_num, note_on);
  }
}

int main()
{
  stdio_init_all();

  printf("\r\n\r\ntusb_ump_host_demo -- USB Host UMP reference app\r\n");
  printf("Waiting for a USB MIDI device...\r\n");

  tusb_rhport_init_t host_init = {};
  host_init.role  = TUSB_ROLE_HOST;
  host_init.speed = TUSB_SPEED_AUTO;
  tusb_init(BOARD_TUH_RHPORT, &host_init);

  // RP2040's host controller only notifies TinyUSB of a device via an edge-
  // triggered connect interrupt -- if a device was already plugged in before
  // tusb_init() ran, there's no edge to catch and it's silently never
  // enumerated (this shows up as needing an unplug/replug on cold boot).
  // Work around it by manually checking connect status once at startup and,
  // if already connected, injecting the same attach event the IRQ handler
  // would have fired.
  if (hcd_port_connect_status(BOARD_TUH_RHPORT))
  {
    printf("Device already attached at boot -- forcing enumeration\r\n");
    hcd_event_device_attach(BOARD_TUH_RHPORT, false);
  }

  printf("Press SPACE to toggle the periodic test Note On/Off write (starts OFF).\r\n");

  uint32_t last_note_ms = 0;
  bool note_on_next = true;
  bool write_enabled = false;

  while (true)
  {
    tuh_task();
    poll_ump_read();

    int c = getchar_timeout_us(0);
    if (c == ' ')
    {
      write_enabled = !write_enabled;
      printf("[UMP] test Note write %s\r\n", write_enabled ? "ENABLED" : "disabled");
    }

    if (write_enabled)
    {
      uint32_t now = to_ms_since_boot(get_absolute_time());
      if (now - last_note_ms >= 1000)
      {
        last_note_ms = now;
        send_test_note(note_on_next);
        note_on_next = !note_on_next;
      }
    }
  }

  return 0;
}
