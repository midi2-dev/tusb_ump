/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023-2026 Michael Loh (AmeNote.com)
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

// tusb_ump_lb - canned USB MIDI 2.0 (UMP) loopback example.
//
// Any UMP word(s) received on the single UMP endpoint are written straight
// back out as soon as tud_task() sees them, on whichever alternate setting
// (MIDI 1.0 byte stream / Alt 0, or UMP / Alt 1) the host has selected.
//
// Board bring-up (stdio_init_all/gpio) below is the only RP2040-specific
// part of this file - the loopback logic and USB MIDI handling call only
// TinyUSB (tud_task/tud_init) and the generic ump_device.c/h driver, so
// porting to another pico-sdk board or another MCU's TinyUSB port only
// requires adjusting this bring-up code.

#include "pico/stdlib.h"
#include "tusb.h"
#include "ump_device.h"

static void led_task(void);
static void log_echoed(uint8_t alt, const uint32_t *words, uint16_t count);

int main(void)
{
    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    printf("tusb_ump_lb: USB MIDI 2.0 (UMP) loopback starting\n");

    tusb_init();

    while (true)
    {
        tud_task(); // let TinyUSB fill the UMP RX FIFO / drain the TX FIFO
        led_task();

        // Drain everything currently queued, one UMP message (<=4 words) at a
        // time, echoing each straight back out.
        while (tud_ump_n_mounted(0) && tud_ump_n_available(0))
        {
            uint32_t words[4];
            uint16_t count = tud_ump_read_ntoh(0, words, 4);

            if (count == 0)
            {
                break;
            }

            log_echoed(tud_alt_setting(0), words, count);
            tud_ump_write_hton(0, words, count);
        }
    }

    return 0;
}

// Invoked when the host mounts the device or switches MIDIStreaming
// alternate setting (0 = legacy MIDI 1.0 byte stream, 1 = UMP / MIDI 2.0)
void tud_ump_set_itf_cb(uint8_t itf, uint8_t alt)
{
    (void) itf;
    printf("tusb_ump_lb: UMP interface active, alt setting %u\n", alt);
}

// Logs each echoed message over stdio (the board's default UART) so the
// loopback is visible without a USB MIDI 2.0 host attached, e.g. while
// bringing up a new port. alt is the MIDIStreaming alternate setting the
// words were read on (0 = legacy MIDI 1.0 byte stream, 1 = UMP / MIDI 2.0).
static void log_echoed(uint8_t alt, const uint32_t *words, uint16_t count)
{
    printf("tusb_ump_lb: echo alt=%u [", alt);
    for (uint16_t i = 0; i < count; i++)
    {
        printf("%s0x%08lx", i ? " " : "", (unsigned long) words[i]);
    }
    printf("]\n");
}

static void led_task(void)
{
    // Blink while unmounted, solid on once the host has the UMP interface open.
    static uint32_t last_toggle_ms = 0;
    uint32_t now = to_ms_since_boot(get_absolute_time());

    if (tud_ump_n_mounted(0))
    {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        return;
    }

    if (now - last_toggle_ms >= 250)
    {
        last_toggle_ms = now;
        gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
    }
}
