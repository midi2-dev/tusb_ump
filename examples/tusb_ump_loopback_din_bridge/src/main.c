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
 * tusb_ump_loopback_din_bridge - demonstrates two independent, simultaneous
 * USB MIDI 2.0 (UMP) interfaces on one device (CFG_TUD_UMP=2, see
 * tusb_config.h and usb_descriptors.c):
 *
 *   - tud_ump itf 0: raw UMP loopback (echoes whatever it receives), same
 *     behavior as ../tusb_ump_lb.
 *   - tud_ump itf 1: DIN MIDI 1.0 <-> UMP bridge over hardware UART, using
 *     the same midi1_bytestream.c/h converter and bridging approach as
 *     ../tusb_ump_uart_din_bridge, just pointed at itf 1 instead of itf 0 --
 *     proof the approach is reusable across interfaces, not just boards.
 *
 * See ../tusb_ump_uart_din_bridge/README.md for why the DIN pins default to
 * UART1 (GPIO4/5) rather than UART0 (this board's console UART).
 */

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "tusb.h"
#include "ump_device.h"

#include "midi1_bytestream.h"

#define LOOPBACK_ITF    0
#define DIN_ITF         1

#define DIN_UART      uart1
#define DIN_TX_PIN    4
#define DIN_RX_PIN    5
#define DIN_BAUD      31250
#define DIN_GROUP     0

static midi1_bs_to_ump_t din_to_ump;
static midi1_ump_to_bs_t ump_to_din;

static void din_uart_init(void)
{
    uart_init(DIN_UART, DIN_BAUD);
    gpio_set_function(DIN_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DIN_RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(DIN_RX_PIN); // DIN MIDI RX is open-drain/idle-high
    uart_set_format(DIN_UART, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(DIN_UART, false, false);
    uart_set_fifo_enabled(DIN_UART, true);
}

// Interface 0: raw UMP loopback, same behavior as tusb_ump_lb.
static void loopback_service(void)
{
    while (tud_ump_n_mounted(LOOPBACK_ITF) && tud_ump_n_available(LOOPBACK_ITF))
    {
        uint32_t words[4];
        uint16_t count = tud_ump_read_ntoh(LOOPBACK_ITF, words, 4);
        if (count == 0) break;
        tud_ump_write_hton(LOOPBACK_ITF, words, count);
    }
}

// Interface 1: DIN MIDI 1.0 <-> UMP bridge, same approach as
// tusb_ump_uart_din_bridge but driving tud_ump itf 1 instead of itf 0.
static void din_bridge_service(void)
{
    while (tud_ump_n_mounted(DIN_ITF) && tud_ump_n_available(DIN_ITF))
    {
        uint32_t words[4];
        uint16_t count = tud_ump_read_ntoh(DIN_ITF, words, 4);
        if (count == 0) break;

        for (uint16_t i = 0; i < count;)
        {
            uint8_t n = midi1_ump_word_count(words[i]);
            uint32_t pair[2] = { words[i], (n >= 2 && i + 1 < count) ? words[i + 1] : 0 };

            uint8_t bytes[8];
            uint8_t byte_count = midi1_ump_to_bs(&ump_to_din, pair, DIN_GROUP, bytes);
            for (uint8_t b = 0; b < byte_count; b++) uart_putc_raw(DIN_UART, bytes[b]);

            i += n;
        }
    }

    while (uart_is_readable(DIN_UART))
    {
        uint8_t ch = uart_getc(DIN_UART);
        if (ch == 0xFE) continue; // Skip Active Sensing

        uint32_t words[2];
        uint8_t word_count = 0;
        midi1_bs_to_ump_parse(&din_to_ump, ch, words, &word_count);
        if (word_count > 0) tud_ump_write_hton(DIN_ITF, words, word_count);
    }
}

int main(void)
{
    stdio_init_all();

    printf("tusb_ump_loopback_din_bridge: starting\n");

    midi1_bs_to_ump_init(&din_to_ump, DIN_GROUP);
    midi1_ump_to_bs_init(&ump_to_din);
    din_uart_init();

    tusb_init();

    while (true)
    {
        tud_task();
        loopback_service();
        din_bridge_service();
    }

    return 0;
}

// Invoked when the host mounts the device or switches MIDIStreaming
// alternate setting (0 = legacy MIDI 1.0 byte stream, 1 = UMP / MIDI 2.0)
void tud_ump_set_itf_cb(uint8_t itf, uint8_t alt)
{
    printf("tusb_ump_loopback_din_bridge: itf=%u alt=%u\n", itf, alt);
}
