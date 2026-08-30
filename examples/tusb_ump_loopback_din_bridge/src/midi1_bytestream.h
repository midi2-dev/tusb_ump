/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2026 Michael Loh (AmeNote.com)
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
 * Minimal, dependency-free MIDI 1.0 byte-stream <-> UMP (MIDI1 Channel
 * Voice / System / Sysex7) converter. No external library dependency --
 * consistent with tusb_ump's own "no dependence on external libraries"
 * design (see ump_device.h's version history) -- and deliberately scoped to
 * plain MIDI 1.0 UMP words (message types 0x1/0x2/0x3), since that's all a
 * physical DIN MIDI 1.0 port can carry. No MIDI2 upscaling.
 */

#ifndef _MIDI1_BYTESTREAM_H_
#define _MIDI1_BYTESTREAM_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------+
// Byte stream -> UMP
//--------------------------------------------------------------------+

typedef struct {
    uint8_t group;             // UMP group (0-based) tagged on every word produced
    uint8_t status;             // current running-status byte, 0 = none
    uint8_t data0;               // first data byte of a 2-data-byte message, once received
    bool have_data0;
    bool in_sysex;
    uint8_t sysex_buf[6];
    uint8_t sysex_have;
    bool sysex_first_packet_sent;
} midi1_bs_to_ump_t;

void midi1_bs_to_ump_init(midi1_bs_to_ump_t *ctx, uint8_t group);

// Feed one incoming DIN byte. Any UMP word(s) produced (0, 1, or 2 -- 2 only
// for a Sysex7 packet) are written to out[] (must have room for 2), with
// *out_count set accordingly. Most calls produce 0 words (still assembling
// a message or a running-status data byte).
void midi1_bs_to_ump_parse(midi1_bs_to_ump_t *ctx, uint8_t byte, uint32_t out[2], uint8_t *out_count);

//--------------------------------------------------------------------+
// UMP -> byte stream
//--------------------------------------------------------------------+

typedef struct {
    bool sysex_open; // true if a 0xF0 has been emitted but no matching 0xF7 yet
} midi1_ump_to_bs_t;

void midi1_ump_to_bs_init(midi1_ump_to_bs_t *ctx);

// Returns how many consecutive 32-bit UMP words the message starting with
// word0 occupies (1, 2, or 4) based on its message type. Use this to know
// how many words to pass to midi1_ump_to_bs() and how far to advance when
// walking a buffer of UMP words (e.g. from tud_ump_read_ntoh()).
uint8_t midi1_ump_word_count(uint32_t word0);

// Converts one UMP message (words[0], plus words[1] for a 2-word Sysex7
// message -- see midi1_ump_word_count()) into 0-8 MIDI 1.0 bytes written to
// out[] (must have room for 8; SysEx framing bytes 0xF0/0xF7 are included).
// Returns the number of bytes written, or 0 if the message's group doesn't
// match `group` or its type isn't MIDI-1.0-representable (e.g. MIDI2
// Channel Voice, UMP Stream/Discovery).
uint8_t midi1_ump_to_bs(midi1_ump_to_bs_t *ctx, const uint32_t words[2], uint8_t group, uint8_t out[8]);

#ifdef __cplusplus
}
#endif

#endif /* _MIDI1_BYTESTREAM_H_ */
