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
 */

#include "midi1_bytestream.h"
#include "tusb.h" // must precede ump.h: defines TU_ATTR_PACKED and friends that ump.h relies on
#include "ump.h"

//--------------------------------------------------------------------+
// Byte stream -> UMP
//--------------------------------------------------------------------+

static void bs_flush_sysex(midi1_bs_to_ump_t *ctx, bool final, uint32_t out[2], uint8_t *out_count) {
    uint8_t status = final
            ? (ctx->sysex_first_packet_sent ? UMP_SYSEX7_END : UMP_SYSEX7_COMPLETE)
            : (ctx->sysex_first_packet_sent ? UMP_SYSEX7_CONTINUE : UMP_SYSEX7_START);
    uint8_t n = ctx->sysex_have;
    uint8_t b0 = n > 0 ? ctx->sysex_buf[0] : 0;
    uint8_t b1 = n > 1 ? ctx->sysex_buf[1] : 0;
    uint8_t b2 = n > 2 ? ctx->sysex_buf[2] : 0;
    uint8_t b3 = n > 3 ? ctx->sysex_buf[3] : 0;
    uint8_t b4 = n > 4 ? ctx->sysex_buf[4] : 0;
    uint8_t b5 = n > 5 ? ctx->sysex_buf[5] : 0;

    out[0] = ((uint32_t) (UMP_MT_DATA_64 | ctx->group) << 24)
            | ((uint32_t) (status | n) << 16)
            | ((uint32_t) b0 << 8) | b1;
    out[1] = ((uint32_t) b2 << 24) | ((uint32_t) b3 << 16) | ((uint32_t) b4 << 8) | b5;
    *out_count = 2;

    ctx->sysex_have = 0;
    ctx->sysex_first_packet_sent = !final;
}

void midi1_bs_to_ump_init(midi1_bs_to_ump_t *ctx, uint8_t group) {
    ctx->group = group & UMP_GROUP_MASK;
    ctx->status = 0;
    ctx->have_data0 = false;
    ctx->in_sysex = false;
    ctx->sysex_have = 0;
    ctx->sysex_first_packet_sent = false;
}

// Number of data bytes (excluding the status byte itself) a channel-voice or
// system-common status byte expects; 0xFF means "not handled here" (System
// Real-Time 0xF8-0xFF is handled directly in the caller before reaching this).
static uint8_t data_bytes_needed(uint8_t status) {
    uint8_t hi = status & 0xF0;
    if (hi == 0x80 || hi == 0x90 || hi == 0xA0 || hi == 0xB0 || hi == 0xE0) return 2; // Note off/on, poly press, CC, pitch bend
    if (hi == 0xC0 || hi == 0xD0) return 1; // Program change, channel pressure
    if (status == UMP_SYSTEM_MTC || status == UMP_SYSTEM_SONG_SELECT) return 1;
    if (status == UMP_SYSTEM_SONG_POS_PTR) return 2;
    if (status == UMP_SYSTEM_TUNE_REQ) return 0;
    return 0xFF; // undefined (0xF4/0xF5) or otherwise unrecognized
}

static void emit_word(midi1_bs_to_ump_t *ctx, uint8_t mt, uint8_t status, uint8_t b1, uint8_t b2,
                       uint32_t out[2], uint8_t *out_count) {
    out[0] = ((uint32_t) (mt | ctx->group) << 24) | ((uint32_t) status << 16) | ((uint32_t) b1 << 8) | b2;
    *out_count = 1;
}

void midi1_bs_to_ump_parse(midi1_bs_to_ump_t *ctx, uint8_t byte, uint32_t out[2], uint8_t *out_count) {
    *out_count = 0;

    // System Real-Time: single status byte, can interleave mid-message, never
    // disturbs running status or sysex assembly.
    if (byte >= UMP_SYSTEM_TIMING_CLK) {
        emit_word(ctx, UMP_MT_SYSTEM, byte, 0, 0, out, out_count);
        return;
    }

    if (byte == MIDI_1_STATUS_SYSEX_START) {
        ctx->status = 0;
        ctx->have_data0 = false;
        ctx->in_sysex = true;
        ctx->sysex_have = 0;
        ctx->sysex_first_packet_sent = false;
        return;
    }

    if (byte == MIDI_1_STATUS_SYSEX_END) {
        if (ctx->in_sysex) {
            bs_flush_sysex(ctx, true, out, out_count);
            ctx->in_sysex = false;
        }
        ctx->status = 0;
        return;
    }

    if (ctx->in_sysex) {
        if (byte >= 0x80) {
            // Malformed stream (a status byte other than F7/realtime arrived
            // mid-sysex) -- close out what we have rather than losing it.
            bs_flush_sysex(ctx, true, out, out_count);
            ctx->in_sysex = false;
            // fall through to treat `byte` as a fresh status byte below
        } else {
            ctx->sysex_buf[ctx->sysex_have++] = byte;
            if (ctx->sysex_have == 6) {
                bs_flush_sysex(ctx, false, out, out_count);
            }
            return;
        }
    }

    if (byte >= 0x80) { // new status byte
        uint8_t need = data_bytes_needed(byte);
        ctx->status = (need == 0xFF) ? 0 : byte;
        ctx->have_data0 = false;

        if (need == 0) { // e.g. Tune Request: no data bytes, emit now
            emit_word(ctx, UMP_MT_SYSTEM, byte, 0, 0, out, out_count);
            ctx->status = 0; // system common messages don't support running status
        }
        return;
    }

    // Data byte, using running status (ctx->status) if one is active.
    if (ctx->status == 0) return; // no context -- ignore stray data byte

    uint8_t need = data_bytes_needed(ctx->status);
    // By construction ctx->status is only ever a recognized channel-voice
    // status (< 0xF0) or system-common status (0xF1-0xF3) here -- see the
    // status-byte branch above, which clears ctx->status for anything else.
    uint8_t mt = ctx->status < 0xF0 ? UMP_MT_MIDI1_CV : UMP_MT_SYSTEM;

    if (need == 1) {
        emit_word(ctx, mt, ctx->status, byte, 0, out, out_count);
        if (mt == UMP_MT_SYSTEM) ctx->status = 0; // system common cancels running status
        return;
    }

    // need == 2
    if (!ctx->have_data0) {
        ctx->data0 = byte;
        ctx->have_data0 = true;
        return;
    }

    emit_word(ctx, mt, ctx->status, ctx->data0, byte, out, out_count);
    ctx->have_data0 = false;
    if (mt == UMP_MT_SYSTEM) ctx->status = 0;
}

//--------------------------------------------------------------------+
// UMP -> byte stream
//--------------------------------------------------------------------+

void midi1_ump_to_bs_init(midi1_ump_to_bs_t *ctx) {
    ctx->sysex_open = false;
}

uint8_t midi1_ump_word_count(uint32_t word0) {
    switch ((word0 >> 24) & UMP_MT_MASK) {
        case UMP_MT_UTILITY:
        case UMP_MT_SYSTEM:
        case UMP_MT_MIDI1_CV:
            return 1;
        case UMP_MT_DATA_64:
        case UMP_MT_MIDI2_CV:
            return 2;
        case UMP_MT_DATA_128:
        case UMP_MT_FLEX_128:
        case UMP_MT_STREAM_128:
            return 4;
        default:
            return 1;
    }
}

uint8_t midi1_ump_to_bs(midi1_ump_to_bs_t *ctx, const uint32_t words[2], uint8_t group, uint8_t out[8]) {
    uint32_t w0 = words[0];
    uint8_t type_group = (w0 >> 24) & 0xFF;
    uint8_t mt = type_group & UMP_MT_MASK;
    uint8_t grp = type_group & UMP_GROUP_MASK;
    if (grp != group) return 0;

    uint8_t n = 0;

    if (mt == UMP_MT_MIDI1_CV) {
        uint8_t status = (w0 >> 16) & 0xFF;
        uint8_t b1 = (w0 >> 8) & 0xFF;
        uint8_t b2 = w0 & 0xFF;
        uint8_t hi = status & 0xF0;
        out[n++] = status;
        out[n++] = b1;
        if (hi != 0xC0 && hi != 0xD0) out[n++] = b2; // 2-data-byte messages only
        return n;
    }

    if (mt == UMP_MT_SYSTEM) {
        uint8_t status = (w0 >> 16) & 0xFF;
        uint8_t b1 = (w0 >> 8) & 0xFF;
        uint8_t b2 = w0 & 0xFF;
        out[n++] = status;
        if (status == UMP_SYSTEM_MTC || status == UMP_SYSTEM_SONG_SELECT) {
            out[n++] = b1;
        } else if (status == UMP_SYSTEM_SONG_POS_PTR) {
            out[n++] = b1;
            out[n++] = b2;
        }
        // Real-Time / Tune Request: status byte only.
        return n;
    }

    if (mt == UMP_MT_DATA_64) {
        uint8_t status_numbytes = (w0 >> 16) & 0xFF;
        uint8_t status = status_numbytes & UMP_SYSEX7_STATUS_MASK;
        uint8_t num_bytes = status_numbytes & UMP_SYSEX7_SIZE_MASK;
        uint8_t data[6];
        data[0] = (w0 >> 8) & 0xFF;
        data[1] = w0 & 0xFF;
        uint32_t w1 = words[1];
        data[2] = (w1 >> 24) & 0xFF;
        data[3] = (w1 >> 16) & 0xFF;
        data[4] = (w1 >> 8) & 0xFF;
        data[5] = w1 & 0xFF;

        if (status == UMP_SYSEX7_COMPLETE || status == UMP_SYSEX7_START) {
            out[n++] = MIDI_1_STATUS_SYSEX_START;
            ctx->sysex_open = true;
        }
        for (uint8_t i = 0; i < num_bytes && i < 6; i++) out[n++] = data[i];
        if (status == UMP_SYSEX7_COMPLETE || status == UMP_SYSEX7_END) {
            out[n++] = MIDI_1_STATUS_SYSEX_END;
            ctx->sysex_open = false;
        }
        return n;
    }

    return 0;
}
