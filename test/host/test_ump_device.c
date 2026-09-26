// Host-side regression tests for tud_ump_read_impl()'s alt-0 (legacy
// USB-MIDI1 byte stream) read path in ump_device.cpp, covering:
//   1. No buffer overflow: a raw word can convert to up to 2 UMP words
//      (a SysEx7 completion), so output must always be bounded by the
//      caller's requested count, not by raw input words consumed.
//   2. No unnecessary 1-word stalling: reserving room for a possible
//      2-word SysEx completion must not short-change a request when the
//      next message is actually a 1-word Channel Voice/System Common.
//   3. Byte continuity across multiple reads mid-SysEx
//   4. A single-byte System Common/real-time message (e.g. Timing Clock)
//      converts to exactly 1 UMP word, not 2 with a spurious zero word.
// and the alt-0 WRITE path's MIDI 2.0 -> MIDI 1.0 translation
// (CFG_UMP_MIDI2_TO_MIDI1, ump.h):
//   5. every MIDI 2.0 Channel Voice opcode becomes the right USB-MIDI 1.0
//      packet(s); per-note messages are consumed and dropped
//   6. a multi-packet translation (an RPN is four Control Changes) is written
//      all or none -- with no room it is left for the caller to offer again
//
// No test framework -- plain assert(), built as a small host executable
// (see Makefile). Uses UMP_DEVICE_UNIT_TEST-guarded hooks in ump_device.cpp
// to inject raw USB-MIDI1 bytes without a real USB enumeration.

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "tusb.h"
#include "ump_device.h"

#define ITF 0

// USB-MIDI1 CIN nibbles (see ump_device.cpp's MIDI_1_CIN_* enum via ump.h).
#define CIN_SYSEX_START   0x4
#define CIN_SYSEX_END_3B  0x7
#define CIN_NOTE_ON       0x9
#define CIN_1BYTE_DATA    0xF

#define MIDI1_TIMING_CLOCK 0xF8

static void push_word(uint8_t cin, uint8_t cable, uint8_t b1, uint8_t b2, uint8_t b3)
{
    uint8_t pkt[4] = { (uint8_t)((cable << 4) | cin), b1, b2, b3 };
    uint16_t written = tud_ump_test_rx_write(ITF, pkt, sizeof(pkt));
    assert(written == sizeof(pkt));
}

static void reset_interface(void)
{
    umpd_init();
    tud_ump_test_set_ep_out(ITF, 0x01); // any nonzero value marks the itf "open"
}

// A raw USB-MIDI1 word carrying a SysEx7 CIN (start/end variants) converts
// to 2 UMP words (a 64-bit SYSEX7 packet); mirrors ump_device.cpp's own
// MIDI_1_CIN_SYSEX_* handling. Encodes "F0 7E 7F 06 01 F7" (a 6-byte
// Universal Device Inquiry -- a real-world message shape, not a contrived
// one) as two USB-MIDI1 packets: CIN 4 (start, carrying F0 + 2 data bytes)
// then CIN 7 (end-3-byte, carrying 2 data bytes + the F7 terminator).
static void push_sysex_message(void)
{
    push_word(CIN_SYSEX_START, 0, 0xF0, 0x7E, 0x7F);
    push_word(CIN_SYSEX_END_3B, 0, 0x06, 0x01, 0xF7);
}

static void test_no_overflow_on_dense_sysex(void)
{
    reset_interface();

    // Queue several complete SysEx7 messages back-to-back. Each message is
    // 2 raw USB-MIDI1 packets (start + end), and each of those individually
    // converts to a 2-word 64-bit UMP packet -- so with numAvail=4 a buggy
    // loop bounding on raw words consumed (instead of UMP words written)
    // could write up to ~2x that in a single call.
    const int num_messages = 8;
    for (int i = 0; i < num_messages; i++)
    {
        push_sysex_message();
    }

    uint32_t canary = 0xDEADBEEFu;
    struct { uint32_t words[4]; uint32_t canary; } guarded;
    guarded.canary = canary;

    uint16_t total = 0;
    for (int call = 0; call < 20; call++)
    {
        uint16_t n = tud_ump_read_ntoh(ITF, guarded.words, 4);
        assert(n <= 4); // never more than requested
        assert(guarded.canary == canary); // never written past the buffer
        total += n;
        if (n == 0) break;
    }
    // Each message = 2 raw packets (start + end), each producing a 2-word
    // UMP packet: 4 UMP words recovered per message.
    assert(total == num_messages * 4);

    printf("PASS: test_no_overflow_on_dense_sysex (%u words recovered)\n", total);
}

static void test_no_unnecessary_stall_on_cv_messages(void)
{
    reset_interface();

    // 4 Channel Voice (Note On) messages queued; each converts to exactly
    // 1 UMP word. A read for numAvail=3 should return all 3 fittable words
    // in one call, not stop 1 short reserving room for a SysEx that isn't
    // coming.
    for (int i = 0; i < 4; i++)
    {
        push_word(CIN_NOTE_ON, 0, 0x90, (uint8_t)(60 + i), 100);
    }

    uint32_t buf[3];
    uint16_t n = tud_ump_read_ntoh(ITF, buf, 3);
    assert(n == 3);

    printf("PASS: test_no_unnecessary_stall_on_cv_messages (n=%u)\n", n);
}

static void test_split_call_continuity(void)
{
    reset_interface();

    for (int i = 0; i < 6; i++)
    {
        push_word(CIN_NOTE_ON, 0, 0x90, (uint8_t)(20 + i), 100);
    }

    // Drain via several small reads instead of one large one, and compare
    // against draining the same input in one big read.
    uint32_t small_run[6] = {0};
    uint16_t total = 0;
    while (total < 6)
    {
        uint32_t chunk[2];
        uint16_t n = tud_ump_read_ntoh(ITF, chunk, 2);
        assert(n > 0);
        memcpy(&small_run[total], chunk, n * sizeof(uint32_t));
        total += n;
    }

    reset_interface();
    for (int i = 0; i < 6; i++)
    {
        push_word(CIN_NOTE_ON, 0, 0x90, (uint8_t)(20 + i), 100);
    }
    uint32_t big_run[6];
    uint16_t n = tud_ump_read_ntoh(ITF, big_run, 6);
    assert(n == 6);

    assert(memcmp(small_run, big_run, sizeof(big_run)) == 0);

    printf("PASS: test_split_call_continuity\n");
}

static void test_system_realtime_single_word(void)
{
    reset_interface();

    // A single-byte System Common/real-time status (CIN 0xF, data byte's
    // high bit set) is reassigned internally to CIN 0x5, which is shared
    // with the genuine 2-word SysEx-end-1-byte completion -- it must still
    // convert to exactly 1 UMP word (MT=1 System), not 2 with a spurious
    // trailing zero word.
    push_word(CIN_1BYTE_DATA, 0, MIDI1_TIMING_CLOCK, 0x00, 0x00);
    push_word(CIN_NOTE_ON, 0, 0x90, 60, 100);

    uint32_t buf[2];
    uint16_t n = tud_ump_read_ntoh(ITF, buf, 2);
    assert(n == 2); // 1 word each, not 2 (clock + spurious zero) leaving the
                     // Note On stranded in the FIFO for another call

    uint32_t expected_clock_word = ((uint32_t)UMP_MT_SYSTEM << 24) | ((uint32_t)MIDI1_TIMING_CLOCK << 16);
    uint32_t expected_note_on_word = ((uint32_t)UMP_MT_MIDI1_CV << 24) | (0x90u << 16) | (60u << 8) | 100u;
    assert(buf[0] == expected_clock_word);
    assert(buf[1] == expected_note_on_word); // fails if a spurious 2nd word
                                              // pushed the real Note On out

    printf("PASS: test_system_realtime_single_word (n=%u)\n", n);
}


// ---- alt-0 write path: MIDI 2.0 Channel Voice -> USB-MIDI 1.0 -----------
extern uint8_t  g_test_in_cap[1024];
extern uint16_t g_test_in_len;
extern bool     g_test_edpt_free;

#if CFG_UMP_MIDI2_TO_MIDI1
static void reset_for_write(void)
{
    reset_interface();
    tud_ump_test_set_ep_in(ITF, 0x81);
    g_test_in_len = 0;
    g_test_edpt_free = true;
}

// Write one MIDI 2.0 CV message (host-order words) and compare what goes out
// with `want` (n packets of 4 bytes).
static void expect_m2(uint32_t w0, uint32_t w1, const uint8_t *want, uint16_t n)
{
    reset_for_write();
    uint32_t w[2] = { w0, w1 };
    uint16_t done = tud_ump_write_hton(ITF, w, 2);
    assert(done == 2);                              // always consumed
    if (g_test_in_len != n * 4 || memcmp(g_test_in_cap, want, n * 4) != 0) {
        printf("  MT4 %08X %08X: got %u bytes:", (unsigned)w0, (unsigned)w1, g_test_in_len);
        for (uint16_t i = 0; i < g_test_in_len; i++) printf(" %02X", g_test_in_cap[i]);
        printf("\n");
        assert(0);
    }
}
#endif

static void test_midi2_cv_translation(void)
{
#if CFG_UMP_MIDI2_TO_MIDI1
    // Note On ch 3, note 60, velocity 0xC800 -> 0x64 (100); group 0 -> cable 0.
    { const uint8_t e[] = {0x09, 0x93, 60, 100}; expect_m2(0x40933C00, 0xC8000000, e, 1); }
    // Note On with a velocity that scales to 0 is sent as velocity 1, not as Note Off.
    { const uint8_t e[] = {0x09, 0x90, 60, 1};   expect_m2(0x40903C00, 0x01000000, e, 1); }
    // Note Off, group 2 -> cable 2.
    { const uint8_t e[] = {0x28, 0x80, 64, 64};  expect_m2(0x42804000, 0x80000000, e, 1); }
    // Poly Pressure / Control Change / Channel Pressure: top 7 bits of 32.
    { const uint8_t e[] = {0x0A, 0xA0, 60, 70};  expect_m2(0x40A03C00, 70u << 25, e, 1); }
    { const uint8_t e[] = {0x0B, 0xB5, 7, 90};   expect_m2(0x40B50700, 90u << 25, e, 1); }
    { const uint8_t e[] = {0x0D, 0xD0, 50, 0};   expect_m2(0x40D00000, 50u << 25, e, 1); }
    // Pitch Bend: 32 -> 14 bits (0x2EE0 -> LSB 0x60, MSB 0x5D).
    { const uint8_t e[] = {0x0E, 0xE0, 0x60, 0x5D}; expect_m2(0x40E00000, 0x2EE0u << 18, e, 1); }
    // Program Change without, then with, Bank Valid.
    { const uint8_t e[] = {0x0C, 0xC4, 5, 0};    expect_m2(0x40C40000, 5u << 24, e, 1); }
    { const uint8_t e[] = {0x0B, 0xB4, 0, 3,  0x0B, 0xB4, 32, 9,  0x0C, 0xC4, 5, 0};
      expect_m2(0x40C40001, (5u << 24) | (3u << 8) | 9u, e, 3); }
    // RPN 0/0 (pitch bend sensitivity) = 14-bit 0x0180, and NRPN 1/2.
    { const uint8_t e[] = {0x0B, 0xB1, 101, 0,  0x0B, 0xB1, 100, 0,  0x0B, 0xB1, 6, 0x03,  0x0B, 0xB1, 38, 0x00};
      expect_m2(0x40210000, 0x0180u << 18, e, 4); }
    { const uint8_t e[] = {0x0B, 0xB1, 99, 1,  0x0B, 0xB1, 98, 2,  0x0B, 0xB1, 6, 0x40,  0x0B, 0xB1, 38, 0x00};
      expect_m2(0x40310102, 0x2000u << 18, e, 4); }
    // Per-note pitch bend (opcode 6) and per-note management (F): no MIDI 1.0
    // form -- consumed, nothing sent.
    expect_m2(0x40603C00, 0x80000000, NULL, 0);
    expect_m2(0x40F03C00, 0x00000000, NULL, 0);
    printf("  MIDI 2.0 -> MIDI 1.0 translation: ok\n");
#else
    printf("  MIDI 2.0 -> MIDI 1.0 translation: disabled (CFG_UMP_MIDI2_TO_MIDI1 0)\n");
#endif
}

static void test_midi2_multi_packet_all_or_none(void)
{
#if CFG_UMP_MIDI2_TO_MIDI1
    // Endpoint busy, so output accumulates in the 64-byte TX FIFO. Fill it
    // to 52 bytes with 13 MIDI 1.0 notes, leaving 12: an RPN needs 16.
    reset_for_write();
    g_test_edpt_free = false;
    uint32_t notes[13];
    for (int i = 0; i < 13; i++) notes[i] = 0x20903C40u;   // MT2 Note On
    assert(tud_ump_write_hton(ITF, notes, 13) == 13);
    uint32_t rpn[3] = { 0x40210000u, 0x0180u << 18, 0x20903C40u };
    uint16_t done = tud_ump_write_hton(ITF, rpn, 3);
    assert(done == 0);                     // RPN not consumed, nor anything after it
    assert(tud_ump_n_writeable(ITF) == 3); // 12 bytes left: nothing partial went in
    printf("  multi-packet translation written all or none: ok\n");
#endif
}

int main(void)
{
    test_no_overflow_on_dense_sysex();
    test_no_unnecessary_stall_on_cv_messages();
    test_split_call_continuity();
    test_system_realtime_single_word();
    test_midi2_cv_translation();
    test_midi2_multi_packet_all_or_none();
    printf("All tests passed.\n");
    return 0;
}
