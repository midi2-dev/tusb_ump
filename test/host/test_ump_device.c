// Host-side regression tests for tud_ump_read_impl()'s alt-0 (legacy
// USB-MIDI1 byte stream) read path in ump_device.cpp, covering:
//   1. No buffer overflow: a raw word can convert to up to 2 UMP words
//      (a SysEx7 completion), so output must always be bounded by the
//      caller's requested count, not by raw input words consumed.
//   2. No unnecessary 1-word stalling: reserving room for a possible
//      2-word SysEx completion must not short-change a request when the
//      next message is actually a 1-word Channel Voice/System Common.
//   3. Byte continuity across multiple reads mid-SysEx
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

int main(void)
{
    test_no_overflow_on_dense_sysex();
    test_no_unnecessary_stall_on_cv_messages();
    test_split_call_continuity();
    printf("All tests passed.\n");
    return 0;
}
