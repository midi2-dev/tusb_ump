// ============================================================
// USB MIDI 2.0 Device — T-Display-S3  (Arduino IDE)
// ============================================================
//
// Setup:
//   1) Board: "ESP32S3 Dev Module" (or LilyGO T-Display-S3)
//   2) Tools > USB Mode        > "USB-OTG (TinyUSB)"
//   3) Tools > USB CDC On Boot > "Disabled"
//   4) Install LovyanGFX via Library Manager
//   5) Copy tusb_ump repo into ~/Arduino/libraries/tusb_ump/
//   6) Copy platform.local.txt.example as platform.local.txt to:
//        ~/.arduino15/packages/esp32/hardware/esp32/<version>/
//      Then restart Arduino IDE.
//
// Controls:
//   BTN1 (GPIO0)  — toggle NoteOn / NoteOff (C5)
//   BTN2 (GPIO14) — cycle test velocity: 32 → 64 → 96 → 127
//
// Copyright (c) 2026 Saulo Verissimo
// SPDX-License-Identifier: MIT
// ============================================================

#include <Arduino.h>
#include "USB.h"
#include "ump_device.h"
#include "ump_stream_handler.h"
#include "UMPDisplay.h"
#include "mapping.h"

// ── Device configuration ─────────────────────────────────────

static const UMPStreamConfig streamCfg = {
    .umpVersionMajor      = 1,
    .umpVersionMinor      = 1,
    .numFunctionBlocks    = 1,
    .staticFunctionBlocks = true,
    .protocolCaps         = UMP_PROTO_CAP_MIDI1 | UMP_PROTO_CAP_MIDI2,
    .manufacturerId       = { 0x00, 0x00, 0x7D },   // 0x7D = educational/dev
    .familyId             = 0x0001,
    .modelId              = 0x0001,
    .swRevision           = { '0', '1', '0', '0' },
    .endpointName         = "ESP32-S3 MIDI2",
    .productInstanceId    = "ESP32S3MIDI001",
    .fbName               = "Group 0",
    .fbDirection          = UMP_FB_DIR_BIDIRECTIONAL,
    .fbFirstGroup         = 0,
    .fbNumGroups          = 1,
    .fbUIHint             = 0x00,
    .fbMidi10             = 0x00,
    .fbMidiCIVer          = 0x00,
    .fbSysEx8             = 0x00,
};

// ── State ────────────────────────────────────────────────────

static UMPDisplay display;
static bool       usbWasConnected = false;
static uint8_t    currentAlt      = 0xFF;

static uint32_t btn1_last = 0;
static uint32_t btn2_last = 0;
static const uint32_t DEBOUNCE_MS = 80;

static bool    noteIsOn   = false;
static uint8_t testNote   = 72;
static uint8_t testVelIdx = 1;
static const uint8_t velocities[] = { 32, 64, 96, 127 };
static uint8_t testVel    = 64;

// ── tusb_ump callbacks ───────────────────────────────────────

extern "C" void tud_ump_set_itf_cb(uint8_t itf, uint8_t alt) {
    (void)itf;
    currentAlt = alt;
    display.setProtocol(alt);
}

extern "C" void tud_ump_rx_cb(uint8_t itf) {
    (void)itf;
}

// ── Stream handler TX callback ───────────────────────────────

static void onStreamTx(const uint32_t* words, uint8_t nw,
                       const char* /* label */)
{
    display.pushTxUMP(words, nw);
}

// ── UMP helpers ──────────────────────────────────────────────

static inline uint8_t umpWordsForMT(uint8_t mt) {
    switch (mt) {
    case 0x0: case 0x1: case 0x2: case 0x6: case 0x7: return 1;
    case 0x3: case 0x4: case 0x8: case 0x9: case 0xA: case 0xD: return 2;
    case 0xB: case 0xC: return 3;
    case 0x5: case 0xE: case 0xF: return 4;
    default: return 1;
    }
}

static inline uint32_t makeUMP_MIDI1CV(uint8_t group, uint8_t status,
                                       uint8_t ch, uint8_t d0, uint8_t d1)
{
    return (uint32_t)d1                               << 24 |
           (uint32_t)d0                               << 16 |
           (uint32_t)((status & 0xF0) | (ch & 0x0F))  << 8  |
           (uint32_t)(0x20 | (group & 0x0F));
}

// ── Send ─────────────────────────────────────────────────────

static void sendNoteOn(uint8_t group, uint8_t ch,
                       uint8_t note, uint8_t vel)
{
    if (!tud_ump_n_mounted(0)) return;
    uint32_t w = makeUMP_MIDI1CV(group, 0x90, ch, note, vel);
    tud_ump_write(0, &w, 1);
    display.pushTxUMP(&w, 1);
}

static void sendNoteOff(uint8_t group, uint8_t ch, uint8_t note)
{
    if (!tud_ump_n_mounted(0)) return;
    uint32_t w = makeUMP_MIDI1CV(group, 0x80, ch, note, 0);
    tud_ump_write(0, &w, 1);
    display.pushTxUMP(&w, 1);
}

static void sendCC(uint8_t group, uint8_t ch, uint8_t cc, uint8_t val)
{
    if (!tud_ump_n_mounted(0)) return;
    uint32_t w = makeUMP_MIDI1CV(group, 0xB0, ch, cc, val);
    tud_ump_write(0, &w, 1);
    display.pushTxUMP(&w, 1);
}

// ── RX ───────────────────────────────────────────────────────

static void processRxUMP()
{
    for (int pass = 0; pass < 16; pass++) {
        if (tud_ump_n_available(0) == 0) break;

        uint32_t firstWord = 0;
        if (tud_ump_read(0, &firstWord, 1) == 0) break;

        uint8_t  mt       = (uint8_t)((firstWord & 0xF0) >> 4);
        uint8_t  numWords = umpWordsForMT(mt);
        uint32_t words[4] = { firstWord, 0, 0, 0 };

        for (uint8_t w = 1; w < numWords; w++) {
            if (tud_ump_n_available(0) == 0) break;
            tud_ump_read(0, &words[w], 1);
        }

        display.pushRxUMP(words, numWords);
        display.pulseRx();

        if (mt == 0xF) {
            // MT=0xF Stream messages — handle with Discovery protocol
            umpStreamHandleRx(0, words, streamCfg, onStreamTx);
        } else {
            // All other message types — echo back (loopback for testing)
            if (tud_ump_n_writeable(0) >= numWords) {
                tud_ump_write(0, words, numWords);
                display.pushTxUMP(words, numWords);
            }
        }
    }
}

// ── Buttons ──────────────────────────────────────────────────

static void handleButtons()
{
    uint32_t now = millis();

    if (digitalRead(BTN1) == LOW && (now - btn1_last) > DEBOUNCE_MS) {
        btn1_last = now;
        if (!noteIsOn) { sendNoteOn(0, 0, testNote, testVel); noteIsOn = true; }
        else           { sendNoteOff(0, 0, testNote);          noteIsOn = false; }
    }

    if (digitalRead(BTN2) == LOW && (now - btn2_last) > DEBOUNCE_MS) {
        btn2_last = now;
        testVelIdx = (testVelIdx + 1) % (sizeof(velocities) / sizeof(velocities[0]));
        testVel = velocities[testVelIdx];
        sendCC(0, 0, 7, testVel);
    }
}

// ── Arduino ──────────────────────────────────────────────────

void setup()
{
    USB.begin();
    Serial.begin(115200);
    pinMode(BTN1, INPUT_PULLUP);
    pinMode(BTN2, INPUT_PULLUP);
    testVel = velocities[testVelIdx];

    display.init();
    display.setConnected(false);
    display.setProtocol(0);

    Serial.println("[MIDI2] USB MIDI 2.0 device started");
    Serial.printf("[MIDI2] EP name: %s\n", streamCfg.endpointName);
    Serial.printf("[MIDI2] Product: %s\n", streamCfg.productInstanceId);
}

void loop()
{
    bool mounted = tud_ump_n_mounted(0);

    if (mounted && !usbWasConnected) {
        usbWasConnected = true;
        display.setConnected(true);
    } else if (!mounted && usbWasConnected) {
        usbWasConnected = false;
        currentAlt = 0xFF;
        display.setConnected(false);
    }

    if (mounted) {
        uint8_t alt = tud_alt_setting(0);
        if (alt != currentAlt) {
            currentAlt = alt;
            display.setProtocol(alt);
        }
        processRxUMP();
    }

    handleButtons();
    delay(2);
}
