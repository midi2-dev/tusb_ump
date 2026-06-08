// UMPDisplay.h — Visual UMP monitor (T-PicoC3 ST7789V 240×135 / Serial stub)
//
// Design mirrors T-Display-S3: header + protocol tabs + hex view +
// velocity bar + event log + status bar.
// Protocol tabs light up orange (MIDI 1.0) or green (MIDI 2.0/UMP).
//
// IMPORTANT: memory_width=240, memory_height=320 must be the full ST7789V RAM
// dimensions, NOT the panel size — required for correct rotation offset math.
//
// Copyright (c) 2026 Saulo Verissimo
// SPDX-License-Identifier: MIT

#ifndef UMP_DISPLAY_H
#define UMP_DISPLAY_H

#include <Arduino.h>
#include "mapping.h"
#include "ump_stream_handler.h"

// ═══════════════════════════════════════════════════════════════
// DISPLAY IMPLEMENTATION — T-PicoC3 (ST7789V 240×135, SPI)
// ═══════════════════════════════════════════════════════════════
#ifdef HAS_DISPLAY

#include <LovyanGFX.h>
#include <cstring>
#include <cstdio>

class UMPDisplay {
public:

    void init() {
        pinMode(PIN_POWER_ON, OUTPUT);
        digitalWrite(PIN_POWER_ON, HIGH);

        _tft.init();
        _tft.setRotation(1);
        _tft.setBrightness(255);
        _tft.setTextDatum(lgfx::top_left);
        _tft.fillScreen(C_BG);

        _proto = 0; _conn = false;
        _rx = 0; _tx = 0; _vel = 0; _nw = 0;
        memset(_w, 0, sizeof(_w));
        memset(_log, 0, sizeof(_log));
        _logN = 0;

        _drawAll();
    }

    void setConnected(bool c) { _conn = c; _drawHeader(); }

    void setProtocol(uint8_t alt) {
        _proto = alt;
        _drawProto();
        _drawStatus();
    }

    void pushRxUMP(const uint32_t* words, uint8_t nw) {
        _rx++;
        _storeWords(words, nw);

        uint8_t b0 = _w[0] & 0xFF;
        uint8_t b1 = (_w[0] >> 8)  & 0xFF;
        uint8_t b2 = (_w[0] >> 16) & 0xFF;
        uint8_t b3 = (_w[0] >> 24) & 0xFF;
        uint8_t mt = (b0 >> 4) & 0xF;

        if (mt == 0x2 || mt == 0x4) {
            uint8_t st = b1 & 0xF0;
            if (st == 0x90 && b3 > 0) {
                _vel = (mt == 0x4 && nw >= 2)
                     ? (uint8_t)((_w[1] >> 17) & 0x7F)
                     : b3;
            } else if (st == 0x80 || (st == 0x90 && b3 == 0)) {
                _vel = 0;
            }
        }

        _drawHex();
        _drawVel();
        _addLog(b0, b1, b2, b3, mt, false);
        _drawStatus();
    }

    void pushTxUMP(const uint32_t* words, uint8_t nw) {
        _tx++;
        uint8_t b0 = words[0] & 0xFF;
        uint8_t b1 = (words[0] >> 8)  & 0xFF;
        uint8_t b2 = (words[0] >> 16) & 0xFF;
        uint8_t b3 = (words[0] >> 24) & 0xFF;
        uint8_t mt = (b0 >> 4) & 0xF;
        _addLog(b0, b1, b2, b3, mt, true);
        _drawStatus();
    }

    void pulseRx() {
        static bool s = false; s = !s;
        _tft.fillCircle(SCR_W - MARGIN - 4, Y_HEADER + H_HEADER / 2,
                        4, s ? C_CYAN : C_HEADER);
    }

private:
    // ── LGFX driver — T-PicoC3 (ST7789V, SPI) ───────────────
    // NOTE: memory_width/height = full ST7789V RAM (240×320), NOT panel size.
    // LovyanGFX uses these to compute rotation offsets: mh-ph-oy, etc.
    // With memory=panel (135×240), rotation=1 gives offset=240-240-40=-40 (wrong).
    // With memory=full (240×320), rotation=1 gives offset=320-240-40=40 (correct).
    class LGFX : public lgfx::LGFX_Device {
    public:
        LGFX() {
            { auto c = _bus.config();
              c.spi_host    = 0;
              c.pin_sclk    = 2;
              c.pin_mosi    = 3;
              c.pin_miso    = -1;
              c.pin_dc      = 1;
              c.freq_write  = 40000000;
              _bus.config(c); _panel.setBus(&_bus); }

            { auto c = _panel.config();
              c.pin_cs      = 5;
              c.pin_rst     = 0;
              c.pin_busy    = -1;
              c.memory_width  = 240;  // full ST7789V RAM width  (NOT panel size)
              c.memory_height = 320;  // full ST7789V RAM height (NOT panel size)
              c.panel_width   = 135;  // actual visible panel
              c.panel_height  = 240;
              c.offset_x      = 52;
              c.offset_y      = 40;
              c.offset_rotation = 0;
              c.readable      = false;
              c.invert        = true;
              c.rgb_order     = false;
              c.dlen_16bit    = false;
              c.bus_shared    = false;
              _panel.config(c); }

            setPanel(&_panel);

            { auto c = _bl.config();
              c.pin_bl      = 4;
              c.invert      = false;
              c.freq        = 44100;
              c.pwm_channel = 0;
              _bl.config(c); _panel.setLight(&_bl); }
        }
    private:
        lgfx::Bus_SPI       _bus;
        lgfx::Panel_ST7789  _panel;
        lgfx::Light_PWM     _bl;
    };

    // ── State ───────────────────────────────────────────────
    LGFX     _tft;
    uint8_t  _proto = 0;
    bool     _conn  = false;
    uint32_t _rx = 0, _tx = 0;
    uint8_t  _vel = 0, _nw = 0;
    uint32_t _w[4] = {};

    struct LogLine { char t[48]; uint32_t c; };
    LogLine  _log[N_LOG];
    int      _logN = 0;

    // ── Helpers ─────────────────────────────────────────────

    void _storeWords(const uint32_t* words, uint8_t nw) {
        _w[0] = words[0];
        _w[1] = (nw >= 2) ? words[1] : 0;
        _w[2] = (nw >= 3) ? words[2] : 0;
        _w[3] = (nw >= 4) ? words[3] : 0;
        _nw = nw;
    }

    void _fill(int y, int h, uint32_t col) {
        _tft.fillRect(0, y, SCR_W, h, col);
    }

    void _hline(int y) {
        _tft.drawFastHLine(0, y, SCR_W, C_DIV);
    }

    void _txt(int x, int y, uint32_t fg, uint32_t bg,
              const lgfx::IFont* f, const char* s) {
        _tft.setFont(f);
        _tft.setTextColor(fg, bg);
        _tft.drawString(s, x, y);
    }

    static void _noteName(uint8_t n, char* buf, size_t sz) {
        static const char* const names[] = {
            "C","C#","D","D#","E","F","F#","G","G#","A","A#","B"
        };
        snprintf(buf, sz, "%s%d", names[n % 12], (int)(n / 12) - 1);
    }

    // ── Full redraw ─────────────────────────────────────────

    void _drawAll() {
        _drawHeader();
        _drawProto();
        _drawHex();
        _drawVel();
        _drawLog();
        _drawStatus();
    }

    // ── HEADER ──────────────────────────────────────────────
    // │  USB MIDI  T-PicoC3 - RP2040                      ●  │
    void _drawHeader() {
        _fill(Y_HEADER, H_HEADER, C_HEADER);
        _txt(MARGIN + 2, Y_HEADER + 2, C_WHITE, C_HEADER,
             &lgfx::fonts::Font2, "USB MIDI");
        _txt(MARGIN + 2 + 88, Y_HEADER + 2, C_CYAN, C_HEADER,
             &lgfx::fonts::Font2, "T-PicoC3 - RP2040");
        uint32_t dot = _conn ? C_GREEN : C_ORANGE;
        _tft.fillCircle(SCR_W - MARGIN - 4, Y_HEADER + H_HEADER / 2, 4, dot);
    }

    // ── PROTO: color tabs ────────────────────────────────────
    // │  MIDI 1.0 (orange=active)  │  MIDI 2.0 / UMP (green=active)  │
    void _drawProto() {
        int half = SCR_W / 2;
        uint32_t l_bg = (_proto == 0) ? C_ORANGE : C_DIV;
        uint32_t l_fg = (_proto == 0) ? C_BG     : C_GRAY;
        uint32_t r_bg = (_proto == 1) ? C_GREEN  : C_DIV;
        uint32_t r_fg = (_proto == 1) ? C_BG     : C_GRAY;

        _tft.fillRect(0,    Y_PROTO, half, H_PROTO, l_bg);
        _tft.fillRect(half, Y_PROTO, half, H_PROTO, r_bg);

        _txt(MARGIN + 4,        Y_PROTO + 1, l_fg, l_bg,
             &lgfx::fonts::Font2, "MIDI 1.0");
        _txt(half + MARGIN + 4, Y_PROTO + 1, r_fg, r_bg,
             &lgfx::fonts::Font2, "MIDI 2.0 / UMP");

        _hline(Y_PROTO + H_PROTO);
    }

    // ── HEX VIEW ─────────────────────────────────────────────
    // Line 1: color-coded hex bytes (Font2)
    // Line 2: decoded description (Font2)
    void _drawHex() {
        _fill(Y_HEX, H_HEX, C_BG);
        _hline(Y_HEX + H_HEX);

        if (_nw == 0) {
            _txt(MARGIN, Y_HEX + 7, C_DIV, C_BG,
                 &lgfx::fonts::Font2, "waiting for data...");
            return;
        }

        uint8_t b0  = _w[0] & 0xFF;
        uint8_t b1  = (_w[0] >> 8)  & 0xFF;
        uint8_t b2  = (_w[0] >> 16) & 0xFF;
        uint8_t b3  = (_w[0] >> 24) & 0xFF;
        uint8_t mt  = (b0 >> 4) & 0xF;
        uint8_t grp = b0 & 0xF;
        uint8_t st  = b1 & 0xF0;
        uint8_t ch  = b1 & 0x0F;

        // Line 1: color-coded hex bytes
        _tft.setFont(&lgfx::fonts::Font2);
        char hex[10]; int cx = MARGIN;

        _tft.setTextColor(C_CYAN, C_BG);
        snprintf(hex, sizeof(hex), "%02X ", b0);
        _tft.drawString(hex, cx, Y_HEX + 2); cx += 24;

        _tft.setTextColor(C_ORANGE, C_BG);
        snprintf(hex, sizeof(hex), "%02X ", b1);
        _tft.drawString(hex, cx, Y_HEX + 2); cx += 24;

        _tft.setTextColor(C_DIV, C_BG);
        snprintf(hex, sizeof(hex), "%02X %02X", b2, b3);
        _tft.drawString(hex, cx, Y_HEX + 2);

        if (_nw >= 2) {
            _tft.setTextColor(C_DIV, C_BG);
            char w2[12];
            snprintf(w2, sizeof(w2), "  %08lX", (unsigned long)_w[1]);
            _tft.drawString(w2, cx + 48, Y_HEX + 2);
        }

        // Line 2: decoded description
        char desc[48]; uint32_t col = C_DIV;
        const char* tag = (mt == 0x4) ? "M2" : "M1";

        if (mt == 0x2 || mt == 0x4) {
            if (st == 0x90 && b3 > 0) {
                char nn[6]; _noteName(b2, nn, sizeof(nn));
                snprintf(desc, sizeof(desc), "%s NoteOn  G%u Ch%u  %s V=%u",
                         tag, grp, ch, nn, b3);
                col = C_GREEN;
            }
            else if (st == 0x80 || (st == 0x90 && b3 == 0)) {
                char nn[6]; _noteName(b2, nn, sizeof(nn));
                snprintf(desc, sizeof(desc), "%s NoteOff G%u Ch%u  %s",
                         tag, grp, ch, nn);
                col = C_GRAY;
            }
            else if (st == 0xB0) {
                snprintf(desc, sizeof(desc), "%s CC G%u Ch%u  CC=%u V=%u",
                         tag, grp, ch, b2, b3);
                col = C_CYAN;
            }
            else if (st == 0xE0) {
                snprintf(desc, sizeof(desc), "%s PBend G%u Ch%u  %02X%02X",
                         tag, grp, ch, b2, b3);
                col = C_MAGENTA;
            }
            else {
                snprintf(desc, sizeof(desc), "%s Msg G%u Ch%u  %02X %02X",
                         tag, grp, ch, b2, b3);
            }
        }
        else if (mt == 0x3 || mt == 0x5) {
            snprintf(desc, sizeof(desc), "SysEx G%u [%u words]", grp, _nw);
            col = C_YELLOW;
        }
        else if (mt == 0xF) {
            uint16_t sts = streamStatus(_w[0]);
            snprintf(desc, sizeof(desc), "<< %s", umpStreamLabel(sts));
            col = C_CYAN;
        }
        else {
            snprintf(desc, sizeof(desc), "MT=%01X G%u  %02X %02X %02X",
                     mt, grp, b1, b2, b3);
        }

        _txt(MARGIN, Y_HEX + 17, col, C_BG, &lgfx::fonts::Font2, desc);
    }

    // ── VEL BAR ─────────────────────────────────────────────
    void _drawVel() {
        _fill(Y_VEL, H_VEL, C_BG);
        _hline(Y_VEL + H_VEL);

        int bx = MARGIN, bw = SCR_W - 72, bh = H_VEL - 4, by = Y_VEL + 2;
        int fill = (_vel * bw) / 127;
        _tft.fillRect(bx,        by, fill,      bh, C_CYAN);
        _tft.fillRect(bx + fill, by, bw - fill, bh, C_DIV);

        char buf[10]; snprintf(buf, sizeof(buf), "V:%3u", _vel);
        _txt(bx + bw + 6, Y_VEL + 1, C_DIV, C_BG, &lgfx::fonts::Font2, buf);
    }

    // ── LOG ─────────────────────────────────────────────────

    void _addLog(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3,
                 uint8_t mt, bool isTx)
    {
        if (mt == 0x0) return;  // skip utility/timing/NOOP

        LogLine& e = _log[_logN % N_LOG];
        uint8_t st  = b1 & 0xF0;
        uint8_t grp = b0 & 0xF;
        uint8_t ch  = b1 & 0x0F;
        const char* dir = isTx ? "T:" : "R:";
        e.c = C_DIV;

        if (mt == 0x2 || mt == 0x4) {
            if (st == 0x90 && b3 > 0) {
                snprintf(e.t, sizeof(e.t), "%sNoteOn  G%u Ch%u N%u V%u",
                         dir, grp, ch, b2, b3);
                e.c = C_GREEN;
            }
            else if (st == 0x80 || (st == 0x90 && b3 == 0)) {
                snprintf(e.t, sizeof(e.t), "%sNoteOff G%u Ch%u N%u",
                         dir, grp, ch, b2);
                e.c = C_GRAY;
            }
            else if (st == 0xB0) {
                snprintf(e.t, sizeof(e.t), "%sCC      G%u Ch%u CC%u V%u",
                         dir, grp, ch, b2, b3);
                e.c = C_CYAN;
            }
            else if (st == 0xE0) {
                snprintf(e.t, sizeof(e.t), "%sPBend   G%u Ch%u %02X%02X",
                         dir, grp, ch, b2, b3);
                e.c = C_MAGENTA;
            }
            else {
                snprintf(e.t, sizeof(e.t), "%s%02X %02X %02X %02X",
                         dir, b0, b1, b2, b3);
            }
        }
        else if (mt == 0x3 || mt == 0x5) {
            snprintf(e.t, sizeof(e.t), "%sSysEx G%u %02X", dir, grp, b1);
            e.c = C_YELLOW;
        }
        else if (mt == 0xF) {
            uint16_t sts = ((uint16_t)(b0 & 0x03) << 8) | b1;
            snprintf(e.t, sizeof(e.t), "%s%s", dir, umpStreamLabel(sts));
            e.c = isTx ? C_GREEN : C_CYAN;
        }
        else {
            snprintf(e.t, sizeof(e.t), "%sMT%01X %02X %02X %02X %02X",
                     dir, mt, b0, b1, b2, b3);
        }

        _logN++;
        _drawLog();
    }

    void _drawLog() {
        _fill(Y_LOG, H_LOG, C_BG);
        _hline(Y_LOG + H_LOG);

        for (int i = 0; i < N_LOG; i++) {
            if (_logN < N_LOG - i) continue;
            int idx = (_logN - N_LOG + i) % N_LOG;
            if (idx < 0) idx += N_LOG;
            _txt(MARGIN, Y_LOG + 1 + i * H_LLINE, _log[idx].c, C_BG,
                 &lgfx::fonts::Font2, _log[idx].t);
        }
    }

    // ── STATUS BAR ──────────────────────────────────────────
    // │  RX: xxxx    TX: xxxx                   MIDI 2.0  │
    void _drawStatus() {
        _fill(Y_STATUS, H_STATUS, C_HEADER);

        char buf[40];
        snprintf(buf, sizeof(buf), "RX: %-5lu   TX: %-5lu",
                 (unsigned long)_rx, (unsigned long)_tx);
        _txt(MARGIN, Y_STATUS + 4, C_WHITE, C_HEADER,
             &lgfx::fonts::Font2, buf);

        const char* ps = (_proto == 1) ? "MIDI 2.0" : "MIDI 1.0";
        uint32_t    pc = (_proto == 1) ? C_GREEN    : C_ORANGE;
        int pw = strlen(ps) * 12;
        _txt(SCR_W - MARGIN - pw, Y_STATUS + 4, pc, C_HEADER,
             &lgfx::fonts::Font2, ps);
    }
};

// ═══════════════════════════════════════════════════════════════
// SERIAL STUB — bare RP2040/RP2350 Pico (no display)
// ═══════════════════════════════════════════════════════════════
#else  // !HAS_DISPLAY

class UMPDisplay {
public:
    void init() {
        pinMode(LED_PIN, OUTPUT);
        digitalWrite(LED_PIN, LOW);
        Serial.println("[MIDI2] USB MIDI 2.0 — RP2040/RP2350");
        Serial.println("[MIDI2] No display: using Serial monitor");
    }

    void setConnected(bool c) {
        Serial.printf("[MIDI2] USB %s\n", c ? "mounted" : "disconnected");
        digitalWrite(LED_PIN, c ? HIGH : LOW);
    }

    void setProtocol(uint8_t alt) {
        Serial.printf("[MIDI2] Alt %d — %s\n", alt,
                      alt ? "MIDI 2.0 / UMP" : "MIDI 1.0");
    }

    void pushRxUMP(const uint32_t* words, uint8_t nw) {
        _rx++;
        char desc[48];
        _decode(words[0], words, nw, desc, sizeof(desc));
        Serial.printf("RX #%-4lu  %08lX  %s\n",
                      (unsigned long)_rx, (unsigned long)words[0], desc);
    }

    void pushTxUMP(const uint32_t* words, uint8_t nw) {
        _tx++;
        char desc[48];
        _decode(words[0], words, nw, desc, sizeof(desc));
        Serial.printf("TX #%-4lu  %08lX  %s\n",
                      (unsigned long)_tx, (unsigned long)words[0], desc);
    }

    void pulseRx() {
        static bool s = false; s = !s;
        digitalWrite(LED_PIN, s ? HIGH : LOW);
    }

private:
    uint32_t _rx = 0, _tx = 0;

    static void _decode(uint32_t w0, const uint32_t* words, uint8_t nw,
                        char* buf, size_t sz)
    {
        uint8_t b0 = w0 & 0xFF, b1 = (w0 >> 8) & 0xFF;
        uint8_t b2 = (w0 >> 16) & 0xFF, b3 = (w0 >> 24) & 0xFF;
        uint8_t mt = (b0 >> 4) & 0xF, grp = b0 & 0xF;
        uint8_t st = b1 & 0xF0, ch = b1 & 0x0F;

        if (mt == 0x2 || mt == 0x4) {
            if      (st == 0x90 && b3 > 0)
                snprintf(buf, sz, "NoteOn G%u Ch%u N%u V%u", grp, ch, b2, b3);
            else if (st == 0x80 || (st == 0x90 && b3 == 0))
                snprintf(buf, sz, "NoteOff G%u Ch%u N%u", grp, ch, b2);
            else if (st == 0xB0)
                snprintf(buf, sz, "CC G%u Ch%u CC%u V%u", grp, ch, b2, b3);
            else
                snprintf(buf, sz, "MT%01X G%u %02X %02X", mt, grp, b2, b3);
        }
        else if (mt == 0xF) {
            uint16_t sts = streamStatus(words[0]);
            snprintf(buf, sz, "%s", umpStreamLabel(sts));
        }
        else {
            snprintf(buf, sz, "MT%01X G%u %02X %02X %02X", mt, grp, b1, b2, b3);
        }
    }
};

#endif  // HAS_DISPLAY

#endif // UMP_DISPLAY_H
