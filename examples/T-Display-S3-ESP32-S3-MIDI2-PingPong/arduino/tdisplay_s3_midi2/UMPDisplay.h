// UMPDisplay.h — Visual UMP monitor for T-Display-S3 (ST7789V 320x170)
//
// Displays real-time UMP traffic with decoded message types, velocity
// bar, scrolling event log with RX/TX direction indicators, and
// protocol negotiation timeline.
//
// Copyright (c) 2026 Saulo Verissimo
// SPDX-License-Identifier: MIT

#ifndef UMP_DISPLAY_H
#define UMP_DISPLAY_H

#include <Arduino.h>
#include <LovyanGFX.h>
#include <cstring>
#include <cstdio>
#include "mapping.h"
#include "ump_stream_handler.h"

class UMPDisplay {
public:

    void init() {
        pinMode(PIN_POWER_ON, OUTPUT);
        digitalWrite(PIN_POWER_ON, HIGH);

        _tft.init();
        _tft.setRotation(2);
        _tft.setBrightness(255);
        _tft.fillScreen(C_BG);
        pinMode(TFT_BL_PIN, OUTPUT);
        digitalWrite(TFT_BL_PIN, HIGH);

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

    // ── RX: push a received UMP packet for display ──────────
    void pushRxUMP(const uint32_t* words, uint8_t nw) {
        _rx++;
        _storeWords(words, nw);

        uint8_t b0 = _w[0] & 0xFF;
        uint8_t b1 = (_w[0] >> 8)  & 0xFF;
        uint8_t b2 = (_w[0] >> 16) & 0xFF;
        uint8_t b3 = (_w[0] >> 24) & 0xFF;
        uint8_t mt = (b0 >> 4) & 0xF;

        // Update velocity from NoteOn/NoteOff (MT=2 or MT=4)
        if (mt == 0x2 || mt == 0x4) {
            uint8_t st = b1 & 0xF0;
            if (st == 0x90 && b3 > 0) {
                _vel = (mt == 0x4 && nw >= 2)
                     ? (uint8_t)((_w[1] >> 17) & 0x7F)  // MT=4: 16-bit vel → 7-bit
                     : b3;                               // MT=2: 7-bit vel
            } else if (st == 0x80 || (st == 0x90 && b3 == 0)) {
                _vel = 0;
            }
        }

        _drawHex();
        _drawVel();
        _addLog(b0, b1, b2, b3, mt, false);
        _drawStatus();
    }

    // ── TX: push a transmitted UMP packet for display ───────
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

    // ── RX activity indicator ───────────────────────────────
    void pulseRx() {
        static bool s = false; s = !s;
        _tft.fillCircle(SCR_W - MARGIN - 4, Y_HEADER + H_HEADER / 2,
                        4, s ? C_CYAN : C_HEADER);
    }

private:
    // ── LGFX — T-Display-S3 (ST7789V 170x320, parallel 8-bit) ──
    class LGFX : public lgfx::LGFX_Device {
    public:
        LGFX() {
            { auto c = _bus.config();
              c.pin_wr=8; c.pin_rd=9; c.pin_rs=7;
              c.pin_d0=39; c.pin_d1=40; c.pin_d2=41; c.pin_d3=42;
              c.pin_d4=45; c.pin_d5=46; c.pin_d6=47; c.pin_d7=48;
              _bus.config(c); _panel.setBus(&_bus); }
            { auto c = _panel.config();
              c.pin_cs=6; c.pin_rst=5; c.pin_busy=-1;
              c.offset_rotation=1; c.offset_x=40;
              c.readable=false; c.invert=true;
              c.rgb_order=false; c.dlen_16bit=false; c.bus_shared=false;
              c.panel_width=170; c.panel_height=320;
              _panel.config(c); }
            setPanel(&_panel);
            { auto c = _bl.config();
              c.pin_bl=38; c.invert=false; c.freq=22000; c.pwm_channel=7;
              _bl.config(c); _panel.setLight(&_bl); }
        }
    private:
        lgfx::Bus_Parallel8 _bus;
        lgfx::Panel_ST7789  _panel;
        lgfx::Light_PWM     _bl;
    };

    LGFX    _tft;
    uint8_t _proto = 0;
    bool    _conn  = false;
    uint32_t _rx = 0, _tx = 0;
    uint8_t  _vel = 0, _nw = 0;
    uint32_t _w[4] = {};

    struct LogLine { char t[48]; uint32_t c; };
    LogLine _log[N_LOG];
    int     _logN = 0;

    // ── helpers ─────────────────────────────────────────────
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

    // ── full repaint ────────────────────────────────────────
    void _drawAll() {
        _drawHeader();
        _drawProto();
        _drawHex();
        _drawVel();
        _drawLog();
        _drawStatus();
    }

    // ── header ──────────────────────────────────────────────
    void _drawHeader() {
        _fill(Y_HEADER, H_HEADER, C_HEADER);
        _txt(MARGIN + 2, Y_HEADER + 4, C_WHITE, C_HEADER,
             &lgfx::fonts::Font2, "USB MIDI 2.0");
        _txt(SCR_W / 2 - 30, Y_HEADER + 4, C_DIV, C_HEADER,
             &lgfx::fonts::Font2, "T-Display-S3");
        uint32_t dot = _conn ? C_GREEN : C_ORANGE;
        _tft.fillCircle(SCR_W - MARGIN - 4, Y_HEADER + H_HEADER / 2, 4, dot);
    }

    // ── protocol tabs ───────────────────────────────────────
    void _drawProto() {
        int half = SCR_W / 2;
        uint32_t l_bg = (_proto == 0) ? C_ORANGE : C_DIV;
        uint32_t l_fg = (_proto == 0) ? C_BG     : C_GRAY;
        uint32_t r_bg = (_proto == 1) ? C_GREEN  : C_DIV;
        uint32_t r_fg = (_proto == 1) ? C_BG     : C_GRAY;

        _tft.fillRect(0,    Y_PROTO, half, H_PROTO, l_bg);
        _tft.fillRect(half, Y_PROTO, half, H_PROTO, r_bg);

        _txt(MARGIN + 4,        Y_PROTO + 2, l_fg, l_bg,
             &lgfx::fonts::Font2, "MIDI 1.0");
        _txt(half + MARGIN + 4, Y_PROTO + 2, r_fg, r_bg,
             &lgfx::fonts::Font2, "MIDI 2.0 / UMP");

        _hline(Y_PROTO + H_PROTO);
    }

    // ── UMP hex view ────────────────────────────────────────
    void _drawHex() {
        _fill(Y_HEX, H_HEX, C_BG);
        _hline(Y_HEX + H_HEX);

        if (_nw == 0) {
            _txt(MARGIN, Y_HEX + 8, C_DIV, C_BG, &lgfx::fonts::Font2,
                 "waiting for data...");
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
        char hex[48]; int cx = MARGIN;
        _tft.setFont(&lgfx::fonts::Font2);

        _tft.setTextColor(C_CYAN, C_BG);
        snprintf(hex, 4, "%02X ", b0);
        _tft.drawString(hex, cx, Y_HEX + 2); cx += 24;

        _tft.setTextColor(C_ORANGE, C_BG);
        snprintf(hex, 4, "%02X ", b1);
        _tft.drawString(hex, cx, Y_HEX + 2); cx += 24;

        _tft.setTextColor(C_DIV, C_BG);
        snprintf(hex, 8, "%02X %02X", b2, b3);
        _tft.drawString(hex, cx, Y_HEX + 2);

        if (_nw >= 2) {
            _tft.setTextColor(C_DIV, C_BG);
            snprintf(hex, 12, "  %08lX", (unsigned long)_w[1]);
            _tft.drawString(hex, cx + 48, Y_HEX + 2);
        }

        // Line 2: decoded description
        char desc[48]; uint32_t col = C_DIV;
        const char* tag = (mt == 0x4) ? "M2" : "M1";

        if (mt == 0x2 || mt == 0x4) {
            if      (st == 0x90 && b3 > 0) {
                snprintf(desc, 48, "%s NoteOn  G%u Ch%u  N=%u V=%u",
                         tag, grp, ch, b2, b3);
                col = C_GREEN;
            }
            else if (st == 0x80 || (st == 0x90 && b3 == 0)) {
                snprintf(desc, 48, "%s NoteOff G%u Ch%u  N=%u",
                         tag, grp, ch, b2);
                col = C_GRAY;
            }
            else if (st == 0xB0) {
                snprintf(desc, 48, "%s CC G%u Ch%u  CC=%u V=%u",
                         tag, grp, ch, b2, b3);
                col = C_CYAN;
            }
            else if (st == 0xE0) {
                snprintf(desc, 48, "%s PBend G%u Ch%u  %02X%02X",
                         tag, grp, ch, b2, b3);
                col = C_MAGENTA;
            }
            else {
                snprintf(desc, 48, "%s Msg G%u Ch%u  %02X %02X",
                         tag, grp, ch, b2, b3);
            }
        }
        else if (mt == 0x3 || mt == 0x5) {
            snprintf(desc, 48, "SysEx G%u [%u words]", grp, _nw);
            col = C_YELLOW;
        }
        else if (mt == 0xF) {
            uint16_t sts = streamStatus(_w[0]);
            const char* lbl = umpStreamLabel(sts);
            snprintf(desc, 48, "<< %s", lbl);
            col = C_CYAN;
        }
        else {
            snprintf(desc, 48, "MT=%01X G%u  %02X %02X %02X",
                     mt, grp, b1, b2, b3);
        }

        _txt(MARGIN, Y_HEX + 17, col, C_BG, &lgfx::fonts::Font2, desc);
    }

    // ── velocity bar ────────────────────────────────────────
    void _drawVel() {
        _fill(Y_VEL, H_VEL, C_BG);
        _hline(Y_VEL + H_VEL);

        int bx = MARGIN, bw = SCR_W - 72, bh = H_VEL - 4, by = Y_VEL + 2;
        int fill = (_vel * bw) / 127;
        _tft.fillRect(bx,        by, fill,      bh, C_CYAN);
        _tft.fillRect(bx + fill, by, bw - fill, bh, C_GRAY);

        char buf[12]; snprintf(buf, sizeof(buf), "V:%3u", _vel);
        _txt(bx + bw + 6, Y_VEL + 1, C_DIV, C_BG, &lgfx::fonts::Font2, buf);
    }

    // ── event log ───────────────────────────────────────────
    void _addLog(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3,
                 uint8_t mt, bool isTx)
    {
        LogLine& e = _log[_logN % N_LOG];
        uint8_t st  = b1 & 0xF0;
        uint8_t grp = b0 & 0xF;
        uint8_t ch  = b1 & 0x0F;
        const char* dir = isTx ? "T:" : "R:";
        e.c = C_DIV;

        if (mt == 0x2 || mt == 0x4) {
            if (st == 0x90 && b3 > 0) {
                snprintf(e.t, 48, "%sNoteOn  G%u Ch%u N%u V%u",
                         dir, grp, ch, b2, b3);
                e.c = C_GREEN;
            }
            else if (st == 0x80 || (st == 0x90 && b3 == 0)) {
                snprintf(e.t, 48, "%sNoteOff G%u Ch%u N%u",
                         dir, grp, ch, b2);
                e.c = C_GRAY;
            }
            else if (st == 0xB0) {
                snprintf(e.t, 48, "%sCC      G%u Ch%u CC%u V%u",
                         dir, grp, ch, b2, b3);
                e.c = C_CYAN;
            }
            else if (st == 0xE0) {
                snprintf(e.t, 48, "%sPBend   G%u Ch%u %02X%02X",
                         dir, grp, ch, b2, b3);
                e.c = C_MAGENTA;
            }
            else {
                snprintf(e.t, 48, "%s%02X %02X %02X %02X",
                         dir, b0, b1, b2, b3);
            }
        }
        else if (mt == 0x3 || mt == 0x5) {
            snprintf(e.t, 48, "%sSysEx G%u %02X", dir, grp, b1);
            e.c = C_YELLOW;
        }
        else if (mt == 0xF) {
            // Decode Stream message subtype
            uint16_t sts = ((uint16_t)(b0 & 0x03) << 8) | b1;
            const char* lbl = umpStreamLabel(sts);
            snprintf(e.t, 48, "%s%s", dir, lbl);
            e.c = isTx ? C_GREEN : C_CYAN;
        }
        else {
            snprintf(e.t, 48, "%sMT%01X %02X %02X %02X %02X",
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
                 &lgfx::fonts::Font0, _log[idx].t);
        }
    }

    // ── status bar ──────────────────────────────────────────
    void _drawStatus() {
        _fill(Y_STATUS, H_STATUS, C_HEADER);

        char buf[52];
        snprintf(buf, sizeof(buf), "RX: %-5lu   TX: %-5lu", _rx, _tx);
        _txt(MARGIN, Y_STATUS + 4, C_WHITE, C_HEADER,
             &lgfx::fonts::Font2, buf);

        const char* ps = (_proto == 1) ? "MIDI 2.0" : "MIDI 1.0";
        uint32_t pc = (_proto == 1) ? C_GREEN : C_ORANGE;
        int pw = strlen(ps) * 12;
        _txt(SCR_W - MARGIN - pw, Y_STATUS + 4, pc, C_HEADER,
             &lgfx::fonts::Font2, ps);
    }
};

#endif // UMP_DISPLAY_H
