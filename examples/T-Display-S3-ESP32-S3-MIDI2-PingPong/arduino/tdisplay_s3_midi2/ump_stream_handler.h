// ump_stream_handler.h — UMP Stream Message (MT=0xF) handler
//
// Implements the device-side UMP Endpoint Discovery protocol per the
// UMP and MIDI 2.0 Protocol Specification v1.1 (M2-104-UM).
//
// When Windows MIDI Services (or any UMP-capable host) switches to
// Alternate Setting 1 (MIDI Streaming 2.0), it sends a series of
// MT=0xF Stream Messages to discover the device's capabilities:
//
//   Host → Device   EP Discovery (status 0x000)
//   Device → Host   EP Info Notification (0x001)
//   Device → Host   Device Identity Notification (0x002)
//   Device → Host   Endpoint Name Notification (0x003)
//   Device → Host   Product Instance ID Notification (0x004)
//
//   Host → Device   Stream Configuration Request (0x005)
//   Device → Host   Stream Configuration Notification (0x006)
//
//   Host → Device   Function Block Discovery (0x010)
//   Device → Host   Function Block Info Notification (0x011)
//   Device → Host   Function Block Name Notification (0x012)
//
// This handler parses incoming requests and generates the correct
// responses.  All responses are sent via tud_ump_write().
//
// Copyright (c) 2026 Saulo Verissimo
// SPDX-License-Identifier: MIT

#ifndef UMP_STREAM_HANDLER_H
#define UMP_STREAM_HANDLER_H

#include <cstdint>
#include <cstring>
#include "ump_device.h"

// ─────────────────────────────────────────────────────────────
// UMP Stream Message status codes (10-bit)
// Reference: M2-104-UM v1.1, Section 7.1
// ─────────────────────────────────────────────────────────────

#define UMP_STREAM_STATUS_EP_DISCOVERY      0x000
#define UMP_STREAM_STATUS_EP_INFO           0x001
#define UMP_STREAM_STATUS_DEVICE_INFO       0x002
#define UMP_STREAM_STATUS_EP_NAME           0x003
#define UMP_STREAM_STATUS_PRODUCT_ID        0x004
#define UMP_STREAM_STATUS_STREAM_CFG_REQ    0x005
#define UMP_STREAM_STATUS_STREAM_CFG        0x006
#define UMP_STREAM_STATUS_FB_DISCOVERY      0x010
#define UMP_STREAM_STATUS_FB_INFO           0x011
#define UMP_STREAM_STATUS_FB_NAME           0x012
#define UMP_STREAM_STATUS_START_CLIP        0x020
#define UMP_STREAM_STATUS_END_CLIP          0x021

// ── Format field (bits [3:2] of wire byte 0) ─────────────────
#define UMP_STREAM_FMT_COMPLETE   0
#define UMP_STREAM_FMT_START      1
#define UMP_STREAM_FMT_CONTINUE   2
#define UMP_STREAM_FMT_END        3

// ── Endpoint Discovery filter bitmap (wire byte 4) ───────────
#define UMP_FILTER_EP_INFO        0x01
#define UMP_FILTER_DEVICE_ID      0x02
#define UMP_FILTER_EP_NAME        0x04
#define UMP_FILTER_PRODUCT_ID     0x08
#define UMP_FILTER_STREAM_CFG     0x10

// ── Protocol capabilities (EP Info byte 6) ───────────────────
#define UMP_PROTO_CAP_MIDI2       0x01   // supports MIDI 2.0 Protocol
#define UMP_PROTO_CAP_MIDI1       0x02   // supports MIDI 1.0 Protocol

// ── Protocol selection (Stream Configuration) ────────────────
#define UMP_PROTO_MIDI1           0x01
#define UMP_PROTO_MIDI2           0x02

// ── Function Block direction ─────────────────────────────────
#define UMP_FB_DIR_INPUT          0x01   // receives from host
#define UMP_FB_DIR_OUTPUT         0x02   // sends to host
#define UMP_FB_DIR_BIDIRECTIONAL  0x03

// ─────────────────────────────────────────────────────────────
// Wire byte packing — ESP32 little-endian ↔ UMP wire order
//
// UMP words are transmitted MSB-first on USB.  On ESP32 (LE),
// the LSB of a uint32_t occupies the lowest memory address,
// which is the first byte sent by the USB bulk endpoint.
//
// packWord(b0, b1, b2, b3) places wire byte 0 at the LSB:
//   address+0 = b0  (first on wire — contains MT nibble)
//   address+1 = b1
//   address+2 = b2
//   address+3 = b3  (last on wire)
// ─────────────────────────────────────────────────────────────

static inline uint32_t packWord(uint8_t b0, uint8_t b1,
                                uint8_t b2, uint8_t b3)
{
    return (uint32_t)b0
         | ((uint32_t)b1 << 8)
         | ((uint32_t)b2 << 16)
         | ((uint32_t)b3 << 24);
}

static inline uint8_t wireB0(uint32_t w) { return (uint8_t)(w);        }
static inline uint8_t wireB1(uint32_t w) { return (uint8_t)(w >> 8);   }
static inline uint8_t wireB2(uint32_t w) { return (uint8_t)(w >> 16);  }
static inline uint8_t wireB3(uint32_t w) { return (uint8_t)(w >> 24);  }

// ── Stream header helpers ────────────────────────────────────

static inline uint32_t streamWord0(uint8_t format, uint16_t status,
                                   uint8_t d2, uint8_t d3)
{
    uint8_t b0 = 0xF0 | ((format & 0x03) << 2) | ((status >> 8) & 0x03);
    uint8_t b1 = status & 0xFF;
    return packWord(b0, b1, d2, d3);
}

static inline uint16_t streamStatus(uint32_t w0)
{
    return ((uint16_t)(wireB0(w0) & 0x03) << 8) | wireB1(w0);
}

static inline uint8_t streamFormat(uint32_t w0)
{
    return (wireB0(w0) >> 2) & 0x03;
}

// ─────────────────────────────────────────────────────────────
// Device configuration — all fields the host may discover
// ─────────────────────────────────────────────────────────────

struct UMPStreamConfig {
    // UMP protocol version
    uint8_t  umpVersionMajor   = 1;
    uint8_t  umpVersionMinor   = 1;

    // Function blocks
    uint8_t  numFunctionBlocks     = 1;
    bool     staticFunctionBlocks  = true;

    // Protocol capabilities
    uint8_t  protocolCaps = UMP_PROTO_CAP_MIDI1 | UMP_PROTO_CAP_MIDI2;

    // SysEx Manufacturer ID (3 bytes — 1-byte IDs use 0x00 0x00 ID)
    uint8_t  manufacturerId[3] = { 0x00, 0x00, 0x7D };  // 0x7D = educational
    uint16_t familyId          = 0x0001;
    uint16_t modelId           = 0x0001;
    char     swRevision[4]     = { '0', '1', '0', '0' };

    // Endpoint strings (UTF-8, null-terminated)
    const char* endpointName      = "MIDI 2.0 Device";
    const char* productInstanceId = "MIDI2-001";

    // Function Block 0
    const char* fbName        = "Group 0";
    uint8_t     fbDirection   = UMP_FB_DIR_BIDIRECTIONAL;
    uint8_t     fbFirstGroup  = 0;
    uint8_t     fbNumGroups   = 1;
    uint8_t     fbUIHint      = 0x00;   // 0=unknown
    uint8_t     fbMidi10      = 0x00;   // 0=not a MIDI 1.0 stream
    uint8_t     fbMidiCIVer   = 0x00;
    uint8_t     fbSysEx8      = 0x00;
};

// ─────────────────────────────────────────────────────────────
// TX callback — invoked after each response is sent
// ─────────────────────────────────────────────────────────────

typedef void (*UMPStreamTxCb)(const uint32_t* words, uint8_t nw,
                              const char* label);

// ─────────────────────────────────────────────────────────────
// Human-readable label for any Stream message status
// ─────────────────────────────────────────────────────────────

static const char* umpStreamLabel(uint16_t status)
{
    switch (status) {
    case UMP_STREAM_STATUS_EP_DISCOVERY:   return "EP Discovery";
    case UMP_STREAM_STATUS_EP_INFO:        return "EP Info";
    case UMP_STREAM_STATUS_DEVICE_INFO:    return "Device ID";
    case UMP_STREAM_STATUS_EP_NAME:        return "EP Name";
    case UMP_STREAM_STATUS_PRODUCT_ID:     return "Product ID";
    case UMP_STREAM_STATUS_STREAM_CFG_REQ: return "StreamCfg Req";
    case UMP_STREAM_STATUS_STREAM_CFG:     return "StreamCfg OK";
    case UMP_STREAM_STATUS_FB_DISCOVERY:   return "FB Discovery";
    case UMP_STREAM_STATUS_FB_INFO:        return "FB Info";
    case UMP_STREAM_STATUS_FB_NAME:        return "FB Name";
    case UMP_STREAM_STATUS_START_CLIP:     return "Start Clip";
    case UMP_STREAM_STATUS_END_CLIP:       return "End Clip";
    default:                               return "Stream ???";
    }
}

// ─────────────────────────────────────────────────────────────
// Internal — send a 4-word Stream message
// ─────────────────────────────────────────────────────────────

static void _streamSend(uint8_t itf, uint32_t w[4],
                        UMPStreamTxCb onTx, const char* label)
{
    tud_ump_write(itf, w, 4);
    if (onTx) onTx(w, 4, label);
}

// ─────────────────────────────────────────────────────────────
// Internal — send a UTF-8 string as one or more Stream packets
//
//   status    : UMP_STREAM_STATUS_EP_NAME / PRODUCT_ID / FB_NAME
//   str       : null-terminated UTF-8 string
//   prefixByte: ≥0 to insert a prefix byte (e.g., FB index) at
//               wire byte 2, shifting chars to byte 3 onward.
//               <0 means no prefix — chars start at byte 2.
// ─────────────────────────────────────────────────────────────

static void _streamSendString(uint8_t itf, uint16_t status,
                              const char* str, int prefixByte,
                              UMPStreamTxCb onTx, const char* label)
{
    const size_t len        = strlen(str);
    const size_t charsPerPkt = (prefixByte >= 0) ? 13 : 14;
    const size_t nPkts       = (len == 0) ? 1 : (len + charsPerPkt - 1) / charsPerPkt;

    size_t pos = 0;
    for (size_t pkt = 0; pkt < nPkts; pkt++) {

        uint8_t fmt;
        if      (nPkts == 1)         fmt = UMP_STREAM_FMT_COMPLETE;
        else if (pkt == 0)           fmt = UMP_STREAM_FMT_START;
        else if (pkt == nPkts - 1)   fmt = UMP_STREAM_FMT_END;
        else                         fmt = UMP_STREAM_FMT_CONTINUE;

        uint8_t buf[16];
        memset(buf, 0, sizeof(buf));

        // Header (bytes 0-1)
        buf[0] = 0xF0 | ((fmt & 0x03) << 2) | ((status >> 8) & 0x03);
        buf[1] = status & 0xFF;

        // Optional prefix (byte 2 for FB Name = FB index)
        size_t off = 2;
        if (prefixByte >= 0)
            buf[off++] = (uint8_t)prefixByte;

        // Fill remaining bytes with string characters
        while (off < 16 && pos < len)
            buf[off++] = (uint8_t)str[pos++];

        uint32_t w[4];
        w[0] = packWord(buf[0],  buf[1],  buf[2],  buf[3]);
        w[1] = packWord(buf[4],  buf[5],  buf[6],  buf[7]);
        w[2] = packWord(buf[8],  buf[9],  buf[10], buf[11]);
        w[3] = packWord(buf[12], buf[13], buf[14], buf[15]);

        _streamSend(itf, w, onTx, label);
    }
}

// ─────────────────────────────────────────────────────────────
// Response generators
// ─────────────────────────────────────────────────────────────

static void _replyEndpointInfo(uint8_t itf, const UMPStreamConfig& cfg,
                               UMPStreamTxCb onTx)
{
    uint32_t w[4];
    w[0] = streamWord0(UMP_STREAM_FMT_COMPLETE, UMP_STREAM_STATUS_EP_INFO,
                       cfg.umpVersionMajor, cfg.umpVersionMinor);
    w[1] = packWord(
        (cfg.staticFunctionBlocks ? 0x80 : 0x00) | (cfg.numFunctionBlocks & 0x7F),
        0x00,                   // JRTS capabilities (none)
        cfg.protocolCaps,
        0x00                    // extensions
    );
    w[2] = 0;
    w[3] = 0;
    _streamSend(itf, w, onTx, "EP Info");
}

static void _replyDeviceIdentity(uint8_t itf, const UMPStreamConfig& cfg,
                                 UMPStreamTxCb onTx)
{
    uint32_t w[4];
    w[0] = streamWord0(UMP_STREAM_FMT_COMPLETE, UMP_STREAM_STATUS_DEVICE_INFO,
                       0x00, 0x00);
    w[1] = packWord(
        0x00,
        cfg.manufacturerId[0],
        cfg.manufacturerId[1],
        cfg.manufacturerId[2]
    );
    w[2] = packWord(
        (cfg.familyId >> 8) & 0xFF, cfg.familyId & 0xFF,
        (cfg.modelId  >> 8) & 0xFF, cfg.modelId  & 0xFF
    );
    w[3] = packWord(
        cfg.swRevision[0], cfg.swRevision[1],
        cfg.swRevision[2], cfg.swRevision[3]
    );
    _streamSend(itf, w, onTx, "Device ID");
}

static void _replyEndpointName(uint8_t itf, const UMPStreamConfig& cfg,
                               UMPStreamTxCb onTx)
{
    _streamSendString(itf, UMP_STREAM_STATUS_EP_NAME,
                      cfg.endpointName, -1, onTx, "EP Name");
}

static void _replyProductInstanceId(uint8_t itf, const UMPStreamConfig& cfg,
                                    UMPStreamTxCb onTx)
{
    _streamSendString(itf, UMP_STREAM_STATUS_PRODUCT_ID,
                      cfg.productInstanceId, -1, onTx, "Product ID");
}

static void _replyStreamConfig(uint8_t itf, uint8_t requestedProto,
                               uint8_t jrts, const UMPStreamConfig& cfg,
                               UMPStreamTxCb onTx)
{
    // Confirm the requested protocol if we support it
    uint8_t proto = requestedProto;
    if (proto == UMP_PROTO_MIDI2 && !(cfg.protocolCaps & UMP_PROTO_CAP_MIDI2))
        proto = UMP_PROTO_MIDI1;
    else if (proto == UMP_PROTO_MIDI1 && !(cfg.protocolCaps & UMP_PROTO_CAP_MIDI1))
        proto = UMP_PROTO_MIDI2;

    uint32_t w[4];
    w[0] = streamWord0(UMP_STREAM_FMT_COMPLETE, UMP_STREAM_STATUS_STREAM_CFG,
                       proto, 0x00);    // JRTS: 0 (not supported)
    w[1] = 0;
    w[2] = 0;
    w[3] = 0;
    _streamSend(itf, w, onTx, "StreamCfg OK");
}

static void _replyFBInfo(uint8_t itf, uint8_t fbIdx,
                         const UMPStreamConfig& cfg, UMPStreamTxCb onTx)
{
    //  Word 0 byte 2: [active (1 bit)] [fb_index (7 bits)]
    //  Word 1 byte 0: [ui_hint (2)] [midi1.0 (2)] [direction (2)] [rsvd (2)]
    //  Word 1 byte 1: first_group
    //  Word 1 byte 2: num_groups
    //  Word 1 byte 3: midi_ci_version
    //  Word 2 byte 0: max_sysex8_streams

    uint32_t w[4];
    w[0] = streamWord0(UMP_STREAM_FMT_COMPLETE, UMP_STREAM_STATUS_FB_INFO,
                       0x80 | (fbIdx & 0x7F),   // active=1
                       0x00);                     // reserved
    w[1] = packWord(
        (cfg.fbUIHint  << 6) |
        (cfg.fbMidi10  << 4) |
        (cfg.fbDirection << 2),
        cfg.fbFirstGroup,
        cfg.fbNumGroups,
        cfg.fbMidiCIVer
    );
    w[2] = packWord(cfg.fbSysEx8, 0x00, 0x00, 0x00);
    w[3] = 0;
    _streamSend(itf, w, onTx, "FB Info");
}

static void _replyFBName(uint8_t itf, uint8_t fbIdx,
                         const UMPStreamConfig& cfg, UMPStreamTxCb onTx)
{
    _streamSendString(itf, UMP_STREAM_STATUS_FB_NAME,
                      cfg.fbName, (int)fbIdx, onTx, "FB Name");
}

// ─────────────────────────────────────────────────────────────
// Main entry point — process a received MT=0xF Stream message
//
// Call this from processRxUMP() when MT == 0xF.
// Returns a human-readable label for the received request,
// or nullptr if the message was not recognized.
// ─────────────────────────────────────────────────────────────

static const char* umpStreamHandleRx(uint8_t itf, const uint32_t* words,
                                     const UMPStreamConfig& cfg,
                                     UMPStreamTxCb onTx)
{
    const uint16_t status = streamStatus(words[0]);

    switch (status) {

    // ── Endpoint Discovery ───────────────────────────────────
    case UMP_STREAM_STATUS_EP_DISCOVERY: {
        const uint8_t filter = wireB0(words[1]);

        if (filter & UMP_FILTER_EP_INFO)
            _replyEndpointInfo(itf, cfg, onTx);
        if (filter & UMP_FILTER_DEVICE_ID)
            _replyDeviceIdentity(itf, cfg, onTx);
        if (filter & UMP_FILTER_EP_NAME)
            _replyEndpointName(itf, cfg, onTx);
        if (filter & UMP_FILTER_PRODUCT_ID)
            _replyProductInstanceId(itf, cfg, onTx);
        if (filter & UMP_FILTER_STREAM_CFG)
            _replyStreamConfig(itf, UMP_PROTO_MIDI2, 0x00, cfg, onTx);

        return "EP Discovery";
    }

    // ── Stream Configuration Request ─────────────────────────
    case UMP_STREAM_STATUS_STREAM_CFG_REQ: {
        const uint8_t protocol = wireB2(words[0]);
        const uint8_t jrts     = wireB3(words[0]);
        _replyStreamConfig(itf, protocol, jrts, cfg, onTx);
        return "StreamCfg Req";
    }

    // ── Function Block Discovery ─────────────────────────────
    case UMP_STREAM_STATUS_FB_DISCOVERY: {
        const uint8_t fbNum  = wireB2(words[0]);   // 0xFF = all
        const uint8_t filter = wireB3(words[0]);

        uint8_t first = (fbNum == 0xFF) ? 0 : fbNum;
        uint8_t last  = (fbNum == 0xFF) ? cfg.numFunctionBlocks : (fbNum + 1);
        if (last > cfg.numFunctionBlocks)
            last = cfg.numFunctionBlocks;

        for (uint8_t i = first; i < last; i++) {
            if (filter & 0x01) _replyFBInfo(itf, i, cfg, onTx);
            if (filter & 0x02) _replyFBName(itf, i, cfg, onTx);
        }
        return "FB Discovery";
    }

    default:
        return nullptr;
    }
}

#endif // UMP_STREAM_HANDLER_H
