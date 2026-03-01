#!/bin/bash
# ump_storm.sh — Intense bidirectional UMP traffic between two ESP32-S3 devices
#
# Sends NoteOn/Off, CC (mod wheel + volume), and Pitch Bend to both
# MIDI 2.0 endpoints simultaneously, creating a rich visual display
# on both T-Display-S3 screens.
#
# Usage: bash ump_storm.sh [rounds]
#   rounds: number of cycles (default: 60)
#
# Copyright (c) 2026 Saulo Verissimo
# SPDX-License-Identifier: MIT

EP_A='\\\\?\\swd#midisrv#midiu_ks_9345629463034498376_outpin.0_inpin.2#{e7cce071-3c03-423f-88d3-f1045d02552b}'
EP_B='\\\\?\\swd#midisrv#midiu_ks_13319646191189016750_outpin.0_inpin.2#{e7cce071-3c03-423f-88d3-f1045d02552b}'

ROUNDS=${1:-60}

send() { midi endpoint "$1" send-message "$2" 2>/dev/null | grep -c "Sent" >/dev/null; }

NOTES_A=(48 55 60 64 67 72 76 79 84 88 91 96)
NOTES_B=(96 91 88 84 79 76 72 67 64 60 55 48)

echo "=== UMP Storm: $ROUNDS rounds (6 msgs/round) ==="
echo "  Device A ←→ Device B"
echo ""

for (( r=0; r<ROUNDS; r++ )); do
    idx=$(( r % 12 ))
    na=${NOTES_A[$idx]}
    nb=${NOTES_B[$idx]}
    va=$(( 30 + (r * 97 / ROUNDS) ))
    vb=$(( 127 - (r * 97 / ROUNDS) ))

    # NoteOn to both
    send "$EP_A" "$(printf '0x%02X%02X%02X%02X' 0x20 0x90 $na $va)"
    send "$EP_B" "$(printf '0x%02X%02X%02X%02X' 0x20 0x91 $nb $vb)"
    printf "\r  [%02d/%02d] A:N%-3d V%-3d | B:N%-3d V%-3d" $((r+1)) $ROUNDS $na $va $nb $vb
    sleep 0.12

    # CC sweep
    cc_a=$(( (r * 127) / ROUNDS ))
    cc_b=$(( 127 - cc_a ))
    send "$EP_A" "$(printf '0x%02X%02X%02X%02X' 0x20 0xB0 0x01 $cc_a)"
    send "$EP_B" "$(printf '0x%02X%02X%02X%02X' 0x20 0xB1 0x07 $cc_b)"
    sleep 0.08

    # Pitch Bend sweep
    if (( r % 4 < 2 )); then
        pb=$(( 64 + r )); if (( pb > 127 )); then pb=127; fi
    else
        pb=$(( 64 - r )); if (( pb < 0 )); then pb=0; fi
    fi
    send "$EP_A" "$(printf '0x%02X%02X%02X%02X' 0x20 0xE0 0x00 $pb)"
    send "$EP_B" "$(printf '0x%02X%02X%02X%02X' 0x20 0xE1 0x00 $((127 - pb)) )"
    sleep 0.08

    # NoteOff both
    send "$EP_A" "$(printf '0x%02X%02X%02X%02X' 0x20 0x80 $na 0)"
    send "$EP_B" "$(printf '0x%02X%02X%02X%02X' 0x20 0x81 $nb 0)"
    sleep 0.1
done

echo ""
echo "=== Storm complete: $ROUNDS rounds × 6 = $((ROUNDS * 6)) UMP packets ==="
