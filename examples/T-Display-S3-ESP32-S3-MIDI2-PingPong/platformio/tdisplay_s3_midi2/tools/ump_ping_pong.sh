#!/bin/bash
# ump_ping_pong.sh — Bidirectional UMP message exchange between two ESP32-S3 devices
#
# Sends alternating NoteOn/NoteOff messages to both MIDI 2.0 endpoints,
# creating a visual ping-pong effect on the T-Display-S3 screens.
#
# Usage: bash ump_ping_pong.sh [rounds]
#   rounds: number of note cycles (default: 20)
#
# Copyright (c) 2026 Saulo Verissimo
# SPDX-License-Identifier: MIT

EP_A='\\\\?\\swd#midisrv#midiu_ks_9345629463034498376_outpin.0_inpin.2#{e7cce071-3c03-423f-88d3-f1045d02552b}'
EP_B='\\\\?\\swd#midisrv#midiu_ks_13319646191189016750_outpin.0_inpin.2#{e7cce071-3c03-423f-88d3-f1045d02552b}'

ROUNDS=${1:-20}

# C major scale across 2 octaves
NOTES=(60 64 67 72 76 79 84 79 76 72 67 64)
VELS=(40 55 70 85 100 115 127 115 100 85 70 55)

send() {
    midi endpoint "$1" send-message "$2" 2>/dev/null | grep -c "Sent" >/dev/null
}

# Build UMP MIDI 1.0 Channel Voice word: MT=2, Group=0, Status+Ch, Data1, Data2
ump_word() {
    printf "0x%02X%02X%02X%02X" 0x20 "$1" "$2" "$3"
}

echo "=== UMP Ping-Pong: $ROUNDS rounds ==="
echo "  Device A ←→ Device B"
echo ""

for (( r=0; r<ROUNDS; r++ )); do
    idx=$(( r % ${#NOTES[@]} ))
    note=${NOTES[$idx]}
    vel=${VELS[$idx]}

    if (( r % 2 == 0 )); then
        target="$EP_A"
        label="A"
    else
        target="$EP_B"
        label="B"
    fi

    # NoteOn
    w=$(ump_word 0x90 $note $vel)
    send "$target" "$w"
    printf "\r  [%02d/%02d] → Device %s: NoteOn  N=%-3d V=%-3d" $((r+1)) $ROUNDS "$label" $note $vel

    sleep 0.25

    # NoteOff
    w=$(ump_word 0x80 $note 0)
    send "$target" "$w"

    sleep 0.15
done

echo ""
echo "=== Done ==="
