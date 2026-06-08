#!/bin/bash
# ump_ping_pong.sh — Bidirectional UMP message exchange: T-PicoC3 ↔ ESP32-S3
#
# Creates a visual ping-pong effect on both displays:
#   T-PicoC3  → ascending  arpeggio (N 60..84)
#   ESP32-S3  → descending mirror   (N 72..48)
#
# Endpoint IDs are machine-specific. Update EP_C and EP_B if the devices
# change USB port or hub. Run to find current IDs:
#   midi enumerate midi-services-endpoints --verbose
#
# Usage:
#   bash ump_ping_pong.sh            # 20 rounds
#   bash ump_ping_pong.sh loop       # infinite loop (Ctrl+C to stop)
#
# Copyright (c) 2026 Saulo Verissimo
# SPDX-License-Identifier: MIT

# ── Endpoint IDs (update if devices change port/hub) ──────────────────────────
EP_C='\\?\swd#midisrv#midiu_ks_404377584258157766_outpin.0_inpin.2#{e7cce071-3c03-423f-88d3-f1045d02552b}'
EP_B='\\?\swd#midisrv#midiu_ks_13319646191189016750_outpin.0_inpin.2#{e7cce071-3c03-423f-88d3-f1045d02552b}'

# ── Config ────────────────────────────────────────────────────────────────────
MODE=${1:-20}
NOTE_HOLD=0.20
NOTE_GAP=0.08

NOTES=(60 64 67 72 76 79 84 79 76 72 67 64)
VELS=( 55 70 85 100 110 120 127 120 110 100 85 70)

# ── Helpers ───────────────────────────────────────────────────────────────────
send() { midi endpoint "$1" send-message "$2" >/dev/null 2>&1; }

note_on()  { send "$1" "$(printf '0x%02X%02X%02X%02X' 0x20 0x90 "$2" "$3")"; }
note_off() { send "$1" "$(printf '0x%02X%02X%02X%02X' 0x20 0x80 "$2" 0)";    }

run_round() {
    local r=$1
    local idx=$(( r % ${#NOTES[@]} ))
    local note=${NOTES[$idx]}
    local vel=${VELS[$idx]}
    local note_b=$(( 132 - note ))

    note_on  "$EP_C" $note   $vel &
    note_on  "$EP_B" $note_b $vel &
    wait

    printf "\r  [%4d] T-PicoC3→N=%-3d  ESP32→N=%-3d  V=%-3d" \
           $((r+1)) $note $note_b $vel

    sleep $NOTE_HOLD

    note_off "$EP_C" $note   &
    note_off "$EP_B" $note_b &
    wait

    sleep $NOTE_GAP
}

# ── Main ──────────────────────────────────────────────────────────────────────
echo "=== UMP Ping-Pong: T-PicoC3 (RP2040) ↔ ESP32-S3 (T-Display-S3) ==="

if [[ "$MODE" == "loop" ]]; then
    echo "  Infinite loop — Ctrl+C to stop"
    echo ""
    r=0
    while true; do run_round $r; (( r++ )); done
else
    for (( r=0; r<MODE; r++ )); do run_round $r; done
    echo ""
    echo "=== Done: $MODE rounds ==="
fi
