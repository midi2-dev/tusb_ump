#!/bin/bash
# ump_solo.sh — Send UMP NoteOn/NoteOff to T-PicoC3 for display demonstration
#
# Animates the T-PicoC3 display for photography and filming:
#   - Display shows R: (incoming notes) and T: (echo back)
#   - Velocity bar, hex view, and RX/TX counters all update live
#
# Endpoint ID is machine-specific. Update EP_C if the device changes port.
# To find the current ID run (in an interactive terminal, not piped):
#   midi enumerate midi-services-endpoints --verbose
#
# Usage:
#   bash ump_solo.sh            # 20 rounds
#   bash ump_solo.sh loop       # infinite loop (Ctrl+C to stop)
#
# Copyright (c) 2026 Saulo Verissimo
# SPDX-License-Identifier: MIT

# ── Endpoint ID (update if device changes port/hub) ───────────────────────────
EP_C='\\?\swd#midisrv#midiu_ks_404377584258157766_outpin.0_inpin.2#{e7cce071-3c03-423f-88d3-f1045d02552b}'

# ── Config ────────────────────────────────────────────────────────────────────
MODE=${1:-20}
NOTE_HOLD=0.35
NOTE_GAP=0.12

NOTES=(60 64 67 72 76 79 84 79 76 72 67 64)
VELS=( 55 70 85 100 110 120 127 120 110 100 85 70)
NAMES=(C4 E4 G4 C5 E5 G5 C6 G5 E5 C5 G4 E4)

# ── Helpers ───────────────────────────────────────────────────────────────────
send()     { midi endpoint "$1" send-message "$2" >/dev/null 2>&1; }
note_on()  { send "$1" "$(printf '0x%02X%02X%02X%02X' 0x20 0x90 "$2" "$3")"; }
note_off() { send "$1" "$(printf '0x%02X%02X%02X%02X' 0x20 0x80 "$2" 0)";    }

vel_bar() {
    local v=$1
    local filled=$(( v * 20 / 127 ))
    local bar=""
    for (( i=0; i<filled; i++ ));  do bar+="█"; done
    for (( i=filled; i<20; i++ )); do bar+="░"; done
    printf "%s" "$bar"
}

run_round() {
    local r=$1
    local idx=$(( r % ${#NOTES[@]} ))
    local note=${NOTES[$idx]}
    local vel=${VELS[$idx]}
    local name=${NAMES[$idx]}

    note_on "$EP_C" $note $vel
    printf "\r  [%4d]  %-3s (N=%-3d)  V=%-3d  |%s|" \
           $((r+1)) "$name" $note $vel "$(vel_bar $vel)"

    sleep $NOTE_HOLD

    note_off "$EP_C" $note
    sleep $NOTE_GAP
}

# ── Main ──────────────────────────────────────────────────────────────────────
echo "=== USB MIDI 2.0 — T-PicoC3 (RP2040) ==="
echo "  Device : RP2040 MIDI 2.0 Device"
echo "  Pattern: C major arpeggio  (ascending + descending)"
echo ""

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
