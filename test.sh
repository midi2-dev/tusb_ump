#!/usr/bin/env bash
# test.sh — tusb_ump project integrity check
#
# Verifies:
#   1. Library core files are present at the repo root
#   2. Each PlatformIO example has all required source files
#   3. symlink://../../../../ paths resolve to the repo root
#   4. PlatformIO can parse every environment (pio project config)
#   5. Internal files (*_handoff.md) are NOT tracked by git
#
# Usage:
#   bash test.sh              # run all checks
#   bash test.sh --no-pio     # skip pio project config (faster, offline)
#
# Exit code: 0 = all pass, 1 = one or more failures

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$SCRIPT_DIR"

# ── Options ──────────────────────────────────────────────────────────────────
RUN_PIO=1
for arg in "$@"; do
    [ "$arg" = "--no-pio" ] && RUN_PIO=0
done

# ── Colour helpers ────────────────────────────────────────────────────────────
if [ -t 1 ]; then
    GREEN='\033[32m'; RED='\033[31m'; CYAN='\033[36m'; RESET='\033[0m'
else
    GREEN=''; RED=''; CYAN=''; RESET=''
fi

PASS=0; FAIL=0

ok()      { PASS=$((PASS+1)); echo -e "${GREEN}[OK]${RESET}   $*"; }
fail()    { FAIL=$((FAIL+1)); echo -e "${RED}[FAIL]${RESET} $*"; }
section() { echo -e "\n${CYAN}── $* ──${RESET}"; }

rel() { echo "${1#$REPO/}"; }   # strip repo prefix for readable output

# ── Check helpers ─────────────────────────────────────────────────────────────
check_file() {
    local f="$1"
    [ -f "$f" ] && ok "$(rel "$f")" || fail "missing: $(rel "$f")"
}

check_symlink_depth() {
    # Verify that symlink://../../../../ from a platformio.ini resolves to repo root.
    # depth = number of "../" steps in the symlink path.
    local ini="$1" depth="$2"
    local dir target
    dir="$(dirname "$ini")"
    target="$dir"
    for _ in $(seq 1 "$depth"); do target="$(dirname "$target")"; done
    if [ -f "$target/ump_device.cpp" ]; then
        ok "symlink depth $depth from $(rel "$ini") → repo root"
    else
        fail "symlink depth $depth from $(rel "$ini") does not reach repo root (ump_device.cpp not found at $target)"
    fi
}

check_pio_config() {
    # Runs 'pio project config' (no -e flag) to validate a platformio.ini.
    # Reports the environments found or the error if it fails.
    local ini_dir="$1" label="$2"
    local out
    if out="$(cd "$ini_dir" && pio project config 2>&1)"; then
        local envs
        envs="$(echo "$out" | grep -E '^\[env:' | sed 's/\[env://;s/\]//' | tr '\n' ' ' | sed 's/ $//')"
        ok "pio project config OK [$label] — envs: ${envs:-<none found>}"
    else
        fail "pio project config FAILED [$label] (in $(rel "$ini_dir"))"
        echo "$out" | tail -5 | sed 's/^/       /'
    fi
}

check_git_not_tracked() {
    local pattern="$1"
    local found
    found="$(cd "$REPO" && git ls-files | grep -E "$pattern" 2>/dev/null || true)"
    if [ -z "$found" ]; then
        ok "git does not track '$pattern'"
    else
        fail "git tracks '$pattern': $found"
    fi
}

check_gitignore_has() {
    local pattern="$1"
    grep -qF "$pattern" "$REPO/.gitignore" 2>/dev/null \
        && ok ".gitignore contains '$pattern'" \
        || fail ".gitignore missing '$pattern'"
}

# ══════════════════════════════════════════════════════════════════════════════
section "1. Library core files"
# ══════════════════════════════════════════════════════════════════════════════
for f in ump_device.cpp ump_device.h ump.h library.properties library.json; do
    check_file "$REPO/$f"
done

# ══════════════════════════════════════════════════════════════════════════════
section "2. T-Display-S3-ESP32-S3-MIDI2-PingPong (PlatformIO)"
# ══════════════════════════════════════════════════════════════════════════════
S3_PIO="$REPO/examples/T-Display-S3-ESP32-S3-MIDI2-PingPong/platformio/tdisplay_s3_midi2"
check_file "$S3_PIO/platformio.ini"
check_symlink_depth "$S3_PIO/platformio.ini" 4
for f in src/main.cpp src/usb_descriptors.cpp src/UMPDisplay.h src/mapping.h src/ump_stream_handler.h; do
    check_file "$S3_PIO/$f"
done

# ══════════════════════════════════════════════════════════════════════════════
section "3. T-Display-S3-ESP32-S3-MIDI2-PingPong (Arduino IDE)"
# ══════════════════════════════════════════════════════════════════════════════
S3_ARD="$REPO/examples/T-Display-S3-ESP32-S3-MIDI2-PingPong/arduino/tdisplay_s3_midi2"
check_file "$S3_ARD/tdisplay_s3_midi2.ino"
for f in usb_descriptors.cpp UMPDisplay.h mapping.h ump_stream_handler.h; do
    check_file "$S3_ARD/$f"
done

# ══════════════════════════════════════════════════════════════════════════════
section "4. T-PicoC3-MIDI2-PingPong (PlatformIO — 3 environments)"
# ══════════════════════════════════════════════════════════════════════════════
PICO_PIO="$REPO/examples/T-PicoC3-MIDI2-PingPong/platformio/tpicoc3_midi2"
check_file "$PICO_PIO/platformio.ini"
check_symlink_depth "$PICO_PIO/platformio.ini" 4
for f in src/main.cpp src/usb_descriptors.cpp src/UMPDisplay.h src/mapping.h src/ump_stream_handler.h; do
    check_file "$PICO_PIO/$f"
done

# ══════════════════════════════════════════════════════════════════════════════
section "5. PlatformIO config validation"
# ══════════════════════════════════════════════════════════════════════════════
if [ "$RUN_PIO" -eq 1 ] && command -v pio > /dev/null 2>&1; then
    check_pio_config "$S3_PIO"   "T-Display-S3-MIDI2"
    check_pio_config "$PICO_PIO" "T-PicoC3-MIDI2 / RP-Pico / RP-Pico2"
elif [ "$RUN_PIO" -eq 0 ]; then
    echo "   (pio checks skipped via --no-pio)"
else
    echo "   (pio not found in PATH — skipping pio checks)"
fi

# ══════════════════════════════════════════════════════════════════════════════
section "6. Git integrity"
# ══════════════════════════════════════════════════════════════════════════════
if command -v git > /dev/null 2>&1 && [ -d "$REPO/.git" ]; then
    check_git_not_tracked "_handoff\\.md$"
    check_gitignore_has "*_handoff.md"
    # Old folder must not appear in git tracking
    old_tracked="$(cd "$REPO" && git ls-files | grep "^examples/esp32_s3/" || true)"
    if [ -z "$old_tracked" ]; then
        ok "old 'examples/esp32_s3/' not tracked by git"
    else
        fail "old 'examples/esp32_s3/' still tracked: $old_tracked"
    fi
else
    echo "   (git not available — skipping git checks)"
fi

# ══════════════════════════════════════════════════════════════════════════════
section "Results"
# ══════════════════════════════════════════════════════════════════════════════
TOTAL=$((PASS+FAIL))
echo ""
echo -e "  Passed: ${GREEN}$PASS${RESET} / $TOTAL"
[ "$FAIL" -gt 0 ] && echo -e "  Failed: ${RED}$FAIL${RESET} / $TOTAL"
echo ""

[ "$FAIL" -eq 0 ] && echo -e "${GREEN}All checks passed.${RESET}" && exit 0
echo -e "${RED}$FAIL check(s) failed.${RESET}" && exit 1
