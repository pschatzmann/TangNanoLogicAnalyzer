#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Phil Schatzmann
# Hardware test with the board connected (no probes needed: uses the
# internal test pattern). Usage: tools/hw_test.sh [port] [baud] [--full]
#   --full also captures the whole memory (4M samples, ~2 min at 921600 baud)
set -u
PORT=${1:-/dev/ttyUSB1}
BAUD=${2:-921600}
FULL=${3:-}
HERE=$(dirname "$(realpath "$0")")
CHECK="$HERE/../sim/pty/check_capture.py"
T=$(mktemp -d)
DEV="ols:conn=$PORT:serialcomm=$BAUD/8n1"
fails=0
# libsigrok 0.5.x writes 4 bytes per sample in binary output, newer 2
UNIT=4

sr() { timeout 600 sigrok-cli -d "$DEV" "$@"; }
check() {   # name, file, mask
    if out=$(python3 "$CHECK" "$2" $UNIT "$3" 1); then echo "ok:   $1: $out"
    else echo "FAIL: $1: $out" | head -3; fails=$((fails + 1)); fi
}

# drop anything left in the USB buffer from a previous bitstream
timeout 2 cat "$PORT" > /dev/null 2>&1

sr --show 2>&1 | grep -E "^ols|Maximum" || { echo "FAIL: device not found"; exit 1; }
sr --show 2>&1 | grep -q "sdram:F" && { echo "FAIL: SDRAM calibration"; fails=$((fails + 1)); }

sr --config samplerate=100M:pattern=Internal --samples 100000 -O binary -o "$T/a.bin"
check "16 channels, 100 MHz, 100k samples" "$T/a.bin" 0xffff
sr --channels 0,1,2,3,4,5,6,7 --config samplerate=100M:pattern=Internal --samples 100000 -O binary -o "$T/b.bin"
check "channels 0-7, 100 MHz, 100k samples" "$T/b.bin" 0xff
sr --channels 8,9,10,11,12,13,14,15 --config samplerate=100M:pattern=Internal --samples 100000 -O binary -o "$T/c.bin"
if python3 - "$T/c.bin" $UNIT <<'PY'
import sys
d = open(sys.argv[1], "rb").read(); u = int(sys.argv[2])
s = [int.from_bytes(d[i:i+u], "little") for i in range(0, len(d), u)]
ok = all(v & 0xff == 0 for v in s) and all(((s[i] >> 8) - (s[i-1] >> 8)) & 0xff in (0, 1) for i in range(1, len(s)))
print(f"{len(s)} samples, high byte {s[0]>>8:#x} .. {s[-1]>>8:#x}")
sys.exit(0 if ok else 1)
PY
then echo "ok:   channels 8-15"; else echo "FAIL: channels 8-15"; fails=$((fails + 1)); fi

# trigger on bit 15 of the test pattern: the sample at the trigger
# position must be 0x8000
for ratio in 50 10; do
    sr --config samplerate=100M:pattern=Internal:captureratio=$ratio -t 15=1 --samples 10000 -O binary -o "$T/t.bin"
    exp=$(python3 -c "r=2500; d=int(r*(1-$ratio/100)); print((r-d)*4-1)")
    got=$(python3 -c "
import sys; d=open('$T/t.bin','rb').read(); u=$UNIT
s=[int.from_bytes(d[i:i+u],'little') for i in range(0,len(d),u)]
print(s.index(0x8000) if 0x8000 in s else None)")
    if [ "$got" = "$exp" ]; then echo "ok:   trigger ratio $ratio%: 0x8000 at sample $got"
    else echo "FAIL: trigger ratio $ratio%: 0x8000 at sample $got, expected $exp"; fails=$((fails + 1)); fi
done

if [ "$FULL" = "--full" ]; then
    sr --config samplerate=100M:pattern=Internal --samples 4194288 -O binary -o "$T/f.bin"
    check "full memory, 4M samples" "$T/f.bin" 0xffff
fi

rm -rf "$T"
[ $fails -eq 0 ] && echo "ALL HARDWARE TESTS PASSED" || echo "$fails HARDWARE TEST(S) FAILED"
exit $fails
