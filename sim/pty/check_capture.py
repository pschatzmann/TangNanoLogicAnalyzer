#!/usr/bin/env python3
"""Check a sigrok-cli binary capture (-O binary) of the test pattern/probes.

usage: check_capture.py FILE UNITSIZE MASK STEP [TRIGGER_BIT]
  every sample must equal the previous one + STEP (compared under MASK);
  with TRIGGER_BIT, also report where that bit first goes high.
"""
import sys

data = open(sys.argv[1], "rb").read()
unit, mask, step = int(sys.argv[2]), int(sys.argv[3], 0), int(sys.argv[4])
trig_bit = int(sys.argv[5]) if len(sys.argv) > 5 else None
s = [int.from_bytes(data[i:i + unit], "little") for i in range(0, len(data), unit)]
bad = [i for i in range(1, len(s)) if ((s[i] - s[i - 1]) & mask) != (step & mask)]
print(f"{len(s)} samples, first {s[0]:#06x}, last {s[-1]:#06x}, {len(bad)} discontinuities")
for i in bad[:5]:
    print(f"  at {i}: {s[i-1]:#06x} -> {s[i]:#06x}")
if trig_bit is not None:
    hi = [i for i, v in enumerate(s) if v >> trig_bit & 1]
    print(f"bit {trig_bit} first high at sample {hi[0] if hi else None}")
sys.exit(1 if bad else 0)
