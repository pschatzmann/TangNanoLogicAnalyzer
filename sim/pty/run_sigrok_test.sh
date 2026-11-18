#!/usr/bin/env bash
# End-to-end test with the real libsigrok "ols" driver (the one PulseView
# uses): the design runs in Verilator behind a pseudo terminal and is
# driven with sigrok-cli. Usage: sim/pty/run_sigrok_test.sh <build-dir>
set -u
B=$(realpath "${1:-build}")/cosim
HERE=$(dirname "$(realpath "$0")")
SIM="$B/obj/Vvtop"
SHIM="$B/ptyshim.so"
fails=0

start_sim() {   # $1 = cycle at which probe 15 goes high (-1: never)
    "$SIM" "$B/ttyLA" "$1" > "$B/sim.log" 2>&1 &
    SIM_PID=$!
    for _ in $(seq 50); do [ -L "$B/ttyLA" ] && break; sleep 0.1; done
    sleep 0.5
    PORT=$(readlink "$B/ttyLA")
}
stop_sim() { kill "$SIM_PID" 2>/dev/null; wait "$SIM_PID" 2>/dev/null; rm -f "$B/ttyLA"; }
sr() { LD_PRELOAD="$SHIM" timeout 600 sigrok-cli -d "ols:conn=$PORT" "$@"; }
check() {       # name, then check_capture.py arguments
    local name=$1; shift
    if out=$(python3 "$HERE/check_capture.py" "$@"); then echo "ok:   $name: $out" | head -1
    else echo "FAIL: $name: $out"; fails=$((fails + 1)); fi
}

start_sim -1
echo "--- device"
sr --show 2>&1 | grep -E "^ols|Maximum number"
sr --config samplerate=100M:pattern=Internal --samples 20000 -O binary -o "$B/c1.bin"
check "internal pattern 100 MHz" "$B/c1.bin" 4 0xffff 1
sr --config samplerate=25M --samples 10000 -O binary -o "$B/c2.bin"
check "probes 25 MHz (step 4)" "$B/c2.bin" 4 0x7fff 4
sr --channels 0,1,2,3,4,5,6,7 --config samplerate=100M:pattern=Internal --samples 30000 -O binary -o "$B/c3.bin"
check "channels 0-7 only" "$B/c3.bin" 4 0xff 1
stop_sim

# probe 15 rises after 6M clocks: the 4M sample ring buffer has wrapped
for ratio in 50 10; do
    start_sim 6000000
    sr --config samplerate=100M:captureratio=$ratio -t 15=1 --samples 10000 -O binary -o "$B/t$ratio.bin"
    expect=$(python3 -c "r=2500; d=int(r*(1-$ratio/100)); print((r-d)*4-1)")
    got=$(python3 "$HERE/check_capture.py" "$B/t$ratio.bin" 4 0x7fff 1 15 | sed -n 's/.*first high at sample //p')
    if [ "$got" = "$expect" ]; then echo "ok:   trigger after wrap, ratio $ratio%: at sample $got"
    else echo "FAIL: trigger ratio $ratio%: at sample $got, expected $expect"; fails=$((fails + 1)); fi
    stop_sim
done

[ $fails -eq 0 ] && echo "ALL SIGROK TESTS PASSED" || echo "$fails SIGROK TEST(S) FAILED"
exit $fails
