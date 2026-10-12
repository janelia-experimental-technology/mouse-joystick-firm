#!/usr/bin/env bash
# 2026-10-01: joystick link pilot on the rev 0 fixture. Protocol over USB CDC
# (tools/link_pilot.py), QS trace on the PIO UART at GP21 (LA5032 ch 17),
# operation marker GP25 (ch 16), tick probe GP0 (ch 8). Observation trace set,
# then the full set; standard Blinky restored at the end.
set -u
S=/tmp/claude-1001/-home-peter-Repositories-orb-weaver-qp-arduino/d382aaa3-ab10-4160-87a8-5da8cf57aa4d/scratchpad
Q=/home/peter/Repositories/orb-weaver/qp-arduino
X=/home/peter/Repositories/orb-weaver/qp-arduino-example
IC=/home/peter/Repositories/orb-weaver/instrument-control
PORT=/dev/serial/by-id/usb-Orb_Weaver_qp-arduino-rp2040_E4654C159F2C2534-if00
QSPY=$Q/.tools/quantum-leaps/qtools-7.3.4/bin/qspy
G() { (cd "$IC" && guix shell -m manifest.scm -- "$@"); }
flash() {
  (cd "$X" && PIXI_PROJECT_ROOT=$X PLATFORMIO_CORE_DIR=$Q/.pio-core pixi run upload rp2040/joystick_link_bsp-rp2040 "$1" "$PORT" 2>&1 | command grep -E 'rebooted to start|SUCCESS|FAILED|error')
  for i in $(seq 1 100); do [ -e "$PORT" ] && break; sleep 0.1; done; sleep 0.5
}
decode() { # name
  G sigrok-cli -i "$S/la/$1.sr" -P uart:rx=CH17:baudrate=115200:format=hex -A uart=rx-data > "$S/uart/$1_hex.txt"
  echo "##### decode $1 bytes=$(wc -l < "$S/uart/$1_hex.txt")"
  python3 - "$S/uart/$1_hex.txt" "$S/uart/$1_qs.bin" <<'PY'
import sys
out = bytearray()
for line in open(sys.argv[1]):
    parts = line.split()
    if len(parts) >= 2 and parts[0].startswith("uart"):
        out.append(int(parts[1], 16))
open(sys.argv[2], "wb").write(out)
print("bin bytes", len(out))
PY
  mkdir -p "$S/qspy/$1"
  (cd "$S/qspy/$1" && rm -f qspy2*.txt && "$QSPY" -k -f "$S/uart/$1_qs.bin" -o -r c0 > qspy-stdout.txt 2>&1
   f=$(ls qspy2*.txt | head -1); cp "$f" offline-decode.txt
   echo "##### qspy $1: $(wc -l < "$f") lines, LINK_RX $(command grep -c ' LINK_RX ' "$f"), LINK_TX $(command grep -c ' LINK_TX ' "$f"), LINK_STATE $(command grep -c ' LINK_STATE ' "$f"), OP_START $(command grep -c ' OP_START ' "$f"), OP_DONE $(command grep -c ' OP_DONE ' "$f"), TRACE_STAT $(command grep -c ' TRACE_STAT ' "$f"), LINK_STAT $(command grep -c ' LINK_STAT ' "$f"), TICK_LOST $(command grep -c '^[0-9]\{10\} TICK_LOST' "$f"), Tran $(command grep -c '===>Tran' "$f"), USER+ $(command grep -c 'USER+' "$f")")
}
run_profile() { # name env cycles samples
  echo "##### $1: flash $2 $(date -Is)"; flash "$2"
  G sigrok-cli --driver kingst-la2016 --config samplerate=1m --samples "$4" --channels CH8,CH16,CH17 --triggers CH8=r -o "$S/la/$1.sr" &
  CAP_PID=$!
  sleep 3
  (cd "$X" && PIXI_PROJECT_ROOT=$X pixi run link-pilot "$PORT" "$S/pilot/$1.json" "$3") > "$S/pilot/$1.txt" 2>&1
  echo "##### pilot $1 exit=$? $(date -Is)"
  wait $CAP_PID; echo "##### capture $1 exit=$? $(date -Is)"
  G sigrok-cli -i "$S/la/$1.sr" -O vcd -o "$S/la/$1.vcd"; echo "##### vcd $1 exit=$?"
  decode "$1"
}
{
  echo "##### link run start $(date -Is)"
  pkill -x qspy; pkill -x sigrok-cli; sleep 1
  run_profile link_obs rp2040_mouse_joystick_trig1 200 40000000
  run_profile link_full rp2040_mouse_joystick_trig1_full 200 40000000
  echo "##### restore standard blinky"
  (cd "$Q" && PIXI_PROJECT_ROOT=$Q pixi run upload examples/blinky_bsp-rp2040 rp2040_mouse_joystick "$PORT" 2>&1 | command grep -E 'rebooted to start|SUCCESS|FAILED|error')
  echo "##### link run done $(date -Is)"
} > "$S/link_run.log" 2>&1
