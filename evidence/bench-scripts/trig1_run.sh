#!/usr/bin/env bash
# 2026-10-01: transmit-only QS over a PIO UART on FW_TRIG1 (GP21 -> LA5032 ch 17),
# Blinky on the rev 0 fixture. Tick probe on GP0 (ch 8), LED on GP25 (ch 16).
set -u
S=/tmp/claude-1001/-home-peter-Repositories-orb-weaver-qp-arduino/d382aaa3-ab10-4160-87a8-5da8cf57aa4d/scratchpad
R=/home/peter/Repositories/orb-weaver/qp-arduino
E=$R/docs/evidence/2026-10-01-rp2040-trig1-trace
Q=$R/.tools/quantum-leaps/qtools-7.3.4/bin/qspy
IC=/home/peter/Repositories/orb-weaver/instrument-control
PORT=/dev/serial/by-id/usb-Orb_Weaver_qp-arduino-rp2040_E4654C159F2C2534-if00
G() { (cd "$IC" && guix shell -m manifest.scm -- "$@"); }
flash() {
  (cd "$R" && pixi run upload examples/blinky_bsp-rp2040 "$1" "$PORT" 2>&1 | command grep -E 'rebooted to start|SUCCESS|FAILED|error')
  for i in $(seq 1 100); do [ -e "$PORT" ] && break; sleep 0.1; done; sleep 0.5
}
cap() { # name channels samples trigger
  G sigrok-cli --driver kingst-la2016 --config samplerate=1m --samples "$3" --channels "$2" --triggers "$4" -o "$S/la/$1.sr"
  echo "##### capture $1 exit=$? $(date -Is)"
  G sigrok-cli -i "$S/la/$1.sr" -O vcd -o "$S/la/$1.vcd"; echo "##### vcd $1 exit=$?"
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
  (cd "$S/qspy/$1" && rm -f qspy2*.txt && "$Q" -k -f "$S/uart/$1_qs.bin" -o -r c0 > qspy-stdout.txt 2>&1
   f=$(ls qspy2*.txt | head -1); cp "$f" offline-decode.txt
   echo "##### qspy $1: $(wc -l < "$f") lines, Dict $(command grep -c 'Dict' "$f"), TRACE_STAT $(command grep -c 'TRACE_STAT' "$f"), USER+008 $(command grep -c 'USER+008' "$f"), TICK_LOST $(command grep -c 'TICK_LOST' "$f"), COMMS $(command grep -c 'COMMS' "$f"), Tran $(command grep -c '===>Tran' "$f")")
}
{
  echo "##### trig1 run start $(date -Is)"
  pkill -x qspy; pkill -x sigrok-cli; sleep 1
  echo "##### flash trig1"; flash rp2040_mouse_joystick_trig1; sleep 2
  echo "##### capture 1: 30 s at 1 MSa/s"; cap trig1_1MHz_30s CH8,CH16,CH17 30000000 CH8=r; decode trig1_1MHz_30s
  python3 "$E/analysis/analyze_blinky_capture.py" "$S/la/trig1_1MHz_30s.vcd" --tick CH8 --led CH16 > "$S/la/result_trig1_1MHz_30s.json"; echo "##### analysis 1 exit=$?"
  echo "##### capture 2: 600 s at 1 MSa/s"; cap trig1_1MHz_600s CH8,CH16,CH17 600000000 CH8=r; decode trig1_1MHz_600s
  python3 "$E/analysis/analyze_blinky_capture.py" "$S/la/trig1_1MHz_600s.vcd" --tick CH8 --led CH16 > "$S/la/result_trig1_1MHz_600s.json"; echo "##### analysis 2 exit=$?"
  echo "##### restore standard blinky"; flash rp2040_mouse_joystick
  echo "##### trig1 run done $(date -Is)"
} > "$S/trig1_run.log" 2>&1
