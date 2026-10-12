#!/usr/bin/env bash
# 2026-10-01: the firmware's QP link application (examples/PyControlLinkQp) on
# the rev 0 fixture. Pass A: bench build (simulated motion, protocol over USB
# CDC) driven by the pilot, trace on FW_TRIG1. Pass B: delivery build
# (TmcMotion, UART0) boot sequence only: signature, beacon, trace start.
set -u
S=/tmp/claude-1001/-home-peter-Repositories-orb-weaver-qp-arduino/d382aaa3-ab10-4160-87a8-5da8cf57aa4d/scratchpad
Q=/home/peter/Repositories/orb-weaver/qp-arduino
X=/home/peter/Repositories/orb-weaver/qp-arduino-example
F=/home/peter/Repositories/orb-weaver/mouse-joystick-firm
IC=/home/peter/Repositories/orb-weaver/instrument-control
PORT=/dev/serial/by-id/usb-mouse-joystick-pcb-placeholder
QSPY=$Q/.tools/quantum-leaps/qtools-7.3.4/bin/qspy
G() { (cd "$IC" && guix shell -m manifest.scm -- "$@"); }
find_port() { for i in $(seq 1 150); do p=$(ls /dev/serial/by-id/ 2>/dev/null | command grep -E 'qp-arduino-rp2040|mouse-joystick-pcb' | head -1); [ -n "$p" ] && { PORT=/dev/serial/by-id/$p; return 0; }; sleep 0.1; done; return 1; }
flash_firm() { find_port; (cd "$F" && PIXI_PROJECT_ROOT=$F pixi run upload examples/PyControlLinkQp "$1" "$PORT" 2>&1 | command grep -E 'rebooted to start|SUCCESS|FAILED|error'); sleep 1; find_port; sleep 0.5; echo "##### port now $PORT"; }
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
   echo "##### qspy $1: $(wc -l < "$f") lines, LINK_RX $(command grep -c ' LINK_RX ' "$f"), LINK_TX $(command grep -c ' LINK_TX ' "$f"), LINK_STATE $(command grep -c ' LINK_STATE ' "$f"), OP_START $(command grep -c ' OP_START ' "$f"), OP_DONE $(command grep -c ' OP_DONE ' "$f"), TRACE_STAT $(command grep -c ' TRACE_STAT ' "$f"), LINK_STAT $(command grep -c ' LINK_STAT ' "$f"), TICK_LOST $(command grep -c '^[0-9]\{10\} TICK_LOST' "$f"), Trg-RST $(command grep -c 'Trg-RST' "$f"), ASSERT $(command grep -c 'ASSERT' "$f")")
}
{
  echo "##### firm run start $(date -Is)"
  pkill -x qspy; pkill -x sigrok-cli; sleep 1
  echo "##### pass A: flash mouse_joystick_qp_sim_usb"; flash_firm mouse_joystick_qp_sim_usb
  G sigrok-cli --driver kingst-la2016 --config samplerate=1m --samples 40000000 --channels CH8,CH16,CH17 -o "$S/la/firm_sim_usb2.sr" &
  CAP=$!; sleep 3
  (cd "$X" && PIXI_PROJECT_ROOT=$X pixi run link-pilot "$PORT" "$S/pilot/firm_sim_usb2.json" 200) > "$S/pilot/firm_sim_usb2.txt" 2>&1
  echo "##### pilot firm_sim_usb2 exit=$? $(date -Is)"
  wait $CAP; echo "##### capture firm_sim_usb2 exit=$?"
  G sigrok-cli -i "$S/la/firm_sim_usb2.sr" -O vcd -o "$S/la/firm_sim_usb2.vcd"
  decode firm_sim_usb2
  echo "##### restore standard blinky"
  find_port; (cd "$Q" && PIXI_PROJECT_ROOT=$Q pixi run upload examples/blinky_bsp-rp2040 rp2040_mouse_joystick "$PORT" 2>&1 | command grep -E 'rebooted to start|SUCCESS|FAILED|error')
  echo "##### firm run done $(date -Is)"
} > "$S/link_run_firm2.log" 2>&1
