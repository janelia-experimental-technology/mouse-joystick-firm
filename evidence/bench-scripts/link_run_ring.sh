#!/usr/bin/env bash
# Fixture run of the firmware's QP bench build with protocol v1 and the capture
# ring (prepared 2026-10-01, to be run only when Peter says so; the 36 V rail
# and board 4 stay off): flash mouse_joystick_qp_sim_usb on the qp-arduino
# fixture, capture QS on FW_TRIG1 (CH17), the operation marker (CH16) and the
# 1 kHz tick probe on the UART0 TX pad (CH8) for 90 s at 1 MSa/s, drive
# tools/link_pilot.py through the full sequence (moves, brake 0, watch, the
# simulator stimulus, fetch, stream on 50 then off, host loss), decode, analyse,
# and write everything under qp-arduino-example/evidence/<date>-rp2040-joystick-ring-capture/
# with the board serial and the firmware version. Blinky is restored at the end.
set -u
DATE=$(date +%F)
X=/home/peter/Repositories/orb-weaver/qp-arduino-example
Q=/home/peter/Repositories/orb-weaver/qp-arduino
F=/home/peter/Repositories/orb-weaver/mouse-joystick-firm
IC=/home/peter/Repositories/orb-weaver/instrument-control
NAME=${NAME:-$DATE-rp2040-joystick-ring-capture} # NAME=... for a rerun into its own directory
EV=$X/evidence/$NAME
QSPY=$Q/.tools/quantum-leaps/qtools-7.3.4/bin/qspy
CYCLES=${CYCLES:-200}
CAPTURE_S=${CAPTURE_S:-90}
PORT=
G() { (cd "$IC" && guix shell -m manifest.scm -- "$@"); }
find_port() { for i in $(seq 1 150); do p=$(ls /dev/serial/by-id/ 2>/dev/null | command grep -E 'qp-arduino-rp2040|mouse-joystick-pcb' | head -1); [ -n "$p" ] && { PORT=/dev/serial/by-id/$p; return 0; }; sleep 0.1; done; return 1; }
flash_firm() { find_port; (cd "$F" && PIXI_PROJECT_ROOT=$F pixi run upload examples/PyControlLinkQp "$1" "$PORT" 2>&1 | command grep -E 'rebooted to start|SUCCESS|FAILED|error'); sleep 1; find_port; sleep 0.5; echo "##### port now $PORT"; }
decode() { # name
  G sigrok-cli -i "$EV/captures/$1.sr" -P uart:rx=CH17:baudrate=115200:format=hex -A uart=rx-data > "$EV/uart/$1_hex.txt"
  echo "##### decode $1 bytes=$(wc -l < "$EV/uart/$1_hex.txt")"
  python3 - "$EV/uart/$1_hex.txt" "$EV/uart/$1_qs.bin" <<'PY'
import sys
out = bytearray()
for line in open(sys.argv[1]):
    parts = line.split()
    if len(parts) >= 2 and parts[0].startswith("uart"):
        out.append(int(parts[1], 16))
open(sys.argv[2], "wb").write(out)
print("bin bytes", len(out))
PY
  mkdir -p "$EV/qspy"
  (cd "$EV/qspy" && rm -f qspy2*.txt && "$QSPY" -k -f "$EV/uart/$1_qs.bin" -o -r c0 > "$1-qspy-stdout.txt" 2>&1
   f=$(ls qspy2*.txt | head -1); mv "$f" "$1-offline-decode.txt"
   echo "##### qspy $1: $(wc -l < "$1-offline-decode.txt") lines, LINK_RX $(command grep -c ' LINK_RX ' "$1-offline-decode.txt"), LINK_TX $(command grep -c ' LINK_TX ' "$1-offline-decode.txt"), SAMPLE_STAT $(command grep -c ' SAMPLE_STAT ' "$1-offline-decode.txt"), LINK_WORK $(command grep -c ' LINK_WORK ' "$1-offline-decode.txt"), TICK_LOST $(command grep -c '^[0-9]\{10\} TICK_LOST' "$1-offline-decode.txt"), Trg-RST $(command grep -c 'Trg-RST' "$1-offline-decode.txt"), ASSERT $(command grep -c 'ASSERT' "$1-offline-decode.txt")")
}
mkdir -p "$EV/captures" "$EV/uart" "$EV/pilot" "$EV/analysis" "$EV/qspy"
{
  echo "##### ring run start $(date -Is)"
  echo "firmware: $(git -C "$F" describe --tags --always --dirty) ($(git -C "$F" rev-parse --short HEAD))" | tee "$EV/meta.txt"
  echo "qp-arduino: $(git -C "$Q" describe --tags --always --dirty) ($(git -C "$Q" rev-parse --short HEAD))" | tee -a "$EV/meta.txt"
  echo "qp-arduino-example: $(git -C "$X" rev-parse --short HEAD)" | tee -a "$EV/meta.txt"
  pkill -x qspy; pkill -x sigrok-cli; sleep 1
  echo "##### flash mouse_joystick_qp_sim_usb"; flash_firm mouse_joystick_qp_sim_usb
  echo "port: $PORT" | tee -a "$EV/meta.txt"
  G sigrok-cli --driver kingst-la2016 --config samplerate=1m --samples $((CAPTURE_S * 1000000)) --channels CH8,CH16,CH17 -o "$EV/captures/ring_sim_usb.sr" &
  CAP=$!; sleep 3
  (cd "$X" && PIXI_PROJECT_ROOT=$X pixi run link-pilot "$PORT" "$EV/pilot/ring_sim_usb.json" "$CYCLES") > "$EV/pilot/ring_sim_usb.txt" 2>&1
  echo "##### pilot exit=$? $(date -Is)"
  python3 - "$EV/pilot/ring_sim_usb.json" >> "$EV/meta.txt" <<'PY'
import json, sys
log = json.load(open(sys.argv[1]))["log"]
ids = [e["text"] for e in log if e["dir"] == "rx" and "serial=" in e["text"]]
print("board: " + (ids[0] if ids else "no id reply in the pilot log"))
PY
  wait $CAP; echo "##### capture exit=$? $(date -Is)"
  G sigrok-cli -i "$EV/captures/ring_sim_usb.sr" -O vcd -o "$EV/captures/ring_sim_usb.vcd"
  decode ring_sim_usb
  python3 "$X/tools/link_trace_analysis.py" --qspy "$EV/qspy/ring_sim_usb-offline-decode.txt" --pilot "$EV/pilot/ring_sim_usb.json" --vcd "$EV/captures/ring_sim_usb.vcd" --tick-us 1000 --json "$EV/analysis/analysis_ring_sim_usb.json" > "$EV/analysis/analysis_ring_sim_usb.txt" 2>&1
  echo "##### analysis exit=$?"
  echo "##### restore standard blinky"
  find_port; (cd "$Q" && PIXI_PROJECT_ROOT=$Q pixi run upload examples/blinky_bsp-rp2040 rp2040_mouse_joystick "$PORT" 2>&1 | command grep -E 'rebooted to start|SUCCESS|FAILED|error')
  echo "##### ring run done $(date -Is)"
} > "$EV/link_run_ring.log" 2>&1
