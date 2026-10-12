#!/usr/bin/env bash
# After the stress: re-measure Blinky timing with the throttled transport in
# every variant, plus a transmit-only capture with dictionaries and TRACE_STAT.
S=/tmp/claude-1001/-home-peter-Repositories-orb-weaver-qp-arduino/e82c616e-e5ea-4fa0-8354-b513af92835e/scratchpad
R=/home/peter/Repositories/orb-weaver/qp-arduino
Q=$R/.tools/quantum-leaps/qtools-7.3.4/bin/qspy
IC=/home/peter/Repositories/orb-weaver/instrument-control
PORT=/dev/serial/by-id/usb-Orb_Weaver_qp-arduino-rp2040_E4654C159F2C2534-if00
G() { (cd "$IC" && guix shell -m manifest.scm -- "$@"); }
flash() { (cd "$R" && PIO="$R/.venv/bin/pio" python3 tools/pio_task.py upload --example examples/blinky_bsp-rp2040 --env "$1" --port "$PORT" 2>&1 | grep -E 'rebooted to start|SUCCESS|FAILED|error'); for i in $(seq 1 100); do [ -e "$PORT" ] && break; sleep 0.1; done; sleep 0.5; }
cap() { G sigrok-cli --driver kingst-la2016 --config samplerate=1m --samples "$3" --channels "$2" --triggers CH17=r -o "$S/la/$1.sr" && G sigrok-cli -i "$S/la/$1.sr" -O vcd -o "$S/la/$1.vcd"; echo "##### capture $1 exit=$?"; }
{
  echo "##### throttle measurement start $(date -Is)"
  pkill -x qspy; pkill -x sigrok-cli; sleep 1
  echo "##### standard build, host attached"; flash rp2040_mouse_joystick
  mkdir -p "$S/qspy-thr"; (cd "$S/qspy-thr" && rm -f qspy2*.txt && timeout 20 "$Q" -k -c "$PORT" -b 115200 -o -r c0 > qspy-stdout.txt 2>&1 &); sleep 4
  cap "thr_std_host_1MHz_6s" CH16,CH17 6000000
  while pgrep -x qspy >/dev/null; do sleep 1; done; sleep 2
  echo "##### standard build, host detached"; cap "thr_std_nohost_1MHz_6s" CH16,CH17 6000000
  echo "##### noqs build"; flash rp2040_mouse_joystick_noqs; sleep 3; cap "thr_noqs_1MHz_6s" CH16,CH17 6000000
  echo "##### uart build"; flash rp2040_mouse_joystick_uart; sleep 3; cap "thr_uart_1MHz_6s" CH8,CH16,CH17 6000000
  echo "##### txonly build: 30 s to catch dictionaries (10 s) and TRACE_STAT (1 s)"; flash rp2040_mouse_joystick_txonly; sleep 2
  cap "thr_txonly_1MHz_30s" CH8,CH16,CH17 30000000
  G sigrok-cli -i "$S/la/thr_txonly_1MHz_30s.sr" -P uart:rx=CH8:baudrate=115200:format=hex -A uart=rx-data > "$S/pv/txonly_uart_hex.txt"; echo "##### txonly decode lines=$(wc -l < "$S/pv/txonly_uart_hex.txt")"
  echo "##### restore standard blinky"; flash rp2040_mouse_joystick
  echo "##### throttle measurement done $(date -Is)"
} > "$S/measure_throttle.log" 2>&1
