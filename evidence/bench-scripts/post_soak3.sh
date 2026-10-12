#!/usr/bin/env bash
# UART trace transport: flash the uart variant, capture GP0 (CH8) with the probes, decode, restore.
S=/tmp/claude-1001/-home-peter-Repositories-orb-weaver-qp-arduino/e82c616e-e5ea-4fa0-8354-b513af92835e/scratchpad
R=/home/peter/Repositories/orb-weaver/qp-arduino
IC=/home/peter/Repositories/orb-weaver/instrument-control
PORT=/dev/serial/by-id/usb-Orb_Weaver_qp-arduino-rp2040_E4654C159F2C2534-if00
G() { (cd "$IC" && guix shell -m manifest.scm -- "$@"); }
flash() { (cd "$R" && PIO="$R/.venv/bin/pio" python3 tools/pio_task.py upload --example "$1" --env "$2" --port "$PORT" 2>&1 | grep -E 'rebooted to start|SUCCESS|FAILED|error'); for i in $(seq 1 100); do [ -e "$PORT" ] && break; sleep 0.1; done; sleep 0.5; }
{
  echo "##### uart start $(date -Is)"
  pkill -x qspy; pkill -x sigrok-cli; sleep 1
  echo "##### flash blinky uart variant"; flash examples/blinky_bsp-rp2040 rp2040_mouse_joystick_uart
  sleep 3
  echo "##### capture CH8 (GP0 UART0 TX), CH16, CH17 at 1 MSa/s for 6 s, trigger CH17 $(date -Is)"
  G sigrok-cli --driver kingst-la2016 --config samplerate=1m --samples 6000000 --channels CH8,CH16,CH17 --triggers CH17=r -o "$S/la/blinky_uart_1MHz_6s.sr"; echo "##### capture exit=$?"
  G sigrok-cli -i "$S/la/blinky_uart_1MHz_6s.sr" -O vcd -o "$S/la/blinky_uart_1MHz_6s.vcd"
  G sigrok-cli -i "$S/la/blinky_uart_1MHz_6s.sr" -P uart:rx=CH8:baudrate=115200:format=hex -A uart=rx-data > "$S/pv/uart_bytes_hex.txt"; echo "##### uart decode exit=$? lines=$(wc -l < "$S/pv/uart_bytes_hex.txt")"
  echo "##### restore standard blinky $(date -Is)"; flash examples/blinky_bsp-rp2040 rp2040_mouse_joystick
  echo "##### uart done $(date -Is)"
} > "$S/post_soak3.log" 2>&1
