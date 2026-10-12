#!/usr/bin/env bash
# After the soak: port-validation procedures, role captures, UART trace, restore.
S=/tmp/claude-1001/-home-peter-Repositories-orb-weaver-qp-arduino/e82c616e-e5ea-4fa0-8354-b513af92835e/scratchpad
R=/home/peter/Repositories/orb-weaver/qp-arduino
Q=$R/.tools/quantum-leaps/qtools-7.3.4/bin/qspy
IC=/home/peter/Repositories/orb-weaver/instrument-control
PORT=/dev/serial/by-id/usb-Orb_Weaver_qp-arduino-rp2040_E4654C159F2C2534-if00
BYID=/dev/serial/by-id/usb-Orb_Weaver_qp-arduino-rp2040_E4654C159F2C2534-if00
G() { (cd "$IC" && guix shell -m manifest.scm -- "$@"); }
flash() { (cd "$R" && PIO="$R/.venv/bin/pio" python3 tools/pio_task.py upload --example "$1" --env "$2" --port "$PORT" 2>&1 | grep -E 'rebooted to start|SUCCESS|FAILED|error'); for i in $(seq 1 100); do [ -e "$BYID" ] && break; sleep 0.1; done; sleep 0.5; }
cap() { G sigrok-cli --driver kingst-la2016 --config samplerate=1m --samples 6000000 --channels "$2" --triggers CH17=r -o "$S/la/$1.sr" && G sigrok-cli -i "$S/la/$1.sr" -O vcd -o "$S/la/$1.vcd"; echo "##### capture $1 exit=$?"; }
{
  echo "##### post-soak start (by-id port) $(date -Is)"
  pkill -x qspy; pkill -x sigrok-cli; sleep 1
  echo "##### flash port_validation-rp2040"; flash examples/port_validation-rp2040 rp2040_mouse_joystick
  echo "##### driver procedures $(date -Is)"
  (cd "$R" && python3 tools/port_validation_rp2040.py --port "$PORT" --qspy "$Q" --out "$S/pv/results.json"); echo "##### driver exit=$? $(date -Is)"
  sleep 2
  for role in 1 2 3; do
    echo "##### role $role capture, host attached $(date -Is)"
    (cd "$R" && python3 tools/port_validation_rp2040.py --port "$PORT" --qspy "$Q" --out "$S/pv/role$role.json" --only-set-role $role --hold 14) &
    sleep 5
    cap "pv_role${role}_1MHz_6s_host" CH16,CH17
    wait
  done
  echo "##### role 1 capture, host detached $(date -Is)"
  cap "pv_role1_1MHz_6s_nohost" CH16,CH17
  echo "##### flash blinky uart variant $(date -Is)"; flash examples/blinky_bsp-rp2040 rp2040_mouse_joystick_uart
  sleep 3
  echo "##### uart trace capture (CH8 = GP0 TX) $(date -Is)"
  cap "blinky_uart_1MHz_6s" CH8,CH16,CH17
  G sigrok-cli -i "$S/la/blinky_uart_1MHz_6s.sr" -P uart:rx=CH8:baudrate=115200:format=hex -A uart=rx-data > "$S/pv/uart_bytes_hex.txt"; echo "##### uart decode exit=$? lines=$(wc -l < "$S/pv/uart_bytes_hex.txt")"
  echo "##### restore standard blinky $(date -Is)"; flash examples/blinky_bsp-rp2040 rp2040_mouse_joystick
  echo "##### post-soak done $(date -Is)"
} > "$S/post_soak.log" 2>&1
