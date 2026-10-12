#!/usr/bin/env bash
# Follow-up after post_soak.sh: re-run the fixed driver, finer role captures, restore Blinky.
S=/tmp/claude-1001/-home-peter-Repositories-orb-weaver-qp-arduino/e82c616e-e5ea-4fa0-8354-b513af92835e/scratchpad
R=/home/peter/Repositories/orb-weaver/qp-arduino
Q=$R/.tools/quantum-leaps/qtools-7.3.4/bin/qspy
IC=/home/peter/Repositories/orb-weaver/instrument-control
PORT=/dev/serial/by-id/usb-Orb_Weaver_qp-arduino-rp2040_E4654C159F2C2534-if00
G() { (cd "$IC" && guix shell -m manifest.scm -- "$@"); }
flash() { (cd "$R" && PIO="$R/.venv/bin/pio" python3 tools/pio_task.py upload --example "$1" --env "$2" --port "$PORT" 2>&1 | grep -E 'rebooted to start|SUCCESS|FAILED|error'); for i in $(seq 1 100); do [ -e "$PORT" ] && break; sleep 0.1; done; sleep 0.5; }
capfast() { G sigrok-cli --driver kingst-la2016 --config samplerate=20m --samples 10000000 --channels CH16,CH17 --triggers CH17=r -o "$S/la/$1.sr" && G sigrok-cli -i "$S/la/$1.sr" -O vcd -o "$S/la/$1.vcd"; echo "##### capture $1 exit=$?"; }
{
  echo "##### follow-up start $(date -Is)"
  pkill -x qspy; pkill -x sigrok-cli; sleep 1
  echo "##### flash port_validation-rp2040"; flash examples/port_validation-rp2040 rp2040_mouse_joystick
  echo "##### driver procedures (fixed) $(date -Is)"
  (cd "$R" && python3 tools/port_validation_rp2040.py --port "$PORT" --qspy "$Q" --out "$S/pv/results2.json"); echo "##### driver exit=$? $(date -Is)"
  sleep 2
  for role in 1 3; do
    echo "##### role $role capture at 20 MSa/s, host attached $(date -Is)"
    (cd "$R" && python3 tools/port_validation_rp2040.py --port "$PORT" --qspy "$Q" --out "$S/pv/role${role}_fast.json" --only-set-role $role --hold 10) &
    sleep 5
    capfast "pv_role${role}_20MHz_500ms_host"
    wait
  done
  echo "##### liveness check after role sessions $(date -Is)"
  (cd "$R" && python3 tools/port_validation_rp2040.py --port "$PORT" --qspy "$Q" --out "$S/pv/liveness.json" --only-set-role 1 --hold 3); echo "##### liveness exit=$?"
  echo "##### untriggered probe snapshot 2 s $(date -Is)"
  G sigrok-cli --driver kingst-la2016 --config samplerate=1m --samples 2000000 --channels CH16,CH17 -o "$S/la/pv_after_roles_snapshot.sr" && G sigrok-cli -i "$S/la/pv_after_roles_snapshot.sr" -O vcd -o "$S/la/pv_after_roles_snapshot.vcd"
  echo "##### restore standard blinky $(date -Is)"; flash examples/blinky_bsp-rp2040 rp2040_mouse_joystick
  echo "##### follow-up done $(date -Is)"
} > "$S/post_soak2.log" 2>&1
