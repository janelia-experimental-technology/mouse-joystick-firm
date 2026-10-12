#!/usr/bin/env bash
# Wait for analyzer capture 2, then: flash the race-fixed validation firmware,
# run a 15-minute reader-cycling stress with a parallel analyzer capture, and
# finally the throttle measurements.
S=/tmp/claude-1001/-home-peter-Repositories-orb-weaver-qp-arduino/e82c616e-e5ea-4fa0-8354-b513af92835e/scratchpad
R=/home/peter/Repositories/orb-weaver/qp-arduino
IC=/home/peter/Repositories/orb-weaver/instrument-control
PORT=/dev/serial/by-id/usb-Orb_Weaver_qp-arduino-rp2040_E4654C159F2C2534-if00
G() { (cd "$IC" && guix shell -m manifest.scm -- "$@"); }
until grep -q 'vcd exit' "$S/stress_la2.log" 2>/dev/null; do sleep 15; done
{
  echo "##### verify start $(date -Is)"
  mv "$S/stress/stress.json" "$S/stress/stress-run2.json" 2>/dev/null; mv "$S/stress/stress.log" "$S/stress/stress-run2.log" 2>/dev/null
  (cd "$R" && PIO="$R/.venv/bin/pio" python3 tools/pio_task.py upload --example examples/port_validation-rp2040 --env rp2040_mouse_joystick --port "$PORT" 2>&1 | grep -E 'rebooted to start|SUCCESS|FAILED|error')
  for i in $(seq 1 100); do [ -e "$PORT" ] && break; sleep 0.1; done; sleep 1
  echo "##### LA3: 100 kSa/s, 900 s $(date -Is)"
  ( G sigrok-cli --driver kingst-la2016 --config samplerate=100k --samples 90000000 --channels CH16,CH17 -o "$S/la/pv_stress3_100kHz_900s.sr"; echo "##### LA3 capture exit=$? $(date -Is)"; G sigrok-cli -i "$S/la/pv_stress3_100kHz_900s.sr" -O vcd -o "$S/la/pv_stress3_100kHz_900s.vcd"; echo "##### LA3 vcd exit=$? $(date -Is)" ) > "$S/stress_la3.log" 2>&1 &
  sleep 3
  echo "##### stress run 3 (race fixed), 870 s $(date -Is)"
  (cd "$R" && python3 tools/port_validation_stress.py --port "$PORT" --qspy "$R/.tools/quantum-leaps/qtools-7.3.4/bin/qspy" --duration 870 --out "$S/stress/stress.json" > "$S/stress/stress.log" 2>&1; echo "exit=$?" >> "$S/stress/stress.log")
  echo "##### stress run 3 done $(date -Is)"
  wait
  echo "##### LA3 done $(date -Is)"
  exec "$S/measure_throttle.sh"
} > "$S/verify_fix.log" 2>&1
