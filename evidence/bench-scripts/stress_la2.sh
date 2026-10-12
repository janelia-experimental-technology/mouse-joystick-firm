#!/usr/bin/env bash
# Second analyzer capture: starts when the first one has finished, covers the
# rest of stress run 2 (hardware-alarm tick) at 100 kSa/s.
S=/tmp/claude-1001/-home-peter-Repositories-orb-weaver-qp-arduino/e82c616e-e5ea-4fa0-8354-b513af92835e/scratchpad
IC=/home/peter/Repositories/orb-weaver/instrument-control
G() { (cd "$IC" && guix shell -m manifest.scm -- "$@"); }
until grep -q 'vcd exit' "$S/stress_la.log" 2>/dev/null; do sleep 15; done
{
  echo "##### stress LA 2: 100 kSa/s, 1200 s, CH16 CH17 $(date -Is)"
  G sigrok-cli --driver kingst-la2016 --config samplerate=100k --samples 120000000 --channels CH16,CH17 -o "$S/la/pv_stress2_100kHz_1200s.sr"
  echo "##### capture exit=$? $(date -Is)"
  G sigrok-cli -i "$S/la/pv_stress2_100kHz_1200s.sr" -O vcd -o "$S/la/pv_stress2_100kHz_1200s.vcd"
  echo "##### vcd exit=$? $(date -Is)"
} > "$S/stress_la2.log" 2>&1
