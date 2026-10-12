#!/usr/bin/env bash
S=/tmp/claude-1001/-home-peter-Repositories-orb-weaver-qp-arduino/e82c616e-e5ea-4fa0-8354-b513af92835e/scratchpad
IC=/home/peter/Repositories/orb-weaver/instrument-control
G() { (cd "$IC" && guix shell -m manifest.scm -- "$@"); }
{
  echo "##### stress LA: 100 kSa/s, 3660 s, CH16 CH17 $(date -Is)"
  G sigrok-cli --driver kingst-la2016 --config samplerate=100k --samples 366000000 --channels CH16,CH17 -o "$S/la/pv_stress_100kHz_3660s.sr"
  echo "##### capture exit=$? $(date -Is)"
  G sigrok-cli -i "$S/la/pv_stress_100kHz_3660s.sr" -O vcd -o "$S/la/pv_stress_100kHz_3660s.vcd"
  echo "##### vcd exit=$? $(date -Is)"
} > "$S/stress_la.log" 2>&1
