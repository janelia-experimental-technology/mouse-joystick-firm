#!/usr/bin/env bash
S=/tmp/claude-1001/-home-peter-Repositories-orb-weaver-qp-arduino/e82c616e-e5ea-4fa0-8354-b513af92835e/scratchpad
IC=/home/peter/Repositories/orb-weaver/instrument-control
G() { (cd "$IC" && guix shell -m manifest.scm -- "$@"); }
{
  echo "##### T2b: 100 kSa/s soak capture, 7200 s (720 M samples) $(date -Is)"
  G sigrok-cli --driver kingst-la2016 --config samplerate=100k --samples 720000000 --channels CH16,CH17 -o "$S/la/dpp_100kHz_7200s.sr"
  echo "##### T2b exit=$? $(date -Is)"
  G sigrok-cli -i "$S/la/dpp_100kHz_7200s.sr" -O vcd -o "$S/la/dpp_100kHz_7200s.vcd"
  echo "##### T2b vcd exit=$? $(date -Is)"
} > "$S/soak_la.log" 2>&1
