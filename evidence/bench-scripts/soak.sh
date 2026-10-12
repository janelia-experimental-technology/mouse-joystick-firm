#!/usr/bin/env bash
# Long hardware runs on the RP2040 fixture with DPP flashed. Detached from the
# agent tool so it survives tool timeouts. Log: $S/soak.log
S=/tmp/claude-1001/-home-peter-Repositories-orb-weaver-qp-arduino/e82c616e-e5ea-4fa0-8354-b513af92835e/scratchpad
L="$S/soak.log"
Q=/home/peter/Repositories/orb-weaver/qp-arduino/.tools/quantum-leaps/qtools-7.3.4/bin/qspy
IC=/home/peter/Repositories/orb-weaver/instrument-control
G() { (cd "$IC" && guix shell -m manifest.scm -- "$@"); }
mkdir -p "$S/qspy-soak"
{
  echo "##### start $(date -Is)"
  (cd "$S/qspy-soak" && rm -f qspy2*.txt && timeout 8100 "$Q" -k -c /dev/ttyACM0 -b 115200 -o -r c0 > qspy-stdout.txt 2>&1) &
  sleep 3
  echo "##### T1: 2 MSa/s, 600 s direct tick capture (1.2 G samples) $(date -Is)"
  G sigrok-cli --driver kingst-la2016 --config samplerate=2m --samples 1200000000 --channels CH16,CH17 -o "$S/la/dpp_2MHz_600s.sr"
  echo "##### T1 exit=$? $(date -Is)"
  G sigrok-cli -i "$S/la/dpp_2MHz_600s.sr" -O vcd -o "$S/la/dpp_2MHz_600s.vcd"
  echo "##### T1 vcd exit=$? $(date -Is)"
  echo "##### T2: 10 kSa/s soak capture, 7200 s $(date -Is)"
  G sigrok-cli --driver kingst-la2016 --config samplerate=10k --samples 72000000 --channels CH16,CH17 -o "$S/la/dpp_10kHz_7200s.sr"
  echo "##### T2 exit=$? $(date -Is)"
  G sigrok-cli -i "$S/la/dpp_10kHz_7200s.sr" -O vcd -o "$S/la/dpp_10kHz_7200s.vcd"
  echo "##### T2 vcd exit=$? $(date -Is)"
  wait
  echo "##### qspy ended $(date -Is)"
  echo "##### done"
} > "$L" 2>&1
