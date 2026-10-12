#!/usr/bin/env bash
# After verify_fix.sh + measure_throttle.sh: analyze run 3, LA3 and the throttle captures.
S=/tmp/claude-1001/-home-peter-Repositories-orb-weaver-qp-arduino/e82c616e-e5ea-4fa0-8354-b513af92835e/scratchpad
R=/home/peter/Repositories/orb-weaver/qp-arduino
B=$R/docs/evidence/2026-09-30-rp2040-blinky
P=$R/docs/evidence/2026-09-30-rp2040-port-validation
Q=$R/.tools/quantum-leaps/qtools-7.3.4/bin/qspy
until grep -q 'throttle measurement done' "$S/measure_throttle.log" 2>/dev/null; do sleep 15; done
{
  echo "##### post-verify start $(date -Is)"
  cp "$S/stress/stress.json" "$P/stress/stress-run3.json"; cp "$S/stress/stress.log" "$P/stress/stress-run3.log"
  python3 "$P/analysis/stress_summary.py" "$P/stress/stress-run3.json" --qspy-logs "$S/stress"
  python3 "$P/analysis/tick_periods.py" "$S/la/pv_stress3_100kHz_900s.vcd" --window "run3-host-cycling:0:870" --window "all:0:1000000" --json "$P/analysis/result_pv_stress3_100kHz_900s.json"
  for c in thr_std_host_1MHz_6s thr_std_nohost_1MHz_6s thr_noqs_1MHz_6s thr_uart_1MHz_6s thr_txonly_1MHz_30s; do
    python3 "$B/analysis/analyze_blinky_capture.py" "$S/la/$c.vcd" > "$B/analysis/result_$c.json" && echo "analyzed $c"
  done
  python3 "$P/analysis/uart_to_qs.py" "$S/pv/txonly_uart_hex.txt" "$S/pv/txonly_qs.bin"
  mkdir -p "$S/qspy-txonly"; (cd "$S/qspy-txonly" && rm -f qspy2*.txt && "$Q" -k -f "$S/pv/txonly_qs.bin" -o -r c0 > qspy-stdout.txt 2>&1; f=$(ls qspy2*.txt | head -1); cp "$f" txonly-offline-decode.txt; echo "txonly decode: $(wc -l < "$f") lines, dict lines $(grep -c 'Dict' "$f"), TRACE_STAT $(grep -c 'TRACE_STAT' "$f"), USER+008 $(grep -c 'USER+008' "$f"), COMMS $(grep -c 'COMMS' "$f")")
  echo "##### post-verify done $(date -Is)"
} > "$S/post_verify.log" 2>&1
echo "##### post-verify done (see post_verify.log) $(date -Is)" >> "$S/measure_throttle.log"
