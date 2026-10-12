#!/usr/bin/env bash
# Chain: wait for the second analyzer capture (which itself waits for the first)
# and for the stress driver to finish, then run the throttle measurements.
S=/tmp/claude-1001/-home-peter-Repositories-orb-weaver-qp-arduino/e82c616e-e5ea-4fa0-8354-b513af92835e/scratchpad
until grep -q 'vcd exit' "$S/stress_la2.log" 2>/dev/null; do sleep 20; done
until grep -q 'exit=' "$S/stress/stress.log" 2>/dev/null; do sleep 10; done
sleep 3
exec "$S/measure_throttle.sh"
