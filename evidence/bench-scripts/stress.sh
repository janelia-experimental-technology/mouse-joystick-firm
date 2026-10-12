#!/usr/bin/env bash
S=/tmp/claude-1001/-home-peter-Repositories-orb-weaver-qp-arduino/e82c616e-e5ea-4fa0-8354-b513af92835e/scratchpad
R=/home/peter/Repositories/orb-weaver/qp-arduino
cd "$R" && python3 tools/port_validation_stress.py --port /dev/serial/by-id/usb-Orb_Weaver_qp-arduino-rp2040_E4654C159F2C2534-if00 --qspy "$R/.tools/quantum-leaps/qtools-7.3.4/bin/qspy" --duration 3600 --out "$S/stress/stress.json" > "$S/stress/stress.log" 2>&1
echo "exit=$?" >> "$S/stress/stress.log"
