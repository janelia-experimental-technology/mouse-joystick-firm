#!/usr/bin/env bash
# Follow-up to unitA_brake_torque.sh: the brake step overshoot at several
# amplitudes with ~BRAKE_FAULT sampled during the step, the ALARM register
# re-read, and the reboot capture with the boot signature now emitted.
set -u
F=/home/peter/Repositories/orb-weaver/mouse-joystick-firm
X=/home/peter/Repositories/orb-weaver/qp-arduino-example
IC=/home/peter/Repositories/orb-weaver/instrument-control
SERIAL=E4654C159F274534
NAME=${NAME:-$(date +%F-%H%M)-unitA-brake-steps}
EV=$X/evidence/$NAME
PORT=
G() { (cd "$IC" && guix shell -m manifest.scm -- "$@"); }
log() { echo "##### $* $(date -Is)" | tee -a "$EV/session.log"; }
find_port() { for i in $(seq 1 600); do p=$(ls /dev/serial/by-id/ 2>/dev/null | command grep -E "$SERIAL" | head -1); [ -n "$p" ] && { PORT=/dev/serial/by-id/$p; return 0; }; sleep 0.1; done; return 1; }
con() { local name=$1 secs=$2; shift 2
  (cd "$F" && pixi run python tools/board_console.py "$PORT" "$secs" "$@") 2>&1 | command grep -v 'WARN the lock' | tee -a "$EV/consoles/$name.log"; }
CH_BOOT=CH0,CH1,CH24,CH27,CH30,CH31
CH_BRAKE=CH0,CH1,CH20,CH21,CH22,CH23,CH24,CH27
capture() { CAPNAME=$1; rm -f "$EV/captures/$1.sr"
  G sigrok-cli --driver kingst-la2016 --config samplerate=$3 --time "$2" --channels "$4" -o "$EV/captures/$1.sr" & CAP=$!
  for i in $(seq 1 120); do pgrep -x sigrok-cli >/dev/null && break; sleep 0.5; done
  pgrep -x sigrok-cli >/dev/null || { log "capture $1 did not start"; return 1; }
  sleep 2; log "capture $1 running for $2 ms at $3 on $4"; }
stop_capture() { log "waiting for capture $CAPNAME"; wait $CAP 2>/dev/null
  if [ -s "$EV/captures/$CAPNAME.sr" ]; then log "capture $CAPNAME complete: $(stat -c %s "$EV/captures/$CAPNAME.sr") bytes"
    G sigrok-cli -i "$EV/captures/$CAPNAME.sr" -O vcd -o "$EV/captures/$CAPNAME.vcd" 2>/dev/null
  else log "capture $CAPNAME: NO FILE"; fi; }
mkdir -p "$EV/captures" "$EV/consoles"
log "unit A brake steps session start"
{ echo "firmware: $(git -C "$F" describe --tags --always --dirty) ($(git -C "$F" rev-parse --short HEAD))"
  echo "build: mouse_joystick_rev2, examples/BrakeAndTorque"; echo "board: $SERIAL (v1.0 unit A, board 6)"; } | tee "$EV/meta.txt"
G true; find_port || { log "unit A not on USB"; exit 1; }; log "unit A at $PORT"
capture p0_reboot 8000 10m "$CH_BOOT" || exit 1
con p0_reboot 1 "R" ; sleep 3; find_port; stop_capture
capture p1_steps 70000 1m "$CH_BRAKE" || exit 1
con p1_bringup 6 "b"
con p1_steps 2 "r"
con p1_steps 3 "S 65535"
con p1_steps 3 "S 58981"
con p1_steps 3 "S 49151"
con p1_steps 3 "S 39321"
con p1_steps 3 "S 32768"
con p1_steps 3 "S 65535"
con p1_steps 2 "d"
con p1_steps 2 "d"
con p1_steps 2 "t"
con p1_steps 3 "x"
stop_capture
log "session done; rail off by x."
