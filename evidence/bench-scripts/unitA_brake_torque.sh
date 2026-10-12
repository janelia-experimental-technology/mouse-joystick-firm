#!/usr/bin/env bash
# v1.0 unit A (E4654C159F274534) stages A4/A5: brake sink and torque bridge,
# driven non-interactively through examples/BrakeAndTorque over USB CDC, with
# the LA5032 on the Digital-50 taps (adapter v2.0, straight cable: CHn = DIGn).
# Never touches the 36 V supply: that is Peter's hand. Evidence goes to
# qp-arduino-example/evidence/<date>-<hhmm>-unitA-brake-torque/.
#
# Phases (each its own capture; the console's `x` ends the session):
#   0. warm reboot under a capture of CH27 (~BUS_SW_EN) + CH0/CH1: the 34 us
#      boot glitch must be gone after the SafeState fix.
#   1. b   rail up, I2C probe, brake DAC configured, ADS8689 configured
#          (I2C on CH22/23, SPI1 on CH16-19, CAL/FAULT on CH20/21).
#   2. r s S  brake ISENSE at 0, the sweep, the step response.
#   3. t T d k   torque: one, 1000 at 1 kHz, registers, shunt-cal step.
#   4. L 10, X   live 1 kHz loop, brake-to-torque coupling.
#   5. i x   status, power down.
set -u
F=/home/peter/Repositories/orb-weaver/mouse-joystick-firm
X=/home/peter/Repositories/orb-weaver/qp-arduino-example
IC=/home/peter/Repositories/orb-weaver/instrument-control
SERIAL=E4654C159F274534
ENV=${ENV:-mouse_joystick_rev2}
NAME=${NAME:-$(date +%F-%H%M)-unitA-brake-torque}
EV=$X/evidence/$NAME
PORT=
G() { (cd "$IC" && guix shell -m manifest.scm -- "$@"); }
log() { echo "##### $* $(date -Is)" | tee -a "$EV/session.log"; }
find_port() { for i in $(seq 1 600); do p=$(ls /dev/serial/by-id/ 2>/dev/null | command grep -E "$SERIAL" | head -1); [ -n "$p" ] && { PORT=/dev/serial/by-id/$p; return 0; }; sleep 0.1; done; return 1; }
con() { # name read_seconds keys... -> console transcript in consoles/name.log
  local name=$1 secs=$2; shift 2
  (cd "$F" && pixi run python tools/board_console.py "$PORT" "$secs" "$@") 2>&1 | tee -a "$EV/consoles/$name.log"
}
# Digital-50 channels (mouse-joystick-firm/src/BoardPins.hpp, v1.0 table).
CH_BOOT=CH0,CH1,CH24,CH27,CH30,CH31
CH_BUSES=CH0,CH1,CH16,CH17,CH18,CH19,CH20,CH21,CH22,CH23,CH24,CH27
capture() { # name ms rate channels -> timed capture; sigrok-cli writes only on completion, never signal it
  CAPNAME=$1; PHASE_START=$(date +%s); rm -f "$EV/captures/$1.sr"
  G sigrok-cli --driver kingst-la2016 --config samplerate=$3 --time "$2" --channels "$4" -o "$EV/captures/$1.sr" & CAP=$!
  for i in $(seq 1 120); do pgrep -x sigrok-cli >/dev/null && break; sleep 0.5; done
  pgrep -x sigrok-cli >/dev/null || { log "capture $1 did not start"; return 1; }
  sleep 2; log "capture $1 running for $2 ms at $3 on $4"; }
stop_capture() { log "waiting for capture $CAPNAME"; wait $CAP 2>/dev/null
  if [ -s "$EV/captures/$CAPNAME.sr" ]; then log "capture $CAPNAME complete: $(stat -c %s "$EV/captures/$CAPNAME.sr") bytes"
    G sigrok-cli -i "$EV/captures/$CAPNAME.sr" -O vcd -o "$EV/captures/$CAPNAME.vcd" 2>/dev/null
  else log "capture $CAPNAME: NO FILE"; fi; }

mkdir -p "$EV/captures" "$EV/consoles"
log "unit A brake/torque session start"
{ echo "firmware: $(git -C "$F" describe --tags --always --dirty) ($(git -C "$F" rev-parse --short HEAD))"
  echo "build: $ENV, examples/BrakeAndTorque"; echo "board: $SERIAL (v1.0 unit A, board 6)"; } | tee "$EV/meta.txt"
log "warming the guix shell"; G true; log "guix shell ready"
find_port || { log "unit A not on USB"; exit 1; }
log "unit A at $PORT"

if [ "${FLASH:-1}" = 1 ]; then
  log "flashing examples/BrakeAndTorque $ENV"
  (cd "$F" && pixi run upload examples/BrakeAndTorque "$ENV" "$PORT" 2>&1 | command grep -E 'rebooted to start|SUCCESS|FAILED|rror' | tee -a "$EV/session.log")
  sleep 2; find_port || { log "unit A did not come back"; exit 1; }
fi

# ---- phase 0: boot glitch on ~BUS_SW_EN
capture p0_reboot 8000 10m "$CH_BOOT" || exit 1
con p0_reboot 1 "R"; sleep 3; find_port; stop_capture

# ---- phase 1: bring-up
capture p1_bringup 20000 10m "$CH_BUSES" || exit 1
con p1_bringup 6 "b"
con p1_status 3 "i"
stop_capture

# ---- phase 2: brake
capture p2_brake 60000 1m "$CH_BUSES" || exit 1
con p2_brake 2 "r"
con p2_brake 6 "s"
con p2_brake 3 "S"
con p2_brake 3 "S 32768"
con p2_brake 2 "0"
stop_capture

# ---- phase 3: torque
capture p3_torque 30000 10m "$CH_BUSES" || exit 1
con p3_torque 2 "t"
con p3_torque 4 "T 1000"
con p3_torque 2 "d"
con p3_torque 5 "k"
con p3_torque 3 "z"
con p3_torque 2 "t"
stop_capture

# ---- phase 4: live loop and coupling
capture p4_live 45000 10m "$CH_BUSES" || exit 1
con p4_live 14 "L 10"
con p4_live 8 "X"
stop_capture

# ---- phase 5: status and power down
con p5_down 3 "i"
con p5_down 3 "x"
log "session done; the rail is off by x. Transcribe consoles/*.log into mouse-joystick-pcb/docs/bringup-log.md under v1.0 unit A."
