#!/usr/bin/env bash
# Board 4 rail-up session (prepared 2026-10-02; run only with Peter at the
# bench). Five phases with explicit stop points; the script never continues
# past a stop without a typed "yes". It never touches the 36 V supply: that is
# Peter's hand. Evidence goes to qp-arduino-example/evidence/<date>-board4-rail-up/
# with the firmware hash and the board serial in meta.txt; the electrical
# numbers are transcribed afterwards into mouse-joystick-pcb/docs/bringup-log.md
# under board 4 (see board4-session.md).
#
#   a. board 4 (E4654C159F171E34, RIGHT joystick, v0.3) on the LA adapter in
#      place of the fixture, RJ-45 unplugged, 36 V off; flash
#      mouse_joystick_qp_rev1_usb; identity check.      STOP: wiring, current limit
#   b. trace running: hv on, the DAC bring-up phases (DAC_STATE records),
#      meter on the angle BNC unhomed, then after home + angscale at three
#      commanded angles against the encoder.             STOP
#   c. the pilot with real motion: home, 20 moves, brake 0, watch, a hand
#      pull, PULL, TRIAL, moveto 0 with the resync, FETCH with lost=,
#      stream on 50 and off, host loss.                  STOP
#   d. 90 s capture: 300 moves with the rail up for SAMPLE_STAT / LINK_WORK;
#      scope on the BNC during the moves for the 1 kHz staircase.
#   e. hv off under a short capture; DAC_STATE down in the trace; meter 0 V.
set -u
DATE=$(date +%F)
X=/home/peter/Repositories/orb-weaver/qp-arduino-example
Q=/home/peter/Repositories/orb-weaver/qp-arduino
F=/home/peter/Repositories/orb-weaver/mouse-joystick-firm
IC=/home/peter/Repositories/orb-weaver/instrument-control
NAME=${NAME:-$DATE-$(date +%H%M)-board4-rail-up} # every attempt gets its own directory
EV=$X/evidence/$NAME
QSPY=$Q/.tools/quantum-leaps/qtools-7.3.4/bin/qspy
SERIAL=E4654C159F171E34
ENV=${ENV:-mouse_joystick_qp_rev1_usb} # ENV=mouse_joystick_qp_rev0_usb for the bisect run
HWREV=rev1; case "$ENV" in *rev0*) HWREV=rev0;; esac # what ID must report for this build
case "$ENV" in *rev0*) export MJ_EXPECT_HW=rev0 ;; *) export MJ_EXPECT_HW=rev1 ;; esac # what check-id expects from ID
PORT=
G() { (cd "$IC" && guix shell -m manifest.scm -- "$@"); }
log() { echo "##### $* $(date -Is)" | tee -a "$EV/session.log"; }
stop() { # a stop point: Peter types yes to continue, anything else ends the session here
  usb_snapshot
  echo; echo "========== STOP: $1"; echo "$2"; echo
  read -r -p "Type yes to continue, anything else to end the session here: " answer
  if [ "$answer" != "yes" ]; then log "session ended by Peter at stop: $1"; exit 0; fi
  log "continue past stop: $1"
}
find_port() { # board 4 by serial, whatever firmware it runs; waits up to 120 s (30 s was too short at the bench, 2026-10-02)
  for i in $(seq 1 1200); do p=$(ls /dev/serial/by-id/ 2>/dev/null | command grep -E "$SERIAL" | head -1); [ -n "$p" ] && { PORT=/dev/serial/by-id/$p; return 0; }; sleep 0.1; done; return 1; }
# Channels on the adapter (mouse-joystick-firm/README.md, "Watching it with the LA5032"):
# CH2 TMC_SPI_SCK, CH3 COPI, CH4 CIPO, CH5 ~TMC_CS (through the bus switch: live only with the
# rail up), CH8 tick probe, CH16 FW_TRIG0 (TICK handler level, breadcrumbs), CH17 FW_TRIG1 (trace).
CH_BASE=CH8,CH16,CH17
CH_SPI=CH2,CH3,CH4,CH5,CH8,CH16,CH17
SPI_ANALYSIS="--spi CH5 CH2"
capture() { # name ms [rate] [channels] -> timed capture (--time); returns once sigrok-cli is actually running, plus 2 s.
  # This sigrok-cli writes its file only when a capture completes on its own: a capture stopped
  # by SIGINT writes nothing (tested 2026-10-02, continuous and --samples alike), which is how the
  # first two captures of that day were lost. So: a fixed --time per phase and never a signal.
  CAPNAME=$1; CAPTURE_LOST=0; PHASE_START=$(date +%s); CAPRATE=${3:-1m}; CAPCH=${4:-$CH_BASE}
  rm -f "$EV/captures/$1.sr"
  G sigrok-cli --driver kingst-la2016 --config samplerate=$CAPRATE --time "$2" --channels "$CAPCH" -o "$EV/captures/$1.sr" & CAP=$!
  for i in $(seq 1 120); do pgrep -x sigrok-cli >/dev/null && break; sleep 0.5; done
  if ! pgrep -x sigrok-cli >/dev/null; then log "capture $1 did not start"; return 1; fi
  sleep 2; log "capture $1 running for $2 ms at $CAPRATE on $CAPCH"; }
stop_capture() { # wait for sigrok-cli to finish on its own, then check the file is this phase's
  log "waiting for capture $CAPNAME to complete"; wait $CAP 2>/dev/null
  if [ -s "$EV/captures/$CAPNAME.sr" ] && [ "$(stat -c %Y "$EV/captures/$CAPNAME.sr")" -ge "$PHASE_START" ]; then
    log "capture $CAPNAME complete: $(stat -c %s "$EV/captures/$CAPNAME.sr") bytes"
  else
    CAPTURE_LOST=1; log "capture $CAPNAME: no file from this phase (sigrok-cli exited without writing)"
  fi; }
boot_report() { # name: re-read ID and log boot= and crumb= (the last breadcrumb survives the watchdog reboot)
  local line=""
  for i in 1 2 3; do
    find_port && steps "$1" check-id --expect-hw "$HWREV" >/dev/null 2>&1
    line=$(command grep -o 'boot=[a-z]*;crumb=[0-9]*' "$EV/steps/$1.txt" 2>/dev/null | head -1)
    [ -n "$line" ] && break; sleep 2
  done
  log "boot report after the failure: ${line:-no ID reply}"
  echo "========== ID says: ${line:-no ID reply} (boot=wdt means the watchdog rebooted a stuck handler; crumb is the last breadcrumb before it)"; }
fail_phase() { # phase reason: never kill the capture; wait for it, decode, report, stop here
  log "phase $1 FAILED: $2"; stop_capture
  if [ "$1" = b ]; then boot_report b_fail; fi
  decode "$CAPNAME"
  stop "phase $1 failed" "$2. The boot report above names the last breadcrumb; the capture is decoded if it completed (DAC_STATE lines above, breadcrumbs on CH16). Type yes only to go on to the next phase regardless."; }
usb_snapshot() { # lsusb device numbers of the LA5032 and board 4, for the record
  local la board="" d
  la=$(lsusb -d 77a1:01a2 2>/dev/null | head -1 | sed 's/ ID.*//')
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/serial" ] && [ "$(cat "$d/serial" 2>/dev/null)" = "$SERIAL" ] && board="Bus $(cat "$d/busnum") Device $(cat "$d/devnum") $(cat "$d/product" 2>/dev/null)"
  done
  log "usb: LA5032 [${la:-absent}]; board 4 [${board:-absent}]"; }
decode() { # name; refuses a lost capture or a file older than the phase it belongs to
  if [ "${CAPTURE_LOST:-0}" = 1 ]; then log "decode $1: refused, no capture file from this phase"; return 1; fi
  [ -s "$EV/captures/$1.sr" ] || { log "decode $1: no capture file"; return 1; }
  if [ "$(stat -c %Y "$EV/captures/$1.sr")" -lt "${PHASE_START:-0}" ]; then log "decode $1: refused, the file predates this phase"; return 1; fi
  G sigrok-cli -i "$EV/captures/$1.sr" -O vcd -o "$EV/captures/$1.vcd"
  G sigrok-cli -i "$EV/captures/$1.sr" -P uart:rx=CH17:baudrate=115200:format=hex -A uart=rx-data > "$EV/uart/$1_hex.txt"
  python3 - "$EV/uart/$1_hex.txt" "$EV/uart/$1_qs.bin" <<'PY'
import sys
out = bytearray()
for line in open(sys.argv[1]):
    parts = line.split()
    if len(parts) >= 2 and parts[0].startswith("uart"):
        out.append(int(parts[1], 16))
open(sys.argv[2], "wb").write(out)
PY
  (cd "$EV/qspy" && rm -f qspy2*.txt && "$QSPY" -k -f "$EV/uart/$1_qs.bin" -o -r c0 > "$1-qspy-stdout.txt" 2>&1; f=$(ls qspy2*.txt | head -1); mv "$f" "$1-offline-decode.txt")
  log "decode $1: $(wc -l < "$EV/qspy/$1-offline-decode.txt") records, DAC_STATE $(command grep -c ' DAC_STATE ' "$EV/qspy/$1-offline-decode.txt"), SAMPLE_STAT $(command grep -c ' SAMPLE_STAT ' "$EV/qspy/$1-offline-decode.txt"), ASSERT $(command grep -c 'ASSERT' "$EV/qspy/$1-offline-decode.txt")"
  command grep ' DAC_STATE ' "$EV/qspy/$1-offline-decode.txt" | tee -a "$EV/session.log"
}
pilot() { # name args...   (absolute paths and an explicit manifest: the cwd is not relied on); returns the tool's exit code
  local name=$1; shift
  PIXI_PROJECT_ROOT=$X pixi run --manifest-path "$X/pixi.toml" python "$X/tools/link_pilot.py" --port "$PORT" --out "$EV/pilot/$name.json" "$@" 2>&1 | tee "$EV/pilot/$name.txt"
  return "${PIPESTATUS[0]}"
}
steps() { # name step args...; returns the tool's exit code
  local name=$1; shift
  PIXI_PROJECT_ROOT=$X pixi run --manifest-path "$X/pixi.toml" python "$X/tools/board4_steps.py" "$@" --port "$PORT" --out "$EV/steps/$name.json" 2>&1 | tee "$EV/steps/$name.txt"
  return "${PIPESTATUS[0]}"
}
mkdir -p "$EV/captures" "$EV/uart" "$EV/qspy" "$EV/pilot" "$EV/steps" "$EV/analysis"
log "board 4 session start"; usb_snapshot
{ echo "firmware: $(git -C "$F" describe --tags --always --dirty) ($(git -C "$F" rev-parse --short HEAD))"
  echo "qp-arduino: $(git -C "$Q" describe --tags --always --dirty) ($(git -C "$Q" rev-parse --short HEAD))"
  echo "qp-arduino-example: $(git -C "$X" rev-parse --short HEAD)"
  echo "build: $ENV"; echo "board: $SERIAL (board 4, RIGHT joystick, pcb v0.3)"; } | tee "$EV/meta.txt"
read -r -p "USB topology (board 4 port / LA5032 port / hub, powered?): " topo; echo "usb topology: $topo" >> "$EV/meta.txt"
read -r -p "36 V supply grounding (chassis/negative to earth? LA ground reference?): " gnd; echo "supply grounding: $gnd" >> "$EV/meta.txt"

# Warm the guix shell now, so the first capture starts within seconds rather
# than after a profile build (the 2026-10-02 session's capture began 8 s after
# the command it was meant to see).
log "warming the guix shell for sigrok-cli"; G true; log "guix shell ready"
echo "Build to flash: $ENV. Breadcrumbs on FW_TRIG0 (CH16): 25 us pulse trains, count = checkpoint (1-8 rail-up, 9 hv ack queued, 11-17 DAC); the watchdog reboots a stuck handler after 2 s and the next boot reports the last breadcrumb."

# ---------------------------------------------------------------- phase a
stop "a1: hardware" "Put board 4 ($SERIAL) on the LA adapter in place of the fixture. RJ-45 UNPLUGGED. 36 V OFF. USB connected. Type yes when it enumerates."
find_port || { log "board 4 not found on USB"; exit 1; }
log "board 4 at $PORT"
(cd "$F" && PIXI_PROJECT_ROOT=$F pixi run --manifest-path "$F/pixi.toml" upload examples/PyControlLinkQp "$ENV" "$PORT" 2>&1 | command grep -E 'rebooted to start|SUCCESS|FAILED|error' | tee -a "$EV/session.log")
sleep 1; find_port; sleep 1; echo "port: $PORT" >> "$EV/meta.txt"
steps check-id check-id --expect-hw "$HWREV" || { log "identity check failed"; exit 1; }
stop "a2: wiring and supply" "Confirm: joystick and encoder cabled, RJ-45 unplugged, LA adapter channels CH8/CH16/CH17 on the probe pads, the 36 V supply connected but OFF with its current limit set. Type yes to go to phase b (the script will send hv on after you switch the supply on)."

# ---------------------------------------------------------------- phase b
read -r -p "Switch the 36 V supply ON now, then type yes: " a; [ "$a" = "yes" ] || { log "ended before hv on"; exit 0; }
capture b_angle 90000 10m "$CH_SPI" || exit 1
if ! steps b_angle angle-check; then fail_phase b "angle-check did not complete"; else stop_capture; decode b_angle; fi
python3 "$X/tools/link_trace_analysis.py" --qspy "$EV/qspy/b_angle-offline-decode.txt" --vcd "$EV/captures/b_angle.vcd" --tick-us 1000 $SPI_ANALYSIS --json "$EV/analysis/analysis_b_angle.json" > "$EV/analysis/analysis_b_angle.txt" 2>&1
command grep -A8 '"spi"' "$EV/analysis/analysis_b_angle.txt" | head -12
stop "b: angle output" "Readings are in $EV/steps/b_angle.json; the DAC phases are above. Type yes to run the pilot with real motion (phase c)."

# ---------------------------------------------------------------- phase c
capture c_pilot 180000 || exit 1
if ! pilot c_pilot --cycles 20 --stimulus hand --hand-timeout-s 60 --home-timeout-s 20 --keep-hv; then fail_phase c "the pilot aborted"; else stop_capture; decode c_pilot; fi
python3 "$X/tools/link_trace_analysis.py" --qspy "$EV/qspy/c_pilot-offline-decode.txt" --pilot "$EV/pilot/c_pilot.json" --vcd "$EV/captures/c_pilot.vcd" --tick-us 1000 --json "$EV/analysis/analysis_c_pilot.json" > "$EV/analysis/analysis_c_pilot.txt" 2>&1
stop "c: pilot with real motion" "Step list in $EV/pilot/c_pilot.txt (note: the host-loss block dropped the rail; the pilot re-asserted hv in the trial block only if it ran first). Type yes for the 90 s SAMPLE_STAT run (phase d); put the SCOPE on the angle BNC first."

# ---------------------------------------------------------------- phase d
capture d_budget 90000 10m "$CH_SPI" || exit 1
if ! pilot d_budget --cycles 300 --no-trial --no-host-loss --home-timeout-s 20 --keep-hv; then fail_phase d "the pilot aborted"; else stop_capture; decode d_budget; fi
python3 "$X/tools/link_trace_analysis.py" --qspy "$EV/qspy/d_budget-offline-decode.txt" --pilot "$EV/pilot/d_budget.json" --vcd "$EV/captures/d_budget.vcd" --tick-us 1000 $SPI_ANALYSIS --json "$EV/analysis/analysis_d_budget.json" > "$EV/analysis/analysis_d_budget.txt" 2>&1
command grep -A8 '"spi"' "$EV/analysis/analysis_d_budget.txt" | head -12
command grep -A3 '"sample_stat"' "$EV/analysis/analysis_d_budget.txt" | head -5

# ---------------------------------------------------------------- phase e
capture e_hvoff 20000 || exit 1
if ! steps e_hvoff hv-off; then fail_phase e "hv-off did not complete"; else stop_capture; decode e_hvoff; fi
log "session done; switch the 36 V supply OFF. Transcribe steps/b_angle.json and steps/e_hvoff.json into bringup-log.md under board 4."
