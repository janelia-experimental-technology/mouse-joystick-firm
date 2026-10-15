#!/usr/bin/env bash
# Rig 1 reflash and check, 2026-10-15, from Peter's laptop (athena).
# Every step logs into the session's evidence directory. Copy this file into
# <evidence>/scripts/ at the end of the session, verbatim.
#
#   rig1_session.sh boards              list joystick boards and the pyboard (by-id links, USB paths)
#   rig1_session.sh usb                 snapshot of every USB device (vendor:product, path, serial)
#   rig1_session.sh flash <serial>      build+upload the delivery firmware to ONE board, timed
#   rig1_session.sh bootsel             what picotool sees in BOOTSEL (fallback path)
#   rig1_session.sh load <bus> <addr>   picotool load of the built UF2 into a board in BOOTSEL
#   rig1_session.sh probe               `id;1` over Breakout ports 1 and 4 through the pyboard REPL
#   rig1_session.sh trial <left|right> [n_trials] [seconds]
#                                       run the rig's own joystick_trial through pyControl (36 V on!)
set -u
export CONDA_OVERRIDE_GLIBC=2.41
REPO=/home/peter/Repositories/orb-weaver/mouse-joystick-firm
RIG=/home/peter/Repositories/github/joystick-rig-feat-mouse-joystick
export PYCONTROL_DIR=/home/peter/Repositories/github/pycontrol-code
PICOTOOL=$REPO/.platformio/packages/tool-picotool-rp2040-earlephilhower/picotool
UF2=$REPO/.pio/build/examples/PyControlLinkQp/mouse_joystick_qp_rev2/firmware.uf2
EV_FILE=$REPO/evidence/.rig1-session-dir
if [ -f "$EV_FILE" ]; then EV=$(cat "$EV_FILE"); else
  EV=$REPO/evidence/$(date +%Y-%m-%d-%H%M)-rig1-reflash; mkdir -p "$EV/scripts"; echo "$EV" > "$EV_FILE"; fi
LOG=$EV/session.log
cd "$REPO"
say() { printf '%s  %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOG"; }

usb_snapshot() {
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/idVendor" ] || continue
    printf '%s:%s  %-12s  %s  %s %s\n' "$(cat $d/idVendor)" "$(cat $d/idProduct)" "$(basename $d)" \
      "$(cat $d/serial 2>/dev/null)" "$(cat $d/manufacturer 2>/dev/null)" "$(cat $d/product 2>/dev/null)"
  done | grep -E '^(2e8a|f055|1d50|77a1):' || echo "(no RP2040, pyboard or LA5032 on USB)"
}

case "${1:-}" in
  boards)
    say "boards:"; ls -l /dev/serial/by-id/ 2>/dev/null | grep -E 'mouse-joystick|Pyboard' | tee -a "$LOG" || say "(none)"
    usb_snapshot | tee -a "$LOG" ;;
  usb)
    { echo "== usb $(date +%T)"; usb_snapshot; } | tee -a "$EV/usb_snapshots.log" ;;
  flash)
    SERIAL=${2:?serial}; PORT=$(ls /dev/serial/by-id/usb-*_mouse-joystick-pcb_${SERIAL}-if00 2>/dev/null | head -1)
    [ -n "$PORT" ] || { say "no board with serial $SERIAL on USB"; exit 1; }
    N=$(ls /dev/serial/by-id/usb-*_mouse-joystick-pcb_*-if00 2>/dev/null | wc -l)
    say "flash $SERIAL via $PORT ($N joystick board(s) on USB; fw to be $(git describe --tags --always --dirty))"
    git status --short | grep -v '^??' | grep -q . && say "WARNING: tracked files modified, the build will be -dirty"
    T0=$EPOCHREALTIME
    pixi run upload examples/PyControlLinkQp mouse_joystick_qp_rev2 "$PORT" 2>&1 | tee "$EV/upload_${SERIAL}.log" | grep -E 'Waiting|Found|Loading|Verif|OK|SUCCESS|FAILED|Error|error|reboot' 
    RC=${PIPESTATUS[0]}; T1=$EPOCHREALTIME
    say "flash $SERIAL: upload exit $RC in $(awk "BEGIN{printf \"%.1f\", $T1-$T0}") s (full log upload_${SERIAL}.log)"
    sleep 2; ls -l /dev/serial/by-id/ 2>/dev/null | grep "$SERIAL" | tee -a "$LOG" || say "board $SERIAL not yet re-enumerated" ;;
  bootsel)
    say "picotool info -a:"; "$PICOTOOL" info -a 2>&1 | tee -a "$LOG" ;;
  load)
    BUS=${2:?bus}; ADDR=${3:?address}; say "picotool load -v -x $UF2 --bus $BUS --address $ADDR"
    "$PICOTOOL" load -v -x "$UF2" --bus "$BUS" --address "$ADDR" 2>&1 | tee -a "$LOG" ;;
  probe)
    PYB=$(ls /dev/serial/by-id/usb-MicroPython_Pyboard_*-if00 2>/dev/null | head -1)
    [ -n "$PYB" ] || { say "no pyboard on USB"; exit 1; }
    say "id probe through $PYB (nothing else may hold this port)"
    pixi run -e pycontrol python tools/pyb_id_probe.py "$PYB" 2>&1 | tee -a "$EV/uart_id_probe.log" | tee -a "$LOG" ;;
  trial)
    SIDE=${2:?left|right}; NT=${3:-2}; SECS=${4:-60}
    PYB=$(ls /dev/serial/by-id/usb-MicroPython_Pyboard_*-if00 2>/dev/null | head -1)
    [ -n "$PYB" ] || { say "no pyboard on USB"; exit 1; }
    OUT=$EV/trial_${SIDE}_${NT}.json
    say "joystick_trial on $SIDE, n_trials=$NT, up to $SECS s, pyboard $PYB -> $OUT (36 V must be on, nothing on the lever)"
    pixi run -e pycontrol python tools/pycontrol_run.py --port "$PYB" \
      --hwd-path "$RIG/pycontrol/hardware_definitions/hardware_definition_joystick.py" \
      --task-dir "$RIG/pycontrol/tasks/rig1" --task joystick_trial \
      --set board="$SIDE" --set n_trials="$NT" --set watch_timeout_ms=2000 --set iti_ms=1000 \
      --seconds "$SECS" --out "$OUT" 2>&1 | tee "$EV/trial_${SIDE}_${NT}.log"
    say "trial $SIDE: exit ${PIPESTATUS[0]}" ;;
  *) sed -n 2,13p "$0"; echo "evidence dir: $EV" ;;
esac
