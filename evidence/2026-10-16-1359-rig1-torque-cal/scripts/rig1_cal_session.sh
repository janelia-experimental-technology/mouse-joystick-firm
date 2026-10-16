#!/usr/bin/env bash
# Rig 1 torque calibration session, 2026-10-16, from Peter's laptop (athena), after the
# reflash of boards 6 and 7 to v1.0.1 (upload_board*.log, uart_id_probe.log in this directory).
# Every step logs into the session's evidence directory; this file is kept there verbatim.
#
#   rig1_cal_session.sh cue                              open the cue window (xterm, Monospace 26) on /tmp/joystick-cues.txt
#   rig1_cal_session.sh gauge [n]                        n readings from the Mark-10 (scripts/gauge_read.py)
#   rig1_cal_session.sh run <left|right> <radius_mm> <name> [extra torque_cal_run.py args]
#                                                        tools/torque_cal_run.py, cycles mode, 10 s countdown, no speech
#   rig1_cal_session.sh reduce <radius_mm> <name>        tools/torque_cal_reduce.py -> <name>.reduce.txt, <name>.json
set -u
export CONDA_OVERRIDE_GLIBC=2.41
REPO=/home/peter/Repositories/orb-weaver/mouse-joystick-firm
EV=$REPO/evidence/2026-10-16-1359-rig1-torque-cal
PY=$REPO/.pixi/envs/pycontrol/bin/python          # standard library is all the tools need; no system python3 in the Guix shell
PYB=/dev/serial/by-id/usb-MicroPython_Pyboard_Virtual_Comm_Port_in_FS_Mode_397734803233-if00
GAUGE=/dev/serial/by-id/usb-Silicon_Labs_MARK-10_USB_Device_3706367-if00-port0
CUES=/tmp/joystick-cues.txt
LOG=$EV/session.log
cd "$REPO"
say() { printf '%s  %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOG"; }

case "${1:-}" in
  cue)
    : > "$CUES"
    DISPLAY=${DISPLAY:-:0} /usr/bin/setsid nohup /usr/bin/xterm -fa Monospace -fs 26 -e "$PY" tools/cue_display.py "$CUES" >/dev/null 2>&1 &
    sleep 1.5; say "cue window: $(/usr/bin/pgrep -af cue_display || echo NOT RUNNING)" ;;
  gauge)
    "$PY" "$EV/scripts/gauge_read.py" "$GAUGE" "${2:-3}" | tee -a "$LOG" ;;
  run)
    SIDE=${2:?left|right}; R=${3:?radius_mm}; NAME=${4:?name}; shift 4
    say "run $NAME: board=$SIDE radius_mm=$R mode=cycles countdown=10 extra='$*' fw on boards: v1.0.1"
    "$PY" tools/torque_cal_run.py --board "$SIDE" --radius-mm "$R" --out "$EV" --name "$NAME" \
      --countdown 10 --no-speech --cue-file "$CUES" --pyb "$PYB" --gauge "$GAUGE" "$@" 2>&1 | tee "$EV/$NAME.stdout"
    say "run $NAME: exit ${PIPESTATUS[0]}" ;;
  reduce)
    R=${2:?radius_mm}; NAME=${3:?name}
    say "reduce $NAME radius_mm=$R"
    "$PY" tools/torque_cal_reduce.py --radius-mm "$R" "$EV/$NAME.log" --json "$EV/$NAME.json" 2>&1 | tee "$EV/$NAME.reduce.txt" ;;
  *) sed -n 2,12p "$0"; echo "evidence dir: $EV" ;;
esac
