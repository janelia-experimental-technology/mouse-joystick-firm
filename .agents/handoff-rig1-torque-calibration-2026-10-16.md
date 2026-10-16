# Handoff — Rig 1 reflash to v1.0.1 and torque calibration (2026-10-16)

**From:** the desk-rig session of 2026-10-16 (Claude Code on Peter's desk
machine), for an agent on Peter's laptop at the researcher's rig the same
afternoon or later.
**Authority:** a work brief. It does not override `.agents/AGENTS.md`
("Hardware Sessions" there has the durable rules this brief relies on), the
board repository's electrical facts, or the protocol specification.
**Procedure of record:** `docs/torque-calibration.md`, section "At Rig 1,
from the laptop". This file adds what that document, written for a person,
leaves out for an agent: the state of every board, the laptop's environment,
how to work with Peter at the lever, and what to record afterwards.

## 0. The job, in one paragraph

Rig 1's two joystick boards still run `v1.0.0-6-g4213e48`, which converts
torque on the sensor's nominal 1.5 mV/V. `v1.0.1` (commit `4517a6a`, tagged
and pushed 2026-10-16) converts on the two certificates' mean, 1.5605 mV/V,
so every reported torque is 0.961 of what the same reading gave before; the
torque BNC is analog and does not change. The desk rig's boards took `v1.0.1`
at 13:25 (`evidence/2026-10-16-1323-deskrig-reflash-v1.0.1/`). The job at the
rig is: reflash both boards to `v1.0.1`, confirm it over the RJ-45s, then run
the force-gauge calibration on each station with Peter at the lever, reduce
it, fill the two Rig 1 rows of
`../mouse-joystick-pcb/docs/station-torque-table.csv`, and record the
session. Everything must happen before the researcher collects data, so the
pyControl log and the OneBox trace are on one torque scale from the first
trial.

## 1. State as of 2026-10-16 13:30

| Board | Serial | Station | Breakout port | Firmware now |
| --- | --- | --- | --- | --- |
| 6 (v1.0 unit A) | `E4654C159F274534` | Rig 1 right | 4 (`UART(3)`) | `v1.0.0-6-g4213e48` |
| 7 (v1.0 unit B) | `E4654C159F404534` | Rig 1 left | 1 (`UART(4)`) | `v1.0.0-6-g4213e48` |
| 8 (unit C) | `E4654C159F504534` | desk left | 1 | `v1.0.1` |
| 9 (unit D) | `E4654C159F234234` | desk right | 4 | `v1.0.1` |

Rig 1's pyboard is PYBv1.1 `397734803233` (MicroPython 1.26.1, pyControl
2.1.1). On 2026-10-15 the two boards' USB and the pyboard's USB were moved
from the rig PC to the laptop for the session (USB paths 1-1.1 board 7, 1-1.5
board 6, 1-1.6 the pyboard); the RJ-45s stayed. The rig's 36 V was on. The
researcher's pyControl GUI on the rig PC was closed first. The record of that
visit, including the helper `scripts/rig1_session.sh` that ran every step, is
`evidence/2026-10-15-1318-rig1-reflash/`.

All four boards have pull sign −1 in `src/BoardCalibration.hpp`: a pull reads
positive torque on the right stations and negative on the left.

The researcher's torque constant and the per-station table are documented in
`../mouse-joystick-pcb/docs/bnc-conversion.md`; the desk rows of the table are
filled, the Rig 1 rows are blank.

## 2. The laptop's environment (athena, Debian 13, Guix profile)

- Checkouts on 2026-10-15: `/home/peter/Repositories/orb-weaver/mouse-joystick-firm`,
  `/home/peter/Repositories/orb-weaver/mouse-joystick-pcb`, the rig branch at
  `/home/peter/Repositories/github/joystick-rig-feat-mouse-joystick`, pyControl at
  `/home/peter/Repositories/github/pycontrol-code`. Pull both Orb Weaver
  repositories first and confirm `git describe --tags --dirty` prints
  `v1.0.1` in the firmware checkout with nothing modified: `ID` reports what
  `git describe` says at build time, and a dirty build misnames the code on
  the board.
- `export CONDA_OVERRIDE_GLIBC=2.41` before any `pixi` command (pixi is
  `~/.pixi/bin/pixi`). The first build on the laptop took two minutes
  including the toolchain download on 2026-10-14; it is cached since.
- Serial devices need the Bash sandbox disabled. Find them with
  `ls -l /dev/serial/by-id/`: the boards are
  `usb-Janelia_Research_Campus_mouse-joystick-pcb_<serial>-if00`, the pyboard
  `usb-MicroPython_Pyboard_Virtual_Comm_Port_in_FS_Mode_<serial>-if00`, the
  gauge `usb-Silicon_Labs_MARK-10_USB_Device_3706367-if00-port0`. Boards
  flashed before 2026-10-12 enumerate as `Orb_Weaver` instead; Rig 1's were
  reflashed after, so they do not.
- No `lsusb`, `curl`, `setsid` or terminal emulator in the Guix profile.
  `/usr/bin/xterm` and `/usr/bin/setsid` exist on the desk machine's Debian;
  check the laptop with `ls /usr/bin/xterm /usr/bin/setsid` before relying on
  them. USB facts come from `/sys/bus/usb/devices/*/{idVendor,idProduct,serial,devnum}`.
- `spd-say` is silent on Peter's machines even when it exits 0. Pass
  `--no-speech` to the run tool and lead the run with the cue window.
- The native tests need `ar`, `gcc`, `g++` under plain names: symlink the
  `x86_64-conda-linux-gnu-*` tools from `.pixi/envs/default/bin` into a scratch
  directory on `PATH`. Not needed for the flash.

## 3. Working with Peter at the lever

Peter is the only person; he holds the gauge, and he also has to plug and
unplug things. He cannot read the terminal while pulling.

- Explain what a run will do, ask for a one-word "ready", start on that reply,
  and say when the lever will move (the run tool's `--countdown 10` puts a
  countdown in the cue window before the first move).
- One physical action per message: "plug the pyboard into the laptop", then
  "close the GUI on the rig PC", then "loop the string", each as its own
  request with a wait for his answer.
- The cue window is the channel during a run: launch it detached so it
  survives the shell, with a large font, on the laptop screen, before the run
  starts:

  ```
  /usr/bin/setsid nohup /usr/bin/xterm -fa Monospace -fs 26 -e python3 tools/cue_display.py /tmp/joystick-cues.txt >/dev/null 2>&1 &
  ```

  The run tool writes the cues to that file. Peter prefers the alternating
  protocol (`--mode cycles`, the default): pull to slip, push to slip, keep
  alternating, no pauses, the hold ending on the clock. Keep the slides slow.
- He can remove or refit the pinion before or after a run, never during a
  hold. The gear-out run (`--mode pinion --no-park`) is optional at the rig;
  the string loop with the gear in is the planned measurement.
- Do not stop the tool mid-run to ask something: it pings the board every
  40 ms, and an interrupted hold leaves the brake on and the rail up. If a run
  must end early, let the tool's own ending run (stop, disarm, `hv off`), or
  say so plainly and clear the fault with the next run's `clrfault`.
- Nothing is committed by the agent. Leave the tree ready and suggest the
  messages.

## 4. Steps

1. **Pull and verify** both repositories (section 2). Confirm the two Rig 1
   rows of the station table are still blank.
2. **GUI closed on the rig PC** (ask Peter). **Boards and pyboard on the
   laptop's USB** (ask, one at a time if he prefers). Check with
   `ls -l /dev/serial/by-id/` that both boards, the pyboard and the gauge are
   present. The rig's 36 V may stay on; the flash does not need it, the
   calibration does.
3. **Reflash, one board at a time, by serial:**

   ```
   export CONDA_OVERRIDE_GLIBC=2.41
   pixi run upload examples/PyControlLinkQp mouse_joystick_qp_rev2 /dev/serial/by-id/usb-Janelia_Research_Campus_mouse-joystick-pcb_E4654C159F274534-if00 2>&1 | tee <evidence>/upload_E4654C159F274534.log
   pixi run upload examples/PyControlLinkQp mouse_joystick_qp_rev2 /dev/serial/by-id/usb-Janelia_Research_Campus_mouse-joystick-pcb_E4654C159F404534-if00 2>&1 | tee <evidence>/upload_E4654C159F404534.log
   ```

   Expect "Forcing reset using 1200bps", "Device rebooted into BOOTSEL mode
   successfully", a load, "Verifying Flash" to 100 % and SUCCESS in about
   ten seconds. If picotool stops part way ("The RP2040 device returned an
   error"), the board is in BOOTSEL: `ls /media/$USER/` shows `RPI-RP2`, and
   `cp .pio/build/examples/PyControlLinkQp/mouse_joystick_qp_rev2/firmware.uf2 /media/$USER/RPI-RP2/ && sync`
   finishes it; the application's port is back a second later. If the touch
   itself is ignored (no BOOTSEL), the board has to be put into BOOTSEL by
   hand, which on this board needs both its USB and its pyControl cable
   unplugged first (`docs/rp2040-bringup.md` has the procedure; ask Peter,
   one action at a time); then `picotool load -v -x <uf2> --bus B --address N`
   with the device's `busnum`/`devnum` from sysfs, or the UF2 copy.
   `evidence/2026-10-15-1318-rig1-reflash/scripts/rig1_session.sh` wraps these
   steps (`flash <serial>`, `bootsel`, `load`, `probe`) with timing and logging;
   its paths are the laptop's as of 2026-10-15 and it writes its own evidence
   directory name into `evidence/.rig1-session-dir`.
4. **Read the versions back** through the pyboard, with nothing else on its
   port:

   ```
   python3 tools/pyb_id_probe.py /dev/serial/by-id/usb-MicroPython_Pyboard_Virtual_Comm_Port_in_FS_Mode_397734803233-if00 2>&1 | tee <evidence>/uart_id_probe.log
   ```

   Both lines must say `fw=v1.0.1`, with `E4654C159F404534` on port 1 and
   `E4654C159F274534` on port 4. Do not go on to the calibration on a board
   that does not.
5. **Optional check that the rig's task still runs** on the new build, as on
   2026-10-15: `rig1_session.sh trial right 2` and `trial left 2` (36 V on,
   nothing on the levers; it uses the rig branch checkout and
   `tools/pycontrol_run.py`). Two `NOMOVE` trials per side, no `nak`, no
   fault. The torque values it prints are now on the new scale.
6. **Calibration, per station.** Gauge on the laptop; cue window up; Peter
   loops the string around the lever and measures the radius from the lever
   axis to the loop in millimetres (72 to 73 at the desk); he zeroes the gauge
   with the hook on the string in the pulling direction. Then, after his
   "ready":

   ```
   python3 tools/torque_cal_run.py --board right --radius-mm <r> --out evidence/<date>-<time>-rig1-torque-cal --name board6_right_run1 --countdown 10 --no-speech
   python3 tools/torque_cal_run.py --board left  --radius-mm <r> --out evidence/<date>-<time>-rig1-torque-cal --name board7_left_run1  --countdown 10 --no-speech
   ```

   Each run raises the rail, homes, arms, presents at −5°, and holds at 3.5,
   0, 1.0, 2.0 and 0.5 N·cm for 90 s at full brake and 75 s otherwise, about
   six and a half minutes, then parks, disarms and drops the rail. Watch the
   terminal for `host_timeout` or any `nak`; the desk runs that completed are
   `board9_right_run4` and `board8_left_run1` in
   `evidence/2026-10-16-0951-deskrig-torque-cal/` for comparison.
7. **Reduce** each log:

   ```
   python3 tools/torque_cal_reduce.py --radius-mm <r> evidence/<session>/board6_right_run1.log --json evidence/<session>/board6_right_run1.json
   ```

   The "pull slides across levels" line gives `|sensor/true|` (expect about
   1.00 on `v1.0.1`; 0.96 would mean the board is still on `v1.0.0`) and the
   friction; the per-level lines give the gauge torque while sliding in each
   direction. Fill the station's row of the table: `friction_pull_ncm` and
   `friction_push_ncm` from the brake-off level's slides, `brake_drag_coil_off_ncm`
   from the sensor excursion at level 0 while sliding, `static_share` from the
   static fit if there was one (else leave blank), `delivered_at_*` as the
   gauge torque while sliding minus the friction at each level,
   `rest_offset_ncm` as the baseline reading, `measured_on`, `method`
   ("string loop, gear in"), `evidence`. The desk rows show the format.
8. **Record.** `meta.txt` in the session directory (firmware, host, boards,
   timeline, every run including aborted ones, the radius and the string),
   the scripts that ran copied verbatim into `scripts/`, a row at the top of
   the table in `evidence/README.md`, the numbers under boards 6 and 7 in
   `../mouse-joystick-pcb/docs/bringup-log.md` (their rows in the table near
   the top, plus a short dated section modelled on the 2026-10-16 desk
   section), the firmware `README.md` paragraph that names what each board
   runs (near the top, "The delivered state is tagged"), and the plan items
   in both repositories' `.agents/PLANS.md`. The citable layer rule applies to
   everything but `.agents/`: other repositories are cited as
   `<repository>/<path>`, no `../`, no URL, no Orb Weaver names.

## 5. What not to do

- Do not run the probe or the calibration while pyControl's GUI or runner is
  attached to the pyboard.
- Do not edit `orb-weaver-context`'s rig plan without asking; its Phase C3
  can be marked done by this method afterwards.
- Do not change `tools/torque_cal_run.py`'s timing on site; a wait without
  pings faults the board.
- Do not rely on speech, a terminal emulator from the Guix profile, or the
  `ttyACM` numbers.
