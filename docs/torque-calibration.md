# Torque Calibration Against A Force Gauge

How a station's torque reading is checked against a known torque, and how its
friction and its brake's delivered torque are measured, with the tools in
`tools/`. First done on the desk rig on 2026-10-16; the numbers are in
`mouse-joystick-pcb/docs/bringup-log.md` under that date and the raw logs in
`evidence/2026-10-16-0951-deskrig-torque-cal/`.

## What is being measured, and what the sensor can see

The reaction torque sensor sits behind the brake. Whatever the lever does, the
sensor reads only the torque that passes through the brake to its housing.
The gear train, the motor and the lever bearings all act on the lever's side,
so their friction is paid by whoever moves the lever and never reaches the
sensor. While the brake slips and the lever slides at a steady few degrees per
second, that friction is kinetic and repeatable, and

```
gauge force × radius  =  torque through the brake  +  friction
sensor reading        =  a × torque through the brake
```

With the brake's coil off the torque through it is only its drag, so the
brake-off slides give the friction directly. `a` is the quantity a
calibration is after: the sensor's reading on the firmware's scale over the
true torque. Firmware from 2026-10-16 on converts on the sensors' certificate
mean, 1.5605 mV/V, so `a` should come out at 1.00; the `v1.0.0` builds
converted on the nominal 1.5 mV/V, where the certificates put `a` at
1.5 / 1.5605 = 0.961, and the 2026-10-16 desk measurements were made on such
a build. The `ID` reply names the build on the board.

Static holds below slip are not a calibration regime. Static friction takes an
indeterminate share of the applied torque, and both desk stations read 0.77 to
0.88 of the gauge there. Only the slides count.

## Equipment

- A Mark-10 Series 3 gauge (the lab's is an M3-05, 250 gF × 0.2 gF, ±0.3 % of
  full scale). Its USB port is a Silicon Labs serial device; it answers `?`
  at 9600 baud with a line such as `-12.4 gF`. A pull on the hook reads
  negative. Zero it with the hook or string attached, in the pulling
  orientation.
- A loop of thread or line around the lever at a marked radius, with the
  gauge hooked to the loop and held 20 to 30 cm away. The string aligns
  itself to the pull; a hook straight on the lever, held by hand, read about
  10 % high on 2026-10-16 from pulls that were not quite perpendicular.
  Measure the radius from the lever axis to the loop to the millimetre.
- The station's pyboard on USB with no pyControl runner attached (close the
  GUI), the board on its delivery firmware. Nothing is reflashed.
- A terminal with a large font for the cue window, on a screen visible from
  the lever.

## Running it

```
python3 tools/cue_display.py /tmp/joystick-cues.txt         # in an xterm with -fa Monospace -fs 26
python3 tools/torque_cal_run.py --board right --radius-mm 72 --out evidence/<date>-<rig>-torque-cal \
        --name board9_right_run1 --countdown 10
```

`--board left` is Breakout port 1 (`UART(4)`), `--board right` port 4
(`UART(3)`). The script finds the pyboard and the gauge by their `by-id`
names, or takes `--pyb` and `--gauge`. It raises the rail, homes, arms,
presents at −5°, turns the 50 Hz stream on and then, for each level in
`--levels` (default 3.5, 0, 1.0, 2.0, 0.5 N·cm), engages the brake, releases
the drive and holds while the cue window tells the person at the lever what to
do; when they take their hands off it moves the lever back and goes on. It
pings the board every 40 ms throughout, because the host watchdog is 100 ms
and a 500 ms pause without a ping faults the board. Every line is logged with
a host timestamp: `B` board traffic with the pyboard's millisecond clock, `G`
gauge readings, `C` cues, `H` notes.

The default `--mode cycles` is the alternating protocol: at every brake
level the window asks for one thing for a fixed time, 90 s at full brake and
75 s otherwise, and the hold ends on the clock, so the person never has to
take their hands off to signal.

- **Brake on.** Pull slowly until the lever slips and slide it a little, then
  push slowly until it slips and slide back, and keep alternating. These give
  the sensor check (at full brake) and the brake's delivered torque (at
  every level).
- **Brake off.** Slide the free lever slowly a few degrees one way, then the
  other, back and forth. This is the friction measurement.

Keep the slides slow, a few degrees per second, and away from the hard stop.
`--mode normal` is the older scripted sequence of ramps and slides; its
static phases turned out not to be a calibration regime and it is kept for
the record of the first runs.

## At Rig 1, from the laptop

1. Close the researcher's pyControl GUI; nothing else may hold the pyboard's
   port.
2. Reflash both boards to the current firmware first, as on 2026-10-15
   (`pixi run upload examples/PyControlLinkQp mouse_joystick_qp_rev2 <board
   port>`, one board at a time, then `tools/pyb_id_probe.py <pyboard tty>` to
   read `fw=` over both RJ-45s): the torque scale changed on 2026-10-16 and
   the check below expects 1.00 on the new build, 0.96 on `v1.0.0`.
3. Plug the gauge into the laptop; it appears as
   `/dev/serial/by-id/usb-Silicon_Labs_MARK-10_*`, and the rig's pyboard as
   `usb-MicroPython_Pyboard_*`. The script finds both; pass `--pyb` or
   `--gauge` if the laptop shows more than one of either.
4. Open the cue window on the laptop screen:
   `xterm -fa Monospace -fs 26 -e python3 tools/cue_display.py /tmp/joystick-cues.txt &`
5. Loop the string on the lever, measure its radius from the lever axis,
   zero the gauge with the hook on the string in the pulling direction.
6. `python3 tools/torque_cal_run.py --board right --radius-mm <r> --out
   evidence/<date>-<time>-rig1-torque-cal --name board6_right_run1
   --countdown 10`, then the same with `--board left` and `board7_left_run1`
   at the left lever. About six minutes each.
7. Reduce each log with `tools/torque_cal_reduce.py --radius-mm <r> <log>
   --json <log>.json` and fill the station's row of
   `mouse-joystick-pcb/docs/station-torque-table.csv`: the friction for each
   direction from the brake-off level, the delivered torque from `gauge −
   friction` at each level, the sensor-over-brake ratio at full brake as the
   check (expect about 1.00 on the current firmware, 0.96 on a `v1.0.0`
   build; a hand-held hook read 5 to 15 % lower than that at the desk, see
   the note on geometry above), and the rest reading. Record the session in the bring-up log under both
   serials and in `evidence/README.md`.

### With the gear out

Removing the pinion takes the gear train out of the friction and makes the
cleanest sensor check, but the board cannot home without it. `--mode pinion
--no-park` homes and brakes with the gear in, then holds for five minutes at
full brake while the pinion is removed, with the brake holding the lever;
the pull cycles follow, then the other levels with the gear out, and the run
ends with stop, disarm and rail off without moving. Refit the pinion
afterwards; the next normal run re-homes. With the gear out the encoder, on
the motor shaft, no longer sees the lever, so reduce such a run by gauge
cycle, not by velocity.

## Reducing it

```
python3 tools/torque_cal_reduce.py --radius-mm 72 evidence/<session>/board9_right_run1.log   # gear in
python3 tools/torque_cal_cycles.py --radius-mm 72 evidence/<session>/board9_right_pinion_run1.log   # gear out
```

`torque_cal_reduce.py` pairs each gauge reading with the nearest stream sample,
takes the samples where the lever moves faster than 0.5 °/s, splits them by
direction, and reports per level the median gauge torque and sensor excursion
while sliding, the friction from the brake-off level, and the ratio
`sensor / (gauge − friction)`. `torque_cal_cycles.py` finds the pull cycles in
the gauge trace instead, with each cycle's rest taken just before it, and
reports the same quantities per cycle; it is the one to use when the stream's
angle does not describe the lever.

Report, per station: the ratio at full brake (expected 1.00 on the current
firmware and 0.96 on a `v1.0.0` build, for a sensor at its certificate), the
friction, the brake's delivered torque at each commanded
level, and the raw rest reading. The researcher-facing copy of those numbers
is `mouse-joystick-pcb/docs/station-torque-table.csv`, read through
`mouse-joystick-pcb/docs/bnc-conversion.md`. The sensor's rest moves with the gear mesh
and with hard loads, by 0.3 N·cm or more, so compare excursions, never
absolute readings, and expect the per-trial tare to remain necessary.

## What it cannot do

It cannot separate a sensor-chain gain error from a mechanical path around the
sensor; only the gear-out, string-loop result, agreeing with the certificate,
says there is no such path on the station measured. It cannot tell where in
the lever axis the friction lives; that needs parts loosened one at a time
with the gauge on the lever. And a hand-held gauge is good to about 10 % on
the sensor scale and about 0.05 N·cm on friction; a string loop and the gear
out bring the scale to a few percent.
