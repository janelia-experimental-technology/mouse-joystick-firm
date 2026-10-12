# Board 4 rail-up session — procedure (prepared 2026-10-02)

Board 4 is `E4654C159F171E34`, pcb v0.3, the RIGHT joystick, characterised in
`mouse-joystick-pcb/docs/bringup-log.md` (stages 1 to 3, the angle output on
2026-09-10). This session puts the QP protocol firmware on it with the rail
up for the first time. **Peter is at the bench; nothing here switches the
36 V supply, and no agent runs this without him.**

Build: `mouse-joystick-firm` `mouse_joystick_qp_rev1_usb` (`TmcMotion` over
the TMC5130A, `MJ_BOARD_REV=1`, DAC on SPI1 = the v0.3 strap, protocol over
USB CDC, QS on `FW_TRIG1`, tick probe on the UART0 TX pad; IRUN at the
bring-up value `kCurrentScaleBringUp`, 0.211 A RMS). Driver:
`board4_session.sh`, which calls `tools/board4_steps.py` for the single
steps and `tools/link_pilot.py` for the protocol sequences. Every phase ends
at a stop where the script waits for a typed `yes`; anything else ends the
session with the rail in whatever state the last command left it, so switch
the supply off by hand before walking away.

**Bench and tooling facts (2026-10-02):** the bench setup is fine as it
is. The LA5032's one re-enumeration that day was its firmware load on the
first sigrok open after a replug, not a hub reset; clean and interrupted
captures leave its device number unchanged. **This `sigrok-cli` writes its
file only when a capture completes on its own; a capture stopped by SIGINT
writes nothing**, continuous or `--samples`, which is how the first two
captures were lost. So every capture is timed with `--time` (b 90 s for the
diagnosis, c 180 s, d 90 s, e 20 s), nothing ever signals it, and a failed
step waits for the capture to finish before decoding. The script logs the
`lsusb` device numbers of the LA5032 and of board 4 at the start and at every
stop, asks for the USB topology and the supply grounding into `meta.txt`,
gives every attempt its own directory (`<date>-<hhmm>-board4-rail-up`), and
refuses to decode a capture file older than its phase. After a failed phase
b it re-reads `ID` and prints `boot=` and `crumb=` at the stop, before the
decode: `boot=wdt` with `crumb=N` names the step the watchdog caught.

| Phase | What the script does | What Peter does | Stop |
|---|---|---|---|
| a | finds board 4 by serial, flashes `mouse_joystick_qp_rev1_usb`, runs `check-id` (serial, `hw=rev1`, `angout` in caps) | swaps the fixture for board 4 on the LA adapter, RJ-45 unplugged, 36 V off; then confirms wiring and the supply's current limit | a1 before flashing, a2 before any `hv on` |
| b | capture until the phase ends; `limits` first, and every target below comes from its `travel_lo`/`travel_hi` (about [−10.1, 0] on this build: parked at 0, travel negative; the simulator's ±20 is not assumed anywhere); `hv on`; meter prompt while unhomed (expect 0 V: the DAC parks at code 0 after its two-phase bring-up, visible as `DAC_STATE 1` then `DAC_STATE 2` in the trace); `home`; `angscale travel_lo travel_hi`; `angout on`; meter at the parked end, then `moveto` to 25 %, 50 % and 75 % of the travel with a meter prompt at each against `status`'s encoder angle; back to park; `angout off`; meter; `disarm`; rail left on. Any nak ends the step with the command named | switches the supply on when asked, reads the meter at each prompt | after b |
| c | capture until the phase ends; the pilot with real motion: id, status, `limits` (every target derived from it; any nak ends the run at once), hv on, home (20 s allowed), 20 moves alternating between travel_lo + 1 and travel_hi − 1, `cfg get`, `moveto travel_hi` (the parked end, so the pull has the travel ahead of it), `brake 0`, `watch` with a 60 s timeout, **prompt to pull the joystick toward +angle past 5°**, `PULL`, `TRIAL`, `moveto 0.0` out of RELEASED (the encoder resync; `status` after it shows angle against xactual), FETCH of the window with `lost=`, `stream on 50` for 2 s and off, disarm, the host-loss fault (drops the rail) and `clrfault`; rail kept as the fault left it | pulls the joystick when prompted; puts the scope on the BNC before confirming the next phase | after c |
| d | capture until the phase ends; the pilot with 300 moves, no trial, no host loss, rail kept on; `SAMPLE_STAT` and `LINK_WORK` against the 400 µs budget with the rail up | watches the BNC on the scope during the moves for the 1 kHz staircase and photographs or saves it | none |
| e | capture until the phase ends; `hv off`; `status` must show `hv=0`; `DAC_STATE 0` in the trace; meter prompt for the BNC at 0 V | reads the meter, then switches the supply off | end |

Expected BNC voltages are computed at run time from `limits`: with
`angscale travel_lo travel_hi` the BNC reads 0.050 V + (angle − travel_lo) /
(travel_hi − travel_lo) × 3.250 V, so the three points at 25 %, 50 % and 75 %
of the travel read 0.8625 V, 1.675 V and 2.4875 V whatever the travel is, and
the parked end (`travel_hi`, where `home` leaves the joystick) reads 3.300 V.
`steps/b_angle.json` stores each expected value next to the meter reading and
the board's encoder angle. The stage measured unity to 0.03 % on this board.
While unhomed, with `angout off`, or with the rail down: 0 V (code 0, the
4.5 mV floor).

| Point | Fraction of travel | Expected BNC |
|---|---|---|
| unhomed, or `angout off`, or rail down | — | 0 V |
| parked end, `travel_hi` | 100 % | 3.300 V |
| third meter point | 75 % | 2.4875 V |
| second meter point | 50 % | 1.675 V |
| first meter point | 25 % | 0.8625 V |

Evidence: `evidence/<date>-board4-rail-up/` with `meta.txt` (firmware hash,
board, port), `steps/*.json` (the meter readings with the board's encoder
angle next to each), `pilot/*.json|txt`, `captures/*.sr|vcd`, `uart/`,
`qspy/*-offline-decode.txt`, `analysis/*.json`, `session.log`. After the
session: the README with the comparison against the simulated runs
(`2026-10-01-rp2040-joystick-no-newlib/`), and the electrical numbers of
phases b and e transcribed into `mouse-joystick-pcb/docs/bringup-log.md`
under board 4 with the firmware hash.

The sequence logic of both tools was checked against the firmware's Python
simulator in-process (`link_pilot.py --port sim:`), which has ±20° of travel,
after the targets were derived from `limits`; the fixture run with the
simulated-motion build was not repeated (no flashing before the session).

## After the first attempt (2026-10-02)

The first attempt hung at `hv on` with the tick alive and the port silent
(`../2026-10-02-board4-rail-up/README.md`). Changes since:

- The firmware has a **2 s hardware watchdog** fed from the idle loop, so a
  stuck handler reboots the chip and the reset drops `HV_EN`; the next boot
  reports `Boot: WATCHDOG TIMEOUT, last breadcrumb N` on USB and `BOOT_INFO`
  in the trace.
- **Breadcrumbs on `FW_TRIG0` (CH16)**: trains of 25 µs pulses, count =
  checkpoint; 1 to 8 are the rail-up steps inside `TmcMotion::setHvEnabled`
  (1 entered, 2 powerUp returned, 3 connectBus, 4 beginSpi, 5 TMC version,
  6 SD_MODE check, 7 configure, 8 cache seeded), 9 the `hv` reply queued,
  11 to 17 the DAC bring-up and release (11 claiming, 12 bus up, 13 soft
  reset sent, 14 writing CONFIG/GAIN/SYNC, 15 configured, 16 releasing, 17
  released). The last count before a watchdog reboot names the step that
  hung. Operation marker pulses are 500 µs or longer and cannot be confused
  with them.
- **Bisect result (10:33)**: `hv on` stalled on `mouse_joystick_qp_rev0_usb`
  too, so the DAC path is cleared and the stall is in the rail-up path; the
  watchdog rebooted the board 2.4 s after `hv on` and `HV_EN` dropped. The
  breadcrumb of that run was lost: that build cleared scratch 0 at boot
  after reading it, the banner went to a USB port no host had open, and the
  trace's one `BOOT_INFO` went into a capture that was never written. Since
  then scratch 0 is never cleared, `ID` reports `boot=` and `crumb=`, and
  `BOOT_INFO` repeats every 10 s.
- **Bisect order** (kept for the next board): run the session first with `ENV=mouse_joystick_qp_rev0_usb`
  (real `TmcMotion`, rev 0 pad map, no DAC path; safe on v0.3). If `hv on`
  completes there, the DAC path is the culprit and the rev1_usb run follows;
  if it hangs there too, the rail-up sequence under QV is.
- The script warms the guix shell before stop a1, starts each capture in
  continuous mode and returns only when `sigrok-cli` is running plus 2 s, so
  the capture always holds the command it is meant to see; a failed step
  stops the capture, decodes what exists and stops at that phase; the fixed
  durations are gone.

**Session of 12:45 (rev 0 bisect build): complete**, see
`../2026-10-02-1245-board4-rail-up/README.md`. **Next bench run: the rev 1
build**, the script's default `ENV`, phase b with the meter as planned. The
phase b and d captures now also take the TMC SPI taps, CH2 `TMC_SPI_SCK`,
CH3 `COPI`, CH4 `CIPO`, CH5 `~TMC_CS` (mode 3; the adapter mapping of
`mouse-joystick-firm/README.md`, "Watching it with the LA5032"), at
10 MSa/s with CH8, CH16 and CH17, and the analysis reports datagrams per
TICK handler, the gaps between them, the clocks per datagram and the real
SCK frequency (`link_trace_analysis.py --spi CH5 CH2`). Confirmed on
2026-10-02 without a board: the LA5032 streams those seven channels at
10 MSa/s for 90 s (900 M samples, 90.02 s in the file, 90,024 tick edges on
CH8 and CH16, none lost; 4.5 MB `.sr` with the rail down, larger once the
SPI lines toggle). Phases c and e stay at 1 MSa/s on the three channels.

**Third attempt, what to look for** (after the 11:16 run reached breadcrumb 9
and then never idled): on CH16 the TICK handler level, one per millisecond,
each a few hundred microseconds at most, every tenth one longer (the full
snapshot); back-to-back levels mean overrun, a level that never falls means
a stuck poll; `SAMPLE_STAT` now carries skipped (idle gap), skipped
(decimated) and the decimated flag after max and last cost. Success is
`hv on` acked within 3 s and `SAMPLE_STAT` reporting the real tick cost.

Known in advance: `hv on` is a long run-to-completion step on `TmcMotion`
(the rail's soft start, then 100 ms settle), so the first `SAMPLE_STAT`
after it will show one tick of 150 to 250 ms and the coalesced samples of
that step; that is the rail, not the sampler. The pull direction is
`pull_sign = +1`, increasing encoder angle; if the hand pull reports `PUSH`,
the sign for this joystick is −1 and goes into `AppConfig.hpp` with the
board. Rev 1's HV fault input and latch are not wired into the QP build
yet (`BoardPins.hpp` carries the v1.0 changes only as notes).
