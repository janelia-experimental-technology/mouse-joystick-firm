# PyControlLinkQp — the pyControl protocol over QP/C++

Stage 4 again, structured as a QP/C++ (v7.3.4, QV kernel) application: the
same `ProtocolBoard` and `TmcMotion` as `examples/PyControlLink`, unchanged,
inside one active object, with a QS trace leaving the board on `FW_TRIG1` so a
logic analyser on the InstrumentPort can see what the firmware did, per
request, without USB and without touching the pyControl link.

It exists because the only thing QP buys this board is observability, and
that had to be measured before deciding anything about the delivery firmware.
The measurement is in `qp-arduino-example/evidence/2026-10-01-rp2040-joystick-link/`
(the pilot this application is derived from) and in the firmware run recorded
alongside it (see "Status"). `examples/PyControlLink` stays the non-QP
delivery sketch; nothing here changes the protocol or the pyControl side.

## What runs where

| Piece | Here | Notes |
|---|---|---|
| `ProtocolBoard`, `TrialWatch`, `TmcMotion`, `SimulatedMotion` | `src/` | shared with `PyControlLink`; protocol v1 (`proto=2`) since 2026-10-01, the 48-case corpus holds in both halves |
| `LinkAo.cpp` | the active object | 64-bit clock, lines in, lines out: what `PyControlLink.ino`'s `loop()` did, as run-to-completion steps with trace records |
| `Bsp.cpp` | board support | `SafeState` first, boot signature and identity beacon, 1 kHz hardware-alarm tick (the sampler) with watchdog, non-blocking line transport, transmit-only QS on a PIO UART at `FW_TRIG1`, the rev 1 angle DAC |
| `AppConfig.hpp` | configuration | the mechanism and protocol numbers of `PyControlLink.ino`; `has_angout` follows `MJ_BOARD_REV`; `pull_sign` comes from `src/BoardCalibration.hpp` by serial once the board is known (board 4 is −1, measured 2026-10-02; others +1, unmeasured) |
| `PyControlLinkQp.ino` | `setup()`/`loop()` only | event pool (8 lines), queue (16 events), start |

The qp-arduino library comes from the sibling checkout `../qp-arduino`
(branch `upgrade/qpcpp-v7`) through `platformio.ini`'s `[qp_env_data]` until
it is released; the link is `symlink://`, never a bare path, for the reason the
board library already documents.

## Signals and ports

| Signal | Pin (from `BoardPins.hpp`) | Behaviour |
|---|---|---|
| pyControl protocol | UART0 `kPycUartTx`/`kPycUartRx`, 115200 8N1 | as before; `MJ_LINK_USB=1` moves it to USB CDC for a bench with no host |
| QS trace | `kFwTrig1` as a PIO UART TX, 115200 8N1, transmit only | starts after the boot signature and the identity beacon, which still use the pin as GPIO; nothing on this line can command the board |
| operation marker | `kFwTrig0` | high from the start of a homing or move to its completion (`OP_START`/`OP_DONE`) |
| USB CDC | `Serial` | a banner at boot; in the bench build, the protocol |

`FW_TRIG1` therefore no longer carries the RELEASED level of `PyControlLink`:
that state is a `LINK_STATE` record. Which marker roles the v1.0 board's
InstrumentPort overlay finally assigns is an `OBS-IF` decision still owed
(`../mouse-joystick-pcb/docs/observability-plan.md`); the two uses here are
one-line changes.

## Trace records

| Record | Fields | Meaning |
|---|---|---|
| `LINK_RX` | seq, verb, length | a protocol line was dispatched to the AO |
| `LINK_TX` | sent flag, ready time, first 20 characters | a reply or event was queued for the port (0 = ring full, dropped) |
| `LINK_STATE` | name | `ProtocolBoard`'s state changed |
| `OP_START` / `OP_DONE` | homing flag, angle | an operation started / completed; `FW_TRIG0` follows |
| `LINK_WORK` | advance µs, feed µs, output µs | where a dispatched line's time went |
| `DAC_STATE` | phase (0 down, 1 reset, 2 up), begin failures | the rev 1 angle DAC's bring-up after `hv=1` and its release when the rail drops |
| `BOOT_INFO` | watchdog timeout?, forced reboot?, last breadcrumb | once after boot: why the chip booted and where the previous run was when the hardware watchdog bit |
| `SAMPLE_STAT` | ticks, handled, coalesced, dropped, max µs, last µs, skipped (idle gap), skipped (decimated), decimated flag | the 1 kHz sampler: ticks the alarm posted, ticks the AO ran, samples lost to a tick still in flight or to a full queue, and the cost of the tick handler (poll, classifier, capture, DAC write), every second |
| `TRACE_STAT` | high-water, buffer-full count, size, ticks | QS buffer use, every second |
| `LINK_STAT` | lines, drops (pool, queue, ring), overlong, queue min free, pool min free, ring high-water | transport and budget use, every second |
| `TICK_LOST` | ticks, re-arms, catch-ups | the tick watchdog fired (never expected) |

Plus QP's own state transitions, entry/exit and assertions. The per-event
dispatch/post/get records are filtered out by default (`LINK_QS_FULL=1` keeps
them, and saturates the line at a 1 kHz tick).

## The sampler under the rail (2026-10-02)

Board 4's second attempt reached breadcrumb 9, `hv on` complete in 272 ms,
and then never idled again: with the bus up every 1 kHz tick polled the
TMC5130A, and the handler ran back to back until the watchdog bit. Four
things changed:

- **Poll cost.** `TmcMotion::poll` reads the encoder alone (X_ENC, two 40-bit
  datagrams) on every tick and the full snapshot (RAMP_STAT, DRV_STATUS,
  XACTUAL, VACTUAL, X_ENC: ten datagrams) every tenth tick, where completion,
  the limit switch and driver faults are decided, so within 10 ms. The SPI
  clock is `kTmcSpiClockHz`, 4 MHz, in `Conversions.hpp` (the library's
  1 MHz default was chosen for a 1 MSa/s analyser; the analyser now needs
  10 MSa/s to see the clock).
- **Overrun guard.** If the previous TICK handler ended less than 200 µs
  before this one starts, the AO is running back to back and the idle loop
  has had no turn: the tick's work is skipped and counted, so idle always
  gets one. If the handler exceeds the 400 µs budget three ticks running,
  polling drops to every other tick until the rail goes down; counted and
  flagged in `SAMPLE_STAT`.
- **Tick level.** In `MJ_TICK_LEVEL` builds (the three `_usb` bench builds)
  `FW_TRIG0` is high for the whole TICK handler: the analyser shows the
  handler's duration directly, back-to-back levels are an overrun, a level
  that never falls is a stuck poll. The operation marker is then a trace
  record only; breadcrumb trains still pulse the pin.
- The static accounting behind this is in the evidence README of the run.
- 2026-10-02 12:45, board 4, `mouse_joystick_qp_rev0_usb`
  (`qp-arduino-example/evidence/2026-10-02-1245-board4-rail-up/`): `hv on`
  acked in 272 ms, the full v1 trial on real motion, 300 of 300 moves, the
  watchdog never fired. The tick cost with the rail up is about 640 µs flat
  while holding and 444 µs p50 moving, which the datagram count does not
  explain, so the sampler ran decimated at 500 Hz for the whole rail-up
  period; the next capture includes the SPI lines. Findings and the
  poll-cost proposal are in that README.

## Watchdog and breadcrumbs

Since 2026-10-02 the BSP enables the RP2040 hardware watchdog, 2 s, after
`SafeState` and the USB host wait, and feeds it in `QV::onIdle()` only: a
run-to-completion step that never returns reboots the chip, and the reset
drops `HV_EN`. `Q_onError` keeps its two-second `FW_TRIG0` signature by
feeding the watchdog inside it. The reason for a boot and the last
breadcrumb are read from the watchdog block and its scratch registers before
anything else runs, printed in the banner (`Boot: WATCHDOG TIMEOUT, last
breadcrumb N`) and traced as `BOOT_INFO`.

`BSP::crumb(n)` is the breadcrumb: a train of `n` pulses, 25 µs high and
25 µs low, on `FW_TRIG0`, and `n` into watchdog scratch 0. An operation
marker pulse is 500 µs or longer, so the two are told apart on the analyzer.
A train during a running operation briefly lowers the marker; none of the
checkpoints below runs during a move.

| Count | Where |
|---|---|
| 1 | `TmcMotion::setHvEnabled(true)` entered, before `powerUp` |
| 2 | `powerUp` returned |
| 3 | `connectBus` done |
| 4 | `beginSpi` done (after its 10 ms delays) |
| 5 | TMC version 0x11 and link verified |
| 6 | SD_MODE check passed |
| 7 | `configure` done, bus up |
| 8 | position cache seeded, `setHvEnabled` returning true |
| 9 | the `hv` reply queued in the port ring (LINE handler) |
| 11 | DAC: claiming the bus (first tick after `hv=1`, rev 1) |
| 12 | DAC: bus up (SPI1 or Wire begun) |
| 13 | DAC: soft reset frame sent |
| 14 | DAC: writing CONFIG, GAIN, SYNC (2 ms later) |
| 15 | DAC: configured |
| 16 | DAC: releasing the bus (rail going down) |
| 17 | DAC: bus released |

The hook into `TmcMotion` is `setCheckpointHook()`; `PyControlLink` does not
install one and pulses nothing.

## Builds

| Env | For |
|---|---|
| `mouse_joystick_qp` | rev 0 (pcb v0.1/v0.2): `TmcMotion`, UART0, trace on `FW_TRIG1` |
| `mouse_joystick_qp_rev1` | rev 1 (pcb v0.3 and later): angle DAC on SPI1 |
| `mouse_joystick_qp_rev1_i2c` | rev 1 with the DAC on I2C, v1.0's strap (compiles; no board to test it on yet) |
| `mouse_joystick_qp_rev1_usb` | board 4's rail-up bench build: `TmcMotion`, rev 1, DAC on SPI1, protocol over USB CDC, trace on `FW_TRIG1`, tick probe on the UART0 TX pad with the RJ-45 unplugged; driven by `qp-arduino-example/evidence/bench-scripts/board4_session.sh`. **Hung on the first `hv on` on board 4, 2026-10-02** (see Status) |
| `mouse_joystick_qp_rev0_usb` | the bisect build for that hang: the same, with the rev 0 pad map and therefore no DAC path at all; safe on a v0.3 board, whose rail-up pins are the same |
| `mouse_joystick_qp_sim_usb` | a board with no 36 V and no host: `SimulatedMotion`, protocol over USB CDC, tick probe on the unused UART0 TX pad (InstrumentPort `DIG8`). Accepts the bench stimulus `sim;<seq>;angle;<deg>` (sets the simulated encoder, as the corpus's `ANGLE` step does, so a `WATCH` outcome can be driven by hand; `ack;<seq>`), which exists in no other build and is not a protocol verb |

```sh
pixi run build examples/PyControlLinkQp mouse_joystick_qp
pixi run upload examples/PyControlLinkQp mouse_joystick_qp_sim_usb /dev/ttyACM0
```

Drive the bench build with `../qp-arduino-example/tools/link_pilot.py` and
read the trace as `qp-arduino-example/evidence/bench-scripts/link_run.sh`
does (sigrok `uart` decoder on channel 17, bytes to `qspy -k -f`).

## Status

- 2026-10-01: all three environments build; the 114 host tests still pass.
  The bench build ran on the rev 0 fixture (board E4654C159F2C2534, 36 V
  unplugged) under the pilot's full sequence: 21 of 21 steps, 1,003 lines
  and 1,206 replies with none dropped, QS high-water 482 of 2,048 bytes with
  zero buffer-full, queue and pool never below 15 of 16 and 6 of 8 free,
  tick exact, host-loss fault 105 ms after the last heartbeat. The delivery
  build was flashed and its boot sequence captured: signature, beacon, then
  the trace with `READY` and a statistics record every second, nothing on
  UART0. Results in
  `qp-arduino-example/evidence/2026-10-01-rp2040-joystick-firm-qp/`.
- **Not yet run:** `TmcMotion` behind this AO with the rail up (as for
  `PyControlLink`, whose first supervised run it still owes), the UART0
  protocol path under QP with a pyControl host, and anything on a v1.0 board,
  whose GPIO changes (`GP24` becomes `HV_LATCH`, `~HV_FAULT` moves, the DAC
  goes to I2C) are not in `BoardPins.hpp` yet.
- The RP2040 hardware watchdog task of `.agents/PLANS.md` applies here as it
  does to `PyControlLink`; `Q_onError` already drops to `SafeState` before
  rebooting.
- 2026-10-01, later: protocol v1 (`proto=2`: BRAKE, WATCH, CFG, chunked
  FETCH, STREAM, TRIAL, 64-bit event times) in both halves with the corpus
  rewritten (46 cases, 120 native and 164 host tests), the tick raised to
  1 kHz as the sampler with the tick coalesced (one TICK_SIG in flight, lost
  samples counted in `SAMPLE_STAT` rather than overflowing the queue behind
  the rail's soft start), and the rev 1 angle output driven from the tick:
  bus claimed and GAIN 0x0101 written only after `hv=1`, parked at the fault
  code while `angleOutputValid()` is false, released when the rail drops.
  **Compiled only**, in all four environments; nothing was flashed. The
  sampler cost is instrumented (`SAMPLE_STAT` max/last, `LINK_WORK`), not
  measured: the 400 us budget is a declaration until a run on the fixture.
- 2026-10-01, review: the capture became a ring (`TrialWatch`): sampled from
  `RELEASED` on, frozen `post_ms` after the outcome around the outcome sample
  (`CFG pre_ms`/`post_ms`, 500/500, sum ≤ 2047), so a pull late in a 10 s
  trial is captured where the old two-second linear buffer missed it. Corpus,
  simulator, firmware and device class changed together.
- 2026-10-01, fixture run of the bench build at firmware 0109ca5
  (`qp-arduino-example/evidence/2026-10-01-rp2040-joystick-ring-capture/`):
  31 of 31 pilot steps, the whole v1 trial shape including the 1,001-sample
  window fetched in 32 chunks and 100 stream lines in 2 s. **Measured
  against the 400 µs budget:** the sampler tick costs 20 to 60 µs idle, the
  alarm tick is exact (stdev 10 µs, none lost), but one protocol line costs
  1.0 ms (`ping`) to 2.8 ms (`status`) to 15.9 ms (a 32-line `FETCH`
  chunk), so one sample is coalesced per host line (2,183 of 86,024 ticks,
  2.5 %, zero dropped, filled on the grid). The cost is `formatNumber`'s
  `snprintf("%.3f")` with software double on the M0+, about 0.5 ms per
  number, pre-dating this example. Also: a `FETCH` chunk's 34 `LINK_TX`
  records saturate the trace (QS high-water 2,048, 35 buffer-full events).
  Fixed the same day, below.
- 2026-10-01, no newlib on the line path
  (`qp-arduino-example/evidence/2026-10-01-rp2040-joystick-no-newlib/`):
  `Protocol.hpp` formats and parses in integer fixed point (exact on the
  binary value, ties to even, 124 native tests), the cache and queue copy
  without `snprintf`, and the trace takes one `LINK_TX` per `FETCH` chunk.
  Rerun on the fixture, 31 of 31 steps: feed cost `ping` 330 µs (was 1,031),
  `status` 519 (2,839), `moveto` 818 (1,841), a `FETCH` chunk 1,201 (15,921);
  coalesced samples 237 of 85,999 (was 2,183), QS buffer-full 4 (was 35),
  worst tick 694 µs (was 2,379), the window in 217 ms (was 774). Still over
  one tick per line: `moveto` (1.13 ms total, the one that coalesces a sample
  per command), `FETCH` chunks (1.6 ms), `id`, `watch`, `brake`, `stream`.
  The floor of about 450 µs per line, with no number in it, is not
  attributed yet; splitting `LINK_WORK` inside `feed()` is the next
  measurement, and whether long replies alternate with the tick is a
  decision, not a default. Accepted as is on 2026-10-02: the floor fits the
  rig's message rate.
- 2026-10-02, before the rail-up session: `FETCH`'s ack carries `lost=N`,
  the window samples that were grid fills after a coalesced tick (spec,
  corpus, simulator, firmware, device class; a fill-path unit test on each
  side); the `DAC_STATE` record; the `mouse_joystick_qp_rev1_usb` build; the
  board 4 session script and procedure. Nothing flashed.
- 2026-10-02, board 4, first rail-up attempt
  (`qp-arduino-example/evidence/2026-10-02-board4-rail-up/`): identity ok,
  `limits` ok (travel [−10.125, 0], `v_max` 36), then **`hv on` never
  answered**: USB stayed enumerated, the tick probe kept running, the trace
  and the port went silent, no reset. The active object never returned from
  the `hv` LINE handler or the first TICK after it. Static reading found no
  unbounded wait that differs from `PyControlLink`'s working sequence
  (`panic()` is excluded by the live tick), so the firmware gained the
  hardware watchdog, the breadcrumbs and the rev 0 bisect build above; the
  reproduction is Peter's, from the bench.
