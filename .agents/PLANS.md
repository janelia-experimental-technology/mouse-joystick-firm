# Plans

## 2026-10-20: explicit revision and observation-profile checks

- [x] Remove silent MJ_BOARD_REV=0 fallback; native environment now explicitly
  sets 0. Existing target environments still select their revisions.
- [x] Firmware-owned schema-2 profile in `instrument/`; boot beacon/runtime
  trace ownership separated without changing runtime firmware.
- [x] `tools/check_board_pins.py` compiles constants/spares against an explicit
  PCB-owned export; bindings contain signal names, not GPIO copies. Rejects
  revision/polarity/coverage mistakes. No sibling dependency in normal builds.
- [x] PCB owner added manufactured logical pad comparison and full observation
  paths/domain/bias facts in batches two/three; original GPIO export scope is
  retained and supplemented, not relabelled as electrical qualification.
- [x] Batch four (2026-10-21): firmware-owned peripheral allocation and generated
  report, checked by kicad-libs against RP2040 capabilities. Every assigned pin,
  five fixed engines and QS PIO capacity checked. Firmware/framework hashes
  guard the reviewed declarations; no GPIO number copy or runtime change.
- [ ] Automatic C++/build-option resource derivation, full SDK/IRQ/DMA/timer
  inventory and measured reset/off-state qualification remain separate.

Checks: 156 native Unity tests; RP2040SelfTest default, safeflash, rev1 and
rev2 compile, plus PyControlLinkQp delivery rev2. Three tool test methods
exercise valid revisions and deliberate missing/invalid revision, GPIO, spare
and polarity mistakes. All 28 production constants and two spares agree with
the selected current PCB export. Both profile phases load under
instrument-control. No flashing or bench action; nothing committed.
See `docs/instrument-profile.md`; cross-repository queue is kicad-libs/PLANS.
Fourth-batch checks: all three pin-tool tests and actual 28-pin/two-spare
compile pass; shared allocation tests (10) pass; report replays identically.
Declaring GP21 as UART1 TX fails before report output. Only records/docs
changed here; no target build, flash or new deployment qualification.

## Current Goal

Delivered: protocol v1 on four pcb v1.0 boards across two rigs (Rig 1
2026-10-09, the desk rig 2026-10-12). `v1.0.1` (the certificate-mean torque
scale) is tagged and every board runs it since 2026-10-16: the desk rig's
two from the desk machine, Rig 1's two from Peter's laptop at the rig, read
back over the RJ-45s through the rig's own pyboard, and every station has
its force-gauge torque row in `mouse-joystick-pcb/docs/station-torque-table.csv`
(`evidence/2026-10-16-1359-rig1-torque-cal/`). The rig branch
(`joystick-rig` `feat/mouse-joystick`) carries the matching device class
since the local commit of 2026-10-15 (the `poll()` UnicodeError guard;
Peter pushes it). Next: the researcher-facing documentation
(`orb-weaver-context/context/mouse-joystick-rig-documentation-plan.md`,
phase 2: architecture page, circuit-to-code map, diagrams).

## Open Item — Two of four v1.0 units ignore the 1200 baud touch (opened 2026-10-13)

Units B and D (boards 7 and 9) take the bootloader touch at most once and
then need the button under `mouse_joystick_qp_rev2`; units A and C take it
every time with the same build. The board-side item is in
`mouse-joystick-pcb/.agents/PLANS.md`. Firmware side:

- [ ] Confirm on a failing unit whether the host's line-coding request
      reaches the CDC at all (USB capture, or a QS record on the CDC
      callback) before anything in the application is suspected; the two
      units took the touch from the self-test build, which has the same
      core.
- [ ] If the request arrives and is ignored, look at what the delivery
      build does differently around USB: the 1 kHz hardware-alarm tick and
      the watchdog (`Bsp.cpp`), and whether TinyUSB's task gets starved.
- [ ] Either way, keep the `B` protocol verb and the button procedure in the
      README as the documented routes into BOOTSEL.
- [ ] 2026-10-14, Peter's laptop: board 9 (unit D) **took the touch** from the
      delivery build `316f802-dirty` and flashed in 4.3 s; board 8 in 7.6 s;
      board 8 took a second touch (a pyserial 1200 baud open, DTR low) a
      minute later. So "ignores it every time" is wrong for board 9 — it is
      intermittent, and the host (laptop vs desk computer, hub) may matter.
      One board at a time: PlatformIO skips the touch when any RP2040 is
      already in BOOTSEL and `picotool load` takes no address.
- [ ] 2026-10-16, Peter's laptop at Rig 1 again: both boards took the touch
      first time from the `v1.0.1` build (board 6 in 7.9 s, board 7 in
      4.4 s), no picotool error on either.
- [ ] 2026-10-15, Peter's laptop at Rig 1: board 7 (unit B) **took the
      touch** first time from `51322e7-dirty` (4.8 s; board 6 9.6 s), with
      both boards on USB but only one addressed. No unit has refused the
      touch from the laptop yet, so look at the host side first (hub,
      kernel, what the desk computer sends) when this is picked up.

## Task: pcb v1.0 Bring-Up (opened 2026-10-04; boards arrive 2026-10-05)

The v1.0 boards (two ordered from the 2026-09-21 fabrication package) are the
first boards with the brake sink, the torque bridge, I2C DACs, LATCH on a GPIO
and the Digital-50 instrument port. The board file is unchanged from
`mouse-joystick-pcb` commit `d5ae4b0` through its fabrication package `646b8f4`
and HEAD `6d5ffd3`; **no `v1.0` tag exists yet** — tagging the fabricated
commit is Peter's call and the firmware comments say "no tag yet" until then.
The pin map was read off footprint `U601` with a script over the `.kicad_pcb`
(the RP2040 is `U601` at v1.0; `U501` is now the 36 V switch), not from prose.
Rig-level sequence: `../orb-weaver-context/context/mouse-joystick-rig-system-plan.md`
section 9, Phase A.

### Done before the boards arrived (software only, 2026-10-04, uncommitted)

- [x] `MJ_BOARD_REV=2` in `src/BoardPins.hpp`: the v1.0 map, `hw=rev2`,
      `HV_LATCH` on GP24 (`MJ_HAS_LATCH_GPIO`/`MJ_LATCH_GPIO` forced, overrides
      refused), `~{HV_FAULT}` on GP23, I2C0 on GP16/GP17 with the DAC
      addresses 0x48/0x49, the torque ADC on SPI1 (GP10/11/12, CONVST GP15),
      `TQ_CAL_EN` GP18, `~{BRAKE_FAULT}` GP13, `BRAKE_ISENSE` GP27,
      `ANGLE_OUT_MON` GP29, spares GP20/GP22. The SPI DAC constants do not
      exist for rev 2, so no SPI DAC code path compiles for it.
- [x] `SafeState`: drives `HV_LATCH` low (auto-retry, the reset default;
      `powerUp()` raises it after the rail is up), `TQ_CAL_EN` low and
      `TQ_CONVST_CS` high behind the open switch, leaves `~{BRAKE_FAULT}`, the
      SPI1 and I2C pins as inputs; `setInstrumentationEnabled()` is a no-op
      on rev 2 (Digital-50 has no enable). Reset-state reads: `R607`, `R401`,
      `R507` on GP23, `R402`/`R403` (I2C idle high), `HV_LATCH` informational.
- [x] `RP2040SelfTest` on rev 2: 2 spare pads, the four straps above, the two
      monitor ADC channels at 0 V with the rail off; **20 checks** by
      derivation from rev 1's 24 (−7 spares −1 buffer OE +2 I2C +2 ADC).
- [x] `TMC5130Communication`: v1.0 designators (`U501`, `R501`, `R506`,
      `R508`, `R509`; no LATCH strap). `MotionAndHoming`, `AngleOutput` and
      `PyControlLinkQp/Bsp.cpp`: the DAC transport follows the revision (rev 1
      SPI1, rev 2 I2C0 on `pins::kDacScl`/`kDacSda`), `MJ_DAC_I2C` is derived
      and a contradiction is a compile error; the v1.0 ÷2 output divider is in
      the printed BNC volts (1.650 V full scale).
- [x] Protocol: `ProtocolBoard::Config::instr_switchable` (false on rev 2, so
      `INSTR` is accepted and `state` is always 1, per uart-protocol.md).
- [x] Envs: `mouse_joystick_rev2`, `mouse_joystick_qp_rev2`,
      `mouse_joystick_qp_rev2_usb`; the two `*_rev1_i2c` envs removed (they
      paired rev 1 pins with an I2C DAC and matched no board).
      `mouse_joystick_rev1` is now documented as v0.3 only, not "HEAD".
- [x] Host tests: 125 pass (a Guix `gcc-toolchain` is needed on this machine;
      the conda compiler in the Pixi env fails at link against the system
      glibc). Target builds for every example and env: see the status line
      below once they have run.

### Still owed, in order (software first, then the bench)

- [x] **Regression of HEAD on v0.3 before the v1.0 boards (2026-10-05, board 4):**
      every stage console on `mouse_joystick_rev1` and the QP session on
      `mouse_joystick_qp_rev1_usb`, no firmware regression; see Status.
- [ ] **Tag `mouse-joystick-pcb` `v1.0` at the fabricated commit** (`646b8f4`
      is the one whose `fab/` package is the order; `d5ae4b0` is the last
      board-file change). Then replace "no tag yet" in `BoardPins.hpp`,
      `platformio.ini`, the README and `docs/rp2040-bringup.md`.
- [ ] **Rig plan Phase A1 cannot see the DACs or the ADS8689 on USB alone.**
      Both DACs run from `+3.3V_FILT`, the ADC from `+3.3V_SW`/`+5V_FILT`, and
      every one of their bus lines is behind the bus switch. "I2C sees 0x48
      and 0x49, SPI1 sees the ADS8689" is an A2/A3 check with the rail up and
      the bus closed. Corrected in the rig plan 2026-10-04; A1 is the
      `RP2040SelfTest` run and nothing more.
- [x] A1: `RP2040SelfTest` on `mouse_joystick_rev2`, USB only (2026-10-05,
      unit A `E4654C159F274534`): **20 checks, 0 failed**, `HV_LATCH` low,
      monitors at 0 V; recorded in the PCB bring-up log. Both v1.0 units
      arrived with a wrong part (green, banded) at every `C2858031` position
      (`C1301` on `+3.3V_MCU`, `C1208` on TMC `5VOUT`, `C1501` on `+5V_SW`);
      removing `C1301` brought the board up. `C1208` must be refitted with a
      real 2.2–4.7 µF before A2.
- [x] A2 (2026-10-05, unit A): `powerUp()` reports the direct method; the
      first attempt failed because latch-off was armed 10 ms after `+3.3V_SW`
      read live while the switch was still charging the bulk (PG lines up at
      6–11 ms, trip at 37 ms). **Fixed in `PowerSequence`: LATCH stays low
      until `HV_SNS` has left the fault band for 20 ms (33–34 ms measured),
      `powerDown()` restores LATCH low, `PowerUpReport::charge_ms` added.**
      Two clean passes, 5/0 register checks each; idle 36 V input current
      43.6–51.3 mA. The LATCH probe's strapped half fails as designed. Numbers
      in the PCB bring-up log. Uncommitted.
- [x] A3 (2026-10-05, unit A): the I2C DAC at 0x48 answers, `GAIN 0x0101`
      written and **read back matched**, `DEVID` 0x115, `REF-ALARM` clear, every
      transaction ACKed on the Digital-50 I2C taps. `ANGLE_OUT_MON`
      readback added to `AngleOutput` (rev 2): midscale 0.8236 V, full scale
      1.6415 V, floor 12 mV at the BNC node, within the RP2040 ADC's accuracy; a
      meter fit is still owed. I2C set to 400 kHz in both bench consoles (the
      QP build already had it); the live loop's worst iteration went 1082 →
      732 µs. Uncommitted.
- [x] A4 (2026-10-06, unit A): `examples/BrakeAndTorque` (new; the console
      for A4 and A5 together, line-oriented commands, `tools/board_console.py`
      drives it) configures the brake `Dac80501` at 0x49 with `GAIN 0x0101`
      read back, sweeps, steps and reads `BRAKE_ISENSE`/`~{BRAKE_FAULT}`.
      Brake chain **linear, full scale 1.0065 V = 48.80 mA** (design 1.006 V),
      ±0.07 % FS above half scale, the 12 V gate drive sufficient. **Finding:
      a step above ~60 % overshoots — 62 mA (127 %) for ~5 ms at full
      scale — and `~{BRAKE_FAULT}` never asserts**, although 62 mA is above
      the 56.4 mA ceiling. Board question filed in the PCB plan; firmware
      consequence: `BRAKE` ramps its setpoint (≥ 6 ms to full scale) and
      checks `BRAKE_ISENSE` against it rather than trusting bit 9. Numbers
      in the PCB bring-up log; `docs/brake-torque-bringup.md` here.
- [x] A5 (2026-10-06, unit A): ADS8689 driver written (`src/ADS8689*.hpp`,
      `src/ADS8689/`, 22 host tests), `RANGE_SEL` 0x0B written and read
      back, SDO proven by the `DATA_VAL` loopback patterns before any
      conversion is trusted; the frames decoded on the Digital-50 taps.
      Torque at rest 1.5956 V (69 mV below the ideal zero: mechanism preload
      or offset, tare covers it), **noise 95–112 µV rms at 1 kHz**, 1000.1 Hz
      live loop at a 295 µs worst tick, no brake-to-torque coupling at
      0.1 mV. **The shunt-cal switch is inert** (0.0 codes of a 0.744 V step),
      as the 2026-10-04 design review predicted; `TQ cal` stays a verb and
      not a calibration.
- [x] **Torque calibration against a force gauge (2026-10-16, desk rig, both
      boards).** `tools/torque_cal_run.py` drives a board through the pyboard
      REPL with a Mark-10 logged beside the 50 Hz stream and a cue window for
      the person at the lever; `tools/torque_cal_reduce.py` and
      `tools/torque_cal_cycles.py` reduce it; `docs/torque-calibration.md` is
      the procedure. Result: the chain is on its certificate (0.965 measured
      against 0.961 expected, gear out, string loop); friction 0.25–0.5 N·cm
      per station in the lever axis; the brakes deliver 0.4–0.7 at the
      commanded 1.0. Numbers in the PCB bring-up log, 2026-10-16.
      **Decided 2026-10-16: the firmware carries the certificate mean.**
      `kTorqueSensitivityVPerV` = 1.5605e-3 in `Conversions.hpp` is the
      chain's constant; every reported torque is 0.961 of what the same
      reading gave on `v1.0.0`, `WATCH`'s `touch` is in true N·cm, the BNC
      is analog and unchanged, and pyControl's log now agrees with the
      0.141 V/N·cm of `mouse-joystick-pcb/docs/bnc-conversion.md`. The
      desk boards took the new build on 2026-10-16 and Rig 1's the same
      afternoon at the rig, followed by both Rig 1 stations' runs
      (`evidence/2026-10-16-1359-rig1-torque-cal/`: friction 0.27/0.12 N·cm
      right and 0.18/0.15 left, the brakes delivering 0.36 and 0.43 at the
      commanded 1.0, sensor over gauge 0.85–1.07 with the hand-held
      geometry; the Rig 1 rows of the station table are filled). Two
      lessons in the run script: a pause without pings faults the 100 ms
      watchdog; `brake 0` answers `ack` and `ev;RELEASED` in one read. Two
      more from Rig 1: soft-reset the pyboard before a REPL-driven run
      (`tools/pyb_soft_reset.py`; pyControl's leftover framework halves the
      heap and the run died of `MemoryError`), and the gauge's zero wanders
      by up to 13 gF, so zero once before the run, never during it, let the
      string go slack at every hands-off and reduce with
      `tools/torque_cal_reduce_zero.py`.
- [ ] Repeat the gear-out string-loop scale check on a Rig 1 station when
      the pinion can be pulled there, and try the slides at the desk's
      2–3 °/s: Rig 1's ran at 4–13 °/s and the friction may depend on it.
- [x] **Brake and torque in the protocol (2026-10-06, both ends, run on unit
      A).** `src/BrakeTorqueBackend.hpp` is the seam (with the simulated
      backend the corpus runs against); `src/BrakeTorqueHw.hpp` the real one
      (brake DAC at 0x49 with a setpoint ramped at 8 ms to full scale,
      `BRAKE_ISENSE` supervision for bit 10, the fault pin for bit 9, the
      ADS8689 sampled on the tick, bits 11–12). `ProtocolBoard`: `caps`
      `brake`/`torque`, `BRAKE` acks then waits for the backend to settle
      before releasing the drive and emitting `RELEASED` (motion verbs busy
      meanwhile, `STOP`/`DISARM` cancel), the drive re-engages BEFORE the
      brake comes off on every way out of `RELEASED` (3a; the old order was
      wrong and no board could show it), `TQ zero|cal|get`, `torque` in
      `STATUS`/events/`TRIAL` (`t_touch`, `tq_peak`, `tq_out`)/capture,
      `TOUCH` on the torque magnitude. Mirrored in `board_sim.py`; corpus
      grammar gained `CAPS <tokens>` and a live `TORQUE` stimulus, 5 new
      cases; device class `torque_get()`/`torque_cal()`. 154 Unity, 178
      pytest. On unit A (`mouse_joystick_qp_rev2_usb`, driven by
      `qp-arduino-example/tools/brake_torque_steps.py`): `RELEASED` 11.6 ms
      after `brake 1.0` and 15–17 ms after `brake 3.4`, torque −0.30 N·cm on
      the nominal zero then tared, `NOMOVE` at 3000.1 ms, `TRIAL` with torque
      fields, `FETCH` with the torque column, clean `hv off`. Decisions the
      spec left open are written up as uart-protocol.md V1.11. Numbers in the
      PCB bring-up log ("brake and torque in the protocol build").
- [ ] **The 1 kHz sampler is decimated whenever the rail is up on v1.0**, and
      was already on 2026-10-05 (`last` 110–750 µs, `max` 0.9–1.5 ms without
      the brake/torque poll; 0.26–1.2 ms and 2.1–2.5 ms with it, 1.2 ms in
      `RELEASED` with a `WATCH` armed). The primitives are cheap (ADS8689
      sample 37 µs, `BRAKE_ISENSE` 4.8 µs, DAC write 110 µs, console `M`);
      `delayMicroseconds()` in the ADS8689 transport was the first suspect
      and was not it (busy wait now, no change). **Attributed with
      `POLL_STAT` (QS_USER+14, per second: each backend's worst poll, its
      calls, and its worst primitive), run `…-0942-unitA-qp-brake-torque`:
      with the rail up EVERY primitive shows 100–500 µs maxima — the X_ENC
      read 350–555 µs, the five-register snapshot 400–670, the ADS8689 sample
      230–480, even the `BRAKE_ISENSE` `analogRead` 110–230 µs — against 30,
      150, 37 and 4.8 µs on the bench console.** A 5 µs ADC read cannot take
      230 µs by itself: this is interrupt time landing inside whichever bus
      primitive is running. The bench builds carry the protocol over USB CDC,
      and the Arduino core services tinyusb from a 1 kHz interrupt
      (`tud_task()` at IRQ level), so the heartbeat and the replies are the
      prime suspect; the QS PIO UART's ISR is the other. Next: an A/B with
      the heartbeat slowed and the trace off, then the delivery build over
      UART0 with the pyboard (which has almost no USB traffic) — the tick
      budget must be judged on that build, not on `_usb`. Until then the
      brake ramp's steps are coarser than designed (three writes instead of
      eight for 3.4 N·cm) though still below the overshoot threshold, and
      `lost=` in a trial's capture will not be 0 under load.
- [x] `SafeState::driveOutput()` writes the level before the direction
      (SDK calls, no `gpio_init()`): the 34 µs `~{BUS_SW_EN}` boot glitch is
      gone, no edge on CH27 across a warm reboot (2026-10-06).
      `measureHvCurrent()` now waits for the rail to read off (and reports
      how long it took) instead of pausing 50 ms, so stage 2's `m` works
      after a live rail; not yet re-run on hardware.
- [x] A6 (2026-10-05, unit A): rail 3 ms / charged 34 ms, TMC 0x11, `vsense`
      1, switch witnessed, homed, **5 of 5 at 2.0 counts**, reversal dead band
      89 µsteps as on board 4 (mechanism, not board), `L` 1000.1 Hz but worst
      loop **1082 µs** over the I2C DAC (v0.3 SPI: 386–527 µs). Numbers in the
      PCB bring-up log.
- [x] After A6 (2026-10-05, unit A): `PyControlLinkQp` on
      `mouse_joystick_qp_rev2_usb`, phases a–e, no asserts; tick cost 305/357 µs
      holding with the angle output on (I2C), 15/52 idle, 116/1081 moving;
      host-loss 105.7 ms; 320 moves. Run with a scratch variant of the session
      script that remaps the Digital-50 channels to the 40-pin names the trace
      tools expect (CH1→CH17, CH0→CH16, CH2→CH8, CH9→CH5, CH8→CH2); the
      bench scripts still hard-code board 4, `rev1` and the 40-pin map and
      are owed a serial/env/channel-map parameterisation.
- [x] Digital-50 observation through the LA5032 adapter **v2.0** and the
      Molex 15020 FFC: straight mapping confirmed 2026-10-05, 26 of 32
      channels exercised by 2026-10-06 (the UART pair last, with a pyboard).
- [x] **First contact over the RJ-45 on v1.0 (2026-10-06, 09:59):** the
      delivery build `mouse_joystick_qp_rev2` (protocol on UART0) answered
      `id` from the pyboard's REPL on **Breakout 1.2 port 4 (USART3)** — the
      right joystick's port, which is the one unit A drives — with the full
      v1.0 identity and caps; ports 1 and 3 silent. Then the whole protocol
      over the real link with the rail up: `hv on`, torque live and tared,
      `home`, `arm`, `brake 1.0` → `RELEASED`, `watch` → `NOMOVE` at
      1500.0 ms, `TRIAL` 1 s later with torque, `fetch` with the torque
      column; a 150 ms gap in the REPL script then tripped the 100 ms
      host-loss watchdog exactly as specified (`FAULT`, `hv=0`, `brake=0.0`,
      `nak;faulted` to the next `brake`, `clrfault` recovered). Decoded on
      CH2/CH3. PCB bring-up log, "first contact over the RJ-45".
- [x] **The tick budget, attributed and half fixed (2026-10-06).** Over
      UART0 the primitives drop to a third of the USB build's maxima (X_ENC
      161 µs, snapshot 310, ADS8689 sample 103, `BRAKE_ISENSE` 57–137), so
      the USB protocol port's interrupt-level servicing was the big inflator
      and the `_usb` builds are not where the budget is judged. What remained
      was structural: both hardware backends polled at every scheduler stop
      (the `RELEASED` ring adds one per millisecond), and the `hv on`
      transient latched the decimated mode until `hv off`. Fixed: hardware
      polls reuse their cache within 500 µs (`TmcMotion`, `BrakeTorqueHw`;
      the simulated backends are unlimited so the corpus is untouched), and
      decimation ends after 500 handled ticks under half the budget
      (`LinkAo`). Measured on the Gate E run — see Status.
- [x] **The watchdog runs with the rail (2026-10-06, Peter's call after Gate
      E).** `hv on` arms the host-loss watchdog, `hv off` or a fault disarms
      it, `arm` refreshes, `disarm` leaves it running while the rail is on
      (`ProtocolBoard::cmdHv/cmdDisarm`, `board_sim._cmd_hv/_cmd_disarm`,
      `uart-protocol.md` V1.12 and its tables). Three new corpus cases; two
      older ones rewritten (`event-time-is-64-bit` advances before the rail,
      `home-timeout-above-the-maximum-is-clipped-not-refused` widens the
      watchdog and heartbeats for its eight seconds), their unit-test mirrors
      in both languages likewise. Consequence: a host heartbeats from
      `hv on`; the device class's `hv()` docstring says so.
- [x] **Tare before the watch (same day).** `mouse_joystick_check` sends
      `tq zero` right before `watch` when the board has `torque`, so the rest
      offset (−0.2 to −0.35 N·cm at rest after `RELEASED`) stops tripping
      `touch=0.3`. Bench knobs `v.wdt_ms` and `v.instr` added for the tick
      attribution runs.
- [x] **The tick budget, round three, attributed and fixed (2026-10-06,
      11:04–11:55).** Measured on the LA5032 rather than with the CPU's own
      clock (PCB bring-up log, "the tick budget attributed on the bus and on
      the pin"; scripts `qp-arduino-example/tools/la/`). The bus was never
      the problem: an X_ENC read is 27 µs on the wire and a snapshot 101 µs
      after the library changes, yet the firmware measured 240–430 and
      400–570 µs for them, a `ping` cost the active object 1.2–1.9 ms
      (`LINK_WORK`), and on the pin (`MJ_TICK_LEVEL`) the `RELEASED` tick was
      758 µs median for a few hundred instructions of classifier work. The
      trace off changed nothing; fewer host lines helped only the idle. **The
      cause: every build ran from the W25Q128JVZPIQ through the generic
      single-lane 03h stage 2** (`boot2_generic_03h_4`, kept "until stage 1
      passes" since the first article, never switched). On the quad-lane
      stage 2 matched to the part (tried as `env:mouse_joystick_qp_rev2_fastflash`, then made the default of every env at 14:05,
      `boot2_w25q128jvxq_4`; boots and runs the task first time) the
      `RELEASED` tick is 346 µs median / 450 p90 / 760 max, the `ARMED` tick
      146 / 235 / 370 at 1000 handlers/s, a `ping` 0.5 ms, every `POLL_STAT`
      maximum 2–3× lower, and the sampler holds 1 kHz in `ARMED`. The budget
      constant (400 µs) then decimated the trial on its own, so it is 700 µs
      (measured headroom in `LinkAo.cpp`); **run 9
      (`…-1147-unitA-gate-e-fastflash/`): 1 kHz for the whole run, trial
      included, never decimated, 79 overrun skips and 47 coalesced ticks in
      50 s (from ≈1900 / ≈1500), handler max 798 µs in `RELEASED`, 371 in
      `ARMED`.** Peter made it the default at 14:05: `env:mouse_joystick`'s `boot2_source` is the quad-lane one, every env inherits it, `env:mouse_joystick_safeflash` keeps the generic stage 2 for a first article, and the `_fastflash` envs are gone. Also done on the way, all
      measured: `TMC51X0` working copy with one buffer transfer per datagram
      (two-buffer form on RP2040/Teensy: 17 → 12 µs per datagram, clocks
      back to back) and the pipelined `readMany()` (snapshot 10 → 6
      datagrams), used by `MotionSetup::snapshot()`; `platformio.ini` pins
      `TMC51X0.git#4.2.0` (the library is versioned and release-checked;
      Peter tags and pushes). Open: (1) make the quad-lane stage 2 the
      default for pcb v1.0 (flip `env:mouse_joystick`'s `boot2_source`, or
      the rev2 envs'); every v1.0 board has the same flash; keep the generic
      one for a first article. (2) The `RELEASED` tick is still 2.4× the
      `ARMED` one for little work — code size and the 16 KB cache; a
      `-Os`/`-O2` and section-placement pass (`__not_in_flash_func` on the
      tick path) is the next lever if ever needed. (3) Unit B and the pyboard
      heartbeat rate: 25/s costs 5 % of the ticks ~40 µs; fine.
- [ ] **The park error: lost motion on direction reversal, measured
      (2026-10-06, 14:07–14:25).** Every trial parked 0.99° short on the
      encoder with the ramp generator satisfied. Probed with the bring-up
      task's `park` knob (PCB bring-up log, "the park error measured";
      evidence `…-1407-unitA-park-error/`): the release, the resync and the
      brake are not it (both frames equal and the brake off for 300 ms, the
      move still lands at −0.991); a chain of six moves loses 0.35–0.77° on
      every reversal and ≤ 0.07° on same-direction moves, independent of
      distance, not accumulating. Not the encoder, not the firmware. At
      IRUN 17 instead of 8 the after-positive state is −0.633 three times
      out of three (from −0.82…−0.99) and the hysteresis shrinks by a third:
      ~0.3° is the stepper's torque angle under friction at the bring-up
      current, 0.3–0.6° is drive-train lost motion. Options, in order of
      preference: (1) settle on the encoder after `TARGET` with one or two
      correction moves of the residual — needs a `TARGET` semantics decision
      in `uart-protocol.md`; (2) same-direction approach (overshoot and
      return); (3) `IRUN` toward 17 when the thermal survey allows. Until
      then both tasks home before every trial.
      **Option (1) done 2026-10-07 (uart-protocol.md V1.13), measured on
      unit A (PCB bring-up log, "the encoder settle after a move"; evidence
      `…-2026-10-07-0807-unitA-encoder-settle/`):** `TmcMotion` commands
      moves relative to the encoder, waits after `position_reached` for
      `X_ENC` to stand still (one count for 10 ms, bounded 100 ms), corrects
      a residual above one count up to twice (`Conversions.hpp`
      `kMoveSettle*`), reports `MOVE_SETTLE` in the QP trace; both simulators
      mirror it with the corpus `SLIP` stimulus and three cases (156 Unity,
      188 pytest, all 15 example/env builds). The same probe chain twice: all 16 `TARGET`s within 0.18°
      (before: eight of eight short by 0.22–0.99°), the park at 0.00 and
      +0.08; a reversal costs 150–260 ms more, an uncorrected move 10 ms.
      The rig task's `home_each_trial` default is 0 now.
- [ ] **Settle tolerance, one count or two (2026-10-07).** `MOVE_SETTLE`
      shows the first correction removing most of a reversal's 0.6–1.0° and
      1–2 counts remaining that a 13–25 microstep correction cannot move
      against static friction at IRUN 8 (the ramp frame advances, the rotor
      does not, and the next same-direction move overshoots by what was
      stuck: probe 3 arrived 0.36° past). A two-count tolerance would skip
      the ~100 ms second correction at the price of the last count; at
      IRUN 17 the torque per microstep doubles. Decide with the thermal
      survey; the constant is `kMoveSettleToleranceMicrosteps`.
- [x] **The rig's trial task, dry run (2026-10-07, 08:40):**
      `joystick-rig/feat/mouse-joystick`'s `hardware_definition_joystick.py`
      and `tasks/rig1/joystick_trial.py` with `home_each_trial = 0`, run
      from their checkout (`tools/pycontrol_run.py --hwd-path/--task-dir`,
      the rig's `opto_led.py` symlinked into pyControl's `devices/`) against
      unit A on the right joystick: eight trials, every present and park
      within one count, no fault (PCB bring-up log, "the rig's trial task,
      dry run"; evidence `…-0840-unitA-rig-trial-dryrun/`). The pyboard's
      flash was full of device files pyControl never deletes
      (`pycontrol/README.md`, "A fourth thing"). What remains for the rig
      session is the solenoid cable on port 5 and the researcher.
- [x] **The reward path and unit A's pull sign (2026-10-07, 08:53–08:59):**
      with the solenoid on port 5 a hand pull was first classified `PUSH`
      (pull sign +1), the park against the held lever faulted
      `not_following`; `BoardCalibration.hpp` gained
      `{"E4654C159F274534", -1.0}`, and the next pull was `PULL` → the
      task's 50 ms solenoid pulse → `TRIAL outcome=PULL` → park at 0.0
      (PCB bring-up log, "the reward path and the pull sign"; evidence
      `…-0855-unitA-solenoid-pull/`). Unit B's sign is to be measured the
      same way (a hand pull during `WATCH`; `PUSH` means flip it).
- [ ] **A lever held through the park faults the board (2026-10-07).**
      Correct for the stall detector, fatal for a session: `TRIAL` comes 1 s
      after the outcome, the task parks, a held lever stalls the drive at
      IRUN 8 → `not_following` → rail off. Decide with the researcher
      (`uart-protocol.md` V1.9): the board defers the move while the torque
      says the lever is held (a `nak;busy` or a wait), or the task recovers
      (`CLRFAULT` → `HV ON` → `HOME`). The rig task has no recovery path yet.
- [ ] **One `badverb` 5 ms after a `FAULT` (2026-10-07, `pull1_60s`):** the
      task's first `faults` request was refused `nak;326;badverb`, the retry
      was acknowledged; the verb dispatches by exact match. Most likely a
      corrupted byte on the way in around the switched-domain collapse; no
      capture was running. Capture CH3 (board RX) at the next fault before
      changing anything.
- [x] **Unit B (board 7, `E4654C159F404534`, the LEFT joystick), stages 1
      through A5 (2026-10-07, 09:07–09:31)** after Peter's capacitor rework:
      20/0 self-test checks, two direct-LATCH power-ups with 5/0 register
      checks and 29.9 mA idle, the angle DAC with `GAIN` read back
      (0.829/1.649 V mid/full on the monitor), the left joystick's switch at
      the positive end and active high with the encoder tracking (no config
      flip), homed, homing spread 9 counts (runs 3–5 within one), live loop
      350 µs, brake sweep linear to 49.11 mA with the same silent 128 %
      full-scale overshoot as unit A, torque 103 µV rms at 1000.1 Hz. The
      second LA5032 (`conn=1.67`) is on its Digital-50, straight cable, map
      proven by beacon. PCB bring-up log, six new unit B sections; evidence
      `…-0908-unitB-bringup/`. **Gate E passed on Breakout port 1 at 09:37**
      (the same sequence as unit A's to tens of ms), and the hand pull was
      `PUSH` then `PULL` once `{"E4654C159F404534", -1.0}` was in the
      calibration table: both joysticks pull toward decreasing angle, the
      left torque sensor reads it negative. Both boards pass under one
      hardware definition on their own ports; a task driving both at once is
      the researcher's.
- [x] **The rig's files fit the pyboard the way the GUI uploads them
      (2026-10-07, 10:30).** Measured: 33.8 KB free after "Load framework",
      the six drivers the rig definition pulls in take 22.3 KB, so the
      hardware definition and task must fit in ~10 KB verbatim; the
      originals (15.5 KB, 95 % comments; 10.2 KB, 47 %) failed on the
      definition at 0 bytes free. On the rig branch the definition's prose
      moved verbatim to `notes/hardware_definition_joystick_notes.md`
      (the `.py` is 2.1 KB, code AST unchanged), the task's long header to
      `notes/261007_mouse_joystick_researcher_guide.md` (6.9 KB, plus a
      `clear_fault()` at `run_start`); the GUI path then leaves 7.7 KB
      free. The researcher guide is the deliverable for the session.
- [x] **400-trial soak (2026-10-07, 10:40–11:14):** the rig task on the
      right joystick unattended, 400 `NOMOVE`, 0 `nak`, 0 `FAULT`, every
      present and park within one count (park 0.0 × 399), period 5.08 ±
      0.02 s with no drift, 53 190 link lines (PCB bring-up log, "400 trials
      unattended"; evidence `…-1040-unitA-soak/`). Not exercised: a lever in
      a hand, both boards at once, an hour or more; an overnight run is the
      next size up when the rig is idle.
- [ ] **Left joystick: the torque sensor rests 1.55 N·cm above its nominal
      zero** (unit A: 0.07 below), forty times the zero-balance spec; a
      mechanical preload at the sensor or lever. The tare hides it; the
      mounting wants a look before absolute torque is trusted. Also a small
      brake-to-torque coupling (−0.006 N·cm at 49 mA) unit A does not show.
      Peter / the rig.
- [ ] **The uploader's bootloader step fails on unit B** three times out of
      four ("The RP2040 device returned an error: <unknown>", error 157,
      after the 1200-baud touch has put it in BOOTSEL); unit A takes the same
      `pixi run upload` every time. Copying the UF2 to the mounted `RPI-RP2`
      drive works every time. Two RP2040s on the bus, a different hub port,
      or picotool's device pick — undiagnosed; the UF2 path is the
      workaround (`tools/pio_task.py` could fall back to it).
- [x] **Gate E (2026-10-06), passed on v1.0 unit A:** `tools/pycontrol_run.py` runs a task on the
      pyboard headlessly through pyControl's own `Pycboard` API (the GUI's
      path, no GUI); `pixi run -e pycontrol pycontrol-run`. Two things
      pyControl 2.1.1 taught us on the way: (1) its device-file discovery is
      a substring match of class names over file TEXT, comments included,
      recursing without a cycle check, so `MouseJoystick` being a prefix of
      `MouseJoystickProtocol` looped it to a `RecursionError` — the codec is
      now `JoystickProtocol` and the constraint is written in its header;
      (2) the pyboard's 95 KB flash had 36 KB free against 60 KB of uploads
      (our files were 60 % prose, and comment-only mentions of `MCP23017`
      and `Poke` pulled a 9 KB driver along), so `tools/pycontrol_lean.py`
      now generates comment- and docstring-free copies into
      `pycontrol/dist/` (49 → 20 KB), which is what `pycontrol-link` points
      pyControl at and what the rig will ship; `tests/test_lean.py` imports
      them. The bring-up task `mouse_joystick_check` speaks protocol v1 with
      `v.board` selecting the side, and keeps its own `v.phase` because the
      task API has no current-state query (the third lesson; the first run
      on the board died on `get_current_state()` one step after `hv on`,
      rail up, host watchdog not yet armed). **Passed twice** (evidence
      `qp-arduino-example/evidence/2026-10-06-1019-unitA-gate-e/`, `-1037-`,
      `-1043-`): identify → `hv on` (180 ms from cold) → `HOMED` → `arm` →
      `TARGET` −5.041° → `BRAKE 1.0` → `RELEASED` 23 ms later → `TOUCH` →
      `NOMOVE` → `TRIAL` → park → `done`; `run_end` stops, disarms, drops
      the rail. Heartbeats 25/s, none lost. Written up in
      `pycontrol/README.md` and the PCB bring-up log's "pyControl
      Integration". Two observations for the rig: the rest torque reading
      (−0.2 to −0.35 N·cm, the PCB plan's third finding) trips a 0.3 N·cm
      `touch` with nobody there — tare before `watch`; and the park after a
      trial ends ~1° short on the encoder (backlash on the direction
      reversal) — home first when the park position matters (fixed
      2026-10-07, V1.13: the board settles on the encoder after every move).

## Parallel Observability Workstream — Updated 2026-09-30

[docs/observability-plan.md](../docs/observability-plan.md) is the local
`OBS-FW` plan. The shared
[experimental-system roadmap](../../orb-weaver-context/context/experimental-system-observability-plan.md)
owns cross-repository dependencies and stages. This optional path runs alongside
existing delivery, bring-up and hardware-watchdog work; it does not replace
those tasks or commit the application to QP.

The [QP plan](../../qp-arduino-example/docs/observability-plan.md) now qualifies
RP2040 first, using the Earle Philhower core family already used here, on a
separate simple non-actuating fixture. Exact core/fixture versions and all
RP2040 qualification gates remain open. Teensy hardware work is deferred and
nonblocking; retained build regressions remain useful. STM32G0B1CEU6 is a
later second target. Current joystick delivery remains non-QP.

Hardware intent is passive Digital-50 plus a separate optional debug/trace
connector and reusable external RP2040 probe. Combined 60-pin wiring and an
onboard second MCU are deferred. This does not allocate pins or change present
board capabilities; `OBS-IF` and the selected PCB revision own those decisions.

- [ ] `OBS-FW-1`: define representative command/event/completion semantics;
      ready now, coordinate interface/fixture fields with `OBS-IF`/`OBS-HOST`.
- [ ] `OBS-FW-2`: prepare good/fault/gap baseline fixtures and expected outcomes.
- [ ] `OBS-FW-3`: implement bounded observation only after agreeing semantics
      and claiming affected files with the delivery/watchdog agent.
- [ ] `OBS-FW-4`: measure the baseline on a qualified board/capture path;
      hardware-gated, with measurements in the PCB bring-up log.
- [x] `OBS-FW-5` (first half, 2026-10-01): the QP comparison slice exists and
      ran. `examples/PyControlLinkQp` puts `ProtocolBoard` and the motion
      backend, unchanged, inside one QP/C++ active object with a transmit-only
      QS trace on `FW_TRIG1`; the simulated-motion bench build ran the pilot's
      full protocol sequence on the rev 0 fixture, the delivery build
      (`TmcMotion`, UART0) builds and boots into the safe state. Evidence in
      `../qp-arduino-example/evidence/2026-10-01-rp2040-joystick-link/` and
      `.../2026-10-01-rp2040-joystick-firm-qp/`. Owner of the example files,
      `platformio.ini`'s `[qp_env_data]` and the three `mouse_joystick_qp*`
      envs: this workstream. **Still owed before any adopt/defer decision:**
      `TmcMotion` behind the AO with the rail up (the same first supervised
      run `PyControlLink` owes), the UART0 path under QP with a pyControl
      host, the `OBS-IF` marker overlay for `FW_TRIG0`/`FW_TRIG1`, and the v1.0
      pin map. Record the decision here when it is made.
- [ ] `OBS-FW-6`: hand off correlated evidence for a supervised observation-only
      experiment pilot.

Claim step, owner and exact paths before implementation; use the local plan's
file-ownership and evidence checklist. Planning alone closes none of these
items. Preserve selected manufactured-tag/serial context rather than assuming
historical firmware wiring matches current PCB development.

## Status

**2026-10-15, 13:21–13:24, Peter's laptop at Rig 1: the researcher's boards
reflashed and checked with the rig's pyboard.** Both boards' USB and the
pyboard's USB on the laptop, the rig PC's GUI closed, 36 V on, no analyzers.
`pixi run upload examples/PyControlLinkQp mouse_joystick_qp_rev2 …<serial>-if00`
with both boards attached: board 6 (`E4654C159F274534`, right, port 4) took
the 1200 baud touch, verified, rebooted in 9.6 s; board 7 (`E4654C159F404534`,
left, port 1) the same in 4.8 s, the touch honoured first time on a unit
that had refused it elsewhere. `tools/pyb_id_probe.py` over the RJ-45:
`fw=v1.0.0-6-g4213e48` on both ports, `boot=reboot`. Then the controller
recipe's verification step with the rig's own files
(`joystick-rig` `feat/mouse-joystick` `d3d30a8`: `hardware_definition_joystick.py`,
`tasks/rig1/joystick_trial.py`) through `tools/pycontrol_run.py`, two trials
per side: identify, `hv on`, `HOMED` (1.25 s right, 1.32 s left), `TARGET`
(−5.041° right; −4.95° left, one count short as in unit A's 2026-10-07 dry
run), `RELEASED`, `NOMOVE` at +2.0 s, `TRIAL`, park at 0.0 (left −0.091 once);
no nak, fault or resend. pyControl re-uploaded `mouse_joystick.py` (hash
mismatch: `dist/` is one commit newer than the rig branch), so the pyboard
now carries `61162fe`'s device class until the GUI restores the rig branch's
copy; Peter decided to copy `dist/devices/mouse_joystick*.py` onto the rig
branch, committed there the same afternoon with the rig's provenance header
kept (one more GUI upload, then the hashes agree). Record:
`evidence/2026-10-15-1318-rig1-reflash/` (`meta.txt`, upload logs, the probe,
both trial logs and JSON, `scripts/rig1_session.sh`).

**2026-10-14, 09:10, Peter's laptop: the desk rig's boards reflashed.**
Practice run for Rig 1. Laptop setup: `pixi install` fails on this machine
until `CONDA_OVERRIDE_GLIBC=2.41` is in the environment (the Guix-built pixi
does not see Debian's glibc; every `pixi run` needs it too). First build of
`examples/PyControlLinkQp mouse_joystick_qp_rev2` 2 min (toolchain and
framework download); image `v1.0.0-2-g109169f` (HEAD; no code change since
the tag, `git diff v1.0.0 HEAD --stat -- src examples platformio.ini` is
empty). `pixi run upload … /dev/serial/by-id/…<serial>-if00` on board 8
(`E4654C159F504534`): touch, load, verify, reboot, 7.6 s. The same on board 9
(`E4654C159F234234`): 4.3 s, touch honoured. Both re-enumerate as `2e8a:f00a`
with their by-id links; `picotool info -a` in BOOTSEL shows the flash id
and no binary info, and the delivery build's USB banner cannot be caught (the
CDC port appears ~1 s after reboot, after the `if (Serial)` check), so the
only read of `fw` is `id` over the RJ-45. Done at 09:20 once Peter plugged
the desk rig's pyboard into the laptop: `id;1` from the pyboard's raw REPL
(the desk-rig session's `pyb_uart_probe.py`, now `tools/pyb_id_probe.py`)
over port 1 `UART(4)` and port 4 `UART(3)`: both boards answer
`fw=v1.0.0-2-g109169f`, `boot=reboot`, caps unchanged
(`evidence/2026-10-14-0910-deskrig-reflash/`). Rig 1 plan: laptop, both
boards' USB and the pyboard's USB plugged in, one upload per board, then the
probe.

**2026-10-06, later: Gate E passed on v1.0 unit A.** pyControl 2.1.1 on
the PYBV11 ran `mouse_joystick_check` against the board over the RJ-45 three
times through the GUI's own `Pycboard` path (`tools/pycontrol_run.py`), the
whole sequence from `id` to `hv off` (identify, rail, home, arm, present,
`BRAKE` → `RELEASED` in 12 ms, `TOUCH`, `NOMOVE`, `TRIAL`, park). Getting
there: the codec renamed `JoystickProtocol` (pyControl's substring device
discovery), comment-free lean copies in `pycontrol/dist/` (36 KB of pyboard
flash), and the task keeping its own `v.phase` (no current-state query in
the task API). The hardware polls are now gated to one per advance of the
protocol clock, which removed the doubled bus reads and most of the overrun
skips; the sampler still runs decimated (500 Hz) with the rail up because
every bus primitive costs 2–4× its console cost in the link build, and
attribution is the next firmware step (plan item "round three"). The board
is flashed with this build (`mouse_joystick_qp_rev2`, 10:43), rail off.
Host tests 154 Unity + 182 pytest; the two qp rev2 envs build.

**2026-10-06, 14:30.** TMC51X0 4.2.0 released and pinned; the quad-lane
flash stage 2 is the default of every env (`mouse_joystick_safeflash` keeps
the generic one); the park error is measured — lost motion on direction
reversal, part torque angle at the bring-up current, part drive train, not
the encoder or the firmware — with the encoder-settle fix as the next
firmware item; the rig repository has branch `feat/mouse-joystick` with the
device files, the hardware definition (ports 1/4, solenoid to port 5) and a
first trial task. Board: unit A on the default build, rail off. Tests 156
Unity + 185 pytest; all 14 example/env builds pass.

**2026-10-06, 12:00: the 1 kHz sampler is back, and the watchdog runs with
the rail.** The tick budget was never the bus: every build ran from flash
through the generic single-lane 03h stage 2, and the quad-lane stage 2
matched to the W25Q128JV (now the default of every env; `mouse_joystick_safeflash` keeps the generic one) halves to
thirds everything the CPU does (tick 758 → 346 µs median in `RELEASED`,
268 → 146 in `ARMED`, `ping` 1.35 → 0.5 ms). With the budget constant at the
measured 700 µs, Gate E run 9 held 1 kHz through the whole trial. On the
way: `TMC51X0` working copy (buffer-transfer datagrams, pipelined
`readMany`; unreleased, `lib_deps` via `symlink://` until Peter tags it),
one hardware poll per advance, the watchdog armed by `hv on` with blocked
time credited (V1.12), the task taring before `watch`. Board: unit A on the
fastflash build, rail off. Tests 156 Unity + 185 pytest + 53 library; all
twelve example/env builds pass.

**2026-10-06: v1.0 unit A stages A4 and A5 passed with the new
`examples/BrakeAndTorque`** (brake sink 0–48.80 mA linear; ADS8689 alive at
95–112 µV rms; both buses decoded on the Digital-50 taps), **and the
protocol build gained `brake` and `torque` the same day, run on unit A**
(ramped `BRAKE` → `RELEASED` in 11.6 / 15–17 ms, `TQ`, torque in every
field, `NOMOVE`/`TRIAL`/`FETCH` with torque; `uart-protocol.md` V1.11 has
the decisions). Two board findings — the brake step overshoot with a silent
`~{BRAKE_FAULT}`, and the inert `TQ_CAL_EN` switch — are filed in the PCB
plan. Host tests 154 Unity + 178 pytest, every example/env build green.
Open: the sampler's tick budget with the rail up (decimated since
2026-10-05, worse with the new poll; being attributed with `POLL_STAT`), and
the pyboard was not on USB this session, so the UART0 path and Gate E remain
owed. Bench state and next steps: `.agents/handoff.local.md`.

**2026-10-05: HEAD (`ec3b36d`) regressed on board 4 (pcb v0.3) before the v1.0
boards arrive — no firmware regression.** Every stage tool ran on
`mouse_joystick_rev1` and the QP session on `mouse_joystick_qp_rev1_usb`,
non-interactively: stage 1 24 checks / 0 failed; stage 2 rail 133 ms, 5 / 0
registers, SPI taps decoded byte-for-byte; stage 3 switch witnessed, homed,
5 of 5 at a 2.0-count spread, live loop 1000.1 Hz; QP a–e with 320 moves,
release/watch/fetch/stream, host-loss fault 124.7 ms, hv off. Host tests 125,
pyControl tests 168, 15 target builds, qp-arduino resolved from the GitHub
tag. Evidence `../qp-arduino-example/evidence/2026-10-05-1142-board4-regression/`; numbers in the PCB
bring-up log under board 4. **Bench finding, settled:** the reversal lost motion (reversals 6–7 counts at
IRUN 8, 4–5 at IRUN 17; continuing moves 1.00; taps = `X_ENC` within 2 counts;
same on a replacement joystick unit; board 2 had −4 counts in September) is the
stepper's dead band against friction, not the TMC, not firmware, not a defect.
`t` fails because its move away off the switch is ended by the bench console's
follow check, which lacks `TmcMotion`'s grace and second window, and is not checked. Nits, none applied: `SafeState::driveOutput()` drives `~{BUS_SW_EN}` low for 34 µs
at boot (direction set before level; seen on v1.0 unit A's Digital-50 tap);
`measureHvCurrent()` 50 ms discharge wait (`m` fails after a live rail; from a
cold rail it measured 12.9 mA), `W` sweeps
with the driver disabled, the session scripts miss pre-dictionary `DAC_STATE`
records. The UART0 path with a pyboard and Gate E remain owed; no pyboard was
on the bench.

**2026-10-02:** protocol v1 (`proto=2`) implemented on both ends and run end to
end on board 4 with the rail up in the QP bench build; watchdog, breadcrumbs,
overrun guard, 4 MHz SPI, follow rule with grace and resync after enable are
in. The rev 1 angle-output session (phases b–e) is the next bench step. Current
local context, bench state and open items: `.agents/handoff.local.md` (top
block). Cross-repository plan of record:
`../orb-weaver-context/context/mouse-joystick-rig-system-plan.md`.


**Stage 1 passed on hardware, 2026-09-01.** Board serial `E4654C159F2C2534`.
27 checks, 0 failed. Recorded in `../mouse-joystick-pcb/docs/bringup-log.md`.

**The LA5032 capture path and `la5032-instrument-port-adapter` v1.0 are both
validated**, same day, against the boot signature. `TRIG0`/`TRIG1` on LA5032
channels 16/17; cable not mirrored. Recorded in
`../la5032-instrument-port-adapter/docs/bringup-log.md`.

**Stage 2 passed 2026-09-02** on both boards, once the 36 V rail collapse was
diagnosed as `R203` strapping the switch's `LATCH` high and worked around with
`PowerSequence::staircaseStart()`. See
[The 36 V Rail Collapse](#the-36-v-rail-collapse-resolved-2026-09-02).

**Stage 3 passed 2026-09-02** on board `E4654C159F4D1334`, the as-built unit.
The motor turns, the encoder tracks it to 1.000, and homing repeats to 4 encoder
counts. Written over **TMC51X0 4.0.3**, pinned in `platformio.ini`. Procedure and
findings in `docs/motion-bringup.md`; measurements in
`../mouse-joystick-pcb/docs/bringup-log.md`.

**Controller MicroPython upgraded 2026-09-03: 1.22.2 -> 1.26.1.** This cleared a
blocker that had not been recorded anywhere: `devices/uart_handler.py` asserts
on MicroPython >= 1.26, added in pyControl 2.1.1, and `MouseJoystick.__init__`
constructs a `UART_handler`. The bench pyboard was on framework 2.1.1 over
MicroPython 1.22.2, so **the device class could not have been instantiated at
all** — every host test passes against stubs and none of them can see this.
Standard `PYBV11` build; the internal filesystem survived (firmware writes flash
sector 0 and sectors 5+, the filesystem lives in sectors 1-4), verified rather
than assumed. Backup of the board's filesystem taken first, at
`~/pyboard-flash-backup-2026-09-03`. `UART.IRQ_RXIDLE` reads `0x10`, matching
`pycontrol/tests/stubs.py`.

**The rig's controller has not been checked.** The board upgraded is a bench
pyboard carrying a rotary-encoder test task, not the rig's
`hardware_definition_joystick.py`. Expect to upgrade the rig's too.

**Ports assigned 2026-09-03: 1 (`UART4`) and 4 (`USART3`).** All three
UART-capable ports were verified on the live controller — constructed at
115200 8N1 with an `IRQ_RXIDLE` handler accepted, achieved baud 115384 (+0.16%)
on `UART4` and `USART3`, 115226 (+0.02%) on `USART1`. Ports 2, 5 and 6 report
`UART = None`. `pycontrol/hardware_definitions/mouse_joystick_bringup.py` now
assigns ports 1 and 4.

**This depends on one rig change that has been requested but not confirmed.**
Peter asked the researcher on 2026-09-03 to move the reward solenoid from
`port_4.POW_A` to `port_5.POW_A`. Until that is reported done and
`joystick-rig`'s `hardware_definition_joystick.py` on `source/main` reads
`port_5.POW_A`, **port 4 is contested** — a solenoid cable still in port 4 means
the `right` board's connector is occupied, and the symptom is a link that never
answers rather than an error. Verify before blaming the protocol.

Why port 3 was given up: it is the **only** Breakout 1.2 port with I2C on
`DIO_A`/`DIO_B` *and* a free `DIO_C`, which is what `LaserLickPort` needs. That
device was converted from UART to I2C (ATtiny 0x15 + MPR121 0x5A share I2C bus 1;
ATtiny PB1 drives `DIO_C`/`X5`), and port 4's `DIO_C` is `X6` = `DAC_2` = the
live rsync — so the lick port has exactly one home and cannot move. The solenoid
needs only a driver line and all six ports have `POW_A`, so it is the thing that
moves. Ports 1 and 4 also give both boards an identical achieved baud, both
being on APB1.

**On port 4, RJ-45 pin 5 must stay unconnected.** It is `DIO_C` = `X6` = the
rsync driver. Rev 0 leaves it open, which is correct; driving or pulling it would
fight the DAC and corrupt the alignment fingerprint the recording pipeline
depends on. Full reasoning:
`../mouse-joystick-pcb/docs/pycontrol-port-pinout.md`.

**Gate C's firmware half written 2026-09-03, and Gate C's corpus requirement is
now met by two independent implementations.** `src/Protocol.hpp` is the framing
codec and `src/ProtocolBoard.hpp` the command layer and state machine; both are
Arduino-free and allocation-free, which is what lets
`test/native/test_protocol/test_protocol.cpp` run the shared corpus on the host.
54 native tests pass, up from 33. All three examples still build for
`mouse_joystick`.

**Mutation-testing the corpus was the part that paid.** Four deliberate defects
were injected to check the tests had teeth; two were not caught at first:

- Inverting the ARMED and range checks in `MOVETO` passed every corpus case, so
  the corpus *documented* a safety-relevant ordering it did not *enforce*. Added
  `armed-is-checked-before-the-range-check`, which both implementations now pass.
- A test asserting that malformed frames do not refresh the host-loss watchdog
  advanced a full extra timeout at the end, tripping the watchdog either way. It
  passed against a board that refreshed on garbage. Margins tightened.

All four mutations are caught now. Neither gap was findable from one
implementation plus its own fixture.

**Standing requirement for firmware integration: event time must not be a raw
32-bit microsecond counter.** `ProtocolBoard` keeps time in `uint64_t` because
`micros()` wraps at 71.6 minutes, which is shorter than a session, and every
event on this link carries `t_us` into a recorded dataset used for alignment. A
wrap would not fail loudly — it would produce timestamps that go backwards.
Whatever feeds this from `micros()` must accumulate the wrap.

**Not yet done:** none of this is wired to a UART or to real motion. Motion in
`ProtocolBoard` is the simulator's linear model at `v_max`, and its defaults are
the simulator's placeholders (travel +/-20 deg, 180 deg/s), **not** the measured
bench values. Replacing them changes corpus timings and belongs in its own
commit with the corpus updated alongside.

**Protocol layer wired to a real UART and real motion, 2026-09-03.** Four new
firmware pieces, and one seam that made them possible:

| File | What it is |
| --- | --- |
| `src/MotionBackend.hpp` | The seam. `ProtocolBoard` owns framing, the state machine, the watchdog and the cache, and owns **no** motion. `SimulatedMotion` (host, injected clock) runs the corpus; `TmcMotion` (firmware) drives the real thing. |
| `src/TmcMotion.hpp` | Non-blocking motion over the TMC5130A, plus the 36 V rail and SPI bring-up. One `snapshot()` per poll, because `RAMP_STAT`'s flags are read-and-clear. |
| `src/PycUart.hpp` | UART0 on the RJ-45, and a 64-bit clock accumulated from `micros()`. |
| `examples/PyControlLink` | The two wired together. 61 native tests pass, all four examples build. |

**Why a seam rather than calling `MotionSetup` directly:** `MotionSetup::moveTo()`
and `homeToSwitch()` are blocking — they return a report once the motion is
done. Right for a bench console, wrong here: the protocol must ack immediately
and keep servicing the link for the whole move, and a 2.5 s blocking home would
silently fail every heartbeat inside a 100 ms timeout. `TmcMotion` drives the
same primitives in the same order without waiting.

The seam also grew two things real hardware needs and the corpus does not:
`consumeFault()`, so a driver over-temperature or an unexpected end stop becomes
a latched fault and a `FAULT` event with no host command in flight; and
`setHvEnabled()`, because the 36 V domain exists to feed the driver and a rail
that fails to come up is a fault rather than a nak. Both are defaulted, so the
corpus behaviour is untouched, and both are tested with stub backends.

**NOT YET RUN ON HARDWARE, and the first run should be supervised.** Stage 3
validated the blocking implementations; this is a faithful rearrangement, which
is an argument and not a measurement. Unproven: the non-blocking homing state
machine, and the travel limits derived from bench geometry rather than exercised
through the protocol.

**Homing over the protocol is deliberately SINGLE-STAGE.** The bench measured
two-stage and single-stage repeatability as the same 2-5 encoder counts; the slow
creep buys datum *accuracy* (89 microsteps of offset at 0.200 rev/s against 4.5
at the creep) and a faster primary approach, not precision. A single stage is a
small amount of code whose failure modes are bounded by the chip's own
reference-switch stop; a blind reimplementation of the validated two-stage
sequence would not be. **Follow-up: two-stage homing over the protocol**, with
the 89-microstep offset as the reason it matters.

**First protocol contact on hardware, 2026-09-03: `id` passed with the 36 V
supply off.** Board `E4654C159F591D34` through Breakout 1.2 port 1, peer driven
from the pyboard REPL with `machine.UART(4)` directly -- no framework, no
hardware definition, no device class, for the same reason stage 2 used raw SPI
datagrams: first contact should not have the framework as a second unknown. The
reply also closed a standing gap by announcing its own serial, so **the left
joystick is board `E4654C159F591D34` and the right is `E4654C159F4D1334`**.
Measurements in `../mouse-joystick-pcb/docs/bringup-log.md` under board 3.

**Two firmware bugs found by that run, both fixed.**

1. **`STATUS` hung the core.** `TmcMotion::angleDeg()`/`xactualDeg()` read
   position over SPI and `STATUS` calls both, so a `status` before `hv on`
   reached for SPI0 before `beginSpi()` had run with the bus switch open. The
   board then stopped answering the link *and* stopped honouring the 1200 baud
   reset touch; recovery needed `SW501` through a power cycle with both USB and
   the RJ-45 unplugged. Fixed: position is a cache refreshed once per `poll()`
   and nothing touches SPI unless `bus_up_`.

2. **`ID` reported the corpus placeholder serial.** The firmware never set it
   from the flash unique id, so every board would have reported
   `0000000000000000` and a host confirming it was talking to the right board
   would have passed against the wrong one. Default is now unmistakable;
   regression test added.

**THE LESSON, which is worth more than either bug.** The corpus could not have
caught the first one. `status-is-safe-before-anything-is-configured` passes on
the host because `SimulatedMotion` just returns a number. **A corpus held by two
implementations pins protocol behaviour, not hardware preconditions** -- one of
the two has no hardware. A green corpus means "the two ends agree about the
wire", not "the firmware is right". Anything the firmware does that the host
model cannot express is unverified by construction, and the SPI-before-bus class
is exactly that.

A latent hazard was closed at the same time: the USB mirror could block when a
host connected and stopped reading, stalling the link, the motion poll and the
watchdog together. It is now dropped rather than waited on -- losing a
diagnostic line is free, losing the link is not.

**Stage 4 ran the full motion path on hardware 2026-09-03**: `hv on`, `home`,
`arm`, `moveto`, `disarm`, `clrfault`, all over the RJ-45 with the motor moving.
Homing repeats to about one encoder count; the commanded-move follow ratio is
0.991. ~150 byte-identical `ping;9` were each answered and each refreshed the
watchdog, which is open question 1's resolution demonstrated on hardware.
Measurements in `../mouse-joystick-pcb/docs/bringup-log.md` under board 3.

**Eight firmware bugs were found by hardware in one session**, none of which the
shared corpus could have caught: the placeholder serial, `STATUS` hanging on SPI
before the bus existed, a `limit` field nothing assigned, the missing encoder
follow check, the homing driver-enable deadlock, a fault that dropped the flag but
not the rail, the loop judging time before reading the link, and the datum offset
below. That ratio is the point, not an accident -- see the corpus-limits note
above.

**A correction to the VALIDATED stage 3 code, not just the new firmware:
`ENC_LATCH` is permanently 0 on rev 0.** It latches on the encoder's N event and
rev 0 leaves the index no-connect with `ENCN` terminated by `R1201`. So
`MoveReport::latch_encoder` and `HomeReport::coarse_encoder`/`fine_encoder` --
all filled from that register -- are structurally zero on this hardware. Any
stage 3 report showing one should be read as "no data".

**RESOLVED: the position-frame offset.** Peter's fix -- zero BOTH frames after
the clearance move rather than at the switch edge -- removed it and did more
besides. `moveto -8.0` now reads back `angle=-8.016` against `xactual=-7.995`,
agreeing to 10 microsteps; `ev;HOMED` reports `angle=0.0`, which is what the
shared corpus expects, so firmware and corpus stopped disagreeing on a pinned
value; zero became a commandable position instead of one the travel limits
exclude; and travel is now -10.125..0.0 deg. `XACTUAL` is still re-based to
`XLATCH` first so the clearance is measured from the switch edge, which is
velocity-independent where the stop position is not.

**Also fixed: the approach keyed off `status_latch_*`, which is read-and-clear.**
A long approach reached the switch (`limit=1`) and then hung until the 8 s
timeout. `status_stop_*` are plain status bits that survive polling, and the
validated `homeToSwitch()` already keys off those with an explicit `XLATCH`
fallback -- the non-blocking version had no fallback. And `ev;HOMED` was
reporting the cache from before the datum writes.

**WITHDRAWN: the "host must poll faster than the draft implies" measurement.**
The two observed `host_timeout` faults were the test harness, not the firmware:
MicroPython's `UART.read()` blocks for its configured `timeout` on an empty
buffer, so a `timeout=50` made "20 ms pings" 72 ms and "40 ms pings" ~90 ms
against a 100 ms watchdog. The loop-ordering fix stands on its own merits -- the
watchdog was genuinely evaluated before the link was read -- but the margin is
unmeasured. Set the UART read timeout deliberately before timing this link.

**`freewheel` works on hardware 2026-09-03, and so do both exits.** The
compliant state entered from ARMED, `status` reported `FREEWHEEL`, and the
encoder moved 0.133 deg while the ramp generator did not -- the divergence that
makes `angle` and `xactual` two quantities, observed without touching the
mechanism. `moveto` re-engages implicitly, `stop` takes the joystick back,
`disarm` leaves it held, and after `disarm` both frames read identically.

**Bug found: leaving FREEWHEEL never re-energised the driver**, despite a comment
claiming the move "re-engages implicitly". All three exits were separate
omissions and now share one `leaveCompliance()`. **Third time the encoder follow
check has caught a state transition that forgot to energise what it was about to
use** -- the other two were homing and, indirectly, the same pattern at `arm`.
That check has paid for itself repeatedly and should not be weakened.

**OPEN: sub-degree discrepancies at the release/re-engage boundary.** After
`freewheel` -> `moveto`, `xactual=-2.995` against `angle=-3.691` (0.70 deg);
after `freewheel` -> `stop`, 0.27 deg. Leading hypothesis is the rotor snapping
when the driver re-energises -- a stepper's electrical angle repeats every four
full steps, bounding it at ~+/-3.6 deg here, and both figures fit. The candidate
fix is to re-enable BEFORE resyncing so the resync captures the post-snap truth,
**deliberately not done** because energising against a stale `XACTUAL` would aim
the snap at where the ramp generator thought the joystick was. Needs a
measurement: park, freewheel, hand-move a known distance, read both frames before
the re-engage, immediately after, and after the resync.

**Stage 4's rev-0 protocol surface is fully exercised on hardware, 2026-09-03.**
`PULL` fires at the crossing (-8.016 against an armed -8.0, twice), and the raw
encoder edges captured on the InstrumentPort **corroborate the driver's `X_ENC`
to 3-8 counts on a ~15 deg hand pull** with zero invalid transitions. So every
"anomalous" encoder reading earlier in the session was real motion, and the
register can be trusted. Measurements in
`../mouse-joystick-pcb/docs/bringup-log.md` under board 3.

**The identity beacon earned itself.** The analyzers were power-cycled to clear a
wedged capture and came back in opposite USB ports, silently inverting the
topology-to-board mapping while the probe cables stayed put -- a capture was
taken on the wrong instrument before Peter noticed the wrong analyzer blinking.
The beacon re-established the mapping in one shot. **`examples/PyControlLink`
now emits it at boot**, because the `I` command exists only in the bench
consoles and the protocol firmware was the one case where identification was
impossible without reflashing.

**Bench consoles can wedge on USB CDC output.** `MotionAndHoming` blocked its
own loop on a `Serial.print` with no host draining, which enumerates fine but
stops answering and stops honouring the 1200 baud reset -- recovery needed
`SW501`. `PyControlLink` is guarded via `usbMirror()`; the three consoles are
not. Worth the same guard if they are touched again.

Read `.agents/handoff.local.md` for bench state and the immediate next actions.

## Milestones

- [x] Pixi + PlatformIO environment, mirroring the `TMC51X0` layout
- [x] `src/BoardPins.hpp` derived from `U501`'s pad-to-net map at PCB tag `v0.1`
- [x] Confirmed PCB tags `v0.1` and `v0.2` have byte-identical RP2040 pad maps
- [x] `SafeState` — safe idle without energising anything
- [x] `examples/RP2040SelfTest` — clocks, flash, RAM, pads, pull resistors, ADC
- [x] Boot signature and heartbeat on `FW_TRIG0`/`FW_TRIG1`, so firmware
      execution is observable without USB
- [x] `MJ_BOARD_REV` so the spare-pad test matches the revision being built
- [x] Compile clean on `mouse_joystick`, `mouse_joystick_safeflash` (was `_fastflash`: the quad-lane stage 2 is the default since 2026-10-06),
      `mouse_joystick_rev1`; 7/7 host tests pass
- [x] BOOTSEL enumerates (`RPI-RP2` mass storage) — passed 2026-09-01
- [x] Firmware flashes, verifies, and runs — **27 checks, 0 failed**, 2026-09-01
- [x] Results recorded in `../mouse-joystick-pcb/docs/bringup-log.md`, 2026-09-01
- [x] LA5032 channel map confirmed against the boot signature, 2026-09-01, and
      recorded in the adapter's own bring-up log
- [x] Stage 2 written: `examples/TMC5130Communication`. Builds clean, plus 8 host
      tests pinning the register decoding and the expected-IOIN claim.
- [x] Stage 2 run on hardware 2026-09-01 — reached gate 2 and stopped there
- [x] 36 V power chain shown to work at least momentarily: rail up in 3 ms to
      3.309 V, measured
- [x] **36 V rail collapse resolved 2026-09-02** — `R203` straps the switch's
      `LATCH` high, so it cannot auto-retry through the inrush. Worked around by
      `PowerSequence::staircaseStart()`; the real fix is a board change, tracked
      in `../mouse-joystick-pcb/.agents/PLANS.md`
- [x] Stage 2 gates 4 and 5: bus switch closed, TMC5130A `IOIN` read, both
      boards, 2026-09-02
- [x] **Stage 3: motion, encoder and homing, passed 2026-09-02** on board
      `E4654C159F4D1334`, with `vsense = 1` set and read back before the output
      stage was ever enabled
- [ ] Resolve whether the motor's 0.6 A rating is RMS or peak, and set the run
      current ceiling deliberately. Needs a coil-current measurement
- [ ] Run stage 3 on board `E4654C159F2C2534`, whose empty-connector
      `REFL_STEP` = 1 is still unexplained
- [x] **Stage 4 host environment, 2026-09-02.** `pycontrol` pixi environment
      (`no-default-feature`, so no PlatformIO), `tools/pycontrol_task.py`, and
      `pycontrol/`. Resolved and locked for linux-64, osx-64, osx-arm64 and
      **win-64** — the rig PC is Windows 11, which is what ruled out a Guix-only
      environment. Verified: PyQt6 6.11.0 / pyqtgraph 0.14.0 / numpy 2.5.2 /
      pyserial 3.5 against pyControl v2.1.1; 6 environment tests pass; firmware
      env unchanged at 33/33
- [x] Stage 4 host half: framing codec, device class, simulator, 22-case corpus
- [x] Stage 4 firmware half: `src/Protocol.hpp` + `src/ProtocolBoard.hpp`, held
      to the same corpus by `test/native/test_protocol/`. 2026-09-03
- [x] Stage 4: protocol layer wired to a real UART and real motion —
      `MotionBackend` seam, `TmcMotion`, `PycUart`, `examples/PyControlLink`.
      2026-09-03, **not yet run on hardware**
- [ ] Stage 4: two-stage homing over the protocol (single-stage today)
- [ ] Stage 4: pyControl UART protocol on hardware (Gate E)
  - [x] **Host half written and green, 2026-09-02.** 128 tests.
        `pycontrol/devices/mouse_joystick_protocol.py` (codec, runs on the board
        and under pytest), `pycontrol/sim/board_sim.py` (simulator with an
        injected clock), `pycontrol/devices/mouse_joystick.py` (device class,
        asserts on `port.UART` and on duplicate `event_name`),
        `pycontrol/tests/fixtures/protocol_cases.txt` (22-case shared corpus),
        plus a bring-up hardware definition and task
  - [x] **Narrowed to rev-0 hardware, 2026-09-02.** `ANGOUT`/`ANGSCALE` removed
        (analog outputs not populated); `FREEWHEEL` and a compliant state added;
        `STATUS` now reports `angle` (encoder) and `xactual` (ramp generator)
        separately; encoder resync on leaving compliance only. Findings appended
        to `../mouse-joystick-pcb/docs/uart-protocol.md` as open questions,
        without changing its command table
  - [ ] **Protocol framer/parser in `src/`, and a host-native test that
        reproduces `protocol_cases.txt`.** Until this exists Gate C is NOT
        passed: a simulator agreeing with a corpus only it has been checked
        against says nothing about the RP2040. The corpus is line-oriented so
        the C++ side is `getline` plus a prefix compare, no JSON library
  - [ ] Resolve the six protocol findings below before freezing the draft
  - [ ] Gate E on the Breakout 1.2, then deliver to `joystick-rig`

## Protocol Findings, 2026-09-02

Raised by implementing `../mouse-joystick-pcb/docs/uart-protocol.md` (Draft
v0.1). All six are now written up in that file under "Open Questions From The
First Implementation", which is where they should be resolved - this repository
implements the protocol and does not define it. Summarised here because they
gate Stage 4.

1. ~~**The retransmission rule breaks the heartbeat.**~~ **RESOLVED
   2026-09-03**, and not by exempting `PING`. It was a layering error: the
   watchdog measures host liveness, so it is now reset on **receipt of any
   well-formed command**, before dispatch and regardless of a cache hit —
   `badframe` excepted, so a wedged serial driver cannot hold the safety
   timeout open. The cache is separately restricted to state-changing commands,
   classified on the *parsed* command because `CAL`/`INSTR`/`HV` are each a read
   or a write depending on their argument. Cache bounded at 2. Note **`MOVE` is
   the only non-idempotent command in the set**, so the cache guards one verb
   and could go entirely if the trial uses only absolute positioning.
   Implemented in `pycontrol/`; the corpus did not change. Owned by
   `../mouse-joystick-pcb/docs/uart-protocol.md`, open question 1.
2. **The 100 ms host timeout is shorter than a long move.** At 180 deg/s an 18
   degree move consumes the whole timeout, and travel is 40 degrees wide. The
   host must ping *during* motion. Possibly fine, but it should be written down
   rather than discovered. Confirm the real velocity and the real timeout
   together.
3. **The command set does not match the trial.** The draft models a pull as
   detection (`ARMPULL threshold`); the rig needs it as mechanical resistance -
   brake sets the force, motor gets out of the way. Trial shape is park,
   present, release, pull, present again.
4. **Brake-then-release cannot be two host commands.** The brake must engage
   before drive is released or the joystick drops. Belongs in one firmware
   operation, `BRAKE <torque>`. `FREEWHEEL` is a rev-0 stand-in to be replaced.
5. **`STATUS`'s "position" is two quantities** once there is a compliant state -
   measured and commanded, disagreeing by the whole pull.
6. **Resyncing `XACTUAL` from the encoder deletes the stall detector.** Resync
   only on leaving compliance. Stall detection is unimplemented: it needs a
   fault bit the draft does not have, and bit 4 is retired for a different
   comparison.

Update 2026-10-01: findings 3, 4 and 5 are closed by protocol v1 (`proto=2`,
signed off and recorded in `../mouse-joystick-pcb/docs/uart-protocol.md`,
"Protocol v1 Proposal"): `BRAKE <torque>` replaces `FREEWHEEL` (release-only
until a board has the brake), `WATCH` replaces `ARMPULL` with thresholds
relative to the release angle and one `TRIAL` summary per arming, `STATUS`
carries `torque`, `brake` and `t`, and `CFG`, chunked `FETCH` and `STREAM`
are new. Implemented in `src/`, `pycontrol/` and both test runners the same
day, compiled into `PyControlLink` and `PyControlLinkQp`, not yet run on a
board. Finding 6 stands.

## Waiting On The Researcher

Not blocking Stage 4, but they change the command set when answered. Raised
2026-09-02; the researcher has not scoped the task yet.

- What the joystick must do in a trial - whether `MOVETO`/`MOVE`/`ARMPULL` is
  the right vocabulary, or whether a hold-against-force mode is needed
- Whether move speed and motor current should be adjustable or fixed defaults.
  Current stays out of the protocol either way, per the draft's "deliberately
  absent" list; speed would be named profiles, not raw numbers
- Whether rev 0's UART-latency-only event path is acceptable for a pilot.
  `../mouse-joystick-pcb/docs/pycontrol-interface.md`: "fine for exercising the
  interfaces, not fine for behavioural data"
- On heartbeat loss mid-trial, whether latching a fault and requiring `CLRFAULT`
  is the behaviour they want, given it ends the session
- ~~Which Breakout 1.2 ports the two boards get~~ — **assigned 2026-09-03:
  ports 1 and 4.** Requested of the researcher: move the reward solenoid from
  `port_4.POW_A` to `port_5.POW_A`. **Awaiting confirmation that it is done** —
  that is the only thing standing between this assignment and a working link.
  Port 3 stays with `LaserLickPort`, which needs the one port having both I2C
  and a free `DIO_C`. Reasoning in
  `../mouse-joystick-pcb/docs/pycontrol-port-pinout.md`
- Whether one `RSYNC` output can drive both boards - still open in Gate A of
  `pycontrol-interface.md`, and now sharper: Breakout 1.2 has **two** BNCs and
  both are already in use (camera trigger, opto gate), against **six** lines
  wanted at rev 1

## Controller Decided 2026-09-02

**Breakout Board 1.2.** UART-capable ports **1** (`UART4`), **3** (`USART1`),
**4** (`USART3`), from `devices/breakout_1_2.py`. Two boards fit with a port
spare.

This closes the "Which breakout the lab actually deploys" unknown in Gate A of
`../mouse-joystick-pcb/docs/pycontrol-interface.md`, and it means **no
`breakout_h7.py` is needed.** pyControl v2.1.1 declares no `port.UART` for any H7
port, so the H7 breakout would have required writing and upstreaming that driver
first. It does not block anything now.

## Revision Facts Established 2026-09-01

Read from `git show <tag>:kicad/mouse-joystick-pcb.kicad_pcb`, footprint `U501`:

- **`v0.1` and `v0.2` RP2040 pad-to-net maps are byte-identical.** The ambiguity
  over which was manufactured does not affect firmware.
- **HEAD differs on three pins only:** `GP10`, `GP11`, `GP13`. HEAD routes them to
  the rev 1 `DAC80501`; `v0.1` leaves them unrouted. Handled by `MJ_BOARD_REV`.
- **Designators rotated between `v0.1` and HEAD.** The 36 V input block went from
  `U201`/`R20x` to `U1901`/`R190x`. `R1901`/`R1902` are 100 ohm pyControl series
  resistors at `v0.1` and 1 kohm `HV_EN`/`HV_DIAG_EN` series resistors at HEAD.
  All designators in this repository are `v0.1` designators.
- **`v0.1` has three `SN74LVC125A` buffers** (`U1601`, `U1701`, `U1801`), all on
  `+3.3V_MCU`. HEAD has a fourth, `U2101`, for the DAC bus.
- **Every instrumentation buffer input is resistor-held**, including the encoder
  taps (`R805`/`R806` 100 kohm to GND), so enabling the port floats nothing.
- **USB is USB-C** (`J301`, `CC1`/`CC2` via 5.1 kohm), with `U301` USBLC6-2P6 on
  the data pair. MCU power chain: `VBUS` -> `D301` Schottky -> `+5V_MCU` ->
  `U701` TPS73733 -> `+3.3V_MCU`. pyControl's 5 V ORs in through `D1901`.
- **BOOTSEL is the standard circuit:** `SW501` shorts `~{USB_BOOT}` to GND,
  `R505` 1 kohm to `QSPI_SS`, `R504` 10 kohm pull-up to `+3.3V_MCU`.
- **Flash is `U502` W25Q128JVZPIQ**, 16 MiB, matching the build configuration.
- **The TMC5130A's clock is an external 16 MHz oscillator `X1201`, powered from
  `+3.3V_SW`.** It is dead until `HV_EN` is raised — a stage 2 consideration.

## Confirmed On Hardware 2026-09-01

Board serial `E4654C159F2C2534`, 36 V absent, USB only.

- **The `generic` board definition boots this board** with
  `boot2_generic_03h_4`. `mouse_joystick_fastflash` is untried on hardware. (2026-10-06: tried, measured, and made the default; the generic one is `mouse_joystick_safeflash` now.)
- **Flash is really 16 MiB.** No aliasing at +2, +4 or +8 MiB, and `picotool`
  verified the whole image byte for byte.
- **Clocks are nominal:** `clk_ref` 12000 kHz, `clk_sys` 133000 kHz measured,
  `clk_peri`/`clk_usb`/`clk_adc` 48000 kHz.
- **Crystal `Y501` absolute error is below this method's noise floor** — −18.7 ppm
  apparent over 9 s against host `time.monotonic()`, where USB CDC jitter alone
  is order 110 ppm. Consistent with a healthy +/-30 ppm part.
- **All four resistor-defined levels are correct:** `R601` 7.87k holds the GPIO
  bus isolated at reset, the buffer OE pull-ups hold the instrument port silent,
  `R403` holds pyControl RX idle high, `R205` is present with no latched fault.
- **All 12 spare pads move both ways.** No solder defects on any unrouted GPIO.
- **`+3.3V_SW` reads 0.034 V with `HV_EN` low** — the switched domain is genuinely
  off and `R401`/`R402` hold it there. `HV_SNS` 0.016 V. Die 26.2 C.
- **The 1200-baud touch works**, so reflashing needs no button and no physical
  access to the board.

## Still Unconfirmed

- ~~**The LA5032 channel mapping.**~~ Confirmed: the rev 0/1 map through the
  adapter v1.0 carried the stage 1 to 3 captures, and the v1.0 Digital-50 map
  through the adapter v2.0 was confirmed straight (`CHn` observes `DIGn`) on
  2026-10-05 (bring-up log, unit `E4654C159F274534`).
- `env:mouse_joystick_fastflash` on hardware. (Done 2026-10-06, see the tick budget item.)
- Everything on the far side of `HV_EN`: the switched rails, the TMC5130A, the
  16 MHz oscillator `X1201`, the motor, encoder, and limit switch.

## Known Documentation Conflict

`../mouse-joystick-pcb/docs/power.md` says `HV_SNS` and `~{HV_FAULT}` "sit
behind" the GPIO isolation bus switch and cannot be read until firmware enables
it. The GPIO table in `docs/interface-map.md` marks both `byp`, and the board's
pad-to-net map agrees at both `v0.1` and HEAD: `GP28`/`GP29` reach the 36 V
switch directly, with no underscore-suffixed twin net. `power.md` is stale on
this point. This firmware follows the netlist. Worth fixing in the PCB
repository.

## Build-System Trap, Fixed 2026-09-01

`lib_deps = src/MouseJoystickBoard` — a bare local path — made PlatformIO copy
the directory into `.pio/libdeps/` and never refresh it, so edits to the
library's `.cpp` files were silently not compiled and builds reported success
while testing stale code. Several "passing" builds during initial development
were affected. Fixed by declaring `symlink://src/MouseJoystickBoard` and adding
`src/MouseJoystickBoard/library.json`, which `symlink://` requires. Verified with
a `#error` canary.

**`TMC51X0` uses the same bare-path arrangement** (`lib_deps_local = src/TMC51X0`)
and so has the same trap. Worth fixing there.

## The 36 V Rail Collapse (resolved, 2026-09-02)

The blocking issue. Full detail and raw data references in
`.agents/handoff.local.md`; the durable summary:

**Symptom.** `HV_EN` high with 36 V on `J201`: `+3.3V_SW` reaches 3.309 V at
t=3 ms, holds rock steady, collapses at t=56-57 ms and stays down. Repeatable to
±1 ms. Captures in `../mouse-joystick-pcb/docs/captures/2026-09-01/`.

**Ruled out.** A resistive short — `+36V_SW` to `GND` measures 0.87 MΩ with a
DMM, board unpowered. A too-low bench supply limit — the supply was not limited
and can deliver 4 A. Note the DMM result does not rule out a *voltage-dependent*
load.

**Leading hypothesis.** Capacitive-charging thermal shutdown in `U201`, which
`tps1htc100-q1.pdf` explicitly warns about. Charging `+36V_SW`'s ~208 µF to 36 V
costs ½CV² = 135 mJ **independent of the current limit**, `T_REL` is 60 °C, and
`R203` ties `LATCH` high which disables the auto-retry TI says is acceptable for
high-impedance loads. 0.87 MΩ is a high-impedance load.

**Not yet distinguishable.** Whether `U201` turned off or `U901`/`U1001` shut
down. Only `+3.3V_SW` is instrumented; `+36V_SW` and `+5V_SW` reach neither the
MCU nor the InstrumentPort. **This cannot be resolved from firmware** — it needs a
scope directly on those two rails. Do not add more ADC reads hoping to close it.

**Trap.** `HV_SNS` at `VSNSFH` does not identify a fault. Per Table 7-2 that
level is shared by overcurrent, overvoltage, short-to-ground and thermal
shutdown, and with `EN` low it is also OFF-state open-load. An earlier diagnosis
of "continuous 1.15 A load" came from misreading it and was wrong.

**If confirmed, the fix is a board change, not firmware:** reduce bulk
capacitance, raise `R206` to *lower* `I_CL` and spread the same energy over more
time, or free `LATCH` for auto-retry. Raising `I_CL` does not help.

## Stage 3, Passed 2026-09-02

Full procedure, measurements and the reasoning behind each gate are in
`docs/motion-bringup.md`. The durable summary:

**It uses TMC51X0.** Stage 2 deliberately did not, because first contact with a
bus should not also be first contact with a library. Stage 3 needs a ramp
generator, an encoder scaler and homing, TMC51X0 has all three, and the bus was
already proven. Pinned to tag `4.0.3` in `platformio.ini` and declared again in
`src/MouseJoystickBoard/library.json` — `lib_ldf_mode = off` means PlatformIO
will not find it from an `#include`.

**`src/MotionSetup.hpp` is glue, not a driver.** It holds only what the library
cannot know: the 16 MHz external clock, the 0.15 ohm shunts, the motor's rating,
the encoder's resolution, the rig's travel, and `CHOPCONF.vsense`.

**The one bit that matters.** TMC51X0's `Registers::Chopconf` has no `VSENSE`
field — it follows the TMC5160 layout, where bit 17 does not exist. On a
TMC5130A that bit is `vsense`, and with 0.15 ohm shunts it is a factor of 1.8 in
motor current. `IHOLD_IRUN` is write-only, so the current scale can never be
read back; `CHOPCONF` is the readable half of the decision. Firmware sets bit 17
with a raw register write, reads `CHOPCONF` back, and refuses to enable the
output stage unless the read-back says 1. **It does not survive a device reset
or `recoverFromDeviceReset()`** — re-assert and re-verify after any power cycle
of the switched domain.

**Measured facts about the rig**, all of which were unknown before:

- Travel is **~4000 microsteps, 28 deg at the motor shaft**: the limit switch
  bounds the positive end, a mechanical stop bounds the negative end. Every
  bring-up distance has to fit inside it.
- The limit switch is **active high** (`REFL` rests low, goes high when hit),
  throw 256 microsteps, and it is at the **positive** end of travel. It is wired
  to `REFL`, which stops *negative* motion, so `SW_MODE.swap_lr` is required or
  the hardware stop guards the wrong direction.
- The encoder **counts down as `XACTUAL` counts up**. `ENC_CONST` resets to zero
  so `X_ENC` does not count until it is written, and the negation is
  **−13 + 0.2**, not −12 and 0.8, because the fraction is always added.
  Bench-confirmed: the other encoding gives a ratio of 0.875 instead of 1.000.
- The switch's reporting delay is **8.74 ms**, measured by taking two
  closing-edge latches at two approach velocities in one position frame. No
  instrument needed, and it is what justifies two-stage homing.
- **Homing is two-stage**: a fast approach latching the closing edge, then a
  slow creep off the switch latching the **release** edge, which is the datum.
  It buys **accuracy, not precision** -- the datum stops depending on approach
  velocity (89 usteps of offset at 0.200 rev/s versus 4.5 at the creep) and the
  primary approach can run four times faster. Repeatability is **2-5 encoder
  counts in both schemes** across four series of five runs, and the rig cannot
  distinguish them at n = 5. An earlier claim that the second stage halved the
  scatter came from one lucky series and is retracted: a repeatable 8.74 ms
  delay is a systematic offset, and systematic offsets do not widen spread.

**Every move is bounded and encoder-watched.** `MotionSetup::moveTo()` targets a
position rather than a stop condition, and compares `X_ENC` against `XACTUAL` —
the ramp generator advances whether or not the shaft follows, so that comparison
is what detects an end stop, a stall, a slipping coupling or a dead encoder. It
found the negative end stop on the first attempt without straining anything.

## Firmware Traps Found On Hardware, Stage 3

All fixed; do not reintroduce. All four were invisible to a compile and to the
host tests, and three would have moved the motor when nothing asked.

- **`RAMPMODE` = hold does not stop the motor**, it freezes the current
  velocity. TMC51X0's `Controller::initialize()` runs during `setupSpi()` and
  leaves `VelocityPositiveMode` with `VMAX` = 10, so the chip is commanded to
  turn from the moment the bus closes. `XACTUAL` crept to 125 microsteps with
  the driver disabled, and the motor would have started the instant
  `~{TMC_DRV_EN}` went low. Bring the ramp to a real standstill — velocity mode,
  `VMAX` = 0, wait for `VACTUAL` = 0, then hold.
- **A real-unit ramp value can round to zero.** One chip unit of acceleration at
  16 MHz is 116.4 microsteps/s^2, so asking for the library's default of 10 in
  real units gave `D1` = 0. The datasheet forbids that in positioning mode and a
  TMC5130A with `D1` = 0 does not fault — it crawls at about `VSTOP`. Symptom:
  `VMAX` 5368 written, 9.5 microsteps/s observed. `conversions::nonZeroRampParameter`
  guards it, with a host test.
- **A limit switch must only block the direction that drives into it.** Checking
  both stop status bits refused the move that gets *off* the switch, in 3 ms.
  Check only the bit guarding the direction of travel, which is what the
  hardware does.
- **A diagnostic that can be truncated must say so.** The encoder round trip ran
  into the switch, lost part of a leg, and reported the resulting 0.786 ratio as
  a possible CPR error. Run it away from the switch, and report `[inconclusive]`
  when a leg does not reach its target.

Also: reading `RAMP_STAT` twice loses flags. Several bits are read-and-clear and
TMC51X0's `leftSwitchActive()` / `leftLatchActive()` / `leftStopEvent()` each
perform their own read, so read it once and decode all of it. `MotionSnapshot`
does.

## Stage 2 Design Notes

- **Raw SPI, not the TMC51X0 library.** Stage 2 asks one question: does this
  board's SPI path reach the driver. Answering it with a 40-bit datagram nobody
  has to trust beats answering it with a library that would then be a second
  unknown. Bringing TMC51X0 up on this board is its own step and starts from a
  bus already known to work. Byte order and register addresses are copied from
  `TMC51X0`'s `SpiProtocol.hpp` and `Registers.hpp` so the two cannot diverge.
- **No register writes at all.** A TMC5130A read requires a correct address over
  MOSI, a clock on SCK, framing on CS, and an answer on MISO, so reads alone
  prove the whole bus. Writes would buy nothing and add a way to leave the chip
  misconfigured.
- **The load-bearing check is `IOIN` bit 4, `DRV_ENN`.** It reads back the level
  firmware drives on `GP3` *through the bus switch*. Nothing else in the test
  proves that path. Expected `IOIN` on this board is `0x11000010` with bit 7
  (`SWCOMP_IN`, a comparator output) free.
- **Ordering is the whole point of the bus switch.** Power up: `HV_EN` high, wait
  for `+3.3V_SW`, settle, *then* close the bus. Power down: open the bus, *then*
  drop `HV_EN`. Get it backwards and MCU pins drive into an unpowered TMC5130A.
  `PowerSequence` enforces this and `connectBus()` refuses while the rail is down.
- **`PowerSequence` is the only file that raises `HV_EN`**, and it refuses to
  start unless `+3.3V_SW` is measurably dead first.
- **`vsense` is reported, never written.** Stage 3 owns setting it to 1 and
  reading it back before it lowers `~{TMC_DRV_EN}`.
- `FW_TRIG1` goes high for the duration of each register-read burst, so the
  LA5032 can trigger on ch 17 rising and capture `DIG2`-`DIG5` for the `spi`
  decoder.
- Gotcha found: `Arduino.h` defines `bitSet()` as a macro, so the helper is named
  `bitIsSet()`. A function called `bitSet` will not compile.

## Instrumentation Reach — A Standing Limitation

Worth stating once because it shaped this session. Firmware can see:

- `+3.3V_SW`, divided, on `GP26`
- `HV_SNS` and `~{HV_FAULT}`, both only while `HV_DIAG_EN` is high
- the TMC5130A's registers, once the bus switch is closed

Firmware cannot see `+36V_SW`, `+5V_SW`, `+3.3V_MCU` (it is the ADC reference),
or `+5V_MCU`. InstrumentPort-40 carries no divided motor rail to any `AIN` by
design. Any question about those rails needs a meter or a scope on the board, and
there are no test points anywhere. Budget for that rather than trying to reason
around it.

## Task: Enable The RP2040 Hardware Watchdog (opened 2026-09-08)

Paired with a **`RUN` reset switch** added to `mouse-joystick-pcb` v1.0. Both
were justified by the same incident and they cover **different** halves of it, so
neither replaces the other.

The incident: `../mouse-joystick-pcb/docs/bringup-log.md` → "`STATUS` hung the
core", 2026-09-03. The board stopped answering the link **and** stopped honouring
the 1200 baud reset touch, so it could not be reflashed. Recovery needed `SW501`
held through a power cycle with **both USB and the RJ-45 unplugged** — pulling USB
alone leaves the board alive on pyControl's 5 V through the ORing diodes.

- [ ] **Enable the RP2040 hardware watchdog.** A transient hang then self-recovers
      unattended, in the rig, with no button and no cables — the `STATUS` hang
      would have auto-recovered.
- [ ] **Decide the petting point deliberately, and write down why.** Petting from
      the main loop catches a wedged loop, which is the observed failure. Petting
      from a timer interrupt would have kept petting through that exact hang and
      bought nothing.
- [ ] **Make the reset safe by construction, and verify it on hardware.** The
      board already argues it is: `HV_EN` releases into `R1904` 10 kΩ pulling `EN`
      low at the `TPS1HTC100` so `+36V_SW` drops, and `~{TMC_DRV_EN}` pulls up to
      `+3.3V_SW` through `R1103` 10 kΩ so the driver disables. Confirm that a
      watchdog reset mid-move actually lands there rather than leaving the motor
      energised.
- [ ] **Guard against a reset loop being worse than a stop.** A watchdog reset
      does **not** enter BOOTSEL, so firmware that hangs early gives a reset loop
      that cannot be reflashed out of — the watchdog makes that case *harder* to
      escape, and the `RUN` switch plus BOOTSEL is the only exit. Consider a
      scratch-register reboot counter so repeated watchdog resets land in a safe
      halt (rail off, driver disabled, link answering) rather than looping.
- [ ] **Record the watchdog reset in `STATUS`** so a reset is visible to the host
      and to the capture instead of looking like a clean boot. `WATCHDOG_REASON`
      distinguishes it from a power-on reset.
- [ ] Note the division of labour in `.agents/AGENTS.md`: **watchdog for "it
      wedged mid-session", `RUN` + BOOTSEL for "I just flashed something
      broken".**

## Not A Priority

- OTA, filesystem, or EEPROM use. `board_build.filesystem_size` is 0.
- Anything that energises the 36 V rail, until stage 1 passes.
