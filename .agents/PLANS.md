# Plans

## Current Goal

Stage 4: the pyControl UART protocol. Stages 1 to 3 have passed on hardware.

## Status

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
- [x] Compile clean on `mouse_joystick`, `mouse_joystick_fastflash`,
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
- [ ] Stage 4: pyControl UART protocol
  - [x] **Host half written and green, 2026-09-02.** 115 tests.
        `pycontrol/devices/mouse_joystick_protocol.py` (codec, runs on the board
        and under pytest), `pycontrol/sim/board_sim.py` (simulator with an
        injected clock), `pycontrol/devices/mouse_joystick.py` (device class,
        asserts on `port.UART` and on duplicate `event_name`),
        `pycontrol/tests/fixtures/protocol_cases.txt` (19-case shared corpus),
        plus a bring-up hardware definition and task
  - [ ] **Protocol framer/parser in `src/`, and a host-native test that
        reproduces `protocol_cases.txt`.** Until this exists Gate C is NOT
        passed: a simulator agreeing with a corpus only it has been checked
        against says nothing about the RP2040. The corpus is line-oriented so
        the C++ side is `getline` plus a prefix compare, no JSON library
  - [ ] Resolve the two protocol findings below before freezing the draft
  - [ ] Gate E on the Breakout 1.2, then deliver to `joystick-rig`

## Protocol Findings, 2026-09-02

Raised by implementing `../mouse-joystick-pcb/docs/uart-protocol.md` (Draft
v0.1). Full list in `pycontrol/README.md`; these two are defects rather than
gaps and belong to the PCB repo's draft, not to this one.

1. **The retransmission rule breaks the heartbeat.** "A repeated `seq` with
   identical content is re-acknowledged without re-executing" assumes
   re-execution is the harm. For `PING` the side effect *is* the command — it
   only resets the host-loss watchdog. Served from cache, the safety timeout
   expires while the host believes it is being answered. A host that increments
   `seq` never hits it, so it would survive review and fire after a 65536 wrap.
   Likely fix: exempt `PING` from the cache.
2. **The 100 ms host timeout is shorter than a long move.** At 180 deg/s an 18
   degree move consumes the whole timeout, and travel is 40 degrees wide. The
   host must ping *during* motion. Possibly fine, but it should be written down
   rather than discovered. Confirm the real velocity and the real timeout
   together.

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
  `boot2_generic_03h_4`. `mouse_joystick_fastflash` is untried on hardware.
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

- **The LA5032 channel mapping.** Documented but not bench-verified. The boot
  signature is a known pattern on known pins, so it doubles as the confirmation
  once the adapter is connected. Nothing in stage 1 needed it, because USB came
  up on the first try.
- `env:mouse_joystick_fastflash` on hardware.
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

## Not A Priority

- OTA, filesystem, or EEPROM use. `board_build.filesystem_size` is 0.
- Anything that energises the 36 V rail, until stage 1 passes.
