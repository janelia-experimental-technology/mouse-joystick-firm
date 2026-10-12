# Stage 3: Motion, Encoder And Homing

**Board:** `mouse-joystick-pcb` rev 0. Passed on board `E4654C159F4D1334`
2026-09-02 — the as-built unit, `R203` intact.
**Program:** `examples/MotionAndHoming`, env `mouse_joystick`
**Needs:** 36 V on `J201`, and the motor, encoder and limit switch connected.
**Scope:** everything downstream of the driver. Stages 1 and 2 are assumed, not
re-proven.

Record results in `mouse-joystick-pcb/docs/bringup-log.md` under the board's
serial, as numbers with units.

## What Changes At This Stage

Stage 2 asked one question — does SPI reach the driver — and answered it with
raw 40-bit datagrams so that no library was a second unknown. That was the right
tool for first contact and it is still what to reach for when the bus itself is
suspect.

Stage 3 needs a ramp generator, an encoder scaler and a homing sequence.
**`TMC51X0` provides all three**, so stage 3 uses it, starting from a bus that
stage 2 already proved. `src/MotionSetup.hpp` is the glue: it holds only the
facts about this board and this rig that the library cannot know.

The dependency is pinned in `platformio.ini`:

```
lib_deps_external =
    https://github.com/janelia-arduino/TMC51X0.git#4.1.0
```

and declared again in `src/MouseJoystickBoard/library.json`, because
`lib_ldf_mode = off` means PlatformIO will not discover it from an `#include`.
Swap in `symlink:///path/to/TMC51X0` to build against a local working copy.

## The Fitted Parts

Both identified 2026-09-02; datasheets in
`mouse-joystick-pcb/docs/datasheets/`.

| | Part | What matters |
|---|---|---|
| Motor | **Lin Engineering WO-208-13-01D** | NEMA 08, 1.8 deg/step, 4-lead bipolar, **0.6 A/phase**, 6.5 ohm/phase, 3 oz-in, no gearbox, rear shaft carrying the encoder |
| Encoder | **US Digital E4T-1000**, single-ended | **1000 CPR → 4000 quadrature counts/rev**, 5 V push-pull, on the motor's rear shaft |
| Limit switch | at `J1301`, debounced by `U1301` MAX6816 | normally closed, so `REFL` rests **low** and goes **high** when triggered |

At 256 microsteps per full step that is **51200 microsteps per revolution** and
**12.8 microsteps per encoder count**, exactly.

## The Gates, And Why They Are In This Order

Stage 3 is the first stage that can put current through a motor. The ordering is
the safety argument.

### 1. Rail, then bus, then driver — `b`

`PowerSequence` raises `HV_EN` and the bus switch closes only after `+3.3V_SW`
is up. Then `TMC51X0` is attached to the already-configured SPI0, told the
device model, and given `~{TMC_DRV_EN}` as its hardware enable pin — which it
drives high, disabled, the moment it is registered.

Expect `IOIN.VERSION` = `0x11`, `SD_MODE` = 0, and `GSTAT.uv_cp` = 0. A rail
that will not come up is stage 2's problem, not this one: use
`examples/TMC5130Communication` and send `p` or `l`.

**`+3.3V_SW` reads ~3.3 V for a few seconds after `HV_EN` drops.** That is
`+36V_SW`'s bulk bleeding down through the buck. `v` immediately after `x` says
"live" and it is not a fault.

### 2. Current, set and read back — `c`

**This is the gate that matters most, and it exists because of one bit.**

`TMC51X0`'s `Registers::Chopconf` has no `VSENSE` field. It is modelled on the
TMC5160, where bit 17 does not exist; on the TMC5130A that bit is `vsense`. With
this board's 0.15 ohm shunts, `vsense` = 0 drives **1.8x** the requested
current, and `IHOLD_IRUN` is **write-only**, so the current scale itself can
never be verified by reading it back.

So `MotionSetup::applyHighSensitivity()` sets bit 17 with a raw register write
and reads `CHOPCONF` back, and `enableDriver()` refuses until that read-back
says 1. CHOPCONF is the readable half of the current decision; `vsense` is the
half that carries the factor of 1.8.

The raw write goes through `registers.write()` so it lands in the library's
mirror, and every library `CHOPCONF` write is a read-modify-write of that
mirror — so the bit survives `driver.setup()` and `driver.enable()`. It does
**not** survive a device reset or `recoverFromDeviceReset()`, because raw writes
are not tracked as replayable configuration. **Re-assert and re-verify after any
power cycle of the switched domain.**

The firmware ceiling is `IRUN` 17 = 0.421 A RMS, from reading the vendor's 0.6 A
as a sine peak. Bring-up runs at `IRUN` 8 = 0.211 A RMS. See the open issue in
the PCB repository's bring-up log: **`IRUN` = 28, which this repository's
`AGENTS.md` still names, predates the motor being identified and is above its
rating on either reading.**

### 3. Witness the limit switch — `w` or `W`

Nothing arms the switch stop until its polarity has been **measured**. Arm it at
a guessed polarity and an untriggered switch reads as a permanent stop, which on
the bench presents as a dead motor.

Two ways to measure it:

- **`w`, by hand.** Watches `IOIN.REFL_STEP` for 12 s while you actuate the
  switch. Works with the driver disabled, so it is the right first look at a
  board nobody has moved yet. Also counts transitions: exactly two per press and
  release, at a 5 ms sample, means the MAX6816 is debouncing.
- **`W`, with the motor.** Sweeps toward the switch in 128-microstep steps until
  `REFL_STEP` changes, then back until it releases. Needs `e`. On this rig the
  switch sits at a limit of travel, so the motor can reach it and no hand is
  needed.

Either way the inference is the same: with the mechanism clear the switch is
untriggered, so **triggered is whatever level `REFL_STEP` is not showing**. What
needs an actual transition is the other half — that the level *can* change,
which is what separates a working `U1301` from an open joint on its output.

On board `E4654C159F4D1334`: resting 0, triggered 1, **active high**,
`pol_stop` = 0, switch throw 256 microsteps.

### 4. Enable, then move — `e`, then `j`/`k`

Every move is bounded by a target position and watched against `X_ENC`, because
the motor is installed in the rig through the gear pair and **the joystick's
travel is about 4000 microsteps, 28 deg at the motor shaft**. A ramp generator
has no idea when it has run out of mechanism. The encoder does.

`MotionSetup::moveTo()` polls every 5 ms and ends the move on any of: target
reached, the switch that guards the direction of travel, the encoder failing to
follow, a driver fault, or a timeout. `MoveReport::follow_ratio` is measured
encoder travel over commanded travel — about 1.0 when the shaft follows, near 0
against an end stop, near −1 if the encoder counts backwards.

Nothing is strained by finding an end stop. At `IRUN` 8 the motor stalls, and
the follow check ends the move within one 200-microstep window.

### 5. Home — `h`, then `H`

**Homing runs in two stages, and the second one is the datum.**

The MAX6816 debounces before the driver ever sees the closure, so the latched
closing position sits past the true switch edge by (reporting delay x approach
velocity). That error is systematic. A single fast approach is therefore
repeatable only to the extent that the delay is, and a single slow approach
spends the whole travel at creep speed.

So:

1. **Approach fast** at `kHomeApproachRevsPerSecond` (0.200 rev/s), stop armed
   through `SW_MODE.swap_lr`, hard stop, latching the **closing** edge. Bounded
   by a target position two motor revolutions away, far beyond the rig's travel,
   so a switch that never asserts ends the move.
2. **Creep off slowly** at `kHomeCreepRevsPerSecond` (0.010 rev/s), re-armed to
   latch the **releasing** edge, in 128-microstep steps until the latch fires.
   `XLATCH` is written by the chip at the edge itself, so the poll interval
   delays the reporting and not the measurement.
3. **The release position is zero.** `XACTUAL` is shifted so that edge is 0, the
   encoder is zeroed there, the stop is disarmed and the mechanism backs off
   clear.

Exactly one latch edge is enabled at a time. The TMC5130A overwrites `XLATCH` on
every enabled latch event, so a value that could have come from either edge is
not a datum.

### What the two stages actually buy, measured

**Not repeatability.** Four series of five runs on board `E4654C159F4D1334`,
2026-09-02:

| Scheme | `ENC_LATCH` spread over 5 runs |
|---|---|
| Single stage, 0.050 rev/s | 51 usteps, 4 counts |
| Two stage, first series | 25 usteps, 2 counts |
| Two stage, second series | 64 usteps, 5 counts |
| Two stage, third series (throwaway home first) | 64 usteps, 5 counts |

**2 to 5 encoder counts in both schemes, and this rig cannot tell them apart at
n = 5.** The 2-count series was the lucky one; an earlier version of this
document claimed the second stage "halved the scatter" on the strength of it,
and repeating the measurement does not support that.

The reason is worth understanding, because it is not a disappointment — it is
the physics. The reporting delay is **8.74 ms and highly repeatable**, so it
contributes a *systematic* offset, not a random one. Systematic offsets move
where zero is; they do not widen run-to-run spread. Precision and accuracy are
different quantities and the earlier claim conflated them.

So what the second stage does buy, all of it real:

- **The datum stops depending on approach velocity.** At 0.200 rev/s the
  closing edge sits 89 usteps past the true switch edge; at the creep velocity,
  4.5 usteps. Without the creep, changing the approach speed moves zero.
- **The primary approach can run four times faster** than a single-stage home
  that has to be slow *because* it is the datum. 365 ms instead of ~1600 ms for
  a full-travel approach.
- **The datum is always approached the same way** — same direction, same low
  speed — so switch hysteresis cannot enter differently between runs.

What it costs: ~2531 ms per home against ~1015 ms, most of it the creep itself
(around 680 usteps at 512 usteps/s). Raising the creep to 0.020 rev/s roughly
halves that and keeps its share of the delay under one encoder count.

**Always record the velocities with a repeatability figure.** The datum moves
with approach speed, so a number without a velocity means nothing.

### The residual 2-5 counts is not yet explained

64 usteps is five times the encoder's own 12.8 usteps quantum, so it is not
quantisation. Candidates, in the order worth testing:

- **the switch's own mechanical repeatability**, amplified to the motor shaft
  through the gear pair — the most likely, and testable by homing against a
  hand-held short at `J1301` instead of the mechanism;
- backlash or stiction between the motor shaft and the lever;
- variation in the reporting delay itself, which `t` can bound by repeating.

`H` runs one throwaway home before the counted series, so the first run does not
start from a different place than the rest. That was added on the suspicion that
the first run was the outlier; it made no difference to the spread, which is
itself evidence that the scatter is real rather than an artifact of the series.

### `t` — measure the switch's reporting delay

Two approaches at two velocities, both latching the closing edge in one position
frame. `latch(v) = edge + v * delay`, so the difference over the velocity
difference is the delay.

**Measured: 8.74 ms** on board `E4654C159F4D1334` — 85 usteps between latches
taken at 10240 and 512 usteps/s. That is debounce plus signal path, and it is
substantially shorter than the tens of milliseconds a MAX6816 might have
suggested. At the approach velocity it costs 89 usteps of overshoot; at the
creep velocity, 4.5 usteps — below the encoder's own 12.8 microstep quantum,
which is why the two-stage datum lands at the measurement floor.

This corrects a claim made earlier in this session's work: the reporting delay
does **not** appear in no register. `XLATCH` minus the settled `XACTUAL` measures
the driver's deceleration overshoot and not the debounce — but two latches at two
velocities separate the delay from the switch's mechanical position, and that
needs no instrument. Use it to size the creep bound, which has to exceed the
approach overshoot plus the switch's throw: 89 + ~630 = ~720 usteps here,
against a bound of 2560.

## Three Orientations That Are Measured, Not Assumed

All three are properties of how the rig is built, all three default to what
board `E4654C159F4D1334` measured, and all three can be flipped at the console.

They are also **exactly the three things a mirrored mechanism changes**, which is
why they live together in one `MotionConfig`. The rig has a left and a right
joystick and the left is mirrored, so on that side the motor turns the other
way, the encoder counts the other way, and the switch is at the other end of
travel in motor coordinates.

**That is handled in the harness, not here.** Decided 2026-09-02: the left
joystick's cable swaps one motor coil's leads and the encoder's A/B channels, so
both stations look identical from the connector inwards and no board carries any
left/right state. Tables and the verification procedure are in
`mouse-joystick-pcb/docs/harness-pinouts.md`.

`MotionConfig` still holds all three, because they are what the bench measures
and because a cable can be built wrong. `m`, `o` and `y` flip them for
diagnosis; the shipped values describe a correctly-built harness.

**The motor direction** -- `m` flips it, through `GCONF.shaft`. Note that
reversing the motor also reverses which end of travel the switch is on in motor
coordinates, so `m` and `y` move together.

**The encoder counts down as `XACTUAL` counts up** — `o` flips it. A mounting
sense, not a fault. `ENC_CONST` resets to **zero**, so `X_ENC` does not count at
all until firmware writes it; an encoder reading a constant 0 is the symptom of
forgetting that, not of a broken encoder. The negated constant is **−13 integer
+ 0.2 fractional**, not −12 and 0.8: `ENC_CONST` is one signed fixed-point value
and the fraction is always added. That encoding is bench-confirmed — the wrong
one gives a follow ratio of 0.875 instead of 1.0.

**The limit switch is at the positive end of travel** — `y` flips it. This is
not cosmetic. `J1301` reaches TMC pin 8, `REFL`, which stops **negative** motion.
A switch at the positive end therefore needs `SW_MODE.swap_lr` set, or the
hardware stop guards the direction that moves *away* from the switch and leaves
the one that drives into it unguarded.

## Four Defects Found By Running It

All fixed. All four were invisible to a compile and to host tests, and three of
them would have moved the motor when nothing asked it to.

1. **`RAMPMODE` = hold does not stop the motor.** `TMC51X0`'s
   `Controller::initialize()` runs during `setupSpi()` and writes its default
   `ControllerParameters`, whose ramp mode is `VelocityPositiveMode` with
   `VMAX` = 10 — so from the moment the bus closes the chip is commanded to turn
   at ~9.5 microsteps/s. Hold mode means "velocity remains unchanged", so it
   *freezes* that creep instead of ending it. Symptom: `XACTUAL` walked to 125
   microsteps with the driver still disabled. Harmless there, and uncommanded
   motion the instant `~{TMC_DRV_EN}` goes low. `configure()` and
   `enableDriver()` now both bring the ramp to a real standstill.

2. **`D1` = 0, from a real-unit value that rounded away.**
   `ControllerParameters` are handed to the library in real units and converted,
   and one chip unit of acceleration at 16 MHz is **116.4 microsteps/s^2** — so
   restating the library's default of 10 in real units produced `D1` = 0. The
   datasheet forbids that in positioning mode, twice, "even if V1=0". A TMC5130A
   with `D1` = 0 does not fault: it **crawls at about `VSTOP`**. Symptom: `VMAX`
   written as 5368 (5119 microsteps/s), 9.5 microsteps/s observed, a 1280
   microstep jog timing out 38 microsteps in. `conversions::nonZeroRampParameter`
   now guards it and a host test pins the arithmetic.

3. **The switch guard blocked the move that gets you off the switch.** It
   checked both stop status bits, so with the joystick resting on the switch
   every move was refused in 3 ms — including the one moving away. It now checks
   only the bit guarding the direction of travel, mirroring the hardware.

4. **The encoder round trip ran into the switch and blamed the encoder.** The
   outbound leg was truncated by the end of travel, and the ratio that came out
   (0.786) was reported as a possible CPR error. It now runs *away* from the
   switch, and any leg that does not reach its target makes the result
   `[inconclusive]` instead of a diagnosis.

## Findings To Take Back To `TMC51X0` 4.0.3

Provenance matters here, so each item says how it was established. Written
against tag `4.0.3`; line references are to that tag.

**Confirmed on hardware:**

- **`Registers::Chopconf` cannot express `vsense`** (bit 17). `grep -i vsense
  src/` returns nothing in the whole library. The struct follows the TMC5160
  layout, where the TMC5160 datasheet rev 1.18 lists bit 17 as "reserved, set to
  0" and bits 23:20 as `TPFD`; on the TMC5130A bit 17 is `vsense` and bits 23:20
  are `SYNC`. On a TMC5130A with low-value shunts this is a factor of 1.8 in
  motor current, and it can only be set with a raw register write.

- **`CHOPCONF`'s seeded reset default is the TMC5160's, unconditionally.**
  `Registers.cpp` `seedStoredResetValues_()` does
  `seedStoredResetValue_(ChopconfAddress, 0x10410150UL)`, and `0x10410150` is
  exactly the value the TMC5160 datasheet gives as CHOPCONF's reset default. The
  TMC5130 datasheet gives no reset default for CHOPCONF, and this board's
  TMC5130A read `0x00000000` from a cold rail during stage 2. Because every
  high-level write is a read-modify-write of the mirror, the first `CHOPCONF`
  write pushes 5160 field values onto a 5130 — `intpol` = 1, and bits 23:20 = 4,
  which on a 5130 enables chopSync at fCLK/(4*64) = 62.5 kHz. Benign here, since
  `TOFF` = 3 ends the chopper cycle before each sync pulse, but inherited rather
  than decided. **The fix pattern already exists in the same function:**
  `PWMCONF` is seeded through `pwmconfResetDefault(device_model_)` while
  `CHOPCONF` is not.

- **`Controller::initialize()` leaves a live velocity command.**
  `initialize()` calls `reinitialize()`, which calls `setup()` with the default
  `ControllerParameters` — ramp mode `VelocityPositiveMode`, `max_velocity` 10.
  So from the moment `setupSpi()` returns, the chip is commanded to turn at
  ~9.5 microsteps/s. Observed: `XACTUAL` walked to 125 microsteps with the
  driver still disabled. Harmless only because `~{DRV_EN}` was high; with the
  output stage already enabled it is uncommanded motion. Note also that
  `RAMPMODE` = `HoldMode` does not fix it, because hold means "velocity remains
  unchanged".

- **`controllerParametersRealToChip()` can convert a legal value to an illegal
  one.** One chip unit of acceleration is fCLK^2/(512*256*2^24) — 116.4
  microsteps/s^2 at 16 MHz — so a small real-unit deceleration rounds to 0.
  Passing the library's own default of 10 in real units yields `D1` = 0, which
  `Controller::writeFirstDeceleration()`'s own comment says not to do, and which
  the TMC5130A datasheet forbids in positioning mode. The chip does not fault:
  it crawls at about `VSTOP`. Observed: `VMAX` written as 5368 (5119
  microsteps/s), 9.5 microsteps/s measured, a 1280-microstep jog timing out 38
  microsteps in. **Clamping `first_deceleration` and `stop_velocity` to >= 1
  after conversion would have caught it.** Relatedly, the default
  `ControllerParameters` has `max_deceleration` = 0, and with `V1` = 0 that is
  the deceleration the ramp actually uses.

**From reading the source, not exercised on hardware:**
- **`beginHomeToSwitch()` / `homed()` report success on standstill alone.** In
  `TMC51X0.cpp`, `homed()` returns true for `HomeMode::Switch` as soon as
  `readActualVelocity() == 0`, with no check that a switch event caused the
  stop — so "stopped at the switch" and "reached the travel bound" are
  indistinguishable. The stall path already got exactly this treatment
  (`home_failed_` when no stall event is confirmed); the switch path did not.
  There is also a race: velocity is still 0 immediately after
  `beginHomeToSwitch()` writes `PositionMode`, so the first `homed()` call can
  return true before the ramp has accelerated — and the shipped example is
  `begin(); while (!homed())`. **Not exercised here**, which is why
  `MotionSetup::homeToSwitch()` implements the datasheet's procedure over the
  library's primitives instead of calling that pair.
- **`leftSwitchActive()`, `leftLatchActive()` and `leftStopEvent()` each read
  `RAMP_STAT` separately**, and several of those flags are read-and-clear — so
  calling two in a row loses whichever flag the first read consumed. Read
  `RAMP_STAT` once and decode all of it; `MotionSnapshot` does.
- **`Controller::initialize()` leaves a live velocity command.** See defect 1.
- **Ramp registers are write-only on the chip**, so "what was written" can only
  come from the library's mirror via `registers.getStored()`. The `d` command
  prints the mirror alongside the readable registers, and that pairing is what
  separates "firmware wrote the wrong number" from "the chip ignored the right
  one".
- **The register access tables are not device-model gated.**
  `Registers::initialize()` sets `writeable_[]`/`readable_[]` identically for
  both parts, including TMC5160-only registers — `GlobalScalerAddress` (0x0B),
  `ShortConfAddress`, `DrvConfAddress`, `OffsetReadAddress`. Since
  `DriverParameters` defaults `global_current_scaler` to 100, every
  `driver.setup()` writes 0x0B, which does not exist on a TMC5130A. Harmless —
  unmapped writes are ignored — but it is a wasted transaction and it puts a
  meaningless entry in the mirror. `setDeviceModel()` is already called before
  this point, so the gate is available.

## Watching It With The LA5032

`FW_TRIG1` is high for the duration of each motion command and `FW_TRIG0` pulses
on each switch transition the hand witness sees. Through the
`la5032-instrument-port-adapter`:

| Signal | `J1501` pin | LA5032 channel |
|--------|------------|----------------|
| `FW_TRIG0` | 26 | 16 |
| `FW_TRIG1` | 28 | 17 |
| `ENC_A` | 2 | 0 |
| `ENC_B` | 3 | 1 |
| `LIMIT_SW` | 19 | 11 |

Trigger on channel 17 rising and capture 0, 1 and 11 to see the quadrature and
the switch against the firmware's own view of the move. Send `i` to enable the
buffers if the port is silent — it is silent from reset by design.

## What Stage 3 Still Cannot Tell You

- **Coil current.** Every current figure here is what firmware *wrote*, scaled by
  a `vsense` bit that was read back. `IHOLD_IRUN` is write-only, so the actual
  current needs a meter — and until it is measured, whether the motor's 0.6 A
  rating is RMS or peak stays open, and with it the run-current ceiling.
- Sense-resistor and motor case temperature, `DIAG0`/`DIAG1` behaviour,
  microstep smoothness, and the step-rate ceiling at 36 V. None of it is reached
  at `IRUN` 8.
- Anything about the pyControl protocol. That is stage 4.
- The switch's mechanical position in absolute terms. The 8.74 ms reporting
  delay is now measured and the datum is the release edge, but where that edge
  sits relative to the mechanism's own hard stop is not known — that needs the
  analyser or a dial indicator.
