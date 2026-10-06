# Stages A4 And A5: Brake Sink And Torque Bridge

`examples/BrakeAndTorque`, built as `env:mouse_joystick_rev2`. pcb v1.0 only:
the brake sink (`brake_current_sink_0_49ma_dac80501`) and the torque bridge
(`torque_bridge_0_3v3_ina821`) exist from that revision. Requires 36 V, the
brake coil and the torque sensor cabled, and stages 1, 2 and A3 passed on the
unit first. The numbers below are the first unit's; the record for every unit
is `../../mouse-joystick-pcb/docs/bringup-log.md`, per serial.

## What Changes At This Stage

Two new parts on the switched domain, two new buses, and for the first time a
board that can read its own outputs back:

- The brake DAC, a second `DAC80501Z` at I2C 0x49 on the same `Wire` bus as
  the angle DAC (0x48), sets a linear current sink from `+36V_SW` through the
  B5Z brake coil: 0–48.78 mA for codes 0–65535, 0.744 µA per code. The coil
  current comes back as `BRAKE_ISENSE` (GP27, ADC1) at 20.625 V/A; the
  ceiling amplifier's `~{BRAKE_FAULT}` (GP13) is low while it limits.
- The torque ADC, an `ADS8689` on SPI1 (GP10/11/12, CONVST/CS on GP15), reads
  `AMP_VOUT`, the conditioned bridge signal before the ÷2 to the BNC: ideal
  zero 1.6667 V, 0.2715 V per N·cm nominal, 78.125 µV per code on the
  0–5.12 V range. `TQ_CAL_EN` (GP18) is meant to switch a 75 kΩ shunt across
  one bridge arm.

Both parts reset with the switched rail, so the console re-asserts what it
needs at every `b` and forgets both at `x` (`invalidate()`, which sends
nothing: clocking a dead part back-powers its rail through the input clamps).

**The motor driver is never enabled.** `~{TMC_DRV_EN}` stays high from
`SafeState` to power-down and SPI0 is never claimed.

## The Gates, And Why They Are In This Order — `b`

1. Rail up through `PowerSequence`, bus switch closed only after.
2. I2C0 claimed on GP16/GP17 at 400 kHz; both addresses probed (rig plan
   A1b); the brake DAC soft-reset, then `CONFIG`, **`GAIN 0x0101`**, `SYNC`,
   and `GAIN` read back before any data write. `DEVID` must say 0x115: a
   `DAC80501M` would reset to midscale, 24 mA into the coil at rail-up.
3. SPI1 claimed; the ADS8689's protocol registers put at their reset values,
   **`RANGE_SEL` written to 0x0B (0 to 1.25 × VREF) and read back**, then the
   device's fixed `DATA_VAL` patterns (all ones, all zeros) read through SDO
   before any conversion is believed. The reset range is ±12.288 V, which
   reads a 0–5 V signal as plausible numbers 4.8× too small.

A register read of this part takes two frames (the value comes in the frame
after the command) and every frame's CS rising edge starts a conversion, so
the transport waits t_conv after each. `src/ADS8689Registers.hpp` has the
frame format; `test/native/test_ads8689` pins it.

## Commands

Lines: a letter, an optional number, Enter (`c 32768`, `T 2000`).

| Command | What it does |
| --- | --- |
| `b` / `i` / `x` | bring-up as above / status / brake 0, bus open, `HV_EN` low |
| `c <code>`, `a <uA>`, `0`, `r` | brake DAC code or current; off; read `BRAKE_ISENSE` and the fault line |
| `s` | sweep 0, 10, …, 100 %, every point approached from 0 (the brake's torque-vs-current has two curves, one per approach direction) |
| `S [code]` | step 0 → code → 0 with `BRAKE_ISENSE` and `~{BRAKE_FAULT}` at 5 kHz for 25 ms each way: t10/t50/t90, the peak, the fault samples |
| `t`, `T [n]`, `z` | one conversion; n at 1 kHz with mean, sd, min, max; tare |
| `k` | shunt-cal step: `TQ_CAL_EN` low / high / low, 500 samples each |
| `d` | ADS8689 register dump |
| `L [s]` | live 1 kHz loop: one conversion and one `BRAKE_ISENSE` read per tick, achieved rate, worst iteration, `FW_TRIG0` high while a tick works |
| `X` | torque statistics with the brake at 0, 50 %, 100 %, 0: brake-to-torque coupling |
| `M` | micro-benchmark: µs per ADS8689 sample, per `BRAKE_ISENSE` read, per fault-pin read, per brake DAC write (what the protocol build pays per tick) |

The host harness is `tools/board_console.py`; the two bench scripts that ran
the first unit are `qp-arduino-example/evidence/bench-scripts/unitA_brake_torque.sh`
and `unitA_brake_steps.sh`.

## Measured On The First Unit, 2026-10-06 (unit A, `E4654C159F274534`)

| What | Result |
| --- | --- |
| I2C | 0x48 and 0x49 ACK; brake `GAIN` read back 0x0101, `DEVID` 0x115, `REF-ALARM` clear |
| ADS8689 | `RANGE_SEL` 0x0B read back; loopback 0xFFFF / 0x0000; the conversions shifted out before the range took effect decode in the reset range to the same voltage, confirming the bipolar coding |
| brake sweep | linear, full scale **1.0065 V = 48.80 mA** (design 1.006 V); within ±0.07 % FS above 50 %, +0.4–0.5 mA-equivalent of monitor floor below |
| brake floor | `BRAKE_ISENSE` 0.0113 V at code 0 (design ~12.5 mV): not current |
| brake step | first ms slews 8–14 mA/ms at every amplitude (coil `L di/dt` against the headroom); **above ~60 % the current overshoots — 27 % at full scale, 62 mA for ~5 ms, settled by 17 ms**; falls 48.8 → 1.4 mA in 8 ms |
| `~{BRAKE_FAULT}` | **never low**, including through the 62 mA overshoot that exceeds the 56.4 mA ceiling (board question, filed in the PCB plan) |
| torque at rest | 1.5956–1.5975 V at `AMP_VOUT`, 69–71 mV below the ideal zero (mechanism preload or front-end offset; tare covers it) |
| torque noise | sd 1.2–1.4 codes = 95–112 µV rms = 0.35–0.41 mN·cm rms at 1 kHz |
| shunt cal | **0.0 codes of step** against 0.744 V designed: the switch is inert on the as-built board (design review of 2026-10-04 predicted it) |
| `ALARM` | 0x0041 at power-up (tripped high-AVDD flag from the rail ramp), 0 on the next read |
| live loop | 1000.1 Hz, worst iteration 295 µs, 0 overruns |
| coupling | torque mean moves 1.3 codes (0.1 mV) between brake 0 and 48.8 mA: none |
| boot glitch | `~{BUS_SW_EN}` shows no edge across a warm reboot since `SafeState::driveOutput()` writes the level before the direction (2026-10-05 had a 34 µs low at every boot) |

## What It Means For The Protocol Firmware

- **`BRAKE` must ramp the setpoint**, over at least ~6 ms to full scale, or
  stage it below 60 % and then step, rather than write the final code at
  once: a step applies up to 127 % of the commanded torque for ~5 ms at the
  very moment the drive is released. The ramp fits inside the B5Z's 6 ms
  mechanical response and the `RELEASED` event follows it.
- **Verify the coil current against the setpoint from `BRAKE_ISENSE`** once
  settled (fault bit 10 `brake_current`), and do not rely on `~{BRAKE_FAULT}`
  (bit 9) to see a ceiling event until the board question is settled.
- Torque in the 1 kHz tick costs one conversion, ~22 µs at 2 MHz; sample on
  the tick and report `torque` in `STATUS`, the events, `TRIAL` and the
  capture ring; tare with `TQ zero`; `TQ cal` is accepted but is not a
  calibration on this board.

## What This Stage Cannot Tell You

- The coil current in amps: `BRAKE_ISENSE` is read against the RP2040's
  3.3 V supply, so ±1 % is the instrument. A meter in series with the coil
  is the calibration.
- The torque scale: the certificate constant (1.544 or 1.577 mV/V per
  sensor) and weights on the lever arm (rig plan C3) give N·cm; this console
  reports the nominal 1.5 mV/V.
- `VEXC` and `VMID`: a meter.
- Which of the three explanations for the silent `~{BRAKE_FAULT}` holds: a
  scope on `V_SENSE` and `LIMIT_OUT`.
