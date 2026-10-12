# Board 4 regression of the HEAD firmware (`ec3b36d`) — 2026-10-05, 11:42

Board 4 (`E4654C159F171E34`, pcb v0.3, RIGHT joystick) on the LA5032 v1.0
adapter, RJ-45 unplugged, no pyboard, 36 V supply on at Peter's hand, rail
switched by firmware. Purpose: run every tool the v1.0 bring-up will use on
known-good v0.3 hardware after the 2026-10-04 v1.0-preparation refactor and
the switch of qp-arduino to its released tag `v7.3.4`, so that a different
result on v1.0 is the board. Driven non-interactively (an agent at the
keyboard): no meter readings, no hand pull, stops skipped. Console logs in
`consoles/`, captures in `captures/` (`.sr` plus `.vcd`), the QP phases in
`steps/`, `pilot/`, `uart/`, `qspy/`, `analysis/`. The electrical numbers
are transcribed in `mouse-joystick-pcb/docs/bringup-log.md` under board 4
("Regression of the HEAD firmware on board 4 — 2026-10-05") and the adapter
channels in `la5032-instrument-port-adapter/docs/bringup-log.md`.

| Step | Build | Result | Files |
|---|---|---|---|
| Stage 1 `RP2040SelfTest` | `mouse_joystick_rev1` | 24 checks, 0 failed; boot signature, heartbeat (500.25 ms half-period), `t` burst and three beacon frames on CH16/CH17 | `consoles/s1_selftest_rev1.log`, `captures/s1_selftest_boot.*` |
| Stage 2 `TMC5130Communication` | `mouse_joystick_rev1` | rail up 133 ms (staircase), peak 3.2944 V; 5 register checks, 0 failed; 16 SPI datagrams decoded from CH2–CH5 equal to the console's bytes at 1.00 MHz; `m` reports `rail: NOT UP` (tool bug, see the bring-up log) | `consoles/s2_tmc_rev1.log`, `captures/s2_tmc_spi.*` |
| Stage 3 `MotionAndHoming`, first pass | `mouse_joystick_rev1` | mis-ordered by the operator (`W` before `e`): the sweep ran 153° with the driver off, the encoder tap shows no motion; `g` from the hard stop failed as its text predicts | `consoles/s3_motion_rev1.log`, `captures/s3_motion.*` |
| Stage 3, correct order `b c e W g h H t L` | `mouse_joystick_rev1` | switch at 8319 µsteps (58.49°) from the negative stop, throw 256, active high; `g` follow 0.9398 / 0.8797, residual −77 µsteps; homed; `H` 5 of 5, spread 26 µsteps = 2.0 counts; `t` FAIL (slow approach ended without a latch); `L` 1000.1 Hz, worst loop 386 µs | `consoles/s3b_motion_rev1.log`, `captures/s3b_motion.*` |
| IRUN discrimination `b c e k k j k j k g`, then `n 17 c e j k j k g` | `mouse_joystick_rev1` | same-direction jogs 1.0000 at both currents; reversals 0.9297/0.9398 at IRUN 8, 0.9500/0.9594 at IRUN 17; taps = `X_ENC` within 2 counts on all 14 moves | `consoles/s6_irun_rev1.log`, `captures/s6_irun.*` |
| Replacement joystick unit, same series plus `h H t` | `mouse_joystick_rev1` | switch 21.65° from rest, throw 256; reversals 0.9297 ×5 at IRUN 8, 0.91–0.97 at IRUN 17, continuing 0.99–1.02; `H` 5 of 5 spread 1.0 count; `t` FAIL (switch left engaged after the fast approach) | `consoles/s7_newjoy_rev1.log`, `captures/s7_newjoy.*` |
| `AngleOutput` | `mouse_joystick_rev1` | rail 132 ms, bus, SPI1, `GAIN 0x101` written, codes commanded, step pulsed; no meter | `consoles/s5_angleoutput_rev1.log` |
| QP a | `mouse_joystick_qp_rev1_usb` | identity ok, `fw=ec3b36d`, `boot=reboot;crumb=0` | `steps/check-id.*` |
| QP b | | `hv on` acked, HOMED, `angscale`, `angout on`, TARGET at 25/50/75 % and park, `angout off`, disarm; meter prompts unanswered | `steps/b_angle.*`, `captures/b_angle.*`, `analysis/analysis_b_angle.*` |
| QP c | | 20 of 20 moves (ack→TARGET p50 434.8 ms), `cfg get`, `brake 0` → RELEASED, `watch` (no hand: first motion of the released joystick, no outcome, `TRIAL outcome=NONE` on leaving RELEASED), FETCH 500 samples `lost=0`, `stream on 50` 100 lines in 2 s, host-loss FAULT after 124.7 ms, `clrfault` | `pilot/c_pilot.*`, `captures/c_pilot.*`, `analysis/analysis_c_pilot.*` |
| QP d | | 300 of 300 moves (ack→TARGET p50 434.5 ms, max 473.9), no naks; SAMPLE_STAT and SPI below | `pilot/d_budget.*`, `captures/d_budget.*`, `analysis/analysis_d_budget.*` |
| QP e | | `hv off`, `hv=0` | `steps/e_hvoff.*`, `captures/e_hvoff.*` |

## QP trace numbers (`mouse_joystick_qp_rev1_usb`, rev 1 pad map, 1 kHz tick)

| Quantity | b: homed, holding, angle output on | c: 20 moves, trial block, mostly idle | d: 300 moves back to back |
|---|---|---|---|
| `SAMPLE_STAT` last tick cost, p50 / p99 (µs) | 158 / 172 | 46 / 526 | 158 / 429 |
| worst tick per second, p50 / max (µs) | 379 / 1346 (`hv on`) | 88 / 3292 | **1081 / 1328** (the command-handling tick, one per move) |
| seconds over the 400 µs budget | 2 of 84 | 17 of 170 | 81 of 81 |
| ticks / handled / coalesced / dropped | 83000 / 82998 / 2 / 0 | 169050 / 168785 / 265 / 0 | 80003 / 79265 / 738 / 0 |
| skipped (idle gap) / skipped (decimated) | 2 / 41498 | 251 / 7496 | 937 / 39164 |
| TICK level on CH16, p50 / p99 / max (µs) | — | 47 / 571 / 1445 | 146 / 542 / 1490 |
| back-to-back TICK levels (overruns) | — | 66 | 96 of 90035 |
| tick period on CH8, p50 / stdev (µs), missed | — | 1000.0 / 7.5, 0 | 1000.0 / 12.4, 0 |
| TMC SPI datagrams, SCK median, per tick p50 / p99 | 122423, 3.33 MHz, 2 / 10 | — (not captured) | 123948, 3.33 MHz, 2 / 10; intra-tick gap p50 5.3 µs |
| `LINK_WORK` advance / feed / output p50 (µs) | — | 298 / 321 / 203 | 254 / 332 / 195 |
| trace high-water max, buffer-full | — | 1544, 0 | 548, 0 |
| line pool / queue min free | — | 1 / 9 | 1 / 9 |
| move ack → TARGET, p50 / max (ms) | — | 434.8 / 472.4 (20) | 434.5 / 473.9 (300) |
| ping round trip over USB CDC, p50 / p99 (ms) | — | 3.1 / 5.2 | 2.9 / 4.1 |

Against 2026-10-02: the rev 0 bisect build moving measured a TICK level of
p50 444 / p99 1138 / max 2065 µs and the rev 1 build holding measured light
201 / full 387 µs. Today's rev 1 HEAD holds at **46 / 88 µs** without the angle
output and **158 / 379 µs** with it (the per-tick DAC write on SPI1), and
moves at p50 146 / p99 542 / max 1490. The handoff's open question on the
tick cost after the one-service change is answered by the 46 / 88 figure. What
exceeds the budget now is the command-handling tick of each `moveto`, about
1.0 to 1.3 ms, not the sampler. `hv on` took one tick of 1346 µs (phase b) and
`DAC_STATE` 1 then 2 followed 7.0 ms apart. The host-loss fault came 124.7 ms
after the last heartbeat with `wdt_ms=100`; `hv off` left `hv=0` and
`DAC_STATE 0`.

## Findings

1. **No regression from the 2026-10-04 refactor or the qp-arduino tag on any
   v0.3 path exercised.** Stage 1 count, stage 2 registers, homing
   repeatability and the QP phases all reproduce the September and
   2026-10-02 results.
2. **The lost motion is a direction-reversal dead band of the stepper against
   the mechanism's friction, not the TMC, not the signals, not the joystick
   unit, and not new.** Taps agree with `X_ENC` within 2 counts on every move.
   Continuing moves follow at 0.99–1.02; every reversal loses 77–90 µsteps
   (6–7 counts) at IRUN 8 and 52–64 µsteps at IRUN 17. A replacement joystick
   unit measured the same 90 µsteps at IRUN 8. On the taps a reversal's first
   count comes 52 ms after a same-direction move's; the shaft settles 25–35 ms
   before the move is declared done. Board 2 showed a −51 µstep residual in
   September. Full table in the PCB bring-up log.
3. **`t` (switch reporting delay) fails on both units**: after the fast
   approach's hard stop on the switch, the move away produced only 14 counts
   and left the switch engaged, so the slow approach never ran; the code
   ignores the away move's report, and the blocking `moveTo()` follow check
   has no grace distance or second window, unlike `TmcMotion`'s.
4. **`m` in `TMC5130Communication` fails whenever the rail was live**:
   `measureHvCurrent()` waits 50 ms after `powerDown()` and `powerUp()`
   refuses while `+3.3V_SW` still reads live (~700 ms bleed). Confirmed by
   running it from a cold rail (`x`, 4 s, `m`): zero 11.77 counts, live 16.76,
   **12.9 mA of 36 V input current** with the bus open and the driver off
   (`consoles/s2b_tmc_m_check.log`).
5. **`W` does not refuse with the driver disabled** and reports a 25 s sweep
   that moved nothing as "ran out of time".
6. **The session scripts' `DAC_STATE` count misses records emitted before the
   first dictionary** (qspy prints them as `USER+012`); phase b had two,
   phases 1 and 2 of the DAC bring-up at `hv on`, 7.0 ms apart.
7. The WATCH classifier behaved per spec without a hand: the released
   joystick drifted +1.7° (first motion at 712 ms), so neither `NOMOVE` nor
   `ABORT` (`maxdur` disabled) could fire; `TRIAL outcome=NONE` came when the
   park move left RELEASED. A bench without a hand should set `maxdur`.


## v1.0 unit A (`E4654C159F274534`), same day, `v1/`

Both v1.0 units arrived dead: a wrong green, banded part at every `C2858031`
position (`C1301` on `+3.3V_MCU`, `C1208` on TMC `5VOUT`, `C1501` on `+5V_SW`)
clamped the always-on rail. With `C1301` removed, `C1208` refitted and `C1501`
removed, unit A passed everything below. Numbers and reasoning are in
`mouse-joystick-pcb/docs/bringup-log.md` (v1.0 unit A sections) and the
Digital-50 channel results in `la5032-instrument-port-adapter/docs/bringup-log.md`.

| File(s) | What |
|---|---|
| `v1_quiet_all32.sr`, `v1_quiet2_all32.sr`, `v1b_quiet_all32.sr` | both units dead: all 32 channels low |
| `s1_selftest_rev2_boardA.log` | stage 1 on `mouse_joystick_rev2`: 20 checks, 0 failed |
| `v1a_boot_all32.sr`, `v1a_signature.sr`, `s1b_*`/`s1c_*.log` | adapter v2.0 first mating: straight cable, beacon decodes to the serial, boot signature 3 × 10 ms / 8 × 1 ms |
| `v1a_s2_tmc.sr`, `s2_tmc_rev2_boardA.log` | stage 2 before the fix: latch-off armed at ~20 ms, trip at ~37 ms |
| `v1a_s2b_tmc.sr`, `s2b_tmc_rev2_boardA.log` | stage 2 after the fix: rail 3–4 ms, bulk charged 33–34 ms, 5/0 registers, twice |
| `v1a_a3_dac.sr`, `a3_*`, `a3b_*.log` | A3: I2C DAC 0x48, GAIN read back, DEVID 0x115; monitor 0.8236 / 1.6415 V |
| `v1a_a6_motion.sr`, `a6_*`, `a6b_*.log` | A6: homing 5 of 5 at 2.0 counts, dead band 89 µsteps, live loop 1082 → 732 µs |
| `qp/` | QP phases a–e on `mouse_joystick_qp_rev2_usb` with the Digital-50 channels remapped for the trace tools |
