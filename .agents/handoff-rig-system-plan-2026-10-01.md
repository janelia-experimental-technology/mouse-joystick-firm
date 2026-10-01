# Handoff — rig system plan → firmware work (2026-10-01)

**From:** Peter, after an end-to-end review of the rig (pyControl task, pyControl
device class, this firmware, the v1.0 board, the OneBox record).
**Authority:** this file is a work brief. It does not override
`.agents/AGENTS.md`, the board repository's electrical facts, or the protocol
specification. Where it proposes protocol changes, the change is made in
`../mouse-joystick-pcb/docs/uart-protocol.md` first and implemented here second.
**Plan of record:** `../orb-weaver-context/context/mouse-joystick-rig-system-plan.md`
(sections 3, 4, 7, 9). Read sections 3 and 4 before starting; they are short.

## 0. What the firmware is for, in one paragraph

Two of these boards sit under a pyControl task that owns the experiment. pyControl
sends **parameters and bounded primitives down**; the board sends **events and a
per-trial summary up**. The board owns everything that must be ordered or timed
better than a few milliseconds: homing, moves, the brake-then-release sequence,
classifying what the animal did, and sampling angle and torque at 1 kHz. The
OneBox records the analog copies at 30 kHz; the UART is *not* a sample path.
One trial, as the task will drive it:

```
MOVETO present → ev;TARGET → (pyControl cues) → BRAKE torque → ev;RELEASED
→ WATCH pull;push;timeout → exactly one of ev;PULL | ev;PUSH | ev;NOMOVE | ev;ABORT
→ MOVETO park → ev;TRIAL (summary) → ev;TARGET
```

Heartbeat is `STATUS` (not `PING`) at 10–25 Hz from the device class; any
well-formed frame resets the host-loss watchdog (already implemented).

## 1. Where things stand (verified in the repos on 2026-10-01)

- `examples/PyControlLink` (non-QP) runs the protocol on hardware: `ID PING
  STATUS LIMITS FAULTS CLRFAULT HV INSTR HOME ARM DISARM MOVETO MOVE STOP
  FREEWHEEL ARMPULL CAL`. Measured on boards 2/3/4 per
  `../mouse-joystick-pcb/docs/bringup-log.md`.
- `examples/PyControlLinkQp` exists: one `LinkAo`, `BSP::nowUs64()`, QS on a
  PIO UART at FW_TRIG1, 100 Hz tick, `mouse_joystick_qp*` envs, sim build over
  USB. Untested with the rail up as of 2026-10-01.
- `pycontrol/devices/mouse_joystick.py` + `mouse_joystick_protocol.py` +
  `sim/board_sim.py` + `tests/fixtures/protocol_cases.txt` (23 cases) are the
  other half of the contract. **Both halves move together.**
- The 1 kHz encoder→DAC loop exists only in `examples/MotionAndHoming` (`L`).
  `ANGOUT`/`ANGSCALE` answer `badverb` in the protocol builds.
- Not built anywhere yet: `BRAKE`, torque sampling, `WATCH` as a classifier,
  per-trial summary, capture/fetch, configurable watchdog, 64-bit event `t`.
- v1.0 boards (brake current sink, torque bridge + ADS8689, I2C DACs, ÷2 output
  dividers) are ordered and expected within days. v0.3 board 4
  (`E4654C159F171E34`, RIGHT) and v0.2 boards are on the bench now.

## 2. Bugs found in the review — fix regardless of anything else

| # | Where | Problem | Fix |
|---|---|---|---|
| 1 | `src/ProtocolBoard.hpp` `beginEvent(name, static_cast<uint32_t>(now_us_))` | Event `t` wraps at 71.6 min. The QP BSP already supplies 64-bit µs; the truncation is inside ProtocolBoard/Protocol. | Carry 64-bit µs end to end; format as decimal. Update corpus EXPECT lines accordingly. |
| 2 | `ProtocolBoard` vs `TmcMotion::expectedHomeDurationUs()` | `home;<s>;<timeout_ms>` faults immediately for any timeout under 8000 ms. | Compare against the requested timeout, bounded by the firmware maximum. |
| 3 | `PyControlLink` setup | `instr on/off` only sets a flag; the buffer is enabled unconditionally. | Make the verb drive the enable, or drop the verb. |
| 4 | `pycontrol/README.md` | Says the retransmit cache holds 16 entries; code has `kRetransmitCacheSize = 2`. | Fix the doc (2 is the designed value). |
| 5 | `ID` | Reports hard-coded `fw=0.0.0`, `hw=rev0`. | Real build version and board revision from the build flags. |
| 6 | `src/TmcMotion.hpp` header | Says "NOT YET RUN ON HARDWARE"; it has run (board 3, 2026-09-03). | Update the comment. |
| 7 | `pycontrol/devices/mouse_joystick.py` | `UART(..., timeout=100)` blocks the pyControl main loop when the buffer is empty (measured: a 20 ms poll became 72 ms). | `timeout=0`, read only `uart.any()` bytes, accumulate partial lines. |

## 3. Protocol v1 — the proposal to take to `uart-protocol.md`

Transcribe this into the spec (as a dated "v1 proposal" section) and get Peter's
sign-off before changing the corpus. Framing, `seq`, `ack`/`nak`, the watchdog
rule and the retransmission rule are unchanged.

### 3.1 New and changed verbs

| Verb | Args | Semantics |
|---|---|---|
| `BRAKE` | `torque_Ncm` | **Atomic** in firmware: (1) set brake DAC to the calibrated code, (2) wait for coil current to reach setpoint or a fixed settle time (B5Z spec ~6 ms; confirm from ISENSE at bring-up), (3) release the driver, (4) `ev;RELEASED;t;angle=…;torque=…;brake=…`. State → `RELEASED`. Refused unless `ARMED`. **On boards without a brake** (`caps` lacks `brake`): accept, ignore `torque`, do (3)–(4) only. This is today's `FREEWHEEL` and **replaces it**; `FREEWHEEL` answers `badverb` after the change. |
| `WATCH` | `pull=deg;push=deg;timeout=ms[;hold=ms][;touch=Ncm][;maxdur=ms]` | One-shot classifier, valid only in `RELEASED`; `nak;notreleased` otherwise. Thresholds are relative to the angle at `RELEASED`. Sign convention: `pull` is positive displacement in the pull direction, `push` is positive in the push direction (document the mechanism's sign in the spec once; the device class hides it). Emits **exactly one** outcome event: `PULL` once angle ≥ pull continuously for `hold` ms (default 0); `PUSH` once angle ≥ push; `NOMOVE` if no first motion (angle change > a fixed deadband) before `timeout`; `ABORT` if first motion occurred but no outcome within `maxdur` (default: disabled). `TOUCH` (torque ≥ `touch`, default disabled) is informational, at most once, may precede the outcome. After the outcome the classifier is disarmed; a new `WATCH` re-arms. **Replaces `ARMPULL`.** |
| `MOVETO` | `deg[;profile]` | Unchanged semantics plus named profiles (`present`, `retract`, `slow`; constants in `AppConfig.hpp` first). Leaving `RELEASED`: re-engage driver, resync `XACTUAL` from `X_ENC` (exit-only, finding 5), emit `ev;TRIAL`, **then** release the brake. |
| `MOVE` | — | Candidate for deletion with the retransmit cache if the trial uses only absolute positions (finding 1). Decision: Peter. |
| `CFG` | `key=value[;…]` / `get` | Bounded runtime configuration. Keys for v1: `wdt_ms` (host-loss timeout, 50–500, default 100), `deadband_deg` (first-motion threshold for `NOMOVE`), `stream_hz`. Values are checked against firmware bounds; out of range → `nak;outofrange`. |
| `FETCH` | — | Returns the last `WATCH` capture: `ack;s;n=N;t0=…;dt_us=1000;pre=P` followed by N lines `cap;k;angle;torque` and `cap;end`. ASCII keeps the protocol line-oriented; 150 samples ≈ 3 kB ≈ 260 ms at 115200, acceptable in an ITI. Capture is a 1 kHz ring of angle+torque started at `WATCH` with `pre` ms of pre-trigger retained from the continuous sampler. |
| `STREAM` | `on;rate_hz` / `off` | Optional decimated `s;t;angle;torque` at ≤100 Hz. Off by default; display only. |
| `TQ` | `zero` / `cal;on|off` / `get` | Torque tare (store offset), shunt-cal switch (`TQ_CAL_EN`), raw read. v1.0 only; `badverb` when `caps` lacks `torque`. |
| `ANGOUT`, `ANGSCALE` | as drafted | Must work in the protocol build. `GAIN=0x0101` written before the first data write; 0 V (or the agreed fault code) while unhomed/faulted/disabled. |

### 3.2 Replies and events

- `STATUS` → `ack;s;state=…;angle=…;xactual=…;torque=…;brake=…;homed=…;limit=…;faults=…;hv=…;t=…`.
  `t` is board µs (64-bit decimal). The device class uses `(t, pyControl_ms)`
  pairs to map clocks; nothing else is needed for alignment.
- New events: `RELEASED` (above), `TOUCH;t;angle;torque`, `ABORT;t;angle;torque`,
  and the per-trial summary:
  ```
  ev;TRIAL;t;outcome=PULL;t_rel=…;t_touch=…;t_move=…;t_out=…;a0=…;a_peak=…;a_end=…;tq_peak=…;tq_out=…;dur_thr=…
  ```
  `t_*` board µs (`t_touch`/`t_move` empty if they did not occur); `a0` angle at
  release; `a_peak`, `tq_peak` extrema in the window (signed, pull-positive);
  `a_end`, `tq_out` at the outcome; `dur_thr` ms spent past the pull threshold.
  Emitted once, when leaving `RELEASED` (or immediately after `NOMOVE`/`ABORT`
  if the host does not move the joystick within 1 s — decide and document).
- `ID` `caps` gains `+brake+torque+watch+fetch+angout` as implemented, so the
  device class gates on capabilities, never on `fw`.
- **Line length:** a full `STATUS` reply is ~125 bytes. Raise `MAX_LINE` and
  both `rxbuf` values (board and MicroPython) together; the device class has
  `rxbuf=130` today. Propose 200.

### 3.3 State machine

```
BOOT → IDLE ─HOME→ HOMED ─ARM→ ARMED ─MOVETO→ ACTIVE → ARMED
                                 └─BRAKE→ RELEASED ─(WATCH outcome; stays RELEASED)
                                              └─MOVETO/STOP/DISARM→ re-engage → ARMED/HOMED
any fault → FAULTED ─CLRFAULT→ IDLE (homed cleared)
```
Rename the state string `FREEWHEEL` → `RELEASED` (`kFreewheel` in
`ProtocolBoard.hpp`; `FW_TRIG1` logic in `PyControlLink` compares that string).

### 3.4 Fault bits (bits 9+ are reserved for exactly this)

| Bit | Name | Cause |
|---|---|---|
| 9 | `brake_fault` | `~{BRAKE_FAULT}` asserted |
| 10 | `brake_current` | `BRAKE_ISENSE` outside tolerance of setpoint for > N ms |
| 11 | `torque_adc` | ADS8689 not responding / conversion timeout |
| 12 | `torque_range` | bridge saturated or excitation out of range |
| 13 | `not_following` | `XACTUAL` vs `X_ENC` divergence **while driving only** (finding 5) |

### 3.5 Safe state on host loss / fault (decision pending with the researcher)

Today: abort motion, `HV` off, motor coasts. With a brake fitted, propose:
brake to a configurable holding level (`CFG hold_Ncm`), driver off, `FAULTED`
latched. Implement behind a config so either behaviour is selectable; the
default stays today's until the researcher answers.

## 4. QP structure — constraints the new behaviour must respect

- **Run-to-completion budget.** The QV kernel is cooperative; every event
  handler runs to completion. The 1 kHz sampler (TMC `X_ENC` snapshot + ADS8689
  read + DAC write) must fit comfortably inside 1 ms *including* a protocol
  line's dispatch. Measured non-QP worst loop was 527 µs with USB servicing;
  budget ≤ 400 µs for the sampler event and measure with the `LINK_WORK`
  record. If it does not fit, move the DAC write to the next tick rather than
  slowing the sampler.
- **Tick.** `BSP::TICKS_PER_SEC` is 100. The sampler needs a 1 kHz source:
  either raise the tick to 1000 and derive the 100 Hz housekeeping from it, or
  add a hardware-alarm ISR that posts `SAMPLE_SIG` with the µs timestamp. Keep
  the protocol clock advance on the same timebase as `t`.
- **One AO or two.** Keep `LinkAo` for framing/dispatch/watchdog. Put the
  sampler, `WATCH` classifier, `BRAKE` sequence and capture ring in a second
  AO (`MotionAo`) only if the queue arithmetic stays bounded (declare pool and
  queue sizes as `PyControlLinkQp.ino` does); otherwise keep one AO and
  separate the concerns by class. The classifier must see every sample in
  order; do not put a queue between sampler and classifier.
- **QS records to add** (observation set, bounded): `OP_BRAKE` (phase 1/2/3 with
  ISENSE), `OP_WATCH` (armed/outcome/thresholds), `SAMPLE_STAT` (per second:
  samples, max cost, misses), `WDT_RESET` (per frame), `FAULT_SET`. The LA5032
  on the Digital-50 port sees the pyControl UART on DIG2/DIG3, FW_TRIG0/1 on
  DIG0/DIG1, TMC SPI and the encoder lines, so every claim below is checkable
  by correlating the QS stream with the capture.
- **No safety on the trace or on USB.** As today: nothing on QS or CDC can
  command the board; the watchdog and safe state never wait for a reader.
- **`SafeState` is the only writer of `HV_EN`; `BoardPins.hpp` is the only
  place a GPIO appears.** New pins for brake/torque come from
  `../mouse-joystick-pcb/docs/interface-map.md` at the manufactured tag, not
  from prose.

## 5. What can be done now vs what waits for v1.0

| Now, on v0.3 board 4 / v0.2 boards | Needs v1.0 |
|---|---|
| Bugs 1–7 | `BRAKE` phases 1–2 (coil current), bits 9–10 |
| `RELEASED` rename; `BRAKE` as release-only; `WATCH` classifier on angle only; `TRIAL` with torque fields empty | Torque sampling, `TQ`, bits 11–12, torque in `STATUS`/`TRIAL`/capture |
| `STATUS` `t` and new fields; 64-bit `t`; `CFG`; `MAX_LINE`/`rxbuf` | I2C DAC transport on the real board (`MJ_DAC_I2C=1` is untestable on v0.3) |
| `ANGOUT`/`ANGSCALE` + 1 kHz DAC loop in the protocol build (board 4 has the SPI DAC) | ÷2 output levels (board-side fact; firmware unchanged) |
| `FETCH` with angle only; `STREAM` | — |
| QS records, sampler timing measurement, hardware watchdog | — |
| Device class: non-blocking reads, named events, heartbeat timer, `AnalogChannel`s, `TRIAL` → `log()`/`print()`, 3.0 names with 2.1.1 shim | — |

Do the "now" column first; it is most of the work and all of the risk.

## 6. Test contract — how to change the protocol without forking it

1. Edit `../mouse-joystick-pcb/docs/uart-protocol.md` (v1 proposal section).
2. Add cases to `pycontrol/tests/fixtures/protocol_cases.txt` for every new
   verb, every `nak` reason, the `RELEASED`→outcome sequences (use `ADVANCE`
   and a simulated angle input; extend the corpus grammar if the simulator
   needs an `ANGLE <deg>` / `TORQUE <Ncm>` stimulus line, and document it in
   the header), and the 64-bit `t` formatting.
3. Make `pycontrol/sim/board_sim.py` pass them (`make pycontrol-test`).
4. Make the C++ half pass them (`pio test -e native`).
5. Update `pycontrol/devices/mouse_joystick.py` and its tests in the same
   change set; the device class refuses unknown `proto` and gates on `caps`.
6. Only then flash and measure. Record numbers, with units and board serial,
   in `../mouse-joystick-pcb/docs/bringup-log.md`.

A change that only one side is checked against silently forks the contract
(the corpus header says the same).

## 7. Evidence Peter needs from this work (feeds the researcher conversation)

- Command round-trip and event latency: UART tap on the LA vs the board's QS
  `LINK_TX` time vs pyControl's event time (device class prints both).
- Sampler cost per event and misses per second at 1 kHz, with and without
  `STREAM` at 100 Hz.
- `BRAKE` ordering on the LA: ISENSE rising → settle → driver enable falling →
  encoder motionless → `RELEASED` on the UART.
- 60-minute `STATUS` heartbeat soak at 25 Hz with no `host_timeout`.
- Fault drills: cable pull mid-`RELEASED`, host stall > `wdt_ms`, `HV` loss.
- Pin-6 current draw of the MCU domain (board fact → bring-up log).

## 8. Decisions reserved for Peter (ask, do not assume)

- Delete `MOVE` and the retransmit cache, or keep them.
- `TRIAL` field list and names (above is the proposal).
- `FETCH` as ASCII lines (proposed) vs a binary block.
- Fault level on `ANGLE_OUT`: code 0 vs code 497 (25 mV), and `GAIN` 0x0101 vs
  0x0000 (open since 2026-09-10).
- Whether `WATCH` outcomes beyond the five above are needed before the
  researcher answers the trial-vocabulary question.
- Default safe state on host loss once the brake exists (section 3.5).

## 9. Do not

- Add any verb that sets raw current, raw PWM, step counts, or unbounded
  velocity. Named profiles only.
- Stream samples by default, or make any safety behaviour depend on the host,
  the trace link, or USB.
- Change the protocol in code before the spec and the corpus.
- Develop on a branch of `joystick-rig`. Deliverables go there only as tested
  files from `pycontrol/`.
- Restate electrical facts here; cite `../mouse-joystick-pcb/docs/*`.
