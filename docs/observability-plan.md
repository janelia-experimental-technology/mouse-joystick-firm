# Firmware observability development plan

Status: revised development intent, 2026-09-30. This repository owns `OBS-FW`: application
semantics, a non-QP baseline, and a possible later QP trace comparison.
The shared [architecture](../../orb-weaver-context/context/experimental-system-observability.md)
records the intent and the shared
[roadmap](../../orb-weaver-context/context/experimental-system-observability-plan.md)
owns cross-repository stages `G0`–`G5`. These are separate from the existing
board bring-up stages and protocol gates in [.agents/PLANS.md](../.agents/PLANS.md).

## Scope and authority

Keep current firmware delivery and local motion/fault protection independent of
this optional observation work. pyControl retains experiment sequencing.
An agent or host monitor can diagnose recorded behavior; it does not replace
firmware timing, limits, watchdogs, or safe-state handling. Existing watchdog
and bring-up tasks remain their own work.

QP/QV is a candidate for applications whose trace visibility justifies its cost.
There is no decision here to rewrite the running joystick firmware. Its current
protocol state machine and `MotionBackend` boundary are the baseline. Structured
application events from this implementation are valid inputs to the observation
system even if QP is never adopted.

The [QP port plan](../../qp-arduino/docs/observability-plan.md) now makes
RP2040 the primary QP `v7.3.4`/QV qualification target, using the Earle
Philhower Arduino core family already selected in this repository's
`platformio.ini`. That effort starts on a simple non-actuating fixture in
`qp-arduino`, not by replacing firmware on a working joystick. Exact fixture,
core and toolchain versions are still open. Teensy builds remain reference
regressions; its deferred hardware gates are not prerequisites. STM32G0B1CEU6
is a later second target with its own evidence requirements.

Initial RP2040 QP work keeps QP/QS and application drivers on core 0, with no
application core-1 work, QP calls or shared drivers. The imported ARMv6-M path
requires actual integration and measurement; Cortex-M0+ PRIMASK masking does
not protect against another core. Port qualification does not by itself qualify
this application's protocol, motion, power sequencing or board revision.

The [PCB interface map](../../mouse-joystick-pcb/docs/interface-map.md),
[power notes](../../mouse-joystick-pcb/docs/power.md),
[protocol specification](../../mouse-joystick-pcb/docs/uart-protocol.md), and
[bring-up log](../../mouse-joystick-pcb/docs/bringup-log.md) own board facts,
protocol decisions and measured board behavior. Select a manufactured PCB tag
and serial before deriving wiring or capabilities; current PCB development and
historical firmware examples describe different revisions. Reconcile the
selected tag's pad-to-net map with firmware before any hardware pilot. Do not
allocate extra UARTs, marker pins, SWD, or trace connections from this document.
New debug/trace access is an `OBS-IF` contract plus `OBS-HW` board decision.
The selected direction keeps passive Digital-50 and adds a separate optional
debug/trace connector using a reusable external RP2040 probe. Combined 60-pin
observation/debug wiring and an onboard second MCU are deferred. Both RP2040
and STM32G0B1CEU6 lack SWO/ITM: UART QS output must be explicitly provisioned
and qualified. These intents do not change pin assignments or imply that
current boards expose any extra UART or debug connection.

## Steps and dependencies

All steps below are open; historical bring-up passes do not complete them.

| Step | Work and deliverable | Dependencies | Exit evidence |
| --- | --- | --- | --- |
| `OBS-FW-1` | Specify one representative command-to-completion sequence, plus cancel, fault and host-loss cases, in terms of existing application behavior. Distinguish arrival, acceptance, execution, device write, measured response and completion. | Ready now; existing protocol and code. Coordinate G0 fields/fixtures with `OBS-IF` and `OBS-HOST`. | Reviewed semantic mapping to code and protocol; expected behavior cites requirements rather than only the implementation under test. |
| `OBS-FW-2` | Produce replayable good/fault/gap fixtures for the non-QP baseline, including late heartbeat ordering, duplicate command, reset, timeout and missing evidence. | `OBS-FW-1`; shared versioned evidence contract before exchange. | G2 contribution: host checks detect each intended defect and distinguish an observation gap from a DUT failure; synthetic and captured fixtures are labeled separately. |
| `OBS-FW-3` | Add the smallest bounded application-record adapter needed for the agreed pilot; preserve existing protocol behavior and portable device/application boundaries. | `OBS-FW-1`; `OBS-IF` for any changed transport/marker use. | Relevant existing native/pyControl tests and target build pass; record dictionary is decodable; overload/disconnected-reader policy and timing budgets are explicit. |
| `OBS-FW-4` | Measure the non-QP implementation with and without the selected observation profile under representative load and observer backpressure. | `OBS-FW-2`, `OBS-FW-3`, matching qualified board/cable/map (G1, `OBS-HW`), supervised bench access. | Response/loop/motion-poll timing, resource use, trace loss and local fault response measured; captures/units and board serial recorded in PCB bring-up evidence. |
| `OBS-FW-5` | Evaluate a QP/QV version of the same slice only if the baseline leaves a useful question or demonstrates a reason to compare. Preserve driver interfaces and behavior. | Design can proceed now with `OBS-QP`; execution waits for its active RP2040 port/core/BSP/target gates and this application's selected-board checks. Deferred Teensy hardware work is not a dependency. | G3 contribution: comparable fixtures and measured overhead/diagnostic value; record an adoption/defer/reject decision. Continuing with non-QP is a valid outcome. |
| `OBS-FW-6` | Supply matched command, event and physical-response evidence for correlation and the supervised experiment pilot. | Baseline `OBS-FW-4` or qualified QP comparison; `OBS-HOST` correlation, `OBS-IF` timing contract and `OBS-HW` evidence. | G4/G5 contribution: a replayable operation with timing uncertainty and coverage limits, followed by an observation-only run reviewed with the lab (`OBS-RUN`). |

`OBS-FW-1` and fixture design can run alongside QP port validation, host replay
implementation and interface design. New electrical access is hardware-gated;
it is not a prerequisite to documenting semantics or exercising host fixtures.
The overall QP evaluation at G3 does not force this RP2040 application to adopt
QP before G4 correlation can use its non-QP baseline. Standalone RP2040 port
qualification, Digital-50 board development and debug/probe qualification are
separate paths; none should force a rewrite of the working joystick delivery.

## 2026-10-01: the QP slice exists

`OBS-FW-5`'s design question was answered in two steps the same day. First the
protocol state machine and simulated backend ran as a QP active object in
`qp-arduino-example/rp2040/joystick_link_bsp-rp2040` on the RP2040 fixture,
traced over a PIO UART on `FW_TRIG1` (the only MCU-driven net besides the
pyControl pair that reaches the v1.0 Digital-50 port), with budgets declared
before the run and met; two defects in the RP2040 BSPs were found there and
fixed. Then the same structure was brought here as `examples/PyControlLinkQp`
with the real `TmcMotion` behind the AO and UART0 as the pyControl port, plus
a simulated-motion USB bench build that ran the pilot sequence on the fixture.
The non-QP `PyControlLink` is untouched. What remains is listed in the
example's README and in `.agents/PLANS.md`; the adopt/defer decision is still
open and belongs there.

## Evidence semantics

- Associate records with firmware build, board serial/revision, session/boot,
  command sequence and trial identity where available. Define how reused
  protocol sequence numbers, retries and resets are distinguished. Do not add
  required fields to the live researcher protocol without a protocol decision.
- Keep record semantics independent of transport and kernel. `ProtocolBoard`
  and device logic should not depend on Arduino trace transport, marker GPIOs,
  BSP or QP tick configuration. Preserve the nonblocking `MotionBackend` seam;
  bounded bus operations and waiting for physical completion are separate acts.
- A register write is evidence of a request reaching a chip, not proof of motion.
  A firmware-reported position and a record copied from that same variable are
  not independent measurements. Map actual electrical witnesses through the
  selected board overlay; use independent physical measurement when the claim
  requires it. Do not assume an encoder signal reaches the MCU.
- Declare local clock units, rollover handling, reset behavior and alignment
  uncertainty. Existing extended application time and QS timestamps need an
  explicit mapping. The identity beacon identifies a connection; it is not a
  new in-session synchronization protocol. Preserve proof-of-life and existing
  marker use until `OBS-IF` agrees any change.
- Bound output and mark loss. Compare stalled/absent observer, high event rate,
  reset and incomplete capture against the same local control requirements.
  Never wait for an agent, trace consumer or USB reader to keep motion safe.
  Buffered analyzer windows must not be reported as continuous observation.
- Keep optional debug command input separate from passive observation. A future
  second command path needs explicit authority/arbitration rules with the
  researcher connection. SWD halts and QS RX commands are bench operations, not
  part of the observation-only experiment profile.

## Parallel agent ownership

An agent first claims a step and exact files in the local plan. These are
proposed implementation boundaries; no new runtime files are created here.
One integrator owns `README.md`, `.agents/PLANS.md`, this plan,
`platformio.ini`, and changes that join the seams below.

| Agent role | Exclusive path scope | Handoff |
| --- | --- | --- |
| Semantics/fixtures | New `docs/observability-semantics.md` and `test/observability/fixtures/` | Versioned good/fault/gap fixtures and expected outcomes to `OBS-HOST`; agree wire-semantic changes with the PCB protocol owner. |
| Firmware observation adapter | New `src/Observation.hpp`, `examples/ObservabilityPilot/`, `test/native/test_observation/` | Bounded adapter, target profile and records; changes to `src/ProtocolBoard.hpp`, `src/TmcMotion.hpp`, `src/PycUart.hpp` or `examples/PyControlLink/` require a coordinated claim because delivery/watchdog work may touch them. |
| Host/application integration | Existing `pycontrol/` paths only after claiming exact files; preserve its separate environment | Implementation/fixture conformance to integrator; shared capture/monitor runtime belongs in `instrument-control`. |
| Bench operator | One explicitly assigned board/rig/analyzer and evidence bundle | Measured results to the board's bring-up log; no concurrent uploads, motion commands or captures on that setup. |

Work on isolated files can proceed in parallel. Integrate a versioned seam
before a downstream agent relies on it; a draft fixture may support preparation
but cannot establish conformance. `kicad-libs` is excluded from the current
documentation task, and the collaborator's `joystick-rig` remains a delivery
target rather than a development checkout.

## Handoff and validation

Each handoff records step/owner/commit, fixture and contract versions, selected
PCB tag and serial, firmware/build environment, dictionary/filter configuration,
clock mapping and uncertainty, artifact locations/hashes, expected versus actual
results, gaps/unrun checks, and the next consumer. Link measured results in the
PCB bring-up log rather than duplicating electrical conclusions here.

For this documentation change, check links and `git diff --check`; no test,
build, upload or new hardware result is implied. Later firmware changes use
relevant native tests and target builds; pyControl changes use its tests too.
State which checks ran. A passing host fixture is not proof of hardware timing,
power sequencing, physical response or experiment correctness.
