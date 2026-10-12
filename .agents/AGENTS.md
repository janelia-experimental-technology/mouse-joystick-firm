# Mouse Joystick Firmware Agent Guidance

This file is the canonical project guidance for agent sessions in this
repository.

## Session Start

Read this file first.

If `AGENT_MEMORY_REPO` is set and points at a directory, read
`$AGENT_MEMORY_REPO/AGENTS.md` before planning or editing.

If sibling checkouts exist, read:

- `../mouse-joystick-pcb/.agents/AGENTS.md` — the board this firmware runs on.
  **It owns every electrical fact.** Do not restate one here; cite it.
- `../mouse-joystick-pcb/docs/interface-map.md` — the RP2040 GPIO allocation.
- `../mouse-joystick-pcb/docs/power.md` — the two power domains and the
  switched-rail rules.
- `../mouse-joystick-pcb/docs/uart-protocol.md` — the pyControl protocol this
  firmware will eventually speak.
- `../mouse-joystick-pcb/docs/bringup-log.md` — where real measurements go.
- `~/Repositories/arduino/TMC51X0` — the driver library this firmware will use,
  and the source of this repository's Pixi/PlatformIO layout.
- `../instrument-control` — host-side logic-analyzer control. It owns the
  `sigrok-cli` environment, the LA5032 quirks, and the identity-beacon decoder;
  `pixi run identify-analyzers` runs its tool under this repo's `pycontrol`
  environment. The beacon's **frame format** is owned by
  `../board-interfaces/docs/instrument-port-identity-beacon.md`. This repository
  implements the board half (`SelfTest::emitIdentityBeacon()`) and must not
  redefine the format.

If `.agents/handoff.local.md` exists, read it before planning or editing. Treat
it as local, current-session context rather than permanent project policy.

If the task is multi-step, risky, or likely to span more than one session, read
`.agents/PLANS.md` and keep it updated as work progresses.

## Repository Purpose

Firmware for `mouse-joystick-pcb`: an RP2040 driving one TMC5130A, exposed to a
pyControl host over one UART. Two of these boards serve the rig's two joysticks.

This repository owns firmware only. It owns no schematic, no netlist, and no
electrical decision.

**Firmware means both ends of the UART protocol.** `src/` and `examples/` are the
RP2040 side. `pycontrol/` is the MicroPython device class, hardware definition
and task that run on the pyControl Breakout 1.2. They are one contract — sequence
numbers, `ack`/`nak`, a nine-bit fault mask — and they version together, which is
why they are not in separate repositories. The protocol is *specified* in
`../mouse-joystick-pcb/docs/uart-protocol.md`; this repository implements it and
does not redefine it.

The rig integration repository is `~/Repositories/github/joystick-rig`, which is a
collaborator's. It is a delivery target, not a development target: deliver a
device class that has passed Gate C, do not develop one on a branch there.

## Bring-Up Is Staged, And The Stages Are Load-Bearing

The board has two power domains and a bus switch between the MCU and everything
else. The stages follow that hardware boundary, not convenience:

1. **RP2040 alone.** 36 V absent, `HV_EN` low, GPIO bus isolated. Proves clocks,
   flash, RAM, USB, ADC, pads, and the board's pull resistors.
   `examples/RP2040SelfTest`.
2. **TMC5130A communication.** 36 V applied, `HV_EN` raised, bus switch enabled,
   driver still disabled. Proves SPI and register reads.
   `examples/TMC5130Communication`. Raw SPI datagrams, no driver library, on
   purpose — first contact with a bus should not also be first contact with a
   library.
3. **Motion.** Current limits set and read back *before* the driver is enabled,
   then motion, the encoder, and homing against the limit switch.
   `examples/MotionAndHoming`, over `TMC51X0`, which is the right layer once the
   bus is known good. Procedure and findings in `docs/motion-bringup.md`.
4. **pyControl UART protocol.** Not written yet.

**Do not let a later stage's code leak into an earlier stage's example.** Stage 1
deliberately has no way to raise `HV_EN` or enable the bus switch. That is not an
oversight to be helpfully fixed.

## Firmware Rules

- **One board, one axis, one UART.** No axis argument, no axis suffix. If you
  find yourself adding one, you are working on the wrong board — the twin-axis
  design is `../mouse-joysticks-pcb`.
- **`src/BoardPins.hpp` is the only place a GPIO number appears.** No literal pin
  numbers in examples or in driver setup.
- **Derive board facts from the `.kicad_pcb` pad-to-net map, at the tag that was
  manufactured, not from prose docs and not from HEAD.** The pin map came from
  footprint `U501` at PCB tag `v0.1`. Prose goes stale — `docs/power.md` still
  claims `HV_SNS` and `~{HV_FAULT}` sit behind the bus switch when the netlist
  says they bypass it. KiCad 10 writes nets as `(net "name")` with no net number,
  so patterns matching a number find nothing.
- **Reference designators are not stable across PCB revisions.** Automatic
  annotation orders by sheet position. `R1901`/`R1902` mean 100 ohm pyControl
  series resistors at v0.1 and 1 kohm `HV_EN`/`HV_DIAG_EN` series resistors at
  HEAD. Quote designators only with the revision attached, and prefer value plus
  net, which are stable.
- **Nothing but `SafeState` touches `HV_EN`.** Grep for it before adding a
  caller.
- **`HV_DIAG_EN` must never idle high.** Held high it leaks ~240 uA into
  `+36V_SW` through the switch's open-load detection, the rail creeps past the
  buck's minimum input in about three seconds, and the switched domain hiccups
  instead of staying off. Raise it only while reading `HV_SNS` or `~{HV_FAULT}`,
  and lower it after.
- **The GPIO bus switch is isolated at reset and firmware must keep it that
  way** until the switched rails are up. R601 7.87 k is the only isolation
  mechanism — all four `SN74CB3Q3245` packages run from the always-on rail, so
  `Ioff` never engages.
- **`vsense = 1` with the 0.15 ohm shunts.** The shunt value and the `vsense`
  bit are one decision. `vsense` resets to 0, which with 0.15 ohm installed
  drives roughly 1.8x the intended current. Firmware must set and read back
  `vsense = 1` before it lowers `~{TMC_DRV_EN}`. Never trade it away.

  `TMC51X0`'s `Registers::Chopconf` **cannot express this bit** — it follows the
  TMC5160 layout, where bit 17 does not exist. Set it with a raw
  `registers.write()` so it lands in the library's mirror, then read `CHOPCONF`
  back. `MotionSetup::applyHighSensitivity()` does; `enableDriver()` refuses
  until the read-back says 1. `IHOLD_IRUN` is write-only, so `CHOPCONF` is the
  only readable half of the current decision.

- **The run current is `IRUN <= 17` (0.421 A RMS) until someone measures a coil
  current.** Corrected 2026-09-02, when the motor was identified as a **Lin
  Engineering WO-208-13-01D rated 0.6 A per phase**. The `IRUN = 28` this file
  used to name is 0.679 A RMS and predates the motor: it is above the rating on
  either reading of that 0.6 A. The vendor does not say whether it is RMS or
  peak, so firmware enforces the conservative peak-of-sine reading and bring-up
  runs at `IRUN = 8` (0.211 A RMS). Settle it with a meter and a case
  temperature, then change the number in `src/Conversions.hpp` and record which
  reading won. `../mouse-joystick-pcb/docs/bringup-log.md` owns the finding.
- **The encoder and limit switch never reach a GPIO, on any revision.** Position
  comes from the driver's `X_ENC`/`XACTUAL`, homing state from driver status,
  both over SPI. There is no PIO encoder cross-check and no `ENCODER_MISMATCH`
  fault; they are deleted, not deferred.

- **`ENC_CONST` resets to zero, so `X_ENC` does not count until it is written.**
  An encoder reading a constant 0 is the symptom of forgetting that, not of a
  broken encoder. The register is write-only, so the only verification is that
  `X_ENC` tracks a commanded move.

- **Every move is bounded by a target position and watched against the
  encoder.** The rig's travel is about 4000 microsteps and the ramp generator
  has no idea when it has run out of mechanism, so `XACTUAL` advancing while
  `X_ENC` does not is how firmware detects an end stop, a stall, a slipping
  coupling or a dead encoder. Do not add a motion path that skips it.

- **The limit switch is at the POSITIVE end of travel and wired to `REFL`**,
  which stops *negative* motion. `SW_MODE.swap_lr` is required, or the hardware
  stop guards the direction that moves away from the switch. Measured
  2026-09-02; the board fact lives in
  `../mouse-joystick-pcb/docs/bringup-log.md`.
- **There are no LEDs, no test points, and on rev 0 no SWD header.** USB CDC and
  the two `FW_TRIG` pins on InstrumentPort-40 are the entire observable surface.
  Do not write a diagnostic that assumes an LED or a debugger.
- **Keep a proof of life that does not depend on USB.** `FW_TRIG0` carries a 1 Hz
  heartbeat and `FW_TRIG1` a boot signature, both reaching InstrumentPort-40
  through `U1801` on the always-on rail. On a board with no LED and no debugger
  that is the only way to distinguish "firmware is not running" from "USB is not
  working". Do not remove it to tidy up.
- **pyControl owns experiment sequencing.** This board owns local motion timing
  and safety. Never expose unbounded trajectories or raw motor power as the
  normal API.
- Rev 0 and rev 1 share the GPIO allocation. One env serves both.

## Safety Rules

Inherited from the board, and they are the board's rules rather than this
repository's to renegotiate:

- On boot: motor rail off, stepper outputs disabled, wait for an explicit `ARM`.
- During operation: require a host heartbeat; enforce current, velocity,
  acceleration, and travel limits; latch faults; assert the `FAULT` line; drop
  `HV_EN`.
- On host loss, heartbeat timeout, reset, or fault: stop motion, report the
  cause.

## Development Environment

Pixi provides PlatformIO; PlatformIO provides the RP2040 toolchain. The Makefile
is a thin wrapper over the Pixi tasks.

**Never change `lib_deps` back to a bare local path.** `symlink://src/MouseJoystickBoard`
is deliberate: a bare path makes PlatformIO copy the directory into
`.pio/libdeps/` and never refresh it, so edits to `src/MouseJoystickBoard/*.cpp`
silently do not get compiled and the build reports success while testing old
code. `symlink://` needs the manifest at `src/MouseJoystickBoard/library.json`;
both go together. Verify with a canary: append `#error` to a library `.cpp` and
confirm the build fails without cleaning first.

- `make install` — resolve the environment
- `make build` / `make rebuild` — compile
- `make upload PORT=/dev/ttyACM0` — flash
- `make monitor` — USB CDC serial, the only feedback channel
- `make test` — host-native Unity tests
- `make format` — clang-format staged changes

Select a different program with `EXAMPLE=`, a different board config with `ENV=`.

### Two Pixi Environments, Deliberately Disjoint

The `pycontrol` environment is declared `no-default-feature`, so PlatformIO and Qt
never resolve together. Someone building firmware never downloads Qt; someone
running the GUI never downloads a toolchain. Do not merge them to save a solve.

- `make pycontrol-install` — resolve it
- `make pycontrol-check` — verify pyControl's dependency gate before blaming the GUI
- `make pycontrol-test` — host-side pytest, no board needed (Gate C)
- `make pycontrol-link` — point a pyControl checkout at `pycontrol/`
- `make gui` — launch the pyControl GUI

Set `PYCONTROL_DIR` to a pyControl checkout, or let the tool autodetect one.

**pyControl fails its dependency check silently** — `pyControl_GUI.pyw` catches the
exception, writes `ErrorLog.txt`, and exits. "The GUI did not open" with nothing on
the terminal is that. Run `make pycontrol-check` first.

**Do not merge this environment with the rig's camera environment.**
`joystick-rig/scripts/constraints.txt` pins `numpy<2` for PySpin 4.3.0.190;
pyqtgraph 0.14 wants numpy 2.x. They are separate processes. `pycontrol/tests`
asserts on this so a merge fails loudly.

## Validation

- Host tests: `make test`. Covers `src/Conversions.hpp` only; everything else
  needs the board.
- Compile check: `make build`, plus `ENV=mouse_joystick_safeflash` when the
  change could touch the flash configuration and `ENV=mouse_joystick_rev1` when
  it touches anything revision-dependent.
- Note `make test` prunes the per-example firmware build directories. Build after
  testing, not before, if you want the artifacts.
- Real hardware: run `examples/RP2040SelfTest` and record the numbers in
  `../mouse-joystick-pcb/docs/bringup-log.md` under the board's serial. Numbers
  with units, not pass marks.

State plainly what you ran and what you did not. A compile is not a bring-up.

## Recording Results

Measured board behaviour belongs in `../mouse-joystick-pcb/docs/bringup-log.md`,
per board serial, not in this repository. Two boards in a rig are independently
calibrated and independently suspect; the serial is the flash unique id the self
test prints.

## Publication Layers

This repository is mirrored to GitHub for the rig's researchers, so it has two
layers (decided 2026-10-12):

- **Citable layer**: `README.md`, `CITATION.cff`, `LICENSE`, `pixi.toml`,
  `pixi.lock`, `platformio.ini`, `src/`, `examples/`, `test/`, `tools/`,
  `pycontrol/` and the documents the README lists. Building, flashing and
  testing must work from `pixi` alone on Linux, macOS and Windows, with no
  Orb Weaver checkout, and nothing in the layer refers to codeberg.org or to
  `orb-weaver-*` repositories. Files in other repositories are cited as
  `<repository>/<path>` with no `../` and no URL; `docs/development.md` maps
  each repository name to its URL. The USB manufacturer string `Orb Weaver`
  in `platformio.ini` is the boards' USB identity and stays.
- **Development layer**: `docs/development.md`, the documents it lists,
  `AGENTS.md`, `.agents/` and the `identify-analyzers` pixi task, which
  needs the `instrument-control` sibling. These may name the sibling
  checkouts and the Orb Weaver repositories.

A new document goes into exactly one of the two lists.
