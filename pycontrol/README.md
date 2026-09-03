# pyControl Host Side (Stage 4)

The other end of the UART protocol in
[`../../mouse-joystick-pcb/docs/uart-protocol.md`](../../mouse-joystick-pcb/docs/uart-protocol.md).
The RP2040 firmware in `../src` speaks it over RJ-45 pins 1 and 4; the MicroPython
device class here speaks it from the pyControl board.

They live in one repository because they are one contract. A protocol with
sequence numbers, `ack`/`nak`, and a nine-bit fault mask cannot have its two ends
version independently.

**Target controller: Breakout Board 1.2.** UART-capable ports are **1**
(`UART4`), **3** (`USART1`), and **4** (`USART3`) — see
`../../mouse-joystick-pcb/docs/pycontrol-port-pinout.md`. Two boards fit with a
port to spare. The device class asserts `port.UART is not None`, so a
non-UART port fails loudly rather than silently not receiving.

## Layout

| Path | Runs on | Delivered by |
|------|---------|--------------|
| `devices/` | pyControl board (MicroPython) | symlinked into pyControl's own `devices/` |
| `hardware_definitions/` | pyControl board (MicroPython) | `config/settings.json` folder redirect |
| `tasks/` | pyControl board (MicroPython) | `config/settings.json` folder redirect |
| `sim/` | host (CPython) | not delivered; test infrastructure |
| `tests/` | host (CPython) | not delivered; this is Gate C |

`devices/` and below is MicroPython that pixi never executes — pixi supplies the
host GUI that *uploads* it. `mouse_joystick_protocol.py` is the exception: it
runs in both places, which is why it has no pyControl imports.

**Sibling imports inside `devices/` must be package-qualified** —
`from devices.mouse_joystick_protocol import ...`, matching pyControl's own
`from devices.poke import _Poke`. On the pyboard the uploaded files live inside
the `devices` package and `/flash/devices` is not on `sys.path`, so a bare import
raises ImportError there while passing every host test.
`tests/test_device_class.py` asserts this against the source text, because no
runtime test on a host can reproduce the board's import environment.

## Two Delivery Mechanisms, Not One

`../../mouse-joystick-pcb/docs/validation-plan.md` Gate E requires that "an
unforked upstream pyControl installation loads the device class through its
normal supported mechanism". `make pycontrol-link` does that, and it needs two
different mechanisms because pyControl treats the folders differently:

- **`hardware_definitions` and `tasks` are redirected** through
  `config/settings.json`. pyControl resolves both through
  `get_setting("folders", ...)`, and upstream's own `.gitignore` excludes them —
  they are declared user-owned. Redirecting them means this repository holds the
  only copy.

- **`devices` is *not* redirected, and must not be.**
  `source/communication/pycboard.py` builds its device-class map by scanning
  exactly one folder (`os.listdir(user_folder("devices"))`). Point that at this
  repository and `Breakout_1_2`, `UART_handler` and every other stock driver
  disappear, so the hardware definition fails to import. The device file is
  symlinked *into* pyControl's `devices/` instead, which is the upstream
  mechanism: any `class` in any `.py` there is auto-discovered with no
  registration step.

On Windows, where symlinks need developer mode, `link` copies and says so. A
copy goes stale, so re-run `make pycontrol-link` after editing the device class.

Undo either with `make pycontrol-unlink`.

## Editing Cycle, And The Trap In It

The collaborator's rig hardware definition
(`~/Repositories/github/joystick-rig/pycontrol/hardware_definitions/hardware_definition_joystick.py`)
documents this from experience, and it applies here unchanged:

> Editing this file on disk does NOT change the pyboard. To make a change take
> effect: edit → Config → Load hardware definition → re-Upload the task →
> restart the session.

And separately: **Config → "Load framework" deletes `hardware_definition.py`
from the pyboard.** After reloading the framework you must reload the hardware
definition, or task upload fails with `ImportError: no module named
'hardware_definition'`.

## Gate C

`../../mouse-joystick-pcb/docs/uart-protocol.md` asks for "a host-side simulator
or protocol fixture so both this firmware and the pyControl device class can be
tested before real hardware and a real pyControl installation are both
available." `make pycontrol-test` runs it: no board, no breakout, no pyControl
checkout.

| File | What it is |
|------|-----------|
| `devices/mouse_joystick_protocol.py` | The framing codec. Runs on the board *and* under pytest, which is why it has no pyControl imports. |
| `devices/mouse_joystick.py` | The device class. |
| `sim/board_sim.py` | Host simulator: state machine, faults, retransmission, watchdog. Injected clock, so every timestamp is deterministic. |
| `tests/fixtures/protocol_cases.txt` | The shared corpus. |

**Gate C is not passed yet, and the missing half is the firmware.** The corpus is
line-oriented rather than JSON specifically so a PlatformIO native C++ test can
parse it with `getline` and a prefix compare. Until that test exists, the
simulator agreeing with a corpus that only the simulator has ever been checked
against proves nothing about the RP2040 — two independent readings of a draft
agree right up until they do not, and the corpus is only worth something once
both are held to it.

The simulator is **not** a model of the board. No TMC5130A, no encoder, no
current control, no real motion profile. Position moves linearly at a configured
velocity because the protocol is what is under test.

## Scoped To Rev 0 Hardware

Narrowed 2026-09-02 to what the boards in hand can actually do. Writing firmware
against absent hardware means the tests assert what was guessed.

| Implemented | Absent, and why |
|---|---|
| `ID` `PING` `STATUS` `LIMITS` `FAULTS` `CLRFAULT` `HV` `INSTR` `CAL` | `BRAKE` / torque — no electronic brake, no torque sensor |
| `HOME` `ARM` `DISARM` `MOVETO` `MOVE` `STOP` | `ANGOUT` `ANGSCALE` — analog outputs not populated |
| `FREEWHEEL`, `ARMPULL` (testable by moving the joystick by hand) | |

Unimplemented verbs answer `badverb`, never a silent ack — a stub that acked
would let a host believe the analog output was live and record a trace of
nothing.

`ID` reports `caps=motion+home+freewheel+instr+hv`. A host should gate on that
rather than on a firmware version, so a rev-1 board simply reports more.

### The trial shape, and what it changed

Park (`HOME`) → present (`MOVETO`) → release (`FREEWHEEL`) → the animal pulls →
present again. Two consequences that are not in the draft:

**Homing is no longer how position is recovered.** The previous rig had no
encoder, so every pull lost the stepper's position outright and every trial had
to start with a home. Here the encoder keeps it. Homing is now for *parking* the
joystick between trials — a choice, not a necessity.

**`FREEWHEEL` is a rev-0 verb with a known expiry date.** On a board with the
brake it must not exist separately: the brake has to engage *before* motor drive
is released, or the joystick drops as it goes compliant. Two host commands cannot
guarantee that ordering — the host can die between them, or the second message
can arrive late over a link whose whole point is that it has latency. It belongs
inside one `BRAKE <torque>`. Expect this verb to be replaced, not extended.

Leaving compliance is likewise **implicit**: a `MOVETO`, `MOVE`, `HOME` or `STOP`
reloads position from the encoder, re-engages, and proceeds — one command where
an explicit "stop freewheeling" would be two, with a window in between.

### The resync fights the stall detector

`.agents/AGENTS.md`: "`XACTUAL` advancing while `X_ENC` does not is how firmware
detects an end stop, a stall, a slipping coupling or a dead encoder. Do not add a
motion path that skips it."

That detector *is* the divergence between the two numbers — so reloading
`XACTUAL` from the encoder deletes it. Doing it continuously, or on a timer,
trades the board's only stall detection for not having to track which state it is
in. The two cases are distinguishable by state and by nothing else:

| State | Divergence means | Response |
|---|---|---|
| driving | end stop, stall, slipping coupling, dead encoder | fault |
| compliant | the animal pulled | resync, on exit only |

`STATUS` therefore reports **both** `angle` (encoder — the joystick's real
position) and `xactual` (ramp generator). The draft's single "position" field
cannot express the difference, and after a pull they disagree by the whole pull.

**Stall detection is not implemented.** It needs a fault bit the draft does not
have — bit 4 is retired and permanently reserved, and it was a *different*
comparison anyway (driver encoder vs. an RP2040 PIO count that never existed).
This is `XACTUAL` vs `X_ENC`, both inside the driver. Raised below rather than
given a bit unilaterally.

## What Implementing The Draft Surfaced

`uart-protocol.md` is marked "Draft v0.1. Not implemented, not frozen." Writing
two ends of it turned up the following. Everything below is marked `SPEC:` at the
point in the code where it was decided.

### Two that are real problems, not just gaps

**1. The retransmission rule breaks the heartbeat.** The draft says "a repeated
`seq` with identical content is treated as a retransmission and re-acknowledged
without re-executing." That rule exists so a retried `MOVETO` does not move the
joystick twice — it assumes re-execution is the harm. For `PING` the side effect
*is* the command: it has no result other than resetting the host-loss watchdog.
Serving a repeated `PING` purely from cache means the safety timeout expires
while the host believes it is being answered, and the board faults mid-experiment
for no reason the host can see. A host that increments `seq` never hits this,
which is exactly why it would survive review and then fire after a 65536 wrap.
The simulator refreshes the watchdog on a cache hit; the cleaner fix is probably
to exempt `PING` from the cache entirely. **Needs a decision before freeze.**

**2. The host timeout is shorter than a long move.** The cited host timeout is
100 ms (`pycontrol-port-pinout.md`). At the simulator's default 180 °/s, an 18°
move takes exactly that long — and the travel range is 40° wide. A task that
issues a long move and then waits for `TARGET` gets faulted for heartbeat loss
every time. This may be fine (the host simply must ping during motion, which
`tasks/mouse_joystick_check.py` does at 40 ms) but it should be stated in the
draft rather than discovered. **Confirm the real velocity and the real timeout
together, not separately.**

### Gaps that were filled to make the thing run

| Item | Draft says | Decided here |
|------|-----------|--------------|
| Verb case | Table is `MOVETO`, examples are `moveto;417;-12.5` | Decode either, emit lowercase |
| `outofrange` example state | `moveto;421;-999 → nak;421;outofrange`, state unstated | ARMED is checked **before** range, so a fresh board answers `notarmed`. Safety rule should not be reachable past an argument check |
| Ack data format | One example, `ack;419;enc_offset=1024;angle_zero=0.0` | All data-carrying acks are `key=value` |
| `ID`/`STATUS`/`LIMITS` fields | Prose lists | Named keys, pinned byte-exact in the corpus |
| Max line length | Only "rxbuf=130" as a hint | `MAX_LINE = 120`, leaving room to answer `badframe` inside a 130-byte buffer |
| `nak` reasons | Names 4: `badverb`, `outofrange`, `badframe`, `latched` | Added `badargs`, `notarmed`, `nothomed`, `faulted`, `busy`, `nohv` |
| Retransmit cache size | Unbounded | 16. Unbounded is a slow leak on a device with no way to report one |
| When the watchdog is armed | Unstated | At `ARM`, released at `DISARM`. Faulting an idle unarmed board before the host has connected would be absurd |
| `HV` as a motion precondition | Unstated | `HOME` and motion require it; new reason `nohv` |
| `\r` handling | "A leading `\r` is tolerated" | Stripped from both ends — a CRLF peer produces a *trailing* one, so the literal reading leaves the common case broken |
| Checksum | Explicitly open | Not implemented. Decide with a measured error rate on the real cable, as the draft says |

## Not Covered By Any Host Test

Interrupt timing, `UART.IRQ_RXIDLE` actually firing, rxbuf overrun, MicroPython's
own UART driver, and every electrical property of the link. Those need the
Breakout 1.2 and a board. A green `make pycontrol-test` is not a bring-up.
