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
| `tests/` | host (CPython) | not delivered; this is Gate C |

Only `tests/` runs under the `pycontrol` pixi environment. Everything else is
MicroPython that pixi never executes — pixi supplies the host GUI that *uploads*
it.

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

## Gate C Comes First

`../../mouse-joystick-pcb/docs/uart-protocol.md` asks for "a host-side simulator
or protocol fixture so both this firmware and the pyControl device class can be
tested before real hardware and a real pyControl installation are both
available." That suite runs in `tests/` with `make pycontrol-test` and needs no
board, no breakout, and no pyControl checkout.

Currently `tests/` contains only `test_environment.py`, which verifies the pixi
environment can actually run pyControl. The protocol conformance tests are the
next piece of work.
