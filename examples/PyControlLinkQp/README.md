# PyControlLinkQp — the pyControl protocol over QP/C++

Stage 4 again, structured as a QP/C++ (v7.3.4, QV kernel) application: the
same `ProtocolBoard` and `TmcMotion` as `examples/PyControlLink`, unchanged,
inside one active object, with a QS trace leaving the board on `FW_TRIG1` so a
logic analyser on the InstrumentPort can see what the firmware did, per
request, without USB and without touching the pyControl link.

It exists because the only thing QP buys this board is observability, and
that had to be measured before deciding anything about the delivery firmware.
The measurement is in `qp-arduino-example/evidence/2026-10-01-rp2040-joystick-link/`
(the pilot this application is derived from) and in the firmware run recorded
alongside it (see "Status"). `examples/PyControlLink` stays the non-QP
delivery sketch; nothing here changes the protocol or the pyControl side.

## What runs where

| Piece | Here | Notes |
|---|---|---|
| `ProtocolBoard`, `TmcMotion`, `SimulatedMotion` | `src/` | untouched; the corpus still holds |
| `LinkAo.cpp` | the active object | 64-bit clock, lines in, lines out: what `PyControlLink.ino`'s `loop()` did, as run-to-completion steps with trace records |
| `Bsp.cpp` | board support | `SafeState` first, boot signature and identity beacon, 100 Hz hardware-alarm tick with watchdog, non-blocking line transport, transmit-only QS on a PIO UART at `FW_TRIG1` |
| `AppConfig.hpp` | configuration | the mechanism and protocol numbers of `PyControlLink.ino` |
| `PyControlLinkQp.ino` | `setup()`/`loop()` only | event pool (8 lines), queue (16 events), start |

The qp-arduino library comes from the sibling checkout `../qp-arduino`
(branch `upgrade/qpcpp-v7`) through `platformio.ini`'s `[qp_env_data]` until
it is released; the link is `symlink://`, never a bare path, for the reason the
board library already documents.

## Signals and ports

| Signal | Pin (from `BoardPins.hpp`) | Behaviour |
|---|---|---|
| pyControl protocol | UART0 `kPycUartTx`/`kPycUartRx`, 115200 8N1 | as before; `MJ_LINK_USB=1` moves it to USB CDC for a bench with no host |
| QS trace | `kFwTrig1` as a PIO UART TX, 115200 8N1, transmit only | starts after the boot signature and the identity beacon, which still use the pin as GPIO; nothing on this line can command the board |
| operation marker | `kFwTrig0` | high from the start of a homing or move to its completion (`OP_START`/`OP_DONE`) |
| USB CDC | `Serial` | a banner at boot; in the bench build, the protocol |

`FW_TRIG1` therefore no longer carries the FREEWHEEL level of `PyControlLink`:
that state is a `LINK_STATE` record. Which marker roles the v1.0 board's
InstrumentPort overlay finally assigns is an `OBS-IF` decision still owed
(`../mouse-joystick-pcb/docs/observability-plan.md`); the two uses here are
one-line changes.

## Trace records

| Record | Fields | Meaning |
|---|---|---|
| `LINK_RX` | seq, verb, length | a protocol line was dispatched to the AO |
| `LINK_TX` | sent flag, ready time, first 20 characters | a reply or event was queued for the port (0 = ring full, dropped) |
| `LINK_STATE` | name | `ProtocolBoard`'s state changed |
| `OP_START` / `OP_DONE` | homing flag, angle | an operation started / completed; `FW_TRIG0` follows |
| `LINK_WORK` | advance µs, feed µs, output µs | where a dispatched line's time went |
| `TRACE_STAT` | high-water, buffer-full count, size, ticks | QS buffer use, every second |
| `LINK_STAT` | lines, drops (pool, queue, ring), overlong, queue min free, pool min free, ring high-water | transport and budget use, every second |
| `TICK_LOST` | ticks, re-arms, catch-ups | the tick watchdog fired (never expected) |

Plus QP's own state transitions, entry/exit and assertions. The per-event
dispatch/post/get records are filtered out by default (`LINK_QS_FULL=1` keeps
them, and saturates the line at a 100 Hz tick).

## Builds

| Env | For |
|---|---|
| `mouse_joystick_qp` | rev 0 (pcb v0.1/v0.2): `TmcMotion`, UART0, trace on `FW_TRIG1` |
| `mouse_joystick_qp_rev1` | rev 1 (pcb v0.3 and later) |
| `mouse_joystick_qp_sim_usb` | a board with no 36 V and no host: `SimulatedMotion`, protocol over USB CDC, tick probe on the unused UART0 TX pad (InstrumentPort `DIG8`) |

```sh
pixi run build examples/PyControlLinkQp mouse_joystick_qp
pixi run upload examples/PyControlLinkQp mouse_joystick_qp_sim_usb /dev/ttyACM0
```

Drive the bench build with `../qp-arduino-example/tools/link_pilot.py` and
read the trace as `qp-arduino-example/evidence/bench-scripts/link_run.sh`
does (sigrok `uart` decoder on channel 17, bytes to `qspy -k -f`).

## Status

- 2026-10-01: all three environments build; the 114 host tests still pass.
  The bench build ran on the rev 0 fixture (board E4654C159F2C2534, 36 V
  unplugged) under the pilot's full sequence: 21 of 21 steps, 1,003 lines
  and 1,206 replies with none dropped, QS high-water 482 of 2,048 bytes with
  zero buffer-full, queue and pool never below 15 of 16 and 6 of 8 free,
  tick exact, host-loss fault 105 ms after the last heartbeat. The delivery
  build was flashed and its boot sequence captured: signature, beacon, then
  the trace with `READY` and a statistics record every second, nothing on
  UART0. Results in
  `qp-arduino-example/evidence/2026-10-01-rp2040-joystick-firm-qp/`.
- **Not yet run:** `TmcMotion` behind this AO with the rail up (as for
  `PyControlLink`, whose first supervised run it still owes), the UART0
  protocol path under QP with a pyControl host, and anything on a v1.0 board,
  whose GPIO changes (`GP24` becomes `HV_LATCH`, `~HV_FAULT` moves, the DAC
  goes to I2C) are not in `BoardPins.hpp` yet.
- The RP2040 hardware watchdog task of `.agents/PLANS.md` applies here as it
  does to `PyControlLink`; `Q_onError` already drops to `SafeState` before
  rebooting.
