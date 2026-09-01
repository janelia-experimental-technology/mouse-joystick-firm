# mouse-joystick-firm

Firmware for `mouse-joystick-pcb`: an RP2040 driving one TMC5130A stepper
controller, exposed to a pyControl host over one UART. One board, one joystick
axis. Two of these boards serve the rig's two joysticks.

The board lives in
[`../mouse-joystick-pcb`](../mouse-joystick-pcb) and owns every electrical fact
this firmware depends on. The driver library is
[`TMC51X0`](https://github.com/janelia-arduino/TMC51X0).

**Status:** stage 1 only. `examples/RP2040SelfTest` compiles and its host tests
pass; it has not yet been run on hardware.

## Quick Start

```sh
make install                  # resolve the pixi environment (pixi + platformio)
make build                    # compile examples/RP2040SelfTest
make test                     # host-native tests, no board needed

# with the board on USB and the 36 V input UNPLUGGED:
make upload PORT=/dev/ttyACM0
make monitor PORT=/dev/ttyACM0
```

Uploading the first time needs the board in BOOTSEL. See
[`docs/rp2040-bringup.md`](docs/rp2040-bringup.md) — on this board that is more
awkward than usual, and the self test gives you a way to avoid repeating it.

## Bring-Up Is Staged

The board has two power domains with a bus switch between the MCU and everything
else. The stages follow that boundary:

| Stage | What runs | 36 V | Bus switch | Status |
|-------|-----------|------|-----------|--------|
| 1 | RP2040 alone | absent | isolated | `examples/RP2040SelfTest` |
| 2 | TMC5130A register reads | applied | enabled | not written |
| 3 | Motion | applied | enabled | not written |
| 4 | pyControl UART protocol | applied | enabled | not written |

**Stage 1 has no way to raise `HV_EN` and no way to enable the bus switch.** That
is deliberate, not an omission: until the MCU is proven there is no reason to put
36 V near it, and an MCU pin driving into an unpowered TMC5130A back-powers a
dead rail through its ESD clamps.

## What Stage 1 Actually Checks

Run `examples/RP2040SelfTest`, read the USB serial monitor. Roughly twenty
checks, each reporting a number or a level:

- **Identity** — the flash unique id, which *is* this board's serial. Calibration
  belongs to the serial, not to a joystick position.
- **Clocks** — every derived clock, plus `clk_sys` measured against the crystal.
  The crystal's own absolute proof is that USB enumerated at all; the `k` command
  measures it against your host clock if you want a number.
- **QSPI flash** — a real `0x4B` read proves the W25Q128 responds, and an address
  aliasing probe at +2, +4 and +8 MiB proves the part is really 16 MiB rather
  than trusting the BOM.
- **RAM** — a four-pattern walk over a 16 KiB heap block.
- **Board pull resistors** — `GP9`, `GP24`, `GP1` and `GP29` read with internal
  pulls off, so each level comes from a board resistor. A failure here is a
  missing or misplaced part, not firmware.
- **Spare pads** — the nine unrouted GPIOs pulled both ways. A pad that will not
  move is shorted.
- **ADC** — the switched-rail sense (expected ~0 V), the 36 V current sense
  (expected ~0 V), and the die temperature, which proves the ADC and `ADC_AVDD`.

Then an interactive menu: re-run, read the ADC, pulse the InstrumentPort
triggers, stream clock ticks, try a pyControl UART loopback, or reboot into
BOOTSEL.

## Things About This Board That Will Bite You

Collected here because none of them are guessable from the code:

- **There are no LEDs.** Not one. USB CDC and the two `FW_TRIG` pins on
  InstrumentPort-40 are the entire observable surface. `LED_BUILTIN` resolves to
  GPIO 25, which here is `FW_TRIG0`.
- **There are no test points, anywhere.** Reach a signal at a component pin or
  through InstrumentPort-40.
- **Rev 0 has no SWD header and no `RUN` connection.** No debugger during
  bring-up. BOOTSEL recovery needs USB *and* pyControl unplugged, because the MCU
  domain is ORed from both.
- **`Serial2` and `Wire1` must never be called.** The `generic` board definition
  maps them onto GPIO 8/9 and 26/27, which on this board are `TMC_DIAG1`,
  `~{BUS_SW_EN}`, the rail sense ADC, and a spare pad.
- **The GPIO bus is isolated at reset and only a pull-up keeps it that way.**
  R601 7.87 kΩ. All four `SN74CB3Q3245` packages run from the always-on rail, so
  `Ioff` never engages — the enable pin is the only mechanism, not the inner of
  two.
- **`HV_DIAG_EN` must never idle high.** Held high it leaks ~240 µA into
  `+36V_SW` through the switch's open-load detection; the rail creeps past the
  buck's minimum input in about three seconds and the switched domain hiccups
  instead of staying off.
- **`vsense = 1` with the 0.15 Ω shunts.** It resets to 0, which at 0.15 Ω drives
  roughly 1.8× the motor's rated current. Stage 3 must set and read it back
  before enabling the driver.
- **The encoder and limit switch never reach a GPIO,** on any revision. Position
  comes from the driver's `X_ENC`/`XACTUAL` over SPI.

## Pin Map

`src/BoardPins.hpp` is the single source of truth and carries the reasoning.
Summary:

| GPIO | Net | Function | Isolated |
|------|-----|----------|----------|
| 0 | `PYC_UART_TX` | pyControl UART0 TX | no |
| 1 | `PYC_UART_RX` | pyControl UART0 RX, 10 kΩ pull-up | no |
| 2 | `~{TMC_CS}` | driver chip select | yes |
| 3 | `~{TMC_DRV_EN}` | driver enable, active low | yes |
| 4 | `TMC_SPI_CIPO` | SPI0 RX | yes |
| 5 | `TMC_DIAG0` | driver diagnostic 0 | yes |
| 6 | `TMC_SPI_SCK` | SPI0 SCK | yes |
| 7 | `TMC_SPI_COPI` | SPI0 TX | yes |
| 8 | `TMC_DIAG1` | driver diagnostic 1 | yes |
| 9 | `~{BUS_SW_EN}` | GPIO isolation enable, active low | — |
| 10, 11, 13 | `DAC_SPI_*`, `~{DAC_CS}` | rev 1 angle DAC, SPI1 | yes |
| 14 | `HV_DIAG_EN` | 36 V switch diagnostics enable | no |
| 19 | `HV_EN` | 36 V input switch enable | no |
| 21, 25 | `FW_TRIG1`, `FW_TRIG0` | firmware timing markers | no |
| 24 | `~{INSTR_BUF_OE}` | instrumentation buffer enable | no |
| 26 | `SW_RAIL_SENSE_DIV_2` | `+3.3V_SW` ÷2, ADC0 | no |
| 28 | `HV_SNS` | 36 V load current sense, ADC2 | no |
| 29 | `~{HV_FAULT}` | 36 V switch fault, 10 kΩ pull-up | no |
| 12, 15, 16, 17, 18, 20, 22, 23, 27 | — | spare, routed nowhere | — |

Rev 0 and rev 1 share this allocation by design. No rev 0 pin moved.

## Layout

```
examples/RP2040SelfTest/   stage 1 program
src/                       board support library
  BoardPins.hpp            GPIO allocation, single source of truth
  Conversions.hpp          ADC scaling, Arduino-free so the host can test it
  SafeState.hpp            safe idle state; the only thing that touches HV_EN
  SelfTest.hpp             the checks
test/native/               host tests for the Arduino-free arithmetic
tools/pio_task.py          example selection without editing platformio.ini
docs/rp2040-bringup.md     the procedure, and what each failure means
.agents/                   agent guidance and plans
```

The example to build is selected by `PLATFORMIO_SRC_DIR`, not by editing
`platformio.ini`, and each example gets its own build directory so switching
between them does not invalidate a cache. Same arrangement as `TMC51X0`.

## Environments

| Env | Purpose |
|-----|---------|
| `mouse_joystick` | the board. Universal `boot2_generic_03h` stage 2 — slower XIP, but boots regardless of the flash's quad-enable bit, which is what a first-article board wants. |
| `mouse_joystick_fastflash` | same board, quad-lane stage 2 matched to the W25Q128JV. Use once stage 1 has passed. |
| `native` | host tests. |

## Recording Results

Measured behaviour goes in `../mouse-joystick-pcb/docs/bringup-log.md`, under the
board's serial, as numbers with units. Two boards in one rig are independently
calibrated and independently suspect.
