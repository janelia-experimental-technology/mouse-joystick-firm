# mouse-joystick-firm

Firmware for `mouse-joystick-pcb`: an RP2040 driving one TMC5130A stepper
controller, exposed to a pyControl host over one UART. One board, one joystick
axis. Two of these boards serve the rig's two joysticks.

The board lives in
[`../mouse-joystick-pcb`](../mouse-joystick-pcb) and owns every electrical fact
this firmware depends on. The driver library is
[`TMC51X0`](https://github.com/janelia-arduino/TMC51X0).

**Status:** stages 1 to 3 have passed on hardware.

- **Stage 1**, 2026-09-01, board `E4654C159F2C2534`: 27 checks, 0 failed.
- **Stage 2**, 2026-09-02, both boards: `IOIN.VERSION` = `0x11`.
- **Stage 3**, 2026-09-02, board `E4654C159F4D1334`: motor turns, encoder
  tracks it to 1.000, homing repeats to 4 encoder counts.

Results are recorded in `../mouse-joystick-pcb/docs/bringup-log.md`, per board
serial. Stage 3's procedure and findings are in `docs/motion-bringup.md`.

The pin map is verified against PCB git tag **v0.1**, the revision manufactured
as rev 0. Tags v0.1 and v0.2 have byte-identical RP2040 pad-to-net maps.

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

| Stage | What runs | 36 V | Bus switch | Driver | Status |
|-------|-----------|------|-----------|--------|--------|
| 1 | RP2040 alone | absent | isolated | disabled | `examples/RP2040SelfTest` — **passed** |
| 2 | TMC5130A register reads | applied | closed | disabled | `examples/TMC5130Communication` — **passed** |
| 3 | Motion, encoder, homing | applied | closed | enabled | `examples/MotionAndHoming` — **passed** |
| 4 | pyControl UART protocol | applied | closed | as needed | not written |

**Stage 1 has no way to raise `HV_EN` and no way to enable the bus switch.** That
is deliberate, not an omission: until the MCU is proven there is no reason to put
36 V near it, and an MCU pin driving into an unpowered TMC5130A back-powers a
dead rail through its ESD clamps.

**Stage 2 has no way to enable the driver.** It reads registers with
`~{TMC_DRV_EN}` held high throughout and writes nothing, so no current can reach
the motor. Enabling the driver depends on the current-sense decision — `vsense = 1`
with the 0.15 Ω shunts — which belongs to stage 3.

`src/PowerSequence.hpp` is the only file that raises `HV_EN`, and it enforces the
one safe ordering: rail up before the bus closes, bus open before the rail drops.

## What Stage 1 Actually Checks

Run `examples/RP2040SelfTest`, read the USB serial monitor. Roughly twenty
checks, each reporting a number or a level:

- **Boot signature and heartbeat** — a distinctive pattern on the InstrumentPort
  triggers before Serial comes up, then a 1 Hz square wave forever after. This is
  the only proof that the RP2040 is running your code that does not depend on
  USB. Watch LA5032 channels 16 and 17.
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
  roughly 1.8× the intended current. Stage 3 sets and reads it back
  before enabling the driver.
- **The encoder and limit switch never reach a GPIO,** on any revision. Position
  comes from the driver's `X_ENC`/`XACTUAL` over SPI.

## Stage 2: What It Checks

Run `examples/TMC5130Communication` with 36 V on `J201`. It is interactive and
starts powered down — send `s` to begin, `x` to power down.

1. Confirms `+3.3V_SW` is genuinely dead before raising anything, and refuses to
   continue if it is not.
2. Raises `HV_EN` and measures how long `+3.3V_SW` takes to come up.
3. Reads the 36 V switch's own diagnostics, raising `HV_DIAG_EN` only for the
   duration of the read and lowering it again.
4. **Then** closes the GPIO isolation bus switch.
5. Reads `IOIN`, `GSTAT`, `GCONF`, `CHOPCONF`, `DRV_STATUS`, `XACTUAL` and
   `X_ENC`, printing every datagram byte so the serial log can be compared
   against a logic analyser capture.

The check that matters most is `IOIN` bit 4, `DRV_ENN`: it reads back the level
firmware drives on `GP3` *through the bus switch*, and nothing else in the test
proves that path. Expected `IOIN` on this board is `0x11000010`, with bit 7 free
because `SWCOMP_IN` is a comparator output.

Two failure signatures are called out explicitly rather than left as "SPI
failed": all bytes `0x00` means CIPO is undriven, which on this board points at
the bus switch enable, then `+3.3V_SW` at the driver's `VCC_IO`, then CS. All
bytes `0xFF` means CIPO is stuck high, which the board's 10 kΩ pull-down should
prevent.

### Watching it with the LA5032

`FW_TRIG1` goes high for the whole register-read burst, so:

```sh
cd ../instrument-control
guix shell -m manifest.scm -- sigrok-cli --driver kingst-la2016 \
  --channels CH2,CH3,CH4,CH5,CH17 --config samplerate=10m \
  --triggers CH17=r --samples 2000000 \
  -P spi:clk=CH2:mosi=CH3:miso=CH4:cs=CH5:cpol=1:cpha=1 -A spi
```

`CH2`–`CH5` are `SCK`, `COPI`, `CIPO`, `~CS`. The TMC5130A is SPI mode 3, hence
`cpol=1:cpha=1`.

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
| 10, 11, 13 | `DAC_SPI_*`, `~{DAC_CS}` | rev 1 angle DAC, SPI1. **Unrouted on rev 0** | yes |
| 14 | `HV_DIAG_EN` | 36 V switch diagnostics enable | no |
| 19 | `HV_EN` | 36 V input switch enable | no |
| 21, 25 | `FW_TRIG1`, `FW_TRIG0` | firmware timing markers | no |
| 24 | `~{INSTR_BUF_OE}` | instrumentation buffer enable | no |
| 26 | `SW_RAIL_SENSE_DIV_2` | `+3.3V_SW` ÷2, ADC0 | no |
| 28 | `HV_SNS` | 36 V load current sense, ADC2 | no |
| 29 | `~{HV_FAULT}` | 36 V switch fault, 10 kΩ pull-up | no |
| 12, 15, 16, 17, 18, 20, 22, 23, 27 | — | spare, routed nowhere | — |

Rev 0 and rev 1 share this allocation by design. No rev 0 pin moved. The only
firmware-visible difference is GP10/GP11/GP13, unrouted on rev 0 and therefore
folded into the spare-pad test — see `MJ_BOARD_REV`.

**Reference designators are not stable across revisions.** Automatic annotation
orders by sheet position, so inserting a part rotates everything after it.
`R1901`/`R1902` are 100 Ω pyControl series resistors at v0.1 and 1 kΩ
`HV_EN`/`HV_DIAG_EN` series resistors at HEAD. Designators quoted in this
repository are **v0.1** designators; identify parts by value and net.

## Layout

```
examples/RP2040SelfTest/       stage 1 program
examples/TMC5130Communication/ stage 2: raw SPI, no driver library
examples/MotionAndHoming/      stage 3: motion, encoder, homing, over TMC51X0
src/                           board support library
  BoardPins.hpp                GPIO allocation, single source of truth
  Conversions.hpp              ADC and motion scaling, Arduino-free so the
                               host can test it; also the fitted parts' facts
  SafeState.hpp                safe idle state
  PowerSequence.hpp            the switched rail; the only thing that touches
                               HV_EN
  SelfTest.hpp                 the stage 1 checks
  TmcSpi.hpp                   stage 2's raw datagram layer
  MotionSetup.hpp              stage 3's TMC51X0 glue: this board's facts, and
                               CHOPCONF.vsense, which TMC51X0 cannot express
test/native/                   host tests for the Arduino-free arithmetic
tools/pio_task.py              example selection without editing platformio.ini
docs/rp2040-bringup.md         stage 1: the procedure, and each failure's meaning
docs/motion-bringup.md         stage 3: the same, plus the rig's measured facts
.agents/                       agent guidance and plans
```

The example to build is selected by `PLATFORMIO_SRC_DIR`, not by editing
`platformio.ini`, and each example gets its own build directory so switching
between them does not invalidate a cache. Same arrangement as `TMC51X0`.

## Environments

| Env | Purpose |
|-----|---------|
| `mouse_joystick` | **the board in hand.** `MJ_BOARD_REV=0`. Universal `boot2_generic_03h` stage 2 — slower XIP, but boots regardless of the flash's quad-enable bit, which is what a first-article board wants. |
| `mouse_joystick_fastflash` | same board, quad-lane stage 2 matched to the W25Q128JV. Use once stage 1 has passed. |
| `mouse_joystick_rev1` | `MJ_BOARD_REV=1`, for PCB HEAD where GP10/GP11/GP13 carry the angle DAC. |
| `native` | host tests. |

### Two Build-System Traps

- **The board support library is declared as `symlink://src/MouseJoystickBoard`,
  not as a bare path.** A bare local path makes PlatformIO *copy* the directory
  into `.pio/libdeps/` and it never refreshes the copy — not on a rebuild, and
  not on `-t clean`. Edits to `src/MouseJoystickBoard/*.cpp` then silently do not
  get compiled and the build succeeds while testing old code. `symlink://`
  requires the library to carry a manifest, which is why
  `src/MouseJoystickBoard/library.json` exists. **`TMC51X0` has the same bare-path
  arrangement and therefore the same trap.**
- **`make test` wipes the firmware build directories.** The native test run uses
  the default build dir and prunes the per-example ones. Harmless, but do not
  read it as a failed build.

## Recording Results

Measured behaviour goes in `../mouse-joystick-pcb/docs/bringup-log.md`, under the
board's serial, as numbers with units. Two boards in one rig are independently
calibrated and independently suspect.
