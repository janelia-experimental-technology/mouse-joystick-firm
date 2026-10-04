# Stage 1: RP2040 Bring-Up

**Board:** `mouse-joystick-pcb` rev 0, first assembled article received
2026-09-01. Verified against PCB git tag **v0.1**. Also run on a **`v0.3`** unit
on 2026-09-09 — see the env table below, which is not the same env.
**Program:** `examples/RP2040SelfTest`, env `mouse_joystick` (`MJ_BOARD_REV=0`)
for a v0.1/v0.2 board, `mouse_joystick_rev1` (`MJ_BOARD_REV=1`) for v0.3.
**Scope:** everything on the always-on `+3.3V_MCU` domain. Nothing else.

Record results in `../mouse-joystick-pcb/docs/bringup-log.md` under the board's
serial, as numbers with units.

## Which PCB Revision This Matches

The firmware pin map was taken from footprint `U501` in the `.kicad_pcb` at tag
**v0.1**. Tags v0.1 and v0.2 have **byte-identical RP2040 pad-to-net maps**, so
the ambiguity between them does not matter. Tag **v0.3** differs on exactly
three pins — `GP10`, `GP11`, `GP13` — which it routes to the angle-output DAC
and v0.1/v0.2 leave unrouted. No connected pin moved.

**pcb v1.0** (the board file from commit `d5ae4b0`, 2026-09-21, unchanged
through the fabrication package and HEAD; no tag yet) is a different map, read
off footprint **`U601`** — the RP2040 moved into the combined MCU block and
`U501` is now the 36 V switch. Eight pins change role and one leaves; see
`src/BoardPins.hpp` and the README's pin map. Its first firmware was written
on 2026-10-04 and has not run on a board.

**Pick the env from the revision the board was fabricated at, not from "is
this the newest board".**

| PCB | env | `MJ_BOARD_REV` | spare pads | stage 1 checks |
| --- | --- | --- | --- | --- |
| `v0.1`, `v0.2` | `mouse_joystick` | 0 | 12 | 27 |
| `v0.3` | `mouse_joystick_rev1` | 1 | 9 | 24 |
| v1.0 (`d5ae4b0`…HEAD) | `mouse_joystick_rev2` | 2 | 2 | 20 (derived from the rev 1 count; confirm on the first board) |

On v1.0, section `[6]` checks `R607` (bus isolated), `R403`/`R402` (the I2C
pair idles high on the MCU side), `R401` (RX idle high) and `R507` on **GP23**
(no 36 V fault), and prints the level of `HV_LATCH` on GP24 without judging
it, because the switch it belongs to is unpowered in stage 1. Section `[8]`
adds `BRAKE_ISENSE` (GP27) and `ANGLE_OUT_MON` (GP29), both expected at 0 V
with the rail off. The `i` command does nothing: the Digital-50 buffers have
no enable.

`env:mouse_joystick_rev1` is **not** a future-only env — a `v0.3` board needs
it. Build `env:mouse_joystick` against `v0.3` and the spare-pad test drives
internal pull-ups and pull-downs onto the DAC's SPI bus and its chip select.
The bus switch is isolated at reset so it does no damage, but that is luck
rather than intent. **The check-count is the tell: a `v0.3` board reporting 27
checks was built with the wrong env.**

Section `[6]` prints the designators for whichever revision was built, and they
differ: the `~{HV_FAULT}` pull-up is `R205` on v0.1/v0.2 and **`R1905`** on
v0.3, and the `HV_SNS` bottom leg is `R207` against **`R1907`**. `R601` and
`R403` are unchanged. The full v0.2→v0.3 mapping is in
`../mouse-joystick-pcb/docs/bringup-log.md` → *Board 4*.

**Reference designators in this document are v0.1 designators, and they are not
stable.** Automatic annotation orders by sheet position, so inserting a part
rotates every designator after it. Between v0.1 and HEAD the 36 V input block
moved from `U201`/`R20x` to `U1901`/`R190x`, and `R1901`/`R1902` changed meaning
entirely — 100 Ω pyControl series resistors at v0.1, 1 kΩ `HV_EN`/`HV_DIAG_EN`
series resistors at HEAD. Identify parts by **value and net**, not by designator.

## Watching The Board With The LA5032

Worth setting up before you plug in USB, because it is the only diagnostic that
works when USB does not.

**pcb v1.0 has a different port.** InstrumentPort-Digital-50 is `J1301`, a
50-way 0.5 mm FFC, read-only and always enabled; it mates the
`la5032-instrument-port-adapter` **v2.0** (built and received, mechanical fit
checked by hand on 2026-09-28, continuity and cable permutation **not yet
measured**) through a Molex Premo-Flex 15020 same-side FFC. Designed mapping:
`CHn` observes `DIGn`, so **`FW_TRIG0` is CH0 and `FW_TRIG1` is CH1**, the
TMC bus is CH8–CH14, the pyControl UART CH2/CH3, the DAC I2C pair CH22/CH23,
the torque bus CH16–CH20, `PG_5V`/`PG_12V` CH30/CH31. A fully reversing cable
would map `CHn` to `CH31-n` instead; the boot signature on CH0/CH1 (or CH31/
CH30) is how to find out. The adapter's own bring-up log owns that result.
The rest of this section is the rev 0/1 port.

InstrumentPort-40 is `J1501`. Through the `la5032-instrument-port-adapter`
v1.0, channels 0–17 map straight onto Strands `DIG0`–`DIG15` and
`TRIG0`–`TRIG1`:

| Signal | `J1501` pin | Strand | LA5032 channel | Useful for |
|--------|------------|--------|----------------|------------|
| `FW_TRIG0` | 26 | `TRIG0` | 16 | **stage 1 proof of life** |
| `FW_TRIG1` | 28 | `TRIG1` | 17 | **stage 1 boot signature** |
| `TMC_SPI_SCK` | 5 | `DIG2` | 2 | stage 2 |
| `TMC_SPI_COPI` | 7 | `DIG3` | 3 | stage 2 |
| `TMC_SPI_CIPO` | 8 | `DIG4` | 4 | stage 2 |
| `~{TMC_CS}` | 10 | `DIG5` | 5 | stage 2 |
| `PYC_UART_RX` | 13 | `DIG7` | 7 | stage 4 |
| `PYC_UART_TX` | 14 | `DIG8` | 8 | stage 4 |
| `ENC_A` | 2 | `DIG0` | 0 | stage 3 |
| `ENC_B` | 3 | `DIG1` | 1 | stage 3 |
| `LIMIT_SW` | 19 | `DIG11` | 11 | stage 3 |

Unlisted odd pins from 1 to 31 are `GND`; connect at least one, more for a fast
or many-channel capture. All eleven taps are buffered by `SN74LVC125A` packages
running from `+3.3V_MCU`, so **they work with the 36 V input absent** — the
buffers are not on the switched domain.

The channel mapping is documented but **not bench-verified**; see
`../board-interfaces/docs/la5032-instrument-port-40-mapping.md`. Confirm it with
the boot signature, which is a known pattern on known pins.

For stage 1 only channels 16 and 17 matter, plus a ground.

## Before You Plug Anything In

**The 36 V input stays unplugged for all of stage 1.** The MCU domain is fed from
USB `VBUS` or pyControl pin 6 through two ORing diodes and never from the 36 V
chain, so USB alone runs this firmware. Applying 36 V now would gain nothing and
risk an unproven board.

Leave the pyControl RJ45 unplugged too. It is a second power source, and having
only one makes BOOTSEL recovery a one-cable operation instead of a two-cable one.

Nothing needs to be connected to the stepper, encoder, or limit switch
connectors.

## Step 0 — Does The Bootloader Come Up?

This is a hardware test, before any firmware exists. Hold the BOOTSEL tactile
switch, plug in USB, release.

A mass storage device named `RPI-RP2` should appear.

```sh
lsblk -o NAME,LABEL,SIZE | grep -i RPI-RP2
```

If you have the LA5032 on channels 16/17, note that **the bootloader drives
neither** — no edges at this stage is correct.

**If it appears, a great deal is already proven:** the 3.3 V regulator, the
RP2040 itself, the 12 MHz crystal (USB full speed needs 48 MHz within 0.25%,
derived from it), the USB connector and its series routing, and the QSPI flash
enough to be written.

**If it does not appear:**

| Symptom | Suspect |
|---------|---------|
| No USB device at all, host logs nothing | `+3.3V_MCU`. The chain is USB-C `J301` → `VBUS` → ORing Schottky `D301` → `+5V_MCU` → `U701` TPS73733 → `+3.3V_MCU`. Measure `+5V_MCU` first: it splits the diode from the regulator. |
| Host logs a device but enumeration fails | crystal, or `USB_DP`/`USB_DM` routing. |
| Enumerates but no mass storage | flash `U502`: `QSPI_SS`/`QSPI_SCLK`/`QSPI_SD0..3`. |
| Never enters BOOTSEL at all | the button circuit: `SW501` shorts `~{USB_BOOT}` to GND, `R505` 1 kΩ ties that to `QSPI_SS`, `R504` 10 kΩ pulls `QSPI_SS` up to `+3.3V_MCU`. Standard circuit, so suspect the switch solder joints. |

Remember there is no SWD header on rev 0, so there is no debugger to fall back
on. The next instrument is a meter and a scope.

## Step 1 — Flash The Self Test

With the board in BOOTSEL:

```sh
make upload PORT=/dev/ttyACM0
```

`picotool` finds the board in BOOTSEL by USB id, so `PORT` does not matter for
this first upload.

After this one, you should not need the button again: the running firmware
honours the 1200 baud touch that PlatformIO uses to reboot into the bootloader,
and the self test's `B` command reboots into BOOTSEL on demand. **That matters
more on this board than on a Pico** — with both USB and pyControl connected, the
manual BOOTSEL dance needs both cables out, because the MCU domain is ORed from
both.

## Step 2 — Proof Of Life That Does Not Need USB

The firmware emits a **boot signature** on the InstrumentPort triggers before it
brings up Serial, and then a **1 Hz heartbeat** on `FW_TRIG0` forever after:

```
TRIG1 (ch 17):  ___|‾|_|‾|_|‾|______________________   3 pulses, 10 ms half period
TRIG0 (ch 16):  ______________|||||||| _|‾‾‾‾|____|‾   8 pulses at 1 ms, then 1 Hz
```

The 10 ms cadence on `TRIG1` appears nowhere else in this firmware, so it is
unambiguous. Read the combinations:

| `TRIG1` signature | `TRIG0` heartbeat | Conclusion |
|-------------------|-------------------|------------|
| present | present | The RP2040 booted from flash and is running `loop()`. If USB is also silent, the fault is in USB or CDC, not in the MCU, the crystal, or the flash. |
| present | stops | Firmware hung or crashed after `setup()`. Note when it stopped relative to the serial output. |
| absent | absent | The RP2040 is not reaching your code. Suspect flash contents, `boot2`, or power. Retry step 0. |
| absent | present | Wiring or channel mapping is wrong — the two are 2 pins apart on `J1501`. Check the adapter map. |

This exists because the board has no LEDs, no test points, and no SWD header on
rev 0. It is the only independent witness to firmware execution.

Send `i` on the serial console to silence the port and the heartbeat with it.

## Step 3 — Read The Serial Output

```sh
make monitor PORT=/dev/ttyACM0
```

The test runs once at boot and prints a summary line. The board waits up to 5 s
for the monitor to attach; if you miss it, send `r`.

### What Each Section Means When It Fails

**`[2] reset`** — after a `picotool` upload this reports `software` with
`watchdog caused reboot: yes`. That is correct, not a fault: `picotool` reboots
the chip through the watchdog to start the application. A genuine cold start
reads `power-on`.

**`[3] clocks`** — a `clk_sys` reading far from 133000 kHz means a PLL
misconfiguration, which would be a firmware or core problem rather than a board
one. The measurement is taken *against* the crystal, so it cannot see an error in
the crystal itself. For that, send `k`: the board emits one line per second
carrying its own microsecond count, and drift against your host's timestamps is
the crystal's absolute error. 1% shows as 10 ms per second.

**`[4] qspi flash`** — the unique id comes from a real `0x4B` command over QSPI,
so a plausible answer means the part is present, selected and responding. The
aliasing probe is the interesting one: a flash smaller than 16 MiB wraps its
address space, so a read at +2 MiB would return the same bytes as offset 0 — and
offset 0 holds boot2, which is certainly not erased. `ALIASES offset 0` means the
fitted part is smaller than the BOM says, or an address line is broken.

**`[5] ram`** — a failure here is an RP2040 fault, which is rare enough to
suspect the test first.

**`[6] resistor-defined levels`** — the highest-value section for a first
article, because every check maps to one part:

| Check | Part | If it reads low |
|-------|------|-----------------|
| `GP9 ~{BUS_SW_EN}` high | `R601` 7.87 kΩ to `+3.3V_MCU` | **Stop.** The GPIO bus is not isolated at reset. Every later stage depends on this being the power-on default; with it low, an MCU pin can back-power the dead switched rail through the TMC5130A's ESD clamps. Find the resistor before going on. |
| `GP24 ~{INSTR_BUF_OE}` high | `R1601`/`R1701`/`R1801` 10 kΩ | The instrumentation buffers are enabled at reset instead of silent. Not dangerous, but the port is meant to be passive until asked. |
| `GP1 PYC_UART_RX` high | `R403` 10 kΩ | The pyControl RX line does not idle high. With the cable unplugged this is R403; with it plugged in, an idle UART TX would also hold it high, so test this unplugged. |
| `GP29 ~{HV_FAULT}` high | `R205` 10 kΩ to `+3.3V_MCU` | Either the pull-up is missing, or the 36 V switch `U201` has a latched fault. With 36 V unplugged it should be the former. This resistor was wired as a pull-*down* in an earlier draft of the design, so if it reads low, check which rail it goes to before suspecting a fault. |

These are captured in the first instruction of `setup()`, with the RP2040's
internal pulls disabled and before anything drives a pin — which has to be that
early, because the boot signature drives `~{INSTR_BUF_OE}`. At reset each pad has
its internal pull-down enabled at ~50–80 kΩ; every pull-up above is 10 kΩ or
less, so the board wins by a wide margin and these levels hold even in the window
before firmware runs.

**`[7] spare pads`** — nine GPIOs routed nowhere on any revision, pulled both
ways. A pad that will not move is shorted to a rail or a neighbour; one that
reads wrong in only one direction is an open bond or a bridge. This is an
assembly check, and it is the only one the board's lack of test points does not
block.

**`[8] adc`** — with 36 V absent:

- `SW_RAIL_SENSE_DIV_2` should read essentially zero. `R401`/`R402` hold
  `+3.3V_SW` at a true 0 V when the domain is off, and that is what makes a
  cross-domain pull-up to that rail a valid LOW. A non-zero reading here means
  the switched rail is somehow alive, which with `HV_EN` low would be a real
  finding.
- `HV_SNS` should read zero — but it reads zero regardless while `HV_DIAG_EN` is
  low, which stage 1 keeps it. So this confirms the quiet state, not the sense
  chain. The sense chain is a stage 2 measurement.
- The die temperature proves the ADC and `ADC_AVDD` work at all. On this board
  `ADC_AVDD` is tied straight to `+3.3V_MCU` with no separate reference and no
  filter, so absolute ADC accuracy is only as good as that rail.

## Step 4 — Optional, With Instruments

**`t` — 5 kHz trigger burst.** 1000 cycles at a 200 µs period on `FW_TRIG0`
(GP25) with `FW_TRIG1` (GP21) at half the rate. Faster than the heartbeat, so it
also shows the pins work at a rate a real timing marker would use. A capture
showing one trigger but not the other separates a pin fault from a buffer fault
in `U1801`.

**`u` — pyControl UART loopback.** Needs `DIO-A` shorted to `DIO-B` on the RJ45.
Without a loopback fitted, zero bytes received is the correct result, not a
fault. With one, this tests GP0, GP1, `R301`, `R302`, and the connector.

**`k` — clock tick stream.** Described above.

## Step 5 — Record It

Copy the numbers into `../mouse-joystick-pcb/docs/bringup-log.md` under this
board's serial — the flash unique id from section 1. Not pass marks: the whole
point of recording a serial is that two boards in a rig are independently
calibrated and independently suspect.

## What Stage 1 Cannot Tell You

- Anything about the TMC5130A, the switched rails, the 36 V input block, the
  motor, the encoder, or the limit switch. All of it is on the far side of the
  bus switch or the high-side switch.
- Whether `+5V_SW` and `+3.3V_SW` come up correctly. That is the first thing
  stage 2 does, and `SW_RAIL_SENSE` on GP26 is how firmware watches it.
- Whether the pyControl link works end to end. Only the loopback and the idle
  level are in reach.
