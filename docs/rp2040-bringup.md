# Stage 1: RP2040 Bring-Up

**Board:** `mouse-joystick-pcb` rev 0 (v0.2), first assembled article received
2026-09-01.
**Program:** `examples/RP2040SelfTest`
**Scope:** everything on the always-on `+3.3V_MCU` domain. Nothing else.

Record results in `../mouse-joystick-pcb/docs/bringup-log.md` under the board's
serial, as numbers with units.

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

**If it appears, a great deal is already proven:** the 3.3 V regulator, the
RP2040 itself, the 12 MHz crystal (USB full speed needs 48 MHz within 0.25%,
derived from it), the USB connector and its series routing, and the QSPI flash
enough to be written.

**If it does not appear:**

| Symptom | Suspect |
|---------|---------|
| No USB device at all, host logs nothing | `+3.3V_MCU`: the ORing diode `D301`, the USB `VBUS` path, or the 3.3 V regulator. Measure the rail. |
| Host logs a device but enumeration fails | crystal, or `USB_DP`/`USB_DM` routing. |
| Enumerates but no mass storage | flash: `QSPI_SS`/`QSPI_SCLK`/`QSPI_SD0..3`. |

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

## Step 2 — Read The Output

```sh
make monitor PORT=/dev/ttyACM0
```

The test runs once at boot and prints a summary line. The board waits up to 5 s
for the monitor to attach; if you miss it, send `r`.

### What Each Section Means When It Fails

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
| `GP9 ~{BUS_SW_EN}` high | R601 7.87 kΩ to `+3.3V_MCU` | **Stop.** The GPIO bus is not isolated at reset. Every later stage depends on this being the power-on default; with it low, an MCU pin can back-power the dead switched rail through the TMC5130A's ESD clamps. Find the resistor before going on. |
| `GP24 ~{INSTR_BUF_OE}` high | R1601/R1701/R1801 10 kΩ | The instrumentation buffers are enabled at reset instead of silent. Not dangerous, but the port is meant to be passive until asked. |
| `GP1 PYC_UART_RX` high | R403 10 kΩ | The pyControl RX line does not idle high. With the cable unplugged this is R403; with it plugged in, an idle UART TX would also hold it high, so test this unplugged. |
| `GP29 ~{HV_FAULT}` high | R1905 10 kΩ | Either the pull-up is missing, or the 36 V switch has a latched fault. With 36 V unplugged it should be the former. Note that `R205` was once wired as a pull-*down* in an earlier draft of the design; if this reads low, check which rail it goes to. |

These are read with the RP2040's internal pulls disabled, so the level comes from
the board. At reset each pad has its internal pull-down enabled at ~50–80 kΩ;
every pull-up above is 10 kΩ or less, so the board wins by a wide margin and
these levels hold even in the window before firmware runs.

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

## Step 3 — Optional, With Instruments

**`t` — pulse the InstrumentPort triggers.** Enables the instrumentation buffers
and drives `FW_TRIG0` (GP25) and `FW_TRIG1` (GP21) with `FW_TRIG1` at half the
rate. Put a scope or logic analyser on InstrumentPort-40 `TRIG0`/`TRIG1`. This is
the only way to see an RP2040 output pin on this board, and it tests the pin, the
`SN74LVC125A` buffer `U1801`, and the port wiring in one shot. A capture showing
one trigger but not the other separates a pin fault from a buffer fault.

**`u` — pyControl UART loopback.** Needs `DIO-A` shorted to `DIO-B` on the RJ45.
Without a loopback fitted, zero bytes received is the correct result, not a
fault. With one, this tests GP0, GP1, `R301`, `R302`, and the connector.

**`k` — clock tick stream.** Described above.

## Step 4 — Record It

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
