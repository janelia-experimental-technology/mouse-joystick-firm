# Plans

## Current Goal

Stage 1: prove the RP2040 subsystem of the first assembled `mouse-joystick-pcb`
rev 0 board, received 2026-09-01. USB only; 36 V stays unplugged.

## Status

**Stage 1 passed on hardware, 2026-09-01.** Board serial `E4654C159F2C2534`.
27 checks, 0 failed. Flash written and verified by `picotool`. Not yet recorded
in the PCB repository's bring-up log, and the LA5032 taps are still unconfirmed.

Stage 2 is unblocked, and needs 36 V.

## Milestones

- [x] Pixi + PlatformIO environment, mirroring the `TMC51X0` layout
- [x] `src/BoardPins.hpp` derived from `U501`'s pad-to-net map at PCB tag `v0.1`
- [x] Confirmed PCB tags `v0.1` and `v0.2` have byte-identical RP2040 pad maps
- [x] `SafeState` — safe idle without energising anything
- [x] `examples/RP2040SelfTest` — clocks, flash, RAM, pads, pull resistors, ADC
- [x] Boot signature and heartbeat on `FW_TRIG0`/`FW_TRIG1`, so firmware
      execution is observable without USB
- [x] `MJ_BOARD_REV` so the spare-pad test matches the revision being built
- [x] Compile clean on `mouse_joystick`, `mouse_joystick_fastflash`,
      `mouse_joystick_rev1`; 7/7 host tests pass
- [x] BOOTSEL enumerates (`RPI-RP2` mass storage) — passed 2026-09-01
- [x] Firmware flashes, verifies, and runs — **27 checks, 0 failed**, 2026-09-01
- [ ] Results recorded in `../mouse-joystick-pcb/docs/bringup-log.md`
- [ ] LA5032 channel map confirmed against the boot signature
- [ ] Stage 2: `examples/TMC5130Communication` — 36 V on, `HV_EN` raised, bus
      switch enabled, driver still disabled, register read proves SPI
- [ ] Stage 3: motion, with `vsense = 1` set and read back first
- [ ] Stage 4: pyControl UART protocol

## Revision Facts Established 2026-09-01

Read from `git show <tag>:kicad/mouse-joystick-pcb.kicad_pcb`, footprint `U501`:

- **`v0.1` and `v0.2` RP2040 pad-to-net maps are byte-identical.** The ambiguity
  over which was manufactured does not affect firmware.
- **HEAD differs on three pins only:** `GP10`, `GP11`, `GP13`. HEAD routes them to
  the rev 1 `DAC80501`; `v0.1` leaves them unrouted. Handled by `MJ_BOARD_REV`.
- **Designators rotated between `v0.1` and HEAD.** The 36 V input block went from
  `U201`/`R20x` to `U1901`/`R190x`. `R1901`/`R1902` are 100 ohm pyControl series
  resistors at `v0.1` and 1 kohm `HV_EN`/`HV_DIAG_EN` series resistors at HEAD.
  All designators in this repository are `v0.1` designators.
- **`v0.1` has three `SN74LVC125A` buffers** (`U1601`, `U1701`, `U1801`), all on
  `+3.3V_MCU`. HEAD has a fourth, `U2101`, for the DAC bus.
- **Every instrumentation buffer input is resistor-held**, including the encoder
  taps (`R805`/`R806` 100 kohm to GND), so enabling the port floats nothing.
- **USB is USB-C** (`J301`, `CC1`/`CC2` via 5.1 kohm), with `U301` USBLC6-2P6 on
  the data pair. MCU power chain: `VBUS` -> `D301` Schottky -> `+5V_MCU` ->
  `U701` TPS73733 -> `+3.3V_MCU`. pyControl's 5 V ORs in through `D1901`.
- **BOOTSEL is the standard circuit:** `SW501` shorts `~{USB_BOOT}` to GND,
  `R505` 1 kohm to `QSPI_SS`, `R504` 10 kohm pull-up to `+3.3V_MCU`.
- **Flash is `U502` W25Q128JVZPIQ**, 16 MiB, matching the build configuration.
- **The TMC5130A's clock is an external 16 MHz oscillator `X1201`, powered from
  `+3.3V_SW`.** It is dead until `HV_EN` is raised — a stage 2 consideration.

## Confirmed On Hardware 2026-09-01

Board serial `E4654C159F2C2534`, 36 V absent, USB only.

- **The `generic` board definition boots this board** with
  `boot2_generic_03h_4`. `mouse_joystick_fastflash` is untried on hardware.
- **Flash is really 16 MiB.** No aliasing at +2, +4 or +8 MiB, and `picotool`
  verified the whole image byte for byte.
- **Clocks are nominal:** `clk_ref` 12000 kHz, `clk_sys` 133000 kHz measured,
  `clk_peri`/`clk_usb`/`clk_adc` 48000 kHz.
- **Crystal `Y501` absolute error is below this method's noise floor** — −18.7 ppm
  apparent over 9 s against host `time.monotonic()`, where USB CDC jitter alone
  is order 110 ppm. Consistent with a healthy +/-30 ppm part.
- **All four resistor-defined levels are correct:** `R601` 7.87k holds the GPIO
  bus isolated at reset, the buffer OE pull-ups hold the instrument port silent,
  `R403` holds pyControl RX idle high, `R205` is present with no latched fault.
- **All 12 spare pads move both ways.** No solder defects on any unrouted GPIO.
- **`+3.3V_SW` reads 0.034 V with `HV_EN` low** — the switched domain is genuinely
  off and `R401`/`R402` hold it there. `HV_SNS` 0.016 V. Die 26.2 C.
- **The 1200-baud touch works**, so reflashing needs no button and no physical
  access to the board.

## Still Unconfirmed

- **The LA5032 channel mapping.** Documented but not bench-verified. The boot
  signature is a known pattern on known pins, so it doubles as the confirmation
  once the adapter is connected. Nothing in stage 1 needed it, because USB came
  up on the first try.
- `env:mouse_joystick_fastflash` on hardware.
- Everything on the far side of `HV_EN`: the switched rails, the TMC5130A, the
  16 MHz oscillator `X1201`, the motor, encoder, and limit switch.

## Known Documentation Conflict

`../mouse-joystick-pcb/docs/power.md` says `HV_SNS` and `~{HV_FAULT}` "sit
behind" the GPIO isolation bus switch and cannot be read until firmware enables
it. The GPIO table in `docs/interface-map.md` marks both `byp`, and the board's
pad-to-net map agrees at both `v0.1` and HEAD: `GP28`/`GP29` reach the 36 V
switch directly, with no underscore-suffixed twin net. `power.md` is stale on
this point. This firmware follows the netlist. Worth fixing in the PCB
repository.

## Build-System Trap, Fixed 2026-09-01

`lib_deps = src/MouseJoystickBoard` — a bare local path — made PlatformIO copy
the directory into `.pio/libdeps/` and never refresh it, so edits to the
library's `.cpp` files were silently not compiled and builds reported success
while testing stale code. Several "passing" builds during initial development
were affected. Fixed by declaring `symlink://src/MouseJoystickBoard` and adding
`src/MouseJoystickBoard/library.json`, which `symlink://` requires. Verified with
a `#error` canary.

**`TMC51X0` uses the same bare-path arrangement** (`lib_deps_local = src/TMC51X0`)
and so has the same trap. Worth fixing there.

## Not A Priority

- OTA, filesystem, or EEPROM use. `board_build.filesystem_size` is 0.
- Anything that energises the 36 V rail, until stage 1 passes.
