# Plans

## Current Goal

Stage 1: prove the RP2040 subsystem of the first assembled `mouse-joystick-pcb`
rev 0 (v0.2) board, received 2026-09-01.

## Status

Repository scaffolded 2026-09-01. `examples/RP2040SelfTest` is written but has
**not** been compiled or run against hardware yet.

## Milestones

- [x] Pixi + PlatformIO environment, mirroring the `TMC51X0` layout
- [x] `src/BoardPins.hpp` derived from the board's pad-to-net map
- [x] `SafeState` — safe idle without energising anything
- [x] `examples/RP2040SelfTest` — clocks, flash, RAM, pads, pull resistors, ADC
- [ ] Compile clean
- [ ] Runs on the rev 0 board, results recorded in the bring-up log
- [ ] Stage 2: `examples/TMC5130Communication` — 36 V on, bus switch enabled,
      driver still disabled, register read proves SPI
- [ ] Stage 3: motion, with `vsense = 1` set and read back first
- [ ] Stage 4: pyControl UART protocol

## Assumptions To Confirm On Hardware

- The `generic` PlatformIO board definition boots this board. The safe
  `boot2_generic_03h` stage 2 is chosen precisely so this is very likely; if it
  fails, the flash or its QSPI routing is the suspect, not the board definition.
- Flash is really 16 MiB. The self test probes for address aliasing at +2, +4 and
  +8 MiB rather than trusting the BOM.
- The 12 MHz crystal is within USB tolerance. Enumeration proves this; the `k`
  command measures it against the host clock for a number.
- `~{INSTR_BUF_OE}` idles high from R1601/R1701/R1801. Confirmed in the netlist,
  unconfirmed on copper.

## Known Documentation Conflict

`../mouse-joystick-pcb/docs/power.md` says `HV_SNS` and `~{HV_FAULT}` "sit
behind" the GPIO isolation bus switch and cannot be read until firmware enables
it. The GPIO table in `docs/interface-map.md` marks both `byp`, and the board's
pad-to-net map agrees: `GP28`/`GP29` reach `U1901` directly, with no
underscore-suffixed twin net. `power.md` is stale on this point. This firmware
follows the netlist. Worth fixing in the PCB repository.

## Not A Priority

- OTA, filesystem, or EEPROM use. `board_build.filesystem_size` is 0.
- A second board definition. Rev 0 and rev 1 share the pin map.
- Anything that energises the 36 V rail, until stage 1 passes.
