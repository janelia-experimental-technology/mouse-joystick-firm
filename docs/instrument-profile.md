# Firmware observation profile and pin check

`instrument/mouse-joystick-v1.0.toml` is the firmware-owned composed profile
for `examples/PyControlLinkQp`, `mouse_joystick_qp_rev2`. It carries the
PCB's Digital-50 map, the measured straight v2 adapter mapping and firmware
semantics. The immutable historical capture profile remains in
`instrument-control/fixtures/mouse-joystick-pcb-v1.0/`.

Select `runtime` for operation markers, QS and the native UART. Select `boot`
for the existing identity beacon. These phases deliberately share FW_TRIG0/1;
the runtime trace pin is not a second independent marker. Decode a capture
under a known phase; do not infer the transition from UART framing errors.
The delivery build has no dedicated SYNC output, and this change adds none.

From an instrument-control environment:

```sh
lai profile check /path/mouse-joystick-firm/instrument/mouse-joystick-v1.0.toml
lai profile show /path/mouse-joystick-firm/instrument/mouse-joystick-v1.0.toml --phase boot
lai monitor decode /path/runtime.sr \
  --profile /path/mouse-joystick-firm/instrument/mouse-joystick-v1.0.toml \
  --phase runtime --out /tmp/joystick-observation
```

Every firmware build now requires explicit `MJ_BOARD_REV`. PlatformIO's
revision-specific environments already set it; native tests explicitly use
revision 0. A custom build omitting it fails instead of silently selecting
the oldest board.

`instrument/board-pin-bindings.json` maps C++ names to PCB signal names without
duplicating GPIO numbers. Check against a deliberately selected hardware export:

```sh
pixi run python tools/check_board_pins.py --facts /path/gpio-facts.json
```

Set `CXX` or pass `--cxx` if the host C++ compiler is not on PATH. The check
requires only Python and a C++17 compiler, no Arduino headers or attached board.
It compares every bound constant and spare, revision and declared active-low
signals. The original GPIO export retains its original scope label. The PCB
owner subsequently added the manufactured logical pad comparison and complete
observation-port map in `mouse-joystick-pcb/kicad/observation/`; those are separate
evidence, not a qualification inferred from a passing compile.

The negative checks run with
`pixi run python -m unittest discover -s test/tools -p 'test_*.py'`.
Ordinary firmware builds/tests remain independent of sibling checkouts.

## Peripheral allocation

The path-only [`readiness-replay.json`](../instrument/readiness-replay.json)
recipe joins the pin compile, GPIO/port export checks, peripheral allocation
and both monitor profiles through `kicad-libs/scripts/readiness_replay.py`.
Follow `kicad-libs/docs/readiness-replay.md` for the single command and explicit
KiCad export/framework inputs. It writes a new report directory and fails if
any check cannot run; it adds no dependency to ordinary firmware builds and
does not upgrade the selected exports to electrical or deployment evidence.

[`peripheral-allocation.json`](../instrument/peripheral-allocation.json) owns
the reviewed rev2 pin-function budget; the generated
[`peripheral-check.json`](../instrument/peripheral-check.json) checks all 28
assigned pins against the PCB export and the RP2040 capability model.

| Allocation | Engine or function |
| --- | --- |
| Native control TX/RX | UART0 |
| Motor SCK/COPI/CIPO | SPI0; CS remains SIO |
| Torque SCK/COPI/CIPO | SPI1; CONVST/CS remains SIO |
| Angle and brake DACs | One shared I2C0 bus |
| Rail, current and voltage monitors | ADC0–3; angle readback is reserved/test access |
| Runtime QS | One PIO SM and six instructions in the reviewed Arduino core |
| Control/status/operation marker | SIO |
| Boot beacon | SIO on the two marker pins before runtime QS ownership |
| SDK default alarm pool | Hardware alarm 3 and TIMER_IRQ_3; retained from startup |
| Arduino USB | USBCTRL_IRQ, one software IRQ (26–31); uses SDK pool software callbacks |
| Arduino core | SysTick cycle counter and SIO_IRQ_PROC0 FIFO handler |
| QP tick | One dynamically claimed hardware alarm and its IRQ, separate from SDK pool |
| Application watchdog | Enabled before the beacon and retained in runtime |
| Native-control receive | UART0_IRQ |

This is a declared allocation, not a claim that every interface is active
during startup or every analog monitor is sampled by the delivery loop.
The installed framework's sources determine the Serial1/SPI/SPI1/Wire mappings
and the six-instruction SerialPIO TX cost. Their hashes and the reviewed
firmware source hashes are retained; drift fails the check. Review new source
files, changed build options and changed dependencies before refreshing hashes.

Run the pin compile above, then follow
`kicad-libs/docs/peripheral-allocation.md` for the shared checker. Supply the
framework directory explicitly with `--implementation-root`. Do not hand-edit
the generated report. The checker permits phase sharing, rejects simultaneous
pin/engine conflicts, and checks per-block PIO capacity. The selected core-0
resource review adds exclusive IRQ/DMA/hardware-alarm checks, including each
alarm's associated IRQ. SDK reservations span `sdk_startup`, `boot` and
`runtime`; dynamic claims keep a consistent assignment across their phases.
Suggested PIO banks, alarm numbers and software IRQs are feasible assignments,
not observations of the runtime allocator.

No DMA channels are declared for the reviewed synchronous buses/ADC, UART RX
or transmit-only QS paths. `resources: []` is explicit reviewed absence in
this subset. New asynchronous transfers, RX trace, core-1 work or a different
USB stack require a fresh review. USB and delay/sleep share software slots in
the SDK alarm pool; this check does not count those slots or establish interrupt
latency. The generic variant's LED and UART defaults are not initialized by
the reviewed startup path. The probe firmware's startup allocations belong to
that separate build and must not be copied into this DUT budget.

`resource_review` records the exact selected example/environment, assumed
defines and exclusions. Framework startup/SDK source files and the installed
RP2040 `libpico.a` are now hash-guarded; disassembly confirmed the default
alarm-3 reservation. Source hashes do not evaluate C++ or prove effective
compiler flags: new sources, dependencies and build overrides still need
review. Throughput, reset/off-state behavior and deployment qualification
remain separate evidence.
