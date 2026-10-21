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
pin/engine conflicts, and checks per-block PIO capacity. Its suggested PIO bank
is not the runtime allocator's assignment. IRQ/DMA/timer and other SDK users,
throughput and deployment qualification remain separate checks.
