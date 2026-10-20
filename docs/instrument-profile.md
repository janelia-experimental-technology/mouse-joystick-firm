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
signals. The current PCB export explicitly says manufactured comparison pending;
a passing compile does not upgrade that evidence.

The negative checks run with
`pixi run python -m unittest discover -s test/tools -p 'test_*.py'`.
Ordinary firmware builds/tests remain independent of sibling checkouts.
