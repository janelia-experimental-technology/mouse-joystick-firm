# Development

Developer-facing notes for this repository: the repositories it was developed
beside, the instrument tooling used to watch the board, and the observability
workstream. Building, flashing and testing the firmware needs only what the
README's Quick Start installs; nothing here is required for that.

## Sibling Repositories

The Makefile, the pixi tasks and the agent guidance assume these as sibling
checkouts (`../mouse-joystick-pcb`, `../instrument-control`, and so on).
Documentation in this repository cites files in them as `<repository>/<path>`.

| Repository | Role | Where |
| --- | --- | --- |
| `mouse-joystick-pcb` | The board this firmware runs on; owns every electrical fact, the UART protocol and the bring-up log | https://github.com/peterpolidoro/mouse-joystick-pcb |
| `camera-gpio-interface-pcb` | The rig's camera interface board | https://github.com/peterpolidoro/camera-gpio-interface-pcb |
| `joystick-rig` | The rig's pyControl tasks, hardware definition and operating notes | https://github.com/sourishmukh/joystick-rig |
| `TMC51X0` | The TMC5130A driver library | https://github.com/janelia-arduino/TMC51X0 |
| `syringe-pump` | pyControl module reference design; the device class is shaped after its `PumpController` | https://github.com/peterpolidoro/syringe-pump |
| pyControl | The host framework; `PYCONTROL_DIR` points at a checkout | https://github.com/pyControl/code |
| `instrument-control` | Logic-analyzer observation of the boards, board identification, `docs/rig-day.md` | https://codeberg.org/orb-weaver/instrument-control |
| `board-interfaces` | Interface contracts: the instrument port, the pyControl behaviour port, the debug/trace header | https://codeberg.org/orb-weaver/board-interfaces |
| `orb-weaver-context` | Shared architecture and planning documents, including the rig system plan | https://codeberg.org/orb-weaver/orb-weaver-context |
| `la5032-instrument-port-adapter` | The LA5032 adapter for the instrument port | https://codeberg.org/orb-weaver/la5032-instrument-port-adapter |
| `qp-arduino-example` | The QP/C++ port plan and pilot | https://codeberg.org/orb-weaver/qp-arduino-example |

This firmware was developed inside the Orb Weaver ecosystem
(https://codeberg.org/orb-weaver), which supplies the instrument tooling and
the planning documents above. The firmware itself depends on none of it.

## Evidence

`evidence/` is a curated copy of the v1.0 verification runs from
`qp-arduino-example/evidence/`, where the bench sessions were driven and
where the raw material still lives in full. The copy keeps every text file
and the logic-analyzer captures up to 1 MB plus the four first-board captures
the bring-up log cites by name; each directory's `OMITTED.sha256` lists the
raw `.sr` and `.vcd` files left behind with their sizes and hashes (about
835 MB). The earlier QP pilot and board 4 (pcb v0.3) runs of 2026-09-30 to
2026-10-02 are not copied; the QP example's README cites them where they
live.

## Observability Development

The [observability plan](observability-plan.md) tracks `OBS-FW`: define
application event semantics, establish a non-QP evidence baseline, and evaluate
an optional QP/QV trace pilot. It links to the shared architecture and parallel
workstreams. Current delivery remains non-QP; this work does not require a
firmware rewrite or delay existing board bring-up and delivery.

The [QP port plan](https://codeberg.org/orb-weaver/qp-arduino-example/src/branch/main/docs/observability-plan.md) now targets RP2040
first with the same Earle Philhower Arduino core family, on a separate simple
non-actuating fixture. Exact core/fixture versions and qualification gates are
open. Teensy hardware work is deferred and does not block this path;
STM32G0B1CEU6 is the later second target. Port qualification remains separate
from a decision to adopt QP in this application.

The hardware direction keeps passive Digital-50 plus a separate optional
debug/trace connector and reusable external RP2040 probe. New connections and
trace throughput need their own board/probe evidence; they are not present
capabilities or new pin allocations. Combined 60-pin wiring and an onboard
second MCU are deferred.

## Watching Stage 2 With The LA5032

`FW_TRIG1` goes high for the whole register-read burst. On a v1.0 board through
the adapter v2.0 (`CHn` observes `DIGn`):

```sh
cd ../instrument-control
guix shell -m manifest.scm -- sigrok-cli --driver kingst-la2016 \
  --channels CH8,CH9,CH10,CH11,CH1 --config samplerate=10m \
  --triggers CH1=r --samples 2000000 \
  -P spi:clk=CH8:mosi=CH10:miso=CH11:cs=CH9:cpol=1:cpha=1 -A spi
```

`CH8`–`CH11` are `SCK`, `~CS`, `COPI`, `CIPO` and `CH1` is `FW_TRIG1`. On a
rev 0/1 board through the adapter v1.0 the bus is `CH2`–`CH5` (`SCK`, `COPI`,
`CIPO`, `~CS`) and the trigger is `CH17`. The TMC5130A is SPI mode 3, hence
`cpol=1:cpha=1`. `docs/rp2040-bringup.md` has both channel maps in full.

## Agent Guidance

`AGENTS.md` and `.agents/` carry the agent-facing guidance, plans and handoffs
for this repository. They assume the sibling checkouts above.
