# Bench scripts from the 2026-09-30 RP2040 runs

The shell scripts that drove the fixture, the LA5032 and QSPY for the
evidence in the sibling directories, kept verbatim as the record of the
procedure. They were written for one agent session and hardcode that
session's scratch directory (`S=/tmp/claude-1001/.../scratchpad`), the
`qp-arduino` checkout (`R=`), the `instrument-control` Guix environment for
`sigrok-cli` (`G()`), and the fixture's `/dev/serial/by-id` node. To reuse
one, change those variables at the top; the `pixi run` tasks in `qp-arduino`
(`validate-rp2040`, `stress-rp2040`, `upload-*`) cover the target side.

| Script | Purpose |
|---|---|
| `soak.sh`, `soak_la.sh` | DPP two-hour soak with QSPY and the 100 kSa/s analyzer capture |
| `post_soak*.sh`, `run_post_soak.sh` | host-driven port-validation procedures and role captures after the soak |
| `stress.sh`, `stress_la.sh`, `stress_la2.sh`, `after_stress.sh` | reader-cycling stress runs 1 and 2 with their analyzer captures |
| `verify_fix.sh` | stress run 3 against the race-fixed firmware with analyzer capture 3, then `measure_throttle.sh` |
| `measure_throttle.sh` | Blinky throttle and transmit-only captures (images F to I) |
| `trig1_run.sh` | Blinky transmit-only trace on the PIO UART at GP21: 30 s and 600 s captures, decode, restore |
| `link_run.sh` | joystick-link pilot: flash a profile, capture 40 s, drive `tools/link_pilot.py`, decode; observation set then full set; restore Blinky (`link_run2/3/4` were this script with one profile and the log name changed) |
| `link_run_firm.sh`, `link_run_firm2.sh` | the firmware's `examples/PyControlLinkQp` on the fixture: bench build under the pilot (the first script's pass A capture never triggered because the probe pad was not yet an output; the second reruns it without a trigger), then the delivery build's boot sequence, then restore |
| `link_run_ring.sh` | prepared 2026-10-01, not yet run: the firmware's `mouse_joystick_qp_sim_usb` with protocol v1 and the capture ring under the pilot's full sequence (moves, brake, watch, simulator stimulus, fetch, stream, host loss), 90 s capture of the tick probe, marker and trace at 1 MSa/s, decode and analysis with `--tick-us 1000`, output under `evidence/<date>-rp2040-joystick-ring-capture/` with `meta.txt` (firmware version, board serial); restores Blinky |
| `board4_session.sh`, `board4-session.md` | prepared 2026-10-02, not yet run: the board 4 (E4654C159F171E34, v0.3) rail-up session with stop points, driving `tools/board4_steps.py` (identity, the angle-output meter procedure, hv off) and `tools/link_pilot.py --stimulus hand` (real motion, a hand pull, FETCH with `lost=`, the 90 s budget run); the procedure document says what each phase expects and where the numbers go |
| `unitA_brake_torque.sh`, `unitA_brake_steps.sh` | 2026-10-06, run: `mouse-joystick-pcb` v1.0 unit A (E4654C159F274534) stages A4/A5 through the firmware's `examples/BrakeAndTorque` over USB CDC (`tools/board_console.py` in the firmware repo), with the LA5032 on the Digital-50 taps: reboot capture for the bus-switch glitch, bring-up, brake sweep and step responses (the second script: steps at several amplitudes with the fault line sampled), torque statistics, shunt-cal, live loop, coupling, power down; evidence `2026-10-06-0848-unitA-brake-torque/` and `-0853-unitA-brake-steps/` |
| `tools/brake_torque_steps.py` (not a shell script) | 2026-10-06, run: the brake and torque verbs of protocol v1 on the `mouse_joystick_qp_rev2_usb` build — caps, tare, `brake` → `RELEASED` timing at two levels, a hand-less `watch`/`TRIAL`/`fetch` with torque, hv off — with a reader thread so its heartbeat cannot starve (the first version did, and the board faulted `host_timeout` as it should); evidence `2026-10-06-09xx-unitA-qp-brake-torque/` |
| `post_verify.sh` | analysis of run 3, capture 3 and the throttle captures into the run directories (then under `qp-arduino/docs/evidence`, moved here 2026-10-01) |

Long runs were started detached (`setsid nohup script.sh & disown`) and
watched through their log files; kill only by exact process name.
