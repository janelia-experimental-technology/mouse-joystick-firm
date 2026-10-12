# Rig 1 observer, mock experiment, 2026-10-09

The first run of the observer on the laptop that carries the three LA5032s,
following `.agents/prompts/rig1-observer.md` and `docs/rig-day.md`. The
desktop ran the rehearsal's two-task mock experiment (two `joystick_trial`
trials on the left board, then two on the right) through the pyControl
pyboard; the laptop only listened. Everything read as the bench rehearsal
of the same morning predicted. This directory is the live run's evidence
as `lai monitor live` wrote it (`health/`, `decode/`, `link/`, `trace/`,
`markers/`, `captures/`); the 3 s identification capture that preceded it
is `../2026-10-09-0919-rig1-which-analyzer/`.

## Laptop setup, what it actually took

- The Guix profile already had `sigrok-cli` 0.7.2 with `kingst-la2016`,
  Python 3.12, `pixi` and `guix`; no `pixi install` was needed here.
- Firmware: copied `../instrument-nonfree/artifacts/kingst-la2016/*` into
  `~/.local/share/sigrok-firmware` (19 files; the doctor counts the 4 the
  LA5032 uses).
- udev: the device nodes were `crw-rw-r-- root root`; the operator
  installed `99-Kingst.rules`, reloaded, re-plugged, and they became 0666.
- qspy: `pixi run qtools-install` in `../qp-arduino` refused to solve from
  this Guix shell ("Virtual package `__glibc` does not match"); it built
  with `CONDA_OVERRIDE_GLIBC=2.41 pixi run qtools-install` (host glibc).
  `lai` then found `.tools/quantum-leaps/qtools-7.3.4/bin/qspy` on its own.
- `PYTHONPATH=src python3 -m pytest -q`: 178 passed, 3 skipped (54 s).
- Doctor: `ready`, every line `ok`, analyzers `1-5.1`, `1-5.3`, `1-5.4`
  (a GenesysLogic hub on bus 1). The paths differ from the bench's
  `1-5.4.x`, as the rig-day doc warns.

## Which analyzer is on which board

3 s capture on all three with the joystick profile
(`../2026-10-09-0919-rig1-which-analyzer/`):

| Topology | Reading | Board | Serial (from the `ok` window's identity ack) |
| --- | --- | --- | --- |
| `1-5.1` | `silent`, trace alive | joystick | `E4654C159F274534`, unit A, right, pyControl port 4 |
| `1-5.3` | `dead` | camera board | — |
| `1-5.4` | `silent`, trace alive | joystick | `E4654C159F404534`, unit B, left, pyControl port 1 |

## Live watch

```sh
lai monitor live \
    --profile fixtures/mouse-joystick-pcb-v1.0/profile.toml --analyzer topology=1-5.1 --name joy1 \
    --profile fixtures/mouse-joystick-pcb-v1.0/profile.toml --analyzer topology=1-5.4 --name joy4 \
    --profile fixtures/camera-gpio-interface-pcb-v1.0/profile.toml --analyzer topology=1-5.3 --name cameras \
    --window 15 --sample-rate-hz 2000000 --trigger-on marker0 --pre-trigger 20 --arm-timeout 90 \
    --out evidence/2026-10-09-rig1   # moved here afterwards
```

Armed 09:20:37, stopped with an interrupt 09:26:01; the desktop started
the tasks at about 09:23:20. Status lines, in order:

| Wall time | Board | Status | What it said |
| --- | --- | --- | --- |
| 09:21:16 – 09:23:23 | cameras | `silent` ×9 | no pulses on any role, gaps 0.8–1.0 s |
| 09:22:42 | joy1, joy4 | `idle` | no marker0 edge in 125 s |
| 09:23:38 | cameras | `ok` | trig_in 1096 @ 150.0 Hz, sync 8/8 on SYNC_OUT and every L0, every L3 1097/1097 within 0.5 µs |
| 09:23:40 | joy4 (left, B) | `ok` | 533 lines, 53.2/s; heartbeat 24.358 Hz; trials 2, NOMOVE 2; naks 0, faults 0, framing 0; 894 trace records, 0 errors; 157 marker0 pulses |
| 09:23:54 | cameras | `degraded` | trig_in 1496 @ 99.8 Hz, "expected 140–160"; sync 9/9, every L3 1497/1497 |
| 09:23:56 | joy1 (right, A) | `ok` | 537 lines, 53.1/s; heartbeat 24.324 Hz; trials 2, NOMOVE 2; naks 0, faults 0, framing 0; 899 trace records, 0 errors; 157 marker0 pulses |
| 09:24:10 | cameras | `ok` | trig_in 142 @ 150.0 Hz clipped at the window start, then quiet |
| 09:24:26 – 09:25:46 | cameras | `silent` ×6 | idle again |
| 09:25:46 | joy4 | `idle` | re-armed, nothing ran |

Both joystick `ok` windows carry the same event set as the rehearsal:
`HOMED` 1, `TARGET` 4, `RELEASED` 2, `NOMOVE` 2, `TRIAL` 2, last event
`TARGET angle=-0.091` (parked). Active link span about 10 s of each 15 s
window. Exposure strobes (`L2`, `OUT`) and `L1` quiet throughout: no camera
was acquiring, which the profile allows.

### The one `degraded` line is the pause between tasks, not the board

Window `cameras-0012` (09:23:39–09:23:54) holds 1496 trigger pulses, which
is 10 s worth at 150 Hz spread over the whole 15 s; its high fraction is
0.333 where a continuous 150 Hz / 3.33 ms train gives 0.5. The left task
had ended and the right had not yet started, so the frame trigger paused
for about 5 s mid-window and the count-over-span rate fell to 99.8 Hz.
Every follower still matched edge for edge (L3 1497/1497, L0 and SYNC_OUT
9/9). The next window shows the tail of the right task's train (142 pulses
clipped at the start) and judges `ok`.

For the health model: a rate from count over span flags any pause between
tasks on the camera board as `degraded`. A rate from the pulse periods
(median period, or the rate within the active span), with the pause
reported separately, would judge this window `ok` and still catch a
trigger that is genuinely slow. Noted for `pulses.py`; not changed today.

## Totals (`health/live-summary.json`)

| Board | Windows | Statuses | Trials | Faults / naks / framing | Captured | In gaps | Armed idle | Trace records |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| joy1 (`1-5.1`, A, right) | 2 | idle 1, ok 1 | 2 (NOMOVE 2) | 0 / 0 / 0 | 14.8 s | 0.002 s | 125.0 s | 899 |
| joy4 (`1-5.4`, B, left) | 3 | idle 2, ok 1 | 2 (NOMOVE 2) | 0 / 0 / 0 | 14.6 s | 0.003 s | 252.4 s | 894 |
| cameras (`1-5.3`) | 19 | silent 16, ok 2, degraded 1 | — | 0 | 285.0 s | 15.9 s | — | — |

`unseen_lines` 0 and `reboots` 0 on both joystick boards (the LINK_STAT
counters saw nothing pass in a gap). The camera board's gaps are blind by
design (no counters); 0.8–1.0 s per 15 s window at 2 MSa/s × 18 channels.

## For the desktop

- The laptop is set up and the observer works end to end; the next real
  session needs only the doctor (for the paths) and the `live` command
  above with the paths it prints.
- Analyzer-to-board map on this laptop today: `1-5.1` → unit A (right),
  `1-5.4` → unit B (left), `1-5.3` → cameras. It follows the hub ports, so
  re-check after any re-plug.
