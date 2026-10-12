# Which analyzer is on which board, Rig 1 laptop, 2026-10-09 09:19

A 3 s capture on all three LA5032s with the joystick profile, run from the
laptop right after `lai monitor doctor` first read `ready` there, to tell
the joystick boards from the camera board without resetting anything
(`docs/rig-day.md`, "Rehearsal before the move", in `instrument-control`).

```sh
lai monitor capture \
    --profile fixtures/mouse-joystick-pcb-v1.0/profile.toml --analyzer topology=1-5.1 --name la1 \
    --profile fixtures/mouse-joystick-pcb-v1.0/profile.toml --analyzer topology=1-5.3 --name la3 \
    --profile fixtures/mouse-joystick-pcb-v1.0/profile.toml --analyzer topology=1-5.4 --name la4 \
    --seconds 3 --sample-rate-hz 1000000 --decode --out evidence/which
```

| Analyzer | Health | Trace | Board |
| --- | --- | --- | --- |
| `1-5.1` (la1) | `silent` | 861 bytes, 13 qspy records (BOOT_INFO, LINK_STAT, POLL_STAT, SAMPLE_STAT, TRACE_STAT) | joystick, later identified as unit A `E4654C159F274534` |
| `1-5.3` (la3) | `dead` | 0 bytes | camera board |
| `1-5.4` (la4) | `silent` | trace alive | joystick, later identified as unit B `E4654C159F404534` |

The serials come from the identity acks in the mock experiment's link logs,
`../2026-10-09-0920-rig1-mock-experiment/README.md`.
