# Scripts that drove this session

Standard-library Python only (no pyserial), written for a shell that had no
`mpremote`, `curl` or `lsusb`. Each takes the pyboard's tty as its first
argument. They talk to the pyboard's REPL, so never run one while
pyControl's runner is connected: the pyboard wedged once that way in this
session and needed its reset button.

| Script | What it does |
| --- | --- |
| `pyb_version.py` | prints the MicroPython version and the top-level `/flash` listing |
| `pyb_backup.py <tty> <dir>` | copies every file under `/flash` off the pyboard through the raw REPL (base64 in 768-byte chunks) and writes `manifest.json` |
| `pyb_uart_probe.py` | sends `id;1` to the joystick board on Breakout port 1 (`UART(4)`) and port 4 (`UART(3)`) and prints the replies: which board is on which port, and its firmware |
| `pyb_uart_faults.py` | the same for port 4 only, sending `faults`, `hv state` and `status`: how the link-loss case left board 9 |

The firmware was built and flashed with `pixi run build` / `pixi run
upload` under `CONDA_OVERRIDE_GLIBC=2.41` (pixi cannot see the host glibc
from inside the Guix shell), `picotool load -x --bus 1 --address N` from
`~/.platformio/packages/tool-picotool-rp2040-earlephilhower/` for a board in
BOOTSEL, and `dfu-util` from `tool-dfuutil-arduino` for the pyboard
(`-a 0 -s :leave -D PYBV11-20250911-v1.26.1.dfu`; without `:leave` the ST
bootloader does not jump). The task runs used `joystick-rig`'s
`pycontrol/tools/run_task_headless.py` under this repository's `pycontrol`
pixi environment.
