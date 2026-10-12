#!/usr/bin/env python3
"""Run a task on the pyControl pyboard headlessly, through pyControl's own
board API -- the same path the GUI uses, with no GUI.

    pixi run -e pycontrol python tools/pycontrol_run.py --port /dev/ttyACM1 \
        --hwd mouse_joystick_bringup --task mouse_joystick_check --seconds 60 \
        [--set board=right --set do_brake=1] [--load-framework] [--out run.json]
        [--hwd-path <file.py> --task-dir <dir>]   # e.g. the rig's own files

Gate E (`mouse-joystick-pcb/docs/validation-plan.md`) asks for an unforked
upstream pyControl loading this repository's device class through its normal
supported path: `Pycboard.load_hardware_definition()` transfers the hardware
definition and the device files it names (our `mouse_joystick.py` and
`mouse_joystick_protocol.py`, symlinked into pyControl's `devices/` by
`pycontrol-link`), `setup_state_machine()` transfers the task, and
`start_framework()` runs it while `process_data()` streams the events,
states, prints and variables back. Every message is printed with its board
time and written to the JSON file.

pyControl must run with its own root as the working directory
(`settings.py` derives every folder from `os.getcwd()`), so this script
changes directory to the checkout before importing it. Set PYCONTROL_DIR or
pass --pycontrol-dir; the default is ~/Repositories/github/pycontrol-code.
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", required=True)
    ap.add_argument("--pycontrol-dir", default=os.environ.get("PYCONTROL_DIR", "~/Repositories/github/pycontrol-code"))
    ap.add_argument("--hwd", default="mouse_joystick_bringup", help="hardware definition name (pycontrol/hardware_definitions, uploaded from dist/)")
    ap.add_argument("--task", default="mouse_joystick_check", help="task name (pycontrol/tasks, uploaded from dist/)")
    ap.add_argument("--hwd-path", default=None,
                    help="a hardware definition file to upload instead of dist/'s (e.g. the rig's own, from its checkout)")
    ap.add_argument("--task-dir", default=None,
                    help="the directory holding <task>.py, instead of dist/tasks (e.g. the rig's tasks/rig1)")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--set", action="append", default=[], metavar="VAR=VALUE",
                    help="set a task variable before the run (Python literal or string)")
    ap.add_argument("--load-framework", action="store_true", help="reload the pyControl framework onto the pyboard first")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    repo = Path(__file__).resolve().parents[1]
    # The pyboard gets the lean copies (tools/pycontrol_lean.py): regenerate
    # them from the sources so an edit cannot be left behind.
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from pycontrol_lean import generate  # noqa: E402

    generate(repo, verbose=False)
    pycontrol_dir = Path(args.pycontrol_dir).expanduser().resolve()
    if not (pycontrol_dir / "source" / "communication" / "pycboard.py").exists():
        sys.exit(f"not a pyControl checkout: {pycontrol_dir}")
    os.chdir(pycontrol_dir)
    sys.path.insert(0, str(pycontrol_dir))
    from source.communication.pycboard import Pycboard, PyboardError  # noqa: E402
    from source.gui.settings import VERSION  # noqa: E402

    log = []
    t0 = time.monotonic_ns()

    def record(kind, text):
        log.append({"t_ns": time.monotonic_ns() - t0, "kind": kind, "text": text})

    def say(*parts, end="\n"):
        text = " ".join(str(p) for p in parts)
        print(text, end=end, flush=True)
        record("board_print", text)

    # The rig's own hardware definition and task can be run from their
    # checkout (2026-10-07): the device files still come from pyControl's
    # devices folder, where pycontrol-link points at this repo's dist/.
    hwd_path = Path(args.hwd_path).expanduser().resolve() if args.hwd_path else \
        repo / "pycontrol" / "dist" / "hardware_definitions" / (args.hwd + ".py")
    tasks_dir = Path(args.task_dir).expanduser().resolve() if args.task_dir else \
        repo / "pycontrol" / "dist" / "tasks"
    print(f"pyControl v{VERSION} at {pycontrol_dir}; hardware definition {hwd_path.name}; task {args.task}")

    board = Pycboard(args.port, print_func=say)
    print(f"pyboard unique id {board.unique_ID}, MicroPython {board.micropython_version}, "
          f"framework {board.status['framework']} {getattr(board, 'framework_version', None)}")
    if args.load_framework or not board.status["framework"] or getattr(board, "framework_version", None) != VERSION:
        print("loading the pyControl framework onto the pyboard")
        board.load_framework()
    board.load_hardware_definition(str(hwd_path))
    board.setup_state_machine(args.task, sm_dir=str(tasks_dir))
    print("variables:", board.sm_info.variables)
    for kv in args.set:
        name, _, value = kv.partition("=")
        try:
            value = eval(value, {}, {})
        except Exception:
            pass
        ok = board.set_variable(name, value)
        print(f"set {name} = {value!r}: {ok}")

    id2name = board.sm_info.ID2name
    rc = 0
    try:
        board.start_framework(data_output=True)
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline and board.framework_running:
            new = board.process_data() or []
            for d in new:
                content = d.content
                if d.type.name in ("EVENT", "STATE"):
                    content = f"{id2name.get(d.content, d.content)}"
                line = f"{d.time:>9} {d.type.name:<5} {d.subtype or '':<2} {content}"
                print(line, flush=True)
                record(d.type.name, line)
            time.sleep(0.01)
        if board.framework_running:
            board.stop_framework()
            time.sleep(0.5)
            for d in board.process_data() or []:
                line = f"{d.time:>9} {d.type.name:<5} {d.subtype or '':<2} {d.content}"
                print(line, flush=True)
                record(d.type.name, line)
    except PyboardError as e:
        print("PyboardError:", e)
        record("error", str(e))
        rc = 2
    finally:
        try:
            board.close()
        except Exception:
            pass
    if args.out:
        with open(args.out, "w") as f:
            json.dump({"pycontrol": VERSION, "hwd": args.hwd, "task": args.task, "log": log}, f, indent=1)
        print("->", args.out)
    sys.exit(rc)


if __name__ == "__main__":
    main()
