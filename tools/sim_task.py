#!/usr/bin/env python3
"""Run a pyControl task on the host against the board simulator, with reply loss.

    pixi run -e pycontrol python tools/sim_task.py TASK.py [SCENARIO] [N_TRIALS]

Real layers in the loop: the task file, the device class, the codec and the
simulator. Faked: the pyControl framework (states, timers, print, the
variables object) and the UART (tests/stubs.SimLink), so a reply line can be
dropped or corrupted on purpose. Virtual time runs in 1 ms steps; the heartbeat,
timed_goto_state and set_timer all fire on it. A pass here says the task's state
machine survives the scenario; it says nothing about the pyboard's flash, its
UART driver or pyControl's own scheduler.

The task is loaded the way pyControl loads it: `from pyControl.utility import *`
and `import hardware_definition`. The fake hardware definition gives it
`right` (the simulated board), `left` (a board that never speaks) and
`solenoid`; a task that needs more is not this tool's job. The simulated board
is a pcb v1.0 (brake, torque, angle output), homing in 2.5 s.

Scenarios:
  nominal          no loss
  drop-hv-ack      the ack to `hv on` is lost once
  drop-arm-acks    every ack to `arm` is lost (the resends run out)
  corrupt-id-ack   the ack to `id` arrives with a byte above 0x7F
  drop-trial       the TRIAL event is lost
  drop-target      the first TARGET event is lost
  drop-released    the RELEASED event is lost

Written 2026-10-12 to test the trial task's reply timeouts (joystick-rig PR 9,
review item 2); the first run of each scenario is recorded in that PR.
"""
import builtins, os, sys, types, importlib.util

HERE = os.path.dirname(os.path.abspath(__file__))
FIRM = os.path.normpath(os.path.join(HERE, "..", "pycontrol"))
sys.path[:0] = [FIRM, FIRM + "/devices", FIRM + "/tests"]
import stubs
from sim import BoardSim

OUT = []
now = 0
def say(s):
    OUT.append((now, s)); builtins.print("%8d  %s" % (now, s))

class LossyLink(stubs.SimLink):
    def __init__(self, sim):
        super().__init__(sim); self.rules = []
    def lose(self, pred, n=1, mode="drop"):
        self.rules.append({"pred": pred, "n": n, "mode": mode})
    def _absorb(self, lines):
        for line in lines:
            for r in self.rules:
                if r["n"] > 0 and r["pred"](self, line):
                    r["n"] -= 1
                    say("[link] %s: %s" % (r["mode"], line))
                    if r["mode"] == "corrupt":
                        self._pending += line[:4].encode() + b"\xff" + line[5:].encode() + b"\n"
                    break
            else:
                self._pending += (line + "\n").encode()

# ---- fake pyControl framework -------------------------------------------------
class SM:
    running = True; current = None; task = None; entered = 0
    def dispatch(self, ev):
        if hasattr(self.task, "all_states"):
            if self.task.all_states(ev): return
        fn = getattr(self.task, self.current, None)
        if fn: fn(ev)
    def goto(self, state):
        self.dispatch("exit")
        timers[:] = [t for t in timers if t[2] != "state"]
        self.current = state; self.entered = now
        say("-> %s" % state)
        self.dispatch("entry")
sm = SM()
timers = []
util = types.ModuleType("pyControl.utility")
class V: pass
util.v = V(); util.ms = 1; util.second = 1000; util.minute = 60000
util.print = say
util.goto_state = lambda s: sm.goto(s)
util.timed_goto_state = lambda s, interval: timers.append([now + interval, s, "state"])
def set_timer(event, interval, output_event=True): timers.append([now + interval, event, "event"])
def disarm_timer(event): timers[:] = [t for t in timers if not (t[1] == event and t[2] == "event")]
def reset_timer(event, interval, output_event=True): disarm_timer(event); set_timer(event, interval, output_event)
util.set_timer = set_timer; util.disarm_timer = disarm_timer; util.reset_timer = reset_timer
def stop_framework(): sm.running = False; say("[fw] stop_framework")
util.stop_framework = stop_framework
pkg = types.ModuleType("pyControl"); pkg.utility = util
sys.modules["pyControl"] = pkg; sys.modules["pyControl.utility"] = util

# ---- hardware -----------------------------------------------------------------
sim = BoardSim(serial="0000000000000000", hw_rev="rev2", home_duration_us=2_500_000)
sim.has_brake = True; sim.has_torque = True; sim.has_angout = True
sim.boot()
link = LossyLink(sim)
teardown = stubs.install(link)
from mouse_joystick import MouseJoystick
MouseJoystick.registered_event_names.clear()
hw = types.ModuleType("hardware_definition")
hw.right = MouseJoystick(port=stubs.FakePort(uart=4), event_name="joystick_R")
class Dummy:
    def poll(self): return []
hw.left = Dummy()
class Solenoid:
    def on(self): say("[solenoid] on")
    def off(self): pass
hw.solenoid = Solenoid()
sys.modules["hardware_definition"] = hw

# ---- the task ------------------------------------------------------------------
TASK = sys.argv[1] if len(sys.argv) > 1 else os.path.join(FIRM, "tasks", "mouse_joystick_check.py")
spec = importlib.util.spec_from_file_location("task_under_test", TASK)
task = importlib.util.module_from_spec(spec); spec.loader.exec_module(task)
sm.task = task
v = task.v
if hasattr(v, "n_trials"):
    v.n_trials = int(sys.argv[3]) if len(sys.argv) > 3 else 2
if hasattr(v, "watch_timeout_ms"):
    v.watch_timeout_ms = 2000
if hasattr(v, "iti_ms"):
    v.iti_ms = 1000

last_cmd = lambda link: link.written[-1] if link.written else ""
scenario = sys.argv[2] if len(sys.argv) > 2 else "nominal"
if scenario == "drop-hv-ack":
    link.lose(lambda l, line: line.startswith("ack;") and last_cmd(l).startswith("hv;"), n=1)
elif scenario == "drop-arm-acks":
    link.lose(lambda l, line: line.startswith("ack;") and last_cmd(l).startswith("arm;"), n=10)
elif scenario == "corrupt-id-ack":
    link.lose(lambda l, line: line.startswith("ack;") and last_cmd(l).startswith("id;"), n=1, mode="corrupt")
elif scenario == "drop-trial":
    link.lose(lambda l, line: line.startswith("ev;TRIAL"), n=1)
elif scenario == "drop-target":
    link.lose(lambda l, line: line.startswith("ev;TARGET"), n=1)
elif scenario == "drop-released":
    link.lose(lambda l, line: line.startswith("ev;RELEASED"), n=1)

# ---- run -----------------------------------------------------------------------
say("[task] %s  scenario %s" % (TASK, scenario))
task.run_start(); sm.current = task.initial_state; say("-> %s" % sm.current); sm.dispatch("entry")
MAX_MS = 60_000
# A task that rests in `done` without stop_framework (the firmware's check task
# does; pyControl would run until its --seconds) counts as finished after 200 ms.
while sm.running and now < MAX_MS and not (sm.current == "done" and now - sm.entered > 200):
    now += 1
    link.advance(1000)
    if link.any(): sm.dispatch("joystick_R")
    while sm.running:
        due = [t for t in timers if t[0] <= now]
        if not due: break
        t = min(due, key=lambda t: t[0]); timers.remove(t)
        if t[2] == "event": sm.dispatch(t[1])
        else: sm.goto(t[1])
task.run_end()
link.advance(1000); hw.right.poll()
say("[end] state=%s board.state=%s hv_on=%s pings=%d lines_written=%d" % (
    sm.current, sim.state, sim.hv_on, sum(1 for w in link.written if w.startswith("ping;")), len(link.written)))
if now >= MAX_MS and sm.current != "done":
    say("[end] TIMED OUT: the session never finished")
    sys.exit(1)
sys.exit(0 if sm.current in ("done", "faulted") else 1)
