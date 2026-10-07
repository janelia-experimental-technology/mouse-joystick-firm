# Stage 4 bring-up exerciser for one mouse-joystick board over pyControl,
# protocol v1 (2026-10-06; the first version spoke the retired rev-0 verbs
# `arm_pull` and `freewheel`, which no longer exist on the device class).
#
# One trial's shape, as far as a bench without an animal can go: identify the
# board, raise its motor rail, home (park), arm, present the joystick with a
# bounded move, engage the brake and release the drive (`BRAKE`, one board
# operation), arm the classifier (`WATCH`), take whatever outcome comes --
# `PULL`/`PUSH` if a hand moves the joystick, `NOMOVE` at the timeout
# otherwise -- read the `TRIAL` summary, park again, and stop. Every line the
# board sends is printed. A bring-up tool, not an experiment: no cue, no
# reward, no subject.
#
# Use with hardware definition `mouse_joystick_bringup.py`, which constructs
# `left` (port 1) and `right` (port 4). `v.board` names the one this run
# drives; the other may be unplugged (its UART is constructed, nothing is sent
# to it). Gate E, `../../../mouse-joystick-pcb/docs/validation-plan.md`.
#
# WHAT TO EXPECT IN THE LOG, in order: the ID reply with the board serial
# (record it -- it attributes the results in
# ../../../mouse-joystick-pcb/docs/bringup-log.md), HV on, HOMED, ARM, a
# small move ending in TARGET, the brake engaging and RELEASED, WATCH, then
# NOMOVE after `v.watch_timeout_ms` (or PULL/PUSH/TOUCH if you move the
# joystick by hand), TRIAL, the park move's TARGET, "done".
#
# SAFETY: this raises the motor rail, moves the joystick, and then RELEASES the
# driver with the brake holding `v.brake_ncm`. Nothing should be attached that a
# 0.211 A RMS stepper can damage, and nothing should be hanging on the joystick
# when it goes compliant. `v.do_move = 0` stops after arming (the right setting
# the first time a board is connected); `v.do_brake = 0` stops after the move.
# Every move is bounded by `v.move_degrees` and by the board's own travel
# limits, which it enforces locally and refuses to exceed.

from pyControl.utility import *
import hardware_definition as hw

# ---------------------------------------------------------------------------
# Variables
# ---------------------------------------------------------------------------

# Which board this run drives: "left" (port 1) or "right" (port 4).
v.board = "right"

# The board faults on heartbeat loss after 100 ms. Pinging at 40 ms leaves room
# for a missed reply without the watchdog expiring. Do NOT raise this above the
# board's timeout while testing something else - a HOST_TIMEOUT fault mid-run
# looks exactly like a firmware bug.
v.heartbeat_interval = 40  # ms

# The presented position. Travel runs negative from the parked zero (the
# switch end); -5 is well inside the ~-10 deg the board reports in LIMITS.
v.move_degrees = -5.0

# Set 0 to stop after arming, which is the right setting the first time a board
# is connected: it proves the link and the state machine without commanding
# motion under host control.
v.do_move = 1

# Set 0 to stop after the move rather than engaging the brake and releasing
# the drive. Compliant, the joystick is held only by the brake.
v.do_brake = 1

# The brake's holding torque for the release (N.cm, 0 to 3.4). 1.0 is about
# 14 mA of coil current on pcb v1.0.
v.brake_ncm = 1.0

# The classifier: pull-positive excursions in degrees from the angle at
# release, the time without first motion before NOMOVE, and the torque at
# which TOUCH is reported (N.cm).
v.pull_degrees = 5.0
v.push_degrees = 5.0
v.watch_timeout_ms = 10000
v.touch_ncm = 0.3

v.serial = ""
v.outcome = ""
# Bench knobs for the tick-budget attribution (2026-10-06): the board's host
# watchdog in ms (0 = leave the default 100), set before the rail comes on so
# a slower heartbeat can be used (heartbeat_interval must stay under it); and
# whether the instrument port, and with it the QS trace, is left on.
v.wdt_ms = 0
v.instr = 1
# Bench knob for the park error (2026-10-06): 1 parks with a move as a trial
# would; 0 leaves RELEASED with `stop` instead and reads `status` right after
# and again 300 ms later, so angle (encoder) and xactual (ramp generator) can
# be compared across the brake release and the driver's re-engage without a
# move in between; 2 does the same and then parks with a move 300 ms after the
# stop, to separate the brake's release from the move itself; 3 parks with a
# move and then runs PROBE_MOVES back to back, printing each TARGET, to tell
# backlash (a fixed loss per direction reversal) from a loss proportional to
# distance (encoder) or accumulating per move (motor).
v.park = 1
v.probe_i = 0
# Bench knobs for the oscilloscope (2026-10-07): hold = 1 stops after RELEASED
# and stays there, brake set and drive released, printing status every
# status_period_ms, until the run ends (no watch, no TRIAL, no park), so a
# hand can move the joystick against the brake while the angle and torque
# BNCs are on a scope; angout = 1 turns the angle output on (ANGSCALE
# ang_lo..ang_hi, the board's travel: LIMITS travel_lo/travel_hi, -10.125..0
# on pcb v1.0 with the rig firmware), sent right behind the present move
# (acks in `moving` are ignored; behind `home` one of them was lost).
v.hold = 0
v.status_period_ms = 2000
v.angout = 0
v.ang_lo = -10.125
v.ang_hi = 0.0
PROBE_MOVES = [-5.0, -2.5, 0.0, -2.5, -5.0, 0.0]
# The state the task is in, kept by hand: pyControl's task API has no
# "current state" query (found on the pyboard 2026-10-06), and the replies to
# `hv`, `arm` and the two moves mean different things in different states.
v.phase = "identifying"

# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

states = [
    "identifying",
    "powering",
    "homing",
    "arming",
    "moving",
    "braking",
    "watching",
    "holding",
    "parking",
    "done",
    "faulted",
]

events = [
    # Raised by pyControl's UART_handler when a link goes idle with data
    # waiting. Both boards' names must be declared here or UART_handler finds
    # no event ID and silently does nothing - see devices/uart_handler.py,
    # which looks the name up in sm.events at initialise time.
    "joystick_L",
    "joystick_R",
    "heartbeat",
    "status_later",
]

initial_state = "identifying"


def board():
    return getattr(hw, v.board)


# ---------------------------------------------------------------------------
# Run start / end
# ---------------------------------------------------------------------------


def run_start():
    set_timer("heartbeat", v.heartbeat_interval * ms, output_event=False)
    # A board left FAULTED by the previous run (a host that died mid-session
    # trips its host-loss watchdog) would refuse `hv on`; `clrfault` on a
    # clean board is a plain ack. Sent BEFORE `id` so its ack lands while the
    # phase is still `identifying`, where no ack is interpreted.
    board().clear_fault()
    print("identifying the {} board".format(v.board))
    board().identify()


def run_end():
    # STOP first: it re-engages the driver and reloads position from the
    # encoder, so the board is not left compliant with an animal on it if the
    # run was stopped mid-trial. Then DISARM and the rail off. The board does
    # not require this - it faults itself on heartbeat loss - but relying on
    # the watchdog to tidy up means every clean stop also logs a fault.
    board().stop()
    board().disarm()
    board().hv(False)


# ---------------------------------------------------------------------------
# States
# ---------------------------------------------------------------------------


def identifying(event):
    if event == "entry":
        v.phase = "identifying"
        # Replies to these carry no data and no phase handles them, so they
        # are ignored by _handle; the id reply is told apart by its serial.
        if v.wdt_ms:
            board().cfg(wdt_ms=v.wdt_ms)
        if not v.instr:
            board().instrumentation(False)


def powering(event):
    if event == "entry":
        v.phase = "powering"
        print("raising the motor rail")
        board().hv(True)


def homing(event):
    if event == "entry":
        v.phase = "homing"
        print("homing")
        board().home()


def arming(event):
    if event == "entry":
        v.phase = "arming"
        board().arm()


def moving(event):
    if event == "entry":
        v.phase = "moving"
        print("presenting: moving to {} deg".format(v.move_degrees))
        board().move_to(v.move_degrees)
        if v.angout:
            # Sent behind the move, not behind `home`: the home blocks the
            # board ~130 ms and three lines queued behind it cost one a
            # `badframe` on 2026-10-07; a move is non-blocking. The acks in
            # `moving` are ignored (an ack in `arming` would be read as arm's).
            board().angle_scale(v.ang_lo, v.ang_hi)
            board().angle_output(True)
            print("angle output on, {} to {} deg onto the BNC".format(v.ang_lo, v.ang_hi))


def braking(event):
    # One board operation: the brake to v.brake_ncm, the coil current settled,
    # then the drive released. RELEASED follows the ack by the brake's ramp
    # and settle (11-17 ms measured on pcb v1.0, 2026-10-06).
    if event == "entry":
        v.phase = "braking"
        print("brake {} N.cm, then release".format(v.brake_ncm))
        board().brake(v.brake_ncm)


def watching(event):
    if event == "entry":
        v.phase = "watching"
        board().status()  # angle vs xactual at RELEASED, printed by _handle
        print("watching: pull past {} deg or push past {} deg within {} ms".format(
            v.pull_degrees, v.push_degrees, v.watch_timeout_ms))
        # Tare first: the reading at rest after RELEASED is -0.2 to -0.35 N.cm
        # on pcb v1.0 unit A (the sensor's rest offset plus the brake's
        # release), which is above a 0.3 N.cm touch level with nobody there
        # (2026-10-06). The board applies the zero before the watch starts;
        # both lines are queued back to back.
        if board().has("torque"):
            board().torque_zero()
        board().watch(v.pull_degrees, v.push_degrees, v.watch_timeout_ms, touch_ncm=v.touch_ncm)


def holding(event):
    # RELEASED with the brake at v.brake_ncm and no watch: the joystick is
    # the hand's until the run ends (run_end stops, disarms, drops the rail).
    if event == "entry":
        v.phase = "holding"
        print("holding RELEASED with brake {} N.cm; move the joystick by hand; status every {} ms".format(
            v.brake_ncm, v.status_period_ms))
        board().status()
        set_timer("status_later", v.status_period_ms * ms, output_event=False)
    elif event == "status_later":
        board().status()
        set_timer("status_later", v.status_period_ms * ms, output_event=False)


def parking(event):
    # Leaving RELEASED: the board re-engages the drive, resyncs from the
    # encoder, emits TRIAL, releases the brake, then moves.
    if event == "entry":
        v.phase = "parking"
        if v.park == 1 or v.park == 3:
            print("parking at 0 deg")
            board().move_to(0.0)
        else:
            print("leaving RELEASED with stop, no move; status now and in 300 ms")
            board().stop()
            board().status()
            set_timer("status_later", 300 * ms, output_event=False)
    elif event == "status_later":
        board().status()
        if v.park == 2:
            print("parking at 0 deg, 300 ms after the stop")
            board().move_to(0.0)
        else:
            timed_goto_state("done", 100 * ms)


def done(event):
    if event == "entry":
        v.phase = "done"
        print("bring-up sequence complete; serial {} outcome {}".format(v.serial, v.outcome))


def faulted(event):
    if event == "entry":
        v.phase = "faulted"
        print("FAULTED - reading the fault mask")
        board().faults()


# ---------------------------------------------------------------------------
# All states
# ---------------------------------------------------------------------------


def all_states(event):
    if event == "heartbeat":
        board().ping()
        set_timer("heartbeat", v.heartbeat_interval * ms, output_event=False)

    elif event == "joystick_L" or event == "joystick_R":
        # One UART idle event can cover several lines, so drain the whole
        # batch. Only the driven board's lines are acted on; the other board,
        # if present, is drained and printed.
        mine = (event == "joystick_R") == (v.board == "right")
        source = board() if mine else getattr(hw, "left" if v.board == "right" else "right")
        for message in source.poll():
            if mine:
                _handle(message)
            else:
                print("other board: {}".format(message))


def _handle(message):
    kind = message[0]

    if kind == "event":
        name = message[1]
        t_us, data = message[2]
        print("ev {} t={} {}".format(name, t_us, data))

        if name == "HOMED":
            goto_state("arming")
        elif name == "TARGET":
            if v.phase == "moving":
                if v.do_brake:
                    goto_state("braking")
                else:
                    goto_state("done")
            elif v.phase == "parking":
                if v.park == 3 and v.probe_i < len(PROBE_MOVES):
                    target = PROBE_MOVES[v.probe_i]
                    v.probe_i += 1
                    print("probe move {} to {} deg".format(v.probe_i, target))
                    board().move_to(target)
                else:
                    goto_state("done")
        elif name == "RELEASED":
            goto_state("holding" if v.hold else "watching")
        elif name in ("PULL", "PUSH", "NOMOVE", "ABORT"):
            v.outcome = name
            print("outcome {} at angle {} torque {}".format(name, data.get("angle"), data.get("torque")))
        elif name == "TRIAL":
            # The summary comes one second after the outcome if the host has
            # not left RELEASED by then; parking leaves it.
            goto_state("parking")
        elif name == "FAULT":
            goto_state("faulted")

    elif kind == "nak":
        # Printed rather than ignored: a nak means the board refused, and a
        # bring-up run that quietly skips a refusal proves nothing.
        print("nak seq={} reason={}".format(message[1], message[2]))
        goto_state("faulted")

    elif kind == "ack":
        data = message[2]

        if "serial" in data:
            board().apply_identity(data)
            v.serial = board().serial
            print("board serial {} fw {} hw {} proto {} caps {}".format(
                data["serial"], data["fw"], data.get("hw"), data["proto"], data.get("caps")))
            goto_state("powering")

        elif "mask" in data:
            print("faults {}".format(data))

        elif "xactual" in data:
            print("status state {} angle {} xactual {} torque {} brake {}".format(
                data.get("state"), data.get("angle"), data.get("xactual"),
                data.get("torque"), data.get("brake")))

        elif v.phase == "powering":
            goto_state("homing")

        elif v.phase == "arming":
            if v.do_move:
                goto_state("moving")
            else:
                goto_state("done")
