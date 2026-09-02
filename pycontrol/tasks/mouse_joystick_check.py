# Stage 4 bring-up exerciser for one mouse-joystick board over pyControl.
#
# Walks the protocol's state machine from boot to a completed move, printing
# every line the board sends. This is a bring-up tool, not an experiment: there
# is no trial structure, no reward, and no subject.
#
# Use with hardware definition `mouse_joystick_bringup.py`.
#
# WHAT TO EXPECT IN THE LOG, in order:
#   ID reply carrying the board serial - record it, it is how this board's
#   bring-up results are attributed in
#   ../../../mouse-joystick-pcb/docs/bringup-log.md
#   then HV on, HOMED, ARM, and a small move ending in TARGET.
#
# SAFETY: this raises the motor rail and moves the joystick. Nothing should be
# attached to the joystick that a 0.211 A RMS stepper can damage. The move is
# bounded by MOVE_DEGREES below and by the board's own travel limits, which it
# enforces locally and will refuse to exceed.

from pyControl.utility import *
import hardware_definition as hw

# ---------------------------------------------------------------------------
# Variables
# ---------------------------------------------------------------------------

# The board faults on heartbeat loss after 100 ms. Pinging at 40 ms leaves room
# for a missed reply without the watchdog expiring. Do NOT raise this above the
# board's timeout while testing something else - a HOST_TIMEOUT fault mid-run
# looks exactly like a firmware bug.
v.heartbeat_interval = 40  # ms

# Small and well inside the board's travel. The board rejects anything outside
# its limits, but a bring-up move should not be relying on that.
v.move_degrees = 5.0

# Set 0 to stop after homing, which is the right setting the first time a board
# is connected: it proves the link and the state machine without commanding
# motion under host control.
v.do_move = 1

v.serial = ""

# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

states = [
    "identifying",
    "powering",
    "homing",
    "arming",
    "moving",
    "done",
    "faulted",
]

events = [
    # Raised by pyControl's UART_handler when the link goes idle with data
    # waiting. Must be declared here or UART_handler finds no event ID and
    # silently does nothing - see devices/uart_handler.py, which looks the name
    # up in sm.events at initialise time.
    "joystick_L",
    "heartbeat",
]

initial_state = "identifying"


# ---------------------------------------------------------------------------
# Run start / end
# ---------------------------------------------------------------------------


def run_start():
    set_timer("heartbeat", v.heartbeat_interval * ms, output_event=False)
    hw.left.identify()


def run_end():
    # Leave the board safe regardless of which state the run stopped in. The
    # board does not require this - it faults itself on heartbeat loss - but
    # relying on the watchdog to tidy up means every clean stop also logs a
    # fault.
    hw.left.disarm()
    hw.left.hv(False)


# ---------------------------------------------------------------------------
# States
# ---------------------------------------------------------------------------


def identifying(event):
    if event == "entry":
        print("identifying board on port 1")


def powering(event):
    if event == "entry":
        print("raising the motor rail")
        hw.left.hv(True)


def homing(event):
    if event == "entry":
        print("homing")
        hw.left.home()


def arming(event):
    if event == "entry":
        hw.left.arm()


def moving(event):
    if event == "entry":
        print("moving to {} deg".format(v.move_degrees))
        hw.left.move_to(v.move_degrees)


def done(event):
    if event == "entry":
        print("bring-up sequence complete; serial {}".format(v.serial))


def faulted(event):
    if event == "entry":
        print("FAULTED - reading the fault mask")
        hw.left.faults()


# ---------------------------------------------------------------------------
# All states
# ---------------------------------------------------------------------------


def all_states(event):
    if event == "heartbeat":
        hw.left.ping()
        set_timer("heartbeat", v.heartbeat_interval * ms, output_event=False)

    elif event == "joystick_L":
        # One UART idle event can cover several lines, so drain the whole batch.
        for message in hw.left.poll():
            _handle(message)


def _handle(message):
    kind = message[0]

    if kind == "event":
        name = message[1]
        t_us, data = message[2]
        print("ev {} t={} {}".format(name, t_us, data))

        if name == "HOMED":
            goto_state("arming")
        elif name == "TARGET":
            goto_state("done")
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
            hw.left.apply_identity(data)
            v.serial = hw.left.serial
            print("board serial {} fw {} proto {}".format(data["serial"], data["fw"], data["proto"]))
            goto_state("powering")

        elif "mask" in data:
            print("faults {}".format(data))

        elif get_current_state() == "powering":
            goto_state("homing")

        elif get_current_state() == "arming":
            if v.do_move:
                goto_state("moving")
            else:
                goto_state("done")
