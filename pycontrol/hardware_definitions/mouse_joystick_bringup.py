# Hardware definition for Stage 4 bring-up of the mouse-joystick boards.
#
# SCOPE: this file is for exercising the joystick boards over pyControl, alone,
# on a bench. It is NOT the rig's hardware definition.
#
# The rig's canonical definition is
#   ~/Repositories/github/joystick-rig/pycontrol/hardware_definitions/
#       hardware_definition_joystick.py
# which owns the rsync output, the 150 Hz camera frame trigger and the reward
# solenoid. When these boards join the rig, `MouseJoystick` instances are ADDED
# to that file. Do not replace it with this one - everything it carries is real
# rig wiring that this file knows nothing about.
#
# Controller: pyControl Breakout Board 1.2.
#
# ---------------------------------------------------------------------------
# Port assignment
# ---------------------------------------------------------------------------
# Breakout 1.2 exposes a UART on ports 1 (UART4), 3 (USART1) and 4 (USART3).
# Two single-axis boards therefore fit with a port to spare. Source:
# `devices/breakout_1_2.py`, recorded in
# `../../../mouse-joystick-pcb/docs/pycontrol-port-pinout.md`.
#
#   Port 1  DIO_A -> board RX, DIO_B -> board TX   left joystick
#   Port 3  DIO_A -> board RX, DIO_B -> board TX   right joystick
#
# On the board's RJ-45, pin 1 (DIO-A) is the board's RX and pin 4 (DIO-B) is the
# board's TX - the controller's TX reaches the board's RX. Pin 6 carries 5 V and
# is one of the two sources that power the board at all, so **the board stays
# reachable over pyControl with USB unplugged and the motor rail off**. That is
# deliberate, per `../../../mouse-joystick-pcb/docs/power.md`.
#
# ---------------------------------------------------------------------------
# What is NOT here, and why
# ---------------------------------------------------------------------------
# `PULL_EVENT`, `FAULT` and `RSYNC` are rev 1. Rev 0 has no hardware-timed event
# or fault path at all: every event is a UART line with UART latency. That is
# fine for exercising the interfaces and NOT fine for behavioural data. Do not
# add BNC wiring here and imply otherwise -
# `../../../mouse-joystick-pcb/docs/pycontrol-interface.md` draws that line.
#
# ---------------------------------------------------------------------------
# EDIT CYCLE - the same trap the rig's definition documents
# ---------------------------------------------------------------------------
# Editing this file on disk does NOT change the pyboard. To make a change take
# effect: edit -> Config -> Load hardware definition -> re-Upload the task ->
# restart the session.
#
# And separately: Config -> "Load framework" DELETES hardware_definition.py from
# the pyboard. After reloading the framework you must reload this file, or task
# upload fails with:
#     ImportError: no module named 'hardware_definition'
# ---------------------------------------------------------------------------

from pyControl.hardware import *
from devices import *

board = Breakout_1_2()

# One object per board. The event names must differ: they are what pyControl
# raises when each link goes idle with data waiting, and a duplicate would
# attribute the right joystick's traffic to the left one. `MouseJoystick`
# asserts on this at construction rather than letting it happen quietly.
left = MouseJoystick(port=board.port_1, event_name="joystick_L")
right = MouseJoystick(port=board.port_3, event_name="joystick_R")

# Both boards must be identified before use. The serial is how a task confirms
# it is talking to the board whose calibration matches the joystick in front of
# it - with two interchangeable single-axis boards in a rig, nothing else
# distinguishes them. The task does this in run_start; it is not done here
# because a hardware definition must not block on a peer that may be unpowered.
