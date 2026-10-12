# Generated from /home/peter/Repositories/github/joystick-rig/pycontrol/hardware_definitions/hardware_definition_joystick.py by tools/pycontrol_lean.py: comments and docstrings
# removed for the pyboard's flash. Edit the source, not this file.
from pyControl.hardware import *
from devices import *
board = Breakout_1_2()
sync_output = Rsync(pin=board.DAC_2, mean_IPI=1000, pulse_dur=25, event_name='rsync')
camera_trigger = Frame_trigger(pin=board.BNC_1, pulse_rate=150, name='cam_trigger')
solenoid = Digital_output(pin=board.port_5.POW_A)
left = MouseJoystick(port=board.port_1, event_name='joystick_L')
right = MouseJoystick(port=board.port_4, event_name='joystick_R')
opto = Opto_LED(pin=board.BNC_2)
