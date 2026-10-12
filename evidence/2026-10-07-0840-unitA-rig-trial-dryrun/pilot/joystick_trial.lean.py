# Generated from /home/peter/Repositories/github/joystick-rig/pycontrol/tasks/rig1/joystick_trial.py by tools/pycontrol_lean.py: comments and docstrings
# removed for the pyboard's flash. Edit the source, not this file.
from pyControl.utility import *
import hardware_definition as hw
v.board = 'right'
v.n_trials = 20
v.present_degrees = -5.0
v.pull_degrees = 5.0
v.push_degrees = 5.0
v.watch_timeout_ms = 10000
v.touch_ncm = 0.3
v.brake_ncm = 1.0
v.reward_duration = 50
v.iti_ms = 3000
v.home_each_trial = 0
v.heartbeat_interval = 40
v.phase = 'idle'
v.trial = 0
v.n_pull = 0
v.n_push = 0
v.n_nomove = 0
v.serial = ''
states = ['identifying', 'powering', 'homing', 'arming', 'presenting', 'braking', 'watching', 'parking', 'iti', 'done', 'faulted']
events = ['joystick_L', 'joystick_R', 'heartbeat', 'solenoid_off', 'rsync']
initial_state = 'identifying'

def board():
    return getattr(hw, v.board)

def run_start():
    set_timer('heartbeat', v.heartbeat_interval * ms, output_event=False)
    hw.solenoid.off()
    print('identifying the {} board'.format(v.board))
    board().identify()

def run_end():
    hw.solenoid.off()
    board().stop()
    board().disarm()
    board().hv(False)

def identifying(event):
    if event == 'entry':
        v.phase = 'identifying'

def powering(event):
    if event == 'entry':
        v.phase = 'powering'
        print('raising the motor rail')
        board().hv(True)

def homing(event):
    if event == 'entry':
        v.phase = 'homing'
        board().home()

def arming(event):
    if event == 'entry':
        v.phase = 'arming'
        board().arm()

def presenting(event):
    if event == 'entry':
        v.phase = 'presenting'
        v.trial += 1
        print('trial {} of {}: presenting at {} deg'.format(v.trial, v.n_trials, v.present_degrees))
        board().move_to(v.present_degrees)

def braking(event):
    if event == 'entry':
        v.phase = 'braking'
        board().brake(v.brake_ncm)

def watching(event):
    if event == 'entry':
        v.phase = 'watching'
        board().torque_zero()
        board().watch(v.pull_degrees, v.push_degrees, v.watch_timeout_ms, touch_ncm=v.touch_ncm)

def parking(event):
    if event == 'entry':
        v.phase = 'parking'
        board().move_to(0.0)

def iti(event):
    if event == 'entry':
        v.phase = 'iti'
        if v.trial >= v.n_trials:
            timed_goto_state('done', 10 * ms)
        elif v.home_each_trial:
            timed_goto_state('homing', v.iti_ms * ms)
        else:
            timed_goto_state('presenting', v.iti_ms * ms)

def done(event):
    if event == 'entry':
        v.phase = 'done'
        print('session complete: {} trials, pull {} push {} nomove {}'.format(v.trial, v.n_pull, v.n_push, v.n_nomove))
        stop_framework()

def faulted(event):
    if event == 'entry':
        v.phase = 'faulted'
        hw.solenoid.off()
        print('FAULTED - reading the fault mask')
        board().faults()

def all_states(event):
    if event == 'heartbeat':
        board().ping()
        set_timer('heartbeat', v.heartbeat_interval * ms, output_event=False)
    elif event == 'solenoid_off':
        hw.solenoid.off()
    elif event == 'joystick_L' or event == 'joystick_R':
        mine = (event == 'joystick_R') == (v.board == 'right')
        source = board() if mine else getattr(hw, 'left' if v.board == 'right' else 'right')
        for message in source.poll():
            if mine:
                _handle(message)
            else:
                print('other board: {}'.format(message))

def _reward():
    hw.solenoid.on()
    set_timer('solenoid_off', v.reward_duration * ms, output_event=False)

def _handle(message):
    kind = message[0]
    if kind == 'event':
        name = message[1]
        t_us, data = message[2]
        print('ev {} t={} {}'.format(name, t_us, data))
        if name == 'HOMED':
            goto_state('arming')
        elif name == 'TARGET':
            if v.phase == 'presenting':
                goto_state('braking')
            elif v.phase == 'parking':
                goto_state('iti')
        elif name == 'RELEASED':
            goto_state('watching')
        elif name == 'PULL':
            v.n_pull += 1
            _reward()
        elif name == 'PUSH':
            v.n_push += 1
        elif name == 'NOMOVE':
            v.n_nomove += 1
        elif name == 'TRIAL':
            goto_state('parking')
        elif name == 'FAULT':
            goto_state('faulted')
    elif kind == 'nak':
        print('nak seq={} reason={}'.format(message[1], message[2]))
        goto_state('faulted')
    elif kind == 'ack':
        data = message[2]
        if 'serial' in data:
            board().apply_identity(data)
            v.serial = board().serial
            print('board serial {} fw {} hw {} proto {} caps {}'.format(data['serial'], data['fw'], data.get('hw'), data['proto'], data.get('caps')))
            goto_state('powering')
        elif 'mask' in data:
            print('faults {}'.format(data))
        elif v.phase == 'powering':
            goto_state('homing')
        elif v.phase == 'arming':
            goto_state('presenting')
