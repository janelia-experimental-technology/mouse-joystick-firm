from pyControl.utility import *
import hardware_definition as hw


# ------------------------------------------------------------------
# Variables
# ------------------------------------------------------------------

v.reward_duration = 50   # ms — solenoid open time for a reward (set by calibration)
v.flushing = 0


# ------------------------------------------------------------------
# State machine
# ------------------------------------------------------------------

states = ["main"]

events = [
    "free_reward",
    "reward_delivered",
    "solenoid_off",
    "flush",
    "rsync",
]

initial_state = "main"


# ------------------------------------------------------------------
# Run start / end
# ------------------------------------------------------------------

def run_end():
    hw.solenoid.off()


# ------------------------------------------------------------------
# States
# ------------------------------------------------------------------

def main(event):
    pass


def all_states(event):

    if event == "free_reward":
        hw.solenoid.on()
        set_timer("solenoid_off", v.reward_duration * ms, output_event=False)
        set_timer("reward_delivered", 1 * ms, output_event=True)

    elif event == "reward_delivered":
        pass

    elif event == "solenoid_off":
        hw.solenoid.off()

    elif event == "flush":
        if v.flushing:
            hw.solenoid.off()
            v.flushing = 0
        else:
            hw.solenoid.on()
            v.flushing = 1

    elif event == "rsync":
        pass
