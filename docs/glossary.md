# Glossary

Terms as this firmware, the protocol and the rig's task use them.

| Term | Meaning |
| --- | --- |
| **Station** | One joystick: its mechanism, its controller board, its harness. The rig has a left and a right station; the left mechanism is the mirror of the right and takes the left harness. |
| **Board, unit** | One `mouse-joystick-pcb`. Identified by its RP2040 flash serial, which `ID` reports and the bring-up log and calibration table key on. Units A to D are the four v1.0 boards. |
| **Home** | Drive toward the limit switch at the positive end of travel until it closes, then set the zero. Position 0° is the parked position at the switch end; travel runs negative from there. |
| **Present** | Move the lever to the presented angle (the task's `present_degrees`, −5° by default) where the animal will take it. |
| **Brake, release** | Set the magnetic particle brake to a torque, wait for the coil current to settle, then release the motor drive so the lever is compliant against the brake only. The board reports `ev RELEASED`. |
| **Watch** | The one-shot classifier armed after release. It reports exactly one outcome: `PULL`, `PUSH`, `NOMOVE` (no first motion within the timeout) or `ABORT`. `TOUCH` may precede the outcome. |
| **Pull, push** | Displacement from the angle at release, pull-positive. On both stations a pull is the decreasing-angle direction; the per-board **pull sign** in `src/BoardCalibration.hpp` makes that true in software. |
| **Tare** | `TQ zero`: store the torque reading at rest as the offset, so a touch is measured from zero. The task tares before every watch. |
| **Park** | Move back to 0° after the trial. A lever held during the park stalls the motor, faults the board and ends the session, by design. |
| **TRIAL** | The per-trial summary event: outcome, angle and torque extrema, and the board times of release, touch, first movement and outcome. Emitted once per watch, when the board leaves `RELEASED` or one second after the outcome. |
| **Heartbeat** | The task's `ping` every 40 ms. Any well-formed line resets the board's host-loss watchdog. |
| **Host-loss watchdog** | Armed by `hv on`: 100 ms without a line from the host drops the motor rail and faults the board. |
| **Fault mask** | The bits `FAULTS` reports and `ev FAULT` announces: driver, motion, host timeout, brake fault, brake current, torque ADC, torque range, not following. `CLRFAULT` clears it and the board returns to `IDLE`. |
| **Sequence number, ack, nak** | Every command carries a number; the board answers `ack;n` or `nak;n;reason`. The task matches the ack to the number it waits for and ignores the heartbeat's. |
| **Switched domain, MCU domain** | The board's two power domains: the always-on MCU side and the 36 V side behind the high-side switch and the GPIO bus switch. |
| **Instrument port** | The 50-pin connector with 32 buffered digital taps for a logic analyzer; `FW_TRIG0` and `FW_TRIG1` carry firmware markers and the QS trace. |
| **Stage** | A bring-up step bounded by a power domain: 1 the MCU alone, 2 driver register reads, 3 motion, 4 the protocol, 5 the analog output, A4 and A5 the brake and torque. |
| **Delivery build** | `examples/PyControlLinkQp` for environment `mouse_joystick_qp_rev2`: the firmware the rigs run. |
| **Lean copies** | The comment-stripped device files generated for the pyboard's small flash and copied into the rig's repository. Edit the sources here, not the copies. |
