"""Host-only simulation of the mouse-joystick board.

Nothing here runs on the pyControl board or on the RP2040. It exists so the
device class and the firmware can each be tested against a third reading of
`mouse-joystick-pcb/docs/uart-protocol.md`.
"""

from .board_sim import BoardSim, Fault, State

__all__ = ["BoardSim", "Fault", "State"]
