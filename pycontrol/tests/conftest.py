"""Import paths and shared fixtures for the host-side pyControl tests.

`devices/` is on the path as a flat directory rather than a package because that
is how it exists on the pyControl board: `transfer_folder` copies loose `.py`
files into one folder and `devices/__init__.py` star-imports them. Testing the
codec through a package import would test an arrangement that never ships.
"""

import sys
from pathlib import Path

import pytest

PYCONTROL_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PYCONTROL_ROOT.parent

for path in (PYCONTROL_ROOT, PYCONTROL_ROOT / "devices"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

# A serial that is not either real board's, so a test can never be mistaken for
# a bring-up record. The real ones are E4654C159F2C2534 and E4654C159F4D1334.
TEST_SERIAL = "0000000000000000"


@pytest.fixture
def sim():
    """A booted board, with the READY event already drained."""
    from sim import BoardSim

    board = BoardSim(serial=TEST_SERIAL)
    board.boot()
    return board


@pytest.fixture
def armed(sim):
    """A board in ARMED: HV on, homed, armed. The precondition for motion."""
    sim.feed("hv;1;on\n")
    sim.feed("home;2\n")
    sim.advance(sim.home_duration_us)
    sim.feed("arm;3\n")
    assert sim.state == "ARMED"
    return sim


@pytest.fixture
def protocol():
    from mouse_joystick_protocol import JoystickProtocol

    return JoystickProtocol
