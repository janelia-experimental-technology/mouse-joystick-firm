"""The lean copies the pyboard receives are the same code as the sources.

`tools/pycontrol_lean.py` removes comments and docstrings for the pyboard's
95 KB flash (the first Gate E upload did not fit, 2026-10-06). The strip is
mechanical, but a strip that broke the code would be found on the board, so
this generates into a temporary directory, compiles every output, imports the
lean codec and exercises it, and checks the device file no longer mentions
unrelated pyControl drivers by class name (pyControl uploads every file whose
class name appears in the text, comments included).
"""

import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

from pycontrol_lean import generate  # noqa: E402


@pytest.fixture(scope="module")
def dist(tmp_path_factory):
    out = tmp_path_factory.mktemp("dist")
    results = generate(ROOT, out_root=out, verbose=False)
    return out, results


def test_every_output_compiles_and_is_smaller(dist):
    out, results = dist
    assert len(results) >= 4
    for src, dst, n_in, n_out in results:
        compile(dst.read_text(encoding="utf-8"), str(dst), "exec")
        assert n_out < n_in, dst.name


def test_the_lean_codec_still_speaks_the_protocol(dist):
    out, _ = dist
    spec = importlib.util.spec_from_file_location("lean_protocol", out / "devices" / "mouse_joystick_protocol.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    P = module.JoystickProtocol
    assert P.encode_command("status", 7) == "status;7\n"
    kind, seq, data = P.decode_reply("ack;7;state=IDLE;angle=0.0;torque=;brake=;homed=0;limit=0;faults=0;hv=0;t=12")
    assert kind == P.KIND_ACK and seq == 7 and data["state"] == "IDLE" and data["t"] == "12"
    assert P.format_number(-12.5) == "-12.5"


def test_the_lean_files_fit_the_pyboard(dist):
    """36 KB were free on the bench pyboard with the framework loaded; the
    four files plus pyControl's own uart_handler.py (0.9 KB) must leave room."""
    out, results = dist
    total = sum(n_out for _, _, _, n_out in results)
    assert total < 30000, total


def test_the_lean_device_file_names_only_what_it_uses(dist):
    out, _ = dist
    text = (out / "devices" / "mouse_joystick.py").read_text(encoding="utf-8")
    assert "JoystickProtocol" in text and "UART_handler" in text
    # Comment-only mentions pulled MCP.py (9 KB) and poke.py onto the board.
    for unrelated in ("MCP23017", "Poke", "MouseJoystickProtocol"):
        assert unrelated not in text, unrelated
