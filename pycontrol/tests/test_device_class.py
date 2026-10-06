"""The pyControl device class, driven against the simulator.

Three real layers in the loop - device class, codec, simulator - with only the
physical UART faked. What is NOT covered here, and needs the Breakout 1.2:
interrupt timing, `UART.IRQ_RXIDLE` firing, rxbuf overrun, and MicroPython's own
UART driver. A pass here is not a pass on hardware.
"""

from pathlib import Path

import pytest

import stubs
from sim import BoardSim
from mouse_joystick_protocol import JoystickProtocol as P

DEVICES_DIR = Path(__file__).resolve().parents[1] / "devices"


def test_sibling_imports_are_package_qualified():
    """A bare sibling import passes every host test and fails on the pyboard.

    Uploaded device files live inside the `devices` package and
    `/flash/devices` is not on the board's sys.path, so
    `from mouse_joystick_protocol import ...` raises ImportError there while
    importing fine on a host that has the folder on sys.path. pyControl's own
    files use the qualified form - `from devices.poke import _Poke`,
    `from devices.MCP import MCP23017`.

    Asserted against the source text because the host cannot reproduce the
    board's import environment, so no runtime test can catch this.
    """
    for path in sorted(DEVICES_DIR.glob("*.py")):
        for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            stripped = line.strip()
            if not stripped.startswith(("import ", "from ")):
                continue
            for sibling in DEVICES_DIR.glob("*.py"):
                name = sibling.stem
                if name == path.stem:
                    continue
                assert not stripped.startswith(
                    ("from %s " % name, "import %s" % name)
                ), "%s:%d imports sibling %r unqualified; use `from devices.%s import ...`" % (
                    path.name,
                    line_no,
                    name,
                    name,
                )


@pytest.fixture
def link():
    board = BoardSim(serial="0000000000000000")
    board.boot()
    return stubs.SimLink(board)


@pytest.fixture
def device(link):
    teardown = stubs.install(link)
    from mouse_joystick import MouseJoystick

    MouseJoystick.registered_event_names.clear()
    joystick = MouseJoystick(port=stubs.FakePort(uart=4), event_name="joystick_L")
    joystick.link = link
    yield joystick
    MouseJoystick.registered_event_names.clear()
    teardown()


def replies(device, kinds=(P.KIND_ACK, P.KIND_NAK)):
    return [m for m in device.poll() if m[0] in kinds]


def events(device):
    return [m for m in device.poll() if m[0] == P.KIND_EVENT]


# ------------------------------------------------------------ port contract


def test_refuses_a_port_without_a_uart(link):
    """Breakout 1.2 exposes a UART on ports 1, 3 and 4 only.

    Without this assertion a wrong port produces a link that silently never
    receives, which looks exactly like a firmware bug.
    """
    teardown = stubs.install(link)
    try:
        from mouse_joystick import MouseJoystick

        MouseJoystick.registered_event_names.clear()
        with pytest.raises(AssertionError, match="port with a UART"):
            MouseJoystick(port=stubs.FakePort(uart=None), event_name="joystick_L")
    finally:
        teardown()


def test_uart_is_configured_as_the_protocol_requires(device, link):
    assert link.init_kwargs["baudrate"] == 115200
    assert link.init_kwargs["bits"] == 8
    assert link.init_kwargs["parity"] is None
    assert link.init_kwargs["stop"] == 1
    # "Keep the device's receive buffer at least as large as the longest command
    # plus margin."
    assert link.init_kwargs["rxbuf"] >= P.MAX_LINE
    # A read must never block the pyControl main loop: timeout 0, and poll()
    # reads only what any() reports (2026-10-01 review, bug 7).
    assert link.init_kwargs["timeout"] == 0


def test_registers_an_rxidle_interrupt_handler(device, link):
    assert link.handler is not None


# ----------------------------------------------------------- two instances


def test_two_instances_on_two_ports_do_not_collide_on_event_names(link):
    """Gate C: two device-class instances on two ports do not collide.

    Neither instance can see the other, so the only place this is checkable is
    the class. A duplicate name would mean the second board's UART_handler
    quietly raises the first board's event, and every event from the right
    joystick would be attributed to the left one.
    """
    teardown = stubs.install(link)
    try:
        from mouse_joystick import MouseJoystick

        MouseJoystick.registered_event_names.clear()
        MouseJoystick(port=stubs.FakePort(uart=4), event_name="joystick_L")

        with pytest.raises(AssertionError, match="already in use"):
            MouseJoystick(port=stubs.FakePort(uart=1), event_name="joystick_L")

        right = MouseJoystick(port=stubs.FakePort(uart=1), event_name="joystick_R")
        assert right.event_name == "joystick_R"
        assert MouseJoystick.registered_event_names == {"joystick_L", "joystick_R"}
        MouseJoystick.registered_event_names.clear()
    finally:
        teardown()


# ------------------------------------------------------------- discovery


def test_identify_records_serial_and_versions(device):
    device.identify()
    message = replies(device)[0]
    kind, seq, data = message
    assert kind == P.KIND_ACK

    serial = device.apply_identity(data)
    assert serial == "0000000000000000"
    assert device.protocol_version == 2
    assert device.axis_count == 1
    assert device.firmware_version == "0.0.0"


def test_serial_is_none_before_identify(device):
    """A task that records provenance must not get a plausible-looking default."""
    assert device.serial is None
    assert device.protocol_version is None


def test_refuses_an_incompatible_protocol_major_version(device):
    """The draft: refuse rather than guess."""
    with pytest.raises(AssertionError, match="protocol v3"):
        device.apply_identity({"proto": "3", "axes": "1", "serial": "X", "fw": "9.9.9"})
    assert device.serial is None, "a refused board must not leave state behind"


def test_refuses_a_twin_axis_board(device):
    """The axis count distinguishes the boards; a product name must not."""
    with pytest.raises(AssertionError, match="2 axes"):
        device.apply_identity({"proto": "2", "axes": "2", "serial": "X", "fw": "0.0.0"})


# ------------------------------------------------------------ command flow


def test_sequence_numbers_increment_per_command(device):
    assert device.ping() == 1
    assert device.ping() == 2
    assert device.status() == 3


def test_sequence_numbers_wrap_at_the_protocol_modulus(device):
    device._seq = P.SEQ_MAX
    assert device.ping() == 0
    assert device.ping() == 1


def test_full_arm_and_move_sequence(device, link):
    """The path a task actually takes, end to end through the real codec."""
    device.hv(True)
    assert replies(device)[0][0] == P.KIND_ACK

    device.home()
    assert replies(device)[0][0] == P.KIND_ACK

    link.advance(link.sim.home_duration_us)
    homed = events(device)
    assert homed[0][1] == "HOMED"

    device.arm()
    assert replies(device)[0][0] == P.KIND_ACK

    device.move_to(9.0)
    assert replies(device)[0][0] == P.KIND_ACK

    link.advance(50_000)
    target = events(device)
    assert target[0][1] == "TARGET"
    assert target[0][2][1] == {"angle": "9.0"}


def test_the_trial_shape_end_to_end(device, link):
    """Park, present, release, pull, present again - through every real layer.

    This is the sequence the rig actually needs, as far as rev 0 hardware can
    go: no brake, so the pull is a hand-moved joystick against a released
    driver, and no analog streaming at all.
    """
    device.hv(True)
    device.home()
    link.advance(link.sim.home_duration_us)
    device.poll()

    device.arm()
    device.move_to(-9.0)  # present the joystick
    link.advance(50_000)
    assert events(device)[-1][1] == "TARGET"

    device.cfg(wdt_ms=500)
    device.brake(0)  # no brake on this board: release only
    device.poll()
    device.watch(3.0, 3.0, 300)  # pull/push 3 deg from the release angle
    device.poll()

    link._absorb(link.sim.move_joystick(-5.0))  # the animal pulls 4 deg
    pull = events(device)
    assert pull[0][1] == "PULL"
    assert pull[0][2][1] == {"angle": "-5.0", "torque": ""}

    # Presenting again must land where it was told, not offset by the pull.
    device.move_to(-9.0)
    for _ in range(6):
        link.advance(40_000)
        device.ping()
        device.poll()
    assert link.sim.angle == pytest.approx(-9.0)
    assert link.sim.xactual == pytest.approx(-9.0)


def test_brake_before_arming_is_refused(device):
    device.brake(0)
    kind, seq, reason = replies(device)[0]
    assert kind == P.KIND_NAK
    assert reason == P.NAK_NOTARMED


def test_move_before_arming_is_nakked_not_silently_dropped(device):
    device.move_to(5.0)
    kind, seq, reason = replies(device)[0]
    assert kind == P.KIND_NAK
    assert reason == P.NAK_NOTARMED


def test_the_capture_window_keys_go_through_cfg(device, link):
    """pre_ms/post_ms are ordinary CFG keys; the board bounds them, the class
    only encodes them."""
    device.cfg(pre_ms=200, post_ms=300)
    assert link.written[-1] == "cfg;1;pre_ms=200;post_ms=300\n"
    device.poll()
    device.cfg_get()
    kind, seq, data = replies(device)[-1]
    assert kind == P.KIND_ACK
    assert data["pre_ms"] == "200" and data["post_ms"] == "300"


def test_float_arguments_are_encoded_stably(device, link):
    """MicroPython and CPython do not agree on str(float); the codec must."""
    device.move_to(-12.5)
    assert link.written[-1] == "moveto;1;-12.5\n"
    device.watch(-10.0, 5, 300)
    assert link.written[-1] == "watch;2;pull=-10.0;push=5;timeout=300\n"
    device.move(0.0)
    assert link.written[-1] == "move;3;0.0\n"


def test_torque_verbs_are_gated_on_the_capability(device, link):
    """TQ exists only on a board whose caps say torque; the class refuses to
    send it elsewhere rather than let the board answer badverb."""
    with pytest.raises(AssertionError):
        device.torque_get()
    device.apply_identity({"proto": "2", "axes": "1", "serial": "X", "fw": "0.0.0",
                           "caps": "motion+home+instr+hv+watch+fetch+stream+cfg+brake+torque"})
    assert device.has("torque") and device.has("brake")
    device.torque_get()
    assert link.written[-1] == "tq;1;get\n"
    device.torque_zero()
    assert link.written[-1] == "tq;2;zero\n"
    device.torque_cal(True)
    assert link.written[-1] == "tq;3;cal;ON\n"
    device.torque_cal(False)
    assert link.written[-1] == "tq;4;cal;OFF\n"


# --------------------------------------------------------------- buffering


def test_poll_reassembles_a_reply_split_across_reads(link):
    """A real UART read returns whatever has arrived, not a whole line.

    Delivering one byte per read is the pathological case, and it is the one that
    shows whether the device class buffers or merely gets lucky.
    """
    teardown = stubs.install(link)
    try:
        from mouse_joystick import MouseJoystick

        MouseJoystick.registered_event_names.clear()
        device = MouseJoystick(port=stubs.FakePort(uart=4), event_name="joystick_solo")
        link.chunk_size = 1

        device.identify()
        collected = []
        for _ in range(200):
            collected += device.poll()
            if collected:
                break

        assert len(collected) == 1
        kind, seq, data = collected[0]
        assert kind == P.KIND_ACK
        assert data["serial"] == "0000000000000000"
        MouseJoystick.registered_event_names.clear()
    finally:
        teardown()


def test_poll_returns_nothing_when_the_board_is_silent(device):
    assert device.poll() == []


def test_events_and_replies_arrive_on_the_same_channel(device, link):
    """A host waiting for an ack has to cope with an event arriving alongside it.

    Provoking a fault at `arm` produces `ev;FAULT` and then `nak`, in that order,
    from a single command.
    """
    device.hv(True)
    device.home()
    link.advance(link.sim.home_duration_us)
    device.poll()

    link.sim.vsense_ok = False
    device.arm()
    messages = device.poll()

    kinds = [m[0] for m in messages]
    assert kinds == [P.KIND_EVENT, P.KIND_NAK]
    assert messages[0][1] == "FAULT"
    assert messages[0][2][1]["detail"] == "vsense_not_1"
    assert messages[1][2] == P.NAK_FAULTED
