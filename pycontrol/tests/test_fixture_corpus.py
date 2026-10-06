"""Check the simulator against the shared protocol corpus.

Gate C: "A host simulator or protocol fixture and the firmware agree on framing
and versions."

This half proves the simulator reproduces `fixtures/protocol_cases.txt`. The
other half is `test/native/test_protocol/test_protocol.cpp`, written 2026-09-03,
which holds the firmware's own `ProtocolBoard` to the same file. Both halves
exist, so the corpus is now evidence rather than a restatement of one
implementation.

Keep them in step: a case added here must be added for both, and mutation-testing
the firmware against this corpus is what found that the ARMED-before-range
ordering was documented but not enforced.

The parser here is deliberately trivial so the C++ side can be a direct
translation rather than a reinterpretation.
"""

from pathlib import Path

import pytest

from sim import BoardSim

CORPUS = Path(__file__).parent / "fixtures" / "protocol_cases.txt"

# Must match the corpus header's stated assumptions.
CORPUS_SERIAL = "0000000000000000"


class Case:
    def __init__(self, name, line_no):
        self.name = name
        self.line_no = line_no
        self.caps = set()  # CAPS: the hardware the board boots with
        self.steps = []  # (kind, payload) where kind is send | advance | expect

    def __repr__(self):
        return "<Case %s @%d>" % (self.name, self.line_no)


def parse_corpus(text):
    """Parse the corpus. Grammar is documented in the corpus file's header."""
    cases = []
    current = None

    for line_no, raw in enumerate(text.splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue

        keyword, _, rest = line.partition(" ")
        rest = rest.strip()

        if keyword == "CASE":
            assert current is None, "unterminated CASE before line %d" % line_no
            current = Case(rest, line_no)
        elif keyword == "END":
            assert current is not None, "END without CASE at line %d" % line_no
            cases.append(current)
            current = None
        else:
            assert current is not None, "%s outside a CASE at line %d" % (keyword, line_no)
            if keyword == "CAPS":
                current.caps |= set(rest.split())
            elif keyword == "SEND":
                # `\r` is written as an escape so the corpus stays one line per
                # step and survives an editor that strips trailing whitespace.
                current.steps.append(("send", rest.replace("\\r", "\r")))
            elif keyword == "ADVANCE":
                current.steps.append(("advance", int(rest)))
            elif keyword == "ANGLE":
                current.steps.append(("angle", float(rest)))
            elif keyword == "TORQUE":
                current.steps.append(("torque", float(rest)))
            elif keyword == "EXPECT":
                current.steps.append(("expect", rest))
            else:
                raise AssertionError("unknown keyword %r at line %d" % (keyword, line_no))

    assert current is None, "unterminated CASE %r" % (current,)
    return cases


CASES = parse_corpus(CORPUS.read_text(encoding="utf-8"))


def test_the_corpus_is_not_empty():
    """A corpus that silently parsed to nothing would make every case vacuous."""
    assert len(CASES) >= 15
    assert all(case.steps for case in CASES)


def test_every_case_has_a_unique_name():
    """The firmware reports failures by name, so duplicates would be ambiguous."""
    names = [case.name for case in CASES]
    assert len(names) == len(set(names))


def test_every_case_expects_something():
    """A case with no EXPECT asserts nothing and would pass against any board."""
    for case in CASES:
        kinds = {kind for kind, _ in case.steps}
        assert "expect" in kinds, "%s expects no output" % case.name


@pytest.mark.parametrize("case", CASES, ids=lambda c: c.name)
def test_simulator_reproduces_corpus_case(case):
    board = BoardSim(serial=CORPUS_SERIAL)
    # The hardware facts of the simulated revision, before boot, as the
    # firmware's test rig sets them from the same tokens.
    board.has_brake = "brake" in case.caps
    board.has_torque = "torque" in case.caps
    board.has_angout = "angout" in case.caps
    board.boot()

    pending = []
    for index, (kind, payload) in enumerate(case.steps):
        if kind == "send":
            pending += board.feed(payload + "\n")
        elif kind == "advance":
            pending += board.advance(payload)
        elif kind == "angle":
            # The animal moves the joystick; one service step at the current
            # time so the classifier sees the sample.
            board.set_encoder_angle(payload)
            pending += board.advance(0)
        elif kind == "torque":
            # The bridge reads this from now on; one service step at the current
            # time so the classifier sees the sample.
            board.set_torque(payload)
            pending += board.advance(0)
        elif kind == "expect":
            assert pending, "%s step %d: expected %r but no output was pending" % (
                case.name,
                index,
                payload,
            )
            actual = pending.pop(0)
            assert actual == payload, "%s step %d" % (case.name, index)

    assert not pending, "%s produced unexpected trailing output: %r" % (case.name, pending)
