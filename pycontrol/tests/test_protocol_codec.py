"""Codec tests: framing only, no state machine.

Every assertion here is traceable to a line of
`../../../mouse-joystick-pcb/docs/uart-protocol.md`. Where the draft gives a
concrete example, the example is the test.
"""

import pytest


# The five concrete wire examples in the draft's "Framing" section. If the codec
# cannot reproduce these byte for byte it does not implement the draft, whatever
# else it does.
DRAFT_EXAMPLES = [
    ("moveto", 417, ["-12.5"], "moveto;417;-12.5"),
    ("home", 418, [], "home;418"),
    ("cal", 419, ["get"], "cal;419;get"),
    ("moveto", 421, ["-999"], "moveto;421;-999"),
]


@pytest.mark.parametrize("verb,seq,args,expected", DRAFT_EXAMPLES)
def test_encodes_the_drafts_own_examples(protocol, verb, seq, args, expected):
    assert protocol.encode_command(verb, seq, args) == expected + "\n"


def test_ack_with_data_matches_the_draft_example(protocol):
    # `cal;419;get -> ack;419;enc_offset=1024;angle_zero=0.0`
    line = protocol.encode_ack(419, {"enc_offset": 1024, "angle_zero": 0.0})
    assert line == "ack;419;enc_offset=1024;angle_zero=0.0\n"


def test_nak_matches_the_draft_examples(protocol):
    assert protocol.encode_nak(420, protocol.NAK_BADVERB) == "nak;420;badverb\n"
    assert protocol.encode_nak(421, protocol.NAK_OUTOFRANGE) == "nak;421;outofrange\n"


def test_event_matches_the_draft_examples(protocol):
    assert protocol.encode_event("PULL", 18420102, {"angle": -3.1}) == "ev;PULL;18420102;angle=-3.1\n"
    assert (
        protocol.encode_event("FAULT", 18599112, {"code": 2, "detail": "over_travel"})
        == "ev;FAULT;18599112;code=2;detail=over_travel\n"
    )


def test_number_formatting_is_stable_across_implementations(protocol):
    """str(float) is not guaranteed to agree between CPython and MicroPython.

    The fixture corpus compares exact bytes, so this rendering is load-bearing
    rather than cosmetic.
    """
    assert protocol.format_number(1024) == "1024"
    assert protocol.format_number(0.0) == "0.0"
    assert protocol.format_number(-3.1) == "-3.1"
    assert protocol.format_number(-12.5) == "-12.5"
    assert protocol.format_number(-0.0) == "0.0"
    assert protocol.format_number(True) == "1"
    assert protocol.format_number(False) == "0"


def test_verbs_decode_case_insensitively(protocol):
    """The draft's table is uppercase, its examples lowercase. Accept both.

    Emitting is lowercase, matching the examples, which are the wire format.
    """
    for text in ("MOVETO;417;-12.5", "moveto;417;-12.5", "MoveTo;417;-12.5"):
        kind, verb, seq, args = protocol.decode_command(text)
        assert (kind, verb, seq, args) == (protocol.KIND_COMMAND, "moveto", 417, ["-12.5"])


def test_trailing_carriage_return_is_tolerated(protocol):
    """The draft says a leading \\r is ignored; a CRLF peer produces a trailing one."""
    kind, verb, seq, _ = protocol.decode_command("ping;7\r\n")
    assert (kind, verb, seq) == (protocol.KIND_COMMAND, "ping", 7)


def test_sequence_numbers_wrap_at_65535(protocol):
    assert protocol.normalize_seq(65535) == 65535
    assert protocol.normalize_seq(65536) == 0
    assert protocol.normalize_seq(65537) == 1
    assert protocol.SEQ_MAX == 65535


def test_over_length_line_is_a_bad_frame(protocol):
    kind, _, _, reason = protocol.decode_command("ping;1;" + "x" * protocol.MAX_LINE)
    assert kind == protocol.KIND_BAD
    assert reason == protocol.NAK_BADFRAME


def test_bad_frame_keeps_the_sequence_number_when_it_is_readable(protocol):
    """`nak;<seq or ?>` should only say `?` when the seq genuinely was not readable."""
    kind, verb, seq, _ = protocol.decode_command("ping;notanumber")
    assert kind == protocol.KIND_BAD
    assert verb == "ping"
    assert seq is None  # unreadable here, so `?` is correct

    kind, verb, seq, args = protocol.decode_command("ping;42")
    assert seq == 42


@pytest.mark.parametrize("line", ["", "\n", ";", ";;", "ping", "\r\n"])
def test_structurally_broken_lines_are_rejected(protocol, line):
    kind = protocol.decode_command(line)[0]
    assert kind == protocol.KIND_BAD


def test_arguments_may_not_smuggle_a_delimiter(protocol):
    """No embedded `;` or newline in values, so encoding one is a programming error."""
    with pytest.raises(ValueError):
        protocol.encode_command("cal", 1, ["set;angle_zero;0"])
    with pytest.raises(ValueError):
        protocol.encode_command("cal", 1, ["a\nb"])


def test_replies_and_events_are_distinguishable_on_one_channel(protocol):
    """A host waiting for an ack must cope with an event arriving first."""
    assert protocol.decode_reply("ack;417")[0] == protocol.KIND_ACK
    assert protocol.decode_reply("nak;420;badverb")[0] == protocol.KIND_NAK
    assert protocol.decode_reply("ev;PULL;18420102;angle=-3.1")[0] == protocol.KIND_EVENT


def test_decode_reply_round_trips_data(protocol):
    kind, seq, data = protocol.decode_reply("ack;419;enc_offset=1024;angle_zero=0.0")
    assert (kind, seq) == (protocol.KIND_ACK, 419)
    assert data == {"enc_offset": "1024", "angle_zero": "0.0"}

    kind, name, payload = protocol.decode_reply("ev;FAULT;18599112;code=2;detail=over_travel")
    t_us, data = payload
    assert (kind, name, t_us) == (protocol.KIND_EVENT, "FAULT", 18599112)
    assert data == {"code": "2", "detail": "over_travel"}


def test_ack_data_without_an_equals_sign_is_a_bad_frame(protocol):
    assert protocol.decode_reply("ack;419;enc_offset")[0] == protocol.KIND_BAD


def test_nak_for_an_unknown_sequence_number_is_legal(protocol):
    """`nak;?;badframe` carries no seq, and that is not itself an error."""
    kind, seq, reason = protocol.decode_reply("nak;?;badframe")
    assert (kind, seq, reason) == (protocol.KIND_NAK, None, "badframe")


def test_protocol_version_and_axis_count_are_declared(protocol):
    """Both are reported by ID, and neither may be inferred by a peer."""
    assert protocol.PROTOCOL_VERSION == 2  # v1 proposal of 2026-10-01
    assert protocol.AXIS_COUNT == 1
