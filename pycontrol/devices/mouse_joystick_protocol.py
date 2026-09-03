"""Framing codec for the mouse-joystick UART protocol.

Specification: `../../../mouse-joystick-pcb/docs/uart-protocol.md` (Draft v0.1).
That document owns the protocol. This file implements it and must not extend it
silently - every place this file had to decide something the draft left open is
marked `SPEC:` below and listed in `../README.md`.

Runs in two places, so it is written to the intersection of MicroPython and
CPython: no `typing`, no `re`, no f-strings in hot paths, no comprehension
cleverness, and tuples rather than dataclasses.

- On the pyControl board, imported by `mouse_joystick.py`.
- On a host, imported directly by `../sim/board_sim.py` and by the tests. It has
  no pyControl imports, which is what makes it testable under plain pytest.

WHY THIS IS A CLASS AND NOT A MODULE OF FUNCTIONS:
pyControl decides which device files to upload by scanning for *class* names -
`pycboard._get_used_device_files` keeps a file only if some class it defines
appears as a substring of the referencing file, using the map built by
`make_device_class2file_map`. A module of plain functions defines no class,
never enters that map, and is therefore never transferred to the pyboard. The
device class would then fail at import on the board while working perfectly on
the host. Keeping the codec inside a class named here and referenced by name in
`mouse_joystick.py` is what makes the upload happen.

Everything also lives inside the class namespace because the board's
`devices/__init__.py` does `from devices.<file> import *` for every file in the
folder. Module-level names like `IDLE` or `MAX_LINE` would leak into every
device driver's namespace.
"""


class MouseJoystickProtocol:
    """Encode and decode one line of the mouse-joystick protocol.

    All methods are static. This class is a namespace and a marker for
    pyControl's uploader, not an object to instantiate.
    """

    # Bumped when a change would make an older peer misbehave rather than fail.
    # `ID` reports this, and the device class refuses a mismatched major version
    # rather than guessing - see the draft's "Versioning" section.
    PROTOCOL_VERSION = 1

    # This board is single-axis. The twin-axis board carries an axis field after
    # the verb; a peer distinguishes the two by this number, never by a product
    # name. Do not default it, do not infer it.
    AXIS_COUNT = 1

    # SPEC: the draft does not state a maximum line length. It says only to size
    # the receive buffer "at least as large as the longest command plus margin"
    # and notes the syringe pump host uses rxbuf=130. 120 leaves the device room
    # to answer `nak;<seq>;badframe` for an over-length line inside a 130-byte
    # buffer instead of overflowing while trying to complain.
    MAX_LINE = 120

    # `seq` is decimal and wraps at 65535, so the valid range is 0..65535 and
    # 65535 + 1 is 0.
    SEQ_MODULUS = 65536
    SEQ_MAX = SEQ_MODULUS - 1

    DELIMITER = ";"
    TERMINATOR = "\n"

    # Reply and event discriminators, which occupy field 0.
    ACK = "ack"
    NAK = "nak"
    EVENT = "ev"

    # Parse-result kinds. Returned as the first element of every decode tuple so
    # a caller can dispatch without exceptions - MicroPython exception handling
    # in an interrupt-adjacent path is worth avoiding.
    KIND_ACK = "ack"
    KIND_NAK = "nak"
    KIND_EVENT = "event"
    KIND_COMMAND = "command"
    KIND_BAD = "bad"

    # nak reasons. `badverb`, `outofrange`, `badframe` and `latched` are named in
    # the draft.
    # SPEC: the remainder are needed to reject the state-machine and argument
    # errors the draft requires ("Motion verbs are refused outside ARMED",
    # "refused if not homed or if faulted") but for which it names no reason
    # string. They are additions to the draft, not readings of it.
    NAK_BADFRAME = "badframe"
    NAK_BADVERB = "badverb"
    NAK_OUTOFRANGE = "outofrange"
    NAK_LATCHED = "latched"
    NAK_BADARGS = "badargs"
    NAK_NOTARMED = "notarmed"
    NAK_NOTHOMED = "nothomed"
    NAK_FAULTED = "faulted"
    NAK_BUSY = "busy"
    # The motor rail is genuinely off at boot - the 36 V input block's ENABLE is
    # pulled low - so refusing motion for want of it is a distinct, common case
    # and deserves its own reason rather than being folded into `notarmed`.
    NAK_NOHV = "nohv"

    # Used where a malformed line did not yield a usable sequence number. The
    # draft writes this position as `<seq or ?>`.
    SEQ_UNKNOWN = "?"

    # ------------------------------------------- retransmission classification
    #
    # Which commands may have their reply served from the retransmission cache.
    # Resolved in `../../../mouse-joystick-pcb/docs/uart-protocol.md`, open
    # question 1: the cache is restricted to state-changing commands, because
    # re-executing a read is better than replaying one -- a retried STATUS
    # returning fresh state beats a stale one, and a retried CLRFAULT
    # re-evaluates whether the cause has gone where a cached "cleared" could
    # mask a fault latched since.
    #
    # Note the cache is NOT what keeps the heartbeat alive. The host-loss
    # watchdog is reset on receipt of any well-formed command, before dispatch,
    # so a cache hit is irrelevant to it.

    # Reads and heartbeats. Never cached; always re-executed.
    READ_VERBS = frozenset({"id", "ping", "status", "limits", "faults"})

    # Verbs that are a read or a write depending on their first argument, which
    # is why classification must happen on the PARSED command and not on the
    # verb name. A verb-name table gets all three of these wrong.
    DUAL_MODE_READ_ARG = {"cal": "get", "instr": "state", "hv": "state"}

    @classmethod
    def changes_state(cls, verb, args=()):
        """Should a repeat of this command be served from the cache?

        Defaults to True for anything unrecognised. Over-caching costs a stale
        ack; under-caching moves the joystick twice, so a verb added carelessly
        should be merely suboptimal rather than dangerous.
        """
        verb = verb.lower()

        read_arg = cls.DUAL_MODE_READ_ARG.get(verb)
        if read_arg is not None:
            return not (args and str(args[0]).lower() == read_arg)

        return verb not in cls.READ_VERBS

    # ---------------------------------------------------------------- helpers

    @staticmethod
    def format_number(value):
        """Render a number identically on CPython and MicroPython.

        `str(float)` is not guaranteed to agree between the two, and this
        protocol is compared byte-for-byte against a fixture corpus that the
        firmware also has to reproduce. Fixing the rendering here is what lets
        that comparison mean anything.

        Integers stay bare (`enc_offset=1024`); floats keep at least one decimal
        place and drop trailing zeros (`angle_zero=0.0`, `angle=-3.1`), matching
        the draft's own examples.
        """
        if isinstance(value, bool):  # bool is an int subclass; check it first.
            return "1" if value else "0"
        if isinstance(value, int):
            return str(value)

        text = "%.3f" % value
        if "." in text:
            text = text.rstrip("0")
            if text.endswith("."):
                text += "0"
        # "-0.0" is a real float but a confusing wire value.
        if text == "-0.0":
            text = "0.0"
        return text

    @staticmethod
    def normalize_seq(seq):
        """Wrap a sequence number into 0..65535, or None if it is not usable."""
        try:
            return int(seq) % MouseJoystickProtocol.SEQ_MODULUS
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _split(line):
        """Strip the terminator and split into fields, or None if unusable.

        SPEC: the draft says "A leading `\\r` is tolerated and ignored". A `\\r`
        produced by a peer using CRLF actually arrives *trailing*, before the
        `\\n`, so it is stripped from both ends here. Reading the draft
        literally would leave the common case broken.
        """
        if line is None:
            return None
        text = line.strip("\r\n").strip("\r")
        if not text:
            return None
        if len(text) > MouseJoystickProtocol.MAX_LINE:
            return None
        return text.split(MouseJoystickProtocol.DELIMITER)

    @staticmethod
    def _parse_kv(fields):
        """Parse `key=value` fields. Returns (dict, ok).

        Values stay strings. Interpreting `angle=-3.1` as a float is the
        caller's business, because only the caller knows which keys are numbers.
        """
        data = {}
        for field in fields:
            if "=" not in field:
                return data, False
            key, _, value = field.partition("=")
            if not key:
                return data, False
            data[key] = value
        return data, True

    # --------------------------------------------------------------- encoding

    @staticmethod
    def encode_command(verb, seq, args=None):
        """Build `VERB;seq[;arg...]` with its terminator.

        SPEC: the draft's command table spells verbs uppercase (`MOVETO`) while
        every concrete example on the wire is lowercase (`moveto;417;-12.5`).
        The examples are the wire format, so lowercase is emitted; decoding
        accepts either. Flagged in `../README.md` as a draft inconsistency to
        resolve before the protocol is frozen.
        """
        normalized = MouseJoystickProtocol.normalize_seq(seq)
        if normalized is None:
            raise ValueError("seq must be an integer")

        parts = [str(verb).lower(), str(normalized)]
        if args:
            for arg in args:
                if isinstance(arg, str):
                    text = arg
                else:
                    text = MouseJoystickProtocol.format_number(arg)
                if MouseJoystickProtocol.DELIMITER in text or "\n" in text:
                    raise ValueError("argument may not contain a delimiter or newline")
                parts.append(text)

        line = MouseJoystickProtocol.DELIMITER.join(parts)
        if len(line) > MouseJoystickProtocol.MAX_LINE:
            raise ValueError("command exceeds MAX_LINE")
        return line + MouseJoystickProtocol.TERMINATOR

    @staticmethod
    def encode_ack(seq, data=None):
        """Build `ack;seq[;key=value...]`.

        SPEC: the draft describes `ID` and `STATUS` replies as prose lists of
        fields and shows only one concrete data-carrying ack,
        `ack;419;enc_offset=1024;angle_zero=0.0`. That one is `key=value`, so
        every data-carrying ack here is `key=value`. Positional reply fields
        would be shorter but would make adding a field a breaking change.
        """
        parts = [MouseJoystickProtocol.ACK, str(seq)]
        if data:
            for key in data:
                value = data[key]
                if isinstance(value, str):
                    text = value
                else:
                    text = MouseJoystickProtocol.format_number(value)
                parts.append(key + "=" + text)
        return MouseJoystickProtocol.DELIMITER.join(parts) + MouseJoystickProtocol.TERMINATOR

    @staticmethod
    def encode_nak(seq, reason):
        """Build `nak;seq;reason`. `seq` may be SEQ_UNKNOWN for a bad frame."""
        parts = [MouseJoystickProtocol.NAK, str(seq), reason]
        return MouseJoystickProtocol.DELIMITER.join(parts) + MouseJoystickProtocol.TERMINATOR

    @staticmethod
    def encode_event(name, t_us, data=None):
        """Build `ev;EVENT;t_us[;key=value...]`.

        Event names are uppercase on the wire in every example the draft gives.
        """
        parts = [MouseJoystickProtocol.EVENT, str(name).upper(), str(int(t_us))]
        if data:
            for key in data:
                value = data[key]
                if isinstance(value, str):
                    text = value
                else:
                    text = MouseJoystickProtocol.format_number(value)
                parts.append(key + "=" + text)
        return MouseJoystickProtocol.DELIMITER.join(parts) + MouseJoystickProtocol.TERMINATOR

    # --------------------------------------------------------------- decoding

    @staticmethod
    def decode_command(line):
        """Parse a host->board line.

        Returns one of:
            (KIND_COMMAND, verb, seq, [args])
            (KIND_BAD, None, seq_or_None, reason)

        A bad frame still reports a sequence number when one could be recovered,
        because the draft requires `nak;<seq or ?>;badframe` - answering `?` when
        the seq was perfectly readable would cost the host its correlation.
        """
        fields = MouseJoystickProtocol._split(line)
        if not fields:
            return (MouseJoystickProtocol.KIND_BAD, None, None, MouseJoystickProtocol.NAK_BADFRAME)

        verb = fields[0].lower()
        if not verb:
            return (MouseJoystickProtocol.KIND_BAD, None, None, MouseJoystickProtocol.NAK_BADFRAME)

        if len(fields) < 2:
            return (MouseJoystickProtocol.KIND_BAD, verb, None, MouseJoystickProtocol.NAK_BADFRAME)

        seq = MouseJoystickProtocol.normalize_seq(fields[1])
        if seq is None:
            return (MouseJoystickProtocol.KIND_BAD, verb, None, MouseJoystickProtocol.NAK_BADFRAME)

        return (MouseJoystickProtocol.KIND_COMMAND, verb, seq, fields[2:])

    @staticmethod
    def decode_reply(line):
        """Parse a board->host line: an ack, a nak, or an asynchronous event.

        Returns one of:
            (KIND_ACK,   seq,  data_dict)
            (KIND_NAK,   seq,  reason)
            (KIND_EVENT, name, (t_us, data_dict))
            (KIND_BAD,   None, reason)

        Events share this channel with replies, so a host that is waiting for an
        ack must be prepared to see an event first. That is the whole reason
        events carry their own discriminator in field 0.
        """
        fields = MouseJoystickProtocol._split(line)
        if not fields:
            return (MouseJoystickProtocol.KIND_BAD, None, MouseJoystickProtocol.NAK_BADFRAME)

        head = fields[0]

        if head == MouseJoystickProtocol.EVENT:
            if len(fields) < 3:
                return (MouseJoystickProtocol.KIND_BAD, None, MouseJoystickProtocol.NAK_BADFRAME)
            name = fields[1]
            try:
                t_us = int(fields[2])
            except ValueError:
                return (MouseJoystickProtocol.KIND_BAD, None, MouseJoystickProtocol.NAK_BADFRAME)
            data, ok = MouseJoystickProtocol._parse_kv(fields[3:])
            if not ok:
                return (MouseJoystickProtocol.KIND_BAD, None, MouseJoystickProtocol.NAK_BADFRAME)
            return (MouseJoystickProtocol.KIND_EVENT, name, (t_us, data))

        if head == MouseJoystickProtocol.ACK:
            if len(fields) < 2:
                return (MouseJoystickProtocol.KIND_BAD, None, MouseJoystickProtocol.NAK_BADFRAME)
            seq = MouseJoystickProtocol.normalize_seq(fields[1])
            if seq is None:
                return (MouseJoystickProtocol.KIND_BAD, None, MouseJoystickProtocol.NAK_BADFRAME)
            data, ok = MouseJoystickProtocol._parse_kv(fields[2:])
            if not ok:
                return (MouseJoystickProtocol.KIND_BAD, None, MouseJoystickProtocol.NAK_BADFRAME)
            return (MouseJoystickProtocol.KIND_ACK, seq, data)

        if head == MouseJoystickProtocol.NAK:
            if len(fields) < 3:
                return (MouseJoystickProtocol.KIND_BAD, None, MouseJoystickProtocol.NAK_BADFRAME)
            seq = MouseJoystickProtocol.normalize_seq(fields[1])
            # `nak;?;badframe` is legal and carries no seq, which is not an error.
            return (MouseJoystickProtocol.KIND_NAK, seq, fields[2])

        return (MouseJoystickProtocol.KIND_BAD, None, MouseJoystickProtocol.NAK_BADFRAME)
