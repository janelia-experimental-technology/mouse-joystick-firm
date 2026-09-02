"""A host-side simulator of the mouse-joystick board's UART protocol.

`../../../mouse-joystick-pcb/docs/uart-protocol.md` asks for exactly this:

    Provide a host-side simulator or protocol fixture so both this firmware and
    the pyControl device class can be tested before real hardware and a real
    pyControl installation are both available.

It implements the draft's state machine, command set, fault model and
retransmission rule. It does NOT simulate the board: there is no TMC5130A model,
no encoder, no current control and no real motion profile. Position moves
linearly at a configured velocity because the protocol is what is under test, and
a plausible-looking motion model would only invite someone to trust it.

**This simulator is not evidence about the firmware.** It and the firmware are
two independent readings of the same draft, and two readings agree right up until
they do not. What binds them is `../tests/fixtures/protocol_cases.json`, a corpus
this simulator is checked against and which the firmware's own host tests must
also reproduce. Gate C is passed by both sides matching the corpus, not by either
side matching itself.

Time is injected rather than read from a clock, so every event timestamp is
deterministic and the corpus can contain exact expected bytes.

Usage:

    sim = BoardSim(serial="E4654C159F4D1334")
    out = sim.boot()                 # ['ev;READY;0;fw=...;proto=1;serial=...']
    out = sim.feed("id;1\\n")         # ['ack;1;hw=rev0;serial=...;...']
    out = sim.advance(200_000)       # time-driven events: watchdog, completions
"""

import sys
from pathlib import Path

# The codec is shared with the device class that runs on the pyControl board, so
# it lives in devices/ where pyControl can upload it. Import it from there rather
# than keeping a second copy that could drift.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "devices"))

from mouse_joystick_protocol import MouseJoystickProtocol as P  # noqa: E402


class Fault:
    """Fault bits from the draft's fault-code table.

    Bit 4 is absent on purpose. `ENCODER_MISMATCH` was retired 2026-08-16 - the
    encoder reaches the driver only, so there is no second count to disagree with
    it - and the draft says to leave the bit permanently reserved so no later
    fault reuses it. Defining it here, even as "retired", is how it gets reused.
    """

    HOST_TIMEOUT = 0
    DRIVER_FAULT = 1
    OVER_TRAVEL = 2
    HOME_TIMEOUT = 3
    # 4: reserved forever. Do not define.
    LIMIT_UNEXPECTED = 5
    SUPPLY_OUT_OF_RANGE = 6
    INTERNAL = 7
    EXT_STOP = 8

    RESERVED_FOREVER = 4

    NAMES = {
        0: "host_timeout",
        1: "driver_fault",
        2: "over_travel",
        3: "home_timeout",
        5: "limit_unexpected",
        6: "supply_out_of_range",
        7: "internal",
        8: "ext_stop",
    }


class State:
    BOOT = "BOOT"
    IDLE = "IDLE"
    HOMED = "HOMED"
    ARMED = "ARMED"
    ACTIVE = "ACTIVE"
    FAULTED = "FAULTED"


class BoardSim:
    """One board on one link. Two instances share nothing."""

    # SPEC: the draft does not bound the retransmission cache. Sixteen entries
    # covers any plausible in-flight window on a 115200 link with a 100 ms host
    # timeout, and bounding it matters because an unbounded cache on the real
    # firmware is a slow memory leak on a device with no way to report one.
    RETRANSMIT_CACHE_SIZE = 16

    # Cited from `../../../mouse-joystick-pcb/docs/pycontrol-port-pinout.md`.
    DEFAULT_HOST_TIMEOUT_US = 100_000

    # The analog output's usable window. Both ends are hardware limits, not
    # choices: the DAC cannot exceed its own 2.5 V reference through the gain
    # stage, and the output amplifier sits ~25 mV off ground at light load, so
    # 0 V stays out-of-band and means unhomed, disarmed, faulted or unpowered.
    ANGOUT_V_MIN = 0.05
    ANGOUT_V_MAX = 3.300

    def __init__(
        self,
        serial="E4654C159F4D1334",
        fw_version="0.0.0",
        hw_rev="rev0",
        travel_deg=(-20.0, 20.0),
        v_max_deg_s=180.0,
        a_max_deg_s2=3600.0,
        i_max_a=0.421,
        host_timeout_us=DEFAULT_HOST_TIMEOUT_US,
        home_duration_us=50_000,
    ):
        self.serial = serial
        self.fw_version = fw_version
        self.hw_rev = hw_rev
        self.travel_lo, self.travel_hi = travel_deg
        self.v_max = v_max_deg_s
        self.a_max = a_max_deg_s2
        self.i_max = i_max_a
        self.host_timeout_us = host_timeout_us
        self.home_duration_us = home_duration_us

        self.now_us = 0
        self.state = State.BOOT
        self.angle = 0.0
        self.homed = False
        self.limit_asserted = False
        self.fault_mask = 0
        self.fault_detail = {}
        self._fault_cause_active = {}

        # Boot leaves the motor rail off: the 36 V input block's ENABLE is pulled
        # low and the rail is genuinely off, which is why HV is an observable
        # state rather than something the host infers.
        self.hv_on = False
        self.instr_on = False
        self.angout_on = False
        self.angout_lo = self.travel_lo
        self.angout_hi = self.travel_hi
        self.pull_threshold = None

        self.cal = {"enc_offset": 1024, "angle_zero": 0.0}

        # Set False to make the ARM-time vsense read-back fail, which is how
        # Gate C's "including the `vsense` self-test path in bit 7" is provoked.
        self.vsense_ok = True

        self._rx = ""
        self._retransmit = {}
        self._retransmit_order = []
        self._watchdog_deadline = None
        self._motion = None
        self._pending = []

    # ------------------------------------------------------------------ clock

    def boot(self):
        """Emit `READY`. The draft: once after boot, host should reconnect and re-ID."""
        self.state = State.IDLE
        self._emit(
            "READY",
            {"fw": self.fw_version, "proto": P.PROTOCOL_VERSION, "serial": self.serial},
        )
        return self._drain()

    def advance(self, delta_us):
        """Advance simulated time, running anything that is time-driven."""
        target = self.now_us + int(delta_us)
        while self.now_us < target:
            # Step to the next scheduled thing, so an event never lands at the
            # wrong timestamp just because the caller advanced in one big jump.
            next_stop = target
            if self._motion is not None and self._motion["end_us"] < next_stop:
                next_stop = self._motion["end_us"]
            if self._watchdog_deadline is not None and self._watchdog_deadline < next_stop:
                next_stop = self._watchdog_deadline

            self.now_us = next_stop
            self._service()

            if next_stop == target:
                break
        self.now_us = target
        self._service()
        return self._drain()

    def _service(self):
        if self._motion is not None and self.now_us >= self._motion["end_us"]:
            motion = self._motion
            self._motion = None
            self.angle = motion["target"]
            if motion["kind"] == "home":
                self.homed = True
                self.angle = 0.0
                self.state = State.HOMED
                self._emit("HOMED", {"angle": self.angle})
            else:
                self.state = State.ARMED
                self._emit("TARGET", {"angle": self.angle})

        if self._watchdog_deadline is not None and self.now_us >= self._watchdog_deadline:
            self._watchdog_deadline = None
            self._raise_fault(Fault.HOST_TIMEOUT, "host_timeout", cause_active=False)

    # ------------------------------------------------------------------- link

    def feed(self, data):
        """Accept received bytes, process complete lines, return output lines.

        Partial lines are buffered, because a real link delivers them that way
        and a parser that only works on whole lines is a parser that has not been
        tested.
        """
        if isinstance(data, (bytes, bytearray)):
            data = data.decode("ascii", "replace")
        self._rx += data

        while "\n" in self._rx:
            line, _, self._rx = self._rx.partition("\n")
            self._handle_line(line)

        # An over-length fragment with no terminator would otherwise grow without
        # bound waiting for one. Answer it and resynchronise at the next newline.
        if len(self._rx) > P.MAX_LINE:
            self._rx = ""
            self._out(P.encode_nak(P.SEQ_UNKNOWN, P.NAK_BADFRAME))

        return self._drain()

    def _handle_line(self, line):
        kind, verb, seq, rest = P.decode_command(line)

        if kind == P.KIND_BAD:
            self._out(P.encode_nak(seq if seq is not None else P.SEQ_UNKNOWN, rest))
            return

        canonical = P.DELIMITER.join([verb, str(seq)] + list(rest))
        cached = self._retransmit.get(seq)
        if cached is not None and cached[0] == canonical:
            # "A repeated `seq` with identical content is treated as a
            # retransmission and re-acknowledged without re-executing."
            #
            # SPEC: that rule and `PING` contradict each other, and the draft
            # does not notice. The rule exists so a retried `MOVETO` does not
            # move the joystick twice - it assumes re-execution is the harm. For
            # `PING` the side effect *is* the command: it has no result other
            # than resetting the host-loss watchdog. Serving a repeated `PING`
            # purely from cache means the safety timeout expires while the host
            # believes it is being answered, and the board faults mid-experiment
            # for no reason the host can see.
            #
            # So the watchdog is refreshed even on a cache hit. A host normally
            # increments `seq`, so this only bites after a 65536 wrap or on a
            # host that reuses numbers - but "only after a wrap" is not a safety
            # argument. Raised in `../README.md` for the draft to resolve
            # properly, most likely by exempting `PING` from the cache entirely.
            if verb == "ping" and self._watchdog_deadline is not None:
                self._watchdog_deadline = self.now_us + self.host_timeout_us
            self._out(cached[1])
            return

        reply = self._dispatch(verb, seq, rest)
        if reply is not None:
            self._remember(seq, canonical, reply)
            self._out(reply)

    def _remember(self, seq, canonical, reply):
        if seq in self._retransmit:
            self._retransmit_order.remove(seq)
        elif len(self._retransmit_order) >= self.RETRANSMIT_CACHE_SIZE:
            self._retransmit.pop(self._retransmit_order.pop(0), None)
        self._retransmit[seq] = (canonical, reply)
        self._retransmit_order.append(seq)

    def _out(self, line):
        self._pending.append(line.rstrip("\n"))

    def _emit(self, name, data=None):
        self._out(P.encode_event(name, self.now_us, data))

    def _drain(self):
        out, self._pending = self._pending, []
        return out

    # -------------------------------------------------------------- dispatch

    def _dispatch(self, verb, seq, args):
        handler = getattr(self, "_cmd_" + verb, None)
        if handler is None:
            # "An unrecognized verb is rejected, never partially executed."
            return P.encode_nak(seq, P.NAK_BADVERB)
        return handler(seq, args)

    # `ID` "must work before ARM and must never fail" - so it is answered in
    # every state including FAULTED, and takes no arguments it could reject.
    def _cmd_id(self, seq, args):
        return P.encode_ack(
            seq,
            {
                "hw": self.hw_rev,
                "serial": self.serial,
                "fw": self.fw_version,
                "proto": P.PROTOCOL_VERSION,
                # The axis count is what distinguishes this board from a
                # twin-axis board on the same link. Never inferred from a name.
                "axes": P.AXIS_COUNT,
                "caps": self._capabilities(),
                "rxbuf": P.MAX_LINE,
            },
        )

    def _capabilities(self):
        # SPEC: the draft says `ID` reports "capabilities" without saying how.
        # A `+`-joined token list avoids the delimiter and stays greppable in a
        # logic-analyzer capture, which is the draft's stated design rule.
        caps = ["motion", "home", "angout", "instr", "hv"]
        return "+".join(caps)

    def _cmd_ping(self, seq, args):
        # The heartbeat. Only meaningful while the watchdog is armed, but always
        # answered: a host has to be able to probe a link it is not yet armed on.
        if self._watchdog_deadline is not None:
            self._watchdog_deadline = self.now_us + self.host_timeout_us
        return P.encode_ack(seq)

    def _cmd_status(self, seq, args):
        # "Safe at any time", so no state check.
        return P.encode_ack(
            seq,
            {
                "state": self.state,
                "angle": self.angle,
                "pos": int(round(self.angle * 100)),
                "homed": self.homed,
                "limit": self.limit_asserted,
                "faults": self.fault_mask,
                "hv": self.hv_on,
                "angout": self._angout_state(),
            },
        )

    def _angout_state(self):
        """Report the analog output as invalid whenever it reads 0 V.

        `ANGOUT OFF`, a fault, and a dead switched rail all produce the same 0 V.
        The draft calls that intentional - one out-of-band level, not three - but
        it means the recorded analog trace cannot say which, so STATUS must.
        """
        if not self.angout_on or self.fault_mask or not self.hv_on or not self.homed:
            return "invalid"
        return "valid"

    def _cmd_limits(self, seq, args):
        return P.encode_ack(
            seq,
            {
                "travel_lo": self.travel_lo,
                "travel_hi": self.travel_hi,
                "v_max": self.v_max,
                "a_max": self.a_max,
                "i_max": self.i_max,
            },
        )

    def _cmd_faults(self, seq, args):
        # "Readable while faulted" - which is the only time it is interesting.
        data = {"mask": self.fault_mask}
        for bit in sorted(self.fault_detail):
            data[Fault.NAMES.get(bit, "bit%d" % bit)] = self.fault_detail[bit]
        return P.encode_ack(seq, data)

    def _cmd_clrfault(self, seq, args):
        if not self.fault_mask:
            return P.encode_ack(seq)

        for bit in list(self.fault_detail):
            if not self._fault_cause_active.get(bit, False):
                self.fault_mask &= ~(1 << bit)
                self.fault_detail.pop(bit, None)
                self._fault_cause_active.pop(bit, None)

        if self.fault_mask:
            # Some cause has not gone away. FAULTED is exited only through
            # CLRFAULT, and only for faults whose cause has cleared.
            return P.encode_nak(seq, P.NAK_LATCHED)

        self.state = State.IDLE
        self.homed = False
        return P.encode_ack(seq)

    def _cmd_hv(self, seq, args):
        if len(args) != 1:
            return P.encode_nak(seq, P.NAK_BADARGS)
        action = args[0].upper()
        if action == "STATE":
            return P.encode_ack(seq, {"hv": self.hv_on})
        if action == "ON":
            if self.state == State.FAULTED:
                return P.encode_nak(seq, P.NAK_FAULTED)
            self.hv_on = True
            return P.encode_ack(seq)
        if action == "OFF":
            # Always permitted: the board never requires a message in order to
            # become safe, and this is the message that makes it safer.
            self.hv_on = False
            if self.state in (State.ARMED, State.ACTIVE):
                self._motion = None
                self.state = State.HOMED
                self._watchdog_deadline = None
            return P.encode_ack(seq)
        return P.encode_nak(seq, P.NAK_BADARGS)

    def _cmd_instr(self, seq, args):
        if len(args) != 1:
            return P.encode_nak(seq, P.NAK_BADARGS)
        action = args[0].upper()
        if action == "STATE":
            return P.encode_ack(seq, {"instr": self.instr_on})
        if action in ("ON", "OFF"):
            self.instr_on = action == "ON"
            return P.encode_ack(seq)
        return P.encode_nak(seq, P.NAK_BADARGS)

    def _cmd_home(self, seq, args):
        if self.state == State.FAULTED:
            return P.encode_nak(seq, P.NAK_FAULTED)
        if not self.hv_on:
            return P.encode_nak(seq, P.NAK_NOHV)
        if self._motion is not None:
            return P.encode_nak(seq, P.NAK_BUSY)

        timeout_us = None
        if args:
            try:
                timeout_us = int(float(args[0]) * 1000)
            except ValueError:
                return P.encode_nak(seq, P.NAK_BADARGS)
            if timeout_us <= 0:
                return P.encode_nak(seq, P.NAK_OUTOFRANGE)

        if timeout_us is not None and timeout_us < self.home_duration_us:
            # Homing is bounded locally: exceeding the bound is a fault, not a
            # nak, because the motion did start.
            self._motion = None
            self._raise_fault(Fault.HOME_TIMEOUT, "home_timeout", cause_active=False)
            return P.encode_ack(seq)

        self._motion = {
            "kind": "home",
            "start": self.angle,
            "target": 0.0,
            "start_us": self.now_us,
            "end_us": self.now_us + self.home_duration_us,
        }
        return P.encode_ack(seq)

    def _cmd_arm(self, seq, args):
        if self.state == State.FAULTED:
            return P.encode_nak(seq, P.NAK_FAULTED)
        if not self.homed:
            return P.encode_nak(seq, P.NAK_NOTHOMED)
        if not self.hv_on:
            return P.encode_nak(seq, P.NAK_NOHV)

        # The current decision and the shunt value are one decision. vsense
        # resets to 0, which with the 0.15 ohm shunts drives ~1.8x the motor's
        # rating, so firmware reads CHOPCONF back before enabling the driver and
        # refuses if it does not say 1. Modelled here so Gate C can provoke it.
        if not self.vsense_ok:
            self._raise_fault(Fault.INTERNAL, "vsense_not_1", cause_active=True)
            return P.encode_nak(seq, P.NAK_FAULTED)

        self.state = State.ARMED
        self._watchdog_deadline = self.now_us + self.host_timeout_us
        return P.encode_ack(seq)

    def _cmd_disarm(self, seq, args):
        # "Always permitted."
        self._motion = None
        self._watchdog_deadline = None
        if self.state in (State.ARMED, State.ACTIVE):
            self.state = State.HOMED
        return P.encode_ack(seq)

    def _cmd_stop(self, seq, args):
        # "Controlled stop; always permitted."
        if self._motion is not None:
            self.angle = self._interpolate()
            self._motion = None
        if self.state == State.ACTIVE:
            self.state = State.ARMED
        return P.encode_ack(seq)

    def _cmd_moveto(self, seq, args):
        return self._move(seq, args, relative=False)

    def _cmd_move(self, seq, args):
        return self._move(seq, args, relative=True)

    def _move(self, seq, args, relative):
        if self.state == State.FAULTED:
            return P.encode_nak(seq, P.NAK_FAULTED)
        # "Motion verbs are refused outside ARMED."
        if self.state != State.ARMED:
            return P.encode_nak(seq, P.NAK_NOTARMED)
        if not args:
            return P.encode_nak(seq, P.NAK_BADARGS)
        try:
            value = float(args[0])
        except ValueError:
            return P.encode_nak(seq, P.NAK_BADARGS)

        target = self.angle + value if relative else value
        if not (self.travel_lo <= target <= self.travel_hi):
            # Rejected before anything moves, so this is a nak and not an
            # OVER_TRAVEL fault. The fault is for a position that got there.
            return P.encode_nak(seq, P.NAK_OUTOFRANGE)

        distance = abs(target - self.angle)
        duration = int(distance / self.v_max * 1_000_000) if self.v_max else 0
        self._motion = {
            "kind": "move",
            "start": self.angle,
            "target": target,
            "start_us": self.now_us,
            "end_us": self.now_us + max(duration, 1),
        }
        self.state = State.ACTIVE
        return P.encode_ack(seq)

    def _cmd_armpull(self, seq, args):
        if len(args) != 1:
            return P.encode_nak(seq, P.NAK_BADARGS)
        try:
            threshold = float(args[0])
        except ValueError:
            return P.encode_nak(seq, P.NAK_BADARGS)
        if not (self.travel_lo <= threshold <= self.travel_hi):
            return P.encode_nak(seq, P.NAK_OUTOFRANGE)
        self.pull_threshold = threshold
        return P.encode_ack(seq)

    def _cmd_angout(self, seq, args):
        if len(args) != 1:
            return P.encode_nak(seq, P.NAK_BADARGS)
        action = args[0].upper()
        if action not in ("ON", "OFF"):
            return P.encode_nak(seq, P.NAK_BADARGS)
        # Firmware ordering, not a protocol matter but easy to get wrong: the
        # DAC's GAIN register must be written to 0x0101 before the first data
        # write, or a full-scale code drives ~4.85 V into the BNC. ANGOUT ON is
        # the natural place to enforce it, which is why it is noted here.
        self.angout_on = action == "ON"
        return P.encode_ack(seq)

    def _cmd_angscale(self, seq, args):
        if len(args) != 2:
            return P.encode_nak(seq, P.NAK_BADARGS)
        try:
            lo = float(args[0])
            hi = float(args[1])
        except ValueError:
            return P.encode_nak(seq, P.NAK_BADARGS)
        if lo >= hi:
            return P.encode_nak(seq, P.NAK_OUTOFRANGE)
        if lo < self.travel_lo or hi > self.travel_hi:
            return P.encode_nak(seq, P.NAK_OUTOFRANGE)
        self.angout_lo, self.angout_hi = lo, hi
        return P.encode_ack(seq)

    def _cmd_cal(self, seq, args):
        if not args:
            return P.encode_nak(seq, P.NAK_BADARGS)
        action = args[0].lower()
        if action == "get":
            return P.encode_ack(seq, dict(self.cal))
        if action == "set":
            if len(args) != 3:
                return P.encode_nak(seq, P.NAK_BADARGS)
            key, raw = args[1], args[2]
            if key not in self.cal:
                return P.encode_nak(seq, P.NAK_BADARGS)
            try:
                value = int(raw) if isinstance(self.cal[key], int) else float(raw)
            except ValueError:
                return P.encode_nak(seq, P.NAK_BADARGS)
            self.cal[key] = value
            return P.encode_ack(seq, {key: value})
        return P.encode_nak(seq, P.NAK_BADARGS)

    # ------------------------------------------------------------------ state

    def _interpolate(self):
        motion = self._motion
        span = motion["end_us"] - motion["start_us"]
        if span <= 0:
            return motion["target"]
        fraction = float(self.now_us - motion["start_us"]) / span
        fraction = min(max(fraction, 0.0), 1.0)
        return motion["start"] + (motion["target"] - motion["start"]) * fraction

    def _raise_fault(self, bit, detail, cause_active):
        """Latch a fault. Motion is already stopped before the event is sent."""
        if bit == Fault.RESERVED_FOREVER:
            raise AssertionError("bit 4 is reserved forever; ENCODER_MISMATCH is retired")

        self._motion = None
        self._watchdog_deadline = None
        self.fault_mask |= 1 << bit
        self.fault_detail[bit] = detail
        self._fault_cause_active[bit] = cause_active
        self.state = State.FAULTED
        # Faults drop HV_EN, per the board's safety rules.
        self.hv_on = False
        self._emit("FAULT", {"code": bit, "detail": detail})

    # --------------------------------------------------- test injection hooks

    def inject_fault(self, bit, detail=None, cause_active=False):
        """Provoke any fault code, which Gate C requires for every one of them."""
        name = Fault.NAMES.get(bit)
        if name is None:
            raise ValueError("no such fault bit: %r" % (bit,))
        self._raise_fault(bit, detail or name, cause_active)
        return self._drain()

    def clear_fault_cause(self, bit):
        """Model the physical cause going away, so CLRFAULT can succeed."""
        self._fault_cause_active[bit] = False

    def inject_pull(self, angle):
        """Fire a pull event. On real hardware this also asserts the TTL line."""
        self._emit("PULL", {"angle": angle})
        return self._drain()

    def inject_limit(self):
        """Assert the limit switch outside homing: LIMIT event plus a fault."""
        self.limit_asserted = True
        self._emit("LIMIT")
        self._raise_fault(Fault.LIMIT_UNEXPECTED, "limit_unexpected", cause_active=True)
        return self._drain()

    def inject_rsync(self, n):
        """Record an RSYNC edge. Post-hoc alignment is done from these."""
        self._emit("RSYNC", {"n": n})
        return self._drain()
