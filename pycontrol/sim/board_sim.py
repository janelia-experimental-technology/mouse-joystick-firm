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
    # Protocol v1, bits 9 and up (uart-protocol.md V1.7): pcb v1.0's brake
    # sink and torque bridge. Raised by the firmware's hardware backend only.
    BRAKE_FAULT = 9
    BRAKE_CURRENT = 10
    TORQUE_ADC = 11
    TORQUE_RANGE = 12

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
        9: "brake_fault",
        10: "brake_current",
        11: "torque_adc",
        12: "torque_range",
    }


class State:
    BOOT = "BOOT"
    IDLE = "IDLE"
    HOMED = "HOMED"
    ARMED = "ARMED"
    ACTIVE = "ACTIVE"
    FAULTED = "FAULTED"

    # The state in which the ANIMAL drives the joystick, not the board. The
    # driver is released so motor torque does not add to the brake's, and the
    # ramp generator's XACTUAL stops meaning anything - only the encoder knows
    # where the joystick is. `BoardSim._resync_from_encoder` reconciles them on
    # the way out, and only on the way out.
    #
    # Not in the draft's state machine. Added 2026-09-02 from the trial
    # structure: home to park, move to present, brake + release for the pull.
    # It is the state that makes the draft's single "position" field ambiguous,
    # because in it the measured and commanded positions genuinely differ.
    RELEASED = "RELEASED"  # FREEWHEEL until protocol v1 (2026-10-01)


class BoardSim:
    """One board on one link. Two instances share nothing."""

    # The draft now bounds the cache at 2 entries
    # (`../../../mouse-joystick-pcb/docs/uart-protocol.md`, framing). The case
    # it exists for is "the host retried immediately because a reply was lost";
    # a deeper window buys nothing and is the only thing that makes `seq` reuse
    # across a 65536 wrap reachable. Bounding it also matters because an
    # unbounded cache on the real firmware is a slow memory leak on a device
    # with no way to report one.
    RETRANSMIT_CACHE_SIZE = 2

    # Cited from `../../../mouse-joystick-pcb/docs/pycontrol-port-pinout.md`.
    DEFAULT_HOST_TIMEOUT_US = 100_000
    # The firmware's maximum for one homing cycle; `home;<s>;<timeout_ms>` is
    # clipped to it (TmcMotion::Config::home_timeout_us is the same 8 s).
    HOME_TIMEOUT_MAX_US = 8_000_000

    # Protocol v1 (2026-10-01).
    WDT_MIN_US = 50_000
    WDT_MAX_US = 500_000
    DEADBAND_MIN_DEG = 0.05
    DEADBAND_MAX_DEG = 10.0
    STREAM_HZ_MIN = 1
    STREAM_HZ_MAX = 50  # at 115200 baud
    FETCH_CHUNK_MAX = 32
    CAPTURE_MAX = 2048  # the ring, 1 kHz
    WINDOW_MAX_MS = CAPTURE_MAX - 1  # pre_ms, post_ms and their sum
    SAMPLE_US = 1_000
    TRIAL_DELAY_US = 1_000_000
    # A board with a brake: BRAKE acks, the brake ramps and settles, RELEASED
    # follows this much later. About what the real sink takes (an 8 ms ramp
    # plus the coil's lag), rounded to a number a corpus can read; the
    # firmware's hardware backend decides for itself from BRAKE_ISENSE.
    BRAKE_SETTLE_US = 10_000
    DEFAULT_PRE_MS = 500
    DEFAULT_POST_MS = 500
    PROFILES = ("present", "retract", "slow")

    def __init__(
        self,
        serial="E4654C159F4D1334",
        fw_version="0.0.0",
        hw_rev="rev0",
        travel_deg=(-20.0, 20.0),
        v_max_deg_s=180.0,
        a_max_deg_s2=3600.0,  # LIMITS a_max: the simulator's programmed ramp (its moves are constant-velocity; the number is reported, not modelled)
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
        # The firmware's maximum for one homing cycle (TmcMotion's own bound).
        # A host timeout is clipped to it and enforced as a deadline.
        self.home_timeout_max_us = self.HOME_TIMEOUT_MAX_US
        self._home_deadline = None

        self.now_us = 0
        self.state = State.BOOT

        # TWO positions, and the distinction is load-bearing.
        #
        #   angle    what the encoder says. The joystick's actual position, and
        #            the only one that is ever true.
        #   xactual  what the ramp generator thinks. True only while the board
        #            is driving; stale the moment the animal moves the joystick
        #            in FREEWHEEL, because nothing turns the ramp generator.
        #
        # The previous rig had no encoder and therefore had to home every trial:
        # each pull lost the position outright. Here the encoder keeps it, so
        # homing becomes a way to park the joystick rather than a way to recover
        # from a pull. Reconciling the two is `_resync_from_encoder`.
        self.angle = 0.0
        self.xactual = 0.0
        self.homed = False
        self.limit_asserted = False
        self.fault_mask = 0
        self.fault_detail = {}
        self._fault_cause_active = {}

        # Boot leaves the motor rail off: the 36 V input block's ENABLE is pulled
        # low and the rail is genuinely off, which is why HV is an observable
        # state rather than something the host infers.
        self.hv_on = False
        # The InstrumentPort buffers are enabled at boot for the proof of life
        # (protocol v1: INSTR STATE reports on at boot).
        self.instr_on = True

        # Protocol v1: capabilities (hardware facts of the simulated revision),
        # the pull direction, the runtime configuration, the classifier and the
        # stream.
        self.has_brake = False
        self.has_torque = False
        self.has_angout = False
        self.pull_sign = 1.0
        self.deadband_deg = 0.5
        self.stream_hz = 10
        self.stream_on = False
        self._stream_period_us = 100_000
        self._stream_next_us = 0
        self.brake_level = 0.0
        # A BRAKE on a board with a brake: acked, the brake settling, the
        # drive not yet released. RELEASED follows BRAKE_SETTLE_US later.
        self._brake_pending = False
        self._brake_settle_at = None
        # Torque on a board with the bridge: the TORQUE stimulus sets the
        # reading (0.0 at boot), TQ zero tares it, TQ cal is a switch.
        self.torque = 0.0
        self.torque_zero_ncm = 0.0
        self.torque_cal_on = False
        self._watch = _TrialWatch()

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
        serviced_at_target = False  # one service per advance (2026-10-02), as the firmware
        while self.now_us < target:
            # Step to the next scheduled thing, so an event never lands at the
            # wrong timestamp just because the caller advanced in one big jump.
            next_stop = target
            if self._motion is not None and self._motion["end_us"] < next_stop:
                next_stop = self._motion["end_us"]
            if self._watchdog_deadline is not None and self._watchdog_deadline < next_stop:
                next_stop = self._watchdog_deadline
            if self._home_deadline is not None and self._home_deadline < next_stop:
                next_stop = self._home_deadline
            watch_at = self._watch.next_stop_us(self.now_us)
            if watch_at is not None and watch_at < next_stop:
                next_stop = watch_at
            if self.stream_on and self._stream_next_us < next_stop:
                next_stop = self._stream_next_us
            if self._brake_pending and self._brake_settle_at is not None and self._brake_settle_at < next_stop:
                next_stop = self._brake_settle_at  # a BRAKE settling: RELEASED on time

            self.now_us = next_stop
            self._service()

            if next_stop == target:
                serviced_at_target = True
                break
        self.now_us = target
        if not serviced_at_target:
            self._service()
        return self._drain()

    def _service(self):
        if self._motion is not None and self.now_us >= self._motion["end_us"]:
            motion = self._motion
            self._motion = None
            self.angle = motion["target"]
            # Under commanded motion the encoder tracks the ramp generator; that
            # they agree here is what makes their disagreement elsewhere mean
            # something.
            self.xactual = self.angle
            if motion["kind"] == "home":
                self._home_deadline = None
                self.homed = True
                self.angle = 0.0
                self.xactual = 0.0
                self.state = State.HOMED
                self._emit("HOMED", {"angle": self.angle})
            else:
                self.state = State.ARMED
                self._emit("TARGET", {"angle": self.angle})

        # BRAKE in progress (a board with a brake): the drive is released only
        # once the brake is at its setpoint, and RELEASED is dated from here.
        if self._brake_pending and self._brake_settle_at is not None and self.now_us >= self._brake_settle_at:
            self._complete_release()

        # RELEASED: the classifier and the capture see every sample, in order,
        # and the TRIAL summary follows an outcome by one second if the host has
        # not left RELEASED by then (protocol v1).
        if self._watch.released:
            torque = self._torque_value()
            self._watch.sample(self.now_us, self.angle, torque)
            outcome = self._watch.evaluate(self.now_us, self.angle, torque)
            # TOUCH is informational and may precede the outcome; when both
            # fall on one sample it goes out first.
            if self._watch.consume_touch():
                self._emit_angle_event("TOUCH", self.angle, with_brake=False)
            if outcome is not None:
                self._emit_angle_event(outcome, self.angle, with_brake=False)
            if self._watch.trial_due_now(self.now_us):
                self._emit_trial()

        # STREAM: decimated telemetry, display only.
        if self.stream_on and self.now_us >= self._stream_next_us:
            self._stream_next_us += self._stream_period_us
            self._emit_angle_event("S", self.angle, with_brake=False)

        # The host's homing bound, checked after completion so a home that
        # finishes on the deadline counts as done.
        if self._home_deadline is not None and self.now_us >= self._home_deadline:
            self._home_deadline = None
            self._raise_fault(Fault.HOME_TIMEOUT, "home_timeout", cause_active=False)
            return

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
            # Deliberately before the watchdog refresh below: a frame that does
            # not parse does not prove the host is alive. A wedged serial driver
            # emitting garbage must not hold the safety timeout open.
            self._out(P.encode_nak(seq if seq is not None else P.SEQ_UNKNOWN, rest))
            return

        # The host-loss watchdog is reset HERE -- on receipt of a well-formed
        # command, before dispatch, cache hit or miss -- and not inside any
        # handler. Liveness is a property of the link, so it is decided where
        # the frame arrives. Resolved 2026-09-03 in
        # `../../../mouse-joystick-pcb/docs/uart-protocol.md` open question 1;
        # the draft used to reset it inside PING, which put a link-layer fact
        # behind an application-layer decision and made the retransmission rule
        # a safety rule by accident.
        self._refresh_watchdog()

        canonical = P.DELIMITER.join([verb, str(seq)] + list(rest))
        cacheable = P.changes_state(verb, rest)

        if cacheable:
            cached = self._retransmit.get(seq)
            if cached is not None and cached[0] == canonical:
                # "A repeated `seq` with identical content and a state-changing
                # verb is treated as a retransmission and re-acknowledged
                # without re-executing." The one command in the set this
                # actually protects is MOVE, which is relative and would double
                # the movement; everything else names a destination.
                self._out(cached[1])
                return

        reply = self._dispatch(verb, seq, rest)
        if reply is not None:
            if cacheable:
                self._remember(seq, canonical, reply)
            self._out(reply)

    def _refresh_watchdog(self):
        # Only meaningful once the watchdog is armed. Before that there is no
        # deadline to push out and nothing to do.
        if self._watchdog_deadline is not None:
            self._watchdog_deadline = self.now_us + self.host_timeout_us

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
                "boot": "por",  # the simulator never reboots
                "crumb": 0,
            },
        )

    def _capabilities(self):
        # SPEC: the draft says `ID` reports "capabilities" without saying how.
        # A `+`-joined token list avoids the delimiter and stays greppable in a
        # logic-analyzer capture, which is the draft's stated design rule.
        #
        # This is the honest list for a rev-0 board: no `angout` because the
        # analog outputs are not populated, no `brake` or `torque` because that
        # hardware does not exist yet. A host should gate on this rather than on
        # the firmware version, so a rev-1 board can simply report more.
        # Fixed order, absent tokens omitted (protocol v1): firmware facts
        # first, then the hardware facts of this revision.
        caps = ["motion", "home", "instr", "hv", "watch", "fetch", "stream", "cfg"]
        if self.has_angout:
            caps.append("angout")
        if self.has_brake:
            caps.append("brake")
        if self.has_torque:
            caps.append("torque")
        return "+".join(caps)

    def _cmd_ping(self, seq, args):
        # Carries no data and has no side effect of its own. The watchdog was
        # already refreshed in `_handle_line`, by the frame arriving, so PING is
        # simply the command to send when there is nothing else to say -- and a
        # STATUS poll keeps the link alive just as well.
        #
        # Always answered, including before ARM: a host has to be able to probe
        # a link it is not yet armed on.
        return P.encode_ack(seq)

    def _cmd_status(self, seq, args):
        # "Safe at any time", so no state check.
        #
        # SPEC: the draft lists "state, angle, position, homed flag, limit
        # state, fault mask" without saying whether "angle" and "position" are
        # the same quantity in different units or two different quantities. With
        # a compliant state they are two different quantities - measured and
        # commanded - and after a pull they disagree by the whole pull. Reported
        # as `angle` (encoder) and `xactual` (ramp generator). Raised in
        # `../README.md`.
        return P.encode_ack(
            seq,
            {
                "state": self.state,
                "angle": self.angle,
                "xactual": self.xactual,
                # Always present, empty without the hardware (protocol v1), so
                # field positions never move between revisions.
                "torque": self._torque_field(),
                "brake": self._brake_field(),
                "homed": self.homed,
                "limit": self.limit_asserted,
                "faults": self.fault_mask,
                "hv": self.hv_on,
                "t": self.now_us,  # the board clock, for the host's alignment
            },
        )

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
            self._home_deadline = None
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
        """Drive to the limit switch and zero.

        With an encoder, homing is no longer how position is RECOVERED - the
        encoder keeps it through a pull. It is how the joystick is PARKED out of
        the animal's way between trials. The previous rig had to home every
        trial because each pull lost the position outright; that requirement is
        gone, and homing every trial is now a choice rather than a necessity.
        """
        if self.state == State.FAULTED:
            return P.encode_nak(seq, P.NAK_FAULTED)
        if not self.hv_on:
            return P.encode_nak(seq, P.NAK_NOHV)
        if self._motion is not None or self._brake_pending:
            return P.encode_nak(seq, P.NAK_BUSY)
        if self.state == State.RELEASED:
            self._close_trial()
            self._resync_from_encoder()
            self.state = State.ARMED
            # Every home re-energises the drive (the firmware enables the
            # driver here); the brake comes off after it.
            self._brake_off()

        timeout_us = None
        if args:
            try:
                timeout_us = int(float(args[0]) * 1000)
            except ValueError:
                return P.encode_nak(seq, P.NAK_BADARGS)
            if timeout_us <= 0:
                return P.encode_nak(seq, P.NAK_OUTOFRANGE)

        # The host's timeout, clipped to the firmware maximum, is a deadline on
        # the running home (enforced in _service); it used to be compared with
        # the expected duration up front, which on the real backend faulted
        # every request under its 8 s bound immediately (2026-10-01, bug 2).
        if timeout_us is not None:
            timeout_us = min(timeout_us, self.home_timeout_max_us)

        self._motion = {
            "kind": "home",
            "start": self.angle,
            "target": 0.0,
            "start_us": self.now_us,
            "end_us": self.now_us + self.home_duration_us,
        }
        if timeout_us is not None:
            self._home_deadline = self.now_us + timeout_us
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
        self._home_deadline = None
        self.stream_on = False
        if self._brake_pending:
            self._brake_off()  # a BRAKE still settling is cancelled; the drive never left
        if self.state == State.RELEASED:
            self._leave_released()
        if self.state in (State.ARMED, State.ACTIVE, State.RELEASED):
            self._resync_from_encoder()
            self.state = State.HOMED
        return P.encode_ack(seq)

    def _cmd_stop(self, seq, args):
        # "Controlled stop; always permitted." From FREEWHEEL this is how a host
        # takes the joystick back: re-engage the driver holding wherever the
        # animal left it.
        if self._motion is not None:
            self.angle = self._interpolate()
            self.xactual = self.angle
            self._motion = None
        self._home_deadline = None
        if self._brake_pending:
            self._brake_off()  # a BRAKE still settling is cancelled; the drive never left
        if self.state == State.RELEASED:
            self._leave_released()
        if self.state in (State.ACTIVE, State.RELEASED):
            self._resync_from_encoder()
            self.state = State.ARMED
        return P.encode_ack(seq)

    def _cmd_moveto(self, seq, args):
        return self._move(seq, args, relative=False)

    def _cmd_move(self, seq, args):
        return self._move(seq, args, relative=True)

    def _move(self, seq, args, relative):
        if self.state == State.FAULTED:
            return P.encode_nak(seq, P.NAK_FAULTED)
        if self._brake_pending:
            return P.encode_nak(seq, P.NAK_BUSY)  # the brake is still engaging
        # "Motion verbs are refused outside ARMED" - and FREEWHEEL, which
        # re-engages implicitly. One command that resyncs, re-engages and moves
        # is better than three the host could be interrupted between.
        if self.state == State.RELEASED:
            self._leave_released()
            self._resync_from_encoder()
            self.state = State.ARMED
        if self.state != State.ARMED:
            return P.encode_nak(seq, P.NAK_NOTARMED)
        if not args:
            return P.encode_nak(seq, P.NAK_BADARGS)
        try:
            value = float(args[0])
        except ValueError:
            return P.encode_nak(seq, P.NAK_BADARGS)
        # Named profiles (protocol v1): accepted; every profile moves at this
        # simulator's one v_max, so corpus timings do not depend on them.
        if len(args) > 1 and args[1].lower() not in self.PROFILES:
            return P.encode_nak(seq, P.NAK_BADARGS)

        # Relative to where the joystick IS, which after a pull is not where the
        # ramp generator thinks it is. Using xactual here would silently add the
        # animal's pull to every subsequent relative move.
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

    def _resync_from_encoder(self):
        """Reload the ramp generator's position from the encoder.

        ONLY on leaving a compliant state. Never during commanded motion, and
        never on a timer.

        `.agents/AGENTS.md`: "XACTUAL advancing while X_ENC does not is how
        firmware detects an end stop, a stall, a slipping coupling or a dead
        encoder. Do not add a motion path that skips it." That detector IS the
        divergence between these two numbers - so a resync deletes it. Doing it
        continuously would trade the board's only stall detection for the
        convenience of not tracking which state it is in.

        The two cases are distinguishable by state, and only by state:
          driving    divergence means something is broken -> fault
          compliant  divergence means the animal pulled  -> resync on exit
        """
        self.xactual = self.angle

    # ------------------------------------------------------ protocol v1 verbs

    def _torque_value(self):
        """The latest tared torque sample in N.cm, or None without the bridge."""
        if not self.has_torque:
            return None
        return self.torque - self.torque_zero_ncm

    def _torque_field(self):
        value = self._torque_value()
        return "" if value is None else value

    def _brake_field(self):
        # Always present: a number on a board with the brake (the commanded
        # level while it engages or holds, 0 off), empty without.
        return self.brake_level if self.has_brake else ""

    def _emit_angle_event(self, name, angle, with_brake):
        data = {"angle": angle, "torque": self._torque_field()}
        if with_brake:
            data["brake"] = self._brake_field()
        self._emit(name, data)

    def _emit_trial(self):
        t = self._watch.take_summary(self.angle, self._torque_value() or 0.0)
        has_torque = self.has_torque
        self._emit(
            "TRIAL",
            {
                "outcome": t["outcome"],
                "t_rel": t["t_rel"],
                "t_touch": t["t_touch"] if (has_torque and t["t_touch"] is not None) else "",
                "t_move": t["t_move"] if t["t_move"] is not None else "",
                "t_out": t["t_out"] if t["t_out"] is not None else "",
                "a0": t["a0"],
                "a_peak": t["a_peak"],
                "a_end": t["a_end"],
                "tq_peak": t["tq_peak"] if has_torque else "",
                "tq_out": t["tq_end"] if has_torque else "",
                "dur_thr": t["dur_thr_ms"],
            },
        )

    def _close_trial(self):
        """Close the RELEASED period's bookkeeping: TRIAL if a WATCH happened
        in it and was not reported, and stop the classifier."""
        if self._watch.trial_pending():
            self._emit_trial()
        self._watch.leave()

    def _brake_off(self):
        """The brake off: the level the fields report and a BRAKE still
        settling. The firmware's backend releases the coil here."""
        self.brake_level = 0.0
        self._brake_pending = False
        self._brake_settle_at = None

    def _leave_released(self):
        """Close RELEASED where the caller does not re-engage the drive (a
        fault, or this simulator's motion verbs, which model no driver): the
        bookkeeping, then the brake off. The firmware orders the brake after
        the drive is back (uart-protocol.md 3a)."""
        self._close_trial()
        self._brake_off()

    def _complete_release(self):
        """The brake is at its setpoint, or there is none: release the drive,
        become RELEASED, start the classifier's ring, announce it."""
        self._brake_pending = False
        self._brake_settle_at = None
        self.state = State.RELEASED
        self._watch.release(self.now_us, self.angle, self.pull_sign)
        self._emit_angle_event("RELEASED", self.angle, with_brake=True)

    @staticmethod
    def _kv(arg):
        key, sep, value = arg.partition("=")
        if not sep or not key:
            return None, None
        return key.lower(), value

    def _cmd_brake(self, seq, args):
        """BRAKE <torque_Ncm>: engage the brake, then release the drive, as one
        operation. Without a brake the torque is ignored and only the release
        happens. The ack comes first and RELEASED when the release has happened.
        """
        if self.state == State.FAULTED:
            return P.encode_nak(seq, P.NAK_FAULTED)
        if self.state == State.ACTIVE or self._brake_pending:
            return P.encode_nak(seq, P.NAK_BUSY)
        if self.state != State.ARMED:
            return P.encode_nak(seq, P.NAK_NOTARMED)
        if len(args) != 1:
            return P.encode_nak(seq, P.NAK_BADARGS)
        try:
            torque = float(args[0])
        except ValueError:
            return P.encode_nak(seq, P.NAK_BADARGS)
        if torque < 0.0:
            return P.encode_nak(seq, P.NAK_OUTOFRANGE)
        self.brake_level = torque if self.has_brake else 0.0
        if self.has_brake:
            # Phases 1 and 2: the brake to its setpoint first; _service()
            # releases the drive and emits RELEASED once it has settled.
            # Motion verbs are busy meanwhile; STOP and DISARM cancel it.
            self._brake_pending = True
            self._brake_settle_at = self.now_us + self.BRAKE_SETTLE_US
            return P.encode_ack(seq)
        # Without a brake there is nothing to wait for: release now.
        self._out(P.encode_ack(seq))
        self._complete_release()
        return None

    def _cmd_watch(self, seq, args):
        if self.state != State.RELEASED:
            return P.encode_nak(seq, P.NAK_NOTRELEASED)
        params = {"pull": None, "push": None, "timeout_us": None, "hold_us": 0, "maxdur_us": None, "touch": None}
        for arg in args:
            key, value = self._kv(arg)
            if key is None:
                return P.encode_nak(seq, P.NAK_BADARGS)
            try:
                v = float(value)
            except ValueError:
                return P.encode_nak(seq, P.NAK_BADARGS)
            if v < 0.0:
                return P.encode_nak(seq, P.NAK_OUTOFRANGE)
            if key == "pull":
                params["pull"] = v
            elif key == "push":
                params["push"] = v
            elif key == "timeout":
                params["timeout_us"] = int(v * 1000)
            elif key == "hold":
                params["hold_us"] = int(v * 1000)
            elif key == "maxdur":
                params["maxdur_us"] = int(v * 1000)
            elif key == "touch":
                params["touch"] = v  # meaningful with the v1.0 bridge; accepted and inert without
            else:
                return P.encode_nak(seq, P.NAK_BADARGS)
        if params["pull"] is None or params["push"] is None or params["timeout_us"] is None:
            return P.encode_nak(seq, P.NAK_BADARGS)
        self._watch.arm(self.now_us, params, self.deadband_deg)
        return P.encode_ack(seq)

    def _cmd_cfg(self, seq, args):
        if not args:
            return P.encode_nak(seq, P.NAK_BADARGS)
        if len(args) == 1 and args[0].lower() == "get":
            return P.encode_ack(
                seq,
                {
                    "wdt_ms": self.host_timeout_us // 1000,
                    "deadband_deg": self.deadband_deg,
                    "stream_hz": self.stream_hz,
                    "pre_ms": self._watch.pre_ms,
                    "post_ms": self._watch.post_ms,
                },
            )
        wdt_us, deadband, stream_hz = self.host_timeout_us, self.deadband_deg, self.stream_hz
        pre_ms, post_ms = self._watch.pre_ms, self._watch.post_ms
        for arg in args:
            key, value = self._kv(arg)
            if key is None:
                return P.encode_nak(seq, P.NAK_BADARGS)
            try:
                v = float(value)
            except ValueError:
                return P.encode_nak(seq, P.NAK_BADARGS)
            if key == "wdt_ms":
                us = v * 1000.0
                if us < self.WDT_MIN_US or us > self.WDT_MAX_US:
                    return P.encode_nak(seq, P.NAK_OUTOFRANGE)
                wdt_us = int(us)
            elif key == "deadband_deg":
                if v < self.DEADBAND_MIN_DEG or v > self.DEADBAND_MAX_DEG:
                    return P.encode_nak(seq, P.NAK_OUTOFRANGE)
                deadband = v
            elif key == "stream_hz":
                if v < self.STREAM_HZ_MIN or v > self.STREAM_HZ_MAX:
                    return P.encode_nak(seq, P.NAK_OUTOFRANGE)
                stream_hz = int(v)
            elif key in ("pre_ms", "post_ms"):
                # The capture window around the outcome: each bounded by the
                # ring, and together (below) by the ring less the outcome sample.
                if v < 0.0 or v > self.WINDOW_MAX_MS:
                    return P.encode_nak(seq, P.NAK_OUTOFRANGE)
                if key == "pre_ms":
                    pre_ms = int(v)
                else:
                    post_ms = int(v)
            else:
                return P.encode_nak(seq, P.NAK_BADARGS)
        if pre_ms + post_ms > self.WINDOW_MAX_MS:
            return P.encode_nak(seq, P.NAK_OUTOFRANGE)
        self.host_timeout_us = wdt_us  # applies from the next watchdog refresh
        self.deadband_deg = deadband
        self.stream_hz = stream_hz
        self._watch.set_window(pre_ms, post_ms)
        return P.encode_ack(seq)

    def _cmd_fetch(self, seq, args):
        """FETCH [k0;n]: the frozen window of the last WATCH in chunks of at
        most FETCH_CHUNK_MAX lines; sample `pre` is the outcome sample."""
        if self._watch.capture_busy():
            return P.encode_nak(seq, P.NAK_BUSY)
        try:
            k0 = int(args[0]) if len(args) >= 1 else 0
            n = int(args[1]) if len(args) >= 2 else self.FETCH_CHUNK_MAX
        except ValueError:
            return P.encode_nak(seq, P.NAK_BADARGS)
        if k0 < 0 or n < 0:
            return P.encode_nak(seq, P.NAK_OUTOFRANGE)
        n = min(n, self.FETCH_CHUNK_MAX)
        count = self._watch.window_count()
        available = count - k0 if k0 < count else 0
        n = min(n, available)
        self._out(
            P.encode_ack(
                seq,
                {
                    "n": n,
                    "k0": k0,
                    "t0": self._watch.window_sample_time_us(k0) if self._watch.has_window() else 0,
                    "dt_us": self.SAMPLE_US,
                    "pre": self._watch.window_pre(),
                    "lost": self._watch.window_lost(),
                },
            )
        )
        for k in range(k0, k0 + n):
            angle = P.format_number(self._watch.window_sample(k))
            torque = P.format_number(self._watch.window_torque(k)) if self.has_torque else ""
            self._out(P.DELIMITER.join([P.CAP, str(k), angle, torque]))
        self._out(P.DELIMITER.join([P.CAP, "end"]))
        return None

    def _cmd_stream(self, seq, args):
        if not args:
            return P.encode_nak(seq, P.NAK_BADARGS)
        action = args[0].lower()
        if action == "off":
            self.stream_on = False
            return P.encode_ack(seq)
        if action != "on":
            return P.encode_nak(seq, P.NAK_BADARGS)
        hz = self.stream_hz
        if len(args) >= 2:
            try:
                v = float(args[1])
            except ValueError:
                return P.encode_nak(seq, P.NAK_BADARGS)
            if v < self.STREAM_HZ_MIN or v > self.STREAM_HZ_MAX:
                return P.encode_nak(seq, P.NAK_OUTOFRANGE)
            hz = int(v)
        self._stream_period_us = 1_000_000 // hz
        self._stream_next_us = self.now_us + self._stream_period_us
        self.stream_on = True
        return P.encode_ack(seq)

    def _cmd_tq(self, seq, args):
        """TQ zero | TQ cal;on|off | TQ get -- boards whose caps include torque.
        `get` reports the tared reading, the raw one (what a tare removed) and
        the calibration switch, in N.cm."""
        if not self.has_torque:
            return P.encode_nak(seq, P.NAK_BADVERB)
        if not args:
            return P.encode_nak(seq, P.NAK_BADARGS)
        if len(args) == 1 and args[0].lower() == "zero":
            self.torque_zero_ncm = self.torque
            return P.encode_ack(seq)
        if len(args) == 1 and args[0].lower() == "get":
            return P.encode_ack(
                seq,
                {
                    "torque": self.torque - self.torque_zero_ncm,
                    "raw": self.torque,
                    "cal": 1 if self.torque_cal_on else 0,
                },
            )
        if len(args) == 2 and args[0].lower() == "cal" and args[1].lower() in ("on", "off"):
            self.torque_cal_on = args[1].lower() == "on"
            return P.encode_ack(seq)
        return P.encode_nak(seq, P.NAK_BADARGS)

    def _cmd_angout(self, seq, args):
        if not self.has_angout:
            return P.encode_nak(seq, P.NAK_BADVERB)
        if len(args) != 1 or args[0].lower() not in ("on", "off"):
            return P.encode_nak(seq, P.NAK_BADARGS)
        self.angout_on = args[0].lower() == "on"
        return P.encode_ack(seq)

    def _cmd_angscale(self, seq, args):
        if not self.has_angout:
            return P.encode_nak(seq, P.NAK_BADVERB)
        if len(args) != 2:
            return P.encode_nak(seq, P.NAK_BADARGS)
        try:
            lo, hi = float(args[0]), float(args[1])
        except ValueError:
            return P.encode_nak(seq, P.NAK_BADARGS)
        if not lo < hi:
            return P.encode_nak(seq, P.NAK_OUTOFRANGE)
        self.angscale = (lo, hi)
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
        self._home_deadline = None
        self.stream_on = False
        if self.state == State.RELEASED:
            self._leave_released()  # the TRIAL summary precedes the FAULT event
        self._brake_off()  # a BRAKE still settling, too (V1.8: today's default is off)
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

    def move_joystick(self, angle):
        """The animal moves the joystick. Only possible while RELEASED.

        Refused in any other state on purpose: with the driver enabled the motor
        holds position, so a test that "pulls" a held joystick is testing
        something that cannot happen. The classifier (WATCH) sees the sample at
        once; returns whatever that produced.
        """
        if self.state != State.RELEASED:
            raise AssertionError("the joystick can only be moved by hand while RELEASED")
        self.angle = float(angle)
        return self.advance(0)

    def set_encoder_angle(self, angle):
        """The corpus's ANGLE stimulus: the encoder reads this from now on, in
        any state; the runner services the board separately."""
        self.angle = float(angle)

    def set_torque(self, torque_ncm):
        """The corpus's TORQUE stimulus: the bridge reads this from now on (a
        board with `torque` in its caps); the runner services separately."""
        self.torque = float(torque_ncm)

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


class _TrialWatch:
    """The WATCH classifier, the 1 kHz capture ring and the TRIAL summary, mirroring
    the firmware's `src/TrialWatch.hpp` step for step so the corpus can hold
    both to the same bytes. Angles in are absolute encoder degrees; decisions
    are on the pull-positive excursion `(angle - a0) * pull_sign`."""

    def __init__(self):
        self.released = False
        self.watched = False  # a WATCH happened since RELEASED
        self.armed = False  # classifier active, no outcome yet
        self.summarized = False
        self.trial_due = False
        self.trial_due_at = 0
        self.a0 = 0.0
        self.pull_sign = 1.0
        self.deadband = 0.5
        self.params = None
        self.t_rel = 0
        self.t_watch = 0
        self.first_motion = False
        self.t_move = None
        self.pull_valid = False
        self.pull_since = 0
        self.above_prev = False
        self.last_eval = 0
        self.dur_thr_us = 0
        self.peak = 0.0
        self.outcome = None
        self.t_out = None
        self.a_end = 0.0
        self.tq_peak = 0.0
        self.tq_end = 0.0
        self.touched = False
        self.touch_pending = False
        self.t_touch = None
        # the ring
        self.ring_running = False
        self.cap_watched = False  # a WATCH armed since this ring started
        self.ring_t0 = 0  # time of absolute sample 0
        self.next_sample = 0
        self.written = 0  # absolute samples written; slot = written % size
        self.trig_set = False
        self.trig_index = 0  # absolute index of the outcome sample
        self.pre_ms = BoardSim.DEFAULT_PRE_MS
        self.post_ms = BoardSim.DEFAULT_POST_MS
        # the frozen window
        self.win_valid = False
        self.win_start = 0
        self.win_count = 0
        self.win_pre = 0
        self.win_lost = 0
        self.cap = [0.0] * BoardSim.CAPTURE_MAX
        self.cap_tq = [0.0] * BoardSim.CAPTURE_MAX  # torque alongside, FETCH's third field
        self.fill = [False] * BoardSim.CAPTURE_MAX  # a grid fill, per slot

    def excursion(self, angle):
        return (angle - self.a0) * self.pull_sign

    def release(self, now_us, a0, pull_sign):
        """RELEASED entered: the ring starts here so a WATCH that follows has
        history behind it; the previous window is gone from here."""
        self.released = True
        self.a0 = a0
        self.pull_sign = pull_sign
        self.t_rel = now_us
        self.watched = False
        self.armed = False
        self.summarized = False
        self.trial_due = False
        self.outcome = None
        self.cap_watched = False
        self.win_valid = False
        self._start_ring(now_us)

    def leave(self):
        """RELEASED left: the window freezes with what exists."""
        if self.ring_running:
            self._freeze_window()
        self.released = False
        self.armed = False
        self.trial_due = False

    def arm(self, now_us, params, deadband_deg):
        self.params = params
        self.deadband = deadband_deg
        self.watched = True
        self.armed = True
        self.summarized = False
        self.trial_due = False
        self.outcome = None
        self.t_watch = now_us
        self.first_motion = False
        self.t_move = None
        self.pull_valid = False
        self.above_prev = False
        self.last_eval = now_us
        self.dur_thr_us = 0
        self.peak = 0.0
        self.a_end = 0.0
        self.t_out = None
        self.tq_peak = 0.0
        self.tq_end = 0.0
        self.touched = False
        self.touch_pending = False
        self.t_touch = None
        self.cap_watched = True
        self.trig_set = False
        if not self.ring_running:  # a WATCH after a frozen window restarts the ring
            self.win_valid = False
            self._start_ring(now_us)

    def set_window(self, pre_ms, post_ms):
        self.pre_ms = int(pre_ms)
        self.post_ms = int(post_ms)

    def sample(self, now_us, angle, torque=0.0):
        """The 1 ms-grid sample into the ring while it runs; idempotent at one
        time. A late caller fills the missed grid points with the current angle
        so sample k stays at t0 + k*dt (the firmware's coalesced tick); a gap
        longer than the whole ring resynchronises the grid instead."""
        if not self.ring_running:
            return
        if now_us >= self.next_sample + BoardSim.CAPTURE_MAX * BoardSim.SAMPLE_US:
            self.next_sample = now_us
        while self.ring_running and now_us >= self.next_sample:
            slot = self.written % BoardSim.CAPTURE_MAX
            self.cap[slot] = float(angle)
            self.cap_tq[slot] = float(torque or 0.0)
            # A grid point more than one period behind `now` was missed (the
            # firmware's coalesced tick) and is filled; FETCH reports `lost`.
            self.fill[slot] = (now_us - self.next_sample) >= BoardSim.SAMPLE_US
            self.written += 1
            self.next_sample += BoardSim.SAMPLE_US
            if self.trig_set and self.written >= self.trig_index + 1 + self.post_ms:
                self._freeze_window()

    def evaluate(self, now_us, angle, torque=0.0):
        if not self.armed:
            return None
        p = self.params
        torque = torque or 0.0
        rel = self.excursion(angle)
        if not self.first_motion and abs(rel) > self.deadband:
            self.first_motion = True
            self.t_move = now_us
        if abs(rel) > abs(self.peak):
            self.peak = rel
        if abs(torque) > abs(self.tq_peak):
            self.tq_peak = torque
        # TOUCH: informational, at most once per WATCH, on the magnitude.
        if p.get("touch") is not None and not self.touched and abs(torque) >= p["touch"]:
            self.touched = True
            self.t_touch = now_us
            self.touch_pending = True
        if self.above_prev:
            self.dur_thr_us += now_us - self.last_eval
        self.last_eval = now_us
        above = rel >= p["pull"]
        self.above_prev = above

        result = None
        if rel <= -p["push"]:
            result = "PUSH"
        elif above:
            if not self.pull_valid:
                self.pull_valid = True
                self.pull_since = now_us
            if now_us - self.pull_since >= p["hold_us"]:
                result = "PULL"
        else:
            self.pull_valid = False
        if result is None and not self.first_motion and now_us >= self.t_watch + p["timeout_us"]:
            result = "NOMOVE"
        if (
            result is None
            and self.first_motion
            and p["maxdur_us"] is not None
            and now_us >= self.t_watch + p["maxdur_us"]
        ):
            result = "ABORT"
        if result is not None:
            self.outcome = result
            self.t_out = now_us
            self.a_end = rel
            self.tq_end = torque
            self.armed = False
            self.trial_due = True
            self.trial_due_at = now_us + BoardSim.TRIAL_DELAY_US
            # The outcome sample is the last one written (sample() runs first
            # in the same service step); the window freezes post_ms later.
            if self.ring_running:
                self.trig_set = True
                self.trig_index = self.written - 1 if self.written > 0 else 0
                if self.written >= self.trig_index + 1 + self.post_ms:
                    self._freeze_window()
        return result

    def next_stop_us(self, now_us):
        candidates = []
        if self.ring_running:
            candidates.append(self.next_sample)
        if self.armed:
            p = self.params
            if not self.first_motion:
                candidates.append(self.t_watch + p["timeout_us"])
            if self.pull_valid:
                candidates.append(self.pull_since + p["hold_us"])
            if self.first_motion and p["maxdur_us"] is not None:
                candidates.append(self.t_watch + p["maxdur_us"])
        if self.trial_due and not self.summarized:
            candidates.append(self.trial_due_at)
        if not candidates:
            return None
        return max(min(candidates), now_us)

    def consume_touch(self):
        """A TOUCH happened since the last call; announced before the outcome
        of the same sample, if both fell on it."""
        pending = self.touch_pending
        self.touch_pending = False
        return pending

    def trial_pending(self):
        return self.released and self.watched and not self.summarized

    def trial_due_now(self, now_us):
        return self.trial_pending() and self.trial_due and now_us >= self.trial_due_at

    # name kept short in the board: trial_due(now) is the one-second rule
    def trial_due_check(self, now_us):
        return self.trial_due_now(now_us)

    def take_summary(self, angle, torque=0.0):
        has_out = self.outcome is not None
        summary = {
            "outcome": self.outcome or "NONE",
            "t_rel": self.t_rel,
            "t_move": self.t_move if self.first_motion else None,
            "t_out": self.t_out if has_out else None,
            "a0": self.a0,
            "a_peak": self.peak,
            "a_end": self.a_end if has_out else self.excursion(angle),
            "dur_thr_ms": self.dur_thr_us // 1000,
            "t_touch": self.t_touch if self.touched else None,
            "tq_peak": self.tq_peak,
            "tq_end": self.tq_end if has_out else (torque or 0.0),
        }
        self.summarized = True
        self.watched = False
        self.armed = False
        self.trial_due = False
        return summary

    # ---------------------------------------------------------------- ring

    def _start_ring(self, now_us):
        self.ring_t0 = now_us
        self.next_sample = now_us
        self.written = 0
        self.trig_set = False
        self.ring_running = True

    def _freeze_window(self):
        """The last pre_ms samples before the outcome sample, the outcome
        sample and what follows it (post_ms when it ran to the end, fewer if
        RELEASED was left first); without an outcome, the last pre_ms samples
        before the exit. `pre` is what the ring actually holds."""
        self.ring_running = False
        oldest = self.written - BoardSim.CAPTURE_MAX if self.written > BoardSim.CAPTURE_MAX else 0
        if self.trig_set:
            self.win_pre = min(self.pre_ms, self.trig_index - oldest)
            self.win_start = self.trig_index - self.win_pre
            self.win_count = self.written - self.win_start
        else:
            self.win_pre = min(self.pre_ms, self.written - oldest)
            self.win_start = self.written - self.win_pre
            self.win_count = self.win_pre
        self.win_valid = self.cap_watched
        self.win_lost = sum(
            1 for k in range(self.win_count) if self.fill[(self.win_start + k) % BoardSim.CAPTURE_MAX]
        )

    def capture_busy(self):
        """FETCH is refused while the capture is live: a WATCH armed, or the
        post window still filling after its outcome."""
        return self.ring_running and (self.armed or self.trig_set)

    def has_window(self):
        return self.win_valid

    def window_count(self):
        return self.win_count if self.win_valid else 0

    def window_pre(self):
        return self.win_pre if self.win_valid else 0

    def window_lost(self):
        return self.win_lost if self.win_valid else 0

    def window_sample_time_us(self, k):
        return self.ring_t0 + (self.win_start + k) * BoardSim.SAMPLE_US

    def window_sample(self, k):
        return self.cap[(self.win_start + k) % BoardSim.CAPTURE_MAX]

    def window_torque(self, k):
        return self.cap_tq[(self.win_start + k) % BoardSim.CAPTURE_MAX]
