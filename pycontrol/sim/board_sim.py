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
    FREEWHEEL = "FREEWHEEL"


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
        self.instr_on = False

        # Set by ARMPULL. While FREEWHEEL, the encoder crossing this emits PULL.
        # Testable on the boards in hand by moving the joystick by hand - it does
        # not need the brake, only a released driver.
        self.pull_threshold = None
        self._pull_reported = False

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
            # Under commanded motion the encoder tracks the ramp generator; that
            # they agree here is what makes their disagreement elsewhere mean
            # something.
            self.xactual = self.angle
            if motion["kind"] == "home":
                self.homed = True
                self.angle = 0.0
                self.xactual = 0.0
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
        caps = ["motion", "home", "freewheel", "instr", "hv"]
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
                "homed": self.homed,
                "limit": self.limit_asserted,
                "faults": self.fault_mask,
                "hv": self.hv_on,
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
        if self._motion is not None:
            return P.encode_nak(seq, P.NAK_BUSY)
        if self.state == State.FREEWHEEL:
            self._resync_from_encoder()
            self.state = State.ARMED

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
        if self.state in (State.ARMED, State.ACTIVE, State.FREEWHEEL):
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
        if self.state in (State.ACTIVE, State.FREEWHEEL):
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
        # "Motion verbs are refused outside ARMED" - and FREEWHEEL, which
        # re-engages implicitly. One command that resyncs, re-engages and moves
        # is better than three the host could be interrupted between.
        if self.state == State.FREEWHEEL:
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
        # Re-arming clears the latch, so one ARMPULL yields one PULL per trial
        # rather than a stream of them while the joystick sits past threshold.
        self._pull_reported = False
        return P.encode_ack(seq)

    # ANGOUT and ANGSCALE are deliberately NOT implemented. The analog outputs
    # are not populated on the boards in hand, so anything here would assert what
    # was guessed rather than what the board does - the 0.05-3.300 V window, the
    # DAC `GAIN = 0x0101` ordering rule, and the "0 V means invalid" model
    # included. They return when there is hardware to check them against, and
    # with a second channel, because position AND torque stream continuously.

    def _cmd_freewheel(self, seq, args):
        """Release the driver so the animal can move the joystick.

        SPEC: not in the draft. Added 2026-09-02 for the rev-0 boards.

        On a board WITH a brake this verb should not exist separately. The brake
        must engage BEFORE motor drive is released, or the joystick drops as it
        goes compliant, and it must be released only after drive is restored, or
        the motor fights the brake. Two host commands cannot guarantee that
        ordering: the host can die between them, or the second message can arrive
        late over a link whose whole point is that it has latency. The sequence
        belongs inside one firmware operation - `BRAKE <torque>` - which is what
        should replace this.

        So this is a bring-up primitive with a known expiry date, not a design.
        """
        if self.state == State.FAULTED:
            return P.encode_nak(seq, P.NAK_FAULTED)
        if self.state == State.ACTIVE:
            return P.encode_nak(seq, P.NAK_BUSY)
        if self.state != State.ARMED:
            return P.encode_nak(seq, P.NAK_NOTARMED)

        self.state = State.FREEWHEEL
        self._pull_reported = False
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

    def move_joystick(self, angle):
        """The animal moves the joystick. Only possible while compliant.

        Refused in any other state on purpose: with the driver enabled the motor
        holds position, so a test that "pulls" a held joystick is testing
        something that cannot happen. Emits PULL if ARMPULL set a threshold and
        this crosses it. On rev 1 hardware that also asserts the TTL line; on
        these boards the UART line is the only report, with UART latency - fine
        for exercising the interface, not fine for behavioural data.
        """
        if self.state != State.FREEWHEEL:
            raise AssertionError("the joystick can only be moved by hand while FREEWHEEL, not in %s" % self.state)

        self.angle = angle
        # xactual deliberately NOT updated: nothing turns the ramp generator
        # here, and that staleness is the thing _resync_from_encoder exists for.

        if self.pull_threshold is not None and not self._pull_reported:
            if abs(angle) >= abs(self.pull_threshold):
                self._pull_reported = True
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
