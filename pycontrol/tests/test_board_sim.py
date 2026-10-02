"""Gate C: protocol conformance.

The checklist is `../../../mouse-joystick-pcb/docs/validation-plan.md`, "Gate C:
Protocol Conformance". Each of its bullets is named in a test below so a gap is
visible as a missing name rather than as an absence.

No board, no breakout, no pyControl installation is needed to run this.
"""

import pytest

from devices.mouse_joystick_protocol import MouseJoystickProtocol as P
from sim import BoardSim, Fault, State


def send(sim, line):
    """Send one command; return (reply, events).

    Events and replies share one channel, so a command can perfectly well
    produce an event first - `arm` on a board whose vsense read-back fails emits
    `ev;FAULT` and then `nak;...;faulted`. Splitting them here is what a host
    has to do anyway.
    """
    out = sim.feed(line if line.endswith("\n") else line + "\n")
    events = [o for o in out if o.startswith("ev;")]
    replies = [o for o in out if not o.startswith("ev;")]
    assert len(replies) == 1, "expected exactly one reply, got %r" % (out,)
    return replies[0], events


def reply_to(sim, line):
    return send(sim, line)[0]


def advance_with_heartbeat(sim, us):
    """Advance time the way a correct host must: pinging inside the timeout.

    This is not test scaffolding, it is the protocol's actual requirement made
    visible. At the cited 100 ms host timeout and the default 180 deg/s, any
    move beyond 18 degrees takes longer than the watchdog allows - and the
    travel range is 40 degrees wide. A task that issues a long move and then
    waits for `TARGET` without pinging gets faulted for heartbeat loss every
    time. See `../README.md`, which raises this as a parameter question for the
    draft.
    """
    events = []
    step = max(sim.host_timeout_us // 2, 1)
    remaining = int(us)
    seq = 50_000  # kept clear of the sequence numbers tests use for commands
    while remaining > 0:
        # Ping first, then wait. A host that waits first and pings afterwards
        # races its own deadline, and loses whenever some earlier call has
        # already eaten part of the window.
        seq += 1
        events += [o for o in sim.feed("ping;%d\n" % seq) if o.startswith("ev;")]
        chunk = min(step, remaining)
        events += sim.advance(chunk)
        remaining -= chunk
    return events


# ---------------------------------------------------------------- boot & ID


def test_boot_emits_ready_with_rail_off_and_driver_disabled():
    """BOOT leaves the motor rail off, the driver disabled, and emits READY."""
    sim = BoardSim(serial="0000000000000000")
    assert sim.hv_on is False
    out = sim.boot()
    assert out == ["ev;READY;0;fw=0.0.0;proto=2;serial=0000000000000000"]
    assert sim.state == State.IDLE
    assert sim.hv_on is False


def test_capability_discovery_reports_axis_count_and_board_serial(sim):
    """Gate C: capability discovery reports the axis count and the board serial.

    The serial is how a task confirms it is talking to the board whose
    calibration matches the joystick in front of it. The axis count is what
    distinguishes this board from a twin-axis board on the same link.
    """
    reply = reply_to(sim, "id;1")
    assert reply.startswith("ack;1;")
    fields = dict(f.split("=", 1) for f in reply.split(";")[2:])
    assert fields["serial"] == "0000000000000000"
    assert fields["axes"] == "1"
    assert fields["proto"] == "2"
    assert "rxbuf" in fields


def test_id_never_fails_in_any_state(sim):
    """The draft: `ID` must work before ARM and must never fail."""
    assert reply_to(sim, "id;1").startswith("ack;1;")

    sim.inject_fault(Fault.DRIVER_FAULT)
    assert sim.state == State.FAULTED
    assert reply_to(sim, "id;2").startswith("ack;2;")


def test_status_is_safe_at_any_time(sim):
    assert reply_to(sim, "status;1").startswith("ack;1;")
    sim.inject_fault(Fault.INTERNAL)
    assert reply_to(sim, "status;2").startswith("ack;2;")


# ------------------------------------------------------- framing robustness


def test_unrecognized_verb_is_rejected_never_partially_executed(sim):
    """Gate C bullet, and the draft's own example: `movetoo;420;-12.5`."""
    before = (sim.state, sim.angle, sim.hv_on)
    assert reply_to(sim, "movetoo;420;-12.5") == "nak;420;badverb"
    assert (sim.state, sim.angle, sim.hv_on) == before


@pytest.mark.parametrize(
    "line,expected",
    [
        ("ping", "nak;?;badframe"),
        ("ping;notanumber", "nak;?;badframe"),
        (";", "nak;?;badframe"),
        ("ping;1;" + "x" * 200, "nak;?;badframe"),
    ],
)
def test_malformed_input_is_nakked_and_discarded(sim, line, expected):
    """Gate C: malformed input is handled."""
    assert reply_to(sim, line) == expected


def test_partial_lines_are_buffered_until_terminated(sim):
    """A real link delivers fragments; a whole-line-only parser is untested."""
    assert sim.feed("pi") == []
    assert sim.feed("ng;") == []
    assert sim.feed("7\n") == ["ack;7"]


def test_unterminated_over_length_input_does_not_grow_without_bound(sim):
    # Length derived from MAX_LINE, which is what this actually tests. It used
    # to be derived from RETRANSMIT_CACHE_SIZE, which is unrelated -- and when
    # that constant dropped from 16 to 2 the input stopped being over-length
    # and the test passed for the wrong reason.
    out = sim.feed("x" * (P.MAX_LINE + 1))
    assert out == ["nak;?;badframe"]
    # Resynchronises on the next complete line rather than staying wedged.
    assert sim.feed("ping;1\n") == ["ack;1"]


def test_repeated_sequence_number_is_reacknowledged_without_reexecuting(armed):
    """Gate C bullet, and the draft's retransmission rule.

    The host lost the ack and retried. Executing the move twice would move the
    joystick twice, which is the failure this rule exists to prevent.
    """
    first = reply_to(armed, "moveto;10;5.0")
    assert first == "ack;10"
    motion = armed._motion
    assert motion is not None

    second = reply_to(armed, "moveto;10;5.0")
    assert second == first
    assert armed._motion is motion, "the move was re-executed"


def test_same_sequence_number_with_different_content_is_a_new_command(armed):
    """Sequence numbers wrap, so a repeat is only a retransmission if it matches."""
    assert reply_to(armed, "moveto;10;5.0") == "ack;10"
    advance_with_heartbeat(armed, 100_000)
    assert armed.angle == pytest.approx(5.0)

    assert reply_to(armed, "moveto;10;-5.0") == "ack;10"
    advance_with_heartbeat(armed, 100_000)
    assert armed.angle == pytest.approx(-5.0), "the second command was swallowed as a retransmission"


def test_out_of_order_sequence_numbers_are_each_executed(armed):
    """Gate C: out-of-order input is handled. A UART is in-order; the numbers need not be."""
    assert reply_to(armed, "moveto;900;1.0") == "ack;900"
    advance_with_heartbeat(armed, 50_000)
    assert reply_to(armed, "moveto;12;2.0") == "ack;12"
    advance_with_heartbeat(armed, 50_000)
    assert reply_to(armed, "moveto;500;3.0") == "ack;500"
    advance_with_heartbeat(armed, 50_000)
    assert armed.angle == pytest.approx(3.0)


def test_retransmission_cache_is_bounded(armed):
    """An unbounded cache is a slow leak on a device with no way to report one.

    Uses `stop`, not `ping`: reads are never cached now, so a ping loop would
    leave the cache empty and this test would pass without exercising anything.
    """
    for seq in range(BoardSim.RETRANSMIT_CACHE_SIZE + 4):
        reply_to(armed, "stop;%d" % seq)
    assert len(armed._retransmit) == BoardSim.RETRANSMIT_CACHE_SIZE


def test_reads_are_never_cached(armed):
    """The cache is restricted to state-changing commands."""
    # The `armed` fixture already ran `home` and `arm`, which are cached, so
    # compare against a snapshot rather than against an empty cache.
    before = dict(armed._retransmit)
    for seq in range(6):
        reply_to(armed, "status;%d" % seq)
        reply_to(armed, "ping;%d" % (100 + seq))
        reply_to(armed, "cal;%d;get" % (200 + seq))
    assert armed._retransmit == before, "a read was cached"


def test_a_repeated_read_is_re_executed_and_returns_fresh_state(armed):
    """A retried STATUS returning fresh state beats a replayed stale one."""
    before = reply_to(armed, "status;9")
    assert reply_to(armed, "moveto;10;5.0") == "ack;10"
    advance_with_heartbeat(armed, 100_000)
    after = reply_to(armed, "status;9")  # same seq, same content as `before`
    assert after != before, "a repeated STATUS was replayed from cache"


def test_a_repeated_relative_move_is_not_re_executed(armed):
    """MOVE is the one non-idempotent command, and the reason the cache exists."""
    assert reply_to(armed, "move;40;4.0") == "ack;40"
    advance_with_heartbeat(armed, 200_000)
    once = armed.angle
    assert reply_to(armed, "move;40;4.0") == "ack;40"  # byte-identical retry
    advance_with_heartbeat(armed, 200_000)
    assert armed.angle == pytest.approx(once), "a retried MOVE moved twice"


def test_dual_mode_verbs_are_classified_on_the_parsed_argument(armed):
    """`CAL get` is a read and `CAL set` is a write, under one verb name."""
    before = dict(armed._retransmit)
    reply_to(armed, "cal;50;get")
    assert armed._retransmit == before, "`CAL get` was cached"
    reply_to(armed, "cal;51;set;angle_zero;1.0")
    assert 51 in armed._retransmit, "`CAL set` was not cached"


# -------------------------------------------------------------- state machine


def test_motion_verbs_are_refused_outside_armed(sim):
    """Gate C bullet, and the draft's state machine."""
    assert reply_to(sim, "moveto;1;5.0") == "nak;1;notarmed"
    assert reply_to(sim, "move;2;5.0") == "nak;2;notarmed"

    reply_to(sim, "hv;3;on")
    assert reply_to(sim, "moveto;4;5.0") == "nak;4;notarmed"

    reply_to(sim, "home;5")
    sim.advance(sim.home_duration_us)
    assert sim.state == State.HOMED
    assert reply_to(sim, "moveto;6;5.0") == "nak;6;notarmed"

    reply_to(sim, "arm;7")
    assert reply_to(sim, "moveto;8;5.0") == "ack;8"


def test_arm_is_refused_if_not_homed(sim):
    reply_to(sim, "hv;1;on")
    assert reply_to(sim, "arm;2") == "nak;2;nothomed"


def test_arm_is_refused_while_faulted(sim):
    sim.inject_fault(Fault.DRIVER_FAULT)
    assert reply_to(sim, "arm;1") == "nak;1;faulted"


def test_homing_requires_the_motor_rail(sim):
    """The rail is genuinely off at boot; refusing for want of it is its own case."""
    assert reply_to(sim, "home;1") == "nak;1;nohv"


def test_disarm_and_stop_are_always_permitted(sim):
    assert reply_to(sim, "disarm;1") == "ack;1"
    assert reply_to(sim, "stop;2") == "ack;2"
    sim.inject_fault(Fault.DRIVER_FAULT)
    assert reply_to(sim, "stop;3") == "ack;3"
    assert reply_to(sim, "disarm;4") == "ack;4"


def test_move_outside_travel_limits_is_rejected_before_anything_moves(armed):
    assert reply_to(armed, "moveto;1;-999") == "nak;1;outofrange"
    assert armed._motion is None
    assert armed.angle == 0.0
    assert armed.fault_mask == 0, "a rejected command must not latch a fault"


def test_relative_move_is_rejected_on_the_resulting_position(armed):
    reply_to(armed, "moveto;1;19.0")
    advance_with_heartbeat(armed, 150_000)
    assert reply_to(armed, "move;2;5.0") == "nak;2;outofrange"
    assert reply_to(armed, "move;3;-5.0") == "ack;3"


def test_completed_move_emits_target_and_returns_to_armed(armed):
    assert reply_to(armed, "moveto;1;9.0") == "ack;1"
    assert armed.state == State.ACTIVE
    events = advance_with_heartbeat(armed, 100_000)
    assert events == ["ev;TARGET;100000;angle=9.0"]
    assert armed.state == State.ARMED
    assert armed.angle == pytest.approx(9.0)


def test_stop_during_motion_halts_where_it_got_to(armed):
    reply_to(armed, "moveto;1;18.0")  # 18 deg at 180 deg/s = 100 ms, the whole timeout
    armed.advance(50_000)
    assert reply_to(armed, "stop;2") == "ack;2"
    assert armed.angle == pytest.approx(9.0)
    assert armed.state == State.ARMED
    assert advance_with_heartbeat(armed, 200_000) == [], "a stopped move must not later report TARGET"
    assert armed.angle == pytest.approx(9.0)


# ------------------------------------------------------------ faults & timeouts


def test_heartbeat_timeout_faults_and_is_observable(armed):
    """Gate C: timeouts and faults are observable."""
    out = armed.advance(armed.host_timeout_us + 1)
    assert len(out) == 1
    assert out[0].startswith("ev;FAULT;")
    assert "code=0" in out[0]
    assert armed.state == State.FAULTED
    assert armed.fault_mask & (1 << Fault.HOST_TIMEOUT)
    assert armed.hv_on is False, "a fault must drop HV_EN"


def test_ping_resets_the_host_loss_watchdog(armed):
    for seq in range(5):
        armed.advance(armed.host_timeout_us - 10_000)
        assert reply_to(armed, "ping;%d" % seq) == "ack;%d" % seq
    assert armed.state == State.ARMED
    assert armed.fault_mask == 0


def test_a_repeated_ping_still_resets_the_watchdog(armed):
    """Resolved 2026-09-03: the watchdog is reset by arrival, not by execution.

    The draft reset it inside PING's handler, so a PING served from the
    retransmission cache did not reset it -- and the host, receiving acks,
    could not see the safety timeout running out. The reset now happens in the
    receive path before dispatch, so a cache hit is irrelevant to it and PING
    needs no exemption.

    Byte-identical retries here, which is the case a retry burst on a degraded
    link actually produces.
    """
    for _ in range(5):
        armed.advance(armed.host_timeout_us - 10_000)
        assert reply_to(armed, "ping;7") == "ack;7"  # the same seq every time
    assert armed.state == State.ARMED, "a repeated PING was served from cache and the watchdog lapsed"
    assert armed.fault_mask == 0


def test_any_well_formed_command_resets_the_watchdog(armed):
    """Not just PING. Liveness is proven by a frame arriving, not by PING.

    This is what makes open question 2 survivable: a host polling STATUS during
    a long move keeps the link alive, where under the draft only PING did.
    """
    for seq in range(5):
        armed.advance(armed.host_timeout_us - 10_000)
        assert reply_to(armed, "status;%d" % seq).startswith("ack;")
    assert armed.state == State.ARMED
    assert armed.fault_mask == 0


def test_malformed_frames_do_not_hold_the_watchdog_open(armed):
    """A host with a wedged serial driver is not usefully alive.

    The refresh sits after the badframe branch deliberately: garbage arriving
    must not keep the safety timeout from expiring.
    """
    for _ in range(5):
        armed.advance(armed.host_timeout_us - 10_000)
        out = armed.feed("!!! not a command\n")
        assert any("badframe" in line for line in out)
    armed.advance(armed.host_timeout_us)
    assert armed.fault_mask & (1 << Fault.HOST_TIMEOUT), (
        "malformed frames kept the host-loss watchdog alive"
    )


def test_watch_thresholds_are_relative_to_the_release_angle(armed):
    """WATCH (protocol v1): pull and push are excursions from the angle at
    RELEASED, pull-positive, not absolute angles; a push past the pull
    magnitude is a PUSH, and the classifier fires once."""
    reply_to(armed, "cfg;1;wdt_ms=500")
    reply_to(armed, "moveto;2;-9.0")
    armed.advance(50_000)
    assert reply_to(armed, "brake;3;0") == "ack;3"
    assert reply_to(armed, "watch;4;pull=5;push=5;timeout=300") == "ack;4"

    # Towards the push side by more than the pull magnitude: PUSH, not PULL.
    events = armed.move_joystick(-15.0)
    assert any(e.startswith("ev;PUSH;") and "angle=-15.0" in e for e in events)
    # Latched: one WATCH, one outcome.
    assert armed.move_joystick(-3.0) == []


def test_watch_pull_is_relative_and_pull_positive(armed):
    """The same thresholds from a different release angle detect the pull."""
    reply_to(armed, "cfg;1;wdt_ms=500")
    reply_to(armed, "moveto;2;-9.0")
    armed.advance(50_000)
    reply_to(armed, "brake;3;0")
    reply_to(armed, "watch;4;pull=5;push=5;timeout=300")

    assert armed.move_joystick(-5.0) == [], "4 deg of pull is under the 5 deg threshold"
    events = armed.move_joystick(-3.0)
    assert any(e.startswith("ev;PULL;") and "angle=-3.0" in e for e in events)


def test_delayed_command_after_a_timeout_is_refused_not_executed(armed):
    """Gate C: delayed input is handled."""
    armed.advance(armed.host_timeout_us + 1)
    assert reply_to(armed, "moveto;1;5.0") == "nak;1;faulted"
    assert armed.angle == 0.0


@pytest.mark.parametrize("bit", sorted(Fault.NAMES))
def test_every_fault_code_can_be_provoked_and_observed(sim, bit):
    """Gate C: every fault code can be provoked and observed."""
    out = sim.inject_fault(bit)
    assert len(out) == 1
    assert "code=%d" % bit in out[0]
    assert sim.state == State.FAULTED

    faults = reply_to(sim, "faults;1")
    assert faults.startswith("ack;1;mask=%d" % (1 << bit))
    assert Fault.NAMES[bit] in faults


def test_bit_four_is_reserved_forever(sim):
    """ENCODER_MISMATCH was retired 2026-08-16: there is no second encoder count.

    The draft says to leave the bit permanently reserved so no later fault reuses
    it. This test is the thing that makes "reserved" enforceable.
    """
    assert 4 not in Fault.NAMES
    assert not hasattr(Fault, "ENCODER_MISMATCH")
    with pytest.raises(ValueError):
        sim.inject_fault(4)


def test_vsense_self_test_failure_reports_on_bit_seven(sim):
    """Gate C: including the `vsense` self-test path in bit 7.

    vsense resets to 0, which with the 0.15 ohm shunts drives ~1.8x the motor's
    rating. Firmware reads CHOPCONF back before enabling the driver and refuses
    if it does not say 1.
    """
    reply_to(sim, "hv;1;on")
    reply_to(sim, "home;2")
    sim.advance(sim.home_duration_us)

    sim.vsense_ok = False
    reply, events = send(sim, "arm;3")
    assert reply == "nak;3;faulted"
    assert events == ["ev;FAULT;50000;code=7;detail=vsense_not_1"]
    assert sim.fault_mask & (1 << Fault.INTERNAL)
    assert sim.fault_detail[Fault.INTERNAL] == "vsense_not_1"
    assert sim.state == State.FAULTED


def test_faulted_is_exited_only_through_clrfault(sim):
    """Gate C: faults recover only through declared paths."""
    sim.inject_fault(Fault.DRIVER_FAULT, cause_active=True)

    for verb in ("arm;1", "home;2", "moveto;3;1.0"):
        assert reply_to(sim, verb).startswith("nak;")
    assert sim.state == State.FAULTED

    # The cause is still present, so the fault stays latched.
    assert reply_to(sim, "clrfault;4") == "nak;4;latched"
    assert sim.state == State.FAULTED

    sim.clear_fault_cause(Fault.DRIVER_FAULT)
    assert reply_to(sim, "clrfault;5") == "ack;5"
    assert sim.state == State.IDLE
    assert sim.fault_mask == 0
    assert sim.homed is False, "clearing a fault must not leave a stale homed flag"


def test_clrfault_clears_only_the_causes_that_have_gone(sim):
    sim.inject_fault(Fault.DRIVER_FAULT, cause_active=True)
    sim.inject_fault(Fault.SUPPLY_OUT_OF_RANGE, cause_active=False)

    assert reply_to(sim, "clrfault;1") == "nak;1;latched"
    assert not sim.fault_mask & (1 << Fault.SUPPLY_OUT_OF_RANGE)
    assert sim.fault_mask & (1 << Fault.DRIVER_FAULT)


def test_limit_asserted_outside_homing_is_a_fault(sim):
    out = sim.inject_limit()
    assert out[0].startswith("ev;LIMIT;")
    assert "code=%d" % Fault.LIMIT_UNEXPECTED in out[1]


# ----------------------------------------------------------- analog output

# ANGOUT and ANGSCALE have no tests because they have no implementation. The
# analog outputs are not populated on these boards, so a test would pin what was
# guessed. They come back with the hardware, and with the second channel -
# position and torque both stream continuously.


def test_analog_output_verbs_are_absent_not_stubbed(sim):
    """An unimplemented verb must be `badverb`, not a silent ack.

    A stub that acks would let a host believe the analog output is live and
    record a trace of nothing.
    """
    assert reply_to(sim, "angout;1;on") == "nak;1;badverb"
    assert reply_to(sim, "angscale;2;-10;10") == "nak;2;badverb"


# -------------------------------------------------------------- compliance


def test_brake_requires_arming_first(sim):
    """A joystick held by an energised motor cannot go compliant by accident."""
    assert reply_to(sim, "brake;1;0") == "nak;1;notarmed"


def test_brake_releases_the_driver(armed):
    reply, events = send(armed, "brake;1;0")
    assert reply == "ack;1"
    assert events == ["ev;RELEASED;50000;angle=0.0;torque=;brake="]
    assert armed.state == State.RELEASED


def test_brake_is_refused_during_motion(armed):
    reply_to(armed, "moveto;1;9.0")
    assert reply_to(armed, "brake;2;0") == "nak;2;busy"


def test_the_animal_can_only_move_a_released_joystick(armed):
    """Modelling a "pull" against an energised motor would test the impossible."""
    with pytest.raises(AssertionError, match="only be moved by hand"):
        armed.move_joystick(5.0)


def test_a_pull_moves_the_encoder_and_strands_the_ramp_generator(armed):
    """The divergence that the previous rig had no way to see.

    Without an encoder every pull lost the position outright, which is why that
    rig homed every trial. Here the encoder keeps it - and the ramp generator
    does not, because nothing turns it while the driver is released.
    """
    reply_to(armed, "brake;1;0")
    armed.move_joystick(-7.5)

    assert armed.angle == pytest.approx(-7.5), "the encoder must follow the joystick"
    assert armed.xactual == pytest.approx(0.0), "nothing turns the ramp generator while compliant"


def test_leaving_compliance_reloads_position_from_the_encoder(armed):
    """Resync happens on the way out, and only on the way out.

    Doing it continuously would delete the board's only stall detector:
    `XACTUAL` advancing while `X_ENC` does not is how an end stop, a stall, a
    slipping coupling or a dead encoder are detected. The two cases are told
    apart by state and by nothing else.
    """
    reply_to(armed, "brake;1;0")
    armed.move_joystick(-7.5)
    assert armed.xactual == pytest.approx(0.0)

    reply_to(armed, "stop;2")
    assert armed.state == State.ARMED
    assert armed.xactual == pytest.approx(-7.5), "position was not reloaded from the encoder"
    assert armed.angle == pytest.approx(-7.5)


def test_a_move_after_a_pull_lands_where_it_was_told(armed):
    """The reason the resync matters, stated as a consequence rather than a mechanism.

    An absolute move must reach the absolute position, whatever the animal did
    in between. Skipping the resync would offset every trial by the last pull.
    """
    reply_to(armed, "brake;1;0")
    armed.move_joystick(-15.0)

    assert reply_to(armed, "moveto;2;5.0") == "ack;2"
    advance_with_heartbeat(armed, 200_000)
    assert armed.angle == pytest.approx(5.0)
    assert armed.xactual == pytest.approx(5.0)


def test_a_relative_move_after_a_pull_is_relative_to_the_joystick(armed):
    """Relative to where the joystick IS, not where the ramp generator believes."""
    reply_to(armed, "brake;1;0")
    armed.move_joystick(-10.0)

    assert reply_to(armed, "move;2;2.5") == "ack;2"
    advance_with_heartbeat(armed, 200_000)
    assert armed.angle == pytest.approx(-7.5)


def test_home_still_works_after_a_pull(armed):
    """Homing is now for parking the joystick, not for recovering position.

    Note the sequence number: the `armed` fixture already sent `home;2`, and a
    second `home;2` with identical content is a retransmission by definition -
    re-acknowledged without re-executing, which is correct and would make this
    test silently assert nothing. A real host increments monotonically, as the
    device class does.
    """
    reply_to(armed, "brake;10;0")
    armed.move_joystick(12.0)

    assert reply_to(armed, "home;11") == "ack;11"
    advance_with_heartbeat(armed, 100_000)
    assert armed.angle == pytest.approx(0.0)
    assert armed.xactual == pytest.approx(0.0)
    assert armed.homed is True


def test_watch_reports_one_outcome_per_arming(armed):
    """One WATCH, one outcome - not a stream while the joystick sits past the
    threshold - and a new WATCH re-arms for the next trial, relative to the
    angle at RELEASED."""
    reply_to(armed, "cfg;1;wdt_ms=500")
    reply_to(armed, "brake;2;0")
    reply_to(armed, "watch;3;pull=5;push=5;timeout=300")

    assert armed.move_joystick(2.0) == [], "below threshold"
    out = armed.move_joystick(6.0)
    assert len(out) == 1 and out[0].startswith("ev;PULL;")
    assert armed.move_joystick(7.0) == [], "already reported"

    armed.move_joystick(2.0)  # the joystick comes back
    reply_to(armed, "watch;4;pull=5;push=5;timeout=300")
    assert armed.move_joystick(4.9) == [], "4.9 deg from the release angle is under the threshold"
    out = armed.move_joystick(8.0)
    assert len(out) == 1 and out[0].startswith("ev;PULL;"), "re-arming must allow the next pull"


# ------------------------------------------------------------------ two boards


def test_two_simulator_instances_on_two_links_do_not_interfere():
    """Gate C: two simulator instances on two links do not interfere.

    This is the two-board case. Each board has its own clock, so cross-joystick
    ordering is a post-processing question answered through RSYNC, not something
    either board can assert.
    """
    left = BoardSim(serial="AAAAAAAAAAAAAAAA")
    right = BoardSim(serial="BBBBBBBBBBBBBBBB")
    left.boot()
    right.boot()

    assert "serial=AAAAAAAAAAAAAAAA" in reply_to(left, "id;1")
    assert "serial=BBBBBBBBBBBBBBBB" in reply_to(right, "id;1")

    left.inject_fault(Fault.DRIVER_FAULT)
    assert left.state == State.FAULTED
    assert right.state == State.IDLE
    assert right.fault_mask == 0

    left.advance(1_000_000)
    assert right.now_us == 0, "the two clocks are independent"


# ------------------------------------------------- 2026-10-01 review bugs


def test_event_timestamps_do_not_wrap_at_32_bits(sim):
    """Bug 1: the firmware cast `t` to 32 bits and wrapped at 71.6 minutes."""
    send(sim, "hv;1;on")
    sim.advance(4294967296)  # 2^32 us into the session
    send(sim, "home;2")
    out = sim.advance(50000)
    assert out == ["ev;HOMED;4295017296;angle=0.0"]


def test_a_home_timeout_shorter_than_the_home_faults_when_it_expires(sim):
    """Bug 2: the timeout is a deadline on the running home, not an up-front
    comparison with the expected duration."""
    send(sim, "hv;1;on")
    assert send(sim, "home;2;30") == ("ack;2", [])
    assert sim.advance(29999) == []
    assert sim.advance(1) == ["ev;FAULT;30000;code=3;detail=home_timeout"]
    assert sim.state == State.FAULTED


def test_a_home_timeout_longer_than_the_home_is_not_a_fault(sim):
    send(sim, "hv;1;on")
    assert send(sim, "home;2;2000") == ("ack;2", [])
    assert sim.advance(50000) == ["ev;HOMED;50000;angle=0.0"]
    assert sim.advance(2000000) == []
    assert sim.state == State.HOMED


def test_a_home_timeout_is_clipped_to_the_firmware_maximum(sim):
    send(sim, "hv;1;on")
    assert send(sim, "home;2;999999") == ("ack;2", [])
    assert sim._home_deadline == sim.now_us + sim.home_timeout_max_us


def test_a_late_tick_fills_the_grid_and_fetch_counts_it():
    """The simulator's ring mirrors the firmware's grid fill: a sample that
    arrives more than one period late fills the missed points and the window
    reports them as lost (the corpus cannot reach this; advance() never skips
    a grid point)."""
    from sim.board_sim import _TrialWatch

    w = _TrialWatch()
    w.set_window(10, 2)
    w.release(1_000_000, 0.0, 1.0)
    w.arm(1_000_000, {"pull": 5.0, "push": 5.0, "timeout_us": 1_000_000, "hold_us": 0, "maxdur_us": None}, 0.5)
    for k in range(5):
        w.sample(1_000_000 + k * 1000, 0.0)
        w.evaluate(1_000_000 + k * 1000, 0.0)
    w.sample(1_008_000, 6.0)  # 3 ms late: grid points +5, +6, +7 are fills
    assert w.evaluate(1_008_000, 6.0) == "PULL"
    for k in (9, 10):
        w.sample(1_000_000 + k * 1000, 6.0)
    assert w.has_window()
    assert (w.window_pre(), w.window_count(), w.window_lost()) == (8, 11, 3)
    assert w.window_sample(4) == 0.0 and w.window_sample(5) == 6.0
