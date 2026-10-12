// pyControl UART protocol — the board's command layer and state machine.
//
// The firmware counterpart of `pycontrol/sim/board_sim.py`. Both are held to
// `pycontrol/tests/fixtures/protocol_cases.txt`; this is the half that made the
// corpus worth having, because a corpus only one implementation has ever been
// checked against proves nothing.
//
// Arduino-free and allocation-free for the reasons in `Protocol.hpp`. This
// class owns framing, the state machine, the host-loss watchdog and the
// retransmission cache, and it owns NO motion: everything about where the
// joystick is and how it gets somewhere sits behind `MotionBackend`.
//
// That seam is what lets one implementation serve both purposes.
// `SimulatedMotion` runs the shared corpus on the host with an injected clock;
// `TmcMotion` drives the real TMC5130A without blocking. Nothing in this file
// knows which it has.
//
// !! `travel_lo`/`travel_hi`/`v_max` BELOW ARE THE SIMULATOR'S PLACEHOLDERS.
//    They are the limits LIMITS reports and MOVETO enforces, and they must
//    agree with whatever the backend can actually do -- a board that reports
//    +/-20 deg while the mechanism has 28 deg of travel would refuse valid
//    positions or command impossible ones. `test_protocol.cpp` asserts the
//    default configuration is self-consistent. The measured bench figures are
//    in `mouse-joystick-pcb/docs/bringup-log.md` under board 2.

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "BrakeTorqueBackend.hpp"
#include "MotionBackend.hpp"
#include "Protocol.hpp"
#include "TrialWatch.hpp"

namespace mouse_joystick {

namespace state_name {
constexpr char kBoot[] = "BOOT";
constexpr char kIdle[] = "IDLE";
constexpr char kHomed[] = "HOMED";
constexpr char kArmed[] = "ARMED";
constexpr char kActive[] = "ACTIVE";
constexpr char kFaulted[] = "FAULTED";
// The state in which the ANIMAL drives the joystick: the brake is engaged
// (where there is one) and the driver released, so motor torque does not add
// to the brake's and the ramp generator's position stops meaning anything --
// only the encoder knows where the joystick is. Called FREEWHEEL until
// protocol v1 (2026-10-01), when BRAKE replaced the FREEWHEEL verb.
constexpr char kReleased[] = "RELEASED";
}  // namespace state_name

// Fault bits. Bit 4 is retired and permanently reserved: it was a driver-encoder
// vs. RP2040-PIO-count comparison, and that PIO count never existed on any
// revision. Do not reuse it -- a stale host would misread the mask.
namespace fault {
constexpr uint8_t kHostTimeout = 0;
constexpr uint8_t kDriverFault = 1;
constexpr uint8_t kOverTravel = 2;
constexpr uint8_t kHomeTimeout = 3;
constexpr uint8_t kReservedForever = 4;
constexpr uint8_t kLimitUnexpected = 5;
constexpr uint8_t kSupplyOutOfRange = 6;
constexpr uint8_t kInternal = 7;
constexpr uint8_t kExtStop = 8;
// Protocol v1, bits 9 and up (uart-protocol.md V1.7), pcb v1.0's brake sink
// and torque bridge. Raised by the BrakeTorqueBackend through consumeFault().
constexpr uint8_t kBrakeFault = 9;    // ~{BRAKE_FAULT} asserted, or no brake to drive
constexpr uint8_t kBrakeCurrent = 10; // BRAKE_ISENSE away from the setpoint too long
constexpr uint8_t kTorqueAdc = 11;    // ADS8689 not answering
constexpr uint8_t kTorqueRange = 12;  // bridge saturated
}  // namespace fault

class ProtocolBoard {
public:
  static constexpr uint8_t kMaxPendingLines =
      36; // a FETCH chunk: ack + 32 cap lines + cap;end
  static constexpr uint8_t kRetransmitCacheSize = 2;
  static constexpr uint32_t kDefaultHostTimeoutUs = 100000u;
  static constexpr uint32_t kDefaultHomeDurationUs = 50000u;

  struct Config {
    // Deliberately NOT a plausible serial. `ID` reports this so a task can
    // confirm it is talking to the board whose calibration matches the joystick
    // in front of it, and with two interchangeable single-axis boards nothing
    // else distinguishes them. A default that looked like a real flash unique id
    // meant a firmware that forgot to set it reported the same value on every
    // board -- and the host's identity check then PASSED against the wrong
    // board. Found on hardware 2026-09-03, in the first READY event this
    // firmware ever emitted. Fail loudly instead.
    const char *serial = "SERIAL-NOT-SET";
    const char *fw_version = "0.0.0";
    const char *hw_rev = "rev0";
    double travel_lo = -20.0;
    double travel_hi = 20.0;
    double v_max = 180.0;
    double a_max = 3600.0;
    double i_max = 0.421;
    uint32_t host_timeout_us = kDefaultHostTimeoutUs;
    // Protocol v1 capabilities, reported in `caps` and gated on: hardware facts
    // of the board revision, set by the application.
    bool has_brake = false;
    bool has_torque = false;
    bool has_angout = false;
    // Whether `INSTR on/off` can switch anything. Rev 0 and rev 1 drive
    // ~{INSTR_BUF_OE}; pcb v1.0's Digital-50 port has no enable, so there the
    // verb is accepted and `state` always reports 1 (uart-protocol.md, INSTR,
    // 2026-10-01). The simulator keeps the switchable default.
    bool instr_switchable = true;
    // Which encoder direction is "pull": +1 if increasing angle is a pull.
    // A board-and-harness fact (bringup-log.md); WATCH thresholds and the
    // TRIAL excursions are expressed pull-positive through it.
    double pull_sign = 1.0;
    // ID boot= and crumb=: the reset reason ("por", "wdt", "reboot") and the
    // last bring-up breadcrumb before a watchdog reboot; the simulator and the
    // non-QP sketch report the defaults.
    const char *boot_reason = "por";
    uint32_t boot_crumb = 0;
  };

  // CFG bounds (protocol v1).
  static constexpr uint32_t kWdtMinUs = 50000u;
  static constexpr uint32_t kWdtMaxUs = 500000u;
  static constexpr double kDeadbandMinDeg = 0.05;
  static constexpr double kDeadbandMaxDeg = 10.0;
  static constexpr uint16_t kStreamHzMin = 1;
  static constexpr uint16_t kStreamHzMax = 50; // at 115200 baud
  static constexpr uint16_t kFetchChunkMax = 32;
  // `home_duration_us` and the vsense read-back used to live here. They are
  // properties of the thing that moves, so they belong to the backend:
  // `SimulatedMotion::Config` for the host, a CHOPCONF read-back for the real
  // driver. Asking the backend keeps one answer instead of two that can drift.

  // The backend is a reference, not owned: the caller decides its lifetime and
  // its identity, which is the whole point of the seam. Two constructors rather
  // than one with `= Config()`, because a default argument naming a nested type
  // needs the enclosing class to be complete and it is not at this point.
  explicit ProtocolBoard(MotionBackend &motion) : motion_(motion) {}
  // `brake_torque` is pcb v1.0's brake sink and torque bridge, or null on a
  // board without them. `cfg.has_brake`/`has_torque` decide what `caps`
  // reports and whether the `brake`/`torque` fields carry numbers; a config
  // that claims either with no backend reports zeros and faults BRAKE.
  ProtocolBoard(MotionBackend &motion, const Config &config,
                BrakeTorqueBackend *brake_torque = nullptr)
      : cfg_(config), motion_(motion), brake_torque_(brake_torque) {}

  // ------------------------------------------------------------------- clock
  //
  // Time is 64-bit microseconds on purpose. A 32-bit microsecond counter wraps
  // at 71.6 minutes, which is shorter than a session, and every event on this
  // link carries `t_us` into a recorded dataset used for alignment. A wrap
  // mid-session would not fail loudly -- it would produce timestamps that go
  // backwards. Firmware feeding this from `micros()` must accumulate the wrap;
  // it must not hand the raw 32-bit value straight through.

  uint64_t nowUs() const { return now_us_; }

  // The draft: emit READY once after boot; the host should reconnect and re-ID.
  void boot() {
    state_ = state_name::kIdle;
    protocol::Line line;
    line.beginEvent("READY", now_us_);
    line.kvStr("fw", cfg_.fw_version);
    line.kvInt("proto", protocol::kProtocolVersion);
    line.kvStr("serial", cfg_.serial);
    push(line);
  }

  // Advance to the next scheduled thing rather than in one jump, so an event
  // never lands at the wrong timestamp because the caller advanced coarsely.
  void advance(uint64_t delta_us) {
    // One advance, however many stops: the hardware backends read their
    // buses on the first service() below and answer the rest from that.
    motion_.beginAdvance();
    if (brake_torque_ != nullptr) {
      brake_torque_->beginAdvance();
    }
    uint64_t target = now_us_ + delta_us;
    bool serviced_at_target = false; // one service per advance (2026-10-02)
    while (now_us_ < target) {
      uint64_t next_stop = target;
      // A backend that can predict its completion gets stepped to exactly that
      // microsecond, so an event never lands late because the caller advanced
      // coarsely. A real backend returns 0 and is simply polled.
      const uint64_t motion_at = motion_.nextEventUs();
      if (motion_at != 0 && motion_at < next_stop) {
        next_stop = motion_at;
      }
      if (watchdog_armed_ && watchdog_deadline_us_ < next_stop) {
        next_stop = watchdog_deadline_us_;
      }
      if (home_deadline_armed_ && home_deadline_us_ < next_stop) {
        next_stop = home_deadline_us_;
      }
      const uint64_t watch_at = watch_.nextStopUs(now_us_);
      if (watch_at != 0 && watch_at < next_stop) {
        next_stop = watch_at;
      }
      if (brake_torque_ != nullptr) { // a BRAKE settling: RELEASED on time
        const uint64_t brake_at = brake_torque_->nextEventUs();
        if (brake_at != 0 && brake_at < next_stop) {
          next_stop = brake_at;
        }
      }
      if (stream_on_ && stream_next_us_ < next_stop) {
        next_stop = stream_next_us_;
      }
      now_us_ = next_stop;
      service();
      if (next_stop == target) {
        serviced_at_target = true;
        break;
      }
    }
    now_us_ = target;
    if (!serviced_at_target) {
      service(); // a zero advance, or no stop at the target: service once
    }
  }

  // Firmware sets this once, in setup(), from the flash unique id. Not done in
  // the constructor because reading it issues a real QSPI command and the board
  // is constructed before the runtime is fully up.
  // ID's boot= and crumb= (2026-10-02): the reset reason and the last
  // bring-up breadcrumb, captured once at boot by the board support.
  void setBootInfo(const char *reason, uint32_t crumb) {
    if (reason != nullptr && *reason != '\0') {
      cfg_.boot_reason = reason;
    }
    cfg_.boot_crumb = crumb;
  }

  // The pull direction for this board (BoardCalibration.hpp), applied with
  // the serial: +1 increasing angle, -1 decreasing.
  void setPullSign(double sign) { cfg_.pull_sign = (sign < 0.0) ? -1.0 : 1.0; }

  void setSerial(const char *serial) {
    if (serial != nullptr && *serial != '\0') {
      cfg_.serial = serial;
    }
  }

  // Firmware registers its microsecond clock here (2026-10-06). Some
  // commands block in the backend -- `hv on` is the rail's soft start, 185 ms
  // on pcb v1.0; `home` and `arm` tens of ms; a `stop` from speed the ramp's
  // deceleration -- while this clock stands at the line's arrival and the
  // host's lines queue up unread. With the watchdog armed from `hv on` that
  // showed twice on Gate E (11:16, 11:25): a deadline armed by the blocked
  // command read as expired on the first advance after it, and heartbeats
  // that arrived during a blocked `home` were read only after the deadline
  // check. After every dispatched line the clock is moved to the wall and
  // the time the board spent blocked is credited to the deadlines then
  // running (host watchdog, home bound): the host could not be heard during
  // it. The simulation registers no clock and the corpus is untouched.
  void setWallClock(uint64_t (*clock)()) { wall_clock_ = clock; }

  // Advance to an absolute time, which is what firmware has. The delta
  // arithmetic lives here so a caller cannot hand over a negative one, and a
  // clock that has not moved is a no-op rather than a rewind.
  void advanceTo(uint64_t now_us) {
    if (now_us > now_us_) {
      advance(now_us - now_us_);
    }
  }

  // -------------------------------------------------------------------- link

  // Accepts partial lines, because a real link delivers them that way and a
  // parser that only works on whole lines is a parser that has not been tested.
  void feed(const char *data) {
    if (data == nullptr) {
      return;
    }
    for (const char *p = data; *p != '\0'; ++p) {
      if (*p == '\n') {
        rx_[rx_len_] = '\0';
        handleLine(rx_);
        rx_len_ = 0;
        rx_overflow_ = false;
        continue;
      }
      if (rx_len_ < sizeof(rx_) - 1) {
        rx_[rx_len_++] = *p;
      } else {
        rx_overflow_ = true;
      }
    }
    // An over-length fragment with no terminator would otherwise grow without
    // bound waiting for one. Answer it and resynchronise at the next newline.
    if (rx_overflow_ || rx_len_ > protocol::kMaxLine) {
      rx_len_ = 0;
      rx_overflow_ = false;
      protocol::Line line;
      line.nakUnknown(protocol::kNakBadFrame);
      push(line);
    }
  }

  // ------------------------------------------------------------------ output

  uint8_t outputCount() const { return pending_count_; }
  const char *output(uint8_t index) const {
    return index < pending_count_ ? pending_[index] : "";
  }
  void clearOutput() { pending_count_ = 0; }
  bool outputOverflowed() const { return pending_overflow_; }

  // ------------------------------------------------------------- observation

  const char *state() const { return state_; }

  // `INSTR ON/OFF` sets this; the application drives the buffers from it (the
  // protocol layer is Arduino-free). Seed it with the board's boot state, so
  // `INSTR STATE` reports what the port is doing rather than what the host
  // last asked. Before 2026-10-01 the verb only set the flag (review bug 3).
  bool instrumentationOn() const { return instr_on_; }
  void setInstrumentationOn(bool on) { instr_on_ = on || !cfg_.instr_switchable; }

  // ANGOUT/ANGSCALE (protocol v1): the application drives the DAC from these
  // once `cfg_.has_angout` is set. The output is valid only while homed,
  // unfaulted and on; otherwise the application parks it at the fault level.
  bool angoutOn() const { return angout_on_; }
  double angleScaleLo() const { return angscale_lo_; }
  double angleScaleHi() const { return angscale_hi_; }
  bool angleOutputValid() const {
    return angout_on_ && homed_ && hv_on_ && !faulted();
  }
  double angle() const { return motion_.angleDeg(); }
  double xactual() const { return motion_.xactualDeg(); }
  bool homed() const { return homed_; }
  bool hvOn() const { return hv_on_; }
  uint32_t faultMask() const { return fault_mask_; }
  // The brake (protocol v1, a board with one): the commanded level while it
  // engages or holds in RELEASED, 0 otherwise; and whether a BRAKE is still
  // settling. The latest tared torque sample, 0.0 without the bridge.
  double brakeLevel() const { return brake_level_; }
  bool brakeEngaging() const { return brake_pending_; }
  double torque() const { return currentTorque(); }

private:
  // ------------------------------------------------------------- line intake

  void handleLine(const char *line) {
    protocol::Command cmd;
    if (!protocol::decodeCommand(line, &cmd)) {
      // Deliberately BEFORE the watchdog refresh: a frame that does not parse
      // does not prove the host is alive. A wedged serial driver emitting
      // garbage must not hold the safety timeout open. Resolved 2026-09-03 in
      // `mouse-joystick-pcb/docs/uart-protocol.md` open question 1.
      protocol::Line reply;
      if (cmd.has_seq) {
        reply.nak(cmd.seq, protocol::kNakBadFrame);
      } else {
        reply.nakUnknown(protocol::kNakBadFrame);
      }
      push(reply);
      return;
    }

    // The host-loss watchdog is reset HERE -- on receipt of a well-formed
    // command, before dispatch, cache hit or miss -- and not inside any
    // handler. Liveness is a property of the link, so it is decided where the
    // frame arrives. Putting it inside PING made the retransmission rule a
    // safety rule by accident.
    refreshWatchdog();

    const bool cacheable =
        protocol::changesState(cmd.verb, cmd.arg_count > 0 ? cmd.arg(0) : "");

    char canonical[protocol::kMaxLine + 2];
    buildCanonical(cmd, canonical, sizeof(canonical));

    if (cacheable) {
      for (uint8_t i = 0; i < cache_count_; ++i) {
        if (cache_[i].seq == cmd.seq &&
            strcmp(cache_[i].canonical, canonical) == 0) {
          // The one command in the set this actually protects is relative
          // MOVE, which would double the movement. Everything else names a
          // destination or a target state and is idempotent.
          protocol::Line replay;
          replay.clear();
          replay.append(cache_[i].reply);
          push(replay);
          return;
        }
      }
    }

    protocol::Line reply;
    const bool answered = dispatch(cmd, &reply);
    syncClock(); // the handler may have blocked; see setWallClock()
    if (!answered) {
      return;
    }
    if (cacheable) {
      remember(cmd.seq, canonical, reply.c_str());
    }
    push(reply);
  }

  static void buildCanonical(const protocol::Command &cmd, char *out,
                             size_t n) {
    // verb;seq[;arg...] -- the same normalisation the Python side uses, so a
    // retransmission is detected on content and not on raw bytes.
    protocol::Line line;
    line.clear();
    line.append(cmd.verb);
    line.appendChar(protocol::kDelimiter);
    line.appendInt(cmd.seq);
    for (uint8_t i = 0; i < cmd.arg_count; ++i) {
      line.appendChar(protocol::kDelimiter);
      line.append(cmd.arg(i));
    }
    protocol::copyText(out, n, line.c_str());
  }

  void remember(uint16_t seq, const char *canonical, const char *reply) {
    for (uint8_t i = 0; i < cache_count_; ++i) {
      if (cache_[i].seq == seq) {
        protocol::copyText(cache_[i].canonical, sizeof(cache_[i].canonical),
                           canonical);
        protocol::copyText(cache_[i].reply, sizeof(cache_[i].reply), reply);
        return;
      }
    }
    if (cache_count_ < kRetransmitCacheSize) {
      ++cache_count_;
    } else {
      // Evict the oldest. The case the cache exists for is "the host retried
      // immediately because a reply was lost", so depth 2 is ample and a deep
      // seq-keyed window is the only thing that makes wrap reuse reachable.
      for (uint8_t i = 1; i < kRetransmitCacheSize; ++i) {
        cache_[i - 1] = cache_[i];
      }
    }
    Entry &slot = cache_[cache_count_ - 1];
    slot.seq = seq;
    protocol::copyText(slot.canonical, sizeof(slot.canonical), canonical);
    protocol::copyText(slot.reply, sizeof(slot.reply), reply);
  }

  void push(const protocol::Line &line) {
    if (line.empty()) {
      return;
    }
    if (pending_count_ >= kMaxPendingLines || line.overflowed()) {
      // Never emit a truncated line: a host cannot tell a truncated reply from
      // a different one, and this protocol is compared byte for byte.
      pending_overflow_ = true;
      return;
    }
    if (!protocol::copyText(pending_[pending_count_], sizeof(pending_[0]),
                            line.c_str())) {
      pending_overflow_ = true; // cannot happen: a Line fits its slot
      return;
    }
    ++pending_count_;
  }

  void emitEvent(const char *name, const char *key, double value) {
    protocol::Line line;
    line.beginEvent(name, now_us_);
    if (key != nullptr) {
      line.kvNumber(key, value);
    }
    push(line);
  }

  // --------------------------------------------------------------- scheduler

  void service() {
    motion_.poll(now_us_);
    if (brake_torque_ != nullptr) {
      brake_torque_->poll(now_us_);
    }

    // A backend fault outranks a completion: if the driver reported a short to
    // ground on the same poll that the move finished, the fault is what
    // happened and TARGET would be a lie.
    uint8_t fault_bit = 0;
    const char *fault_detail = nullptr;
    if (!faulted() && motion_.consumeFault(&fault_bit, &fault_detail)) {
      raiseFault(fault_bit, fault_detail != nullptr ? fault_detail : "motion",
                 /*cause_active=*/false);
      return;
    }
    // The brake and torque parts live on the switched domain: a fault they
    // report with the rail down (the collapse itself saturates the ADC for a
    // tick) is consumed and dropped, not raised.
    if (brake_torque_ != nullptr &&
        brake_torque_->consumeFault(&fault_bit, &fault_detail)) {
      if (hv_on_ && !faulted()) {
        raiseFault(fault_bit,
                   fault_detail != nullptr ? fault_detail : "brake_torque",
                   /*cause_active=*/false);
        return;
      }
    }

    bool was_home = false;
    if (motion_.consumeCompletion(&was_home)) {
      if (was_home) {
        home_deadline_armed_ = false;
        homed_ = true;
        state_ = state_name::kHomed;
        emitEvent("HOMED", "angle", motion_.angleDeg());
      } else {
        state_ = state_name::kArmed;
        emitEvent("TARGET", "angle", motion_.angleDeg());
      }
    }

    // BRAKE in progress (a board with a brake): the drive is released only
    // once the brake is at its setpoint, and RELEASED is dated from here.
    if (brake_pending_ && brake_torque_ != nullptr &&
        brake_torque_->brakeSettled()) {
      completeRelease();
    }

    // RELEASED: the classifier and the capture see every sample, in order,
    // and the TRIAL summary follows an outcome by one second if the host has
    // not left RELEASED by then (protocol v1).
    if (watch_.released()) {
      const double angle = motion_.angleDeg();
      const double torque = currentTorque();
      watch_.sample(now_us_, angle, torque);
      const TrialWatch::Outcome outcome =
          watch_.evaluate(now_us_, angle, torque);
      // TOUCH is informational and may precede the outcome; when both fall
      // on one sample it goes out first.
      if (watch_.consumeTouch()) {
        emitAngleEvent("TOUCH", angle, /*with_brake=*/false);
      }
      if (outcome != TrialWatch::Outcome::kNone) {
        emitAngleEvent(TrialWatch::name(outcome), angle, /*with_brake=*/false);
      }
      if (watch_.trialDue(now_us_)) {
        emitTrial();
      }
    }

    // STREAM: decimated telemetry, display only.
    if (stream_on_ && now_us_ >= stream_next_us_) {
      stream_next_us_ += stream_period_us_;
      emitAngleEvent("S", motion_.angleDeg(), /*with_brake=*/false);
    }

    // The host's homing bound (`home;<s>;<timeout_ms>`), checked after the
    // completion above so a home that finishes on the deadline counts as done.
    if (home_deadline_armed_ && now_us_ >= home_deadline_us_) {
      home_deadline_armed_ = false;
      raiseFault(fault::kHomeTimeout, "home_timeout", false);
      return;
    }

    if (watchdog_armed_ && now_us_ >= watchdog_deadline_us_) {
      watchdog_armed_ = false;
      raiseFault(fault::kHostTimeout, "host_timeout", false);
    }
  }

  // See setWallClock(). Only ever moves the clock forward.
  void syncClock() {
    if (wall_clock_ == nullptr) {
      return;
    }
    const uint64_t wall = wall_clock_();
    if (wall <= now_us_) {
      return;
    }
    const uint64_t blocked = wall - now_us_;
    if (watchdog_armed_) {
      watchdog_deadline_us_ += blocked;
    }
    if (home_deadline_armed_) {
      home_deadline_us_ += blocked;
    }
    now_us_ = wall;
  }

  void refreshWatchdog() {
    // Only meaningful once armed. Before that there is no deadline to push out.
    if (watchdog_armed_) {
      watchdog_deadline_us_ = now_us_ + cfg_.host_timeout_us;
    }
  }

  // On `cause_active`, and why every fault this board raises passes `false`.
  //
  // The draft says CLRFAULT "clears only clearable faults whose cause has gone".
  // That test is not implementable for any cause this board cannot probe without
  // retrying: whether the mechanism is still against a stop, whether the rail
  // will come up, whether CHOPCONF reads back 1 -- each is only answerable by
  // attempting the thing again.
  //
  // Marking them active produced a DEADLOCK, found on hardware 2026-09-03: a
  // `not_following` fault at the mechanical stop refused every CLRFAULT with
  // `latched`, and the remedy for being against a stop is to MOVE, which is
  // refused while faulted. The board could only be recovered by a reset -- and
  // it silently invalidated two measurements that appeared to run and did not.
  //
  // So the board clears and re-faults instead: CLRFAULT succeeds, and if the
  // cause is still there the next command that touches it faults again
  // immediately. That is recoverable, and it reports the truth just as loudly.
  // Raised as an open question in `mouse-joystick-pcb/docs/uart-protocol.md`.
  void raiseFault(uint8_t bit, const char *detail, bool cause_active) {
    motion_.abort();
    watchdog_armed_ = false;
    home_deadline_armed_ = false;
    stream_on_ = false;
    if (inState(state_name::kReleased)) {
      leaveReleased(); // the TRIAL summary precedes the FAULT event
    }
    brakeOff(); // a BRAKE still settling, too (V1.8: today's default is off)
    fault_mask_ |= (1u << bit);
    fault_bit_ = bit;
    fault_detail_ = detail;
    fault_cause_active_ = cause_active;
    state_ = state_name::kFaulted;

    // Faults drop HV_EN, per the board's safety rules -- and that has to mean
    // the RAIL and not just this flag. Found on hardware 2026-09-03: after the
    // first fault, `status` reported `hv=0` while the 36 V rail was still
    // physically up and the driver still enabled, because only the flag was
    // cleared. A board that reports a safe state it has not reached is worse
    // than one that reports the unsafe state honestly.
    motion_.setHvEnabled(false);
    hv_on_ = false;

    protocol::Line line;
    line.beginEvent("FAULT", now_us_);
    line.kvInt("code", bit);
    line.kvStr("detail", detail);
    push(line);
  }

  // Leaving a compliant state, as ONE operation: reconcile the ramp generator
  // with the encoder, re-energise the driver, and become ARMED again.
  //
  // Found on hardware 2026-09-03: `freewheel` releases the driver, and every
  // path out of FREEWHEEL resynced and set ARMED without ever re-enabling it.
  // The ramp generator then advanced with no torque, the encoder did not
  // follow, and the follow check faulted `not_following` after about 1.4 deg --
  // correct for what it saw, and the third time that check has caught a state
  // transition that forgot to energise what it was about to use.
  //
  // It is one function because the host must not be able to be interrupted
  // between the parts. Returns false if the driver refused, which the caller
  // turns into a fault: a compliant joystick that will not re-engage is not a
  // state to keep answering commands from.
  bool leaveCompliance() {
    closeTrial();
    motion_.resyncFromEncoder();
    const bool ok = motion_.enableDriver();
    // The brake comes off AFTER the drive is back, or the joystick drops in
    // between (uart-protocol.md 3a). Until 2026-10-06 this ran first, which
    // no board could show: none had a brake.
    brakeOff();
    return ok;
  }

  // Close the RELEASED period's bookkeeping: emit the TRIAL summary if a WATCH
  // happened in it and has not been reported, and stop the classifier.
  void closeTrial() {
    if (watch_.trialPending()) {
      emitTrial();
    }
    watch_.leave();
  }

  // The brake off: the level the fields report, a BRAKE still settling, and
  // the hardware where there is one.
  void brakeOff() {
    brake_level_ = 0.0;
    brake_pending_ = false;
    if (brake_torque_ != nullptr) {
      brake_torque_->brakeRelease();
    }
  }

  // Close RELEASED where the caller does not re-engage the drive (a fault):
  // the bookkeeping, then the brake off.
  void leaveReleased() {
    closeTrial();
    brakeOff();
  }

  // The brake is at its setpoint, or there is none: release the drive, become
  // RELEASED, start the classifier's ring, announce it.
  void completeRelease() {
    brake_pending_ = false;
    motion_.releaseDriver();
    state_ = state_name::kReleased;
    watch_.release(now_us_, motion_.angleDeg(), cfg_.pull_sign);
    emitAngleEvent("RELEASED", motion_.angleDeg(), /*with_brake=*/true);
  }

  bool faulted() const { return strcmp(state_, state_name::kFaulted) == 0; }
  bool inState(const char *s) const { return strcmp(state_, s) == 0; }

  // ---------------------------------------------------------------- dispatch

  bool dispatch(const protocol::Command &cmd, protocol::Line *out) {
    const char *v = cmd.verb;
    if (strcmp(v, "id") == 0) return cmdId(cmd, out);
    if (strcmp(v, "ping") == 0) return cmdPing(cmd, out);
    if (strcmp(v, "status") == 0) return cmdStatus(cmd, out);
    if (strcmp(v, "limits") == 0) return cmdLimits(cmd, out);
    if (strcmp(v, "faults") == 0) return cmdFaults(cmd, out);
    if (strcmp(v, "clrfault") == 0) return cmdClrFault(cmd, out);
    if (strcmp(v, "hv") == 0) return cmdHv(cmd, out);
    if (strcmp(v, "instr") == 0) return cmdInstr(cmd, out);
    if (strcmp(v, "home") == 0) return cmdHome(cmd, out);
    if (strcmp(v, "arm") == 0) return cmdArm(cmd, out);
    if (strcmp(v, "disarm") == 0) return cmdDisarm(cmd, out);
    if (strcmp(v, "stop") == 0) return cmdStop(cmd, out);
    if (strcmp(v, "moveto") == 0) return cmdMove(cmd, out, false);
    if (strcmp(v, "move") == 0)
      return cmdMove(cmd, out, true);
    if (strcmp(v, "cal") == 0) return cmdCal(cmd, out);
    // Protocol v1 (2026-10-01). FREEWHEEL and ARMPULL are gone, not aliased.
    if (strcmp(v, "brake") == 0)
      return cmdBrake(cmd, out);
    if (strcmp(v, "watch") == 0)
      return cmdWatch(cmd, out);
    if (strcmp(v, "cfg") == 0)
      return cmdCfg(cmd, out);
    if (strcmp(v, "fetch") == 0)
      return cmdFetch(cmd, out);
    if (strcmp(v, "stream") == 0)
      return cmdStream(cmd, out);
    if (strcmp(v, "tq") == 0)
      return cmdTq(cmd, out);
    if (strcmp(v, "angout") == 0)
      return cmdAngout(cmd, out);
    if (strcmp(v, "angscale") == 0)
      return cmdAngscale(cmd, out);

    // "An unrecognized verb is rejected, never partially executed."
    out->nak(cmd.seq, protocol::kNakBadVerb);
    return true;
  }

  // `ID` must work before ARM and must never fail, so it is answered in every
  // state including FAULTED and takes no argument it could reject.
  bool cmdId(const protocol::Command &cmd, protocol::Line *out) {
    out->beginAck(cmd.seq);
    out->kvStr("hw", cfg_.hw_rev);
    out->kvStr("serial", cfg_.serial);
    out->kvStr("fw", cfg_.fw_version);
    out->kvInt("proto", protocol::kProtocolVersion);
    // The axis count distinguishes this board from a twin-axis board on the
    // same link. Never inferred from a name.
    out->kvInt("axes", protocol::kAxisCount);
    // A `+`-joined token list: avoids the delimiter and stays greppable in a
    // logic-analyzer capture. The honest list for a rev-0 board -- no `angout`
    // because the outputs are unpopulated, no `brake` because that hardware
    // does not exist. A host should gate on this, not on the firmware version.
    // Fixed order, absent tokens omitted (protocol v1): the firmware facts
    // first, then the hardware facts of this revision.
    char caps[80] = "motion+home+instr+hv+watch+fetch+stream+cfg";
    if (cfg_.has_angout)
      strcat(caps, "+angout");
    if (cfg_.has_brake)
      strcat(caps, "+brake");
    if (cfg_.has_torque)
      strcat(caps, "+torque");
    out->kvStr("caps", caps);
    out->kvInt("rxbuf", protocol::kMaxLine);
    out->kvStr("boot", cfg_.boot_reason);
    out->kvInt("crumb", static_cast<long>(cfg_.boot_crumb));
    return true;
  }

  bool cmdPing(const protocol::Command &cmd, protocol::Line *out) {
    // Carries no data and has no side effect of its own: the watchdog was
    // already refreshed by the frame arriving. Always answered, including
    // before ARM -- a host has to be able to probe a link it is not armed on.
    out->beginAck(cmd.seq);
    return true;
  }

  bool cmdStatus(const protocol::Command &cmd, protocol::Line *out) {
    out->beginAck(cmd.seq);
    out->kvStr("state", state_);
    // TWO positions, and the distinction is load-bearing. `angle` is what the
    // encoder says -- the only one that is ever true. `xactual` is what the
    // ramp generator thinks, true only while the board is driving and stale the
    // moment the animal moves the joystick in FREEWHEEL.
    out->kvNumber("angle", motion_.angleDeg());
    out->kvNumber("xactual", motion_.xactualDeg());
    // Always present, empty without the hardware (protocol v1), so field
    // positions never move between revisions.
    appendTorque(out);
    appendBrake(out);
    out->kvBool("homed", homed_);
    out->kvBool("limit", motion_.limitAsserted());
    out->kvInt("faults", static_cast<long>(fault_mask_));
    out->kvBool("hv", hv_on_);
    out->kvU64("t", now_us_); // the board clock, for the host's alignment
    return true;
  }

  bool cmdLimits(const protocol::Command &cmd, protocol::Line *out) {
    out->beginAck(cmd.seq);
    out->kvNumber("travel_lo", cfg_.travel_lo);
    out->kvNumber("travel_hi", cfg_.travel_hi);
    out->kvNumber("v_max", cfg_.v_max);
    out->kvNumber("a_max", cfg_.a_max);
    out->kvNumber("i_max", cfg_.i_max);
    return true;
  }

  bool cmdFaults(const protocol::Command &cmd, protocol::Line *out) {
    out->beginAck(cmd.seq);
    out->kvInt("mask", static_cast<long>(fault_mask_));
    if (fault_mask_ != 0 && fault_detail_ != nullptr) {
      out->kvStr(faultName(fault_bit_), fault_detail_);
    }
    return true;
  }

  bool cmdClrFault(const protocol::Command &cmd, protocol::Line *out) {
    if (fault_mask_ == 0) {
      out->beginAck(cmd.seq);
      return true;
    }
    if (!fault_cause_active_) {
      fault_mask_ = 0;
      fault_detail_ = nullptr;
    }
    if (fault_mask_ != 0) {
      // FAULTED is exited only through CLRFAULT, and only for faults whose
      // cause has actually gone.
      out->nak(cmd.seq, protocol::kNakLatched);
      return true;
    }
    state_ = state_name::kIdle;
    homed_ = false;
    out->beginAck(cmd.seq);
    return true;
  }

  bool cmdHv(const protocol::Command &cmd, protocol::Line *out) {
    if (cmd.arg_count != 1) {
      out->nak(cmd.seq, protocol::kNakBadArgs);
      return true;
    }
    if (argEquals(cmd.arg(0), "state")) {
      out->beginAck(cmd.seq);
      out->kvBool("hv", hv_on_);
      return true;
    }
    if (argEquals(cmd.arg(0), "on")) {
      if (faulted()) {
        out->nak(cmd.seq, protocol::kNakFaulted);
        return true;
      }
      if (!motion_.setHvEnabled(true)) {
        // The rail was asked for and did not come up. That is a fault and not
        // a nak: the attempt was made and the hardware answered.
        out->nak(cmd.seq, protocol::kNakFaulted);
        push(*out);
        out->clear();
        raiseFault(fault::kSupplyOutOfRange, "hv_did_not_come_up", false);
        return false;
      }
      hv_on_ = true;
      // The heartbeat requirement begins with the rail (2026-10-06): a host
      // that dies between `hv on` and `arm` must not leave the rail up, which
      // is what happened when a task crashed one step after this ack. Armed
      // here, refreshed by every well-formed line, disarmed by `hv off` or a
      // fault; `disarm` leaves it running while the rail is on.
      watchdog_armed_ = true;
      watchdog_deadline_us_ = now_us_ + cfg_.host_timeout_us;
      out->beginAck(cmd.seq);
      return true;
    }
    if (argEquals(cmd.arg(0), "off")) {
      // Always permitted: the board never requires a message in order to become
      // safe, and this is the message that makes it safer. The return value is
      // ignored on purpose -- there is no useful answer to "turning it off
      // failed", and refusing to report success would leave a host unable to
      // confirm the safe state it just asked for.
      home_deadline_armed_ = false;
      motion_.setHvEnabled(false);
      hv_on_ = false;
      watchdog_armed_ = false; // the rail is off: nothing left to protect
      if (inState(state_name::kArmed) || inState(state_name::kActive)) {
        motion_.abort();
        state_ = state_name::kHomed;
      }
      out->beginAck(cmd.seq);
      return true;
    }
    out->nak(cmd.seq, protocol::kNakBadArgs);
    return true;
  }

  bool cmdInstr(const protocol::Command &cmd, protocol::Line *out) {
    if (cmd.arg_count != 1) {
      out->nak(cmd.seq, protocol::kNakBadArgs);
      return true;
    }
    if (argEquals(cmd.arg(0), "state")) {
      out->beginAck(cmd.seq);
      out->kvBool("instr", instr_on_);
      return true;
    }
    if (argEquals(cmd.arg(0), "on") || argEquals(cmd.arg(0), "off")) {
      // A board whose port cannot be disabled accepts the verb and stays on.
      instr_on_ = argEquals(cmd.arg(0), "on") || !cfg_.instr_switchable;
      out->beginAck(cmd.seq);
      return true;
    }
    out->nak(cmd.seq, protocol::kNakBadArgs);
    return true;
  }

  // With an encoder, homing is no longer how position is RECOVERED -- the
  // encoder keeps it through a pull. It is how the joystick is PARKED out of
  // the animal's way between trials.
  bool cmdHome(const protocol::Command &cmd, protocol::Line *out) {
    if (faulted()) {
      out->nak(cmd.seq, protocol::kNakFaulted);
      return true;
    }
    if (!hv_on_) {
      out->nak(cmd.seq, protocol::kNakNoHv);
      return true;
    }
    if (motion_.isMoving() || brake_pending_) {
      out->nak(cmd.seq, protocol::kNakBusy);
      return true;
    }
    const bool from_released = inState(state_name::kReleased);
    if (from_released) {
      // No re-engage here: `enableDriver()` is called below for every home, so
      // adding one would be a redundant SPI round trip. The brake comes off
      // after it, below.
      closeTrial();
      motion_.resyncFromEncoder();
      state_ = state_name::kArmed;
    }

    bool bounded = false;
    uint32_t timeout_us = 0;
    if (cmd.arg_count > 0) {
      double ms = 0.0;
      if (!protocol::parseNumber(cmd.arg(0), &ms)) {
        out->nak(cmd.seq, protocol::kNakBadArgs);
        return true;
      }
      long us = static_cast<long>(ms * 1000.0);
      if (us <= 0) {
        out->nak(cmd.seq, protocol::kNakOutOfRange);
        return true;
      }
      bounded = true;
      timeout_us = static_cast<uint32_t>(us);
    }

    // The host's timeout, clipped to the firmware maximum, becomes a deadline
    // that `service()` enforces while the home runs: exceeding it aborts the
    // motion and faults `home_timeout`, because by then the motion did start.
    // It used to be compared against the backend's bound up front, which with
    // the real backend's 8 s bound faulted every shorter request immediately
    // (2026-10-01 review, bug 2).
    if (bounded && timeout_us > motion_.homeTimeoutMaxUs()) {
      timeout_us = motion_.homeTimeoutMaxUs();
    }

    // HOME has to energise the driver itself, and the draft does not say so.
    //
    // Found on hardware 2026-09-03: `ARM` is "refused if not homed", and `ARM`
    // is what enables the driver -- so homing ran with `~{TMC_DRV_EN}` high, the
    // ramp generator advanced, and the shaft never moved. The follow check
    // caught it as `mechanical_stop_first` after about 400 microsteps, which is
    // the correct report for what it saw and a confusing one for what was wrong.
    //
    // The resolution is a distinction the draft's state machine does not draw:
    // HOST-COMMANDED motion needs `ARM` and therefore needs `homed`, because an
    // unhomed board does not know where its limits are. BOARD-CONTROLLED homing
    // is the one motion that is safe while unhomed -- it is bounded by the
    // switch stop, the follow check and a timeout, and no host argument reaches
    // it. So HOME manages the driver for itself and the `ARM`-requires-`homed`
    // rule stands. Raised in `mouse-joystick-pcb/docs/uart-protocol.md`.
    //
    // The driver is deliberately LEFT enabled afterwards, so the joystick holds
    // the position homing just parked it at instead of going limp.
    if (!motion_.enableDriver()) {
      out->nak(cmd.seq, protocol::kNakFaulted);
      push(*out);
      out->clear();
      raiseFault(fault::kInternal, "vsense_not_1", false);
      return false;
    }
    if (from_released) {
      brakeOff(); // the drive is back
    }
    if (!motion_.startHome()) {
      out->nak(cmd.seq, protocol::kNakBusy);
      return true;
    }
    if (bounded) {
      home_deadline_armed_ = true;
      home_deadline_us_ = now_us_ + timeout_us;
    }
    out->beginAck(cmd.seq);
    return true;
  }

  bool cmdArm(const protocol::Command &cmd, protocol::Line *out) {
    if (faulted()) {
      out->nak(cmd.seq, protocol::kNakFaulted);
      return true;
    }
    if (!homed_) {
      out->nak(cmd.seq, protocol::kNakNotHomed);
      return true;
    }
    if (!hv_on_) {
      out->nak(cmd.seq, protocol::kNakNoHv);
      return true;
    }
    if (!motion_.enableDriver()) {
      // The current decision and the shunt value are one decision. On real
      // hardware this is the CHOPCONF.vsense read-back: it resets to 0, which
      // against this board's 0.15 ohm shunts drives roughly 1.8x the motor's
      // rating. Refuse rather than drive it.
      out->nak(cmd.seq, protocol::kNakFaulted);
      push(*out);
      out->clear();
      raiseFault(fault::kInternal, "vsense_not_1", false);
      return false;
    }
    state_ = state_name::kArmed;
    watchdog_armed_ = true;
    watchdog_deadline_us_ = now_us_ + cfg_.host_timeout_us;
    out->beginAck(cmd.seq);
    return true;
  }

  bool cmdDisarm(const protocol::Command &cmd, protocol::Line *out) {
    motion_.abort();
    watchdog_armed_ = hv_on_; // runs for as long as the rail is on
    home_deadline_armed_ = false;
    stream_on_ = false;
    if (brake_pending_) {
      brakeOff(); // a BRAKE still settling is cancelled; the drive never left
    }
    if (inState(state_name::kReleased)) {
      // Re-engage on the way out even though this ends in HOMED rather than
      // ARMED. "Disarmed" has to mean "not accepting motion commands", not
      // "limp": a joystick left compliant within an animal's reach is a worse
      // resting state than one held where it was parked. The return value is
      // ignored because DISARM is always permitted and must not be refusable --
      // a driver that will not re-engage shows up on the next command.
      (void)leaveCompliance();
      state_ = state_name::kHomed;
    } else if (inState(state_name::kArmed) || inState(state_name::kActive)) {
      motion_.resyncFromEncoder();
      state_ = state_name::kHomed;
    }
    out->beginAck(cmd.seq);
    return true;
  }

  bool cmdStop(const protocol::Command &cmd, protocol::Line *out) {
    // From FREEWHEEL this is how a host takes the joystick back: re-engage the
    // driver holding wherever the animal left it.
    // `abort()` holds position wherever the joystick is, which is what makes
    // this safe to send at any time -- including mid-move.
    motion_.abort();
    home_deadline_armed_ = false;
    if (brake_pending_) {
      brakeOff(); // a BRAKE still settling is cancelled; the drive never left
    }
    if (inState(state_name::kReleased)) {
      // "From FREEWHEEL this is how a host takes the joystick back: re-engage
      // the driver holding wherever the animal left it." Re-engaging is the
      // whole point of the command in this state.
      if (!leaveCompliance()) {
        out->nak(cmd.seq, protocol::kNakFaulted);
        push(*out);
        out->clear();
        raiseFault(fault::kInternal, "vsense_not_1", false);
        return false;
      }
      state_ = state_name::kArmed;
    } else if (inState(state_name::kActive)) {
      motion_.resyncFromEncoder();
      state_ = state_name::kArmed;
    }
    out->beginAck(cmd.seq);
    return true;
  }

  bool cmdMove(const protocol::Command &cmd, protocol::Line *out,
               bool relative) {
    if (faulted()) {
      out->nak(cmd.seq, protocol::kNakFaulted);
      return true;
    }
    if (brake_pending_) {
      out->nak(cmd.seq, protocol::kNakBusy); // the brake is still engaging
      return true;
    }
    // FREEWHEEL re-engages implicitly. One command that resyncs, re-engages and
    // moves is better than three the host could be interrupted between.
    if (inState(state_name::kReleased)) {
      if (!leaveCompliance()) {
        out->nak(cmd.seq, protocol::kNakFaulted);
        push(*out);
        out->clear();
        raiseFault(fault::kInternal, "vsense_not_1", false);
        return false;
      }
      state_ = state_name::kArmed;
    }
    // Checked BEFORE the argument, deliberately: refusing motion outside ARMED
    // is the safety-relevant rule and must not be reachable past a parse error.
    if (!inState(state_name::kArmed)) {
      out->nak(cmd.seq, protocol::kNakNotArmed);
      return true;
    }
    if (cmd.arg_count == 0) {
      out->nak(cmd.seq, protocol::kNakBadArgs);
      return true;
    }
    double value = 0.0;
    if (!protocol::parseNumber(cmd.arg(0), &value)) {
      out->nak(cmd.seq, protocol::kNakBadArgs);
      return true;
    }
    // Named profiles (protocol v1): accepted here; the velocity each one
    // selects is the application's constant (AppConfig.hpp), the same for all
    // of them in the simulated backend so corpus timings do not depend on it.
    if (cmd.arg_count > 1 && !isProfileName(cmd.arg(1))) {
      out->nak(cmd.seq, protocol::kNakBadArgs);
      return true;
    }

    // Relative to where the joystick IS, which after a pull is not where the
    // ramp generator thinks it is. Using xactual would silently add the
    // animal's pull to every subsequent relative move.
    double target = relative ? motion_.angleDeg() + value : value;
    if (target < cfg_.travel_lo || target > cfg_.travel_hi) {
      // Rejected before anything moves, so a nak and not an OVER_TRAVEL fault.
      // The fault is for a position that got there.
      out->nak(cmd.seq, protocol::kNakOutOfRange);
      return true;
    }

    if (!motion_.startMoveTo(target)) {
      out->nak(cmd.seq, protocol::kNakBusy);
      return true;
    }
    state_ = state_name::kActive;
    out->beginAck(cmd.seq);
    return true;
  }

  // ------------------------------------------------------ protocol v1 verbs

  static bool isProfileName(const char *arg) {
    return argEquals(arg, "present") || argEquals(arg, "retract") ||
           argEquals(arg, "slow");
  }

  // `key=value` argument: splits at the first '=' into `key` and `value`.
  static bool splitKv(const char *arg, char *key, size_t key_n,
                      const char **value) {
    const char *eq = strchr(arg, '=');
    if (eq == nullptr || eq == arg) {
      return false;
    }
    size_t klen = static_cast<size_t>(eq - arg);
    if (klen >= key_n) {
      return false;
    }
    for (size_t i = 0; i < klen; ++i) {
      key[i] = protocol::lowerAscii(arg[i]);
    }
    key[klen] = '\0';
    *value = eq + 1;
    return true;
  }

  // The latest torque sample, tared, in N.cm as the sensor reads it; 0.0 on a
  // board whose config claims the bridge without a backend.
  double currentTorque() const {
    return (cfg_.has_torque && brake_torque_ != nullptr)
               ? brake_torque_->torqueNcm()
               : 0.0;
  }
  // Always present: a number on a board with the hardware, empty without, so
  // field positions never move between revisions (protocol v1).
  void appendTorque(protocol::Line *out) const {
    if (cfg_.has_torque) {
      out->kvNumber("torque", currentTorque());
    } else {
      out->kvStr("torque", "");
    }
  }
  void appendBrake(protocol::Line *out) const {
    if (cfg_.has_brake) {
      out->kvNumber("brake", brake_level_); // commanded: engaging or held, else 0
    } else {
      out->kvStr("brake", "");
    }
  }

  // `ev;NAME;t;angle=…;torque=…[;brake=…]`
  void emitAngleEvent(const char *name, double angle, bool with_brake) {
    protocol::Line line;
    line.beginEvent(name, now_us_);
    line.kvNumber("angle", angle);
    appendTorque(&line);
    if (with_brake) {
      appendBrake(&line);
    }
    push(line);
  }

  void emitTrial() {
    const TrialWatch::Summary s =
        watch_.takeSummary(motion_.angleDeg(), currentTorque());
    protocol::Line line;
    line.beginEvent("TRIAL", now_us_);
    line.kvStr("outcome", TrialWatch::name(s.outcome));
    line.kvU64("t_rel", s.t_rel);
    if (cfg_.has_torque && s.has_touch) {
      line.kvU64("t_touch", s.t_touch);
    } else {
      line.kvStr("t_touch", "");
    }
    if (s.has_move) {
      line.kvU64("t_move", s.t_move);
    } else {
      line.kvStr("t_move", "");
    }
    if (s.has_out) {
      line.kvU64("t_out", s.t_out);
    } else {
      line.kvStr("t_out", "");
    }
    line.kvNumber("a0", s.a0);
    line.kvNumber("a_peak", s.a_peak);
    line.kvNumber("a_end", s.a_end);
    if (cfg_.has_torque) {
      line.kvNumber("tq_peak", s.tq_peak);
      line.kvNumber("tq_out", s.tq_end);
    } else {
      line.kvStr("tq_peak", "");
      line.kvStr("tq_out", "");
    }
    line.kvInt("dur_thr", static_cast<long>(s.dur_thr_ms));
    push(line);
  }

  // BRAKE <torque_Ncm>: engage the brake, then release the drive, as one
  // operation. On a board without a brake the torque is ignored and only the
  // release happens -- today's FREEWHEEL, under the verb that will carry the
  // brake, so the trial structure can be piloted now. The ack comes first and
  // RELEASED when the release has happened.
  bool cmdBrake(const protocol::Command &cmd, protocol::Line *out) {
    if (faulted()) {
      out->nak(cmd.seq, protocol::kNakFaulted);
      return true;
    }
    if (inState(state_name::kActive) || brake_pending_) {
      out->nak(cmd.seq, protocol::kNakBusy);
      return true;
    }
    if (!inState(state_name::kArmed)) {
      // A joystick held by an energised motor cannot go compliant by accident;
      // and a second BRAKE while RELEASED is refused too (the brief's rule).
      out->nak(cmd.seq, protocol::kNakNotArmed);
      return true;
    }
    if (cmd.arg_count != 1) {
      out->nak(cmd.seq, protocol::kNakBadArgs);
      return true;
    }
    double torque = 0.0;
    if (!protocol::parseNumber(cmd.arg(0), &torque)) {
      out->nak(cmd.seq, protocol::kNakBadArgs);
      return true;
    }
    if (torque < 0.0) {
      out->nak(cmd.seq, protocol::kNakOutOfRange);
      return true;
    }
    brake_level_ = cfg_.has_brake ? torque : 0.0;
    if (cfg_.has_brake) {
      // Phases 1 and 2: the brake to its setpoint first. The backend owns the
      // torque-to-current law and the ramp (a full-scale step overshoots by
      // 27 % on pcb v1.0, measured 2026-10-06); service() releases the drive
      // and emits RELEASED once it reports settled. Motion verbs are busy
      // meanwhile; STOP and DISARM cancel it.
      if (brake_torque_ == nullptr || !brake_torque_->brakeEngage(torque)) {
        brake_level_ = 0.0;
        out->nak(cmd.seq, protocol::kNakFaulted);
        push(*out);
        out->clear();
        raiseFault(fault::kBrakeFault, "brake_unavailable", false);
        return false;
      }
      brake_pending_ = true;
      out->beginAck(cmd.seq);
      return true;
    }
    // Without a brake there is nothing to wait for: release now.
    out->beginAck(cmd.seq);
    push(*out);
    out->clear();
    completeRelease();
    return false; // already pushed: ack, then RELEASED
  }

  // WATCH pull=deg;push=deg;timeout=ms[;hold=ms][;touch=Ncm][;maxdur=ms]
  bool cmdWatch(const protocol::Command &cmd, protocol::Line *out) {
    if (!inState(state_name::kReleased)) {
      out->nak(cmd.seq, protocol::kNakNotReleased);
      return true;
    }
    TrialWatch::Params p;
    bool has_pull = false, has_push = false, has_timeout = false;
    for (uint8_t i = 0; i < cmd.arg_count; ++i) {
      char key[16];
      const char *value = nullptr;
      double v = 0.0;
      if (!splitKv(cmd.arg(i), key, sizeof(key), &value) ||
          !protocol::parseNumber(value, &v)) {
        out->nak(cmd.seq, protocol::kNakBadArgs);
        return true;
      }
      if (v < 0.0) {
        out->nak(cmd.seq, protocol::kNakOutOfRange);
        return true;
      }
      if (strcmp(key, "pull") == 0) {
        p.pull = v;
        has_pull = true;
      } else if (strcmp(key, "push") == 0) {
        p.push = v;
        has_push = true;
      } else if (strcmp(key, "timeout") == 0) {
        p.timeout_us = static_cast<uint32_t>(v * 1000.0);
        has_timeout = true;
      } else if (strcmp(key, "hold") == 0) {
        p.hold_us = static_cast<uint32_t>(v * 1000.0);
      } else if (strcmp(key, "maxdur") == 0) {
        p.has_maxdur = true;
        p.maxdur_us = static_cast<uint32_t>(v * 1000.0);
      } else if (strcmp(key, "touch") == 0) {
        // Meaningful with the v1.0 bridge; accepted and inert without it.
        p.has_touch = true;
        p.touch = v;
      } else {
        out->nak(cmd.seq, protocol::kNakBadArgs);
        return true;
      }
    }
    if (!has_pull || !has_push || !has_timeout) {
      out->nak(cmd.seq, protocol::kNakBadArgs);
      return true;
    }
    watch_.arm(now_us_, p, deadband_deg_);
    out->beginAck(cmd.seq);
    return true;
  }

  // CFG get | CFG key=value[;key=value…]: all pairs are checked, then all
  // applied, or none.
  bool cmdCfg(const protocol::Command &cmd, protocol::Line *out) {
    if (cmd.arg_count == 0) {
      out->nak(cmd.seq, protocol::kNakBadArgs);
      return true;
    }
    if (cmd.arg_count == 1 && argEquals(cmd.arg(0), "get")) {
      out->beginAck(cmd.seq);
      out->kvInt("wdt_ms", static_cast<long>(cfg_.host_timeout_us / 1000u));
      out->kvNumber("deadband_deg", deadband_deg_);
      out->kvInt("stream_hz", stream_hz_);
      out->kvInt("pre_ms", watch_.preMs());
      out->kvInt("post_ms", watch_.postMs());
      return true;
    }
    uint32_t wdt_us = cfg_.host_timeout_us;
    double deadband = deadband_deg_;
    uint16_t stream_hz = stream_hz_;
    long pre_ms = watch_.preMs();
    long post_ms = watch_.postMs();
    for (uint8_t i = 0; i < cmd.arg_count; ++i) {
      char key[16];
      const char *value = nullptr;
      double v = 0.0;
      if (!splitKv(cmd.arg(i), key, sizeof(key), &value) ||
          !protocol::parseNumber(value, &v)) {
        out->nak(cmd.seq, protocol::kNakBadArgs);
        return true;
      }
      if (strcmp(key, "wdt_ms") == 0) {
        const double us = v * 1000.0;
        if (us < kWdtMinUs || us > kWdtMaxUs) {
          out->nak(cmd.seq, protocol::kNakOutOfRange);
          return true;
        }
        wdt_us = static_cast<uint32_t>(us);
      } else if (strcmp(key, "deadband_deg") == 0) {
        if (v < kDeadbandMinDeg || v > kDeadbandMaxDeg) {
          out->nak(cmd.seq, protocol::kNakOutOfRange);
          return true;
        }
        deadband = v;
      } else if (strcmp(key, "stream_hz") == 0) {
        if (v < kStreamHzMin || v > kStreamHzMax) {
          out->nak(cmd.seq, protocol::kNakOutOfRange);
          return true;
        }
        stream_hz = static_cast<uint16_t>(v);
      } else if (strcmp(key, "pre_ms") == 0 || strcmp(key, "post_ms") == 0) {
        // The capture window around the outcome: each bounded by the ring,
        // and together (checked below) by the ring less the outcome sample.
        if (v < 0.0 || v > TrialWatch::kWindowMaxMs) {
          out->nak(cmd.seq, protocol::kNakOutOfRange);
          return true;
        }
        (key[1] == 'r' ? pre_ms : post_ms) = static_cast<long>(v);
      } else {
        out->nak(cmd.seq, protocol::kNakBadArgs);
        return true;
      }
    }
    if (pre_ms + post_ms > TrialWatch::kWindowMaxMs) {
      out->nak(cmd.seq, protocol::kNakOutOfRange);
      return true;
    }
    cfg_.host_timeout_us = wdt_us; // applies from the next watchdog refresh
    deadband_deg_ = deadband;
    stream_hz_ = stream_hz;
    watch_.setWindow(static_cast<uint16_t>(pre_ms),
                     static_cast<uint16_t>(post_ms));
    out->beginAck(cmd.seq);
    return true;
  }

  // FETCH [k0;n]: the frozen window of the last WATCH, in chunks of at most
  // kFetchChunkMax lines. Sample `pre` is the outcome sample.
  bool cmdFetch(const protocol::Command &cmd, protocol::Line *out) {
    if (watch_.captureBusy()) {
      out->nak(cmd.seq, protocol::kNakBusy);
      return true;
    }
    long k0 = 0;
    long n = kFetchChunkMax;
    if (cmd.arg_count >= 1 && !protocol::parseInt(cmd.arg(0), &k0)) {
      out->nak(cmd.seq, protocol::kNakBadArgs);
      return true;
    }
    if (cmd.arg_count >= 2 && !protocol::parseInt(cmd.arg(1), &n)) {
      out->nak(cmd.seq, protocol::kNakBadArgs);
      return true;
    }
    if (k0 < 0 || n < 0) {
      out->nak(cmd.seq, protocol::kNakOutOfRange);
      return true;
    }
    if (n > kFetchChunkMax) {
      n = kFetchChunkMax;
    }
    const long count = static_cast<long>(watch_.windowCount());
    const long available = (k0 < count) ? (count - k0) : 0;
    if (n > available) {
      n = available;
    }
    out->beginAck(cmd.seq);
    out->kvInt("n", n);
    out->kvInt("k0", k0);
    out->kvU64("t0", watch_.hasWindow()
                         ? watch_.windowSampleTimeUs(static_cast<uint32_t>(k0))
                         : 0u);
    out->kvInt("dt_us", static_cast<long>(TrialWatch::kSampleUs));
    out->kvInt("pre", static_cast<long>(watch_.windowPre()));
    out->kvInt("lost", static_cast<long>(watch_.windowLost()));
    push(*out);
    out->clear();
    protocol::Line line;
    for (long k = k0; k < k0 + n; ++k) {
      line.clear();
      line.append("cap");
      line.appendChar(protocol::kDelimiter);
      line.appendInt(k);
      line.appendChar(protocol::kDelimiter);
      line.appendNumber(watch_.windowSample(static_cast<uint32_t>(k)));
      line.appendChar(protocol::kDelimiter);
      if (cfg_.has_torque) { // empty without the bridge; same field count
        line.appendNumber(watch_.windowTorque(static_cast<uint32_t>(k)));
      }
      push(line);
    }
    line.clear();
    line.append("cap");
    line.appendChar(protocol::kDelimiter);
    line.append("end");
    push(line);
    return false; // already pushed
  }

  // STREAM on[;rate_hz] | STREAM off
  bool cmdStream(const protocol::Command &cmd, protocol::Line *out) {
    if (cmd.arg_count == 0) {
      out->nak(cmd.seq, protocol::kNakBadArgs);
      return true;
    }
    if (argEquals(cmd.arg(0), "off")) {
      stream_on_ = false;
      out->beginAck(cmd.seq);
      return true;
    }
    if (!argEquals(cmd.arg(0), "on")) {
      out->nak(cmd.seq, protocol::kNakBadArgs);
      return true;
    }
    uint16_t hz = stream_hz_;
    if (cmd.arg_count >= 2) {
      double v = 0.0;
      if (!protocol::parseNumber(cmd.arg(1), &v)) {
        out->nak(cmd.seq, protocol::kNakBadArgs);
        return true;
      }
      if (v < kStreamHzMin || v > kStreamHzMax) {
        out->nak(cmd.seq, protocol::kNakOutOfRange);
        return true;
      }
      hz = static_cast<uint16_t>(v);
    }
    stream_period_us_ = 1000000u / hz;
    stream_next_us_ = now_us_ + stream_period_us_;
    stream_on_ = true;
    out->beginAck(cmd.seq);
    return true;
  }

  // TQ zero | cal;on|off | get: v1.0 boards only.
  // TQ zero | TQ cal;on|off | TQ get -- pcb v1.0 boards (`caps` torque).
  // `get` reports the tared reading, the raw one (nominal scale about the
  // nominal zero, what a tare removed) and the calibration switch's state,
  // all in N.cm; the switch is a verb and not a calibration on v1.0 as built.
  bool cmdTq(const protocol::Command &cmd, protocol::Line *out) {
    if (!cfg_.has_torque || brake_torque_ == nullptr) {
      out->nak(cmd.seq, protocol::kNakBadVerb);
      return true;
    }
    if (cmd.arg_count == 0) {
      out->nak(cmd.seq, protocol::kNakBadArgs);
      return true;
    }
    if (cmd.arg_count == 1 && argEquals(cmd.arg(0), "zero")) {
      brake_torque_->torqueZero();
      out->beginAck(cmd.seq);
      return true;
    }
    if (cmd.arg_count == 1 && argEquals(cmd.arg(0), "get")) {
      out->beginAck(cmd.seq);
      out->kvNumber("torque", brake_torque_->torqueNcm());
      out->kvNumber("raw", brake_torque_->torqueRawNcm());
      out->kvInt("cal", brake_torque_->torqueCalOn() ? 1 : 0);
      return true;
    }
    if (cmd.arg_count == 2 && argEquals(cmd.arg(0), "cal") &&
        (argEquals(cmd.arg(1), "on") || argEquals(cmd.arg(1), "off"))) {
      brake_torque_->torqueCal(argEquals(cmd.arg(1), "on"));
      out->beginAck(cmd.seq);
      return true;
    }
    out->nak(cmd.seq, protocol::kNakBadArgs);
    return true;
  }

  // ANGOUT on|off: the application drives the DAC from `angoutOn()` and
  // `angleOutputValid()`; a board without the output answers badverb so a
  // host cannot believe it is live and record a trace of nothing.
  bool cmdAngout(const protocol::Command &cmd, protocol::Line *out) {
    if (!cfg_.has_angout) {
      out->nak(cmd.seq, protocol::kNakBadVerb);
      return true;
    }
    if (cmd.arg_count != 1) {
      out->nak(cmd.seq, protocol::kNakBadArgs);
      return true;
    }
    if (argEquals(cmd.arg(0), "on") || argEquals(cmd.arg(0), "off")) {
      angout_on_ = argEquals(cmd.arg(0), "on");
      out->beginAck(cmd.seq);
      return true;
    }
    out->nak(cmd.seq, protocol::kNakBadArgs);
    return true;
  }

  // ANGSCALE deg_lo deg_hi: maps [lo, hi] onto 0.05-3.300 V at the BNC.
  bool cmdAngscale(const protocol::Command &cmd, protocol::Line *out) {
    if (!cfg_.has_angout) {
      out->nak(cmd.seq, protocol::kNakBadVerb);
      return true;
    }
    if (cmd.arg_count != 2) {
      out->nak(cmd.seq, protocol::kNakBadArgs);
      return true;
    }
    double lo = 0.0, hi = 0.0;
    if (!protocol::parseNumber(cmd.arg(0), &lo) ||
        !protocol::parseNumber(cmd.arg(1), &hi)) {
      out->nak(cmd.seq, protocol::kNakBadArgs);
      return true;
    }
    if (!(lo < hi)) {
      out->nak(cmd.seq, protocol::kNakOutOfRange);
      return true;
    }
    angscale_lo_ = lo;
    angscale_hi_ = hi;
    out->beginAck(cmd.seq);
    return true;
  }

  bool cmdCal(const protocol::Command &cmd, protocol::Line *out) {
    if (cmd.arg_count == 0) {
      out->nak(cmd.seq, protocol::kNakBadArgs);
      return true;
    }
    if (argEquals(cmd.arg(0), "get")) {
      out->beginAck(cmd.seq);
      out->kvInt("enc_offset", cal_enc_offset_);
      out->kvNumber("angle_zero", cal_angle_zero_);
      return true;
    }
    if (argEquals(cmd.arg(0), "set")) {
      if (cmd.arg_count != 3) {
        out->nak(cmd.seq, protocol::kNakBadArgs);
        return true;
      }
      const char *key = cmd.arg(1);
      const char *raw = cmd.arg(2);
      if (strcmp(key, "enc_offset") == 0) {
        long value = 0;
        if (!protocol::parseInt(raw, &value)) {
          out->nak(cmd.seq, protocol::kNakBadArgs);
          return true;
        }
        cal_enc_offset_ = value;
        out->beginAck(cmd.seq);
        out->kvInt("enc_offset", cal_enc_offset_);
        return true;
      }
      if (strcmp(key, "angle_zero") == 0) {
        double value = 0.0;
        if (!protocol::parseNumber(raw, &value)) {
          out->nak(cmd.seq, protocol::kNakBadArgs);
          return true;
        }
        cal_angle_zero_ = value;
        out->beginAck(cmd.seq);
        out->kvNumber("angle_zero", cal_angle_zero_);
        return true;
      }
      out->nak(cmd.seq, protocol::kNakBadArgs);
      return true;
    }
    out->nak(cmd.seq, protocol::kNakBadArgs);
    return true;
  }

  static bool argEquals(const char *arg, const char *lower) {
    for (; *arg != '\0' && *lower != '\0'; ++arg, ++lower) {
      if (protocol::lowerAscii(*arg) != *lower) {
        return false;
      }
    }
    return *arg == '\0' && *lower == '\0';
  }

  static const char *faultName(uint8_t bit) {
    switch (bit) {
      case fault::kHostTimeout: return "host_timeout";
      case fault::kDriverFault: return "driver_fault";
      case fault::kOverTravel: return "over_travel";
      case fault::kHomeTimeout: return "home_timeout";
      case fault::kLimitUnexpected: return "limit_unexpected";
      case fault::kSupplyOutOfRange: return "supply_out_of_range";
      case fault::kInternal: return "internal";
      case fault::kExtStop: return "ext_stop";
      case fault::kBrakeFault: return "brake_fault";
      case fault::kBrakeCurrent: return "brake_current";
      case fault::kTorqueAdc: return "torque_adc";
      case fault::kTorqueRange: return "torque_range";
      default: return "reserved";
    }
  }

  struct Entry {
    uint16_t seq = 0;
    char canonical[protocol::kMaxLine + 2] = {0};
    char reply[protocol::kMaxLine + 2] = {0};
  };

  Config cfg_;
  MotionBackend &motion_;
  BrakeTorqueBackend *brake_torque_ = nullptr; // null without the hardware

  uint64_t now_us_ = 0;
  uint64_t (*wall_clock_)() = nullptr; // firmware's clock, see setWallClock()
  const char *state_ = state_name::kBoot;

  bool homed_ = false;
  uint32_t fault_mask_ = 0;
  uint8_t fault_bit_ = 0;
  const char *fault_detail_ = nullptr;
  bool fault_cause_active_ = false;

  // Boot leaves the motor rail off: the 36 V input block's ENABLE is pulled low
  // and the rail is genuinely off, which is why HV is observable rather than
  // something the host infers.
  bool hv_on_ = false;
  // The InstrumentPort buffers are enabled at boot for the proof of life
  // (protocol v1: INSTR STATE reports on at boot).
  bool instr_on_ = true;

  // Protocol v1 state.
  TrialWatch watch_;
  double brake_level_ = 0.0;   // commanded N.cm while engaging or RELEASED
  bool brake_pending_ = false; // BRAKE acked, brake settling, drive not released
  double deadband_deg_ = 0.5;
  uint16_t stream_hz_ = 10;
  bool stream_on_ = false;
  uint32_t stream_period_us_ = 100000u;
  uint64_t stream_next_us_ = 0;
  bool angout_on_ = false;
  double angscale_lo_ = -20.0;
  double angscale_hi_ = 20.0;

  long cal_enc_offset_ = 1024;
  double cal_angle_zero_ = 0.0;

  bool watchdog_armed_ = false;
  uint64_t watchdog_deadline_us_ = 0;

  // `home;<s>;<timeout_ms>`: the host's bound on the running home, clipped to
  // the firmware maximum. Cleared by completion, STOP, DISARM and any fault.
  bool home_deadline_armed_ = false;
  uint64_t home_deadline_us_ = 0;

  char rx_[protocol::kMaxLine * 2] = {0};
  size_t rx_len_ = 0;
  bool rx_overflow_ = false;

  Entry cache_[kRetransmitCacheSize];
  uint8_t cache_count_ = 0;

  char pending_[kMaxPendingLines][protocol::kMaxLine + 2] = {{0}};
  uint8_t pending_count_ = 0;
  bool pending_overflow_ = false;
};

}  // namespace mouse_joystick
