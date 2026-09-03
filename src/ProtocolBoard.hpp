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
//    in `../mouse-joystick-pcb/docs/bringup-log.md` under board 2.

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "MotionBackend.hpp"
#include "Protocol.hpp"

namespace mouse_joystick {

namespace state_name {
constexpr char kBoot[] = "BOOT";
constexpr char kIdle[] = "IDLE";
constexpr char kHomed[] = "HOMED";
constexpr char kArmed[] = "ARMED";
constexpr char kActive[] = "ACTIVE";
constexpr char kFaulted[] = "FAULTED";
// The state in which the ANIMAL drives the joystick. The driver is released so
// motor torque does not add to the brake's, and the ramp generator's position
// stops meaning anything -- only the encoder knows where the joystick is.
constexpr char kFreewheel[] = "FREEWHEEL";
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
}  // namespace fault

class ProtocolBoard {
public:
  static constexpr uint8_t kMaxPendingLines = 8;
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
  };
  // `home_duration_us` and the vsense read-back used to live here. They are
  // properties of the thing that moves, so they belong to the backend:
  // `SimulatedMotion::Config` for the host, a CHOPCONF read-back for the real
  // driver. Asking the backend keeps one answer instead of two that can drift.

  // The backend is a reference, not owned: the caller decides its lifetime and
  // its identity, which is the whole point of the seam. Two constructors rather
  // than one with `= Config()`, because a default argument naming a nested type
  // needs the enclosing class to be complete and it is not at this point.
  explicit ProtocolBoard(MotionBackend &motion) : motion_(motion) {}
  ProtocolBoard(MotionBackend &motion, const Config &config)
      : cfg_(config), motion_(motion) {}

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
    line.beginEvent("READY", static_cast<uint32_t>(now_us_));
    line.kvStr("fw", cfg_.fw_version);
    line.kvInt("proto", protocol::kProtocolVersion);
    line.kvStr("serial", cfg_.serial);
    push(line);
  }

  // Advance to the next scheduled thing rather than in one jump, so an event
  // never lands at the wrong timestamp because the caller advanced coarsely.
  void advance(uint64_t delta_us) {
    uint64_t target = now_us_ + delta_us;
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
      now_us_ = next_stop;
      service();
      if (next_stop == target) {
        break;
      }
    }
    now_us_ = target;
    service();
  }

  // Firmware sets this once, in setup(), from the flash unique id. Not done in
  // the constructor because reading it issues a real QSPI command and the board
  // is constructed before the runtime is fully up.
  void setSerial(const char *serial) {
    if (serial != nullptr && *serial != '\0') {
      cfg_.serial = serial;
    }
  }

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
  double angle() const { return motion_.angleDeg(); }
  double xactual() const { return motion_.xactualDeg(); }
  bool homed() const { return homed_; }
  bool hvOn() const { return hv_on_; }
  uint32_t faultMask() const { return fault_mask_; }

private:
  // ------------------------------------------------------------- line intake

  void handleLine(const char *line) {
    protocol::Command cmd;
    if (!protocol::decodeCommand(line, &cmd)) {
      // Deliberately BEFORE the watchdog refresh: a frame that does not parse
      // does not prove the host is alive. A wedged serial driver emitting
      // garbage must not hold the safety timeout open. Resolved 2026-09-03 in
      // `../mouse-joystick-pcb/docs/uart-protocol.md` open question 1.
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
    snprintf(out, n, "%s", line.c_str());
  }

  void remember(uint16_t seq, const char *canonical, const char *reply) {
    for (uint8_t i = 0; i < cache_count_; ++i) {
      if (cache_[i].seq == seq) {
        snprintf(cache_[i].canonical, sizeof(cache_[i].canonical), "%s",
                 canonical);
        snprintf(cache_[i].reply, sizeof(cache_[i].reply), "%s", reply);
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
    snprintf(slot.canonical, sizeof(slot.canonical), "%s", canonical);
    snprintf(slot.reply, sizeof(slot.reply), "%s", reply);
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
    snprintf(pending_[pending_count_], sizeof(pending_[0]), "%s", line.c_str());
    ++pending_count_;
  }

  void emitEvent(const char *name, const char *key, double value) {
    protocol::Line line;
    line.beginEvent(name, static_cast<uint32_t>(now_us_));
    if (key != nullptr) {
      line.kvNumber(key, value);
    }
    push(line);
  }

  // --------------------------------------------------------------- scheduler

  void service() {
    motion_.poll(now_us_);

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

    bool was_home = false;
    if (motion_.consumeCompletion(&was_home)) {
      if (was_home) {
        homed_ = true;
        state_ = state_name::kHomed;
        emitEvent("HOMED", "angle", motion_.angleDeg());
      } else {
        state_ = state_name::kArmed;
        emitEvent("TARGET", "angle", motion_.angleDeg());
      }
    }

    // PULL: the joystick crossed the armed threshold while compliant.
    //
    // Only in FREEWHEEL, deliberately. There the joystick is driven by the
    // animal and a crossing means something; under commanded motion a crossing
    // means the board did it, and reporting that as a pull would put a
    // fabricated event into a behavioural dataset.
    //
    // Which side counts as "past" is recorded at ARMPULL time rather than
    // baked in as a sign. The trial presents the joystick somewhere and the
    // animal moves it away, so the direction follows from where it was when
    // detection was armed -- and this way the same firmware serves the left and
    // right joysticks, whose travel is mirrored, with no side-specific state.
    //
    // Latched: one ARMPULL yields one PULL. Re-arming is how a task asks for
    // the next trial's.
    if (has_pull_threshold_ && !pull_reported_ &&
        inState(state_name::kFreewheel)) {
      const double angle = motion_.angleDeg();
      const bool crossed = pull_armed_above_ ? (angle < pull_threshold_)
                                             : (angle > pull_threshold_);
      if (crossed) {
        pull_reported_ = true;
        emitEvent("PULL", "angle", angle);
      }
    }

    if (watchdog_armed_ && now_us_ >= watchdog_deadline_us_) {
      watchdog_armed_ = false;
      raiseFault(fault::kHostTimeout, "host_timeout", false);
    }
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
  // Raised as an open question in `../mouse-joystick-pcb/docs/uart-protocol.md`.
  void raiseFault(uint8_t bit, const char *detail, bool cause_active) {
    motion_.abort();
    watchdog_armed_ = false;
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
    line.beginEvent("FAULT", static_cast<uint32_t>(now_us_));
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
    motion_.resyncFromEncoder();
    return motion_.enableDriver();
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
    if (strcmp(v, "move") == 0) return cmdMove(cmd, out, true);
    if (strcmp(v, "freewheel") == 0) return cmdFreewheel(cmd, out);
    if (strcmp(v, "armpull") == 0) return cmdArmPull(cmd, out);
    if (strcmp(v, "cal") == 0) return cmdCal(cmd, out);

    // "An unrecognized verb is rejected, never partially executed."
    //
    // ANGOUT and ANGSCALE land here deliberately. The analog outputs are not
    // populated on the boards in hand, so implementing them would assert what
    // was guessed. `badverb` means a host cannot believe the analog output is
    // live and record a trace of nothing.
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
    out->kvStr("caps", "motion+home+freewheel+instr+hv");
    out->kvInt("rxbuf", protocol::kMaxLine);
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
    out->kvBool("homed", homed_);
    out->kvBool("limit", motion_.limitAsserted());
    out->kvInt("faults", static_cast<long>(fault_mask_));
    out->kvBool("hv", hv_on_);
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
      out->beginAck(cmd.seq);
      return true;
    }
    if (argEquals(cmd.arg(0), "off")) {
      // Always permitted: the board never requires a message in order to become
      // safe, and this is the message that makes it safer. The return value is
      // ignored on purpose -- there is no useful answer to "turning it off
      // failed", and refusing to report success would leave a host unable to
      // confirm the safe state it just asked for.
      motion_.setHvEnabled(false);
      hv_on_ = false;
      if (inState(state_name::kArmed) || inState(state_name::kActive)) {
        motion_.abort();
        state_ = state_name::kHomed;
        watchdog_armed_ = false;
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
      instr_on_ = argEquals(cmd.arg(0), "on");
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
    if (motion_.isMoving()) {
      out->nak(cmd.seq, protocol::kNakBusy);
      return true;
    }
    if (inState(state_name::kFreewheel)) {
      // No re-engage here: `enableDriver()` is called below for every home, so
      // adding one would be a redundant SPI round trip.
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

    if (bounded && timeout_us < motion_.expectedHomeDurationUs()) {
      // Homing is bounded locally, and exceeding the bound is a fault rather
      // than a nak because the motion did start. The bound comes from the
      // backend, which is the only thing that knows how long homing takes.
      motion_.abort();
      out->beginAck(cmd.seq);
      push(*out);
      out->clear();
      raiseFault(fault::kHomeTimeout, "home_timeout", false);
      return false;  // already pushed, in order: ack then the FAULT event
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
    // rule stands. Raised in `../mouse-joystick-pcb/docs/uart-protocol.md`.
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
    if (!motion_.startHome()) {
      out->nak(cmd.seq, protocol::kNakBusy);
      return true;
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
    watchdog_armed_ = false;
    if (inState(state_name::kFreewheel)) {
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
    if (inState(state_name::kFreewheel)) {
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
    // FREEWHEEL re-engages implicitly. One command that resyncs, re-engages and
    // moves is better than three the host could be interrupted between.
    if (inState(state_name::kFreewheel)) {
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

  // Release the driver so the animal can move the joystick.
  //
  // On a board WITH a brake this verb should not exist separately: the brake
  // must engage BEFORE drive is released or the joystick drops, and be released
  // only after drive is restored or the motor fights it. Two host commands
  // cannot guarantee that ordering across a link whose whole point is that it
  // has latency. Expect `BRAKE <torque>` to REPLACE this, not extend it.
  bool cmdFreewheel(const protocol::Command &cmd, protocol::Line *out) {
    if (faulted()) {
      out->nak(cmd.seq, protocol::kNakFaulted);
      return true;
    }
    if (inState(state_name::kActive)) {
      out->nak(cmd.seq, protocol::kNakBusy);
      return true;
    }
    if (!inState(state_name::kArmed)) {
      // A joystick held by an energised motor cannot go compliant by accident.
      out->nak(cmd.seq, protocol::kNakNotArmed);
      return true;
    }
    motion_.releaseDriver();
    state_ = state_name::kFreewheel;
    pull_reported_ = false;
    out->beginAck(cmd.seq);
    return true;
  }

  bool cmdArmPull(const protocol::Command &cmd, protocol::Line *out) {
    if (cmd.arg_count != 1) {
      out->nak(cmd.seq, protocol::kNakBadArgs);
      return true;
    }
    double threshold = 0.0;
    if (!protocol::parseNumber(cmd.arg(0), &threshold)) {
      out->nak(cmd.seq, protocol::kNakBadArgs);
      return true;
    }
    if (threshold < cfg_.travel_lo || threshold > cfg_.travel_hi) {
      out->nak(cmd.seq, protocol::kNakOutOfRange);
      return true;
    }
    pull_threshold_ = threshold;
    has_pull_threshold_ = true;
    // Which side of the threshold the joystick is on NOW. A crossing to the
    // other side is the pull. Equality counts as below, so a threshold armed
    // exactly at the current position fires on the first movement upward
    // rather than immediately.
    pull_armed_above_ = motion_.angleDeg() > threshold;
    // Re-arming clears the latch, so one ARMPULL yields one PULL per trial
    // rather than a stream while the joystick sits past threshold.
    pull_reported_ = false;
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

  uint64_t now_us_ = 0;
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
  bool instr_on_ = false;

  double pull_threshold_ = 0.0;
  bool has_pull_threshold_ = false;
  bool pull_armed_above_ = false;
  bool pull_reported_ = false;

  long cal_enc_offset_ = 1024;
  double cal_angle_zero_ = 0.0;

  bool watchdog_armed_ = false;
  uint64_t watchdog_deadline_us_ = 0;

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
