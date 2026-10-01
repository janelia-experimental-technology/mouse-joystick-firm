// The seam between the pyControl protocol layer and whatever actually moves.
//
// `ProtocolBoard` owns framing, the state machine, the watchdog and the
// retransmission cache. It owns no motion. Everything about *where the joystick
// is* and *how it gets somewhere* lives behind this interface, for two reasons:
//
//  1. The shared corpus (`pycontrol/tests/fixtures/protocol_cases.txt`) has to
//     run on the host, where there is no TMC5130A and no clock. `SimulatedMotion`
//     below is what makes that possible, and it is deliberately the same linear
//     model the Python simulator uses so the two agree on protocol behaviour
//     without either pretending to model a ramp generator.
//
//  2. Real motion is non-blocking and the bench console's is not.
//     `MotionSetup::moveTo()` returns a `MoveReport` *after* the move finishes,
//     which is right for a `j`-at-a-terminal workflow and wrong here: the
//     protocol must answer `ack` immediately and keep servicing the link and the
//     host-loss watchdog for the whole move. `TmcMotion` (firmware only) drives
//     the same TMC51X0 primitives without waiting.
//
// Arduino-free, like everything the native test compiles.

#pragma once

#include <stdint.h>

namespace mouse_joystick {

// The firmware maximum for one homing cycle; the real backend's own bound
// (`TmcMotion::Config::home_timeout_us`) is the same number.
constexpr uint32_t kHomeTimeoutMaxUs = 8000000u;

class MotionBackend {
public:
  virtual ~MotionBackend() {}

  // Advance internal state to `now_us`. A real backend uses this as its poll
  // tick; the simulated one uses it to complete a move at a computed deadline.
  virtual void poll(uint64_t now_us) = 0;

  // True while a commanded move is in progress.
  virtual bool isMoving() const = 0;

  // True exactly once per completed move, so a completion cannot be reported
  // twice or missed. `was_home` distinguishes HOMED from TARGET.
  virtual bool consumeCompletion(bool *was_home) = 0;

  // The earliest time `poll()` could produce a completion, or 0 for "unknown".
  // The simulated backend returns its motion deadline, which is what lets a
  // coarse `advance()` still land an event on the exact microsecond the corpus
  // expects. A real backend returns 0: completion is discovered by polling, not
  // predicted.
  virtual uint64_t nextEventUs() const = 0;

  // Begin homing. The datum is applied by the backend on completion, so after a
  // successful home `angleDeg()` reads zero without the protocol layer having to
  // reach in and set it.
  virtual bool startHome() = 0;

  // Begin a bounded absolute move, in degrees.
  virtual bool startMoveTo(double target_deg) = 0;

  // Stop and hold wherever the joystick is. Used by STOP, DISARM, HV OFF and
  // every fault path, so it must be safe to call when nothing is moving.
  virtual void abort() = 0;

  // What the encoder says. The only position that is ever true.
  virtual double angleDeg() const = 0;

  // What the ramp generator thinks. True only while the board is driving, and
  // stale the moment the animal moves the joystick in FREEWHEEL.
  virtual double xactualDeg() const = 0;

  // Reload the ramp generator from the encoder. ONLY on leaving a compliant
  // state -- never during commanded motion and never on a timer. `XACTUAL`
  // advancing while `X_ENC` does not is the board's only detector for an end
  // stop, a stall, a slipping coupling or a dead encoder, and that divergence
  // IS the detector, so a resync deletes it.
  virtual void resyncFromEncoder() = 0;

  // Energise the driver. False means refused, and the caller faults rather than
  // proceeding: on real hardware this is where `CHOPCONF.vsense` is read back,
  // because vsense resets to 0 and against this board's 0.15 ohm shunts that
  // drives roughly 1.8x the motor's rating.
  virtual bool enableDriver() = 0;

  // Release the driver so the animal can move the joystick.
  virtual void releaseDriver() = 0;

  // How long this backend expects homing to take (the simulator's fixed
  // duration; the real backend reports its own bound). Informational: the
  // protocol layer no longer refuses a home because the host's timeout is
  // shorter than this. Found in the 2026-10-01 review: comparing the request
  // against the real backend's 8 s bound faulted every `home;<s>;<ms>` under
  // 8000 ms immediately.
  virtual uint32_t expectedHomeDurationUs() const = 0;

  // The firmware's own maximum for a homing cycle. A host-supplied timeout is
  // clipped to this and enforced as a deadline while homing runs; exceeding it
  // is a fault and not a nak, because by then the motion has started.
  virtual uint32_t homeTimeoutMaxUs() const { return kHomeTimeoutMaxUs; }

  // Is the limit switch asserted? One of the six fields `STATUS` must report,
  // and a host may reasonably use it to decide whether homing is safe to start.
  //
  // Defaulted false rather than pure because the host model has no switch. Note
  // the draft gives `STATUS` no way to encode "unknown", so a backend that
  // cannot read the switch -- no bus yet, for instance -- has to say false. That
  // is a gap in the draft, not a measurement.
  virtual bool limitAsserted() const { return false; }

  // Switch the motor rail. Part of this interface rather than a separate seam
  // because the 36 V domain exists to feed the driver -- it is the supply of
  // the thing that moves, and nothing else on the board uses it.
  //
  // Returns false if the rail did not come up, which the protocol layer turns
  // into a fault rather than a nak: the attempt was made and the hardware
  // answered. On real hardware this is not a pin wiggle. With `R203` fitted --
  // which is every board in hand -- a single `HV_EN` assertion cannot charge
  // the 208 uF bulk and the switch latches off; the staircase soft-start is the
  // path that works, and it takes about 60 ms. That is comfortably inside the
  // host's timeout, and the watchdog is not armed until ARM.
  //
  // Defaulted, so a backend with no rail need not restate it.
  virtual bool setHvEnabled(bool on) {
    (void)on;
    return true;
  }

  // Asynchronous failure, reported once and then cleared. Real motion fails in
  // ways no command can predict -- driver over-temperature, a short to ground,
  // an unexpected end stop, homing that never saw the switch -- and the
  // protocol layer has to turn those into a latched fault and a `FAULT` event
  // without a host command in flight. Returns true at most once per fault.
  //
  // Defaulted rather than pure so a backend with no failure modes of its own
  // need not restate that. `SimulatedMotion` is such a backend: its faults are
  // provoked through the protocol, not discovered by polling.
  virtual bool consumeFault(uint8_t *bit, const char **detail) {
    (void)bit;
    (void)detail;
    return false;
  }
};

// ---------------------------------------------------------------------------
// The host-testable model. Linear travel at a fixed velocity, a fixed homing
// duration, and an injected clock -- which is exactly `BoardSim`'s model, and
// the reason the two implementations can be held to one corpus.
//
// !! THE DEFAULTS ARE THE SIMULATOR'S PLACEHOLDERS, NOT MEASURED VALUES.
//    180 deg/s and a 50 ms home are what the corpus was written against so its
//    move distances divide evenly. The bench measured travel at ~4000
//    microsteps (~28 deg at the motor shaft), jogs at 0.100 rev/s, and homing at
//    2531 ms of which the approach is 365 ms -- see
//    `../mouse-joystick-pcb/docs/bringup-log.md` under board 2. Changing these
//    changes corpus timings, so it is a deliberate act with a corpus update
//    alongside, not a tidy-up.
// ---------------------------------------------------------------------------
class SimulatedMotion : public MotionBackend {
public:
  struct Config {
    double v_max_deg_s = 180.0;
    uint32_t home_duration_us = 50000u;
    // Set false to make the ARM-time driver enable refuse, which is how the
    // vsense self-test path is provoked without hardware.
    bool driver_enable_ok = true;
  };

  SimulatedMotion() {}
  explicit SimulatedMotion(const Config &config) : cfg_(config) {}

  void poll(uint64_t now_us) override {
    now_us_ = now_us;
    if (moving_ && now_us_ >= end_us_) {
      moving_ = false;
      angle_ = target_;
      // Under commanded motion the encoder tracks the ramp generator. That they
      // agree here is what makes their disagreement elsewhere mean something.
      xactual_ = angle_;
      if (is_home_) {
        angle_ = 0.0;
        xactual_ = 0.0;
      }
      completion_pending_ = true;
      completion_was_home_ = is_home_;
    }
  }

  bool isMoving() const override { return moving_; }

  bool consumeCompletion(bool *was_home) override {
    if (!completion_pending_) {
      return false;
    }
    completion_pending_ = false;
    if (was_home != nullptr) {
      *was_home = completion_was_home_;
    }
    return true;
  }

  uint64_t nextEventUs() const override { return moving_ ? end_us_ : 0; }

  bool startHome() override {
    start(true, 0.0, cfg_.home_duration_us);
    return true;
  }

  bool startMoveTo(double target_deg) override {
    double distance = target_deg - angle_;
    if (distance < 0.0) {
      distance = -distance;
    }
    uint32_t duration_us = 0;
    if (cfg_.v_max_deg_s > 0.0) {
      duration_us =
          static_cast<uint32_t>(distance / cfg_.v_max_deg_s * 1000000.0);
    }
    start(false, target_deg, duration_us);
    return true;
  }

  void abort() override {
    if (moving_) {
      angle_ = interpolate();
      xactual_ = angle_;
      moving_ = false;
    }
    completion_pending_ = false;
  }

  double angleDeg() const override { return angle_; }
  double xactualDeg() const override { return xactual_; }
  void resyncFromEncoder() override { xactual_ = angle_; }

  bool enableDriver() override { return cfg_.driver_enable_ok; }
  void releaseDriver() override {}

  uint32_t expectedHomeDurationUs() const override {
    return cfg_.home_duration_us;
  }

  // Test hook: move the joystick the way an animal would, with no driver
  // involvement, so the encoder and the ramp generator genuinely diverge.
  void setEncoderAngleDeg(double deg) { angle_ = deg; }

private:
  void start(bool is_home, double target, uint32_t duration_us) {
    moving_ = true;
    is_home_ = is_home;
    start_ = angle_;
    target_ = target;
    start_us_ = now_us_;
    // At least one microsecond, so a zero-distance move still completes through
    // the same path as any other rather than never completing at all.
    end_us_ = now_us_ + (duration_us > 0 ? duration_us : 1);
    completion_pending_ = false;
  }

  double interpolate() const {
    if (!moving_) {
      return angle_;
    }
    uint64_t span = end_us_ - start_us_;
    if (span == 0) {
      return target_;
    }
    double fraction = static_cast<double>(now_us_ - start_us_) /
                      static_cast<double>(span);
    if (fraction < 0.0) {
      fraction = 0.0;
    }
    if (fraction > 1.0) {
      fraction = 1.0;
    }
    return start_ + (target_ - start_) * fraction;
  }

  Config cfg_;
  uint64_t now_us_ = 0;

  double angle_ = 0.0;
  double xactual_ = 0.0;

  bool moving_ = false;
  bool is_home_ = false;
  double start_ = 0.0;
  double target_ = 0.0;
  uint64_t start_us_ = 0;
  uint64_t end_us_ = 0;

  bool completion_pending_ = false;
  bool completion_was_home_ = false;
};

}  // namespace mouse_joystick
