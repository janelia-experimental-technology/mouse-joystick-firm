// A non-blocking `MotionBackend` over the TMC5130A.
//
// FIRMWARE ONLY. Needs Arduino and TMC51X0, so this header is deliberately not
// reachable from `MouseJoystickBoard.hpp` or from the native test -- the
// protocol layer and the corpus stay host-testable precisely because they do
// not include this.
//
// WHY THIS EXISTS RATHER THAN CALLING MotionSetup DIRECTLY
//
// `MotionSetup::moveTo()` and `homeToSwitch()` are blocking: they return a
// report once the motion has finished. That is right for the bench console --
// type `j`, read a paragraph -- and wrong here. The protocol must answer `ack`
// immediately and keep servicing the link for the whole move, because the host
// polls a heartbeat inside a 100 ms timeout and a 2.5 s blocking home would
// silently fail every one of them. So this drives the same TMC51X0 primitives
// `MotionSetup` does, in the same order, without waiting.
//
// Every register read goes through `MotionSetup::snapshot()` ONCE per poll.
// `RAMP_STAT`'s latch and event flags are read-and-clear, so a second read in
// the same poll loses whichever flag the first consumed -- a trap that cost a
// bench cycle on 2026-09-02 and is recorded in `docs/motion-bringup.md`.
//
// FOUND ON THE FIRST HARDWARE RUN, 2026-09-03, AND FIXED HERE
//
// `angleDeg()` and `xactualDeg()` used to read position over SPI on every call.
// `STATUS` calls both, and `STATUS` is specified "safe at any time" -- so the
// first `status` sent before `hv on` reached for SPI0 before `beginSpi()` had
// ever run, with the isolation bus switch still open, and hung the core. The
// board then stopped answering the link AND stopped honouring the 1200 baud
// reset touch, which needed `SW501` and a power cycle to recover.
//
// The corpus did not catch it and could not have. `status-is-safe-before-
// anything-is-configured` passes on the host because `SimulatedMotion` just
// returns a number; the corpus pins protocol BEHAVIOUR, not hardware
// preconditions. That is a real limit of holding two implementations to one
// fixture, and it is worth knowing rather than being surprised by twice.
//
// The rule now: **nothing here touches SPI unless `bus_up_`**, and position is
// read once per `poll()` into a cache that the accessors return. That also
// removes several bus transactions per reply, and it is what makes a pull
// visible in FREEWHEEL -- the encoder moves with no commanded motion, so a
// cache refreshed only during a move would report a stale angle for the whole
// pull.
//
// THE FOLLOW CHECK, ADDED 2026-09-03 BEFORE THE FIRST HOME
//
// The validated `homeToSwitch()` runs its approach through `moveTo()`, which
// carries an encoder follow check: `MoveOutcome::kNotFollowing` maps to
// `HomeOutcome::kMechanicalStopFirst`. The first version of this file commanded
// the target directly and had no such check, which made it strictly weaker than
// the implementation stage 3 proved -- and weaker in exactly the wrong place.
//
// The homing approach commands `kHomeTravelBoundMicrosteps` = 102400 microsteps
// = 720 deg at the motor shaft, against a mechanism with about 28 deg of
// travel. The chip's reference-switch stop is what normally ends it. If the
// switch does not assert -- a stuck switch, an open `U1301` output, a harness
// fault -- then without a follow check nothing notices the mechanism has
// stopped, and the motor pushes against it until the 8 s timeout.
//
// `XACTUAL` advancing while `X_ENC` does not is the board's only detector for
// that, and it is the same divergence the protocol layer must NOT resync away.
// Thresholds are taken from `Conversions.hpp` rather than chosen here, so this
// and `MotionSetup::moveTo()` agree.
//
// RAN ON HARDWARE 2026-09-03, over the protocol (`hv on`, `home`, `arm`,
// `moveto`, `disarm`, `clrfault` on the RJ-45 with the motor moving): homing
// repeats to about one encoder count and the commanded-move follow ratio is
// 0.991. Record in `.agents/PLANS.md` ("Stage 4 ran the full motion path on
// hardware") and `../mouse-joystick-pcb/docs/bringup-log.md` under board 3.
// Stage 3 had validated the *blocking* implementations in `MotionSetup`; this
// is the non-blocking rearrangement of the same calls. What is deliberately NOT
// reimplemented here is the second homing stage -- see `startHome()`.

#pragma once

#include <Arduino.h>
#include <TMC51X0.hpp>
#include <pico/time.h> // time_us_64
#include <stdint.h>

#include "Conversions.hpp"
#include "MotionBackend.hpp"
#include "MotionSetup.hpp"
#include "ProtocolBoard.hpp"

namespace mouse_joystick {

class TmcMotion : public MotionBackend {
public:
  // Bring-up breadcrumbs (2026-10-02): an optional hook called with a
  // checkpoint number at each step of the rail-up sequence, so a hang that no
  // trace can flush is placed from a GPIO pulse train and the watchdog
  // scratch register. The QP BSP installs one; nothing happens without it.
  //   1 setHvEnabled(true) entered, before powerUp
  //   2 powerUp returned                 3 connectBus done
  //   4 beginSpi done                    5 TMC version and link verified
  //   6 SD_MODE check passed             7 configure done, bus up
  //   8 position cache seeded, returning true
  using CheckpointHook = void (*)(uint8_t);
  void setCheckpointHook(CheckpointHook hook) { checkpoint_hook_ = hook; }

private:
  CheckpointHook checkpoint_hook_ = nullptr;
  void checkpoint(uint8_t n) const {
    if (checkpoint_hook_ != nullptr) {
      checkpoint_hook_(n);
    }
  }

public:
  struct Config {
    // Bounds. Motion that exceeds one of these is a fault and not a nak,
    // because by then it has started.
    uint32_t move_timeout_us = 5000000u;   // 5 s; the widest travel is ~0.3 s
    uint32_t home_timeout_us = 8000000u;   // 8 s; a measured home is ~2.5 s
    int32_t home_travel_bound_microsteps = conversions::kHomeTravelBoundMicrosteps;
    int32_t home_clearance_microsteps = conversions::kHomeBackoffMicrosteps;
  };

  TmcMotion(TMC51X0 &stepper, const MotionConfig &motion, const Config &config)
      : stepper_(stepper), motion_(motion), cfg_(config) {}

  // ------------------------------------------------------------------ policy

  void poll(uint64_t now_us) override {
    now_us_ = now_us;

    // No bus, no reads. Before `hv on` there is no SPI0 and the isolation bus
    // switch is open; reaching for the driver here is what hung the core on the
    // first hardware run.
    if (!bus_up_) {
      return;
    }
    // Once per tick, not once per scheduler stop. The protocol layer services
    // at every scheduled instant, and in RELEASED the classifier's 1 kHz ring
    // adds one stop per millisecond, so a tick that covers two milliseconds
    // (the decimated sampler) was reading the bus three times: that, times
    // ~300 us a read with the brake and torque poll alongside, was the 1.2 ms
    // tick measured on pcb v1.0 unit A on 2026-10-06. A second poll inside
    // the same half-millisecond reuses the cache; the simulated backend has
    // no such limit, so the corpus timing is untouched.
    //
    // Two guards, both learnt on unit A on 2026-10-06. Keyed to `now_us`
    // the limit let every catch-up stop of a late tick hit the bus (they
    // are a millisecond apart on the protocol clock, microseconds apart on
    // the wall): 1000 polls/s against 500 handled ticks, handlers of
    // 1-2.4 ms. Keyed to the wall clock alone it still let the SECOND stop
    // poll once the first poll had itself taken longer than the window
    // (730 polls/s, 10:37). So: at most one bus read per advance of the
    // protocol clock (beginAdvance()), and never two within half a
    // millisecond of wall-clock time (a host line arrives between ticks
    // and advances twice).
    const uint64_t wall_us = time_us_64();
    if (polled_this_advance_ ||
        (last_bus_poll_wall_us_ != 0 &&
         (wall_us - last_bus_poll_wall_us_) < kPollMinIntervalUs)) {
      ++poll_reused_;
      return;
    }
    polled_this_advance_ = true;
    last_bus_poll_wall_us_ = wall_us;
    const uint32_t poll_t0 = micros();
    pollBus();
    const uint32_t cost = micros() - poll_t0;
    ++poll_calls_;
    if (cost > poll_cost_max_us_) {
      poll_cost_max_us_ = cost;
    }
  }

  // Bring-up accounting (2026-10-06): the worst `poll()` since the last read
  // and how many there were, for the POLL_STAT trace record. Reset on read.
  uint32_t takePollCostMaxUs(uint32_t *calls, uint32_t *enc_max_us = nullptr,
                             uint32_t *snapshot_max_us = nullptr) {
    const uint32_t m = poll_cost_max_us_;
    if (calls != nullptr) {
      *calls = poll_calls_;
    }
    if (enc_max_us != nullptr) {
      *enc_max_us = enc_read_max_us_;
    }
    if (snapshot_max_us != nullptr) {
      *snapshot_max_us = snapshot_max_us_;
    }
    poll_cost_max_us_ = 0;
    poll_calls_ = 0;
    enc_read_max_us_ = 0;
    snapshot_max_us_ = 0;
    return m;
  }

  void beginAdvance() override { polled_this_advance_ = false; }

  // Bus reads closer together than this, on the wall clock, reuse the cache
  // (see poll()).
  static constexpr uint64_t kPollMinIntervalUs = 500u;
  uint32_t pollsReused() const { return poll_reused_; }

private:
  uint32_t poll_cost_max_us_ = 0;
  uint32_t poll_calls_ = 0;
  uint32_t enc_read_max_us_ = 0;
  uint32_t snapshot_max_us_ = 0;
  uint64_t last_bus_poll_wall_us_ = 0; // time_us_64() of the last bus poll
  bool polled_this_advance_ = false;
  uint32_t poll_reused_ = 0;

  void pollBus() {

    // Every poll reads the encoder (X_ENC, one register, two datagrams),
    // because the encoder moves when the animal moves the joystick and
    // nothing else would notice. Every kFullSnapshotEvery-th poll reads the
    // full snapshot (RAMP_STAT, DRV_STATUS, XACTUAL, VACTUAL and X_ENC) and
    // decides completion, the limit switch and driver faults, so those are
    // seen within ten polls. (2026-10-02: the full snapshot on every 1 kHz
    // tick, at the library's 1 MHz, was most of the tick.) See the header
    // note about read-and-clear flags: RAMP_STAT is read once per snapshot.
    ++poll_count_;
    if ((poll_count_ % kFullSnapshotEvery) != 0u) {
      const uint32_t e0 = micros();
      angle_deg_ = conversions::microstepsToDegrees(
          stepper_.encoder.readActualPosition());
      const uint32_t e_cost = micros() - e0;
      if (e_cost > enc_read_max_us_) {
        enc_read_max_us_ = e_cost;
      }
      return;
    }
    const uint32_t sn0 = micros();
    const MotionSnapshot s = MotionSetup::snapshot(stepper_);
    const uint32_t sn_cost = micros() - sn0;
    if (sn_cost > snapshot_max_us_) {
      snapshot_max_us_ = sn_cost;
    }
    angle_deg_ = conversions::microstepsToDegrees(s.encoder);
    xactual_deg_ = conversions::microstepsToDegrees(s.position);

    // Only meaningful while the driver is energised. A disabled driver's
    // over-temperature and short-to-ground bits are not a fault report.
    if (driver_enabled_) {
      tmc51x0::Registers::DrvStatus drv;
      drv.raw = s.drv_status;
      if (drv.ot()) {
        fail(fault::kDriverFault, "driver_overtemp");
        return;
      }
      if (drv.s2ga() || drv.s2gb()) {
        fail(fault::kDriverFault, "driver_short_to_gnd");
        return;
      }
    }

    if (phase_ == Phase::kIdle) {
      return;
    }

    tmc51x0::Registers::RampStat ramp;
    ramp.raw = s.ramp_stat;

    // The limit switch, from the snapshot already taken rather than a second
    // read of IOIN. `status_stop_*` are plain status bits, not read-and-clear,
    // and they reflect the pin whether or not the hardware stop is armed --
    // which is what makes them usable for reporting and not just for stopping.
    // Which of the two carries the single fitted switch is set by
    // `SW_MODE.swap_lr` in `armStop()`, from the end of travel it sits on.
    limit_asserted_ = (motion_.switch_end == HomeEnd::kPositive)
                          ? ramp.status_stop_r()
                          : ramp.status_stop_l();

    switch (phase_) {
      case Phase::kMove:
        pollMove(s, ramp);
        break;
      case Phase::kHomeBackoff:
        pollHomeBackoff(s, ramp);
        break;
      case Phase::kHomeApproach:
        pollHomeApproach(s, ramp);
        break;
      case Phase::kHomeClearance:
        pollHomeClearance(s, ramp);
        break;
      case Phase::kIdle:
        break;
    }
  }

public:
  bool isMoving() const override { return phase_ != Phase::kIdle; }

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

  bool consumeFault(uint8_t *bit, const char **detail) override {
    if (!fault_pending_) {
      return false;
    }
    fault_pending_ = false;
    if (bit != nullptr) {
      *bit = fault_bit_;
    }
    if (detail != nullptr) {
      *detail = fault_detail_;
    }
    return true;
  }

  // Real completion is discovered by polling, never predicted, so the protocol
  // layer's scheduler gets no deadline from here and simply polls.
  uint64_t nextEventUs() const override { return 0; }

  // Single-stage homing: fast approach, stop at the switch's CLOSING edge, take
  // that as the datum, then clear the switch.
  //
  // THE SECOND STAGE IS DELIBERATELY ABSENT, and the bench is why. Measured
  // 2026-09-02 over four series of five runs: two-stage and single-stage homing
  // have the SAME repeatability, 2-5 encoder counts. What the slow creep buys is
  // datum *accuracy* -- the datum stops moving with approach velocity, 89
  // microsteps of offset at 0.200 rev/s against 4.5 at the creep -- and a
  // faster primary approach. Not precision.
  //
  // So a single stage is a defensible rev-0 implementation with a known,
  // measured cost, and it is a small amount of code whose failure modes are
  // bounded by the chip's own reference-switch stop. A blind six-state
  // reimplementation of the validated two-stage sequence would not be.
  // Recorded as a follow-up in `.agents/PLANS.md`.
  bool startHome() override {
    if (!bus_up_ || phase_ != Phase::kIdle) {
      return false;
    }
    started_us_ = now_us_;

    // Datasheet step 1: do not approach a switch that is already pressed.
    // Nothing is armed for this, which is correct -- a switch reading active
    // while the mechanism is nowhere near it would otherwise block every move.
    MotionSetup::armStop(stepper_, /*armed=*/false, motion_.switch_active_high,
                         motion_.switch_end, LatchEdge::kActive);
    MotionSetup::setJogVelocity(stepper_);

    if (MotionSetup::switchActive(stepper_)) {
      const int32_t here = stepper_.controller.readActualPosition();
      commandTo(here - toward() * cfg_.home_clearance_microsteps);
      phase_ = Phase::kHomeBackoff;
      return true;
    }
    beginApproach();
    return true;
  }

  bool startMoveTo(double target_deg) override {
    if (!bus_up_ || phase_ != Phase::kIdle) {
      return false;
    }
    started_us_ = now_us_;
    MotionSetup::setJogVelocity(stepper_);
    move_start_position_ = stepper_.controller.readActualPosition();
    commandTo(conversions::degreesToMicrosteps(static_cast<float>(target_deg)));
    phase_ = Phase::kMove;
    return true;
  }

  void abort() override {
    // The datasheet's own stop, and it is bounded internally. Safe to call when
    // nothing is moving, which is what lets STOP, DISARM, HV OFF and every
    // fault path share one call -- including fault paths that fire before the
    // bus exists, which is why the guard is here and not at the call sites.
    if (bus_up_) {
      MotionSetup::stopAndHold(stepper_);
    }
    phase_ = Phase::kIdle;
    completion_pending_ = false;
  }

  // Cache, refreshed once per `poll()`. Deliberately NOT an SPI read: these are
  // called several times per reply and before the bus exists at all.
  bool limitAsserted() const override { return limit_asserted_; }

  double angleDeg() const override { return angle_deg_; }
  double xactualDeg() const override { return xactual_deg_; }

  void resyncFromEncoder() override {
    // Called on leaving a compliant state (the protocol layer owns that rule)
    // and, since 2026-10-02, after every driver enable once the rotor has
    // settled. Writing both registers, because a target left where the ramp
    // generator used to think it was would start the next move with a jump.
    if (!bus_up_) {
      return;
    }
    const int32_t encoder = stepper_.encoder.readActualPosition();
    stepper_.controller.writeActualPosition(encoder);
    stepper_.controller.writeTargetPosition(encoder);
    seedFollowWindow();  // XACTUAL moved under the check
  }

  // The whole "rail up, bus closed, driver attached and configured" sequence --
  // gate `b` of `examples/MotionAndHoming`, which is where this is copied from
  // rather than reinvented. `powerUp()` picks the staircase automatically, which
  // matters because every board in hand has `R203` fitted and a single `HV_EN`
  // assertion cannot charge the 208 uF bulk.
  //
  // Idempotent: a second `hv on` finds the rail already live and re-runs a
  // sequence that is safe to re-run, rather than refusing.
  bool setHvEnabled(bool on) override {
    if (!on) {
      if (bus_up_) {
        MotionSetup::disableDriver(stepper_);
        MotionSetup::endSpi(stepper_);
      }
      PowerSequence::powerDown();
      bus_up_ = false;
      driver_enabled_ = false;
      phase_ = Phase::kIdle;
      return true;
    }

    checkpoint(1);
    PowerUpReport report;
    const PowerUpResult result = PowerSequence::powerUp(report);
    checkpoint(2);
    if (result != PowerUpResult::kOk &&
        result != PowerUpResult::kRailAlreadyLive) {
      return false;
    }
    if (!PowerSequence::connectBus()) {
      return false;
    }
    checkpoint(3);
    delay(10);

    MotionSetup::beginSpi(stepper_);
    delay(10);
    checkpoint(4);

    // 0x11 is the TMC5130A signature. A wrong version here means the bus, not
    // the protocol, and stage 2 is the tool for that.
    if (stepper_.readVersion() != 0x11 || !stepper_.communicating()) {
      return false;
    }
    checkpoint(5);
    // SD_MODE is tied to GND on this board; if it reads set, SPI motion
    // commands do nothing at all and every later ack would be a lie.
    if (stepper_.controller.stepAndDirectionMode()) {
      return false;
    }
    checkpoint(6);

    MotionSetup::configure(stepper_, motion_);
    bus_up_ = true;
    checkpoint(7);
    seedFollowWindow();

    // Seed the cache immediately, so the first STATUS after `hv on` reports
    // real numbers rather than the zeros it was holding while dark.
    const MotionSnapshot s = MotionSetup::snapshot(stepper_);
    angle_deg_ = conversions::microstepsToDegrees(s.encoder);
    xactual_deg_ = conversions::microstepsToDegrees(s.position);
    checkpoint(8);
    return true;
  }

  bool enableDriver() override {
    // The current decision and the shunt value are one decision.
    // `applyHighSensitivity` sets CHOPCONF.vsense and reads it back;
    // `enableDriver` refuses unless the read-back says 1, because vsense resets
    // to 0 and against this board's 0.15 ohm shunts that drives roughly 1.8x
    // the motor's rating.
    if (!bus_up_) {
      return false;
    }
    const CurrentCheck check = MotionSetup::applyHighSensitivity(
        stepper_, motion_.run_current_scale, motion_.hold_current_scale);
    driver_enabled_ = MotionSetup::enableDriver(stepper_, check);
    if (driver_enabled_) {
      settleAfterEnable();
    }
    return driver_enabled_;
  }

  // What the last enable did to the rotor: how long X_ENC took to stand
  // still, how far it moved meanwhile, and how many enables there have been
  // (the application records them). Bring-up figures for the IRUN decision.
  uint32_t enableSettleUs() const { return enable_settle_us_; }
  int32_t enableSnapMicrosteps() const { return enable_snap_microsteps_; }
  uint32_t enableCount() const { return enable_count_; }

  void releaseDriver() override {
    // Stop first, then release, so the joystick is not handed to the animal
    // mid-ramp. On a board with the electronic brake this ordering is not
    // sufficient and this call must not exist separately -- the brake has to
    // engage BEFORE drive is released. See `FREEWHEEL` in
    // `../mouse-joystick-pcb/docs/uart-protocol.md`.
    if (bus_up_) {
      MotionSetup::stopAndHold(stepper_);
      MotionSetup::disableDriver(stepper_);
    }
    driver_enabled_ = false;
    phase_ = Phase::kIdle;
  }

  uint32_t expectedHomeDurationUs() const override {
    return cfg_.home_timeout_us;
  }
  uint32_t homeTimeoutMaxUs() const override { return cfg_.home_timeout_us; }

private:
  enum class Phase : uint8_t {
    kIdle,
    kMove,
    kHomeBackoff,
    kHomeApproach,
    kHomeClearance,
  };

  int32_t toward() const {
    return motion_.switch_end == HomeEnd::kPositive ? 1 : -1;
  }

  void commandTo(int32_t position) {
    stepper_.controller.writeTargetPosition(position);
    stepper_.controller.writeRampMode(tmc51x0::PositionMode);
  }

  // The follow check, matching `MotionSetup::moveTo()` exactly: evaluate only
  // once the ramp generator has commanded `kFollowWindowMicrosteps` of travel,
  // so acceleration from standstill and the encoder's 12.8 microstep quantum
  // cannot trip it, then compare measured against commanded.
  bool following(const MotionSnapshot &s) {
    const int32_t commanded = s.position - window_position_;
    const int32_t magnitude = (commanded < 0) ? -commanded : commanded;
    // Grace after a driver enable (2026-10-02, board 4): the rotor's snap to
    // the electrical angle at energisation is not a stall. The first degree
    // of commanded travel after the enable is not judged, and the window is
    // re-seeded where it ends so the snap never enters a measurement. Armed
    // per enable, not per move: a walk in small steps toward a stop must stay
    // checked after the first degree (hardware, 2026-09-03).
    if (follow_grace_) {
      if (magnitude < conversions::kFollowGraceMicrosteps) {
        return true;
      }
      follow_grace_ = false;
      window_position_ = s.position;
      window_encoder_ = s.encoder;
      follow_failures_ = 0;
      return true;
    }
    if (magnitude < conversions::kFollowWindowMicrosteps) {
      return true;  // not enough commanded travel to judge yet
    }
    const int32_t measured = s.encoder - window_encoder_;
    float ratio =
        static_cast<float>(measured) / static_cast<float>(commanded);
    // Absolute value: an encoder wired for the opposite sense reads about -1.0,
    // which is an ENC_CONST sign to flip and not a stall.
    if (ratio < 0.0f) {
      ratio = -ratio;
    }
    follow_ratio_ = ratio;
    window_position_ = s.position;
    window_encoder_ = s.encoder;
    if (ratio >= conversions::kFollowFractionMin) {
      follow_failures_ = 0;
      return true;
    }
    // One window below the fraction is a warning; the fault needs the next
    // one too (2026-10-02). A real stop fails both.
    ++follow_failures_;
    return follow_failures_ < conversions::kFollowFailuresToFault;
  }

  // After the coils come on the rotor may snap to the driver's electrical
  // angle (board 4, 2026-10-02: 1.0 deg in 17 ms, settled within ~45 ms).
  // Wait, bounded, for X_ENC to stand still, then put XACTUAL and XTARGET
  // where the rotor actually is and arm the follow grace, so that no home or
  // move starts from a position the ramp generator only believes.
  void settleAfterEnable() {
    const int32_t before = stepper_.encoder.readActualPosition();
    int32_t last = before;
    uint32_t stable_us = 0;
    uint32_t waited_us = 0;
    while (waited_us < conversions::kEnableSettleMaxUs) {
      delay(conversions::kEnableSettlePollUs / 1000u);
      waited_us += conversions::kEnableSettlePollUs;
      const int32_t enc = stepper_.encoder.readActualPosition();
      const int32_t d = enc - last;
      if (d <= conversions::kEnableSettleToleranceMicrosteps &&
          d >= -conversions::kEnableSettleToleranceMicrosteps) {
        stable_us += conversions::kEnableSettlePollUs;
        if (stable_us >= conversions::kEnableSettleStableUs) {
          break;
        }
      } else {
        stable_us = 0;
        last = enc;
      }
    }
    enable_snap_microsteps_ = stepper_.encoder.readActualPosition() - before;
    enable_settle_us_ = waited_us;
    ++enable_count_;
    resyncFromEncoder();
    follow_grace_ = true;
    follow_failures_ = 0;
  }

  // Seed the follow-check window.
  //
  // Called ONLY where `XACTUAL` is written out from under the check -- the
  // homing datum, a compliance resync, and bus bring-up -- and deliberately NOT
  // per move.
  //
  // Found on hardware 2026-09-03: re-seeding at every `startMoveTo()` meant the
  // window restarted before it could ever reach its 200 microstep evaluation
  // threshold, so ANY move smaller than 1.4 deg was never follow-checked at all.
  // A walk toward the mechanical stop in 0.25 deg steps (36 microsteps each)
  // pushed the motor against it repeatedly with no detection, while the same
  // walk in 2 deg steps faulted correctly. Small commanded moves are exactly
  // what a trial does, so this was a stall detector that switched itself off
  // for the normal case.
  //
  // Letting the window persist across moves is what makes accumulated commanded
  // travel the thing being checked. Between moves the encoder catches up, the
  // ratio comes out near 1, and the window resets itself; against a stop the
  // commanded side keeps growing while the measured side does not.
  void seedFollowWindow() {
    window_position_ = stepper_.controller.readActualPosition();
    window_encoder_ = stepper_.encoder.readActualPosition();
  }

  void beginApproach() {
    // Armed, latching the closing edge. The chip does the stopping: the
    // TMC5130A's left stop acts on negative motion and its right stop on
    // positive, so this blocks only the direction that drives INTO the switch
    // and never the move that gets off it.
    MotionSetup::armStop(stepper_, /*armed=*/true, motion_.switch_active_high,
                         motion_.switch_end, LatchEdge::kActive);
    MotionSetup::setHomeApproachVelocity(stepper_);
    approach_start_position_ = stepper_.controller.readActualPosition();
    commandTo(approach_start_position_ +
              toward() * cfg_.home_travel_bound_microsteps);
    phase_ = Phase::kHomeApproach;
  }

  void fail(uint8_t bit, const char *detail) {
    MotionSetup::stopAndHold(stepper_);
    phase_ = Phase::kIdle;
    completion_pending_ = false;
    fault_pending_ = true;
    fault_bit_ = bit;
    fault_detail_ = detail;
  }

  bool timedOut(uint32_t budget_us) const {
    return (now_us_ - started_us_) > budget_us;
  }

  void pollMove(const MotionSnapshot &s, const tmc51x0::Registers::RampStat &ramp) {
    // An end stop during a commanded move is not a limit doing its job -- the
    // move was range-checked against the travel limits before it started, so
    // reaching a switch means the limits and the mechanism disagree.
    if (stoppedInDirectionOfTravel(ramp, s.position >= move_start_position_)) {
      fail(fault::kLimitUnexpected, "limit_unexpected");
      return;
    }
    if (ramp.position_reached()) {
      finish(/*was_home=*/false);
      return;
    }
    // The mechanism stopped following the ramp generator: an end stop, a stall,
    // a slipping coupling or a dead encoder. SPEC: the draft has no fault bit
    // for a stall, so this borrows `over_travel` -- the commanded position went
    // somewhere the mechanism could not. Raised as an open question in
    // `../mouse-joystick-pcb/docs/uart-protocol.md`.
    if (!following(s)) {
      fail(fault::kOverTravel, "not_following");
      return;
    }
    if (timedOut(cfg_.move_timeout_us)) {
      fail(fault::kOverTravel, "move_timeout");
    }
  }

  void pollHomeBackoff(const MotionSnapshot &s,
                       const tmc51x0::Registers::RampStat &ramp) {
    if (!ramp.position_reached() && !following(s)) {
      fail(fault::kHomeTimeout, "mechanical_stop_first");
      return;
    }
    if (ramp.position_reached()) {
      if (MotionSetup::switchActive(stepper_)) {
        // Backed off a whole jog step and the switch still reads active. That
        // is not a mechanism at the end of travel, it is a stuck switch or an
        // open `U1301` output, and homing into it would drive the mechanism
        // with no prospect of an edge.
        fail(fault::kHomeTimeout, "switch_stuck_active");
        return;
      }
      beginApproach();
      return;
    }
    if (timedOut(cfg_.home_timeout_us)) {
      fail(fault::kHomeTimeout, "home_timeout");
    }
  }

  void pollHomeApproach(const MotionSnapshot &s,
                        const tmc51x0::Registers::RampStat &ramp) {
    // The switch stop is detected on `status_stop_*`, NOT on the latch flag.
    //
    // Measured 2026-09-03: an approach that started ~25 deg from the switch
    // reached it -- `status` showed `limit=1` -- and then sat until the 8 s
    // timeout, faulting `home_timeout` with the joystick resting on the switch.
    // Four earlier homes, all starting close to the switch, had succeeded.
    //
    // The cause is that `status_latch_*` are READ-AND-CLEAR, so whether a poll
    // catches them depends on its timing against the edge. `status_stop_*` are
    // plain status bits that survive being polled. The validated
    // `MotionSetup::homeToSwitch()` already knows this: it keys off
    // `MoveOutcome::kSwitchStopped` and treats the latch as a preference, with
    // an explicit fallback to "read XLATCH now if the poll that ended the move
    // never saw the flag". This had no fallback, so a missed flag meant a hang.
    //
    // XLATCH itself is written by the chip at the edge and is not cleared by
    // reading, so reading it a few polls later is the same value.
    const bool stopped = stoppedInDirectionOfTravel(ramp, toward() > 0);
    if (stopped || ramp.status_latch_l() || ramp.status_latch_r()) {
      // XLATCH has to be read on the same poll that saw the flag, because
      // snapshot() consumed it. The chip wrote XLATCH at the switch edge, so
      // the poll interval delays the reporting and not the measurement.
      const int32_t latch = static_cast<int32_t>(
          stepper_.registers.read(tmc51x0::Registers::XlatchAddress));
      const int32_t here = stepper_.controller.readActualPosition();

      // Re-base XACTUAL so the switch edge is zero. This is NOT the final
      // datum -- see `pollHomeClearance()`, which zeroes both frames at the
      // parked position -- but it is what makes the clearance move a fixed
      // distance from the switch EDGE rather than from wherever the chip
      // happened to stop. The stop position varies with approach velocity (89
      // microsteps at 0.200 rev/s against 4.5 at the creep, measured
      // 2026-09-02); the latch does not.
      //
      const int32_t overshoot = here - latch;
      stepper_.controller.writeActualPosition(overshoot);
      stepper_.controller.writeTargetPosition(overshoot);
      seedFollowWindow();  // XACTUAL moved under the check

      // Clearance, so a later move is not blocked by a stop that is still
      // active and the next approach starts from outside the switch.
      MotionSetup::setJogVelocity(stepper_);
      commandTo(-toward() * cfg_.home_clearance_microsteps);
      phase_ = Phase::kHomeClearance;
      return;
    }
    if (ramp.position_reached()) {
      // Travelled the whole bound without an edge.
      fail(fault::kHomeTimeout, "switch_never_asserted");
      return;
    }
    // Checked AFTER the latch and the bound, matching `moveTo()`'s order: a
    // switch stop is a success and its deceleration must not be read as a
    // stall. This is the protection that stops the motor pushing against a
    // mechanical end for the full timeout when the switch never asserts --
    // `HomeOutcome::kMechanicalStopFirst` in the blocking implementation.
    if (!following(s)) {
      fail(fault::kHomeTimeout, "mechanical_stop_first");
      return;
    }
    if (timedOut(cfg_.home_timeout_us)) {
      fail(fault::kHomeTimeout, "home_timeout");
    }
  }

  void pollHomeClearance(const MotionSnapshot &s,
                         const tmc51x0::Registers::RampStat &ramp) {
    if (!ramp.position_reached() && !following(s)) {
      fail(fault::kHomeTimeout, "mechanical_stop_first");
      return;
    }
    if (ramp.position_reached()) {
      // THE DATUM, and both frames get it.
      //
      // Zeroing both at the PARKED position rather than at the switch edge is
      // what removes the offset that two earlier attempts did not. Measured
      // 2026-09-03: re-basing XACTUAL to the latch while merely zeroing the
      // encoder left them differing by ~166 microsteps (1.2 deg) forever, and a
      // host commanding -16.0 read back -14.766 on a ~10 deg usable range.
      // Writing both here makes them agree by construction, and the 0.991
      // follow ratio then keeps them within about one encoder count across the
      // whole range.
      //
      // It also makes zero MEAN something. The park is where homing leaves the
      // joystick, it is now a commandable position rather than one the travel
      // limits exclude, and `ev;HOMED` reports `angle=0.0` -- which is what the
      // shared corpus already expects, so the firmware and the corpus stop
      // disagreeing about it.
      //
      // Actual before target, matching `homeToSwitch()`. There is a
      // microseconds-wide window where the target still names the old frame; at
      // this velocity, from standstill, the ramp cannot move meaningfully in it.
      stepper_.controller.writeActualPosition(0);
      stepper_.controller.writeTargetPosition(0);
      stepper_.encoder.writeActualPosition(0);
      seedFollowWindow();  // both frames rewritten

      // Refresh the cache before reporting completion. The snapshot at the top
      // of this poll predates the writes above, so without this the `HOMED`
      // event carries the position from BEFORE the datum -- observed on
      // hardware as `ev;HOMED;...;angle=8.459` immediately followed by a
      // `status` reading `angle=0.0`. The event is the thing a host timestamps
      // and records, so it is the one that must not be stale.
      angle_deg_ = 0.0;
      xactual_deg_ = 0.0;

      finish(/*was_home=*/true);
      return;
    }
    if (timedOut(cfg_.home_timeout_us)) {
      fail(fault::kHomeTimeout, "home_timeout");
    }
  }

  // Only the switch guarding the direction of travel, mirroring the hardware:
  // checking both bits would block the one move that must always be allowed,
  // the one that gets off the switch.
  static bool stoppedInDirectionOfTravel(
      const tmc51x0::Registers::RampStat &ramp, bool moving_positive) {
    return moving_positive ? ramp.status_stop_r() : ramp.status_stop_l();
  }

  void finish(bool was_home) {
    phase_ = Phase::kIdle;
    completion_pending_ = true;
    completion_was_home_ = was_home;
  }

  TMC51X0 &stepper_;
  MotionConfig motion_;
  Config cfg_;

  uint64_t now_us_ = 0;
  uint64_t started_us_ = 0;
  Phase phase_ = Phase::kIdle;

  int32_t move_start_position_ = 0;
  int32_t window_position_ = 0;
  int32_t window_encoder_ = 0;
  float follow_ratio_ = 0.0f;
  bool follow_grace_ = false;     // first degree after an enable not judged
  uint8_t follow_failures_ = 0;   // consecutive windows below the fraction
  uint32_t enable_settle_us_ = 0; // the last enable's settle wait
  int32_t enable_snap_microsteps_ = 0; // X_ENC change during that wait
  uint32_t enable_count_ = 0;
  int32_t approach_start_position_ = 0;

  bool completion_pending_ = false;
  bool completion_was_home_ = false;

  // The invariant this file now turns on: no SPI unless the bus switch is
  // closed and `beginSpi()` has run.
  bool bus_up_ = false;
  bool driver_enabled_ = false;
  bool limit_asserted_ = false;

  double angle_deg_ = 0.0;
  double xactual_deg_ = 0.0;

  bool fault_pending_ = false;
  uint8_t fault_bit_ = 0;
  // Poll divider: the full snapshot every tenth poll, the encoder alone in
  // between (2026-10-02). At the 1 kHz tick: completion and faults within
  // 10 ms, the encoder every 1 ms.
  static constexpr uint32_t kFullSnapshotEvery = 10u;
  uint32_t poll_count_ = 0u;
  const char *fault_detail_ = "";
};

}  // namespace mouse_joystick
