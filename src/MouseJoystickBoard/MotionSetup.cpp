#include <SPI.h>

#include "BoardPins.hpp"
#include "Conversions.hpp"
#include "MotionSetup.hpp"

namespace mouse_joystick {

namespace conv = conversions;
using Regs = tmc51x0::Registers;

bool MotionSetup::driver_enabled_ = false;

namespace {

// How often a bounded move looks at the chip. At the jog velocity this is
// about 26 microsteps per poll, so the follow window of 200 microsteps spans
// eight polls and a stall is caught in well under a tenth of a second.
constexpr uint32_t kPollMs = 5;

// The configured ramp velocity, in chip units. Kept here because `stopAndHold`
// has to zero VMAX and then put it back, and TMC51X0's own
// begin/endRampToZeroVelocity pair restores the ramp *mode* along with it --
// which would flip back to positioning with the old target still set and
// restart the motion the stop was meant to end.
uint32_t configured_max_velocity_chip = 0;

} // namespace

void MotionSetup::beginSpi(TMC51X0 &stepper) {
  // SPI0's pin assignment is forced by RP2040 muxing, and BoardPins is the only
  // place these numbers are allowed to appear.
  SPI.setSCK(pins::kTmcSpiSck);
  SPI.setTX(pins::kTmcSpiCopi);
  SPI.setRX(pins::kTmcSpiCipo);
  SPI.begin();

  const auto spi_parameters = tmc51x0::SpiParameters{}
                                  .withSpi(&SPI)
                                  .withClockRate(conv::kTmcSpiClockHz)
                                  .withChipSelectPin(pins::kTmcCsN);

  // Declaring the model seeds the register mirror with TMC5130A reset defaults
  // rather than inferring them from the first identity read.
  stepper.setupSpi(spi_parameters, Regs::DeviceModel::TMC5130A);

  // Registering the pin drives it HIGH -- driver disabled -- before anything
  // else happens, which is the same level SafeState already established.
  stepper.driver.setEnableHardwarePin(pins::kTmcDrvEnN);
  driver_enabled_ = false;
}

void MotionSetup::endSpi(TMC51X0 &stepper) {
  disableDriver(stepper);
  SPI.end();
  digitalWrite(pins::kTmcCsN, HIGH);
}

void MotionSetup::configure(TMC51X0 &stepper, const MotionConfig &config) {
  const uint8_t run_current_scale = config.run_current_scale;
  const uint8_t hold_current_scale = config.hold_current_scale;
  // One microstep per real position unit and one second per real velocity unit,
  // so "real" units are microsteps and microsteps/s. That keeps the library's
  // integer conversions exact and leaves this firmware only the revolutions and
  // degrees arithmetic, which lives in Conversions.hpp and is host-tested.
  const auto converter_parameters =
      tmc51x0::ConverterParameters{}
          .withClockFrequencyMHz(conv::kTmcClockMhz)
          .withMicrostepsPerRealPositionUnit(1)
          .withSecondsPerRealVelocityUnit(1);
  stepper.converter.setup(converter_parameters);

  // Chip units, not percent. TMC51X0's percent path maps 0..100 onto CS 0..31,
  // which cannot express a specific current scale exactly -- and the current
  // scale is the number the motor's rating is checked against.
  const auto driver_parameters =
      tmc51x0::DriverParameters{}
          .withRunCurrent(run_current_scale)
          .withHoldCurrent(hold_current_scale)
          .withHoldDelay(6)
          .withChopperMode(tmc51x0::SpreadCycleMode)
          .withStealthChopEnabled(false)
          .withCoolStepEnabled(false)
          .withHighVelocityFullstepEnabled(false)
          .withStandstillMode(tmc51x0::NormalMode)
          // The board's 0.15 ohm shunts need the low sense-resistor voltage,
          // or every current is 1.8x what was asked for. TMC51X0 4.1.0 carries
          // this through driver.setup() and replays it after a recovery;
          // before 4.1.0 it could not express the bit at all.
          .withSenseVoltageMode(tmc51x0::LowSenseVoltageMode)
          .withMotorDirection(config.motor_reversed
                                  ? tmc51x0::ReverseDirection
                                  : tmc51x0::ForwardDirection);
  stepper.driver.setup(driver_parameters);

  const auto controller_parameters_real =
      tmc51x0::ControllerParameters{}
          .withRampMode(tmc51x0::HoldMode)
          .withStopMode(tmc51x0::HardMode)
          .withMaxVelocity(
              conv::revsPerSecondToMicrostepsPerSecond(conv::kJogRevsPerSecond))
          .withMaxAcceleration(
              conv::revsPerSecondToMicrostepsPerSecond(conv::kRevsPerSecondSq))
          // DMAX defaults to zero, and with V1 = 0 the ramp uses AMAX and DMAX
          // only -- so a zero here is a ramp that accelerates and then has no
          // configured way to slow down. Match it to AMAX.
          .withMaxDeceleration(
              conv::revsPerSecondToMicrostepsPerSecond(conv::kRevsPerSecondSq))
          .withStartVelocity(0)
          // VSTOP and D1 must never be zero in positioning mode, even with
          // V1 = 0. Ask for values that survive the conversion to chip units
          // rather than for the library's raw defaults: D1 matches DMAX, and
          // VSTOP is small but above the 0.95 microsteps/s chip quantum.
          .withStopVelocity(10)
          .withFirstDeceleration(
              conv::revsPerSecondToMicrostepsPerSecond(conv::kRevsPerSecondSq));

  tmc51x0::ControllerParameters controller_parameters_chip =
      stepper.converter.controllerParametersRealToChip(
          controller_parameters_real);

  // And then check, rather than trusting the arithmetic. See the note on
  // conversions::nonZeroRampParameter: a zero here is a ramp that crawls at
  // VSTOP instead of running, with nothing reported anywhere.
  controller_parameters_chip.first_deceleration =
      conv::nonZeroRampParameter(controller_parameters_chip.first_deceleration,
                                 controller_parameters_chip.max_deceleration);
  controller_parameters_chip.stop_velocity =
      conv::nonZeroRampParameter(controller_parameters_chip.stop_velocity, 10u);

  stepper.controller.setup(controller_parameters_chip);
  configured_max_velocity_chip = controller_parameters_chip.max_velocity;

  // Stop the ramp generator, properly, before anything else.
  //
  // This is not belt-and-braces. TMC51X0's `Controller::initialize()` runs
  // during `setupSpi()` and writes its default ControllerParameters, whose ramp
  // mode is VelocityPositiveMode with VMAX = 10 -- so from the moment the bus
  // is closed the chip is commanded to turn at about 9.5 microsteps/s.
  // RAMPMODE = hold does NOT cancel that: hold means "velocity remains
  // unchanged", so it freezes the creep instead of ending it. Measured on
  // board E4654C159F4D1334 on 2026-09-02: VACTUAL sat at 10 and XACTUAL had
  // walked to 125 microsteps with the driver still disabled.
  //
  // Harmless while ~{TMC_DRV_EN} is high, and uncommanded motion the instant it
  // goes low. So the ramp is brought to a real standstill here, and again in
  // enableDriver().
  stopAndHold(stepper);
  stepper.controller.zeroActualPosition();
  stepper.controller.zeroTargetPosition();

  // Report the reference switch, but let it stop nothing yet. Polarity is a
  // property of the fitted switch's wiring, so stage 3 measures it before it
  // arms the stop -- an inverted guess reads an untriggered switch as a
  // permanent stop, which presents as "the motor will not move".
  //
  // The swap_lr mapping is set here even though the stop is disarmed, so that
  // the status bit the switch reports on matches its physical end for every
  // move rather than only for an armed homing approach.
  armStop(stepper, /*armed=*/false, config.switch_active_high,
          config.switch_end, LatchEdge::kActive);

  // ENC_CONST resets to zero, so X_ENC does not move until this runs. Scaled
  // to 12.8 microsteps per quadrature count, X_ENC reads in microsteps and can
  // be compared with XACTUAL directly. Decimal mode because 12.8 is exact
  // there and recurring in binary.
  setEncoderInverted(stepper, config.encoder_inverted);
}

CurrentCheck MotionSetup::applyHighSensitivity(TMC51X0 &stepper,
                                               uint8_t run_current_scale,
                                               uint8_t hold_current_scale) {
  // Ask the library for it, then read CHOPCONF back and believe the read.
  //
  // The write is now TMC51X0's job -- `configure()` sets
  // `sense_voltage_mode`, and 4.1.0 carries it through `driver.setup()`,
  // through homing, and through a recovery replay. What stays this
  // firmware's job is the *verification*: IHOLD_IRUN is write-only, so
  // CHOPCONF is the only readable half of the current setting, and the gate
  // below is what stands between a wrong bit and 1.8x the intended current.
  //
  // Note the write is a no-op unless the library knows the part is a
  // TMC5130A, since bit 17 is reserved on the TMC5160 -- `beginSpi()` passes
  // the model to `setupSpi()`, so it does.
  stepper.driver.writeSenseVoltageMode(tmc51x0::LowSenseVoltageMode);

  CurrentCheck check;
  check.chopconf = stepper.registers.read(Regs::ChopconfAddress);
  Regs::Chopconf chopconf;
  chopconf.raw = check.chopconf;
  check.vsense = chopconf.vsense();
  check.toff_nonzero = (chopconf.toff() != 0);
  check.run_current_scale = run_current_scale;
  check.hold_current_scale = hold_current_scale;
  check.run_amps_rms =
      conv::currentScaleToAmpsRms(run_current_scale, check.vsense);
  check.hold_amps_rms =
      conv::currentScaleToAmpsRms(hold_current_scale, check.vsense);
  check.within_ceiling = (run_current_scale <= conv::kCurrentScaleCeiling);
  check.safe = check.vsense && check.within_ceiling;
  return check;
}

bool MotionSetup::enableDriver(TMC51X0 &stepper, const CurrentCheck &check) {
  if (!check.safe) {
    stepper.driver.disable();
    driver_enabled_ = false;
    return false;
  }
  // Enabling the output stage must never be the thing that starts a move. The
  // ramp generator can be carrying a velocity command from before the driver
  // was enabled -- TMC51X0's setup leaves one -- so bring it to a real
  // standstill first. Cheap, and it makes "enabled" and "moving" independent.
  stopAndHold(stepper);
  stepper.driver.enable();
  driver_enabled_ = true;
  return true;
}

void MotionSetup::disableDriver(TMC51X0 &stepper) {
  stepper.driver.disable();
  driver_enabled_ = false;
}

MotionSnapshot MotionSetup::snapshot(TMC51X0 &stepper) {
  MotionSnapshot s;
  // Exactly one RAMP_STAT read. See the note in MotionSetup.hpp: the library's
  // per-flag accessors each read it again, and several of those flags are
  // read-and-clear.
  //
  // One pipelined transaction (TMC51X0 `readMany`, 2026-10-06): six datagrams
  // instead of ten, 270 us on the bus instead of 450 at 4 MHz. RAMP_STAT goes
  // first because the last address is sent twice and it is read-and-clear;
  // X_ENC last is harmless to read again.
  const Regs::RegisterAddress addresses[5] = {
      Regs::RampStatAddress, Regs::DrvStatusAddress, Regs::XactualAddress,
      Regs::VactualAddress, Regs::XencAddress};
  uint32_t values[5] = {0, 0, 0, 0, 0};
  stepper.registers.readMany(addresses, values, 5);
  s.ramp_stat = values[0];
  s.drv_status = values[1];
  s.position = static_cast<int32_t>(values[2]);
  // VACTUAL is 24-bit two's complement in a 32-bit register, as
  // Controller::readActualVelocity() converts it.
  int32_t velocity = static_cast<int32_t>(values[3]);
  if (velocity > 8388607) {
    velocity -= 16777216;
  }
  s.velocity = velocity;
  s.encoder = static_cast<int32_t>(values[4]);
  return s;
}

bool MotionSetup::stopAndHold(TMC51X0 &stepper, uint32_t timeout_ms) {
  // The datasheet's own stop: a velocity-mode target of zero, decelerating at
  // AMAX. Velocity mode also means VSTART/VSTOP play no part, so there is no
  // short-distance edge case to reason about.
  stepper.controller.writeMaxVelocity(0);
  stepper.controller.writeRampMode(tmc51x0::VelocityPositiveMode);

  const uint32_t deadline = millis() + timeout_ms;
  bool stopped = false;
  while (millis() < deadline) {
    // VACTUAL, not RAMP_STAT.vzero: reading RAMP_STAT clears event flags that
    // the caller may still want to report.
    if (stepper.controller.readActualVelocity() == 0) {
      stopped = true;
      break;
    }
    delay(1);
  }

  // Hold before restoring the velocity, so restoring cannot restart a move
  // toward a target that is still set. Hold mode is also what the datasheet
  // names as the way to clear a reference-switch stop condition.
  stepper.controller.writeRampMode(tmc51x0::HoldMode);
  stepper.controller.writeMaxVelocity(configured_max_velocity_chip);
  return stopped;
}

MoveReport MotionSetup::moveTo(TMC51X0 &stepper, int32_t target_position,
                               uint32_t timeout_ms) {
  MoveReport report;
  report.start_position = stepper.controller.readActualPosition();
  report.start_encoder = stepper.encoder.readActualPosition();
  report.end_position = report.start_position;
  report.end_encoder = report.start_encoder;
  report.elapsed_ms = 0;
  report.follow_ratio = 0.0f;
  report.ramp_stat_at_end = 0;
  report.drv_status_at_end = 0;
  report.latch_seen = false;
  report.latch_position = 0;
  report.latch_encoder = 0;
  report.outcome = MoveOutcome::kTargetReached;

  if (target_position == report.start_position) {
    return report;
  }

  const bool moving_positive = (target_position > report.start_position);
  int32_t window_position = report.start_position;
  int32_t window_encoder = report.start_encoder;

  stepper.controller.writeMaxVelocity(configured_max_velocity_chip);
  stepper.controller.writeTargetPosition(target_position);
  stepper.controller.writeRampMode(tmc51x0::PositionMode);

  const uint32_t started = millis();
  MoveOutcome outcome = MoveOutcome::kTimedOut;

  while ((millis() - started) < timeout_ms) {
    const MotionSnapshot s = snapshot(stepper);
    report.ramp_stat_at_end = s.ramp_stat;
    report.drv_status_at_end = s.drv_status;
    report.end_position = s.position;
    report.end_encoder = s.encoder;

    Regs::DrvStatus drv;
    drv.raw = s.drv_status;
    if (drv.ot() || drv.s2ga() || drv.s2gb()) {
      outcome = MoveOutcome::kDriverFault;
      break;
    }

    Regs::RampStat ramp;
    ramp.raw = s.ramp_stat;

    // The latch flags are read-and-clear and snapshot() consumed them, so
    // XLATCH has to be read on the same poll that saw them or the event is
    // lost. XLATCH itself is written by the chip at the switch edge, so the
    // 5 ms poll interval delays the *reporting* and not the measurement.
    if (!report.latch_seen &&
        (ramp.status_latch_l() || ramp.status_latch_r())) {
      report.latch_seen = true;
      report.latch_position =
          static_cast<int32_t>(stepper.registers.read(Regs::XlatchAddress));
      report.latch_encoder =
          static_cast<int32_t>(stepper.registers.read(Regs::EncLatchAddress));
    }

    // Only the switch that guards the DIRECTION OF TRAVEL. This mirrors the
    // hardware exactly -- the TMC5130A's left stop acts on negative motion and
    // its right stop on positive motion -- and it is the difference between a
    // limit switch and a lock. Checking both bits blocks the move that gets you
    // OFF the switch, which is the one move that must always be allowed;
    // measured on the rig 2026-09-02, where a jog away from a resting switch
    // was refused in 3 ms.
    //
    // Which bit the single fitted switch appears on is set by SW_MODE.swap_lr
    // in armStop(), from the end of travel the switch is physically on.
    // status_stop_* are plain status bits rather than read-and-clear, so they
    // survive being polled; event_stop_* is recorded in ramp_stat_at_end.
    //
    // This fires whether or not the hardware stop is armed, so an unarmed jog
    // still ends at the switch rather than pressing on. It is the slower of the
    // two protections -- one poll interval rather than immediate -- and it is
    // not a substitute for arming the stop.
    const bool blocking_switch =
        moving_positive ? (ramp.status_stop_r() || ramp.event_stop_r())
                        : (ramp.status_stop_l() || ramp.event_stop_l());
    if (blocking_switch) {
      outcome = MoveOutcome::kSwitchStopped;
      break;
    }

    if (ramp.position_reached()) {
      outcome = MoveOutcome::kTargetReached;
      break;
    }

    // Follow check. XACTUAL is the ramp generator's idea of where the motor is
    // and advances whether or not the shaft moved, so measuring X_ENC against
    // it is what distinguishes real motion from an end stop or a stall.
    const int32_t commanded = s.position - window_position;
    if (commanded >= conv::kFollowWindowMicrosteps ||
        commanded <= -conv::kFollowWindowMicrosteps) {
      const int32_t measured = s.encoder - window_encoder;
      const float ratio =
          static_cast<float>(measured) / static_cast<float>(commanded);
      report.follow_ratio = ratio;
      // Absolute value: an encoder wired for the opposite sense reads about
      // -1.0, which is an ENC_CONST sign to flip, not a stall.
      const float magnitude = (ratio < 0.0f) ? -ratio : ratio;
      if (magnitude < conv::kFollowFractionMin) {
        outcome = MoveOutcome::kNotFollowing;
        break;
      }
      window_position = s.position;
      window_encoder = s.encoder;
    }

    delay(kPollMs);
  }

  report.outcome = outcome;
  stopAndHold(stepper);

  report.end_position = stepper.controller.readActualPosition();
  report.end_encoder = stepper.encoder.readActualPosition();
  report.elapsed_ms = millis() - started;

  const int32_t commanded_total = report.end_position - report.start_position;
  const int32_t measured_total = report.end_encoder - report.start_encoder;
  if (commanded_total != 0) {
    report.follow_ratio = static_cast<float>(measured_total) /
                          static_cast<float>(commanded_total);
  }
  return report;
}

MoveReport MotionSetup::moveBy(TMC51X0 &stepper, int32_t delta_microsteps,
                               uint32_t timeout_ms) {
  const int32_t here = stepper.controller.readActualPosition();
  return moveTo(stepper, here + delta_microsteps, timeout_ms);
}

void MotionSetup::setVelocityRevsPerSecond(TMC51X0 &stepper,
                                           float revs_per_second) {
  const int32_t chip = stepper.converter.velocityRealToChip(
      conv::revsPerSecondToMicrostepsPerSecond(revs_per_second));
  // A velocity that rounds to zero is a move that never starts. One chip unit
  // is 0.95 microsteps/s at 16 MHz so the creep is nowhere near it, but the
  // same rounding that zeroed D1 lives on this path too.
  configured_max_velocity_chip =
      conv::nonZeroRampParameter(static_cast<uint32_t>(chip), 1u);
  stepper.controller.writeMaxVelocity(configured_max_velocity_chip);
}

void MotionSetup::setJogVelocity(TMC51X0 &stepper) {
  setVelocityRevsPerSecond(stepper, conv::kJogRevsPerSecond);
}

void MotionSetup::setHomeApproachVelocity(TMC51X0 &stepper) {
  setVelocityRevsPerSecond(stepper, conv::kHomeApproachRevsPerSecond);
}

void MotionSetup::setHomeCreepVelocity(TMC51X0 &stepper) {
  setVelocityRevsPerSecond(stepper, conv::kHomeCreepRevsPerSecond);
}

void MotionSetup::setEncoderInverted(TMC51X0 &stepper, bool inverted) {
  // One signed value; TMC51X0 4.1.0 does the fixed-point split. Doing it by
  // hand is how -12.8 becomes -11.2 by accident.
  const int32_t scaled = inverted ? -conv::kEncoderMicrostepsPerPulseScaled
                                  : conv::kEncoderMicrostepsPerPulseScaled;
  const auto encoder_parameters =
      tmc51x0::EncoderParameters{}.withMicrostepsPerPulse(tmc51x0::DecimalMode,
                                                          scaled);
  stepper.encoder.setup(encoder_parameters);
  stepper.encoder.zeroActualPosition();
}

void MotionSetup::armStop(TMC51X0 &stepper, bool armed, bool switch_active_high,
                          HomeEnd end, LatchEdge edge) {
  // pol_stop_* = 0 means a HIGH level on the input stops the motor, so the
  // invert flag is the complement of the level the fitted switch produces when
  // triggered -- which stage 3 measures rather than assumes.
  const bool invert = !switch_active_high;
  const bool positive = (end == HomeEnd::kPositive);

  // swap_lr routes the REFL pin to the right-stop logic, which is the only way
  // a switch on REFL can stop positive motion. Exactly one stop is ever
  // enabled: the other input is REFR, tied to GND on this board.
  //
  // Exactly one latch edge, on both sides, because the TMC5130A overwrites
  // XLATCH on every enabled latch event -- a value that could have come from
  // either edge is not a datum.
  const bool on_active = (edge == LatchEdge::kActive);
  const auto switch_parameters = tmc51x0::SwitchParameters{}
                                     .withSwapLeftRight(positive)
                                     .withLeftStopEnabled(armed && !positive)
                                     .withRightStopEnabled(armed && positive)
                                     .withInvertLeftPolarity(invert)
                                     .withInvertRightPolarity(invert)
                                     .withLatchLeftActive(on_active)
                                     .withLatchRightActive(on_active)
                                     .withLatchLeftInactive(!on_active)
                                     .withLatchRightInactive(!on_active)
                                     .withLatchEncoderEnabled(true);
  stepper.controller.setupSwitches(switch_parameters);
  stepper.controller.writeStopMode(tmc51x0::HardMode);
}

MotionSetup::SwitchWitness MotionSetup::witnessByMotion(
    TMC51X0 &stepper, HomeEnd end, uint32_t move_timeout_ms,
    uint32_t sweep_timeout_ms) {
  SwitchWitness w;
  w.stop_reason = SwitchWitness::StopReason::kBudgetExpired;
  w.transition_seen = false;
  w.released_again = false;
  w.engage_position = 0;
  w.release_position = 0;
  w.travel_used = 0;
  w.approach_outcome = MoveOutcome::kTargetReached;

  Regs::Ioin ioin;
  ioin.raw = stepper.registers.read(Regs::IoinAddress);
  w.resting_level = ioin.refl_step();

  // Triggered is the level it is not showing now. Write that polarity before
  // moving, so the direction-aware guard in moveTo is reading the switch the
  // right way round for the sweep.
  w.active_high = !w.resting_level;
  armStop(stepper, /*armed=*/false, w.active_high, end, LatchEdge::kActive);
  // The sweep only has to LOCATE the switch, not measure it, so it runs at the
  // jog velocity. Precision comes later, from the hardware latch.
  setJogVelocity(stepper);

  const int32_t toward = (end == HomeEnd::kPositive) ? 1 : -1;
  const int32_t step = conv::kWitnessStepMicrosteps;
  const int32_t start = stepper.controller.readActualPosition();
  const uint32_t deadline = millis() + sweep_timeout_ms;

  while (millis() < deadline) {
    const MoveReport r = moveBy(stepper, toward * step, move_timeout_ms);
    w.travel_used = r.end_position - start;
    w.approach_outcome = r.outcome;

    ioin.raw = stepper.registers.read(Regs::IoinAddress);
    if (ioin.refl_step() != w.resting_level) {
      w.transition_seen = true;
      w.stop_reason = SwitchWitness::StopReason::kTransitionSeen;
      w.engage_position = r.end_position;
      break;
    }
    // The mechanism ended the step for a reason other than reaching its
    // target, and the switch has not changed: an end stop before the switch,
    // or a switch that does not report. Either way, stop sweeping.
    if (r.outcome != MoveOutcome::kTargetReached) {
      w.stop_reason = SwitchWitness::StopReason::kMoveBlocked;
      break;
    }
    if ((w.travel_used > conv::kHomeTravelBoundMicrosteps) ||
        (w.travel_used < -conv::kHomeTravelBoundMicrosteps)) {
      w.stop_reason = SwitchWitness::StopReason::kTravelBound;
      break;
    }
  }

  if (!w.transition_seen) {
    setJogVelocity(stepper);
    return w;
  }

  // And back off until it lets go, which confirms the transition was the switch
  // rather than a one-way glitch.
  // The release sweep only has to back off a few steps, but give it the same
  // budget rather than a second borrowed constant -- that reuse is what caused
  // the bug this function was just fixed for.
  const uint32_t release_deadline = millis() + sweep_timeout_ms;
  while (millis() < release_deadline) {
    const MoveReport r = moveBy(stepper, -toward * step, move_timeout_ms);
    ioin.raw = stepper.registers.read(Regs::IoinAddress);
    if (ioin.refl_step() == w.resting_level) {
      w.released_again = true;
      w.release_position = r.end_position;
      break;
    }
    if (r.outcome != MoveOutcome::kTargetReached) {
      break;
    }
  }

  setJogVelocity(stepper);
  return w;
}

HomeReport MotionSetup::homeToSwitch(TMC51X0 &stepper, bool switch_active_high,
                                     HomeEnd end, uint32_t timeout_ms) {
  HomeReport report;
  report.outcome = HomeOutcome::kTimedOut;
  report.coarse_latch = 0;
  report.coarse_encoder = 0;
  report.settled_position = 0;
  report.overshoot = 0;
  report.creep_ran = false;
  report.release_latched = false;
  report.fine_latch = 0;
  report.fine_encoder = 0;
  report.creep_distance = 0;
  report.coarse_to_fine = 0;
  report.backoff_position = 0;
  report.released_after_backoff = false;
  report.elapsed_ms = 0;

  const uint32_t started = millis();

  // The approach runs toward the switch and the creep and clearance run away
  // from it, so every distance takes its sign from which end the switch is on.
  const int32_t toward = (end == HomeEnd::kPositive) ? 1 : -1;

  // Datasheet step 1: make sure the switch is not already pressed. Nothing is
  // armed yet, so this move is protected only by the follow check -- which is
  // the right protection here, because a switch that reads active when the
  // mechanism is nowhere near it would otherwise block every move.
  armStop(stepper, /*armed=*/false, switch_active_high, end,
          LatchEdge::kActive);
  setJogVelocity(stepper);
  if (switchActive(stepper)) {
    const MoveReport away =
        moveBy(stepper, -toward * conv::kHomeBackoffMicrosteps, timeout_ms);
    report.approach = away;
    if (switchActive(stepper)) {
      report.outcome = HomeOutcome::kSwitchStuckActive;
      report.elapsed_ms = millis() - started;
      return report;
    }
  }

  // ---------------------------------------------------------------------
  // Stage 1: fast approach, latching the closing edge, bounded by position.
  // ---------------------------------------------------------------------
  armStop(stepper, /*armed=*/true, switch_active_high, end, LatchEdge::kActive);
  setHomeApproachVelocity(stepper);

  const int32_t start = stepper.controller.readActualPosition();
  report.approach = moveTo(
      stepper, start + toward * conv::kHomeTravelBoundMicrosteps, timeout_ms);

  if (report.approach.outcome != MoveOutcome::kSwitchStopped) {
    armStop(stepper, /*armed=*/false, switch_active_high, end,
            LatchEdge::kActive);
    setJogVelocity(stepper);
    switch (report.approach.outcome) {
    case MoveOutcome::kTargetReached:
      report.outcome = HomeOutcome::kSwitchNeverAsserted;
      break;
    case MoveOutcome::kNotFollowing:
      report.outcome = HomeOutcome::kMechanicalStopFirst;
      break;
    case MoveOutcome::kDriverFault:
      report.outcome = HomeOutcome::kDriverFault;
      break;
    default:
      report.outcome = HomeOutcome::kTimedOut;
      break;
    }
    report.elapsed_ms = millis() - started;
    return report;
  }

  // moveTo() already ended at standstill, so the latch is stable. Prefer the
  // latch the chip took at the edge; fall back to XLATCH now if the poll that
  // ended the move never saw the flag.
  report.coarse_latch =
      report.approach.latch_seen
          ? report.approach.latch_position
          : static_cast<int32_t>(stepper.registers.read(Regs::XlatchAddress));
  report.coarse_encoder =
      report.approach.latch_seen
          ? report.approach.latch_encoder
          : static_cast<int32_t>(stepper.registers.read(Regs::EncLatchAddress));
  report.settled_position = stepper.controller.readActualPosition();
  report.overshoot = report.settled_position - report.coarse_latch;

  // ---------------------------------------------------------------------
  // Stage 2: creep off the switch and take the RELEASE edge as the datum.
  // ---------------------------------------------------------------------
  // Why this is the better datum: the MAX6816 debounces before the driver sees
  // anything, so the closing latch sits past the true edge by (debounce delay x
  // approach velocity). The creep runs 20x slower, so its share of that error
  // is 20x smaller, and it costs only the overshoot plus the switch throw.
  //
  // Re-arming for the inactive edge is what makes it a hardware measurement.
  // The stop stays armed: it does not block motion away from the switch, and
  // the datasheet names a move in the opposite direction as one of the two ways
  // to clear the stop condition -- hold mode, which moveTo() already left, is
  // the other.
  armStop(stepper, /*armed=*/true, switch_active_high, end,
          LatchEdge::kInactive);
  setHomeCreepVelocity(stepper);
  report.creep_ran = true;

  const int32_t creep_start = stepper.controller.readActualPosition();
  const uint32_t creep_deadline = millis() + timeout_ms;
  while (millis() < creep_deadline) {
    const MoveReport step =
        moveBy(stepper, -toward * conv::kHomeCreepStepMicrosteps, timeout_ms);
    report.creep_distance = step.end_position - creep_start;

    if (step.latch_seen) {
      report.release_latched = true;
      report.fine_latch = step.latch_position;
      report.fine_encoder = step.latch_encoder;
      break;
    }
    // The step ended on something other than its target, and the switch has
    // not released: stop creeping rather than pressing on.
    if (step.outcome != MoveOutcome::kTargetReached) {
      break;
    }
    if ((report.creep_distance > conv::kHomeCreepBoundMicrosteps) ||
        (report.creep_distance < -conv::kHomeCreepBoundMicrosteps)) {
      break;
    }
  }

  armStop(stepper, /*armed=*/false, switch_active_high, end,
          LatchEdge::kActive);
  setJogVelocity(stepper);

  if (!report.release_latched) {
    report.outcome = HomeOutcome::kReleaseNeverLatched;
    report.elapsed_ms = millis() - started;
    return report;
  }
  report.coarse_to_fine = report.coarse_latch - report.fine_latch;

  // ---------------------------------------------------------------------
  // Move the reference so the RELEASE edge is position zero.
  // ---------------------------------------------------------------------
  // moveTo left the ramp in hold mode, which is both required before writing
  // XACTUAL -- in positioning mode a write to it starts a motion -- and what
  // the datasheet names as the way to clear a switch stop condition.
  const int32_t settled_after_creep = stepper.controller.readActualPosition();
  const int32_t zeroed = settled_after_creep - report.fine_latch;
  stepper.controller.writeActualPosition(zeroed);
  stepper.controller.writeTargetPosition(zeroed);
  stepper.encoder.zeroActualPosition();

  // Clearance, so a later move cannot be blocked by a stop that is still active
  // and the next approach starts from outside the switch.
  const MoveReport backoff =
      moveBy(stepper, -toward * conv::kHomeBackoffMicrosteps, timeout_ms);
  report.backoff_position = backoff.end_position;
  report.released_after_backoff = !switchActive(stepper);
  report.outcome = HomeOutcome::kHomed;
  report.elapsed_ms = millis() - started;
  return report;
}

SwitchDelay MotionSetup::measureSwitchDelay(TMC51X0 &stepper,
                                            bool switch_active_high,
                                            HomeEnd end, uint32_t timeout_ms) {
  SwitchDelay d;
  d.valid = false;
  d.latch_fast = 0;
  d.latch_slow = 0;
  d.velocity_fast = conv::revsPerSecondToMicrostepsPerSecond(
      conv::kHomeApproachRevsPerSecond);
  d.velocity_slow =
      conv::revsPerSecondToMicrostepsPerSecond(conv::kHomeCreepRevsPerSecond);
  d.delay_ms = 0.0f;

  const int32_t toward = (end == HomeEnd::kPositive) ? 1 : -1;
  const float velocities[2] = {conv::kHomeApproachRevsPerSecond,
                               conv::kHomeCreepRevsPerSecond};
  int32_t latches[2] = {0, 0};

  for (uint8_t i = 0; i < 2; ++i) {
    // Start clear of the switch, without re-zeroing: the two latches have to
    // share one position frame or their difference means nothing.
    armStop(stepper, /*armed=*/false, switch_active_high, end,
            LatchEdge::kActive);
    setJogVelocity(stepper);
    if (switchActive(stepper)) {
      const MoveReport away =
          moveBy(stepper, -toward * conv::kHomeBackoffMicrosteps, timeout_ms);
      (void)away;
      if (switchActive(stepper)) {
        return d;
      }
    }

    armStop(stepper, /*armed=*/true, switch_active_high, end,
            LatchEdge::kActive);
    setVelocityRevsPerSecond(stepper, velocities[i]);
    const int32_t start = stepper.controller.readActualPosition();
    const MoveReport r = moveTo(
        stepper, start + toward * conv::kHomeTravelBoundMicrosteps, timeout_ms);
    if (r.outcome != MoveOutcome::kSwitchStopped || !r.latch_seen) {
      armStop(stepper, /*armed=*/false, switch_active_high, end,
              LatchEdge::kActive);
      setJogVelocity(stepper);
      return d;
    }
    latches[i] = r.latch_position;
  }

  // Leave the mechanism clear of the switch, stop disarmed, jog velocity.
  armStop(stepper, /*armed=*/false, switch_active_high, end,
          LatchEdge::kActive);
  setJogVelocity(stepper);
  moveBy(stepper, -toward * conv::kHomeBackoffMicrosteps, timeout_ms);

  d.latch_fast = latches[0];
  d.latch_slow = latches[1];

  // latch(v) = edge + toward * v * delay, so the signed difference divided by
  // the velocity difference is the delay regardless of which end we homed to.
  const float delta_position = static_cast<float>(d.latch_fast - d.latch_slow) *
                               static_cast<float>(toward);
  const float delta_velocity = d.velocity_fast - d.velocity_slow;
  if (delta_velocity <= 0.0f) {
    return d;
  }
  d.delay_ms = 1000.0f * delta_position / delta_velocity;
  d.valid = true;
  return d;
}

bool MotionSetup::switchActive(TMC51X0 &stepper) {
  // Both bits, because swap_lr decides which one the single fitted switch shows
  // up on, and the unused input is tied to GND so it cannot read active at the
  // non-inverted polarity this board uses.
  Regs::RampStat ramp;
  ramp.raw = stepper.registers.read(Regs::RampStatAddress);
  return ramp.status_stop_l() || ramp.status_stop_r();
}

uint32_t MotionSetup::readIoin(TMC51X0 &stepper) {
  return stepper.registers.read(Regs::IoinAddress);
}

MotionSetup::RampConfig MotionSetup::readRampConfig(TMC51X0 &stepper) {
  RampConfig c;
  c.rampmode = stepper.registers.read(Regs::RampmodeAddress);
  c.target = static_cast<int32_t>(stepper.registers.read(Regs::XtargetAddress));
  c.actual = stepper.controller.readActualPosition();
  c.velocity = stepper.controller.readActualVelocity();
  c.tstep = stepper.controller.readTstep();
  c.sw_mode = stepper.registers.read(Regs::SwModeAddress);
  c.vmax_mirror = stepper.registers.getStored(Regs::VmaxAddress);
  c.amax_mirror = stepper.registers.getStored(Regs::AmaxAddress);
  c.dmax_mirror = stepper.registers.getStored(Regs::DmaxAddress);
  c.vstart_mirror = stepper.registers.getStored(Regs::VstartAddress);
  c.vstop_mirror = stepper.registers.getStored(Regs::VstopAddress);
  c.v1_mirror = stepper.registers.getStored(Regs::Velocity1Address);
  c.a1_mirror = stepper.registers.getStored(Regs::Acceleration1Address);
  c.d1_mirror = stepper.registers.getStored(Regs::Deceleration1Address);
  return c;
}

} // namespace mouse_joystick
