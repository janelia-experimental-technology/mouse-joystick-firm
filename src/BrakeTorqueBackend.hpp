// The seam between the protocol layer and pcb v1.0's brake sink and torque
// bridge.
//
// `ProtocolBoard` owns the state machine, the fields and the timing rules;
// it owns no brake current law and no ADC. Everything about *how much coil
// current a torque is* and *what the bridge reads* sits behind this
// interface, for the same two reasons as `MotionBackend`:
//
//  1. The shared corpus (`pycontrol/tests/fixtures/protocol_cases.txt`) has
//     to run on the host, where there is no DAC80501 and no ADS8689.
//     `SimulatedBrakeTorque` below is what makes that possible, and it is
//     deliberately the same model the Python simulator uses -- a brake that
//     settles a fixed time after it is commanded, a torque that is whatever
//     the TORQUE stimulus last set -- so the two agree on protocol behaviour
//     without either pretending to model a coil.
//
//  2. The real brake is not a step. Measured on v1.0 unit E4654C159F274534
//     on 2026-10-06: a full-scale code step overshoots to 127 % of the
//     setpoint for ~5 ms (the coil's L di/dt against the sink's headroom
//     winds the loop integrator up), so the hardware backend ramps the code
//     and reports `brakeSettled()` only once the coil current has arrived.
//     The protocol releases the drive at that moment and not before, which is
//     the ordering uart-protocol.md 3a asks for.
//
// Arduino-free, like everything the native test compiles. The hardware
// implementation is `BrakeTorqueHw.hpp` (firmware only).

#pragma once

#include <stdint.h>

namespace mouse_joystick {

class BrakeTorqueBackend {
public:
  virtual ~BrakeTorqueBackend() {}

  // Advance to `now_us`: sample the bridge, step the brake ramp, judge the
  // coil current. Called once per service step, which is the firmware's
  // 1 kHz tick.
  virtual void poll(uint64_t now_us) = 0;
  // See MotionBackend::beginAdvance(): one bus read per advance.
  virtual void beginAdvance() {}

  // The earliest time `poll()` could change `brakeSettled()`, or 0 for
  // "unknown". The simulated backend returns its settle instant so the clock
  // is stepped to it exactly and RELEASED lands on the microsecond the corpus
  // expects. The real backend returns 0: it is polled.
  virtual uint64_t nextEventUs() const { return 0; }

  // ------------------------------------------------------------ torque

  // The bridge is powered, the ADC configured and answering, and the latest
  // conversion inside its range.
  virtual bool torqueValid() const = 0;

  // The latest sample, tared, in N.cm, with the sign the sensor gives it.
  // Which sign is a pull is a board-and-harness fact for the calibration
  // (rig plan C3); the protocol reports what it reads.
  virtual double torqueNcm() const = 0;

  // The latest sample before the tare: the nominal scale about the nominal
  // zero. `TQ get` reports both so a host can see what a tare removed.
  virtual double torqueRawNcm() const = 0;

  // Tare at the latest sample.
  virtual void torqueZero() = 0;

  // The shunt-calibration switch. On pcb v1.0 as built it does nothing (the
  // switch's MOSFET cannot turn on, 2026-10-06), so this is a verb and not a
  // calibration there; the protocol accepts it either way.
  virtual void torqueCal(bool on) = 0;
  virtual bool torqueCalOn() const = 0;

  // ------------------------------------------------------------- brake

  // Begin engaging the brake toward `torque_ncm`. The backend owns the
  // torque-to-current law and the ramp. False means the brake cannot be
  // driven now (its supply is down, its DAC is not configured), which the
  // protocol layer turns into a fault rather than a nak: the attempt was made.
  virtual bool brakeEngage(double torque_ncm) = 0;

  // The brake is at its setpoint: the ramp has finished and the coil current
  // has settled. The protocol releases the drive only then.
  virtual bool brakeSettled() const = 0;

  // Back to zero current, now. Falls are clean on this sink (48.8 to 1.4 mA
  // in 8 ms, no undershoot), so no ramp down.
  virtual void brakeRelease() = 0;

  // What the coil is actually doing, from the sink's own monitor, through
  // the nominal current-to-torque line. For STATUS-level supervision, not
  // for control: the current loop is closed in hardware.
  virtual double brakeMeasuredNcm() const = 0;

  // Asynchronous failure, reported once and then cleared: fault bits 9-12.
  virtual bool consumeFault(uint8_t *bit, const char **detail) {
    (void)bit;
    (void)detail;
    return false;
  }
};

// ---------------------------------------------------------------------------
// The host-testable model, held to the corpus together with the Python
// simulator's: the brake settles kSettleUs after `brakeEngage()`, the torque
// is whatever `setTorqueNcm()` (the corpus's TORQUE stimulus) last set and
// is 0.0 at boot, `torqueZero()` tares it, the cal switch is a flag.
// ---------------------------------------------------------------------------
class SimulatedBrakeTorque : public BrakeTorqueBackend {
public:
  // About what the real sink takes to ramp to full scale and settle (an 8 ms
  // ramp plus the coil's lag), rounded to a number a corpus can read.
  static constexpr uint32_t kSettleUs = 10000u;

  void poll(uint64_t now_us) override { now_us_ = now_us; }

  uint64_t nextEventUs() const override {
    return (engaged_ && now_us_ < settle_at_us_) ? settle_at_us_ : 0u;
  }

  bool torqueValid() const override { return true; }
  double torqueNcm() const override { return torque_ - zero_; }
  double torqueRawNcm() const override { return torque_; }
  void torqueZero() override { zero_ = torque_; }
  void torqueCal(bool on) override { cal_on_ = on; }
  bool torqueCalOn() const override { return cal_on_; }

  bool brakeEngage(double torque_ncm) override {
    setpoint_ = torque_ncm;
    engaged_ = true;
    settle_at_us_ = now_us_ + kSettleUs;
    return true;
  }
  bool brakeSettled() const override {
    return engaged_ && now_us_ >= settle_at_us_;
  }
  void brakeRelease() override {
    engaged_ = false;
    setpoint_ = 0.0;
  }
  double brakeMeasuredNcm() const override {
    return brakeSettled() ? setpoint_ : 0.0;
  }

  // Test hook: the TORQUE stimulus. The sensor reads this from now on.
  void setTorqueNcm(double torque_ncm) { torque_ = torque_ncm; }

private:
  uint64_t now_us_ = 0;
  double torque_ = 0.0;
  double zero_ = 0.0;
  bool cal_on_ = false;
  bool engaged_ = false;
  double setpoint_ = 0.0;
  uint64_t settle_at_us_ = 0;
};

}  // namespace mouse_joystick
