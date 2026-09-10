// Stage 3: this board's TMC5130A motion configuration, over TMC51X0.
//
// Stage 2 proved the SPI path with raw 40-bit datagrams and no library, which
// was the right tool for "does the bus reach the chip". Stage 3 needs a ramp
// generator, an encoder scaler and a homing sequence, and TMC51X0 already has
// all three, bench-validated on the `tmc51x0-driver-dev-pcb` predecessor. So
// this file is glue, not a driver: everything here is a fact about
// `mouse-joystick-pcb` that TMC51X0 cannot know.
//
// Three of those facts are load bearing:
//
//   1. **The clock is external and 16 MHz.** X1201, on +3.3V_SW. TMC51X0's
//      ConverterParameters default to 12 MHz, the internal clock, so leaving it
//      unset makes every velocity 33% wrong in a way nothing reports.
//
//   2. **CHOPCONF.vsense is the whole current decision, and only half of it
//      is readable.** With this board's 0.15 ohm shunts, vsense = 0 is the
//      difference between 0.21 A and 0.38 A at CS 8, or between 0.68 A and
//      1.23 A at CS 28. TMC51X0 4.1.0 can set it --
//      `DriverParameters::withSenseVoltageMode()`, carried through
//      `driver.setup()`, through homing, and through a recovery replay -- so
//      `configure()` just asks for it.
//
//      What this firmware still owns is the verification. IHOLD_IRUN is
//      write-only, so CHOPCONF is the only register that can confirm any part
//      of the current setting. `applyHighSensitivity()` reads it back and
//      `enableDriver()` refuses until the read-back says 1. That is the
//      AGENTS.md rule "set and read back vsense = 1 before lowering
//      ~{TMC_DRV_EN}", and it stays a rule even though the library now does
//      the writing: a gate you can read is worth more than a write you
//      assume took.
//
//      Requires TMC51X0 >= 4.1.0. Before that the bit was unreachable through
//      the API and had to be set with a raw register write, which did not
//      survive homing or a recovery replay.
//
//   3. **The encoder never reaches a GPIO.** Position comes from X_ENC and
//      XACTUAL over SPI, and X_ENC does not move at all until ENC_CONST is
//      written, because its reset default is zero.
//
// The bus switch: everything here assumes `PowerSequence::connectBus()` has
// already returned true. Configuring SPI0's pins earlier is harmless -- the
// switch is what isolates them -- but talking to the driver is not.

#pragma once

#include <Arduino.h>
#include <TMC51X0.hpp>
#include <stdint.h>

namespace mouse_joystick {

// What the current configuration actually reads back as. `safe` is the whole
// point: it is the gate on enabling the output stage.
struct CurrentCheck {
  uint32_t chopconf;          // CHOPCONF as read back from the chip
  bool vsense;                // bit 17, read back rather than assumed
  bool toff_nonzero;          // TOFF != 0, i.e. the chopper is not disabled
  uint8_t run_current_scale;  // IRUN as written; the register is write-only
  uint8_t hold_current_scale; // IHOLD as written
  float run_amps_rms;         // what that IRUN means at the read-back vsense
  float hold_amps_rms;
  bool within_ceiling; // run_current_scale <= kCurrentScaleCeiling
  bool safe;           // vsense && within_ceiling
};

// One atomic sample. RAMP_STAT is read exactly once here on purpose:
// status_latch_l and the event_* bits are read-and-clear, and TMC51X0's
// `leftSwitchActive()` / `leftLatchActive()` / `leftStopEvent()` each perform
// their own read, so calling two of them in a row loses whichever flag the
// first read consumed.
struct MotionSnapshot {
  uint32_t ramp_stat;
  uint32_t drv_status;
  int32_t position; // XACTUAL, microsteps
  int32_t velocity; // VACTUAL, microsteps/t, signed
  int32_t encoder;  // X_ENC, scaled to microsteps by ENC_CONST
};

enum class MoveOutcome {
  kTargetReached,
  kSwitchStopped, // the reference switch stopped the ramp
  kNotFollowing,  // the encoder did not track the ramp: end stop or stall
  kDriverFault,   // over-temperature or short to ground
  kTimedOut,
};

struct MoveReport {
  MoveOutcome outcome;
  int32_t start_position;
  int32_t end_position;
  int32_t start_encoder;
  int32_t end_encoder;
  uint32_t elapsed_ms;
  // Measured encoder travel divided by commanded travel. About 1.0 when the
  // shaft follows; near 0 against an end stop; near -1 if the encoder counts
  // backwards, which is an ENC_CONST sign to flip rather than a fault.
  float follow_ratio;
  uint32_t ramp_stat_at_end;
  uint32_t drv_status_at_end;
  // A reference-switch latch seen during the move, captured by the chip at the
  // edge itself rather than by a poll. That distinction is what makes the
  // creep's datum better than the poll interval: polling every 5 ms at the
  // creep velocity would quantise the datum to 5 microsteps, and the hardware
  // latch has no such floor.
  bool latch_seen;
  int32_t latch_position; // XLATCH
  int32_t latch_encoder;  // ENC_LATCH
};

// Which end of travel the limit switch sits at.
//
// This is not a preference, it is a wiring fact with a consequence. The
// TMC5130A's REFL input stops NEGATIVE motion and REFR stops POSITIVE motion,
// and on `mouse-joystick-pcb` the only switch is wired to REFL -- J1301 ->
// U1301 MAX6816 -> LIMIT_SW -> TMC pin 8, with REFR tied to GND. So if the
// switch is physically at the positive end of travel, arming the left stop
// protects the wrong direction: the driver would happily drive into the switch
// and stop only on the way back out.
//
// SW_MODE.swap_lr exists for exactly this. Setting it makes the REFL pin act as
// the right switch, so it stops positive motion. `armStop()` does that, and
// `kPositive` is what the rig measured on 2026-09-02: a +1280 microstep jog
// took REFL_STEP from 0 to 1.
// Which switch edge latches XLATCH. Exactly one at a time: the TMC5130A
// overwrites XLATCH on every enabled latch event, so enabling both edges means
// the value never tells you which edge produced it.
enum class LatchEdge {
  kActive,   // the switch closing -- what the fast approach uses
  kInactive, // the switch releasing -- what the slow creep uses, and the datum
};

enum class HomeEnd {
  kNegative, // switch at decreasing XACTUAL; REFL used as the left stop
  kPositive, // switch at increasing XACTUAL; REFL swapped to act as the right
};

enum class HomeOutcome {
  kHomed,
  kSwitchNeverAsserted, // travelled the whole bound with no switch event
  kMechanicalStopFirst, // the encoder stopped following before the switch did
  kSwitchStuckActive,   // could not get clear of the switch to start
  kReleaseNeverLatched, // the fast approach found it, the creep never left it
  kDriverFault,
  kTimedOut,
};

struct HomeReport {
  HomeOutcome outcome;

  // Stage 1: the fast approach, ending on the switch.
  MoveReport approach;
  int32_t coarse_latch;     // XLATCH at the closing edge, pre-zero frame
  int32_t coarse_encoder;   // ENC_LATCH likewise
  int32_t settled_position; // XACTUAL once the ramp stopped
  // settled_position - coarse_latch. Zero for a hard stop, which is what this
  // board uses; a large value means the stop was soft. It does NOT include the
  // debounce delay, which elapses before the driver sees anything.
  int32_t overshoot;

  // Stage 2: the slow creep off the switch. `fine_latch` is the datum.
  bool creep_ran;
  bool release_latched;
  int32_t fine_latch;     // XLATCH at the releasing edge, pre-zero frame
  int32_t fine_encoder;   // ENC_LATCH likewise
  int32_t creep_distance; // how far the creep travelled to find the release

  // coarse_latch - fine_latch, i.e. the approach's overshoot plus the switch's
  // throw. Grows with approach velocity, which is what makes a single fast
  // approach a worse datum than this one.
  int32_t coarse_to_fine;

  int32_t backoff_position;    // where the clearance move ended, post-zero
  bool released_after_backoff; // the switch let go and stayed released
  uint32_t elapsed_ms;
};

// Two engage latches taken at two approach velocities, which is enough to solve
// for the debounce delay: latch(v) = edge + v * delay, so the difference over
// the velocity difference is the delay. Both latches are taken in one frame,
// with no re-zero in between, or the subtraction means nothing.
struct SwitchDelay {
  bool valid;
  int32_t latch_fast;
  int32_t latch_slow;
  float velocity_fast; // microsteps/s
  float velocity_slow;
  float delay_ms;
};

// The measured board facts stage 3 configures the driver from. Every field is
// something the bench established rather than something the datasheet says.
struct MotionConfig {
  uint8_t run_current_scale;
  uint8_t hold_current_scale;

  // The three facts a mirrored mechanism changes. The rig has a left and a
  // right joystick and the left is mechanically mirrored, so on that side the
  // motor turns the other way, the encoder counts the other way, and the limit
  // switch sits at the other end of travel in motor coordinates.
  //
  // Keeping all three here means "which side am I" is one decision with three
  // consequences, wherever that decision comes from -- a host command, a
  // board strap, or nothing at all if the harnesses are built mirrored so both
  // sides look identical from the connector inwards.
  //
  // GCONF.shaft, via TMC51X0's writeMotorDirection().
  bool motor_reversed;
  // X_ENC counts down while XACTUAL counts up. A mounting sense, not a fault.
  bool encoder_inverted;
  // The level IOIN.REFL_STEP shows when the switch is triggered.
  bool switch_active_high;
  HomeEnd switch_end;
};

class MotionSetup {
public:
  // SPI0 on this board's pins, then hand the configured bus to TMC51X0 and
  // tell it where ~{TMC_DRV_EN} is. TMC51X0 drives that pin high (disabled) as
  // soon as it is registered, so this cannot energise anything.
  //
  // Expects the bus switch closed. Declaring the device model up front seeds
  // the register mirror with TMC5130A reset defaults instead of guessing from
  // the first identity read.
  static void beginSpi(TMC51X0 &stepper);
  static void endSpi(TMC51X0 &stepper);

  // Converter, driver, controller and encoder, with this board's parameters and
  // the bring-up motion envelope. Leaves the output stage disabled.
  //
  // StealthChop is off deliberately. It is TMC51X0's default and it is the
  // wrong default for a first move on an uncharacterised motor: its PWM
  // autoscale needs a tuning pass, and until it has had one the coil current at
  // low velocity is not the current that was asked for.
  //
  // The switch orientation is applied here too, with the stop disarmed. That
  // matters: SW_MODE.swap_lr decides which status bit the one fitted switch
  // appears on, and `moveTo`'s software guard reads those bits to decide
  // whether a move is heading into the switch. Set it once, here, so the
  // mapping holds for every move and not only for an armed homing approach.
  static void configure(TMC51X0 &stepper, const MotionConfig &config);

  // Set CHOPCONF.vsense, then read CHOPCONF back and report what it says.
  // Call after `configure()` and after any recovery, and check `safe` before
  // enabling anything.
  static CurrentCheck applyHighSensitivity(TMC51X0 &stepper,
                                           uint8_t run_current_scale,
                                           uint8_t hold_current_scale);

  // Lower ~{TMC_DRV_EN} and set a non-zero TOFF -- but only if `check.safe`.
  // Returns false and leaves the driver disabled otherwise.
  static bool enableDriver(TMC51X0 &stepper, const CurrentCheck &check);
  static void disableDriver(TMC51X0 &stepper);

  static MotionSnapshot snapshot(TMC51X0 &stepper);

  // Decelerate to standstill at AMAX and hold. The datasheet's own stop: a
  // velocity-mode target of zero. Returns when VACTUAL reads zero or the
  // timeout expires.
  static bool stopAndHold(TMC51X0 &stepper, uint32_t timeout_ms = 500);

  // A bounded, encoder-watched move to an absolute microstep position.
  //
  // This is the only motion primitive stage 3 uses for jogging, because the
  // travel of the rig's joystick has not been measured and a plain
  // `writeTargetPosition()` has no idea when it has run out of mechanism. The
  // ramp generator's XACTUAL advances whether or not the shaft follows, so
  // comparing it against X_ENC detects an end stop, a stall, a slipping
  // coupling and a dead encoder -- and stops instead of pushing harder.
  //
  // Always ends at standstill.
  static MoveReport moveTo(TMC51X0 &stepper, int32_t target_position,
                           uint32_t timeout_ms);

  // Relative form, from the current XACTUAL.
  static MoveReport moveBy(TMC51X0 &stepper, int32_t delta_microsteps,
                           uint32_t timeout_ms);

  // Velocity selection. Two named speeds rather than a parameter, because the
  // homing speed is a safety number: the MAX6816's debounce delay is added to
  // every approach, so the driver learns of the closure late by that delay
  // times the approach velocity, and the overshoot scales with speed.
  static void setJogVelocity(TMC51X0 &stepper);
  static void setHomeApproachVelocity(TMC51X0 &stepper);
  static void setHomeCreepVelocity(TMC51X0 &stepper);
  static void setVelocityRevsPerSecond(TMC51X0 &stepper, float revs_per_second);

  // Reverse the encoder's counting direction, or put it back. ENC_CONST is
  // write-only, so this cannot be verified by reading it back -- the
  // verification is that a move's follow ratio comes out positive.
  static void setEncoderInverted(TMC51X0 &stepper, bool inverted);

  // Arm or disarm the reference switch stop, at a MEASURED polarity and for the
  // end of travel the switch is actually on. See HomeEnd: on this board that
  // choice decides whether SW_MODE.swap_lr is set, and getting it wrong arms
  // the stop against the direction that moves AWAY from the switch.
  //
  // Whichever end is selected, the other stop stays disabled. REFR is tied to
  // GND, so enabling the unused stop would read a permanent stop and block
  // motion in that direction.
  static void armStop(TMC51X0 &stepper, bool armed, bool switch_active_high,
                      HomeEnd end, LatchEdge edge);

  // What a motion-driven look at the limit switch established.
  struct SwitchWitness {
    bool resting_level;       // IOIN.REFL_STEP with the mechanism clear
    bool active_high;         // the level the switch shows when triggered
    bool transition_seen;     // the level actually changed under motion
    bool released_again;      // and changed back when the mechanism backed off
    int32_t engage_position;  // XACTUAL where it first read triggered
    int32_t release_position; // XACTUAL where it read untriggered again
    int32_t travel_used;      // how far the approach went to find it
    MoveOutcome approach_outcome;

    // Why the sweep stopped, when it stopped without seeing a transition.
    // WITHOUT THIS the three cases are indistinguishable in the report, and
    // the failure text blamed U1301's output joint for what was actually the
    // sweep running out of time. That happened three times on 2026-09-10.
    enum class StopReason : uint8_t {
      kTransitionSeen, // the switch changed -- not a failure
      kBudgetExpired,  // ran out of sweep time before reaching the switch
      kTravelBound,    // hit the distance bound
      kMoveBlocked,    // a step ended on something other than its target
    };
    StopReason stop_reason;
  };

  // Establish the switch's polarity and prove its path, using the motor.
  //
  // The polarity does not actually need an actuation to infer: with the
  // mechanism clear of the end of travel the switch is untriggered, so
  // "triggered" is whatever level IOIN.REFL_STEP is not showing. What needs an
  // actuation is the other half -- that the level can change at all, which is
  // what separates a working U1301 from an open joint on its output, and which
  // the two rig boards' disagreement left open.
  //
  // On this rig the switch sits at a limit of travel, so the motor can reach it
  // and no hand is needed. The approach is stepped rather than continuous, and
  // each step is bounded and encoder-watched, so a mechanism that runs out
  // before the switch stops the sweep instead of straining.
  //
  // Safe against its own assumption being wrong: the polarity is written before
  // the sweep, so if the switch were somehow already triggered, the very first
  // step reads as blocked and the sweep reports no travel rather than driving.
  // `move_timeout_ms` bounds each individual 128-microstep step.
  // `sweep_timeout_ms` bounds the whole search, and it must be much larger.
  //
  // THESE WERE ONE PARAMETER AND THAT WAS A BUG. Both were passed
  // kJogTimeoutMs = 4000, which is right for one jog and badly wrong for a
  // sweep: at jog velocity a 128-microstep step costs ~150 ms including
  // overhead, so 4 s covers only about 27 steps -- 24 degrees, less than this
  // rig's ~60 degree mechanism. A sweep starting more than 24 degrees from the
  // switch could never reach it, and reported the switch as not responding.
  static SwitchWitness witnessByMotion(TMC51X0 &stepper, HomeEnd end,
                                       uint32_t move_timeout_ms,
                                       uint32_t sweep_timeout_ms);

  // The datasheet's homing procedure, section 14.4, with this board's bounds.
  //
  // Homing assumes the switch is at the NEGATIVE end of travel. The approach is
  // bounded by a target position rather than by a stop condition, so a switch
  // that never asserts ends the move instead of running the mechanism into
  // something -- and the encoder follow check ends it sooner still if a
  // mechanical stop comes first.
  //
  // On success the reference is moved so that position 0 is the switch point,
  // the encoder is zeroed there too, the stop is disarmed and the mechanism is
  // backed off clear of the switch.
  //
  // Deliberately not TMC51X0's own beginHomeToSwitch()/homed(): that pair
  // reports success on standstill alone, so "stopped at the switch" and "ran
  // out of travel bound" are indistinguishable -- which is exactly the
  // distinction a bring-up has to make.
  static HomeReport homeToSwitch(TMC51X0 &stepper, bool switch_active_high,
                                 HomeEnd end, uint32_t timeout_ms);

  // Approach the switch twice at two velocities and solve for the MAX6816's
  // debounce delay. Leaves the mechanism clear of the switch and does NOT
  // re-zero, because the two latches have to share a frame.
  //
  // This is the measurement that sizes the creep bound and justifies the
  // approach velocity, and it is the reason to prefer a two-stage home: the
  // delay it reports is exactly the error a single fast approach would bake
  // into the datum.
  static SwitchDelay measureSwitchDelay(TMC51X0 &stepper,
                                        bool switch_active_high, HomeEnd end,
                                        uint32_t timeout_ms);

  // Whether the reference switch reads active right now, at the current
  // polarity and swap setting. Checks both stop status bits, because which one
  // the single fitted switch appears on depends on swap_lr. One RAMP_STAT
  // read.
  static bool switchActive(TMC51X0 &stepper);

  // IOIN, for the raw REFL_STEP level regardless of polarity configuration.
  static uint32_t readIoin(TMC51X0 &stepper);

  // What the ramp generator was actually told, and what it is actually doing.
  //
  // Most of the ramp registers are write-only on the TMC5130A, so "what was
  // written" can only come from TMC51X0's software mirror while "what is
  // happening" has to come from the readable ones. Keeping both in one struct
  // is what separates "firmware wrote the wrong number" from "the chip ignored
  // the right one" -- a distinction that cost a bench session to make by
  // inference.
  struct RampConfig {
    uint32_t rampmode;    // 0x20, readable
    int32_t target;       // 0x2D, readable
    int32_t actual;       // 0x21, readable
    int32_t velocity;     // 0x22, readable, signed 24-bit
    uint32_t tstep;       // 0x12, readable: measured microstep period
    uint32_t vmax_mirror; // 0x27, write-only on the chip
    uint32_t amax_mirror; // 0x26
    uint32_t dmax_mirror; // 0x28
    uint32_t vstart_mirror;
    uint32_t vstop_mirror;
    uint32_t v1_mirror;
    uint32_t a1_mirror;
    uint32_t d1_mirror;
    uint32_t sw_mode; // 0x34, readable
  };
  static RampConfig readRampConfig(TMC51X0 &stepper);

private:
  static bool driver_enabled_;
};

} // namespace mouse_joystick
