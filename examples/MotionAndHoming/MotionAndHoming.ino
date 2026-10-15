// Stage 3 bring-up: move the motor, read the encoder, home against the limit
// switch.
//
// REQUIRES 36 V on J201, and the motor, encoder and limit switch connected.
// Stages 1 and 2 must have passed first: this program assumes the MCU is known
// good and that SPI reaches the TMC5130A, and it re-proves neither.
//
// This is the first stage that can put current through a motor, so the order of
// the gates below is the safety argument, not a style choice:
//
//   1. Rail up through PowerSequence, bus switch closed only after that.
//   2. TMC51X0 attached, driver still disabled, identity confirmed.
//   3. CHOPCONF.vsense set to 1 and READ BACK. Nothing enables the output
//      stage until that read-back succeeds -- with this board's 0.15 ohm
//      shunts, vsense = 0 is 1.8x the requested current, and IHOLD_IRUN is
//      write-only so the current scale itself cannot be verified at all.
//   4. The limit switch is *witnessed* before it is armed. Its polarity is a
//      property of the fitted switch's wiring; an inverted guess makes an
//      untriggered switch read as a permanent stop, which presents as a dead
//      motor.
//   5. Motion, in bounded moves watched by the encoder.
//
// Every move is bounded by a target position and watched against X_ENC,
// because the motor is installed in the rig through the gear pair and the
// joystick's travel has not been measured. A ramp generator has no idea when it
// has run out of mechanism; the encoder does.
//
// Watching it with a logic analyser: FW_TRIG1 is high for the duration of each
// motion command and FW_TRIG0 pulses on each switch transition the witness test
// sees. On a v1.0 board trigger on LA5032 channel 1 or 0 and capture 28 and 29
// for ENCA/ENCB and 15 for LIMIT_SW; on rev 0/1 trigger on 17 or 16 and capture
// 0 and 1 for ENC_A/ENC_B and 11 for LIMIT_SW.

#include <MotionSetup.hpp>
#include <MouseJoystickBoard.hpp>
#include <TMC51X0.hpp>

// The angle-output DAC, rev 1 and later only. It is here rather than in
// examples/AngleOutput because the measurement that needs both is coupling:
// park the output at a static DC level, move the motor, and anything AC on the
// BNC is the motor getting into the analog chain. Neither example could do that
// alone -- AngleOutput has no motion and this one had no DAC.
#if MJ_BOARD_REV != 0
#include <DAC80501.hpp>
#endif

using mouse_joystick::CurrentCheck;
using mouse_joystick::HomeOutcome;
using mouse_joystick::HomeReport;
using mouse_joystick::MotionSetup;
using mouse_joystick::MotionSnapshot;
using mouse_joystick::MoveOutcome;
using mouse_joystick::MoveReport;
using mouse_joystick::PowerSequence;
using mouse_joystick::PowerUpReport;
using mouse_joystick::PowerUpResult;
using mouse_joystick::RailState;
using mouse_joystick::SafeState;
using mouse_joystick::SelfTest;

namespace conv = mouse_joystick::conversions;
#if MJ_BOARD_REV != 0
namespace pins = mouse_joystick::pins;
#endif

constexpr uint32_t kSerialBaudRate = 115200;
constexpr uint32_t kSerialWaitMs = 5000;

// Sized from the motion envelope: one jog step at the jog velocity is a
// quarter of a second, and the whole homing bound at the homing velocity is
// forty seconds.
constexpr uint32_t kJogTimeoutMs = 4000;
constexpr uint32_t kHomeTimeoutMs = 60000;
constexpr uint32_t kWitnessWindowMs = 12000;

// PowerSequence::kRailOffTimeoutMs is the same figure, kept there so the
// power-sequence code and this example cannot drift apart.
constexpr uint32_t kRailOffTimeoutMs = PowerSequence::kRailOffTimeoutMs;

TMC51X0 stepper;

// The gates are cumulative, so their results have to survive between commands.
bool bus_up = false;

#if MJ_BOARD_REV == 1
// SPI1, separate from the TMC's SPI0, so the two never contend. GP12 is SPI1's
// RX candidate that is routed nowhere on v0.3; the DAC has no SDO so nothing is
// read, but the core wants a pin assigned.
constexpr uint8_t kSpi1Rx = pins::kSpi1RxUnused; // BoardPins owns the number
#endif
#if MJ_BOARD_REV != 0
// REF-DIV 1 with BUFF-GAIN 1: a 2.5 V DAC full scale, which the OPA197's 1.32
// gain turns into 3.3 V at the BNC. Measured on board E4654C159F171E34 on
// 2026-09-10 as unity gain to within 0.03%.
constexpr uint16_t kDacGainRegister =
    dac80501::kGainRefDiv | dac80501::kGainBuffGain;
#if MJ_BOARD_REV == 1
constexpr float kDacOutputStageGain = 1.32f;
#else
// pcb v1.0 halves everything after the OPA197 with a 3.2k/3.2k divider
// (R807/R811): 2.5 V at the DAC is 1.650 V at the BNC and at ANGLE_OUT_MON.
// The 1.32 is the same amplifier (R804/R803); the divider is the v1.0 change.
// Not yet measured on a v1.0 board.
constexpr float kDacOutputStageGain = 1.32f * 0.5f;
#endif

// The live angle output loop.
//
// RATE: 1 kHz. docs/encoder-analog-output.md calls the update rate "an open
// decision" and asks for the achieved rate to be documented as a MEASURED
// claim, not a target -- so this loop measures itself and reports. 1 kHz is the
// rate the circuit was designed around: the 2-pole Sallen-Key corner is 987 Hz,
// and that same doc puts "roughly 1 kHz of real angle bandwidth" as what the
// loop can sustain alongside motor control. Going faster puts steps into a
// filter that cannot pass them.
//
// Note what this loop IS: a sampling operation. The 987 Hz filter sits AFTER
// the DAC, so it smooths the output steps but cannot undo aliasing from
// sampling the angle at 1 kHz. Mechanical content above ~500 Hz folds into the
// record and nothing downstream removes it. Mouse limb motion should be far
// below that, but it is a distinct issue from the OneBox aliasing the design
// notes already cover.
constexpr uint32_t kAngleUpdateIntervalUs = 1000; // 1 kHz

// Angle-to-volts mapping. The valid output range starts at 50 mV, which
// reserves everything below it as out-of-band -- see the floor discussion in
// docs/encoder-analog-output.md, and note the floor measured 4.5 mV on this
// board rather than the ~25 mV the doc assumes.
//
// Span is deliberately 90 deg at the motor against a mechanism of roughly
// 60 deg. The hard stops are movable and the travel is NOT settled, so a span
// that comfortably exceeds the mechanism is honest; a span fitted to one
// measured swing would silently misreport a re-adjusted stop. Gearing is 1:1,
// so motor degrees and joystick degrees are the same number.
constexpr float kAngleSpanDegrees = 90.0f;
#if MJ_BOARD_REV == 1
constexpr float kAngleOutMinVolts = 0.050f;
constexpr float kAngleOutMaxVolts = 3.300f;
#else
// Half of the above at the v1.0 BNC, so the DAC codes the loop writes are the
// same as on v0.3 and only the meter reading halves.
constexpr float kAngleOutMinVolts = 0.025f;
constexpr float kAngleOutMaxVolts = 1.650f;
#endif

// TRANSPORT SELECTION. The board revision decides it; MJ_DAC_I2C is derived.
//
// pcb v0.3 (rev 1) IS SPI: `SPI2C` (U2001 pad 5 at v0.3) is tied DIRECTLY TO
// GND in copper -- no strap resistor -- so v0.3 cannot run I2C without cutting
// a trace. pcb v1.0 (rev 2) IS I2C: both DACs strap SPI2C high (R808, R1809)
// and share I2C0 on GP16/GP17 behind the bus switch, and GP10/GP11/GP13 carry
// the torque ADC and the brake fault input instead, so the SPI DAC path must
// not even compile for it. v1.0 chose I2C for two reasons that have nothing
// to do with speed: STATUS.REF-ALARM is unreadable over SPI, and the 4.5 mV
// output floor measured on 2026-09-10 makes the analog level a weaker fault
// discriminator than the design assumed. Bandwidth is not a factor -- I2C at
// 400 kHz adds well under 100 us to a measured 527 us worst-case loop.
#if MJ_BOARD_REV >= 2
#if defined(MJ_DAC_I2C) && !MJ_DAC_I2C
#error "pcb v1.0 straps both DACs to I2C (R808, R1809); there is no SPI DAC path on this revision"
#endif
#undef MJ_DAC_I2C
#define MJ_DAC_I2C 1
#elif !defined(MJ_DAC_I2C)
#define MJ_DAC_I2C 0
#elif MJ_DAC_I2C
#error "no v0.3 board can run the I2C path: SPI2C is tied to GND in copper. The I2C DAC is MJ_BOARD_REV 2 (pcb v1.0)."
#endif

#if MJ_DAC_I2C
// A0 selects one of FOUR addresses by what it is tied to: AGND 0x48, VDD 0x49,
// SDA 0x4A, SCL 0x4B. So the angle DAC and the brake DAC share one bus with
// unique addresses and two spare -- no second bus, and 2 GPIOs instead of the
// 4 that SPI needs for two chip selects.
//
// The angle output takes 0x48 (R806 to GND). Whichever address a board straps
// has to match here; there is no way to discover it, because a wrong address
// simply NAKs. BoardPins.hpp owns the number, as it owns the pins.
constexpr uint8_t kDacI2cAddress = mouse_joystick::kAngleDacI2cAddress;
dac80501::I2cTransport dac_transport(Wire, kDacI2cAddress);
#else
dac80501::SpiTransport dac_transport(SPI1, pins::kDacCsN);
#endif
dac80501::Dac80501 dac(dac_transport);
#endif
bool configured = false;
CurrentCheck current_check;
bool polarity_measured = false;
bool switch_active_high = true; // a normally-closed switch reads high when hit
bool homed = false;
uint8_t run_current_scale = conv::kCurrentScaleBringUp;

// Measured on the rig, 2026-09-02, board E4654C159F4D1334.
//
// The switch is at the positive end of travel: a +1280 microstep jog took
// IOIN.REFL_STEP from 0 to 1.
//
// The encoder counts UP with XACTUAL, so ENC_CONST is positive. That is with
// the harness built to the table in
// mouse-joystick-pcb/docs/harness-pinouts.md, where the encoder's A and B
// channels are crossed relative to the board's ENC_A/ENC_B pins -- the E4T sits
// on the motor's rear shaft, so its "A leads B for clockwise" is referenced to
// the opposite end from the one the gear train is driven from. Measured both
// ways round on hardware: crossed gives a follow ratio of +1.0102, straight
// through gave -0.89.
//
// These are defaults rather than assumptions. 'm', 'o' and 'y' flip the three
// of them, and the reports that use them re-check each one.
mouse_joystick::HomeEnd switch_end = mouse_joystick::HomeEnd::kPositive;
bool encoder_inverted = false;
bool motor_reversed = false;

mouse_joystick::MotionConfig motionConfig() {
  mouse_joystick::MotionConfig config;
  config.run_current_scale = run_current_scale;
  config.hold_current_scale = conv::kCurrentScaleHold;
  config.motor_reversed = motor_reversed;
  config.encoder_inverted = encoder_inverted;
  config.switch_active_high = switch_active_high;
  config.switch_end = switch_end;
  return config;
}

// ---------------------------------------------------------------------------
// Printing
// ---------------------------------------------------------------------------

void printHex32(Print &out, uint32_t v) {
  out.print(F("0x"));
  for (int shift = 28; shift >= 0; shift -= 4) {
    out.print((v >> shift) & 0xF, HEX);
  }
}

void printPosition(Print &out, const char *label, int32_t microsteps) {
  out.print(label);
  out.print(microsteps);
  out.print(F(" usteps ("));
  out.print(conv::microstepsToDegrees(microsteps), 3);
  out.print(F(" deg, "));
  out.print(conv::microstepsToRevs(microsteps), 4);
  out.println(F(" rev)"));
}

void printMenu(Print &out) {
  out.println(F("commands"));
  out.println(F("  ?  this menu"));
  out.println(F("  s  run the whole stage 3 sequence"));
  out.println();
  out.println(F("  gates, in order"));
  out.println(F("  b  rail up, bus closed, TMC51X0 attached and configured"));
  out.println(F("  c  set CHOPCONF.vsense and read it back"));
  out.println(F("  w  witness the limit switch by hand: actuate it while"));
  out.println(F("     this runs (works with the driver disabled)"));
  out.println(F("  W  witness it with the motor: sweep onto the switch and"));
  out.println(F("     off again (needs 'e', so run 'c' first)"));
  out.println(F("  e  ENABLE the driver (refuses unless vsense reads back 1)"));
  out.println(F("  E  disable the driver"));
  out.println();
  out.println(F("  motion"));
  out.println(F("  j  jog +1 step     J  jog +10 steps"));
  out.println(F("  k  jog -1 step     K  jog -10 steps"));
  out.println(F("  g  encoder check: jog out and back, compare X_ENC"));
  out.println(F("  h  home against the limit switch (bounded)"));
  out.println(F("  H  homing repeatability, 5 runs"));
  out.println(F("  t  measure the switch's reporting delay, two velocities"));
  out.println(F("  n  set run current scale"));
  out.println(F("  o  flip the encoder counting sense"));
  out.println(F("  m  flip the motor direction (GCONF.shaft)"));
  out.println(F("  y  flip which end of travel the switch is on"));
#if MJ_BOARD_REV != 0
  out.println();
  out.println(F("  angle output, for the motor-coupling test"));
  out.println(F("  A  park the DAC at midscale (a static DC level)"));
  out.println(F("  Z  park the DAC at zero scale"));
  out.println(F("  F  park the DAC at full scale"));
  out.println(F("     Park it, then jog or home, and AC-couple a scope on the"));
  out.println(F("     BNC. The setpoint is DC, so anything AC is coupling."));
  out.println(F("  L  LIVE: drive the output from the encoder at 1 kHz until"));
  out.println(F("     a key is pressed. Needs 'h' first. Reports the"));
  out.println(F("     ACHIEVED rate, not the requested one."));
#endif
  out.println();
  out.println(F("  reporting"));
  out.println(F("  p  position, velocity, encoder, switch, driver status"));
  out.println(F("  d  ramp generator: mode, target, and every ramp register"));
  out.println(F("  i  IOIN"));
  out.println(F("  v  +3.3V_SW rail state"));
  out.println(F("  I  beacon the board serial on the InstrumentPort"));
  out.println(F("     triggers (no 36 V, no bus, no motion)"));
  out.println(F("  x  power down: driver off, bus open, HV_EN low"));
  out.println(F("  B  reboot into BOOTSEL      R  reboot"));
  out.println();
  out.print(F("  one jog step = "));
  out.print(conv::kJogMicrosteps);
  out.print(F(" usteps = "));
  out.print(conv::microstepsToDegrees(conv::kJogMicrosteps), 2);
  out.println(F(" deg at the motor shaft"));
  out.println();
}

const __FlashStringHelper *outcomeName(MoveOutcome outcome) {
  switch (outcome) {
  case MoveOutcome::kTargetReached:
    return F("target reached");
  case MoveOutcome::kSwitchStopped:
    return F("STOPPED BY THE SWITCH");
  case MoveOutcome::kNotFollowing:
    return F("NOT FOLLOWING -- end stop, stall, or dead encoder");
  case MoveOutcome::kDriverFault:
    return F("DRIVER FAULT");
  case MoveOutcome::kTimedOut:
    return F("TIMED OUT");
  }
  return F("unknown");
}

void printMoveReport(Print &out, const MoveReport &r) {
  out.print(F("    outcome: "));
  out.println(outcomeName(r.outcome));
  printPosition(out, "    from:    ", r.start_position);
  printPosition(out, "    to:      ", r.end_position);
  out.print(F("    commanded "));
  out.print(r.end_position - r.start_position);
  out.print(F(" usteps, encoder measured "));
  out.print(r.end_encoder - r.start_encoder);
  out.println(F(" usteps"));
  out.print(F("    follow ratio: "));
  out.print(r.follow_ratio, 4);
  if (r.follow_ratio < 0.0f) {
    out.println(F("  (NEGATIVE: the encoder counts the other way)"));
  } else {
    out.println();
  }
  out.print(F("    elapsed: "));
  out.print(r.elapsed_ms);
  out.println(F(" ms"));

  tmc51x0::Registers::DrvStatus drv;
  drv.raw = r.drv_status_at_end;
  if (drv.ot() || drv.otpw() || drv.s2ga() || drv.s2gb()) {
    out.print(F("    [!!] DRV_STATUS "));
    printHex32(out, r.drv_status_at_end);
    out.print(F("  ot="));
    out.print(drv.ot() ? F("1") : F("0"));
    out.print(F(" otpw="));
    out.print(drv.otpw() ? F("1") : F("0"));
    out.print(F(" s2ga="));
    out.print(drv.s2ga() ? F("1") : F("0"));
    out.print(F(" s2gb="));
    out.println(drv.s2gb() ? F("1") : F("0"));
  }
}

void printRampConfig(Print &out) {
  if (!bus_up) {
    out.println(F("Bus is open. Run 'b' first."));
    return;
  }
  const MotionSetup::RampConfig c = MotionSetup::readRampConfig(stepper);
  out.print(F("    RAMPMODE = "));
  out.print(c.rampmode);
  out.println(F("   0 position, 1 velocity+, 2 velocity-, 3 hold"));
  out.print(F("    XTARGET  = "));
  out.print(c.target);
  out.print(F("   XACTUAL = "));
  out.print(c.actual);
  out.print(F("   VACTUAL = "));
  out.println(c.velocity);
  out.print(F("    TSTEP    = "));
  out.print(c.tstep);
  if (c.tstep > 0 && c.tstep < 0xFFFFFu) {
    out.print(F("  -> "));
    out.print(16000000.0f / static_cast<float>(c.tstep), 1);
    out.print(F(" usteps/s measured"));
  } else {
    out.print(F("  (saturated: standstill or slower than 15 usteps/s)"));
  }
  out.println();
  out.print(F("    SW_MODE  = "));
  printHex32(out, c.sw_mode);
  out.println();
  out.println(F("    written (from TMC51X0's mirror -- these registers are"));
  out.println(F("    write-only on the chip, so this is intent, not truth):"));
  out.print(F("      VMAX   = "));
  out.print(c.vmax_mirror);
  out.print(F("  -> "));
  out.print(static_cast<float>(c.vmax_mirror) * 16000000.0f / 16777216.0f, 1);
  out.println(F(" usteps/s"));
  out.print(F("      AMAX   = "));
  out.print(c.amax_mirror);
  out.print(F("   DMAX = "));
  out.println(c.dmax_mirror);
  out.print(F("      VSTART = "));
  out.print(c.vstart_mirror);
  out.print(F("   VSTOP = "));
  out.println(c.vstop_mirror);
  out.print(F("      V1     = "));
  out.print(c.v1_mirror);
  out.print(F("   A1 = "));
  out.print(c.a1_mirror);
  out.print(F("   D1 = "));
  out.println(c.d1_mirror);
  if (c.d1_mirror == 0) {
    out.println(
        F("      [!!] D1 = 0. The datasheet forbids it in positioning"));
    out.println(F("      mode even when V1 = 0."));
  }
  if (c.vstop_mirror == 0) {
    out.println(F("      [!!] VSTOP = 0, also forbidden in positioning mode."));
  }
}

void printStatus(Print &out) {
  if (!bus_up) {
    out.println(F("Bus is open. Run 'b' first."));
    return;
  }
  const MotionSnapshot s = MotionSetup::snapshot(stepper);
  tmc51x0::Registers::RampStat ramp;
  ramp.raw = s.ramp_stat;
  tmc51x0::Registers::DrvStatus drv;
  drv.raw = s.drv_status;

  printPosition(out, "    XACTUAL: ", s.position);
  printPosition(out, "    X_ENC:   ", s.encoder);
  out.print(F("    VACTUAL: "));
  out.print(s.velocity);
  out.println(F(" usteps/t"));
  out.print(F("    switch:  status_stop_l="));
  out.print(ramp.status_stop_l() ? F("1") : F("0"));
  out.print(F(" latch_l="));
  out.print(ramp.status_latch_l() ? F("1") : F("0"));
  out.print(F("  vzero="));
  out.print(ramp.vzero() ? F("1") : F("0"));
  out.print(F(" position_reached="));
  out.println(ramp.position_reached() ? F("1") : F("0"));
  out.print(F("    DRV_STATUS "));
  printHex32(out, s.drv_status);
  out.print(F("  stst="));
  out.print(drv.stst() ? F("1") : F("0"));
  out.print(F(" cs_actual="));
  out.print(drv.cs_actual());
  out.print(F(" sg_result="));
  out.print(drv.sg_result());
  out.print(F(" ot="));
  out.print(drv.ot() ? F("1") : F("0"));
  out.print(F(" otpw="));
  out.println(drv.otpw() ? F("1") : F("0"));
  out.print(F("    ola="));
  out.print(drv.ola() ? F("1") : F("0"));
  out.print(F(" olb="));
  out.print(drv.olb() ? F("1") : F("0"));
  out.println(F("   (open load; only meaningful during slow motion)"));
}

// ---------------------------------------------------------------------------
// Gate 1: rail, bus, driver attach and configure
// ---------------------------------------------------------------------------

bool bringUpBus(Print &out) {
  out.println(F("[1] rail and bus"));
  PowerSequence::powerDown();

  // Wait for the rail to fall rather than sampling once. Dropping HV_EN does
  // not drain +36V_SW's bulk, so the switched rail lives on for seconds -- and
  // sampling at 50 ms made a second 'b' in a row read 3.31 V and blame the
  // board for a rail that was simply still discharging.
  if (!PowerSequence::waitForRailOff(kRailOffTimeoutMs)) {
    const RailState rail = PowerSequence::readRail();
    out.print(F("    [FAIL] +3.3V_SW is still at "));
    out.print(rail.rail_volts, 3);
    out.print(F(" V after "));
    out.print(kRailOffTimeoutMs / 1000);
    out.println(F(" s with HV_EN low."));
    out.println(
        F("      +36V_SW's bulk normally bleeds down in a few seconds."));
    out.println(
        F("      Longer than that means 36 V is reaching the switched"));
    out.println(
        F("      domain around the high-side switch, or the switch is"));
    out.println(F("      passing with EN low."));
    return false;
  }

  PowerUpReport report;
  const PowerUpResult result = PowerSequence::powerUp(report);
  if (result != PowerUpResult::kOk) {
    out.println(F("    [FAIL] the switched rail did not come up."));
    out.println(F("      Diagnose it with examples/TMC5130Communication: 'p'"));
    out.println(F("      for a millisecond-resolution profile, 'l' to check"));
    out.println(F("      whether this revision still needs the LATCH"));
    out.println(F("      workaround. Stage 3 does not duplicate that."));
    return false;
  }
  out.print(F("    [ ok ] +3.3V_SW up in "));
  out.print(report.rise_time_ms);
  out.print(F(" ms, peak "));
  out.print(report.peak_rail_volts, 3);
  out.print(F(" V, method: "));
  out.println(report.method == mouse_joystick::PowerUpMethod::kLatchGpio
                  ? F("direct HV_EN")
                  : F("firmware staircase"));
  if (report.method == mouse_joystick::PowerUpMethod::kLatchGpio) {
    out.print(F("    [ ok ] +36V_SW bulk charged at "));
    out.print(report.charge_ms);
    out.println(F(" ms (HV_SNS below the fault level); latch-off armed"));
  }

  if (!PowerSequence::connectBus()) {
    out.println(F("    [FAIL] refused to close the bus: rail not live."));
    return false;
  }
  delay(10);
  out.println(F("    [ ok ] bus closed, ~{TMC_DRV_EN} still high"));
  out.println();

  out.println(F("[2] TMC51X0 over SPI0, driver disabled"));
  MotionSetup::beginSpi(stepper);
  delay(10);

  const uint8_t version = stepper.readVersion();
  out.print(F("    IOIN.VERSION = 0x"));
  out.print(version, HEX);
  if (version != 0x11) {
    out.println(F("  [FAIL] expected 0x11, the TMC5130A signature."));
    out.println(F("      Stage 2 is the tool for a broken bus, not this."));
    return false;
  }
  out.println(F("  [ ok ] TMC5130A"));

  if (!stepper.communicating()) {
    out.println(F("    [FAIL] TMC51X0 reports no communication."));
    return false;
  }
  if (stepper.controller.stepAndDirectionMode()) {
    out.println(
        F("    [FAIL] SD_MODE is set, so step/direction mode is on and"));
    out.println(F("      SPI motion commands do nothing. SD_MODE is supposed"));
    out.println(F("      to be tied to GND on this board."));
    return false;
  }
  out.println(F("    [ ok ] motion-controller mode"));

  const tmc51x0::HealthStatus health = stepper.readHealthStatus();
  out.print(F("    GSTAT: reset="));
  out.print(health.reset ? F("1") : F("0"));
  out.print(F(" drv_err="));
  out.print(health.driver_error ? F("1") : F("0"));
  out.print(F(" uv_cp="));
  out.println(health.charge_pump_undervoltage ? F("1") : F("0"));
  out.println(F("      reset=1 is expected on the first read after a rail"));
  out.println(F("      power-up. uv_cp=0 means the charge pump is happy,"));
  out.println(F("      which needs VS genuinely up."));
  if (health.driver_error) {
    out.println(F("    [FAIL] GSTAT.drv_err is set before anything was"));
    out.println(F("      enabled. Stopping."));
    return false;
  }

  MotionSetup::configure(stepper, motionConfig());
  out.println(
      F("    [ ok ] converter, driver, controller, encoder configured"));
  out.print(F("      clock "));
  out.print(conv::kTmcClockMhz);
  out.println(F(" MHz external (X1201), 256 usteps/fullstep,"));
  out.println(F("      SpreadCycle, StealthChop off, CoolStep off"));
  out.print(F("      ENC_CONST = "));
  out.print(encoder_inverted ? F("-") : F("+"));
  out.print(
      static_cast<float>(conv::kEncoderMicrostepsPerPulseScaled) / 10000.0f, 1);
  out.println(F(" usteps per quadrature count, decimal mode."));
  out.println(F("      X_ENC now reads in microsteps, comparable to XACTUAL."));
  out.println(F("      Its scale reset to ZERO, so this write is what makes"));
  out.println(F("      the encoder count at all."));

  bus_up = true;
  configured = true;

#if MJ_BOARD_REV != 0
  // Only now. The DAC runs from the switched domain (+5V_SW on v0.3, filtered
  // +3.3V_SW on v1.0), so driving its bus before the rail is up pushes current
  // through its input clamps and back-powers that domain. On v0.3 the SPI bus
  // reaches it on the MCU side of the switch; on v1.0 the I2C pair is behind
  // the switch like everything else and the bus must be closed first.
  out.println(F("[4] angle-output DAC"));
#if MJ_DAC_I2C
  // I2C0 on the v1.0 pins. The core's Wire defaults are GP4/GP5, which here
  // are the TMC's CIPO and DIAG0, so the pins must be set before begin().
  Wire.setSCL(pins::kDacScl);
  Wire.setSDA(pins::kDacSda);
  dac_transport.begin();
  // 400 kHz, as the QP build: at the 100 kHz default one DAC write costs
  // ~400 us and the 1 kHz live loop's worst iteration reached 1082 us
  // (v1.0 unit E4654C159F274534, 2026-10-05).
  Wire.setClock(400000U);
  out.print(F("    transport: I2C at 0x"));
  out.println(kDacI2cAddress, HEX);
  out.println(F("    Pull-ups must be BOARD-level, not per instance -- two"));
  out.println(F("    DACs each carrying them would halve the resistance."));
#else
  SPI1.setSCK(pins::kDacSpiSck);
  SPI1.setTX(pins::kDacSpiCopi);
  SPI1.setRX(kSpi1Rx);
  SPI1.begin();
  dac_transport.begin();
  out.println(F("    transport: SPI1, write-only"));
#endif
  dac80501::Config dac_config = dac80501::arduinoConfig();
  dac_config.gain_register = kDacGainRegister;
  if (!dac.begin(dac_config)) {
#if MJ_DAC_I2C
    out.println(F("    [FAIL] no ACK. Over I2C that is real information:"));
    out.println(F("    wrong address strap, absent part, or missing"));
    out.println(F("    pull-ups. Unlike SPI, this failure is detectable."));
#else
    out.println(F("    [FAIL] DAC configuration frame failed. Over SPI the"));
    out.println(F("    transport can barely report anything -- suspect the"));
    out.println(F("    bus switch."));
#endif
  } else {
    out.print(F("    [ ok ] GAIN 0x"));
    out.print(kDacGainRegister, HEX);
    out.print(F(", "));
    out.print(dac.fullScale() * kDacOutputStageGain, 4);
    out.println(F(" V full scale at the BNC"));
    out.println(F("      Output is at zero scale, which measures 4.5 mV on"));
    out.println(F("      this board -- a clamp, not an offset. Park it with"));
    out.println(F("      'A' before looking for motor coupling."));
  }
#endif

  out.println();
  return true;
}

// ---------------------------------------------------------------------------
// Gate 2: the current decision
// ---------------------------------------------------------------------------

bool applyCurrent(Print &out) {
  if (!configured) {
    out.println(F("Not configured. Run 'b' first."));
    return false;
  }
  out.println(F("[3] current: CHOPCONF.vsense, set and read back"));

  current_check = MotionSetup::applyHighSensitivity(stepper, run_current_scale,
                                                    conv::kCurrentScaleHold);

  out.print(F("    CHOPCONF = "));
  printHex32(out, current_check.chopconf);
  out.println();
  out.print(F("    vsense read back = "));
  out.println(current_check.vsense ? F("1  [ ok ]") : F("0  [FAIL]"));
  out.print(F("    TOFF nonzero = "));
  out.println(current_check.toff_nonzero ? F("yes") : F("no (chopper off)"));
  out.print(F("    IRUN  = "));
  out.print(current_check.run_current_scale);
  out.print(F("  -> "));
  out.print(current_check.run_amps_rms, 3);
  out.println(F(" A RMS per coil"));
  out.print(F("    IHOLD = "));
  out.print(current_check.hold_current_scale);
  out.print(F("  -> "));
  out.print(current_check.hold_amps_rms, 3);
  out.println(F(" A RMS per coil"));
  out.println(
      F("      IHOLD_IRUN is write-only on the TMC5130A, so those two"));
  out.println(F("      are what firmware wrote, not what was read back."));
  out.println(F("      CHOPCONF is the readable half of the decision, and"));
  out.println(F("      vsense is the bit that carries the 1.8x error."));

  out.print(F("    motor WO-208-13-01D, 0.6 A rated. Ceiling IRUN "));
  out.print(conv::kCurrentScaleCeiling);
  out.print(F(" = "));
  out.print(conv::currentScaleToAmpsRms(conv::kCurrentScaleCeiling, true), 3);
  out.println(F(" A RMS"));
  out.println(F("      That ceiling reads the vendor's 0.6 A as a sine peak,"));
  out.println(
      F("      the conservative of the two possible readings. Measure"));
  out.println(F("      coil current and case temperature before raising it."));

  if (!current_check.vsense) {
    out.println(
        F("    [FAIL] vsense did not read back as 1. The driver stays"));
    out.println(F("      disabled -- every current above would be 1.8x what"));
    out.println(F("      is printed."));
    return false;
  }
  if (!current_check.within_ceiling) {
    out.print(F("    [FAIL] IRUN "));
    out.print(current_check.run_current_scale);
    out.print(F(" is above the ceiling of "));
    out.println(conv::kCurrentScaleCeiling);
    return false;
  }
  out.println(F("    [ ok ] safe to enable the output stage"));
  out.println();
  return true;
}

// ---------------------------------------------------------------------------
// Gate 3: witness the limit switch before arming it
// ---------------------------------------------------------------------------

// Watches IOIN.REFL_STEP and reports every transition. This settles two things
// at once: whether U1301's output path works on this board at all, and which
// level the fitted switch produces when it is triggered.
//
// It matters on this board specifically. With the connectors empty the two rig
// boards disagreed -- E4654C159F2C2534 read REFL_STEP = 1 and
// E4654C159F4D1334 read 0 -- so neither value is a known-good baseline, and the
// level with a switch attached has to be measured. See
// mouse-joystick-pcb/docs/bringup-log.md, "The two rig boards disagree on
// REFL_STEP and ENCB".
void witnessSwitch(Print &out, uint32_t window_ms) {
  if (!bus_up) {
    out.println(F("Bus is open. Run 'b' first."));
    return;
  }

  out.println(F("[4] limit switch witness"));
  out.print(F("    Watching IOIN.REFL_STEP for "));
  out.print(window_ms / 1000);
  out.println(F(" s. ACTUATE THE SWITCH NOW -- move the joystick onto it by"));
  out.println(F("    hand, or press the switch. Release it before the window"));
  out.println(F("    ends, so its resting level is recorded too."));
  out.println();

  SafeState::setInstrumentationEnabled(true);

  tmc51x0::Registers::Ioin ioin;
  ioin.raw = MotionSetup::readIoin(stepper);
  bool level = ioin.refl_step();
  const bool initial_level = level;

  uint16_t transitions = 0;
  bool saw_high = level;
  bool saw_low = !level;

  const uint32_t started = millis();
  while ((millis() - started) < window_ms) {
    ioin.raw = MotionSetup::readIoin(stepper);
    const bool now = ioin.refl_step();
    if (now != level) {
      ++transitions;
      // A marker on FW_TRIG0, so a capture lines the firmware's view up against
      // LIMIT_SW itself (channels 0 and 15 on v1.0, 16 and 11 on rev 0/1).
      SafeState::setTrigger0(true);
      out.print(F("    t="));
      out.print(millis() - started);
      out.print(F(" ms  REFL_STEP "));
      out.print(level ? F("1") : F("0"));
      out.print(F(" -> "));
      out.println(now ? F("1") : F("0"));
      SafeState::setTrigger0(false);
      level = now;
      if (now) {
        saw_high = true;
      } else {
        saw_low = true;
      }
    }
    delay(5);
  }

  out.println();
  out.print(F("    initial level: "));
  out.println(initial_level ? F("1") : F("0"));
  out.print(F("    final level:   "));
  out.println(level ? F("1") : F("0"));
  out.print(F("    transitions:   "));
  out.println(transitions);

  if (!saw_high || !saw_low) {
    out.println(F("    [FAIL] REFL_STEP never changed. Homing cannot work"));
    out.println(F("      until it does, so stage 3 stops here rather than"));
    out.println(F("      driving the mechanism at a switch that is not"));
    out.println(F("      reporting. Two candidates, and one test separates"));
    out.println(F("      them:"));
    out.println(F("        - the switch is not actually being actuated."));
    out.println(F("        - U1301's OUT joint is open, in which case R1405"));
    out.println(F("          holds LIMIT_SW low and nothing the switch does"));
    out.println(F("          ever reaches the driver."));
    out.println(
        F("      Short J1301 pin 1 to pin 2 by hand and re-run: pin 1"));
    out.println(
        F("      is GND, pin 2 is IN. If REFL_STEP moves for the hand"));
    out.println(F("      short but not for the switch, the board is fine and"));
    out.println(F("      the switch or its harness is not."));
    polarity_measured = false;
    return;
  }

  // The switch is triggered while it is NOT at its resting level, and the
  // resting level is the one it started and ended at with an actuation in
  // between.
  switch_active_high = !initial_level;
  polarity_measured = true;

  out.print(F("    [ ok ] the path works. Triggered level = "));
  out.println(switch_active_high ? F("1 (active high)") : F("0 (active low)"));
  out.print(F("      -> SW_MODE.pol_stop_l = "));
  out.println(switch_active_high ? F("0, non-inverted") : F("1, inverted"));
  if (transitions == 2) {
    out.println(
        F("      Exactly two transitions for one press and release, at"));
    out.println(F("      a 5 ms sample: the MAX6816 is debouncing."));
  } else {
    out.print(F("      "));
    out.print(transitions);
    out.println(F(" transitions. More than two per actuation at a 5 ms"));
    out.println(F("      sample is worth a second look -- either the switch"));
    out.println(
        F("      was actuated more than once, or the debouncer is not"));
    out.println(F("      doing its job."));
  }
  out.println(F("      The MAX6816's debounce delay is added to every homing"));
  out.println(
      F("      approach: the driver learns of the closure late by that"));
  out.println(
      F("      delay times the approach velocity, and no register can"));
  out.println(F("      show it. That is why homing runs at 0.05 rev/s."));
  out.println();
}

// ---------------------------------------------------------------------------
// Motion
// ---------------------------------------------------------------------------

// Drive the mechanism onto the switch and off again, and let that settle the
// polarity and the wiring. Needs the driver enabled, so it comes after the
// current gate rather than before it -- unlike the hand-actuated 'w', which
// works with everything de-energised and is the right first look at a board
// nobody has moved yet.
void witnessByMotion(Print &out) {
  if (!bus_up || !configured) {
    out.println(F("Bus is open. Run 'b' first."));
    return;
  }
  if (!current_check.safe) {
    out.println(F("Current is not verified. Run 'c' first."));
    return;
  }

  out.print(F("Sweeping toward the "));
  out.print(switch_end == mouse_joystick::HomeEnd::kPositive ? F("positive")
                                                             : F("negative"));
  out.print(F(" end in "));
  out.print(conv::kWitnessStepMicrosteps);
  out.println(F(" ustep steps to find the switch..."));

  SafeState::setInstrumentationEnabled(true);
  SafeState::setTrigger1(true);
  const MotionSetup::SwitchWitness w =
      MotionSetup::witnessByMotion(stepper, switch_end, kJogTimeoutMs,
                                   conv::kWitnessSweepTimeoutMs);
  SafeState::setTrigger1(false);

  out.print(F("  resting REFL_STEP: "));
  out.println(w.resting_level ? F("1") : F("0"));
  out.print(F("  travel used:       "));
  out.print(w.travel_used);
  out.print(F(" usteps ("));
  out.print(conv::microstepsToDegrees(w.travel_used), 2);
  out.println(F(" deg)"));

  if (!w.transition_seen) {
    out.println(F("  [FAIL] REFL_STEP never changed during the sweep."));
    out.print(F("    last approach outcome: "));
    out.println(outcomeName(w.approach_outcome));

    // Name the sweep's OWN limits before naming a component. The previous
    // version of this text went straight to "U1301's OUT joint" and was wrong
    // all three times it fired -- the sweep had simply run out of budget with
    // the mechanism parked far from the switch.
    switch (w.stop_reason) {
    case MotionSetup::SwitchWitness::StopReason::kBudgetExpired:
      out.println(F("    reason: THE SWEEP RAN OUT OF TIME, not the switch."));
      out.print(F("    It covered "));
      out.print(conv::microstepsToDegrees(w.travel_used), 1);
      out.print(F(" deg in "));
      out.print(conv::kWitnessSweepTimeoutMs / 1000);
      out.println(F(" s and never got there."));
      out.println(F("    Almost certainly the mechanism is parked too far"));
      out.println(F("    from the switch. Jog toward it with 'j'/'k' while"));
      out.println(F("    watching 'i' for REFL_STEP, then re-run 'W'."));
      out.println(F("    Suspect the board only after that."));
      break;
    case MotionSetup::SwitchWitness::StopReason::kMoveBlocked:
      out.println(F("    reason: a step ended on something other than its"));
      out.println(F("    target, so the mechanism hit an END STOP before the"));
      out.println(F("    switch. The switch is not at this end -- send 'y' to"));
      out.println(F("    sweep the other way."));
      break;
    case MotionSetup::SwitchWitness::StopReason::kTravelBound:
      out.println(F("    reason: the full distance bound was swept without"));
      out.println(F("    the switch reporting. THIS is the case that"));
      out.println(F("    implicates the board:"));
      out.println(F("    short J1301 pin 1 to pin 2 by hand (pin 1 GND,"));
      out.println(F("    pin 2 IN) and watch 'i'. No change there means"));
      out.println(F("    U1301's OUT joint or its harness."));
      break;
    case MotionSetup::SwitchWitness::StopReason::kTransitionSeen:
      break;
    }
    polarity_measured = false;
    return;
  }

  printPosition(out, "  engaged at:        ", w.engage_position);
  out.print(F("  released again:    "));
  out.println(w.released_again ? F("yes") : F("NO"));
  if (w.released_again) {
    printPosition(out, "  released at:       ", w.release_position);
    out.print(F("  switch throw:      "));
    out.print(w.engage_position - w.release_position);
    out.println(F(" usteps between engage and release"));
  }

  switch_active_high = w.active_high;
  polarity_measured = w.transition_seen && w.released_again;

  out.print(F("  [ ok ] triggered level = "));
  out.println(switch_active_high ? F("1 (active high)") : F("0 (active low)"));
  out.print(F("    -> SW_MODE.pol_stop = "));
  out.println(switch_active_high ? F("0, non-inverted") : F("1, inverted"));
  out.println(F("    The path from J1301 through U1301 to TMC REFL works, so"));
  out.println(
      F("    this board is not the one with an open debouncer output."));
  if (!w.released_again) {
    polarity_measured = false;
    out.println(F("  [FAIL] it engaged but never let go on the way back, so"));
    out.println(
        F("    homing is not armed. A switch that latches on is either"));
    out.println(F("    mechanically stuck or being held by the mechanism."));
  }
  out.println();
}

bool readyToMove(Print &out) {
  if (!bus_up || !configured) {
    out.println(F("Bus is open. Run 'b' first."));
    return false;
  }
  if (!current_check.safe) {
    out.println(F("Current is not verified. Run 'c' first."));
    return false;
  }
  return true;
}

#if MJ_BOARD_REV != 0
// Park the angle output at a static level, for the motor-coupling test.
//
// A STATIC setpoint is the point. The intended signal is then pure DC, so any
// AC on the BNC is the motor getting into the analog chain and a scope can be
// AC-coupled at high sensitivity to see it. Driving a live angle while moving
// conflates coupling with the filter's response to real motion, which is a
// different and later question.
//
// What to look for: a chopper-correlated artefact. The AD7616-P's antialias
// filter is first order at ~39 kHz against fixed 30.3 kHz sampling, so
// out-of-band content folds in nearly unattenuated -- and anything like that
// would be far larger than one bit, which makes the ÷2 span question academic
// until it is fixed.
void parkAngleOutput(Print &out, uint16_t code) {
  if (!dac.configured()) {
    out.println(F("Run 'b' first -- the DAC needs the rail and the bus."));
    return;
  }
  if (!dac.setCode(code)) {
    out.println(F("  [FAIL] DAC write refused or failed."));
    return;
  }
  const float dac_volts = dac80501::codeToVolts(code, kDacGainRegister);
  out.print(F("Angle output parked: code "));
  out.print(code);
  out.print(F(" -> DAC "));
  out.print(dac_volts, 6);
  out.print(F(" V -> BNC "));
  out.print(dac_volts * kDacOutputStageGain, 6);
  out.println(F(" V expected"));
  if (code == 0) {
    out.println(F("  Zero scale measures 4.5 mV on this board, not 0 V --"));
    out.println(F("  the OPA197 clamps there. Midscale is the better"));
    out.println(F("  setpoint for a coupling hunt: it sits mid-span, so a"));
    out.println(F("  disturbance in either direction is visible."));
  }
  out.println();
}
#endif

#if MJ_BOARD_REV != 0
// Drive the analog output from the live encoder position at a fixed rate.
//
// Runs until any serial byte arrives. Reports the ACHIEVED rate and the worst
// loop time, because that is the claim the design doc asks for and an assumed
// rate is worth nothing.
void liveAngleOutput(Print &out) {
  if (!bus_up || !configured) {
    out.println(F("Run 'b' first."));
    return;
  }
  if (!dac.configured()) {
    out.println(F("The DAC is not configured. Run 'b'."));
    return;
  }
  if (!homed) {
    // Without a datum the angle is measured from wherever the mechanism
    // happened to be, which is exactly the out-of-band "unhomed" case the
    // design reserves a level for. Refuse rather than emit a plausible lie.
    out.println(F("NOT HOMED. Run 'h' first -- without a datum the angle is"));
    out.println(F("measured from wherever the mechanism was left, and the"));
    out.println(F("output would be a plausible-looking wrong number."));
    return;
  }

  out.print(F("Live angle output at "));
  out.print(1000000.0f / static_cast<float>(kAngleUpdateIntervalUs), 0);
  out.println(F(" Hz. Any key stops it."));
  out.print(F("  mapping: 0 to "));
  out.print(kAngleSpanDegrees, 1);
  out.print(F(" deg of excursion -> "));
  out.print(kAngleOutMinVolts, 3);
  out.print(F(" to "));
  out.print(kAngleOutMaxVolts, 3);
  out.println(F(" V at the BNC"));
  out.print(F("  one encoder count = "));
  out.print(360.0f / static_cast<float>(conv::kEncoderCountsPerRev), 4);
  out.print(F(" deg = "));
  out.print((kAngleOutMaxVolts - kAngleOutMinVolts) /
                kAngleSpanDegrees * 360.0f /
                static_cast<float>(conv::kEncoderCountsPerRev) * 1000.0f,
            2);
  out.println(F(" mV at the BNC"));
  if (dac80501::kCodeMax > 0) {
    out.println(F("  With the driver DISABLED ('E') the motor freewheels and"));
    out.println(F("  the joystick can be moved by hand -- the encoder still"));
    out.println(F("  counts, because it is read by the TMC5130A and does not"));
    out.println(F("  depend on the output stage."));
  }
  out.println();

  while (Serial.available()) {
    Serial.read();
  }

  const float volts_per_degree =
      (kAngleOutMaxVolts - kAngleOutMinVolts) / kAngleSpanDegrees;
  uint32_t updates = 0;
  uint32_t worst_loop_us = 0;
  int32_t min_code = dac80501::kCodeMax;
  int32_t max_code = 0;
  const uint32_t started_us = micros();
  uint32_t next_us = started_us;

  SafeState::setInstrumentationEnabled(true);
  SafeState::setTrigger1(true);

  while (!Serial.available()) {
    // Absolute scheduling, not a delay, so a slow iteration does not push the
    // whole schedule out.
    while (static_cast<int32_t>(micros() - next_us) < 0) {
    }
    const uint32_t loop_start_us = micros();
    next_us += kAngleUpdateIntervalUs;

    const int32_t microsteps = stepper.encoder.readActualPosition();
    // Excursion away from the datum. Position 0 is the switch release edge at
    // the positive end, so travel into the mechanism is negative.
    float excursion_deg = -conv::microstepsToDegrees(microsteps);
    if (excursion_deg < 0.0f) {
      excursion_deg = 0.0f;
    } else if (excursion_deg > kAngleSpanDegrees) {
      excursion_deg = kAngleSpanDegrees;
    }
    const float volts = kAngleOutMinVolts + excursion_deg * volts_per_degree;
    // Divide out the output stage: the DAC has to produce the BNC voltage
    // divided by the amplifier's gain.
    dac.setVolts(volts / kDacOutputStageGain);

    const uint16_t code = dac.lastCode();
    if (code < min_code) {
      min_code = code;
    }
    if (code > max_code) {
      max_code = code;
    }
    ++updates;
    const uint32_t elapsed_us = micros() - loop_start_us;
    if (elapsed_us > worst_loop_us) {
      worst_loop_us = elapsed_us;
    }
  }

  const uint32_t total_us = micros() - started_us;
  SafeState::setTrigger1(false);
  while (Serial.available()) {
    Serial.read();
  }

  out.println(F("Stopped."));
  out.print(F("  updates:        "));
  out.println(updates);
  out.print(F("  elapsed:        "));
  out.print(static_cast<float>(total_us) / 1000.0f, 1);
  out.println(F(" ms"));
  out.print(F("  ACHIEVED rate:  "));
  out.print(static_cast<float>(updates) * 1000000.0f /
                static_cast<float>(total_us),
            1);
  out.println(F(" Hz"));
  out.print(F("  worst loop:     "));
  out.print(worst_loop_us);
  out.print(F(" us of the "));
  out.print(kAngleUpdateIntervalUs);
  out.println(F(" us budget"));
  out.print(F("  code range:     "));
  out.print(min_code);
  out.print(F(" to "));
  out.print(max_code);
  out.print(F("  ("));
  out.print(dac80501::codeToVolts(static_cast<uint16_t>(min_code),
                                  kDacGainRegister) *
                kDacOutputStageGain,
            4);
  out.print(F(" to "));
  out.print(dac80501::codeToVolts(static_cast<uint16_t>(max_code),
                                  kDacGainRegister) *
                kDacOutputStageGain,
            4);
  out.println(F(" V at the BNC)"));
  if (worst_loop_us > kAngleUpdateIntervalUs) {
    out.println(F("  [!!] a loop exceeded its budget, so the schedule"));
    out.println(F("  slipped. The achieved rate above is the real one."));
  }
  out.println();
}
#endif

void jog(Print &out, int32_t microsteps) {
  if (!readyToMove(out)) {
    return;
  }
  out.print(F("Jogging "));
  out.print(microsteps);
  out.print(F(" usteps ("));
  out.print(conv::microstepsToDegrees(microsteps), 2);
  out.println(F(" deg)..."));

  SafeState::setInstrumentationEnabled(true);
  SafeState::setTrigger1(true);
  MotionSetup::setJogVelocity(stepper);
  const MoveReport r = MotionSetup::moveBy(stepper, microsteps, kJogTimeoutMs);
  SafeState::setTrigger1(false);

  printMoveReport(out, r);
  if (r.outcome == MoveOutcome::kNotFollowing) {
    out.println(
        F("      The ramp advanced and the encoder did not follow, so"));
    out.println(
        F("      the move stopped itself. Most likely a mechanical end"));
    out.println(
        F("      stop, which at this current is harmless -- the motor"));
    out.println(F("      simply stalls. Jog the other way. If it happens in"));
    out.println(F("      BOTH directions from the same spot, suspect the"));
    out.println(F("      encoder or the coupling rather than the mechanism."));
  }
  out.println();
}

// Jog out and back, and report what the encoder made of it. Returning to the
// start leaves the mechanism where it was found, and the round trip exposes
// backlash in the gear pair as a residual.
void encoderCheck(Print &out) {
  if (!readyToMove(out)) {
    return;
  }
  // Out AWAY from the limit switch and back, not toward it. A leg that ends on
  // the switch is truncated, and a truncated leg makes the ratio and the
  // residual meaningless -- the first run of this test reported 0.786 and
  // blamed the encoder's CPR when what had actually happened was the joystick
  // reaching the end of its travel.
  const int32_t away =
      (switch_end == mouse_joystick::HomeEnd::kPositive) ? -1 : 1;
  // One jog step each way. The rig's travel is only about three jog steps end
  // to end, so a longer round trip cannot fit inside it from an arbitrary
  // starting position.
  const int32_t distance = away * conv::kJogMicrosteps;

  out.print(F("Encoder check: "));
  out.print(distance);
  out.print(F(" usteps out (away from the switch, at the "));
  out.print(switch_end == mouse_joystick::HomeEnd::kPositive ? F("positive")
                                                             : F("negative"));
  out.println(F(" end), then back."));
  out.println(F("  Run this from mid-travel, or straight after 'h'. It jogs"));
  out.println(F("    toward the FAR hard stop first, so from rest against"));
  out.println(F("    that stop it fails on the mechanism, not the encoder."));
  out.print(F("  Expecting "));
  out.print(static_cast<float>(distance < 0 ? -distance : distance) /
                conv::kMicrostepsPerEncoderCount,
            1);
  out.print(F(" quadrature counts each way at "));
  out.print(conv::kEncoderCpr);
  out.println(F(" CPR"));
  out.println();

  SafeState::setInstrumentationEnabled(true);
  SafeState::setTrigger1(true);
  MotionSetup::setJogVelocity(stepper);
  const int32_t start_encoder = stepper.encoder.readActualPosition();
  const MoveReport out_report =
      MotionSetup::moveBy(stepper, distance, kJogTimeoutMs);
  out.println(F("  outbound:"));
  printMoveReport(out, out_report);

  const MoveReport back_report =
      MotionSetup::moveBy(stepper, -distance, kJogTimeoutMs);
  out.println(F("  return:"));
  printMoveReport(out, back_report);
  SafeState::setTrigger1(false);

  const int32_t residual = stepper.encoder.readActualPosition() - start_encoder;

  out.println();
  out.print(F("  encoder residual after the round trip: "));
  out.print(residual);
  out.print(F(" usteps ("));
  out.print(static_cast<float>(residual) / conv::kMicrostepsPerEncoderCount, 1);
  out.println(F(" counts)"));
  out.println(
      F("    A few counts is quantisation plus gear backlash. Tens of"));
  out.println(F("    counts is real lost motion, and the places to look are"));
  out.println(F("    the encoder hub and the gear mesh."));

  const float ratio = out_report.follow_ratio;
  const float magnitude = (ratio < 0.0f) ? -ratio : ratio;

  // A leg that did not simply reach its target measures the mechanism, not the
  // encoder. Say so instead of computing a ratio nobody should read.
  const bool legs_clean = out_report.outcome == MoveOutcome::kTargetReached &&
                          back_report.outcome == MoveOutcome::kTargetReached;
  if (!legs_clean) {
    out.println();
    out.println(F("  [inconclusive] a leg ended on something other than its"));
    out.println(
        F("    target, so neither the ratio nor the residual above is"));
    out.println(F("    an encoder measurement -- they describe where the"));
    out.println(
        F("    mechanism ran out. Jog clear of the end of travel with"));
    out.println(F("    'j'/'k' and run 'g' again from the middle."));
    out.println();
    return;
  }

  if (ratio < 0.0f) {
    out.println();
    out.println(F("  [note] the encoder counts opposite to the commanded"));
    out.println(
        F("    direction. That is a mounting sense, not a fault: give"));
    out.println(F("    ENC_CONST a negative integer part to match, through"));
    out.println(F("    TMC51X0's EncoderParameters. Homing works either way;"));
    out.println(F("    a position loop would not."));
  }

  if (magnitude > 0.9f && magnitude < 1.1f) {
    out.println();
    out.print(F("  [ ok ] the encoder tracks the ramp to within "));
    out.print((magnitude > 1.0f ? magnitude - 1.0f : 1.0f - magnitude) * 100.0f,
              1);
    out.println(F("%, which"));
    out.print(F("    confirms "));
    out.print(conv::kEncoderCpr);
    out.println(F(" CPR against 51200 usteps/rev. A ratio near 2 or"));
    out.println(F("    0.5 would mean the fitted encoder is not the CPR this"));
    out.println(F("    firmware was told about."));
  } else if (magnitude < 0.1f) {
    out.println();
    out.println(F("  [FAIL] the encoder did not move. In order: is ENC_CONST"));
    out.println(
        F("    written (it resets to zero)? Is the encoder harness on"));
    out.println(F("    J1103 the right way round -- pin 1 GND, pin 3 ENC_A,"));
    out.println(F("    pin 4 +5V_SW, pin 5 ENC_B? Does +5V_SW reach it? Are"));
    out.println(F("    ENCA/ENCB moving in IOIN while the shaft turns?"));
  } else {
    // Distinguish a hard stop from an encoder problem before blaming the
    // encoder. The `legs_clean` guard above does NOT catch a stall: driving
    // into an end stop open-loop still "reaches target" as far as the ramp
    // generator is concerned, because it only counts the steps it issued. The
    // encoder is the sole witness, and the signature is asymmetry -- both legs
    // cover the same commanded distance, so if the encoder saw substantially
    // less on one of them the mechanism was obstructed in that direction and
    // nothing here measures the encoder.
    //
    // Measured on board E4654C159F171E34, 2026-09-09, starting against the far
    // stop: outbound ratio 0.550 with 704 usteps seen, return ratio 0.920 with
    // 1177 seen, residual +473 = exactly the difference. The old text sent the
    // reader to the CPR, the RC corner and the coupling; all three were fine,
    // and two plain jogs the other way returned 1.0000.
    const int32_t out_travel = out_report.end_encoder - out_report.start_encoder;
    const int32_t back_travel =
        back_report.end_encoder - back_report.start_encoder;
    const int32_t out_abs = out_travel < 0 ? -out_travel : out_travel;
    const int32_t back_abs = back_travel < 0 ? -back_travel : back_travel;
    const mouse_joystick::RoundTripVerdict verdict =
        mouse_joystick::classifyRoundTrip(out_travel, back_travel);

    out.println();
    if (verdict != mouse_joystick::RoundTripVerdict::kSymmetric) {
      const bool outbound_short =
          verdict == mouse_joystick::RoundTripVerdict::kOutboundObstructed;
      out.println(F("  [FAIL] the two legs disagree, so this is the"));
      out.println(F("    MECHANISM and not the encoder."));
      out.print(F("    outbound saw "));
      out.print(out_abs);
      out.print(F(" usteps, return saw "));
      out.print(back_abs);
      out.println(F(" usteps, for the"));
      out.println(F("    same commanded distance. A leg that comes up short"));
      out.println(F("    while the other runs clean means the mechanism hit"));
      out.println(F("    an END STOP -- and driving into one open-loop still"));
      out.println(F("    reports 'target reached', because the ramp generator"));
      out.println(F("    only counts the steps it issued."));
      out.print(F("    The obstruction is in the "));
      if (outbound_short) {
        out.print(switch_end == mouse_joystick::HomeEnd::kPositive
                      ? F("NEGATIVE")
                      : F("POSITIVE"));
        out.println(F(" direction,"));
        out.println(F("    away from the switch: the far hard stop. 'g' jogs"));
        out.println(F("    that way first, so running it from rest near that"));
        out.println(F("    stop fails exactly like this."));
      } else {
        out.print(switch_end == mouse_joystick::HomeEnd::kPositive
                      ? F("POSITIVE")
                      : F("NEGATIVE"));
        out.println(F(" direction,"));
        out.println(F("    toward the switch."));
      }
      out.println(F("    The residual above is that difference, not lost"));
      out.println(F("    motion: the motor gained open-loop position while"));
      out.println(F("    the mechanism was blocked."));
      out.println();
      out.println(F("    Do this: 'h' to home, or jog clear with 'j'/'k',"));
      out.println(F("    then run 'g' again from mid-travel. Only if it"));
      out.println(F("    still fails SYMMETRICALLY is the encoder implicated."));
    } else {
      out.print(F("  [FAIL] follow ratio "));
      out.print(magnitude, 3);
      out.println(F(" is neither ~1 nor ~0, and both"));
      out.println(F("    legs agree, so the mechanism moved freely and this"));
      out.println(F("    really is the encoder."));
      out.println(F("    A clean 2.0 or 0.5 means the CPR is wrong. Anything"));
      out.println(F("    else means lost counts: check the RC corner against"));
      out.println(F("    the edge rate, and the coupling for slip."));
    }
  }
  out.println();
}

// ---------------------------------------------------------------------------
// Homing
// ---------------------------------------------------------------------------

void printHomeReport(Print &out, const HomeReport &h) {
  out.println(F("  approach:"));
  printMoveReport(out, h.approach);

  switch (h.outcome) {
  case HomeOutcome::kHomed:
    printPosition(out, "  stage 1 XLATCH, closing edge: ", h.coarse_latch);
    printPosition(out, "  stage 1 settled:              ", h.settled_position);
    out.print(F("    overshoot past the closing latch: "));
    out.print(h.overshoot);
    out.println(F(" usteps"));
    out.println(F("      A hard stop latches and stops at the same position,"));
    out.println(F("      so zero here is correct. It does NOT include the"));
    out.println(
        F("      debounce delay, which elapses before the driver sees"));
    out.println(F("      anything -- that is what stage 2 is for."));
    printPosition(out, "  stage 2 XLATCH, release edge: ", h.fine_latch);
    out.print(F("    creep travelled "));
    out.print(h.creep_distance);
    out.print(F(" usteps at "));
    out.print(conv::kHomeCreepRevsPerSecond, 3);
    out.println(F(" rev/s"));
    out.print(F("    closing to release: "));
    out.print(h.coarse_to_fine);
    out.print(F(" usteps ("));
    out.print(conv::microstepsToDegrees(h.coarse_to_fine), 3);
    out.println(F(" deg)"));
    out.println(F("      That is the approach's debounce overshoot plus the"));
    out.println(
        F("      switch's own throw. It grows with approach velocity,"));
    out.println(F("      which is why the release edge is the datum and the"));
    out.println(F("      closing edge is not."));
    out.print(F("  backed off to "));
    out.print(h.backoff_position);
    out.print(F(" usteps; switch "));
    out.println(h.released_after_backoff ? F("released") : F("STILL ACTIVE"));
    if (!h.released_after_backoff) {
      out.println(F("    [warn] a long-throw switch, or too small a back-off"));
      out.println(F("    for this mechanism."));
    }
    out.println(F("  [ ok ] homed. Position 0 is the RELEASE edge, taken at"));
    out.print(F("  the creep velocity, and the encoder was zeroed there too."));
    out.println();
    break;
  case HomeOutcome::kReleaseNeverLatched:
    out.println(F("  [FAIL] the fast approach found the switch but the creep"));
    out.print(F("    never left it within "));
    out.print(conv::kHomeCreepBoundMicrosteps);
    out.println(F(" usteps."));
    out.print(F("    Creep travelled "));
    out.print(h.creep_distance);
    out.println(F(" usteps."));
    out.println(F("    Either the approach overshot further than the creep"));
    out.println(F("    bound allows -- lower the approach velocity or raise"));
    out.println(F("    the bound -- or the switch is not releasing."));
    break;
  case HomeOutcome::kSwitchNeverAsserted:
    out.print(F("  [FAIL] travelled the whole "));
    out.print(conv::kHomeTravelBoundMicrosteps);
    out.println(F(" ustep bound without the"));
    out.println(
        F("    switch asserting. That bound is two motor revolutions,"));
    out.println(F("    far beyond the rig's travel, so this is a switch that"));
    out.println(F("    never reported rather than an approach that was slow."));
    break;
  case HomeOutcome::kMechanicalStopFirst:
    out.println(F("  [FAIL] the mechanism stopped before the switch did. The"));
    out.println(
        F("    encoder stopped following while the ramp kept going, so"));
    out.println(F("    a mechanical end stop was reached ahead of the limit"));
    out.println(F("    switch -- or the switch is at the other end of travel"));
    out.println(F("    and homing should be going positive. Nothing was"));
    out.println(F("    forced: the move ended itself."));
    break;
  case HomeOutcome::kSwitchStuckActive:
    out.println(F("  [FAIL] the switch still reads active after backing off."));
    out.println(F("    Either the back-off direction is wrong -- homing"));
    out.println(F("    assumes the switch is at the NEGATIVE end -- or the"));
    out.println(F("    switch is stuck. Jog with 'j'/'k' and watch 'p'."));
    break;
  case HomeOutcome::kDriverFault:
    out.println(F("  [FAIL] the driver faulted during the approach."));
    break;
  case HomeOutcome::kTimedOut:
    out.println(F("  [FAIL] the approach timed out."));
    break;
  }
  out.print(F("  total: "));
  out.print(h.elapsed_ms);
  out.println(F(" ms"));
}

bool runHome(Print &out) {
  if (!readyToMove(out)) {
    return false;
  }
  if (!polarity_measured) {
    out.println(F("Switch polarity has not been measured. Run 'w' first."));
    out.println(F("  Arming the stop at a guessed polarity can read an"));
    out.println(F("  untriggered switch as a permanent stop, which looks"));
    out.println(F("  exactly like a dead motor."));
    return false;
  }

  out.print(F("Homing toward the "));
  out.print(switch_end == mouse_joystick::HomeEnd::kPositive ? F("POSITIVE")
                                                             : F("NEGATIVE"));
  out.print(F(" end, switch active "));
  out.print(switch_active_high ? F("high") : F("low"));
  out.print(F(". Approach "));
  out.print(conv::kHomeApproachRevsPerSecond, 3);
  out.print(F(" rev/s, then creep off at "));
  out.print(conv::kHomeCreepRevsPerSecond, 3);
  out.println(F(" rev/s..."));

  SafeState::setInstrumentationEnabled(true);
  SafeState::setTrigger1(true);
  const HomeReport h = MotionSetup::homeToSwitch(stepper, switch_active_high,
                                                 switch_end, kHomeTimeoutMs);
  SafeState::setTrigger1(false);

  printHomeReport(out, h);
  out.println();
  return h.outcome == HomeOutcome::kHomed;
}

// Two approaches at two velocities, solved for the debounce delay. This is the
// number that justifies the two-stage home: it is exactly the position error a
// single fast approach bakes into the datum.
void measureSwitchDelay(Print &out) {
  if (!readyToMove(out)) {
    return;
  }
  if (!polarity_measured) {
    out.println(F("Run 'w' or 'W' first."));
    return;
  }

  out.println(F("Measuring the switch's reporting delay: one approach at"));
  out.print(F("  "));
  out.print(conv::kHomeApproachRevsPerSecond, 3);
  out.print(F(" rev/s and one at "));
  out.print(conv::kHomeCreepRevsPerSecond, 3);
  out.println(F(" rev/s, both latched by the chip"));
  out.println(F("  at the closing edge, in one position frame."));
  out.println();

  SafeState::setInstrumentationEnabled(true);
  SafeState::setTrigger1(true);
  const mouse_joystick::SwitchDelay d = MotionSetup::measureSwitchDelay(
      stepper, switch_active_high, switch_end, kHomeTimeoutMs);
  SafeState::setTrigger1(false);

  if (!d.valid) {
    out.println(F("  [FAIL] one of the two approaches did not end on the"));
    out.println(F("  switch with a latch, so there is nothing to subtract."));
    return;
  }

  out.print(F("  fast latch: "));
  out.print(d.latch_fast);
  out.print(F(" usteps at "));
  out.print(d.velocity_fast, 0);
  out.println(F(" usteps/s"));
  out.print(F("  slow latch: "));
  out.print(d.latch_slow);
  out.print(F(" usteps at "));
  out.print(d.velocity_slow, 0);
  out.println(F(" usteps/s"));
  out.print(F("  difference: "));
  out.print(d.latch_fast - d.latch_slow);
  out.println(F(" usteps"));
  out.print(F("  -> reporting delay = "));
  out.print(d.delay_ms, 2);
  out.println(F(" ms"));
  out.println(
      F("    Debounce plus signal path. At the approach velocity that"));
  out.print(F("    is "));
  out.print(d.delay_ms * d.velocity_fast / 1000.0f, 0);
  out.print(F(" usteps of overshoot, and at the creep velocity "));
  out.print(d.delay_ms * d.velocity_slow / 1000.0f, 1);
  out.println(F(" usteps."));
  out.println(F("    The ratio of those two is what the second homing stage"));
  out.println(F("    buys. Check the creep bound exceeds the first figure."));
  out.println();
}

void homeRepeatability(Print &out, uint8_t runs) {
  if (!readyToMove(out) || !polarity_measured) {
    out.println(F("Run 'b', 'c' and 'w' first."));
    return;
  }

  out.print(F("Homing repeatability, "));
  out.print(runs);
  out.println(F(" runs. Each run re-homes from the previous back-off."));
  out.println(F("One throwaway home runs first and is NOT counted: the first"));
  out.println(F("home of a series starts from wherever the last command left"));
  out.println(F("the mechanism, so including it measures approach history"));
  out.println(F("rather than repeatability. Measured 2026-09-02: with the"));
  out.println(F("first run counted, one series read 5 counts and another 2,"));
  out.println(F("while the settled runs within each agreed to about 1."));
  out.println(F("Every run re-zeroes at its own switch point, so the"));
  out.println(
      F("run-to-run scatter shows up as spread in the encoder reading"));
  out.println(F("at the moment of the latch."));
  out.println();

  int32_t first_latch = 0;
  int32_t min_delta = 0;
  int32_t max_delta = 0;
  uint8_t succeeded = 0;

  const HomeReport settle = MotionSetup::homeToSwitch(
      stepper, switch_active_high, switch_end, kHomeTimeoutMs);
  if (settle.outcome != HomeOutcome::kHomed) {
    out.println(F("  the throwaway home did not succeed:"));
    printHomeReport(out, settle);
    return;
  }
  out.println(F("  throwaway home done; starting the counted series."));
  out.println();
  delay(500);

  for (uint8_t i = 0; i < runs; ++i) {
    out.print(F("run "));
    out.print(i + 1);
    out.println(F(":"));
    const HomeReport h = MotionSetup::homeToSwitch(stepper, switch_active_high,
                                                   switch_end, kHomeTimeoutMs);
    if (h.outcome != HomeOutcome::kHomed) {
      printHomeReport(out, h);
      out.println(F("  stopping the series: a run did not home."));
      break;
    }

    // ENC_LATCH is the encoder's reading at the switch event, taken in the
    // frame established by the previous run's zero. Its scatter across runs is
    // the repeatability, and it is measured by the encoder rather than by the
    // ramp generator that commanded the move.
    const int32_t latch = h.fine_encoder;
    if (succeeded == 0) {
      first_latch = latch;
    } else {
      const int32_t delta = latch - first_latch;
      if (delta < min_delta) {
        min_delta = delta;
      }
      if (delta > max_delta) {
        max_delta = delta;
      }
    }
    ++succeeded;
    out.print(F("  release ENC_LATCH "));
    out.print(latch);
    out.print(F(" usteps, closing-to-release "));
    out.print(h.coarse_to_fine);
    out.print(F(" usteps, "));
    out.print(h.elapsed_ms);
    out.println(F(" ms"));
    delay(500);
  }

  out.println();
  out.print(F("succeeded: "));
  out.print(succeeded);
  out.print(F(" of "));
  out.println(runs);
  if (succeeded >= 2) {
    const int32_t spread = max_delta - min_delta;
    out.print(F("spread: "));
    out.print(spread);
    out.print(F(" usteps ("));
    out.print(conv::microstepsToDegrees(spread), 3);
    out.print(F(" deg, "));
    out.print(static_cast<float>(spread) / conv::kMicrostepsPerEncoderCount, 1);
    out.println(F(" encoder counts)"));
    out.println(
        F("  Record this in mouse-joystick-pcb/docs/bringup-log.md"));
    out.println(F("  under Homing, WITH the homing velocity: the MAX6816's"));
    out.println(F("  debounce delay scales the offset with speed, so a"));
    out.println(F("  repeatability figure without a velocity means nothing."));
  }
  out.println();
}

// ---------------------------------------------------------------------------
// The whole sequence
// ---------------------------------------------------------------------------

void runSequence(Print &out) {
  out.println();
  out.println(
      F("================================================================"));
  out.println(F(" mouse-joystick-pcb  stage 3: motion, encoder, homing"));
  out.println(
      F("================================================================"));
  out.println(F(" Energises 36 V and CAN turn the motor. The joystick will"));
  out.println(F(" move. Make sure nothing is in its way."));
  out.println();

  if (!bringUpBus(out)) {
    PowerSequence::powerDown();
    return;
  }
  if (!applyCurrent(out)) {
    MotionSetup::disableDriver(stepper);
    return;
  }

  witnessSwitch(out, kWitnessWindowMs);
  if (!polarity_measured) {
    out.println(F("Stopping before any motion: the limit switch has to be"));
    out.println(F("witnessed before it can be trusted to stop a move."));
    return;
  }

  out.println(F("[5] enabling the output stage"));
  if (!MotionSetup::enableDriver(stepper, current_check)) {
    out.println(F("    [FAIL] refused: the current check did not pass."));
    return;
  }
  out.print(F("    [ ok ] ~{TMC_DRV_EN} low, IRUN "));
  out.print(current_check.run_current_scale);
  out.print(F(" = "));
  out.print(current_check.run_amps_rms, 3);
  out.println(F(" A RMS"));
  delay(100);
  printStatus(out);
  out.println();

  out.println(F("[6] first motion, one jog step each way"));
  jog(out, conv::kJogMicrosteps);
  jog(out, -conv::kJogMicrosteps);

  out.println(F("[7] encoder"));
  encoderCheck(out);

  out.println(F("[8] homing"));
  homed = runHome(out);

  out.println(
      F("----------------------------------------------------------------"));
  if (homed) {
    out.println(
        F(" Stage 3 passed. The rail is up, the driver is enabled and"));
    out.println(F(" position 0 is the switch point. 'j'/'k' to jog, 'H' for"));
    out.println(F(" repeatability, 'x' to power down."));
  } else {
    out.println(F(" Motion and the encoder got as far as the report above;"));
    out.println(F(" homing did not pass. The driver is still enabled. 'x'"));
    out.println(F(" powers everything down."));
  }
  out.println(
      F("----------------------------------------------------------------"));
  out.println();
}

// ---------------------------------------------------------------------------

void setup() {
  SafeState::apply();

  Serial.begin(kSerialBaudRate);
  const uint32_t deadline = millis() + kSerialWaitMs;
  while (!Serial && millis() < deadline) {
    delay(10);
  }
  delay(100);

  // Only print if a host is actually attached. arduino-pico's SerialUSB::write
  // spins until the host drains the endpoint, and a kilobyte of banner against
  // a 256-byte CDC FIFO parks the sketch inside setup() when nothing is
  // listening. That happened on 2026-09-02 and cost a physical replug, because
  // the 1200-baud touch needs the DTR request that was itself stalling.
  if (!Serial) {
    return;
  }

  Serial.println();
  Serial.println(F("mouse-joystick-pcb stage 3, idle and powered down."));
  Serial.println(F("36 V on J201, plus motor, encoder and limit switch."));
  Serial.println(
      F("Send 's' for the whole sequence, or 'b' to gate manually."));
  Serial.println();
  printMenu(Serial);
}

void loop() {
  if (!Serial.available()) {
    delay(10);
    return;
  }

  const int command = Serial.read();
  switch (command) {
  case '?':
    printMenu(Serial);
    break;
  case 's':
    runSequence(Serial);
    break;
  case 'b':
    bringUpBus(Serial);
    break;
  case 'c':
    applyCurrent(Serial);
    break;
  case 'w':
    witnessSwitch(Serial, kWitnessWindowMs);
    break;
  case 'W':
    witnessByMotion(Serial);
    break;
  case 'e':
    if (!configured) {
      Serial.println(F("Not configured. Run 'b' first."));
      break;
    }
    if (MotionSetup::enableDriver(stepper, current_check)) {
      Serial.print(F("Driver ENABLED, IRUN "));
      Serial.print(current_check.run_current_scale);
      Serial.print(F(" = "));
      Serial.print(current_check.run_amps_rms, 3);
      Serial.println(F(" A RMS."));
    } else {
      Serial.println(F("Refused. Run 'c' and check vsense reads back as 1."));
    }
    break;
  case 'E':
    MotionSetup::disableDriver(stepper);
    Serial.println(F("Driver disabled, ~{TMC_DRV_EN} high."));
    break;
  case 'j':
    jog(Serial, conv::kJogMicrosteps);
    break;
  case 'k':
    jog(Serial, -conv::kJogMicrosteps);
    break;
  case 'J':
    jog(Serial, conv::kJogMicrosteps * 10);
    break;
  case 'K':
    jog(Serial, -conv::kJogMicrosteps * 10);
    break;
  case 'g':
    encoderCheck(Serial);
    break;
  case 'h':
    homed = runHome(Serial);
    break;
  case 'H':
    homeRepeatability(Serial, 5);
    break;
  case 't':
    measureSwitchDelay(Serial);
    break;
  case 'n': {
    Serial.print(F("Run current scale 0.."));
    Serial.print(conv::kCurrentScaleCeiling);
    Serial.println(F(", digits then Enter:"));
    uint8_t value = 0;
    const uint32_t entry_deadline = millis() + 10000;
    while (millis() < entry_deadline) {
      if (!Serial.available()) {
        delay(5);
        continue;
      }
      const int c = Serial.read();
      if (c == '\r' || c == '\n') {
        break;
      }
      if (c < '0' || c > '9') {
        continue;
      }
      value = static_cast<uint8_t>(value * 10 + (c - '0'));
    }
    if (value > conv::kCurrentScaleCeiling) {
      Serial.print(F("Refused: above the ceiling of "));
      Serial.println(conv::kCurrentScaleCeiling);
      break;
    }
    run_current_scale = value;
    Serial.print(F("Run current scale set to "));
    Serial.print(run_current_scale);
    Serial.print(F(" = "));
    Serial.print(conv::currentScaleToAmpsRms(run_current_scale, true), 3);
    Serial.println(F(" A RMS."));
    if (configured) {
      // Rewriting the parameters clears the verification, because the current
      // that was verified is no longer the current that is set.
      MotionSetup::configure(stepper, motionConfig());
      current_check.safe = false;
      Serial.println(F("Re-run 'c' to write vsense and verify it again."));
    }
    break;
  }
  case 'o':
    encoder_inverted = !encoder_inverted;
    Serial.print(F("Encoder sense: "));
    Serial.println(encoder_inverted ? F("INVERTED (ENC_CONST negative)")
                                    : F("normal"));
    if (bus_up) {
      MotionSetup::setEncoderInverted(stepper, encoder_inverted);
      Serial.println(
          F("Applied and X_ENC zeroed. ENC_CONST is write-only, so"));
      Serial.println(
          F("the check is a jog's follow ratio coming out positive."));
    }
    break;
  case 'm':
    motor_reversed = !motor_reversed;
    Serial.print(F("Motor direction: "));
    Serial.println(motor_reversed ? F("REVERSED (GCONF.shaft)") : F("forward"));
    if (bus_up) {
      MotionSetup::configure(stepper, motionConfig());
      current_check.safe = false;
      Serial.println(F("Reapplied. Re-run 'c' to verify vsense again."));
      Serial.println(
          F("Reversing the motor also reverses which end of travel"));
      Serial.println(F("the switch is at, in motor coordinates -- check 'y'."));
    }
    break;
  case 'y':
    switch_end = (switch_end == mouse_joystick::HomeEnd::kPositive)
                     ? mouse_joystick::HomeEnd::kNegative
                     : mouse_joystick::HomeEnd::kPositive;
    Serial.print(F("Limit switch is at the "));
    Serial.print(switch_end == mouse_joystick::HomeEnd::kPositive
                     ? F("POSITIVE")
                     : F("NEGATIVE"));
    Serial.println(F(" end of travel."));
    Serial.println(F("  This sets SW_MODE.swap_lr. On this board the only"));
    Serial.println(F("  switch is on REFL, which stops NEGATIVE motion, so a"));
    Serial.println(F("  switch at the positive end needs swap_lr to stop the"));
    Serial.println(F("  direction that drives into it."));
    if (bus_up) {
      MotionSetup::armStop(stepper, /*armed=*/false, switch_active_high,
                           switch_end, mouse_joystick::LatchEdge::kActive);
      Serial.println(F("  Applied, stop still disarmed."));
    }
    break;
  case 'p':
    printStatus(Serial);
    break;
  case 'd':
    printRampConfig(Serial);
    break;
  case 'i': {
    if (!bus_up) {
      Serial.println(F("Bus is open. Run 'b' first."));
      break;
    }
    tmc51x0::Registers::Ioin ioin;
    ioin.raw = MotionSetup::readIoin(stepper);
    Serial.print(F("IOIN = "));
    printHex32(Serial, ioin.raw);
    Serial.print(F("  VERSION=0x"));
    Serial.print(ioin.version(), HEX);
    Serial.print(F(" REFL_STEP="));
    Serial.print(ioin.refl_step() ? F("1") : F("0"));
    Serial.print(F(" DRV_ENN="));
    Serial.print(ioin.drv_enn() ? F("1") : F("0"));
    Serial.print(F(" ENCA="));
    Serial.print(ioin.enca_dcin_cfg5() ? F("1") : F("0"));
    Serial.print(F(" ENCB="));
    Serial.println(ioin.encb_dcen_cfg4() ? F("1") : F("0"));
    break;
  }
  case 'v': {
    const RailState rail = PowerSequence::readRail();
    Serial.print(F("+3.3V_SW: "));
    Serial.print(rail.rail_volts, 4);
    Serial.print(F(" V -- "));
    Serial.println(rail.up ? F("live") : (rail.off ? F("off") : F("BETWEEN")));
    break;
  }
#if MJ_BOARD_REV != 0
  case 'A':
    parkAngleOutput(Serial, 0x8000);
    break;
  case 'Z':
    parkAngleOutput(Serial, 0x0000);
    break;
  case 'F':
    parkAngleOutput(Serial, 0xFFFF);
    break;
  case 'L':
    liveAngleOutput(Serial);
    break;
#endif
  case 'x':
    MotionSetup::endSpi(stepper);
    PowerSequence::powerDown();
    bus_up = false;
    configured = false;
    homed = false;
    current_check.safe = false;
#if MJ_BOARD_REV != 0
    // The DAC just lost VDD, so its GAIN is back to a 5.0 V full scale. Tell
    // the driver, or it keeps accepting data writes against a span that no
    // longer exists. invalidate() rather than softReset() because the latter
    // would send a frame into an unpowered part.
    dac.invalidate();
#endif
    Serial.println(F("Powered down: driver disabled, bus open, HV_EN low."));
    break;
  case 'I':
    SelfTest::emitIdentityBeacon(Serial, 3);
    break;
  case 'B':
    MotionSetup::disableDriver(stepper);
    PowerSequence::powerDown();
    Serial.println(F("Powered down. Rebooting into BOOTSEL."));
    Serial.flush();
    delay(100);
    rp2040.rebootToBootloader();
    break;
  case 'R':
    MotionSetup::disableDriver(stepper);
    PowerSequence::powerDown();
    Serial.println(F("Powered down. Rebooting."));
    Serial.flush();
    delay(100);
    rp2040.reboot();
    break;
  case '\r':
  case '\n':
    break;
  default:
    Serial.print(F("unknown command: '"));
    Serial.print(static_cast<char>(command));
    Serial.println(F("'. Send ? for the menu."));
    break;
  }
}
