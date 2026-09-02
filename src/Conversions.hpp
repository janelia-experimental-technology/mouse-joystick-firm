// Scaling between raw ADC counts and board-level quantities.
//
// Header-only and free of Arduino dependencies on purpose, so the arithmetic
// can be tested on the host under `pixi run test` without hardware.

#pragma once

#include <stdint.h>

namespace mouse_joystick {
namespace conversions {

// The RP2040's ADC reference is ADC_AVDD, which on this board is tied straight
// to +3.3V_MCU (U501 pad 43). There is no separate reference and no filter, so
// absolute accuracy is only as good as that rail.
constexpr float kAdcReferenceVolts = 3.3f;

// The RP2040's native ADC width. The Arduino core defaults to 10 bits, so
// every read path must set this explicitly -- a reader that forgets divides by
// the wrong full scale and reports a quarter of the real voltage. These two
// constants must agree; a host test asserts it.
constexpr uint8_t kAdcResolutionBits = 12;
constexpr uint16_t kAdcFullScale = 4095;

// R401 1k / R402 1k inside the mcu sheet.
constexpr float kSwRailDividerRatio = 2.0f;

// R1907 on the TPS1HTC100-Q1's SNS output.
constexpr float kHvSenseResistorOhms = 249.0f;

// Nominal switched 3.3 V rail, for judging a rail-sense reading.
constexpr float kSwRailNominalVolts = 3.3f;

constexpr float countsToVolts(uint16_t counts,
                              uint16_t full_scale = kAdcFullScale,
                              float reference_volts = kAdcReferenceVolts) {
  return (full_scale == 0) ? 0.0f
                           : (static_cast<float>(counts) * reference_volts /
                              static_cast<float>(full_scale));
}

// Raw +3.3V_SW, undoing the divider.
constexpr float swRailVolts(uint16_t counts,
                            uint16_t full_scale = kAdcFullScale,
                            float reference_volts = kAdcReferenceVolts) {
  return countsToVolts(counts, full_scale, reference_volts) *
         kSwRailDividerRatio;
}

// TPS1HTC100-Q1 current-sense ratio, `KSNS` = 800 A/A, constant across
// temperature and supply voltage (datasheet Section 5.5). With R207 = 249 ohm
// that makes the sense chain 0.311 V/A, so the ADC's 0.806 mV LSB is 2.6 mA of
// load current.
constexpr float kHvSenseRatio = 800.0f;

// The switch reports faults on the same pin by forcing `ISNSFLT`, 5.2-6.4 mA,
// instead of KSNS-divided load current. Into 249 ohm that is 1.29-1.59 V, and
// measured parts sit a little above the datasheet maximum (6.3-6.5 mA has been
// seen on this board). Anything above this threshold is the fault current
// source, NOT a load reading -- converting it to amps yields a fictitious 4 A.
constexpr float kHvSenseFaultVolts = 1.20f;

// Measured quiet-state floor on this board: 18-22 counts with HV_EN low and no
// load at all, i.e. ADC offset plus leakage. One count is 2.6 mA of load
// current, so at the ~10-15 mA idle draw the sense chain resolves nothing.
// Printing a reading this small as amps is exactly how HV_SNS has been
// misread before -- twice.
constexpr uint16_t kHvSenseFloorCounts = 40;

// Voltage across R1907. Converting this to load current needs the
// TPS1HTC100-Q1's current-sense ratio, which this firmware does not use yet:
// stage 1 runs with the 36 V input absent, so the only expected reading is
// zero. Add the ratio from `tps1htc100-q1.pdf` when a stage that actually
// energises the rail needs amps rather than volts.
constexpr float hvSenseVolts(uint16_t counts,
                             uint16_t full_scale = kAdcFullScale,
                             float reference_volts = kAdcReferenceVolts) {
  return countsToVolts(counts, full_scale, reference_volts);
}

// Load current through the high-side switch, from the SNS voltage. Only
// meaningful below kHvSenseFaultVolts -- check hvSenseIsFault() first.
constexpr float hvLoadAmps(uint16_t counts, uint16_t full_scale = kAdcFullScale,
                           float reference_volts = kAdcReferenceVolts) {
  return countsToVolts(counts, full_scale, reference_volts) * kHvSenseRatio /
         kHvSenseResistorOhms;
}

constexpr bool hvSenseIsFault(uint16_t counts,
                              uint16_t full_scale = kAdcFullScale,
                              float reference_volts = kAdcReferenceVolts) {
  return countsToVolts(counts, full_scale, reference_volts) >=
         kHvSenseFaultVolts;
}

// =============================================================================
// Motion. Stage 3.
// =============================================================================
// The physical facts of the fitted parts, plus the one piece of TMC5130A
// arithmetic the TMC51X0 driver library does not carry. Everything here is
// Arduino-free, so `make test` covers it -- which matters, because a factor of
// two in a current constant is invisible in a serial log and expensive on the
// bench.
//
// Division of labour with TMC51X0: that library owns register access, the
// ramp generator, homing, and the velocity/acceleration unit conversions. It
// does NOT know this board's sense resistors, this motor's rating, or this
// encoder's resolution, and it cannot express `CHOPCONF.vsense` at all. That
// gap is what this section fills.

// -----------------------------------------------------------------------------
// The driver's clock. X1201, an external 16 MHz oscillator on +3.3V_SW.
// -----------------------------------------------------------------------------
// Every TMC5130A time, velocity and acceleration setting is referenced to it,
// so this is the number to hand tmc51x0::ConverterParameters. The library
// defaults to 12 MHz, the internal clock, which this board does not use -- a
// silent 33% error on every velocity if it is left unset.
constexpr uint8_t kTmcClockMhz = 16;

// -----------------------------------------------------------------------------
// The motor. Lin Engineering WO-208-13-01D, identified 2026-09-02.
// -----------------------------------------------------------------------------
// NEMA 08, 1.8 deg/step, 4-lead bipolar, 0.6 A per phase, 6.5 ohm per phase,
// 3 oz-in holding torque, no gearbox, rear shaft carrying the encoder.
// Datasheet: ../mouse-joystick-pcb/docs/datasheets/motor.pdf.
constexpr int32_t kFullStepsPerRev = 200;       // 1.8 deg/step
constexpr int32_t kMicrostepsPerFullStep = 256; // CHOPCONF.MRES = 0
constexpr int32_t kMicrostepsPerRev =
    kFullStepsPerRev * kMicrostepsPerFullStep; // 51200

constexpr float kMotorPhaseOhms = 6.5f;

// The vendor gives one current figure and does not say whether it is RMS or
// peak. Both readings are recorded rather than one being picked silently:
//
//   equal-heating reading    0.600 A RMS  -- same I^2*R as the DC rating with
//                                           both phases energised
//   peak-of-sine reading     0.424 A RMS  -- 0.6 A read as the sine peak
//
// Firmware enforces the conservative one. This motor has torque to spare for a
// joystick behind the rig's gear pair, and the cheap reading cannot cook a
// NEMA 08 while the question is open. Resolve it by measuring coil current and
// case temperature, then raise the ceiling deliberately and record why.
constexpr float kMotorRatedAmps = 0.6f;
constexpr float kMotorConservativeAmpsRms = 0.424f;

// -----------------------------------------------------------------------------
// Current scaling. TMC5130A datasheet rev 1.21 section 20:
//   I_RMS = (CS+1)/32 * VFS/(R_SENSE + 20 mohm) * 1/sqrt(2)
// -----------------------------------------------------------------------------
// R1203/R1204 are 0.15 ohm on this board. VFS comes from CHOPCONF.vsense:
// V_SRTL = 325 mV at vsense = 0, V_SRTH = 180 mV at vsense = 1, a ratio of
// 1.806. So leaving vsense at its reset value of 0 drives 1.8x the intended
// current -- the most expensive single-bit mistake available on this board,
// and the reason `MotionSetup` refuses to enable the driver until it has read
// the bit back as 1.
constexpr float kSenseResistorOhms = 0.15f;
constexpr float kTmcSenseOffsetOhms = 0.02f;
constexpr float kVfsVsenseLowSensitivity = 0.325f;  // vsense = 0
constexpr float kVfsVsenseHighSensitivity = 0.180f; // vsense = 1
constexpr float kSqrt2 = 1.41421356f;

constexpr float vfsVolts(bool vsense) {
  return vsense ? kVfsVsenseHighSensitivity : kVfsVsenseLowSensitivity;
}

// Coil current in amps RMS for a current-scale setting, CS = 0..31.
constexpr float currentScaleToAmpsRms(uint8_t cs, bool vsense) {
  return (static_cast<float>(cs) + 1.0f) / 32.0f *
         (vfsVolts(vsense) / (kSenseResistorOhms + kTmcSenseOffsetOhms)) /
         kSqrt2;
}

// The inverse, rounded DOWN so a request never becomes more current than was
// asked for.
constexpr uint8_t ampsRmsToCurrentScale(float amps_rms, bool vsense) {
  const float exact = amps_rms * kSqrt2 *
                          (kSenseResistorOhms + kTmcSenseOffsetOhms) /
                          vfsVolts(vsense) * 32.0f -
                      1.0f;
  return (exact <= 0.0f)
             ? 0u
             : ((exact >= 31.0f) ? 31u : static_cast<uint8_t>(exact));
}

// The highest run current this firmware will write, from the conservative
// reading of the motor's rating: CS 17 -> 0.421 A RMS with vsense = 1, the
// largest scale that stays under 0.424 A.
constexpr uint8_t kCurrentScaleCeiling =
    ampsRmsToCurrentScale(kMotorConservativeAmpsRms, /*vsense=*/true);

// First-motion currents. A NEMA 08 turning an unloaded joystick through a gear
// pair needs a fraction of its rating, and a stall against the rig's end stop
// at this current cannot hurt the mechanism.
constexpr uint8_t kCurrentScaleBringUp = 8; // 0.211 A RMS
constexpr uint8_t kCurrentScaleHold = 2;    // 0.070 A RMS

// -----------------------------------------------------------------------------
// The encoder. US Digital E4T-1000, single-ended, on the motor's rear shaft.
// -----------------------------------------------------------------------------
// 1000 CPR, so 4000 quadrature counts per revolution at the TMC's A/B inputs.
// 5 V push-pull through the SN74LVC2G17 translator; the board's 100 kohm
// pull-downs assume push-pull and would be wrong for an open-collector part.
// Datasheet: ../mouse-joystick-pcb/docs/datasheets/encoder.pdf.
//
// The encoder reaches the TMC5130A only. It does not reach a GPIO on any board
// revision, so every position reading is an SPI read of X_ENC or XACTUAL.
constexpr int32_t kEncoderCpr = 1000;
constexpr int32_t kEncoderCountsPerRev = 4 * kEncoderCpr; // 4000

// 51200 / 4000 = 12.8 microsteps per encoder count, exactly.
constexpr float kMicrostepsPerEncoderCount =
    static_cast<float>(kMicrostepsPerRev) /
    static_cast<float>(kEncoderCountsPerRev);

// ENC_CONST scales X_ENC, and its reset default is ZERO -- so X_ENC never
// moves until firmware writes it. An encoder that reads a constant 0 is the
// expected symptom of skipping this, not of a broken encoder. The register is
// write-only, so it cannot be verified by reading it back; the verification is
// that X_ENC tracks a commanded move.
//
// Scaled this way, X_ENC reads in microsteps and is directly comparable with
// XACTUAL. 12.8 is exact in TMC51X0's DecimalMode (fractional part /10000) and
// recurring in BinaryMode (/65536), so decimal is the right choice here.
// One signed value in TMC51X0's DecimalMode units, so 12.8 microsteps per
// quadrature count is 128000. Negate it for an encoder mounted so that it
// counts down as XACTUAL counts up, and let the library do the fixed-point
// split: ENC_CONST's fractional part is always ADDED to a signed integer part,
// so -12.8 is -13 + 0.2 and the obvious -12 with 0.8 would be -11.2, a silent
// 12.5% scale error. Requires TMC51X0 >= 4.1.0.
constexpr int32_t kEncoderMicrostepsPerPulseScaled =
    static_cast<int32_t>(kMicrostepsPerEncoderCount * 10000.0f); // 128000

// -----------------------------------------------------------------------------
// Position conversions.
// -----------------------------------------------------------------------------
constexpr float microstepsToRevs(int32_t microsteps) {
  return static_cast<float>(microsteps) / static_cast<float>(kMicrostepsPerRev);
}

constexpr int32_t revsToMicrosteps(float revs) {
  return static_cast<int32_t>(revs * static_cast<float>(kMicrostepsPerRev));
}

constexpr float microstepsToDegrees(int32_t microsteps) {
  return microstepsToRevs(microsteps) * 360.0f;
}

// Velocity and acceleration are handed to TMC51X0 in microsteps, with
// ConverterParameters left at one microstep per real unit and one second per
// real velocity unit. That keeps the library's integer conversions exact and
// leaves the only unit arithmetic here.
constexpr int32_t revsPerSecondToMicrostepsPerSecond(float revs_per_second) {
  return static_cast<int32_t>(revs_per_second *
                              static_cast<float>(kMicrostepsPerRev));
}

// -----------------------------------------------------------------------------
// Bring-up motion envelope.
// -----------------------------------------------------------------------------
// The motor is installed in the rig, coupled through the gear pair to a
// joystick lever with mechanical end stops, and the travel has not been
// measured. So every default is small and every move is bounded by a target
// position rather than by a stop condition.
constexpr float kJogRevsPerSecond = 0.10f;
constexpr float kRevsPerSecondSq = 0.50f;

// Homing runs in two stages, and the two velocities are the whole point.
//
// The MAX6816 debounces before the driver ever sees the closure, so the latched
// position sits past the true switch edge by (debounce delay x approach
// velocity). That error is systematic, so a single fast approach is repeatable
// only to the extent that the delay is; a single slow approach is accurate but
// spends the whole travel at creep speed.
//
// So: approach fast to find the switch, then creep off it slowly and take the
// RELEASE edge as the datum. The creep covers only the overshoot plus the
// switch throw, so it costs a fraction of a second, and at 20x lower velocity
// its share of the debounce error is 20x smaller. Approaching the datum from
// the same direction every time also removes any hysteresis in the switch.
constexpr float kHomeApproachRevsPerSecond = 0.20f;
constexpr float kHomeCreepRevsPerSecond = 0.01f;

// One fortieth of a motor revolution: 9 degrees at the motor shaft, less at the
// joystick. Small enough to be safe against an unmeasured end stop, large
// enough that the encoder resolves it 100 counts deep.
constexpr int32_t kJogMicrosteps = kMicrostepsPerRev / 40; // 1280

// The furthest one homing approach may travel before giving up. Two motor
// revolutions is far beyond the rig's travel, so reaching this bound means the
// switch never asserted rather than that homing was slow.
constexpr int32_t kHomeTravelBoundMicrosteps = 2 * kMicrostepsPerRev;

// How far the motion-driven switch witness moves per step while looking for the
// switch. Small enough that it locates the switch to better than a degree at
// the motor shaft, and large enough that the sweep crosses the rig's whole
// travel in about a dozen steps.
constexpr int32_t kWitnessStepMicrosteps = 128; // 0.9 deg at the motor

// The rig's actual travel, measured on 2026-09-02 on board E4654C159F4D1334:
// the limit switch bounds the positive end and a mechanical stop bounds the
// negative end, about 4000 microsteps apart -- 28 degrees at the motor shaft,
// less at the joystick through the gear pair.
//
// This is why the distances below are small. It was not known when the envelope
// was first written, and the first encoder round trip of 4 x 1280 microsteps
// ran straight off the end of the mechanism. Anything that moves more than a
// couple of jog steps in one go has to be checked against this number.
constexpr int32_t kMeasuredTravelMicrosteps = 4000;

// After homing, back off far enough that the switch releases and the next
// approach starts from outside it -- but well inside the travel, so the
// back-off cannot land on the opposite end stop. One jog step, a third of the
// measured travel.
constexpr int32_t kHomeBackoffMicrosteps = kJogMicrosteps; // 1280

// The creep off the switch moves in steps and stops as soon as the release edge
// latches, so the bound only has to exceed the worst case: the approach's
// overshoot plus the switch's throw. Measured throw is 256 microsteps, and the
// overshoot at the approach velocity is (debounce delay x 10240 microsteps/s)
// -- about 410 microsteps at 40 ms. Two jog steps is comfortably clear of both
// and still a third of the travel.
constexpr int32_t kHomeCreepStepMicrosteps = 128;
constexpr int32_t kHomeCreepBoundMicrosteps = 2 * kJogMicrosteps; // 2560

// Encoder-based follow check. Below this fraction of the expected encoder
// travel, the shaft is not following the ramp: an end stop, a stalled motor, a
// slipping coupling or a dead encoder. All four want the move stopped rather
// than pushed harder.
// D1 and VSTOP must never be zero in the TMC5130A's positioning mode. The
// datasheet says so explicitly for both, and adds "even if V1=0" -- V1 = 0
// disables the A1/D1 ramp phase but does not make a zero D1 safe.
//
// They are easy to zero by accident, and the accident is not obvious. TMC51X0's
// ControllerParameters are handed over in real units and converted, and a small
// real deceleration rounds down to zero chip units: at 16 MHz the chip unit is
// 116.4 microsteps/s^2, so any real deceleration under that becomes 0. A
// TMC5130A with D1 = 0 in positioning mode does not fault -- it crawls at about
// VSTOP and never reaches the target. Measured on board E4654C159F4D1334 on
// 2026-09-02: VMAX written as 5368 (5119 microsteps/s), 9.5 microsteps/s
// observed, the move timing out 38 microsteps into a 1280 microstep jog.
//
// So every converted ramp parameter that must be non-zero goes through this.
constexpr uint32_t nonZeroRampParameter(uint32_t converted, uint32_t fallback) {
  return (converted != 0) ? converted : ((fallback != 0) ? fallback : 1u);
}

constexpr float kFollowFractionMin = 0.25f;

// The follow check is only evaluated once the ramp generator has commanded this
// much travel, so acceleration from standstill and the encoder's 12.8 microstep
// quantum cannot trip it. 200 microsteps is about 16 encoder counts.
constexpr int32_t kFollowWindowMicrosteps = 200;

} // namespace conversions
} // namespace mouse_joystick
