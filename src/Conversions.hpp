// Scaling between raw ADC counts and board-level quantities.
//
// Header-only and free of Arduino dependencies on purpose, so the arithmetic
// can be tested on the host under `pixi run test` without hardware.

#pragma once

#include <stdint.h>

#include "DAC80501Registers.hpp" // the brake setpoint DAC's transfer function

namespace mouse_joystick {
namespace conversions {

// The RP2040's ADC reference is ADC_AVDD, which on this board is tied straight
// to +3.3V_MCU (U501 pad 43). There is no separate reference and no filter, so
// absolute accuracy is only as good as that rail.
constexpr float kAdcReferenceVolts = 3.3f;

// ---------------------------------------------------------- angle output
//
// Rev 1's DAC80501 (GAIN register 0x0101: REF-DIV 1, BUFF-GAIN 1, a 2.5 V
// full scale) behind the OPA197's 1.32 gain gives 3.300 V at the BNC. The
// numbers are the board's, cited from
// `mouse-joystick-pcb/docs/bringup-log.md`
// ("ANGLE OUTPUT CHARACTERISED -- 2026-09-10, board 4": unity to 0.03 %, a
// 4.5 mV floor at code 0) and from `uart-protocol.md` (ANGSCALE maps onto
// 0.05-3.300 V so that 0 V stays out of band). Which code the firmware parks
// at while the output is invalid, 0 or 497 (25 mV), is that document's
// reserved decision; it is implemented here only, as the 0 V the spec's
// current text describes.
constexpr float kAngleOutStageGain = 1.32f;
constexpr float kAngleOutMinVolts = 0.050f;
constexpr float kAngleOutMaxVolts = 3.300f;
constexpr uint16_t kAngleOutFaultCode = 0;

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

// The SPI clock the MCU drives the TMC5130A at. The TMC51X0 library defaults
// to 1 MHz, chosen on this board for a logic analyser at 1 MSa/s. At 1 MHz a
// register read (two 40-bit datagrams) is 80 us of bus time and the full
// five-register snapshot 400 us, most of a 1 kHz tick (2026-10-02, board 4).
// 4 MHz is the SCK ceiling the TMC5130A datasheet gives for its SPI slave
// (to be confirmed against the datasheet PDF before the first run at this
// rate; the bus switch and the short traces are not the limit); the analyser
// then needs 10 MSa/s or more to resolve the clock.
constexpr uint32_t kTmcSpiClockHz = 4000000;

// -----------------------------------------------------------------------------
// The motor. Lin Engineering WO-208-13-01D, identified 2026-09-02.
// -----------------------------------------------------------------------------
// NEMA 08, 1.8 deg/step, 4-lead bipolar, 0.6 A per phase, 6.5 ohm per phase,
// 3 oz-in holding torque, no gearbox, rear shaft carrying the encoder.
// Datasheet: mouse-joystick-pcb/docs/datasheets/motor.pdf.
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
// Datasheet: mouse-joystick-pcb/docs/datasheets/encoder.pdf.
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

// The inverse, needed because the pyControl protocol speaks degrees and the
// driver speaks microsteps. Truncates toward zero like the rest of this file;
// one microstep is 0.007 deg at the motor shaft, which is far below the 2-5
// encoder counts of homing repeatability measured on the bench, so the rounding
// is not a term worth carrying.
constexpr int32_t degreesToMicrosteps(float degrees) {
  return revsToMicrosteps(degrees / 360.0f);
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

// How long the whole witness sweep may take, as distinct from the per-step
// move timeout. Sized from the mechanism, not borrowed from the jog timeout.
//
// The rig's swing is about 60 deg = 8533 microsteps = 67 steps of 128. Measured
// on 2026-09-10, a 128-microstep step costs ~150 ms including moveBy overhead,
// so a full-length sweep is ~10 s. 25 s leaves margin for a sweep that starts
// at the far stop and for a slower step.
//
// This constant exists because kJogTimeoutMs (4000) was serving as both the
// per-step timeout and the sweep budget, and 4 s only buys ~27 steps = 24 deg.
// A sweep starting further than that from the switch could never reach it, and
// the failure text blamed the debouncer's output joint instead. Three sessions
// lost time to it.
constexpr uint32_t kWitnessSweepTimeoutMs = 25000;

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

// The follow rule's two guards, from board 4 on 2026-10-02 (qp-arduino-example/
// evidence/2026-10-02-1346-board4-rail-up/): at the first energisation of the
// coils the rotor snapped to the driver's electrical angle, 141 microsteps
// (1.0 deg) the wrong way in 17 ms, then followed the ramp at 0.8; the first
// 200-microstep window read that as a stop. So: after each driver enable the
// first degree of commanded travel is not judged and the window is re-seeded
// at its end, and a fault needs two consecutive windows below the fraction.
// The TARGET offsets of the 12:45 run, 0.06 to 0.6 deg, are the same snap and
// stay under the grace.
constexpr int32_t kFollowGraceMicrosteps = 142; // 1.0 deg
constexpr uint8_t kFollowFailuresToFault = 2;

// After each driver enable the rotor is given time to settle before XACTUAL
// and XTARGET are resynced to the encoder: X_ENC within one encoder count
// (12.8 microsteps) for 20 ms, polled every 2 ms, bounded at 150 ms (the
// snap on board 4 took about 45 ms to turn into forward motion).
constexpr int32_t kEnableSettleToleranceMicrosteps = 13;
constexpr uint32_t kEnableSettleStableUs = 20000;
constexpr uint32_t kEnableSettlePollUs = 2000;
constexpr uint32_t kEnableSettleMaxUs = 150000;

// The follow check is only evaluated once the ramp generator has commanded this
// much travel, so acceleration from standstill and the encoder's 12.8 microstep
// quantum cannot trip it. 200 microsteps is about 16 encoder counts.
constexpr int32_t kFollowWindowMicrosteps = 200;

// Settling on the encoder after a move (2026-10-07; uart-protocol.md V1.13).
// Measured on pcb v1.0 unit A on 2026-10-06 (the PCB bring-up log, "the park
// error measured"): with the ramp generator at its target the encoder landed
// 0.35-0.77 deg short on every direction reversal and within 0.07 deg on a
// same-direction move, independent of distance and not accumulating -- the
// rotor's torque angle under the mechanism's friction at the bring-up current
// plus lost motion in the drive train, not the encoder and not a step loss.
// So a move is commanded relative to the encoder (XTARGET = XACTUAL +
// (target - X_ENC)) and, once the ramp generator reports the target, the
// firmware waits for X_ENC to stand still (within one count for
// kMoveSettleStableUs, bounded at kMoveSettleMaxUs), then commands a
// correction of the residual when it exceeds one encoder count, at most
// kMoveSettleMaxCorrections times. TARGET carries the encoder's angle either
// way. XACTUAL is never rewritten for this: a resync deletes the stall
// detector (uart-protocol.md finding 5).
constexpr int32_t kMoveSettleToleranceMicrosteps = 13; // one encoder count
constexpr uint32_t kMoveSettleStableUs = 10000;
constexpr uint32_t kMoveSettleMaxUs = 100000;
constexpr uint8_t kMoveSettleMaxCorrections = 2;
// The same tolerance in degrees, for the simulated backends (0.0914 deg).
constexpr float kMoveSettleToleranceDeg =
    microstepsToDegrees(kMoveSettleToleranceMicrosteps);

// =============================================================================
// pcb v1.0: the brake current sink and the torque bridge.
// =============================================================================
// Board facts, cited rather than chosen. The circuits are design blocks in
// `kicad-libs`: `brake_current_sink_0_49ma_dac80501`
// (docs/brake-current-sink-design.md) and `torque_bridge_0_3v3_ina821`
// (docs/torque-sensor-interface-design.md). The measured behaviour belongs in
// `mouse-joystick-pcb/docs/bringup-log.md` per board serial; the numbers
// here are the DESIGN values the first measurement is compared against.

// -----------------------------------------------------------------------------
// The brake. Placid Industries B5Z-24-1R magnetic particle brake.
// -----------------------------------------------------------------------------
// 24 V winding: 467 ohm at 20 C, 49 mA = 100 % input current = 3.5 N.cm of
// slip torque, 6 ms response. Torque is proportional to current with a
// direction-dependent hysteresis the datasheet draws as two curves, which is
// why the firmware contract says to approach every setpoint from zero and why
// the torque sensor, not the shunt, is the calibration reference. Datasheet:
// mouse-joystick-pcb/docs/datasheets/brake.pdf.
constexpr float kBrakeCoilOhms = 467.0f;
constexpr float kBrakeRatedAmps = 0.049f;
constexpr float kBrakeRatedTorqueNcm = 3.5f;
constexpr uint32_t kBrakeResponseMs = 6;

// The sink. A DAC80501Z at I2C 0x49 (GAIN 0x0101, 2.5 V full scale) through a
// 3.2 k / 16.4 k divider sets V_SET across a 10 ohm shunt under a linear pass
// device from +36V_SW: 2.5 V x 0.19512 / 10 ohm = 48.78 mA full scale, 99.6 %
// of the brake's rating, 0.744 uA per code. An independent ceiling amplifier
// limits at V_LIMIT 0.564 V / 10 ohm = 56.4 mA and pulls ~{BRAKE_FAULT} low
// while it is limiting -- a continuous indication, not a latch.
constexpr uint16_t kBrakeDacGainRegister =
    dac80501::kGainRefDiv | dac80501::kGainBuffGain; // 2.5 V full scale
constexpr float kBrakeSetDividerRatio = 3.2f / 16.4f; // 0.19512
constexpr float kBrakeShuntOhms = 10.0f;
constexpr float kBrakeFullScaleAmps =
    dac80501::fullScaleVolts(kBrakeDacGainRegister) * kBrakeSetDividerRatio /
    kBrakeShuntOhms; // 0.04878
constexpr float kBrakeCeilingAmps = 0.564f / kBrakeShuntOhms; // 0.0564

// The monitor. The shunt voltage through a x4.125 amplifier and a 3.2 k /
// 3.2 k divider reaches BRAKE_ISENSE (GP27, ADC1) at 20.625 V per amp:
// 0-1.006 V for 0-48.78 mA, 1.163 V at the ceiling. The monitor amplifier's
// output floor is about 25 mV before the divider, so ~12.5 mV at the pin:
// readings below that mean "unpowered or zero", not a current.
constexpr float kBrakeMonitorGain = 4.125f * 0.5f;
constexpr float kBrakeIsenseVoltsPerAmp = kBrakeShuntOhms * kBrakeMonitorGain; // 20.625
constexpr float kBrakeIsenseFloorVolts = 0.0125f;

// Commanded coil current for a DAC code.
constexpr float brakeCodeToAmps(uint16_t code) {
  return dac80501::codeToVolts(code, kBrakeDacGainRegister) *
         kBrakeSetDividerRatio / kBrakeShuntOhms;
}

// Nearest DAC code for a coil current, clamped at full scale. Never more than
// asked: the brake's rating and the sink's full scale are 0.4 % apart.
inline uint16_t brakeAmpsToCode(float amps) {
  return dac80501::voltsToCode(amps * kBrakeShuntOhms / kBrakeSetDividerRatio,
                               kBrakeDacGainRegister);
}

constexpr float brakeIsenseVoltsToAmps(float volts) {
  return volts / kBrakeIsenseVoltsPerAmp;
}

constexpr float brakeIsenseCountsToAmps(uint16_t counts,
                                        uint16_t full_scale = kAdcFullScale,
                                        float reference_volts = kAdcReferenceVolts) {
  return brakeIsenseVoltsToAmps(
      countsToVolts(counts, full_scale, reference_volts));
}

// The datasheet's nominal straight line; the real curve has hysteresis and
// a per-unit calibration (rig plan phase C4) owns the number that matters.
constexpr float brakeAmpsToTorqueNcm(float amps) {
  return amps / kBrakeRatedAmps * kBrakeRatedTorqueNcm;
}

// -----------------------------------------------------------------------------
// The torque sensor. Transducer Techniques RTS-5 reaction torque sensor.
// -----------------------------------------------------------------------------
// 5 in-oz = 3.5309 N.cm rated, 1.5 mV/V nominal (the two certificates read
// 1.544 and 1.577 mV/V: a per-sensor constant to store against the board
// serial it is paired with), 350 ohm bridge, 10 V excitation, 150 % safe
// overload. Datasheet: mouse-joystick-pcb/docs/datasheets/torque-sensor-specs.png.
constexpr float kTorqueRatedNcm = 3.5309f;
constexpr float kTorqueSensitivityNominalVPerV = 1.5e-3f;
constexpr float kTorqueBridgeOhms = 350.0f;
constexpr float kTorqueExcitationVolts = 10.0f;

// The front end. VEXC 10.000 V (REF5010A), VMID 5.000 V, an INA821 with
// R_G = 249 + 10 = 259 ohm (G = 1 + 49.4 k / R_G = 191.73) referenced to VMID,
// then a divide-by-3 to AMP_VOUT. The ADS8689 reads AMP_VOUT at full
// amplitude through 249 ohm; a further divide-by-2 feeds the BNC and
// TQ_AOUT_MON. Zero torque is therefore 5 V / 3 = 1.6667 V at the ADC and
// 0.8333 V at the BNC, and the sign of torque is the sign of the departure
// from that.
constexpr float kTorqueInaGain = 1.0f + 49400.0f / 259.0f; // 191.73
constexpr float kTorqueMidVolts = 5.0f;
constexpr float kTorqueOutputDivider = 1.0f / 3.0f;
constexpr float kTorqueBncDivider = 0.5f;
constexpr float kTorqueZeroVolts = kTorqueMidVolts * kTorqueOutputDivider; // 1.6667

// Volts at AMP_VOUT per N.cm, for the nominal sensitivity: 15 mV x 191.73 / 3
// over 3.5309 N.cm = 0.2715 V per N.cm, so +/-rated spans 0.708-2.625 V and
// +/-150 % 0.229-3.105 V, all inside the ADC's 0-5.12 V range.
constexpr float kTorqueVoltsPerNcm =
    kTorqueSensitivityNominalVPerV * kTorqueExcitationVolts * kTorqueInaGain *
    kTorqueOutputDivider / kTorqueRatedNcm;

// The ADS8689 on the 0 to 1.25 x VREF range: 5.12 V over 65536 codes.
constexpr float kTorqueAdcFullScaleVolts = 1.25f * 4.096f;
constexpr float kTorqueAdcVoltsPerCode = kTorqueAdcFullScaleVolts / 65536.0f; // 78.125 uV
constexpr uint16_t kTorqueZeroCodeNominal =
    static_cast<uint16_t>(kTorqueZeroVolts / kTorqueAdcVoltsPerCode + 0.5f); // 21333

constexpr float torqueAdcCodeToVolts(uint16_t code) {
  return static_cast<float>(code) * kTorqueAdcVoltsPerCode;
}

// Torque from the ADC reading, relative to a zero. The zero is a MEASURED
// code (a tare at rest), falling back to the nominal; the scale is nominal
// until the per-sensor certificate constant replaces it (rig plan C3).
constexpr float torqueVoltsToNcm(float volts, float zero_volts = kTorqueZeroVolts,
                                 float volts_per_ncm = kTorqueVoltsPerNcm) {
  return (volts - zero_volts) / volts_per_ncm;
}

constexpr float torqueAdcCodeToNcm(uint16_t code,
                                   uint16_t zero_code = kTorqueZeroCodeNominal,
                                   float volts_per_ncm = kTorqueVoltsPerNcm) {
  return torqueVoltsToNcm(torqueAdcCodeToVolts(code),
                          torqueAdcCodeToVolts(zero_code), volts_per_ncm);
}

// Shunt calibration. TQ_CAL_EN (GP18) is meant to switch R2003 75 kohm across
// one 350 ohm arm, unbalancing the bridge by R / (2 (2 R_cal + R)) =
// 1.164 mV/V, i.e. 77.6 % of the nominal rated output, which is a 0.744 V
// step at AMP_VOUT (9523 codes). The 2026-10-04 design review found the
// switch's N-channel MOSFET has its source at ~5 V and a 3.3 V gate, so it
// may not turn on; the step is what the bench looks for, and its absence is
// a board finding, not a firmware one.
constexpr float kTorqueShuntCalOhms = 75000.0f;
constexpr float kTorqueShuntCalVoltsPerVolt =
    kTorqueBridgeOhms / (2.0f * (2.0f * kTorqueShuntCalOhms + kTorqueBridgeOhms));
constexpr float kTorqueShuntCalStepVolts =
    kTorqueShuntCalVoltsPerVolt * kTorqueExcitationVolts * kTorqueInaGain *
    kTorqueOutputDivider; // 0.744 V at AMP_VOUT


} // namespace conversions

// What an out-and-back round trip says about the mechanism, from the encoder
// travel on each leg alone. Lives here rather than beside MoveReport in
// MotionSetup.hpp because that header needs Arduino.h and TMC51X0.hpp, and
// this is arithmetic that host tests must be able to reach.
//
// Both legs command the same distance, so unequal encoder travel means the
// mechanism was obstructed on the shorter one. This is a separate check from
// MoveOutcome, and it has to be: driving into an end stop open-loop still
// reports kTargetReached whenever the follow window does not happen to trip,
// because the ramp generator only counts the steps it issued. Two real cases,
// both measured on board E4654C159F171E34 on 2026-09-09:
//
//   hard against the stop from the start -> the follow check trips mid-move and
//     MoveOutcome says so. Already handled by the caller.
//   PARTIAL clearance -> the move completes, reports kTargetReached, and the
//     asymmetry is the only remaining witness. Outbound saw 704 usteps against
//     the return's 1177, and the whole 473-ustep residual was that difference.
//     That case was misread as an encoder CPR fault.
enum class RoundTripVerdict {
  kSymmetric,          // both legs agree; any ratio error is really the encoder
  kOutboundObstructed, // the outbound leg came up short
  kReturnObstructed,   // the return leg came up short
};

// `tolerance_percent` is how far apart the two legs may be before the mechanism
// is implicated.
//
// Backlash is what sets the floor, and it is NOT negligible. It costs a fixed
// amount on each reversal, and whether the outbound or the return pays it
// depends on which way the mechanism was last moving -- homing ends with a
// back-off, so the first leg of a round trip may already be taken up. That
// makes legitimate asymmetry one-sided rather than symmetric. Measured on
// board E4654C159F171E34, 2026-09-09, both from mid-travel after homing:
//
//   1203 out / 1190 back   1.1% apart   free mechanism
//   1280 out / 1191 back   7.0% apart   free mechanism, one backlash-worth
//    704 out / 1177 back  40.2% apart   OUTBOUND AGAINST THE FAR HARD STOP
//
// So the real envelope is "up to ~7% is backlash, ~40% is a stop", and the
// threshold sits between them. 20% is roughly the geometric middle and is what
// this defaults to. An earlier draft used 10%, which is only 1.4x above the
// measured backlash figure and would have called a free mechanism obstructed on
// a slightly stiffer gear pair.
//
// A genuine CPR error lowers both legs equally, so it stays symmetric and its
// diagnosis stays reachable -- that is deliberate.
constexpr RoundTripVerdict classifyRoundTrip(int32_t out_travel,
                                             int32_t back_travel,
                                             int32_t tolerance_percent = 20) {
  const int32_t out_abs = out_travel < 0 ? -out_travel : out_travel;
  const int32_t back_abs = back_travel < 0 ? -back_travel : back_travel;
  const int32_t larger = out_abs > back_abs ? out_abs : back_abs;
  const int32_t gap =
      out_abs > back_abs ? out_abs - back_abs : back_abs - out_abs;
  if (larger == 0 || gap * 100 <= larger * tolerance_percent) {
    return RoundTripVerdict::kSymmetric;
  }
  return out_abs < back_abs ? RoundTripVerdict::kOutboundObstructed
                            : RoundTripVerdict::kReturnObstructed;
}

} // namespace mouse_joystick
