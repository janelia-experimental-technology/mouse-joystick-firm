// Host tests for the stage 3 motion arithmetic in src/Conversions.hpp.
//
// These pin the numbers that turn into current through a motor and into
// distance through a mechanism. Both are expensive to get wrong and neither is
// visible in a serial log: a current constant that is 1.8x out prints exactly
// the same as a correct one, and an encoder scale that is 2x out reads as a
// plausible position.
//
// The values here are computed independently from the datasheet formulae rather
// than copied from the implementation, so a test failure means the arithmetic
// disagrees with the datasheet rather than with itself.

#include <unity.h>

#include "Conversions.hpp"

using namespace mouse_joystick::conversions;

void setUp(void) {}
void tearDown(void) {}

// ---------------------------------------------------------------------------
// Fitted parts
// ---------------------------------------------------------------------------

void test_fitted_parts(void) {
  // Lin Engineering WO-208-13-01D: 1.8 deg/step, 0.6 A, 6.5 ohm.
  TEST_ASSERT_EQUAL_INT32(200, kFullStepsPerRev);
  TEST_ASSERT_EQUAL_INT32(256, kMicrostepsPerFullStep);
  TEST_ASSERT_EQUAL_INT32(51200, kMicrostepsPerRev);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.6f, kMotorRatedAmps);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 6.5f, kMotorPhaseOhms);

  // US Digital E4T-1000, quadrature.
  TEST_ASSERT_EQUAL_INT32(1000, kEncoderCpr);
  TEST_ASSERT_EQUAL_INT32(4000, kEncoderCountsPerRev);

  // R1203/R1204 on this board, plus the TMC5130A's 20 mohm internal offset.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.15f, kSenseResistorOhms);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.02f, kTmcSenseOffsetOhms);

  // X1201. TMC51X0 defaults to 12, the internal clock, which this board does
  // not use -- so a 12 here would be a silent 33% velocity error.
  TEST_ASSERT_EQUAL_UINT8(16, kTmcClockMhz);
}

// ---------------------------------------------------------------------------
// Current scaling
// ---------------------------------------------------------------------------

void test_vfs_matches_the_datasheet(void) {
  // V_SRTL and V_SRTH from TMC5130A rev 1.21 electrical characteristics.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.325f, vfsVolts(false));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.180f, vfsVolts(true));
}

void test_current_scale_to_amps_at_vsense_one(void) {
  // I_RMS = (CS+1)/32 * 0.180/(0.15+0.02) / sqrt(2)
  //       = (CS+1) * 0.0233967 A
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0234f, currentScaleToAmpsRms(0, true));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0702f, currentScaleToAmpsRms(2, true));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.2106f, currentScaleToAmpsRms(8, true));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.4211f, currentScaleToAmpsRms(17, true));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.7487f, currentScaleToAmpsRms(31, true));
}

void test_vsense_zero_is_1_8x_the_current(void) {
  // This is the whole reason MotionSetup reads CHOPCONF back. 0.325/0.180 is
  // 1.806, so every current is 1.8x larger with vsense left at its reset value
  // -- and IHOLD_IRUN is write-only, so IRUN itself can never be checked.
  const float ratio =
      currentScaleToAmpsRms(8, false) / currentScaleToAmpsRms(8, true);
  TEST_ASSERT_FLOAT_WITHIN(2e-3f, 1.806f, ratio);

  // The historical IRUN = 28 figure, both ways round. 1.226 A through a motor
  // rated 0.6 A is what a missing vsense bit costs.
  TEST_ASSERT_FLOAT_WITHIN(2e-3f, 0.6785f, currentScaleToAmpsRms(28, true));
  TEST_ASSERT_FLOAT_WITHIN(2e-3f, 1.2255f, currentScaleToAmpsRms(28, false));
}

void test_amps_to_current_scale_rounds_down(void) {
  // Never more current than was asked for.
  TEST_ASSERT_EQUAL_UINT8(17, ampsRmsToCurrentScale(0.424f, true));
  TEST_ASSERT_EQUAL_UINT8(8, ampsRmsToCurrentScale(0.211f, true));
  TEST_ASSERT_EQUAL_UINT8(0, ampsRmsToCurrentScale(0.0f, true));
  TEST_ASSERT_EQUAL_UINT8(31, ampsRmsToCurrentScale(10.0f, true));

  // Round trip: the scale chosen for a target never exceeds the target.
  for (uint8_t cs = 0; cs <= 31; ++cs) {
    const float amps = currentScaleToAmpsRms(cs, true);
    TEST_ASSERT_TRUE(currentScaleToAmpsRms(ampsRmsToCurrentScale(amps, true),
                                           true) <= amps + 1e-4f);
  }
}

void test_the_ceiling_stays_under_the_motor_rating(void) {
  // The conservative reading of the vendor's 0.6 A: treat it as the sine peak,
  // so 0.6/sqrt(2) = 0.424 A RMS.
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, kMotorRatedAmps / 1.41421356f,
                           kMotorConservativeAmpsRms);

  TEST_ASSERT_EQUAL_UINT8(17, kCurrentScaleCeiling);
  TEST_ASSERT_TRUE(currentScaleToAmpsRms(kCurrentScaleCeiling, true) <=
                   kMotorConservativeAmpsRms);
  // And one scale higher would not be.
  TEST_ASSERT_TRUE(currentScaleToAmpsRms(kCurrentScaleCeiling + 1, true) >
                   kMotorConservativeAmpsRms);
}

void test_bring_up_currents_are_within_the_ceiling(void) {
  TEST_ASSERT_TRUE(kCurrentScaleBringUp <= kCurrentScaleCeiling);
  TEST_ASSERT_TRUE(kCurrentScaleHold <= kCurrentScaleBringUp);

  // Standstill dissipation at the bring-up current, both phases: well under a
  // watt, which a NEMA 08 shrugs off.
  const float amps = currentScaleToAmpsRms(kCurrentScaleBringUp, true);
  const float watts = 2.0f * amps * amps * kMotorPhaseOhms;
  TEST_ASSERT_TRUE(watts < 1.0f);
}

// ---------------------------------------------------------------------------
// Position and the encoder
// ---------------------------------------------------------------------------

void test_position_conversions(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, microstepsToRevs(51200));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.5f, microstepsToRevs(-25600));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 360.0f, microstepsToDegrees(51200));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.8f, microstepsToDegrees(256));
  TEST_ASSERT_EQUAL_INT32(51200, revsToMicrosteps(1.0f));
  TEST_ASSERT_EQUAL_INT32(-1280, revsToMicrosteps(-0.025f));
}

void test_encoder_scale_is_exact(void) {
  // 51200 microsteps / 4000 counts = 12.8, exactly.
  TEST_ASSERT_FLOAT_WITHIN(1e-9f, 12.8f, kMicrostepsPerEncoderCount);

  // Which is why ENC_CONST goes in decimal mode: 12.8 is exact against a
  // /10000 fraction and recurs against /65536. The scaled form is that value
  // times 10000, and TMC51X0 splits it into the signed fixed-point pair.
  TEST_ASSERT_EQUAL_INT32(128000, kEncoderMicrostepsPerPulseScaled);
  TEST_ASSERT_FLOAT_WITHIN(
      1e-9f, kMicrostepsPerEncoderCount,
      static_cast<float>(kEncoderMicrostepsPerPulseScaled) / 10000.0f);

  // Exact in decimal, so nothing is lost either way round.
  TEST_ASSERT_EQUAL_INT32(kEncoderMicrostepsPerPulseScaled,
                          -(-kEncoderMicrostepsPerPulseScaled));
}

void test_one_jog_step_is_resolvable_by_the_encoder(void) {
  // 1280 microsteps is 9 degrees at the motor shaft and 100 encoder counts, so
  // the follow check has two decimal digits of signal to work with rather than
  // fighting the 12.8 microstep quantum.
  TEST_ASSERT_EQUAL_INT32(1280, kJogMicrosteps);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 9.0f, microstepsToDegrees(kJogMicrosteps));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.0f,
                           static_cast<float>(kJogMicrosteps) /
                               kMicrostepsPerEncoderCount);

  // And the follow window is at least ten counts, so a single count of
  // quantisation cannot swing the ratio past the 0.25 threshold.
  TEST_ASSERT_TRUE((static_cast<float>(kFollowWindowMicrosteps) /
                    kMicrostepsPerEncoderCount) >= 10.0f);
}

// ---------------------------------------------------------------------------
// The motion envelope
// ---------------------------------------------------------------------------

void test_velocity_unit_conversion(void) {
  TEST_ASSERT_EQUAL_INT32(5120, revsPerSecondToMicrostepsPerSecond(0.10f));
  TEST_ASSERT_EQUAL_INT32(2560, revsPerSecondToMicrostepsPerSecond(0.05f));
  TEST_ASSERT_EQUAL_INT32(25600, revsPerSecondToMicrostepsPerSecond(0.50f));
}

void test_two_stage_homing_velocities(void) {
  // The creep has to be much slower than the approach or the second stage buys
  // nothing: the debounce error scales with velocity, so the improvement in the
  // datum is exactly the ratio of the two.
  TEST_ASSERT_TRUE(kHomeCreepRevsPerSecond < kHomeApproachRevsPerSecond);
  TEST_ASSERT_TRUE((kHomeApproachRevsPerSecond / kHomeCreepRevsPerSecond) >=
                   10.0f);

  // The approach may be faster than a jog -- it is bounded, and it is not the
  // datum -- but the creep must be slower than everything else.
  TEST_ASSERT_TRUE(kHomeCreepRevsPerSecond < kJogRevsPerSecond);

  // At the creep velocity a 40 ms reporting delay has to cost less than a
  // couple of encoder counts, which is the finest the datum can be read
  // anyway -- encoder counts, not degrees, because a count is the measurement
  // floor. 0.01 rev/s x 40 ms is 20 microsteps, which is 1.6 counts.
  const float creep_offset =
      0.040f * static_cast<float>(
                   revsPerSecondToMicrostepsPerSecond(kHomeCreepRevsPerSecond));
  TEST_ASSERT_TRUE((creep_offset / kMicrostepsPerEncoderCount) < 2.0f);

  // And the same delay at the approach velocity costs far more than that,
  // which is the entire reason the second stage exists.
  const float approach_offset =
      0.040f * static_cast<float>(revsPerSecondToMicrostepsPerSecond(
                   kHomeApproachRevsPerSecond));
  TEST_ASSERT_TRUE((approach_offset / kMicrostepsPerEncoderCount) > 10.0f);
}

void test_creep_bound_covers_the_approach_overshoot(void) {
  // The creep has to travel the approach's debounce overshoot plus the switch's
  // measured 256-microstep throw before the release edge appears. At 40 ms --
  // the order of magnitude for a MAX6816 -- that is:
  const float approach_overshoot =
      0.040f * static_cast<float>(revsPerSecondToMicrostepsPerSecond(
                   kHomeApproachRevsPerSecond));
  const float needed = approach_overshoot + 256.0f;
  TEST_ASSERT_TRUE(static_cast<float>(kHomeCreepBoundMicrosteps) > needed);

  // And the bound has to fit inside the travel, since the creep runs inward.
  TEST_ASSERT_TRUE(kHomeCreepBoundMicrosteps < kMeasuredTravelMicrosteps);

  // The creep steps have to resolve the throw, or the release could be missed
  // between two steps. The latch is captured in hardware so the step size does
  // not quantise the datum, but it does bound how far past the edge the creep
  // travels before it notices.
  TEST_ASSERT_TRUE(kHomeCreepStepMicrosteps < 256);
}

void test_homing_bounds(void) {
  // The travel bound is far beyond the rig's travel, so reaching it means the
  // switch never asserted rather than that homing was slow.
  TEST_ASSERT_EQUAL_INT32(2 * kMicrostepsPerRev, kHomeTravelBoundMicrosteps);
  TEST_ASSERT_TRUE(kHomeTravelBoundMicrosteps > 10 * kJogMicrosteps);

  // The back-off has to clear the switch without landing on the opposite end
  // stop, so it has to fit comfortably inside the measured travel.
  TEST_ASSERT_EQUAL_INT32(1280, kHomeBackoffMicrosteps);
  TEST_ASSERT_TRUE(kHomeBackoffMicrosteps < kHomeTravelBoundMicrosteps);
  TEST_ASSERT_TRUE(kHomeBackoffMicrosteps * 2 < kMeasuredTravelMicrosteps);

  // And a single jog step has to fit inside the travel with room to spare, or
  // the interactive commands would run off the end every other press.
  TEST_ASSERT_TRUE(kJogMicrosteps * 3 <= kMeasuredTravelMicrosteps);
}

void test_non_zero_ramp_parameter_guard(void) {
  // A converted value that survived is used as it stands.
  TEST_ASSERT_EQUAL_UINT32(219, nonZeroRampParameter(219, 219));
  TEST_ASSERT_EQUAL_UINT32(1, nonZeroRampParameter(1, 500));

  // A value that rounded to zero falls back, which is the case that bit: D1
  // asked for in real units, converted to 0 chip units, and a TMC5130A that
  // then crawls at VSTOP instead of running the ramp.
  TEST_ASSERT_EQUAL_UINT32(219, nonZeroRampParameter(0, 219));

  // And if the fallback is zero too, the guard still cannot return zero,
  // because zero is the one value the datasheet forbids.
  TEST_ASSERT_EQUAL_UINT32(1, nonZeroRampParameter(0, 0));
}

void test_the_chip_acceleration_quantum_is_the_trap(void) {
  // At 16 MHz one chip unit of acceleration is fCLK^2/(512*256*2^24) =
  // 116.4 microsteps/s^2, so any real deceleration below that rounds to zero.
  // This is why asking for the library's default of 10 in real units produced
  // D1 = 0. The bring-up envelope's acceleration is three orders of magnitude
  // above the quantum, so it converts to a healthy 219.
  const float quantum =
      16000000.0f * 16000000.0f / (512.0f * 256.0f) / 16777216.0f;
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 116.4f, quantum);
  TEST_ASSERT_TRUE(10.0f < quantum); // the value that rounded to zero
  TEST_ASSERT_TRUE(static_cast<float>(revsPerSecondToMicrostepsPerSecond(
                       kRevsPerSecondSq)) > 100.0f * quantum);
}

void test_follow_threshold_is_a_fraction(void) {
  TEST_ASSERT_TRUE(kFollowFractionMin > 0.0f);
  TEST_ASSERT_TRUE(kFollowFractionMin < 1.0f);
}

// ---------------------------------------------------------------------------
// Round-trip classification: mechanism versus encoder
// ---------------------------------------------------------------------------

void test_round_trip_symmetric_legs_are_not_the_mechanism(void) {
  using mouse_joystick::classifyRoundTrip;
  using mouse_joystick::RoundTripVerdict;
  // Measured from mid-travel on board E4654C159F171E34, 2026-09-09, after
  // homing: 1203 out against 1190 back. Both ratios sat near 0.94 because a
  // reversal costs backlash, but the legs agree to 1.1% so the mechanism was
  // free and any error left is genuinely the encoder's.
  TEST_ASSERT_EQUAL(RoundTripVerdict::kSymmetric,
                    classifyRoundTrip(-1203, 1190));
  // Exactly equal legs.
  TEST_ASSERT_EQUAL(RoundTripVerdict::kSymmetric,
                    classifyRoundTrip(-1280, 1280));
  // A genuine 2:1 CPR error lowers BOTH legs equally and must stay symmetric,
  // or the CPR diagnosis becomes unreachable.
  TEST_ASSERT_EQUAL(RoundTripVerdict::kSymmetric, classifyRoundTrip(-640, 640));
  // One backlash-worth of one-sided loss. Measured on the same board and the
  // same day as the obstructed case, also from mid-travel after homing, with
  // the mechanism demonstrably free: 1280 out against 1191 back, 7.0% apart.
  // This is the case that makes a 10% threshold too tight.
  TEST_ASSERT_EQUAL(RoundTripVerdict::kSymmetric,
                    classifyRoundTrip(-1280, 1191));
  // Neither leg moved: a dead encoder, not an obstruction.
  TEST_ASSERT_EQUAL(RoundTripVerdict::kSymmetric, classifyRoundTrip(0, 0));
}

void test_round_trip_detects_the_obstructed_leg(void) {
  using mouse_joystick::classifyRoundTrip;
  using mouse_joystick::RoundTripVerdict;
  // The case that was misdiagnosed. Measured on board E4654C159F171E34 starting
  // against the far stop: outbound saw 704 usteps, the return ran clear at
  // 1177, and the old code blamed the encoder's CPR.
  TEST_ASSERT_EQUAL(RoundTripVerdict::kOutboundObstructed,
                    classifyRoundTrip(-704, 1177));
  // Mirrored: obstructed on the way back instead.
  TEST_ASSERT_EQUAL(RoundTripVerdict::kReturnObstructed,
                    classifyRoundTrip(-1177, 704));
  // Sign of the travel must not change the verdict -- only magnitude matters,
  // so an encoder mounted the other way round still classifies correctly.
  TEST_ASSERT_EQUAL(RoundTripVerdict::kOutboundObstructed,
                    classifyRoundTrip(704, -1177));
  // One leg fully blocked.
  TEST_ASSERT_EQUAL(RoundTripVerdict::kOutboundObstructed,
                    classifyRoundTrip(0, 1280));
}

void test_round_trip_threshold_boundary(void) {
  using mouse_joystick::classifyRoundTrip;
  using mouse_joystick::RoundTripVerdict;
  // Exactly at the default 20% tolerance is still symmetric; just past it is
  // not. 1280 * 0.8 = 1024.
  TEST_ASSERT_EQUAL(RoundTripVerdict::kSymmetric,
                    classifyRoundTrip(-1024, 1280));
  TEST_ASSERT_EQUAL(RoundTripVerdict::kOutboundObstructed,
                    classifyRoundTrip(-1023, 1280));
  // The threshold has to sit above the measured backlash figure and below the
  // measured obstructed one, for every tolerance a caller might reasonably
  // pass. Both bounds, explicitly.
  TEST_ASSERT_EQUAL(RoundTripVerdict::kSymmetric,
                    classifyRoundTrip(-1191, 1280, 10));
  TEST_ASSERT_EQUAL(RoundTripVerdict::kOutboundObstructed,
                    classifyRoundTrip(-704, 1177, 30));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_fitted_parts);
  RUN_TEST(test_vfs_matches_the_datasheet);
  RUN_TEST(test_current_scale_to_amps_at_vsense_one);
  RUN_TEST(test_vsense_zero_is_1_8x_the_current);
  RUN_TEST(test_amps_to_current_scale_rounds_down);
  RUN_TEST(test_the_ceiling_stays_under_the_motor_rating);
  RUN_TEST(test_bring_up_currents_are_within_the_ceiling);
  RUN_TEST(test_position_conversions);
  RUN_TEST(test_encoder_scale_is_exact);
  RUN_TEST(test_one_jog_step_is_resolvable_by_the_encoder);
  RUN_TEST(test_velocity_unit_conversion);
  RUN_TEST(test_two_stage_homing_velocities);
  RUN_TEST(test_creep_bound_covers_the_approach_overshoot);
  RUN_TEST(test_homing_bounds);
  RUN_TEST(test_non_zero_ramp_parameter_guard);
  RUN_TEST(test_the_chip_acceleration_quantum_is_the_trap);
  RUN_TEST(test_follow_threshold_is_a_fraction);
  RUN_TEST(test_round_trip_symmetric_legs_are_not_the_mechanism);
  RUN_TEST(test_round_trip_detects_the_obstructed_leg);
  RUN_TEST(test_round_trip_threshold_boundary);
  return UNITY_END();
}
