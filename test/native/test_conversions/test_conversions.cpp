// Host tests for the ADC scaling in src/Conversions.hpp.
//
// This is the only part of the firmware that can be tested without the board.
// It is worth testing anyway: a divider ratio or a full-scale constant that is
// silently wrong turns a bring-up reading into a plausible lie.

#include <unity.h>

#include "Conversions.hpp"

using namespace mouse_joystick::conversions;

void setUp(void) {}
void tearDown(void) {}

void test_counts_to_volts_endpoints(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, countsToVolts(0));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, kAdcReferenceVolts,
                           countsToVolts(kAdcFullScale));
}

void test_counts_to_volts_midscale(void) {
  // 2048 of 4095 is a hair over half scale.
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.6504f, countsToVolts(2048));
}

void test_counts_to_volts_survives_zero_full_scale(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, countsToVolts(1234, 0));
}

void test_sw_rail_undoes_the_divider(void) {
  // R401/R402 are 1k/1k, so the rail is twice the measured node. A nominal
  // 3.3 V rail lands at 1.65 V on GP26, which is 2048 counts.
  TEST_ASSERT_FLOAT_WITHIN(2e-3f, 3.3009f, swRailVolts(2048));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, swRailVolts(0));
}

void test_sw_rail_off_reads_below_threshold(void) {
  // The self test calls +3.3V_SW off below 0.30 V on the undivided rail. A few
  // counts of ADC offset must not trip that.
  TEST_ASSERT_TRUE(swRailVolts(10) < 0.30f);
  // A live rail must trip it.
  TEST_ASSERT_TRUE(swRailVolts(2048) > 0.30f);
}

void test_hv_sense_is_raw_volts(void) {
  // hvSenseVolts() reports the voltage across R1907 without converting to
  // current: that needs the TPS1HTC100-Q1 sense ratio, which this firmware does
  // not carry yet.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, countsToVolts(500), hvSenseVolts(500));
}

void test_full_scale_matches_declared_resolution(void) {
  // These two drifting apart is not hypothetical: stage 2 initially read at the
  // Arduino core's default 10 bits while dividing by the 12-bit full scale, and
  // reported a quarter of every voltage.
  TEST_ASSERT_EQUAL_UINT16((1u << kAdcResolutionBits) - 1u, kAdcFullScale);
}

void test_board_constants(void) {
  TEST_ASSERT_EQUAL_UINT8(12, kAdcResolutionBits);
  TEST_ASSERT_EQUAL_UINT16(4095, kAdcFullScale);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.3f, kAdcReferenceVolts);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, kSwRailDividerRatio);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 249.0f, kHvSenseResistorOhms);
}

void test_degrees_round_trip_through_microsteps(void) {
  // The protocol speaks degrees and the driver speaks microsteps, so this pair
  // sits on every commanded position. A sign error here would move the joystick
  // the wrong way and a scale error would silently rescale the travel limits.
  TEST_ASSERT_EQUAL_INT32(0, degreesToMicrosteps(0.0f));
  TEST_ASSERT_EQUAL_INT32(kMicrostepsPerRev, degreesToMicrosteps(360.0f));
  TEST_ASSERT_EQUAL_INT32(-kMicrostepsPerRev, degreesToMicrosteps(-360.0f));

  // One jog step is 1280 microsteps = 9.00 deg, measured on the bench.
  TEST_ASSERT_EQUAL_INT32(kJogMicrosteps, degreesToMicrosteps(9.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 9.0f, microstepsToDegrees(kJogMicrosteps));

  for (int32_t deg = -28; deg <= 28; ++deg) {
    const float back =
        microstepsToDegrees(degreesToMicrosteps(static_cast<float>(deg)));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, static_cast<float>(deg), back);
  }
}

// ---------------------------------------------------------------------------
// pcb v1.0: brake sink and torque bridge (design values from the kicad-libs
// design notes; the bench compares against these)
// ---------------------------------------------------------------------------

void test_brake_full_scale_is_the_designed_48_78_ma(void) {
  // 2.5 V x 3.2k/16.4k / 10 ohm. 99.6 % of the B5Z-24's 49 mA rating.
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.04878f, kBrakeFullScaleAmps);
  TEST_ASSERT_TRUE(kBrakeFullScaleAmps < kBrakeRatedAmps);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0564f, kBrakeCeilingAmps);
  TEST_ASSERT_TRUE(kBrakeCeilingAmps > kBrakeFullScaleAmps);
}

void test_brake_code_to_amps_endpoints(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, brakeCodeToAmps(0));
  // Code 0xFFFF is 65535/65536 of full scale, like the DAC's own arithmetic.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, kBrakeFullScaleAmps * 65535.0f / 65536.0f,
                           brakeCodeToAmps(0xFFFF));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, kBrakeFullScaleAmps / 2.0f,
                           brakeCodeToAmps(0x8000));
  // One code is 0.744 uA.
  TEST_ASSERT_FLOAT_WITHIN(1e-8f, 0.744e-6f, brakeCodeToAmps(1));
}

void test_brake_amps_to_code_round_trip_and_clamp(void) {
  TEST_ASSERT_EQUAL_UINT16(0, brakeAmpsToCode(0.0f));
  TEST_ASSERT_EQUAL_UINT16(0, brakeAmpsToCode(-0.001f));
  TEST_ASSERT_EQUAL_UINT16(0xFFFF, brakeAmpsToCode(0.049f));  // the rating, clamped
  TEST_ASSERT_EQUAL_UINT16(0xFFFF, brakeAmpsToCode(1.0f));
  TEST_ASSERT_UINT16_WITHIN(1, 0x8000, brakeAmpsToCode(kBrakeFullScaleAmps / 2.0f));
  for (uint16_t code = 0; code < 0xFF00; code += 0x0FFF) {
    TEST_ASSERT_UINT16_WITHIN(1, code, brakeAmpsToCode(brakeCodeToAmps(code)));
  }
}

void test_brake_isense_scale(void) {
  // 10 ohm x 4.125 x 1/2 = 20.625 V/A: 1.006 V at full scale.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 20.625f, kBrakeIsenseVoltsPerAmp);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.006f, kBrakeFullScaleAmps * kBrakeIsenseVoltsPerAmp);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.04878f, brakeIsenseVoltsToAmps(1.006f));
  // Through the RP2040 ADC: 1248 counts of 4095 at 3.3 V is 1.0057 V.
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.04876f, brakeIsenseCountsToAmps(1248));
  // The ceiling reads above full scale, so a limit event is visible.
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.163f, kBrakeCeilingAmps * kBrakeIsenseVoltsPerAmp);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 3.5f, brakeAmpsToTorqueNcm(0.049f));
}

void test_torque_front_end_design_values(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 191.73f, kTorqueInaGain);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.6667f, kTorqueZeroVolts);
  // 15 mV x 191.73 / 3 over 3.5309 N.cm.
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.2715f, kTorqueVoltsPerNcm);
  TEST_ASSERT_FLOAT_WITHIN(1e-9f, 78.125e-6f, kTorqueAdcVoltsPerCode);
  TEST_ASSERT_EQUAL_UINT16(21333, kTorqueZeroCodeNominal);
  // +/-rated and +/-150 % both fit the 0-5.12 V range.
  const float rated_lo = kTorqueZeroVolts - kTorqueRatedNcm * kTorqueVoltsPerNcm;
  const float rated_hi = kTorqueZeroVolts + kTorqueRatedNcm * kTorqueVoltsPerNcm;
  TEST_ASSERT_FLOAT_WITHIN(2e-3f, 0.708f, rated_lo);
  TEST_ASSERT_FLOAT_WITHIN(2e-3f, 2.625f, rated_hi);
  TEST_ASSERT_TRUE(kTorqueZeroVolts - 1.5f * kTorqueRatedNcm * kTorqueVoltsPerNcm > 0.0f);
  TEST_ASSERT_TRUE(kTorqueZeroVolts + 1.5f * kTorqueRatedNcm * kTorqueVoltsPerNcm <
                   kTorqueAdcFullScaleVolts);
}

void test_torque_code_to_ncm(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.66664f, torqueAdcCodeToVolts(21333));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, torqueAdcCodeToNcm(21333));
  // One N.cm is 0.2715 V = 3475 codes above zero.
  TEST_ASSERT_FLOAT_WITHIN(2e-3f, 1.0f, torqueAdcCodeToNcm(21333 + 3475));
  TEST_ASSERT_FLOAT_WITHIN(2e-3f, -1.0f, torqueAdcCodeToNcm(21333 - 3475));
  // A measured tare moves the zero, not the scale.
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, torqueAdcCodeToNcm(21500, 21500));
  TEST_ASSERT_FLOAT_WITHIN(2e-3f, 1.0f, torqueAdcCodeToNcm(21500 + 3475, 21500));
}

void test_torque_shunt_cal_step(void) {
  // 350 / (2 (2 x 75k + 350)) = 1.164 mV/V -> 0.744 V at AMP_VOUT.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.164e-3f, kTorqueShuntCalVoltsPerVolt);
  TEST_ASSERT_FLOAT_WITHIN(2e-3f, 0.744f, kTorqueShuntCalStepVolts);
  // That is 77.6 % of the nominal rated output.
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 0.776f,
                           kTorqueShuntCalVoltsPerVolt / kTorqueSensitivityNominalVPerV);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_degrees_round_trip_through_microsteps);
  RUN_TEST(test_counts_to_volts_endpoints);
  RUN_TEST(test_counts_to_volts_midscale);
  RUN_TEST(test_counts_to_volts_survives_zero_full_scale);
  RUN_TEST(test_sw_rail_undoes_the_divider);
  RUN_TEST(test_sw_rail_off_reads_below_threshold);
  RUN_TEST(test_hv_sense_is_raw_volts);
  RUN_TEST(test_full_scale_matches_declared_resolution);
  RUN_TEST(test_board_constants);
  RUN_TEST(test_brake_full_scale_is_the_designed_48_78_ma);
  RUN_TEST(test_brake_code_to_amps_endpoints);
  RUN_TEST(test_brake_amps_to_code_round_trip_and_clamp);
  RUN_TEST(test_brake_isense_scale);
  RUN_TEST(test_torque_front_end_design_values);
  RUN_TEST(test_torque_code_to_ncm);
  RUN_TEST(test_torque_shunt_cal_step);
  return UNITY_END();
}
