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
  return UNITY_END();
}
