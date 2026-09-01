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

void test_board_constants(void) {
  TEST_ASSERT_EQUAL_UINT16(4095, kAdcFullScale);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.3f, kAdcReferenceVolts);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, kSwRailDividerRatio);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 249.0f, kHvSenseResistorOhms);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_counts_to_volts_endpoints);
  RUN_TEST(test_counts_to_volts_midscale);
  RUN_TEST(test_counts_to_volts_survives_zero_full_scale);
  RUN_TEST(test_sw_rail_undoes_the_divider);
  RUN_TEST(test_sw_rail_off_reads_below_threshold);
  RUN_TEST(test_hv_sense_is_raw_volts);
  RUN_TEST(test_board_constants);
  return UNITY_END();
}
