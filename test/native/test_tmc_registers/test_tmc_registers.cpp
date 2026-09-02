// Host tests for the TMC5130A register decoding in src/TmcRegisters.hpp.
//
// The expected-IOIN value encodes a claim about the board, not about the chip:
// which of the driver's inputs are tied where. Getting it wrong would make
// stage 2 either miss a real defect or cry wolf, so it is worth pinning down
// away from hardware.

#include <unity.h>

#include "TmcRegisters.hpp"

using namespace mouse_joystick::tmc;

void setUp(void) {}
void tearDown(void) {}

void test_addresses_match_tmc51x0(void) {
  TEST_ASSERT_EQUAL_HEX8(0x00, kGconf);
  TEST_ASSERT_EQUAL_HEX8(0x01, kGstat);
  TEST_ASSERT_EQUAL_HEX8(0x04, kIoin);
  TEST_ASSERT_EQUAL_HEX8(0x21, kXactual);
  TEST_ASSERT_EQUAL_HEX8(0x39, kXenc);
  TEST_ASSERT_EQUAL_HEX8(0x6C, kChopconf);
  TEST_ASSERT_EQUAL_HEX8(0x6F, kDrvStatus);
}

void test_version_constants(void) {
  TEST_ASSERT_EQUAL_HEX8(0x11, kVersionTmc5130);
  TEST_ASSERT_EQUAL_HEX8(0x30, kVersionTmc5160);
}

void test_decode_expected_board_ioin(void) {
  // What this board should actually return: version 0x11, DRV_ENN high because
  // firmware holds it high, everything else pulled low.
  const IoinDecode d = decodeIoin(0x11000010u);
  TEST_ASSERT_EQUAL_HEX8(0x11, d.version);
  TEST_ASSERT_TRUE(d.drv_enn);
  TEST_ASSERT_FALSE(d.refl_step);
  TEST_ASSERT_FALSE(d.refr_dir);
  TEST_ASSERT_FALSE(d.enca_dcin_cfg5);
  TEST_ASSERT_FALSE(d.encb_dcen_cfg4);
  TEST_ASSERT_FALSE(d.enc_n_dco_cfg6);
  TEST_ASSERT_FALSE(d.sd_mode);
}

void test_decode_bit_positions_are_distinct(void) {
  // Walk one bit at a time; each must land in exactly the right field.
  TEST_ASSERT_TRUE(decodeIoin(1u << 0).refl_step);
  TEST_ASSERT_TRUE(decodeIoin(1u << 1).refr_dir);
  TEST_ASSERT_TRUE(decodeIoin(1u << 2).encb_dcen_cfg4);
  TEST_ASSERT_TRUE(decodeIoin(1u << 3).enca_dcin_cfg5);
  TEST_ASSERT_TRUE(decodeIoin(1u << 4).drv_enn);
  TEST_ASSERT_TRUE(decodeIoin(1u << 5).enc_n_dco_cfg6);
  TEST_ASSERT_TRUE(decodeIoin(1u << 6).sd_mode);
  TEST_ASSERT_TRUE(decodeIoin(1u << 7).swcomp_in);
  TEST_ASSERT_FALSE(decodeIoin(1u << 0).drv_enn);
  TEST_ASSERT_EQUAL_HEX8(0x00, decodeIoin(0x00FFFFFFu).version);
}

void test_expected_ioin_matcher_accepts_the_right_value(void) {
  TEST_ASSERT_TRUE(ioinMatchesExpected(0x11000010u));
}

void test_expected_ioin_matcher_ignores_swcomp(void) {
  // SWCOMP_IN is a comparator output and must not affect the verdict.
  TEST_ASSERT_TRUE(ioinMatchesExpected(0x11000010u | (1u << 7)));
}

void test_expected_ioin_matcher_rejects_real_faults(void) {
  // Wrong chip.
  TEST_ASSERT_FALSE(ioinMatchesExpected(0x30000010u));
  // Driver reads enabled -- the case that must never pass silently.
  TEST_ASSERT_FALSE(ioinMatchesExpected(0x11000000u));
  // SD_MODE strapped high instead of to GND.
  TEST_ASSERT_FALSE(ioinMatchesExpected(0x11000050u));
  // A limit switch or encoder input stuck high.
  TEST_ASSERT_FALSE(ioinMatchesExpected(0x11000011u));
  TEST_ASSERT_FALSE(ioinMatchesExpected(0x11000018u));
  // All zeros and all ones, the two dead-bus signatures.
  TEST_ASSERT_FALSE(ioinMatchesExpected(0x00000000u));
  TEST_ASSERT_FALSE(ioinMatchesExpected(0xFFFFFFFFu));
}

void test_vsense(void) {
  TEST_ASSERT_FALSE(vsenseSet(0x00000000u));
  TEST_ASSERT_TRUE(vsenseSet(1u << 17));
  TEST_ASSERT_FALSE(vsenseSet(1u << 16));
  TEST_ASSERT_FALSE(vsenseSet(1u << 18));
  // A CHOPCONF value with vsense clear is the dangerous case: with this board's
  // 0.15 ohm shunts that is roughly 1.8x the motor's rated current, and stage 3
  // must fix it before enabling the driver.
  TEST_ASSERT_FALSE(vsenseSet(0x10410150u));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_addresses_match_tmc51x0);
  RUN_TEST(test_version_constants);
  RUN_TEST(test_decode_expected_board_ioin);
  RUN_TEST(test_decode_bit_positions_are_distinct);
  RUN_TEST(test_expected_ioin_matcher_accepts_the_right_value);
  RUN_TEST(test_expected_ioin_matcher_ignores_swcomp);
  RUN_TEST(test_expected_ioin_matcher_rejects_real_faults);
  RUN_TEST(test_vsense);
  return UNITY_END();
}
