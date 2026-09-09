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

void test_ioin_invariants_accept_the_bare_board_reading(void) {
  TEST_ASSERT_TRUE(ioinInvariantsMatch(0x11000010u));
}

void test_ioin_invariants_ignore_swcomp(void) {
  // SWCOMP_IN is a comparator output and must not affect the verdict.
  TEST_ASSERT_TRUE(ioinInvariantsMatch(0x11000010u | (1u << 7)));
}

void test_ioin_invariants_accept_real_measured_boards(void) {
  // These are readings taken off real hardware, and the old whole-byte matcher
  // rejected every one of them. They must all pass: five IOIN bits are driven
  // by what is plugged into the motor, encoder and limit-switch connectors, so
  // no fixed expectation can be right.
  //
  // Limit switch UNPLUGGED. The MAX6816 drives its own pull-up high and there
  // is no LIMIT_SW pull-down on v0.3, so REFL_STEP = 1 is the only reading the
  // board can give. Measured on boards E4654C159F2C2534 and E4654C159F171E34.
  TEST_ASSERT_TRUE(ioinInvariantsMatch(0x11000011u));
  // Encoder connected, a legitimate quadrature state. Measured on board
  // E4654C159F4D1334.
  TEST_ASSERT_TRUE(ioinInvariantsMatch(0x11000014u));
  // Everything connected, switch at rest (normally closed, so REFL_STEP = 0)
  // and the encoder mid-count. Measured on board E4654C159F171E34, 2026-09-09.
  TEST_ASSERT_TRUE(ioinInvariantsMatch(0x11000018u));
  // Every connector-dependent bit high at once is still not a fault.
  TEST_ASSERT_TRUE(ioinInvariantsMatch(0x11000010u | kIoinConnectorMask));
}

void test_ioin_invariants_reject_real_faults(void) {
  // Wrong chip.
  TEST_ASSERT_FALSE(ioinInvariantsMatch(0x30000010u));
  // Driver reads enabled -- the case that must never pass silently, and the
  // reason bit 4 is asserted at all: it is the only read-back proving GP3
  // reaches the driver through the bus switch.
  TEST_ASSERT_FALSE(ioinInvariantsMatch(0x11000000u));
  // SD_MODE strapped high instead of to GND.
  TEST_ASSERT_FALSE(ioinInvariantsMatch(0x11000050u));
  // REFR_DIR high. That pin is tied to GND, so this is a real defect and not a
  // connector state.
  TEST_ASSERT_FALSE(ioinInvariantsMatch(0x11000012u));
  // All zeros and all ones, the two dead-bus signatures.
  TEST_ASSERT_FALSE(ioinInvariantsMatch(0x00000000u));
  TEST_ASSERT_FALSE(ioinInvariantsMatch(0xFFFFFFFFu));
}

void test_ioin_masks_do_not_overlap(void) {
  // Every low-byte bit is either asserted or explicitly connector-dependent,
  // and none is both. This is what stops a bit quietly slipping out of both
  // sets during a future edit.
  TEST_ASSERT_EQUAL_HEX32(0u, kIoinInvariantMask & kIoinConnectorMask);
  TEST_ASSERT_EQUAL_HEX32(
      0xFFu, (kIoinInvariantMask | kIoinConnectorMask) & 0xFFu);
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
  RUN_TEST(test_ioin_invariants_accept_the_bare_board_reading);
  RUN_TEST(test_ioin_invariants_ignore_swcomp);
  RUN_TEST(test_ioin_invariants_accept_real_measured_boards);
  RUN_TEST(test_ioin_invariants_reject_real_faults);
  RUN_TEST(test_ioin_masks_do_not_overlap);
  RUN_TEST(test_vsense);
  return UNITY_END();
}
