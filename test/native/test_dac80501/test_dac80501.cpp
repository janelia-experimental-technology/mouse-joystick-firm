// Host tests for the DACx0501 register map and output arithmetic.
//
// Every expected value here is computed from TI SBAS794E independently of the
// implementation, or is a reset value quoted directly from section 8.6. A
// failure therefore means the code disagrees with the datasheet rather than
// with itself.
//
// The reason to test this at all: a DAC's output is a number a researcher reads
// off a recording weeks later. A gain field set the wrong way, or a code scaled
// by the wrong denominator, produces a perfectly plausible trace that is simply
// wrong by a factor of two, and nothing downstream can detect it.

#include <unity.h>

#include "DAC80501Registers.hpp"

using namespace dac80501;

void setUp(void) {}
void tearDown(void) {}

// ---------------------------------------------------------------------------
// Register addresses (datasheet table 8-4)
// ---------------------------------------------------------------------------

void test_register_addresses(void) {
  TEST_ASSERT_EQUAL_HEX8(0x00, kRegNoop);
  TEST_ASSERT_EQUAL_HEX8(0x01, kRegDevid);
  TEST_ASSERT_EQUAL_HEX8(0x02, kRegSync);
  TEST_ASSERT_EQUAL_HEX8(0x03, kRegConfig);
  TEST_ASSERT_EQUAL_HEX8(0x04, kRegGain);
  TEST_ASSERT_EQUAL_HEX8(0x05, kRegTrigger);
  // Offset 6h does not exist in the map. STATUS is 7h, not 6h -- an off-by-one
  // here would read a reserved address and return nothing useful.
  TEST_ASSERT_EQUAL_HEX8(0x07, kRegStatus);
  TEST_ASSERT_EQUAL_HEX8(0x08, kRegDacData);
}

void test_reset_values(void) {
  // The one that carries the safety argument: GAIN comes up as BUFF-GAIN 1,
  // REF-DIV 0, which is a 5.0 V full scale from the internal 2.5 V reference.
  TEST_ASSERT_EQUAL_HEX16(0x0001, kGainReset);
  TEST_ASSERT_EQUAL_FLOAT(5.0f, fullScaleVolts(kGainReset));

  TEST_ASSERT_EQUAL_HEX16(0x0000, kConfigReset);
  TEST_ASSERT_EQUAL_HEX16(0x0000, kSyncReset);
  TEST_ASSERT_EQUAL_HEX16(0x0000, kTriggerReset);
  TEST_ASSERT_EQUAL_HEX16(0x0000, kStatusReset);
  // The fitted part is a DAC80501Z: zero-scale reset, so the output is 0 V
  // until data is written even though GAIN is wrong for this board at that
  // point. An M part would sit at midscale of a 5 V span, i.e. 2.5 V.
  TEST_ASSERT_EQUAL_HEX16(0x0000, kDacDataResetZ);
  TEST_ASSERT_EQUAL_HEX16(0x8000, kDacDataResetM);
}

void test_field_bit_positions(void) {
  TEST_ASSERT_EQUAL_HEX16(1u << 8, kGainRefDiv);
  TEST_ASSERT_EQUAL_HEX16(1u << 0, kGainBuffGain);
  TEST_ASSERT_EQUAL_HEX16(1u << 8, kConfigRefPwdwn);
  TEST_ASSERT_EQUAL_HEX16(1u << 0, kConfigDacPwdwn);
  TEST_ASSERT_EQUAL_HEX16(1u << 4, kTriggerLdac);
  TEST_ASSERT_EQUAL_HEX16(0x000A, kTriggerSoftResetCode);
  TEST_ASSERT_EQUAL_HEX16(1u << 0, kStatusRefAlarm);
  TEST_ASSERT_EQUAL_HEX16(1u << 0, kSyncDacSyncEn);
}

// ---------------------------------------------------------------------------
// I2C addressing (datasheet tables 8-2 and 8-3)
// ---------------------------------------------------------------------------

void test_i2c_addresses(void) {
  // Upper four bits factory-preset to 1001, low three from the A0 tie.
  TEST_ASSERT_EQUAL_HEX8(0x48, kI2cAddrA0Agnd);
  TEST_ASSERT_EQUAL_HEX8(0x49, kI2cAddrA0Vdd);
  TEST_ASSERT_EQUAL_HEX8(0x4A, kI2cAddrA0Sda);
  TEST_ASSERT_EQUAL_HEX8(0x4B, kI2cAddrA0Scl);
  // All four must share the 1001 prefix, or one of them is a typo.
  TEST_ASSERT_EQUAL_HEX8(0x48, kI2cAddrA0Agnd & 0x78);
  TEST_ASSERT_EQUAL_HEX8(0x48, kI2cAddrA0Scl & 0x78);
}

// ---------------------------------------------------------------------------
// Full scale
// ---------------------------------------------------------------------------

void test_full_scale_the_three_advertised_ranges(void) {
  // The datasheet advertises 5 V, 2.5 V and 1.25 V buffered output ranges from
  // the internal 2.5 V reference. All three must be reachable.
  TEST_ASSERT_EQUAL_FLOAT(5.0f, fullScaleVolts(kGainBuffGain));
  TEST_ASSERT_EQUAL_FLOAT(2.5f, fullScaleVolts(0x0000));
  TEST_ASSERT_EQUAL_FLOAT(2.5f, fullScaleVolts(kGainRefDiv | kGainBuffGain));
  TEST_ASSERT_EQUAL_FLOAT(1.25f, fullScaleVolts(kGainRefDiv));
}

void test_the_two_ways_to_get_2v5_agree(void) {
  // 0x0000 (2.5 V reference, gain 1) and 0x0101 (1.25 V reference, gain 2) are
  // different signal paths that produce the same span. mouse-joystick-pcb's
  // plan specifies 0x0101; this pins that they really are equal in span, so a
  // later switch between them is a noise decision and not a calibration one.
  TEST_ASSERT_EQUAL_FLOAT(fullScaleVolts(0x0000),
                          fullScaleVolts(kGainRefDiv | kGainBuffGain));
  TEST_ASSERT_EQUAL_FLOAT(voltsPerCode(0x0000),
                          voltsPerCode(kGainRefDiv | kGainBuffGain));
}

void test_full_scale_with_an_external_reference(void) {
  // Boards may drive VREFIO. 3.0 V external, gain 1, no divide.
  TEST_ASSERT_EQUAL_FLOAT(3.0f, fullScaleVolts(0x0000, 3.0f));
  TEST_ASSERT_EQUAL_FLOAT(6.0f, fullScaleVolts(kGainBuffGain, 3.0f));
  TEST_ASSERT_EQUAL_FLOAT(1.5f, fullScaleVolts(kGainRefDiv, 3.0f));
}

// ---------------------------------------------------------------------------
// Code and voltage
// ---------------------------------------------------------------------------

void test_code_to_volts(void) {
  const uint16_t gain = kGainRefDiv | kGainBuffGain; // 2.5 V full scale

  TEST_ASSERT_EQUAL_FLOAT(0.0f, codeToVolts(0, gain));
  // Midscale is exactly half of full scale, because the denominator is 65536
  // and not 65535.
  TEST_ASSERT_EQUAL_FLOAT(1.25f, codeToVolts(0x8000, gain));
  // Full-scale code is one LSB short of full scale. Getting this wrong by using
  // 65535 as the denominator is the classic DAC arithmetic error; it puts every
  // reading out by one LSB and is invisible without a reference measurement.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.5f * 65535.0f / 65536.0f,
                           codeToVolts(0xFFFF, gain));
}

void test_volts_per_code(void) {
  // 2.5 V over 65536 codes is 38.1 uV. That figure is what a span argument
  // turns on, so it is worth pinning independently.
  TEST_ASSERT_FLOAT_WITHIN(1e-9f, 2.5f / 65536.0f,
                           voltsPerCode(kGainRefDiv | kGainBuffGain));
  TEST_ASSERT_FLOAT_WITHIN(1e-9f, 5.0f / 65536.0f, voltsPerCode(kGainReset));
}

void test_volts_to_code_round_trips(void) {
  const uint16_t gain = 0x0000; // 2.5 V full scale
  TEST_ASSERT_EQUAL_UINT16(0, voltsToCode(0.0f, gain));
  TEST_ASSERT_EQUAL_UINT16(0x8000, voltsToCode(1.25f, gain));
  // Round trip a spread of codes through volts and back.
  const uint16_t codes[] = {1, 137, 4096, 0x4000, 0xC000, 0xFFFE};
  for (unsigned i = 0; i < sizeof(codes) / sizeof(codes[0]); ++i) {
    const float volts = codeToVolts(codes[i], gain);
    TEST_ASSERT_EQUAL_UINT16(codes[i], voltsToCode(volts, gain));
  }
}

void test_volts_to_code_rounds_to_nearest(void) {
  const uint16_t gain = 0x0000;
  const float lsb = voltsPerCode(gain);
  // Just under and just over the midpoint between codes 100 and 101.
  TEST_ASSERT_EQUAL_UINT16(100, voltsToCode(100.0f * lsb + 0.49f * lsb, gain));
  TEST_ASSERT_EQUAL_UINT16(101, voltsToCode(100.0f * lsb + 0.51f * lsb, gain));
}

void test_volts_to_code_clamps_rather_than_wrapping(void) {
  const uint16_t gain = 0x0000; // 2.5 V full scale
  // Above full scale clamps to the top code. Wrapping here would turn an
  // over-range request into a reading near zero, which under an angle output is
  // the difference between "at the end of travel" and "at the start".
  TEST_ASSERT_EQUAL_UINT16(kCodeMax, voltsToCode(2.5f, gain));
  TEST_ASSERT_EQUAL_UINT16(kCodeMax, voltsToCode(10.0f, gain));
  // Negative and zero clamp to the bottom.
  TEST_ASSERT_EQUAL_UINT16(0, voltsToCode(-1.0f, gain));
  TEST_ASSERT_EQUAL_UINT16(0, voltsToCode(0.0f, gain));
}

void test_volts_to_code_survives_a_degenerate_gain(void) {
  // A zero or negative reference is a caller error, but it must not divide by
  // zero on a board that is mid-bring-up.
  TEST_ASSERT_EQUAL_UINT16(0, voltsToCode(1.0f, 0x0000, 0.0f));
  TEST_ASSERT_EQUAL_UINT16(0, voltsToCode(1.0f, 0x0000, -2.5f));
}

// ---------------------------------------------------------------------------
// DEVID decode
// ---------------------------------------------------------------------------

void test_devid_reset_values(void) {
  TEST_ASSERT_EQUAL_HEX16(0x0115, kDevidReset80501Z);
  TEST_ASSERT_EQUAL_HEX16(0x0195, kDevidReset80501M);
  TEST_ASSERT_EQUAL_HEX16(0x1115, kDevidReset70501Z);
  TEST_ASSERT_EQUAL_HEX16(0x2115, kDevidReset60501Z);
}

void test_devid_decodes_resolution_and_reset_scale(void) {
  TEST_ASSERT_EQUAL(Resolution::kBits16,
                    resolutionFromDevid(kDevidReset80501Z));
  TEST_ASSERT_EQUAL(Resolution::kBits14,
                    resolutionFromDevid(kDevidReset70501Z));
  TEST_ASSERT_EQUAL(Resolution::kBits12,
                    resolutionFromDevid(kDevidReset60501Z));

  // The fitted part is a Z: zero-scale reset.
  TEST_ASSERT_FALSE(devidResetsToMidscale(kDevidReset80501Z));
  TEST_ASSERT_TRUE(devidResetsToMidscale(kDevidReset80501M));
}

void test_devid_plausibility_rejects_a_broken_bus(void) {
  TEST_ASSERT_TRUE(devidIsPlausible(kDevidReset80501Z));
  TEST_ASSERT_TRUE(devidIsPlausible(kDevidReset80501M));
  TEST_ASSERT_TRUE(devidIsPlausible(kDevidReset70501Z));
  TEST_ASSERT_TRUE(devidIsPlausible(kDevidReset60501Z));

  // The two dead-bus signatures, which is the whole point of a plausibility
  // check rather than an equality check against one part number.
  TEST_ASSERT_FALSE(devidIsPlausible(0x0000));
  TEST_ASSERT_FALSE(devidIsPlausible(0xFFFF));
  // Reserved fields wrong: DB15 set, DB[11:8] not 1h, DB[6:0] not 15h.
  TEST_ASSERT_FALSE(devidIsPlausible(0x8115));
  TEST_ASSERT_FALSE(devidIsPlausible(0x0215));
  TEST_ASSERT_FALSE(devidIsPlausible(0x0114));
  // A resolution field of 3h or above is not a member of this family.
  TEST_ASSERT_FALSE(devidIsPlausible(0x3115));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_register_addresses);
  RUN_TEST(test_reset_values);
  RUN_TEST(test_field_bit_positions);
  RUN_TEST(test_i2c_addresses);
  RUN_TEST(test_full_scale_the_three_advertised_ranges);
  RUN_TEST(test_the_two_ways_to_get_2v5_agree);
  RUN_TEST(test_full_scale_with_an_external_reference);
  RUN_TEST(test_code_to_volts);
  RUN_TEST(test_volts_per_code);
  RUN_TEST(test_volts_to_code_round_trips);
  RUN_TEST(test_volts_to_code_rounds_to_nearest);
  RUN_TEST(test_volts_to_code_clamps_rather_than_wrapping);
  RUN_TEST(test_volts_to_code_survives_a_degenerate_gain);
  RUN_TEST(test_devid_reset_values);
  RUN_TEST(test_devid_decodes_resolution_and_reset_scale);
  RUN_TEST(test_devid_plausibility_rejects_a_broken_bus);
  return UNITY_END();
}
