// Host tests for the ADS8689 framing, register map, transfer function and
// device logic, against a fake transport.
//
// Every expected value is computed from TI SBAS633E independently of the
// implementation. The properties worth pinning: a command word's bit layout
// (an off-by-one in the opcode shift addresses a different register and the
// device says nothing), the two-frame register read (a one-frame "read"
// returns a conversion result that looks plausible), and the range written
// and read back before any conversion is believed (the reset range is
// +/-12.288 V, a silent 4.8x scale error on a 0-5 V signal).

#include <unity.h>

#include <vector>

#include "ADS8689Device.hpp"
#include "ADS8689Registers.hpp"

using namespace ads8689;

namespace {

// Models the part's shift register: the word shifted OUT during a frame was
// decided by the PREVIOUS frame (a READ_HWORD, a DATA_VAL pattern, or the
// conversion). Writes land in a register file indexed by half-word address.
class FakeTransport : public Transport {
public:
  uint32_t transferFrame(uint32_t out) override {
    frames.push_back(out);
    const uint32_t response = next_out_;
    const uint32_t op = (out >> 25) & kOpcodeMask;
    const uint16_t addr = static_cast<uint16_t>((out >> 16) & kAddressMask);
    const uint16_t data = static_cast<uint16_t>(out & 0xFFFFu);
    const uint16_t hw = static_cast<uint16_t>(addr & ~1u);
    if (hw < kRegs) {
      if (op == kOpWriteHword) {
        regs_[hw] = data;
      } else if (op == kOpSetHword) {
        regs_[hw] |= data;
      } else if (op == kOpClearHword) {
        regs_[hw] &= static_cast<uint16_t>(~data);
      }
    }
    ++conversions_started;
    if (op == kOpReadHword) {
      next_out_ = static_cast<uint32_t>(hw < kRegs ? regs_[hw] : 0u) << 16;
    } else {
      next_out_ = patternOrConversion();
    }
    return sdo_stuck_low ? 0u : response;
  }

  void pulseConvst() override {
    ++pulses;
    ++conversions_started;
    next_out_ = patternOrConversion();
  }

  uint16_t reg(uint8_t hw) const { return regs_[hw]; }

  std::vector<uint32_t> frames;
  unsigned pulses = 0;
  unsigned conversions_started = 0;
  uint16_t conversion = 0x5333u; // 21299, about 1.664 V on the 5.12 V range
  bool sdo_stuck_low = false;
  bool corrupt_ones_pattern = false;

private:
  static constexpr uint16_t kRegs = 0x30;

  uint32_t patternOrConversion() const {
    const uint16_t data_val = regs_[kRegDataoutCtl] & kDataValMask;
    uint16_t upper = conversion;
    if (data_val == kDataValAllZeros) {
      upper = 0x0000u;
    } else if (data_val == kDataValAllOnes) {
      upper = corrupt_ones_pattern ? 0x7FFFu : 0xFFFFu;
    } else if (data_val == kDataValAlternatingBits) {
      upper = 0xAAAAu;
    } else if (data_val == kDataValAlternatingPairs) {
      upper = 0xCCCCu;
    }
    return static_cast<uint32_t>(upper) << 16;
  }

  uint16_t regs_[kRegs] = {0};
  uint32_t next_out_ = 0;
};

} // namespace

void setUp(void) {}
void tearDown(void) {}

// ---------------------------------------------------------------------------
// Framing (section 7.5.2, table 7-5)
// ---------------------------------------------------------------------------

void test_write_frame_layout(void) {
  // WRITE = 11010_00, RANGE_SEL = 0x14, data 0x000B (0 to 1.25 x VREF):
  // 1101000 0_0001 0100 0000 0000 0000 1011 = 0xD014000B.
  TEST_ASSERT_EQUAL_HEX32(0xD014000Bu, writeFrame(kRegRangeSel, 0x000Bu));
}

void test_read_frame_layout(void) {
  // READ_HWORD = 11001_00 -> 0xC8000000 | address << 16, data zero.
  TEST_ASSERT_EQUAL_HEX32(0xC8140000u, readFrame(kRegRangeSel));
  TEST_ASSERT_EQUAL_HEX32(0xC8000000u, readFrame(kRegDeviceId));
}

void test_set_and_clear_frames(void) {
  TEST_ASSERT_EQUAL_HEX32(0xD8046901u,
                          setBitsFrame(kRegRstPwrctl, 0x6901u)); // 11011_00
  TEST_ASSERT_EQUAL_HEX32(0xC0100007u,
                          clearBitsFrame(kRegDataoutCtl, 0x0007u)); // 11000_00
  TEST_ASSERT_EQUAL_HEX32(0x00000000u, kFrameNop);
}

void test_address_is_nine_bits(void) {
  // A 9-bit address field: a register address above 0xFF cannot appear, and
  // bit 24 (the leading zero) stays clear for every real register.
  TEST_ASSERT_EQUAL_HEX32(0u, writeFrame(kRegAlarmLTh, 0) & (1u << 24));
  TEST_ASSERT_EQUAL_HEX32(0xC8280000u, readFrame(kRegAlarmLTh));
}

void test_upper_half_carries_the_payload(void) {
  TEST_ASSERT_EQUAL_HEX16(0x1234u, upperHalf(0x12340000u));
  TEST_ASSERT_EQUAL_HEX16(0xFFFFu, upperHalf(0xFFFF0000u));
  TEST_ASSERT_EQUAL_HEX16(0x0000u, upperHalf(0x0000FFFFu));
}

// ---------------------------------------------------------------------------
// Register map (table 7-10) and field constants
// ---------------------------------------------------------------------------

void test_register_addresses(void) {
  TEST_ASSERT_EQUAL_HEX8(0x00, kRegDeviceId);
  TEST_ASSERT_EQUAL_HEX8(0x04, kRegRstPwrctl);
  TEST_ASSERT_EQUAL_HEX8(0x08, kRegSdiCtl);
  TEST_ASSERT_EQUAL_HEX8(0x0C, kRegSdoCtl);
  TEST_ASSERT_EQUAL_HEX8(0x10, kRegDataoutCtl);
  TEST_ASSERT_EQUAL_HEX8(0x14, kRegRangeSel);
  TEST_ASSERT_EQUAL_HEX8(0x20, kRegAlarm);
  TEST_ASSERT_EQUAL_HEX8(0x24, kRegAlarmHTh);
  TEST_ASSERT_EQUAL_HEX8(0x28, kRegAlarmLTh);
}

void test_rst_pwrctl_key_and_bits(void) {
  TEST_ASSERT_EQUAL_HEX16(0x6900u, kRstPwrctlKey);
  TEST_ASSERT_EQUAL_HEX16(0x0001u, kRstPwrctlPwrdn);
  TEST_ASSERT_EQUAL_HEX16(0x0002u, kRstPwrctlNapEn);
  TEST_ASSERT_EQUAL_HEX16(0x0004u, kRstPwrctlRstnApp);
  TEST_ASSERT_EQUAL_HEX16(0x0010u, kRstPwrctlInAlDis);
  TEST_ASSERT_EQUAL_HEX16(0x0020u, kRstPwrctlVddAlDis);
}

void test_range_codes(void) {
  TEST_ASSERT_EQUAL_HEX16(0x0u, kRangeBipolar3);
  TEST_ASSERT_EQUAL_HEX16(0x3u, kRangeBipolar1p25);
  TEST_ASSERT_EQUAL_HEX16(0x4u, kRangeBipolar0p625);
  TEST_ASSERT_EQUAL_HEX16(0x8u, kRangeUnipolar3);
  TEST_ASSERT_EQUAL_HEX16(0xBu, kRangeUnipolar1p25);
  TEST_ASSERT_EQUAL_HEX16(0x0040u, kRangeIntrefDis);
  TEST_ASSERT_EQUAL_HEX16(0x0000u, kRangeSelReset);
  TEST_ASSERT_TRUE(rangeIsUnipolar(kRangeUnipolar1p25));
  TEST_ASSERT_FALSE(rangeIsUnipolar(kRangeBipolar1p25));
}

void test_dataout_patterns(void) {
  TEST_ASSERT_EQUAL_HEX16(0x4u, kDataValAllZeros);
  TEST_ASSERT_EQUAL_HEX16(0x5u, kDataValAllOnes);
  TEST_ASSERT_EQUAL_HEX16(0x6u, kDataValAlternatingBits);
  TEST_ASSERT_EQUAL_HEX16(0x7u, kDataValAlternatingPairs);
  TEST_ASSERT_EQUAL_HEX16(0x0008u, kDataoutParEn);
  TEST_ASSERT_EQUAL_HEX16(0x0100u, kDataoutRangeIncl);
  TEST_ASSERT_EQUAL_HEX16(0x4000u, kDataoutDeviceAddrIncl);
}

void test_timing_constants(void) {
  // ADS8689 column: 5 us conversion, 5 us acquisition, 100 kSPS.
  TEST_ASSERT_EQUAL_UINT32(5000u, kConversionTimeMaxNs);
  TEST_ASSERT_EQUAL_UINT32(5000u, kAcquisitionTimeMinNs);
  TEST_ASSERT_EQUAL_UINT32(100000u, kSampleRateMaxHz);
  TEST_ASSERT_EQUAL_UINT32(66670000u, kSclkMaxHz);
}

// ---------------------------------------------------------------------------
// Transfer function
// ---------------------------------------------------------------------------

void test_unipolar_1p25_range_arithmetic(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 5.12f, fullScaleVolts(kRangeUnipolar1p25));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 5.12f, spanVolts(kRangeUnipolar1p25));
  TEST_ASSERT_FLOAT_WITHIN(1e-9f, 78.125e-6f, voltsPerCode(kRangeUnipolar1p25));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, codeToVolts(0, kRangeUnipolar1p25));
  // 21333 codes is the torque bridge's nominal zero, 1.6667 V.
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.66664f, codeToVolts(21333, kRangeUnipolar1p25));
  TEST_ASSERT_EQUAL_UINT16(21333, voltsToCode(1.66664f, kRangeUnipolar1p25));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 5.12f - 78.125e-6f,
                           codeToVolts(0xFFFF, kRangeUnipolar1p25));
  TEST_ASSERT_EQUAL_UINT16(0xFFFF, voltsToCode(6.0f, kRangeUnipolar1p25));
  TEST_ASSERT_EQUAL_UINT16(0, voltsToCode(-1.0f, kRangeUnipolar1p25));
}

void test_bipolar_reset_range_arithmetic(void) {
  // The reset range: +/-12.288 V, 24.576 V over 65536 codes = 375 uV.
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 12.288f, fullScaleVolts(kRangeBipolar3));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 24.576f, spanVolts(kRangeBipolar3));
  TEST_ASSERT_FLOAT_WITHIN(1e-8f, 375e-6f, voltsPerCode(kRangeBipolar3));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, codeToVolts(0x8000, kRangeBipolar3));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -12.288f, codeToVolts(0x0000, kRangeBipolar3));
  TEST_ASSERT_EQUAL_UINT16(0x8000, voltsToCode(0.0f, kRangeBipolar3));
  // And the point of writing RANGE_SEL first: 1.6667 V read in the reset
  // range is code 37213, which scaled as 0-5.12 V would claim 2.907 V.
  TEST_ASSERT_EQUAL_UINT16(37213, voltsToCode(1.6667f, kRangeBipolar3));
}

void test_invalid_range_codes_have_no_span(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, rangeMultiplier(0x5));
  TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, rangeMultiplier(0xF));
}

// ---------------------------------------------------------------------------
// Device logic
// ---------------------------------------------------------------------------

void test_register_read_takes_exactly_two_frames(void) {
  FakeTransport t;
  Ads8689 adc(t);
  adc.writeRegister(kRegRangeSel, 0x000Bu);
  const size_t before = t.frames.size();
  const uint16_t value = adc.readRegister(kRegRangeSel);
  TEST_ASSERT_EQUAL_UINT32(before + 2, t.frames.size());
  TEST_ASSERT_EQUAL_HEX32(readFrame(kRegRangeSel), t.frames[before]);
  TEST_ASSERT_EQUAL_HEX32(kFrameNop, t.frames[before + 1]);
  TEST_ASSERT_EQUAL_HEX16(0x000Bu, value);
}

void test_begin_writes_range_and_reads_it_back(void) {
  FakeTransport t;
  Ads8689 adc(t);
  BeginReport report;
  Config config;
  config.range = kRangeUnipolar1p25;
  TEST_ASSERT_TRUE(adc.begin(config, &report));
  TEST_ASSERT_TRUE(adc.configured());
  TEST_ASSERT_EQUAL(static_cast<int>(BeginResult::kOk), static_cast<int>(report.result));
  TEST_ASSERT_EQUAL_HEX16(0x000Bu, t.reg(kRegRangeSel));
  TEST_ASSERT_EQUAL_HEX16(0x000Bu, report.range_written);
  TEST_ASSERT_EQUAL_HEX16(0x000Bu, report.range_readback);
  // The protocol registers were put at their reset values first.
  TEST_ASSERT_EQUAL_HEX32(writeFrame(kRegSdiCtl, kSdiModeSpi00), t.frames[0]);
  TEST_ASSERT_EQUAL_HEX32(writeFrame(kRegSdoCtl, kSdoCtlDefault), t.frames[1]);
  TEST_ASSERT_EQUAL_HEX32(writeFrame(kRegDataoutCtl, kDataValConversion), t.frames[2]);
  TEST_ASSERT_EQUAL_HEX32(writeFrame(kRegRangeSel, 0x000Bu), t.frames[3]);
  // The loopback ran and was clean, and DATAOUT_CTL was left at conversion data.
  TEST_ASSERT_EQUAL_HEX16(0xFFFFu, report.loopback_ones);
  TEST_ASSERT_EQUAL_HEX16(0x0000u, report.loopback_zeros);
  TEST_ASSERT_EQUAL_HEX16(kDataValConversion, t.reg(kRegDataoutCtl) & kDataValMask);
  TEST_ASSERT_EQUAL_HEX16(kDataValConversion, report.dataout_readback & kDataValMask);
}

void test_begin_refuses_when_nothing_answers(void) {
  // SDO held low by R2012 with no part driving it: every read is 0, the
  // range read-back is 0 against the 0x0B written, and nothing is believed.
  FakeTransport t;
  t.sdo_stuck_low = true;
  Ads8689 adc(t);
  BeginReport report;
  TEST_ASSERT_FALSE(adc.begin(Config{}, &report));
  TEST_ASSERT_FALSE(adc.configured());
  TEST_ASSERT_EQUAL(static_cast<int>(BeginResult::kRangeReadbackMismatch),
                    static_cast<int>(report.result));
  TEST_ASSERT_EQUAL_HEX16(0x0000u, report.range_readback);
}

void test_begin_refuses_a_corrupt_loopback(void) {
  FakeTransport t;
  t.corrupt_ones_pattern = true; // one SDO bit never comes back high
  Ads8689 adc(t);
  BeginReport report;
  TEST_ASSERT_FALSE(adc.begin(Config{}, &report));
  TEST_ASSERT_FALSE(adc.configured());
  TEST_ASSERT_EQUAL(static_cast<int>(BeginResult::kLoopbackOnesFailed),
                    static_cast<int>(report.result));
  TEST_ASSERT_EQUAL_HEX16(0x7FFFu, report.loopback_ones);
  // The pattern register must not be left selecting a fixed pattern.
  TEST_ASSERT_EQUAL_HEX16(kDataValConversion, t.reg(kRegDataoutCtl) & kDataValMask);
}

void test_loopback_can_be_skipped(void) {
  FakeTransport t;
  t.corrupt_ones_pattern = true;
  Ads8689 adc(t);
  Config config;
  config.loopback_check = false;
  BeginReport report;
  TEST_ASSERT_TRUE(adc.begin(config, &report));
  TEST_ASSERT_EQUAL_HEX16(0u, report.loopback_ones);
}

void test_sample_pulses_then_reads(void) {
  FakeTransport t;
  Ads8689 adc(t);
  TEST_ASSERT_TRUE(adc.begin(Config{}));
  const size_t before = t.frames.size();
  const unsigned pulses_before = t.pulses;
  const uint16_t code = adc.sample();
  TEST_ASSERT_EQUAL_UINT32(pulses_before + 1, t.pulses);
  TEST_ASSERT_EQUAL_UINT32(before + 1, t.frames.size());
  TEST_ASSERT_EQUAL_HEX32(kFrameNop, t.frames[before]);
  TEST_ASSERT_EQUAL_HEX16(0x5333u, code);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.66398f, adc.codeToVolts(code));
}

void test_read_last_is_one_frame(void) {
  FakeTransport t;
  Ads8689 adc(t);
  TEST_ASSERT_TRUE(adc.begin(Config{}));
  const size_t before = t.frames.size();
  TEST_ASSERT_EQUAL_HEX16(0x5333u, adc.readLast());
  TEST_ASSERT_EQUAL_UINT32(before + 1, t.frames.size());
}

void test_power_control_writes_carry_the_key(void) {
  FakeTransport t;
  Ads8689 adc(t);
  adc.writePowerControl(kRstPwrctlNapEn);
  TEST_ASSERT_EQUAL_HEX32(writeFrame(kRegRstPwrctl, 0x6902u), t.frames.back());
  TEST_ASSERT_EQUAL_HEX16(0x6902u, t.reg(kRegRstPwrctl));
}

void test_invalidate_forgets_without_a_frame(void) {
  FakeTransport t;
  Ads8689 adc(t);
  TEST_ASSERT_TRUE(adc.begin(Config{}));
  const size_t before = t.frames.size();
  adc.invalidate();
  TEST_ASSERT_FALSE(adc.configured());
  TEST_ASSERT_EQUAL_UINT32(before, t.frames.size());
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_write_frame_layout);
  RUN_TEST(test_read_frame_layout);
  RUN_TEST(test_set_and_clear_frames);
  RUN_TEST(test_address_is_nine_bits);
  RUN_TEST(test_upper_half_carries_the_payload);
  RUN_TEST(test_register_addresses);
  RUN_TEST(test_rst_pwrctl_key_and_bits);
  RUN_TEST(test_range_codes);
  RUN_TEST(test_dataout_patterns);
  RUN_TEST(test_timing_constants);
  RUN_TEST(test_unipolar_1p25_range_arithmetic);
  RUN_TEST(test_bipolar_reset_range_arithmetic);
  RUN_TEST(test_invalid_range_codes_have_no_span);
  RUN_TEST(test_register_read_takes_exactly_two_frames);
  RUN_TEST(test_begin_writes_range_and_reads_it_back);
  RUN_TEST(test_begin_refuses_when_nothing_answers);
  RUN_TEST(test_begin_refuses_a_corrupt_loopback);
  RUN_TEST(test_loopback_can_be_skipped);
  RUN_TEST(test_sample_pulses_then_reads);
  RUN_TEST(test_read_last_is_one_frame);
  RUN_TEST(test_power_control_writes_carry_the_key);
  RUN_TEST(test_invalidate_forgets_without_a_frame);
  return UNITY_END();
}
