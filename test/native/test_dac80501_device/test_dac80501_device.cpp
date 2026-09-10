// Host tests for the DACx0501 device logic, against a fake transport.
//
// The property worth testing here is the ORDERING INTERLOCK. GAIN resets to a
// 5.0 V full scale; a board wanting 2.5 V that writes DAC data before writing
// GAIN presents double the intended voltage, and on an angle output that means
// a researcher's recorded trace is wrong by a factor of two with nothing
// downstream able to notice. The interlock makes that unrepresentable, and a
// property like that should be pinned by a test rather than by one manual poke
// at a bench console.
//
// The fake also lets the I2C paths be tested at all. mouse-joystick-pcb v0.3 is
// strapped for SPI, so `I2cTransport` has no hardware to run against on this
// bench -- these tests are the only coverage the readback logic has until a
// board straps SPI2C the other way.

#include <unity.h>

#include <vector>

#include "DAC80501Device.hpp"

using namespace dac80501;

namespace {

struct Frame {
  uint8_t reg;
  uint16_t value;
};

// Records every write, answers reads from a register file, and can be told to
// fail a specific write so the failure paths are reachable.
class FakeTransport : public Transport {
public:
  explicit FakeTransport(bool can_read) : can_read_(can_read) {}

  bool writeRegister(uint8_t reg, uint16_t value) override {
    if (fail_on_reg_valid_ && reg == fail_on_reg_) {
      return false;
    }
    writes.push_back({reg, value});
    regs_[reg] = value;
    return true;
  }

  bool readRegister(uint8_t reg, uint16_t &out) override {
    if (!can_read_ || read_fails_) {
      return false;
    }
    out = regs_[reg];
    return true;
  }

  bool canRead() const override { return can_read_; }

  void failWritesTo(uint8_t reg) {
    fail_on_reg_ = reg;
    fail_on_reg_valid_ = true;
  }
  void setReadFails(bool fails) { read_fails_ = fails; }
  void presetRegister(uint8_t reg, uint16_t value) { regs_[reg] = value; }

  std::vector<Frame> writes;

private:
  bool can_read_;
  bool read_fails_ = false;
  uint8_t fail_on_reg_ = 0;
  bool fail_on_reg_valid_ = false;
  uint16_t regs_[16] = {0};
};

unsigned delay_calls = 0;
uint32_t delay_total_ms = 0;

void countingDelay(uint32_t ms) {
  ++delay_calls;
  delay_total_ms += ms;
}

Config testConfig(uint16_t gain = kGainRefDiv | kGainBuffGain) {
  Config config;
  config.gain_register = gain;
  config.soft_reset_first = false;
  config.delay_ms = &countingDelay;
  return config;
}

} // namespace

void setUp(void) {
  delay_calls = 0;
  delay_total_ms = 0;
}
void tearDown(void) {}

// ---------------------------------------------------------------------------
// The ordering interlock
// ---------------------------------------------------------------------------

void test_data_writes_are_refused_before_begin(void) {
  FakeTransport transport(false);
  Dac80501 dac(transport);

  TEST_ASSERT_FALSE(dac.configured());
  // This is the whole point: at this moment GAIN is at its reset 5.0 V full
  // scale, so a code here would mean roughly double the intended output.
  TEST_ASSERT_FALSE(dac.setCode(0xFFFF));
  TEST_ASSERT_FALSE(dac.setVolts(1.25f));
  TEST_ASSERT_FALSE(dac.trigger());
  // And nothing reached the bus at all -- refusing must not half-write.
  TEST_ASSERT_EQUAL_UINT32(0, transport.writes.size());
}

void test_begin_writes_gain_before_any_dac_data(void) {
  FakeTransport transport(false);
  Dac80501 dac(transport);
  TEST_ASSERT_TRUE(dac.begin(testConfig()));
  TEST_ASSERT_TRUE(dac.setCode(0x8000));

  // Find the first GAIN write and the first DAC-data write, and insist on the
  // order. Checking the order rather than exact indices leaves room for the
  // configuration sequence to grow.
  int gain_at = -1;
  int data_at = -1;
  for (size_t i = 0; i < transport.writes.size(); ++i) {
    if (transport.writes[i].reg == kRegGain && gain_at < 0) {
      gain_at = static_cast<int>(i);
    }
    if (transport.writes[i].reg == kRegDacData && data_at < 0) {
      data_at = static_cast<int>(i);
    }
  }
  TEST_ASSERT_GREATER_OR_EQUAL(0, gain_at);
  TEST_ASSERT_GREATER_OR_EQUAL(0, data_at);
  TEST_ASSERT_TRUE(gain_at < data_at);
}

void test_begin_writes_the_requested_gain(void) {
  FakeTransport transport(false);
  Dac80501 dac(transport);
  const uint16_t gain = kGainRefDiv | kGainBuffGain;
  TEST_ASSERT_TRUE(dac.begin(testConfig(gain)));

  bool seen = false;
  for (const Frame &frame : transport.writes) {
    if (frame.reg == kRegGain) {
      TEST_ASSERT_EQUAL_HEX16(gain, frame.value);
      seen = true;
    }
  }
  TEST_ASSERT_TRUE(seen);
  TEST_ASSERT_EQUAL_FLOAT(2.5f, dac.fullScale());
}

void test_a_failed_gain_write_leaves_data_writes_blocked(void) {
  FakeTransport transport(false);
  transport.failWritesTo(kRegGain);
  Dac80501 dac(transport);

  TEST_ASSERT_FALSE(dac.begin(testConfig()));
  TEST_ASSERT_FALSE(dac.configured());
  // The dangerous case: begin() failed partway, so the span is unknown. Data
  // writes must stay shut rather than run against a half-applied config.
  TEST_ASSERT_FALSE(dac.setCode(0xFFFF));
}

void test_soft_reset_reblocks_data_writes(void) {
  FakeTransport transport(false);
  Dac80501 dac(transport);
  TEST_ASSERT_TRUE(dac.begin(testConfig()));
  TEST_ASSERT_TRUE(dac.setCode(0x1234));
  TEST_ASSERT_EQUAL_UINT16(0x1234, dac.lastCode());

  TEST_ASSERT_TRUE(dac.softReset());
  // After a reset GAIN is back to 5.0 V full scale, so the interlock has to
  // re-arm. This was checked on hardware too, on board E4654C159F171E34.
  TEST_ASSERT_FALSE(dac.configured());
  TEST_ASSERT_FALSE(dac.setCode(0xFFFF));
  TEST_ASSERT_EQUAL_UINT16(0, dac.lastCode());
}

void test_soft_reset_reblocks_even_when_the_frame_fails(void) {
  FakeTransport transport(false);
  Dac80501 dac(transport);
  TEST_ASSERT_TRUE(dac.begin(testConfig()));

  transport.failWritesTo(kRegTrigger);
  TEST_ASSERT_FALSE(dac.softReset());
  // The part may or may not have reset. Assuming it did not, and leaving data
  // writes open, is the unsafe assumption of the two.
  TEST_ASSERT_FALSE(dac.configured());
}

void test_invalidate_reblocks_writes_without_touching_the_bus(void) {
  FakeTransport transport(false);
  Dac80501 dac(transport);
  TEST_ASSERT_TRUE(dac.begin(testConfig()));
  TEST_ASSERT_TRUE(dac.setCode(0x4321));
  transport.writes.clear();

  dac.invalidate();

  // Blocked, exactly as after a soft reset...
  TEST_ASSERT_FALSE(dac.configured());
  TEST_ASSERT_FALSE(dac.setCode(0xFFFF));
  TEST_ASSERT_EQUAL_UINT16(0, dac.lastCode());
  // ...but with NOTHING written. This is called when the DAC has lost VDD, and
  // driving SYNC or SCLK into an unpowered part back-powers its supply through
  // the input clamps. A softReset() here would send a frame and do exactly
  // that, which is why invalidate() exists separately.
  TEST_ASSERT_EQUAL_UINT32(0, transport.writes.size());
}

void test_soft_reset_uses_the_reserved_code(void) {
  FakeTransport transport(false);
  Dac80501 dac(transport);
  TEST_ASSERT_TRUE(dac.softReset());
  TEST_ASSERT_EQUAL_UINT32(1, transport.writes.size());
  TEST_ASSERT_EQUAL_HEX8(kRegTrigger, transport.writes[0].reg);
  // 1010b in SOFT-RESET[3:0]. Any other value is not a reset.
  TEST_ASSERT_EQUAL_HEX16(0x000A, transport.writes[0].value);
}

// ---------------------------------------------------------------------------
// begin() sequence details
// ---------------------------------------------------------------------------

void test_begin_honours_the_soft_reset_option(void) {
  FakeTransport transport(false);
  Dac80501 dac(transport);
  Config config = testConfig();
  config.soft_reset_first = true;
  config.reset_settle_ms = 5;
  TEST_ASSERT_TRUE(dac.begin(config));

  TEST_ASSERT_EQUAL_HEX8(kRegTrigger, transport.writes[0].reg);
  TEST_ASSERT_EQUAL_HEX16(0x000A, transport.writes[0].value);
  // The injected delay actually ran, and with the configured duration. Without
  // this the settle is silently skipped on hardware.
  TEST_ASSERT_EQUAL_UINT32(1, delay_calls);
  TEST_ASSERT_EQUAL_UINT32(5, delay_total_ms);
}

void test_begin_skips_the_delay_when_none_is_injected(void) {
  FakeTransport transport(false);
  Dac80501 dac(transport);
  Config config = testConfig();
  config.soft_reset_first = true;
  config.delay_ms = nullptr;
  // A null delay must be a skip, not a crash. This is the host-test default.
  TEST_ASSERT_TRUE(dac.begin(config));
  TEST_ASSERT_EQUAL_UINT32(0, delay_calls);
}

void test_begin_sets_sync_mode(void) {
  FakeTransport async_transport(false);
  Dac80501 async_dac(async_transport);
  Config async_config = testConfig();
  async_config.synchronous = false;
  TEST_ASSERT_TRUE(async_dac.begin(async_config));

  FakeTransport sync_transport(false);
  Dac80501 sync_dac(sync_transport);
  Config sync_config = testConfig();
  sync_config.synchronous = true;
  TEST_ASSERT_TRUE(sync_dac.begin(sync_config));

  for (const Frame &frame : async_transport.writes) {
    if (frame.reg == kRegSync) {
      TEST_ASSERT_EQUAL_HEX16(0x0000, frame.value);
    }
  }
  for (const Frame &frame : sync_transport.writes) {
    if (frame.reg == kRegSync) {
      TEST_ASSERT_EQUAL_HEX16(kSyncDacSyncEn, frame.value);
    }
  }
}

// ---------------------------------------------------------------------------
// Data writes
// ---------------------------------------------------------------------------

void test_set_code_writes_the_dac_register(void) {
  FakeTransport transport(false);
  Dac80501 dac(transport);
  TEST_ASSERT_TRUE(dac.begin(testConfig()));
  transport.writes.clear();

  TEST_ASSERT_TRUE(dac.setCode(0xBEEF));
  TEST_ASSERT_EQUAL_UINT32(1, transport.writes.size());
  TEST_ASSERT_EQUAL_HEX8(kRegDacData, transport.writes[0].reg);
  TEST_ASSERT_EQUAL_HEX16(0xBEEF, transport.writes[0].value);
  TEST_ASSERT_EQUAL_UINT16(0xBEEF, dac.lastCode());
}

void test_set_volts_converts_and_clamps(void) {
  FakeTransport transport(false);
  Dac80501 dac(transport);
  TEST_ASSERT_TRUE(dac.begin(testConfig())); // 2.5 V full scale
  transport.writes.clear();

  TEST_ASSERT_TRUE(dac.setVolts(1.25f));
  TEST_ASSERT_EQUAL_HEX16(0x8000, transport.writes.back().value);

  // Over range clamps to the top code rather than wrapping to near zero.
  TEST_ASSERT_TRUE(dac.setVolts(99.0f));
  TEST_ASSERT_EQUAL_HEX16(0xFFFF, transport.writes.back().value);

  TEST_ASSERT_TRUE(dac.setVolts(-1.0f));
  TEST_ASSERT_EQUAL_HEX16(0x0000, transport.writes.back().value);
}

void test_a_failed_data_write_does_not_update_last_code(void) {
  FakeTransport transport(false);
  Dac80501 dac(transport);
  TEST_ASSERT_TRUE(dac.begin(testConfig()));
  TEST_ASSERT_TRUE(dac.setCode(0x0100));

  transport.failWritesTo(kRegDacData);
  TEST_ASSERT_FALSE(dac.setCode(0x0200));
  // lastCode() is what a console prints as the current output. Advancing it on
  // a failed write would misreport the board's state.
  TEST_ASSERT_EQUAL_UINT16(0x0100, dac.lastCode());
}

// ---------------------------------------------------------------------------
// verifyConfiguration, all four outcomes
// ---------------------------------------------------------------------------

void test_verify_reports_not_supported_on_a_write_only_transport(void) {
  FakeTransport transport(false); // SPI: cannot read
  Dac80501 dac(transport);
  TEST_ASSERT_TRUE(dac.begin(testConfig()));
  transport.writes.clear();

  TEST_ASSERT_EQUAL(VerifyResult::kNotSupported, dac.verifyConfiguration());
  // And it did not pretend to try. A bus transaction here would be misleading
  // in a capture.
  TEST_ASSERT_EQUAL_UINT32(0, transport.writes.size());
}

void test_verify_reports_verified_when_the_readback_matches(void) {
  FakeTransport transport(true); // I2C: can read
  Dac80501 dac(transport);
  TEST_ASSERT_TRUE(dac.begin(testConfig()));
  TEST_ASSERT_EQUAL(VerifyResult::kVerified, dac.verifyConfiguration());
}

void test_verify_reports_mismatch(void) {
  FakeTransport transport(true);
  Dac80501 dac(transport);
  TEST_ASSERT_TRUE(dac.begin(testConfig(kGainRefDiv | kGainBuffGain)));
  // Something else moved GAIN -- a stray write, a reset, a bus fault.
  transport.presetRegister(kRegGain, kGainReset);
  TEST_ASSERT_EQUAL(VerifyResult::kMismatch, dac.verifyConfiguration());
}

void test_verify_ignores_reserved_bits(void) {
  FakeTransport transport(true);
  Dac80501 dac(transport);
  const uint16_t gain = kGainRefDiv | kGainBuffGain;
  TEST_ASSERT_TRUE(dac.begin(testConfig(gain)));
  // Reserved bits set. Those read as zero on today's silicon, but asserting on
  // them would make a future revision look like a fault.
  transport.presetRegister(kRegGain, gain | 0x7E00u | 0x00FEu);
  TEST_ASSERT_EQUAL(VerifyResult::kVerified, dac.verifyConfiguration());
}

void test_verify_reports_read_failed(void) {
  FakeTransport transport(true);
  Dac80501 dac(transport);
  TEST_ASSERT_TRUE(dac.begin(testConfig()));
  transport.setReadFails(true);
  // Distinct from kNotSupported: this transport can read, and the read broke.
  TEST_ASSERT_EQUAL(VerifyResult::kReadFailed, dac.verifyConfiguration());
}

// ---------------------------------------------------------------------------
// Readback helpers
// ---------------------------------------------------------------------------

void test_devid_and_alarm_are_unavailable_over_spi(void) {
  FakeTransport transport(false);
  Dac80501 dac(transport);
  uint16_t devid = 0xFFFF;
  bool alarm = true;
  TEST_ASSERT_FALSE(dac.readDevid(devid));
  TEST_ASSERT_FALSE(dac.readReferenceAlarm(alarm));
  // The out-parameters must be left alone on failure, so a caller that ignores
  // the return value cannot silently read a fabricated "no alarm".
  TEST_ASSERT_EQUAL_HEX16(0xFFFF, devid);
  TEST_ASSERT_TRUE(alarm);
}

void test_devid_and_alarm_over_i2c(void) {
  FakeTransport transport(true);
  transport.presetRegister(kRegDevid, kDevidReset80501Z);
  transport.presetRegister(kRegStatus, kStatusRefAlarm);
  Dac80501 dac(transport);

  uint16_t devid = 0;
  TEST_ASSERT_TRUE(dac.readDevid(devid));
  TEST_ASSERT_EQUAL_HEX16(kDevidReset80501Z, devid);
  TEST_ASSERT_TRUE(devidIsPlausible(devid));

  bool alarm = false;
  TEST_ASSERT_TRUE(dac.readReferenceAlarm(alarm));
  // The whole reason the I2C strap is worth considering: this bit is how a
  // reference alarm is told apart from a legitimate 0 V output.
  TEST_ASSERT_TRUE(alarm);

  transport.presetRegister(kRegStatus, 0x0000);
  TEST_ASSERT_TRUE(dac.readReferenceAlarm(alarm));
  TEST_ASSERT_FALSE(alarm);
}

// ---------------------------------------------------------------------------
// Power down
// ---------------------------------------------------------------------------

void test_power_down_sets_the_config_bits(void) {
  FakeTransport transport(false);
  Dac80501 dac(transport);
  TEST_ASSERT_TRUE(dac.begin(testConfig()));
  transport.writes.clear();

  TEST_ASSERT_TRUE(dac.setPowerDown(true, false));
  TEST_ASSERT_EQUAL_HEX16(kConfigDacPwdwn, transport.writes.back().value);

  TEST_ASSERT_TRUE(dac.setPowerDown(false, true));
  TEST_ASSERT_EQUAL_HEX16(kConfigRefPwdwn, transport.writes.back().value);

  TEST_ASSERT_TRUE(dac.setPowerDown(true, true));
  TEST_ASSERT_EQUAL_HEX16(kConfigDacPwdwn | kConfigRefPwdwn,
                          transport.writes.back().value);

  TEST_ASSERT_TRUE(dac.setPowerDown(false, false));
  TEST_ASSERT_EQUAL_HEX16(0x0000, transport.writes.back().value);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_data_writes_are_refused_before_begin);
  RUN_TEST(test_begin_writes_gain_before_any_dac_data);
  RUN_TEST(test_begin_writes_the_requested_gain);
  RUN_TEST(test_a_failed_gain_write_leaves_data_writes_blocked);
  RUN_TEST(test_soft_reset_reblocks_data_writes);
  RUN_TEST(test_soft_reset_reblocks_even_when_the_frame_fails);
  RUN_TEST(test_invalidate_reblocks_writes_without_touching_the_bus);
  RUN_TEST(test_soft_reset_uses_the_reserved_code);
  RUN_TEST(test_begin_honours_the_soft_reset_option);
  RUN_TEST(test_begin_skips_the_delay_when_none_is_injected);
  RUN_TEST(test_begin_sets_sync_mode);
  RUN_TEST(test_set_code_writes_the_dac_register);
  RUN_TEST(test_set_volts_converts_and_clamps);
  RUN_TEST(test_a_failed_data_write_does_not_update_last_code);
  RUN_TEST(test_verify_reports_not_supported_on_a_write_only_transport);
  RUN_TEST(test_verify_reports_verified_when_the_readback_matches);
  RUN_TEST(test_verify_reports_mismatch);
  RUN_TEST(test_verify_ignores_reserved_bits);
  RUN_TEST(test_verify_reports_read_failed);
  RUN_TEST(test_devid_and_alarm_are_unavailable_over_spi);
  RUN_TEST(test_devid_and_alarm_over_i2c);
  RUN_TEST(test_power_down_sets_the_config_bits);
  return UNITY_END();
}
