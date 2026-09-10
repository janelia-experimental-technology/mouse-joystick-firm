// DACx0501 device logic: transport-agnostic, Arduino-free, header-only.
//
// Split out from DAC80501.hpp so the host tests can reach it. The class holds
// the ordering interlock that stops a board presenting double its intended
// output voltage, and that is exactly the kind of property that should be
// pinned by a test rather than by one manual poke at a bench console.
//
// Nothing here touches a pin, a bus or a clock. `Transport` is the whole
// hardware surface, and the Arduino implementations of it live in DAC80501.hpp.
//
// Read DAC80501Registers.hpp first: it carries the register map, the output
// arithmetic, and the reasons this part cannot be read back over SPI.

#pragma once

#include <stdint.h>

#include "DAC80501Registers.hpp"

namespace dac80501 {

// ---------------------------------------------------------------------------
// Transport
// ---------------------------------------------------------------------------
// Two methods, because that is the whole hardware interface: a 24-bit write of
// a command byte plus a 16-bit payload, and -- on I2C only -- a read of the
// same shape.
//
// `canRead()` is separate from `readRegister()` returning false on purpose.
// "This transport physically cannot read" and "the read failed" are different
// facts, and a caller explaining itself to a human needs to tell them apart.
class Transport {
public:
  virtual ~Transport() = default;

  virtual bool writeRegister(uint8_t reg, uint16_t value) = 0;
  virtual bool readRegister(uint8_t reg, uint16_t &out) = 0;
  virtual bool canRead() const = 0;
};

// What `verifyConfiguration()` found. Four outcomes and not a bool, because
// "could not check" is a different thing from "checked and it was right", and
// collapsing them is how a board ends up trusted on no evidence.
enum class VerifyResult : uint8_t {
  kVerified,     // read back and matched
  kMismatch,     // read back and did NOT match
  kNotSupported, // this transport cannot read; nothing was checked
  kReadFailed,   // the transport can read but the read did not complete
};

struct Config {
  // The GAIN register value, which sets full scale. See the discussion of
  // 0x0000 against 0x0101 in DAC80501Registers.hpp -- both give a 2.5 V full
  // scale from a 2.5 V reference and they are not equivalent.
  uint16_t gain_register = kGainReset;

  // The reference the part actually has. Leave at the internal 2.5 V unless the
  // board drives VREFIO externally.
  float reference_volts = kInternalReferenceVolts;

  // Update the output on write (asynchronous, the reset default) or wait for an
  // LDAC trigger. Synchronous mode matters when several DACs must step together.
  bool synchronous = false;

  // Issue a software reset before configuring. Costs one extra frame and makes
  // start-up independent of whatever the previous firmware left behind, which
  // is worth it on a bring-up board that gets reflashed constantly.
  bool soft_reset_first = true;

  // Injected so this class stays Arduino-free. The Arduino side passes `delay`.
  // Left null, begin() skips the settle entirely -- which is correct for a host
  // test and wrong on hardware, so DAC80501.hpp's helper fills it in rather
  // than leaving it to each caller to remember.
  void (*delay_ms)(uint32_t) = nullptr;

  // How long to settle after a soft reset. The datasheet gives no explicit
  // reset recovery time and the reset bits are self-clearing, so this is
  // simply far longer than anything in the timing tables and costs nothing at
  // start-up.
  uint32_t reset_settle_ms = 1;
};

class Dac80501 {
public:
  explicit Dac80501(Transport &transport) : transport_(transport) {}

  // Writes CONFIG, GAIN and SYNC, in that order, and only then permits data
  // writes. Returns false if any frame failed.
  //
  // ORDERING IS THE POINT. GAIN resets to a 5.0 V full scale, so a board that
  // wants 2.5 V and writes data first asks for double. setCode() and setVolts()
  // refuse until this has succeeded, which makes that mistake unrepresentable
  // rather than merely documented.
  bool begin(const Config &config) {
    // Clear this first. If any frame below fails, data writes must stay blocked
    // rather than run against a half-applied configuration.
    configured_ = false;
    config_ = config;

    if (config_.soft_reset_first) {
      if (!softReset()) {
        return false;
      }
      if (config_.delay_ms != nullptr) {
        config_.delay_ms(config_.reset_settle_ms);
      }
    }

    // CONFIG before GAIN: bring the reference and output stage up in a known
    // state, then set the span.
    if (!transport_.writeRegister(kRegConfig, kConfigReset)) {
      return false;
    }
    if (!transport_.writeRegister(kRegGain, config_.gain_register)) {
      return false;
    }
    const uint16_t sync_value =
        config_.synchronous ? kSyncDacSyncEn : kSyncReset;
    if (!transport_.writeRegister(kRegSync, sync_value)) {
      return false;
    }

    configured_ = true;
    return true;
  }

  // Read back what begin() wrote, where the transport allows it. On SPI this
  // returns kNotSupported without touching the bus -- the honest answer, and
  // the reason the caller should be measuring the output instead.
  VerifyResult verifyConfiguration() {
    if (!transport_.canRead()) {
      return VerifyResult::kNotSupported;
    }
    uint16_t gain = 0;
    if (!transport_.readRegister(kRegGain, gain)) {
      return VerifyResult::kReadFailed;
    }
    // Compare only the two fields that exist. The reserved bits read as zero,
    // but asserting on them would make a future silicon revision look like a
    // fault.
    constexpr uint16_t kGainFields = kGainRefDiv | kGainBuffGain;
    if ((gain & kGainFields) != (config_.gain_register & kGainFields)) {
      return VerifyResult::kMismatch;
    }
    return VerifyResult::kVerified;
  }

  // DEVID, for transports that can read it. Confirms the part is present, its
  // resolution, and whether it resets to zero scale or midscale.
  bool readDevid(uint16_t &devid) {
    if (!transport_.canRead()) {
      return false;
    }
    return transport_.readRegister(kRegDevid, devid);
  }

  // STATUS.REF-ALARM. When set, the reference buffer is shut down and the
  // output is 0 V while the DAC codes are unaffected -- so the output reads as
  // a legitimate zero and only this bit distinguishes the two. Unreachable over
  // SPI, which is the strongest argument for the I2C strap.
  bool readReferenceAlarm(bool &alarm_active) {
    if (!transport_.canRead()) {
      return false;
    }
    uint16_t status = 0;
    if (!transport_.readRegister(kRegStatus, status)) {
      return false;
    }
    alarm_active = (status & kStatusRefAlarm) != 0;
    return true;
  }

  // Straight binary, MSB aligned. Refuses before begin() has succeeded.
  bool setCode(uint16_t code) {
    // The ordering guarantee. Before begin() succeeds, GAIN is at its reset
    // 5.0 V full scale and any code here means roughly double the intended
    // output.
    if (!configured_) {
      return false;
    }
    if (!transport_.writeRegister(kRegDacData, code)) {
      return false;
    }
    last_code_ = code;
    return true;
  }

  // Rounds to the nearest code and clamps to the representable range. Refuses
  // before begin() has succeeded.
  bool setVolts(float volts) {
    if (!configured_) {
      return false;
    }
    return setCode(
        voltsToCode(volts, config_.gain_register, config_.reference_volts));
  }

  // Pulse LDAC, for synchronous mode.
  bool trigger() {
    if (!configured_) {
      return false;
    }
    return transport_.writeRegister(kRegTrigger, kTriggerLdac);
  }

  // Software reset. Clears `configured()`, because after a reset GAIN is back
  // to its 5.0 V full scale and data writes must be blocked again.
  bool softReset() {
    // Deliberately not gated on configured_: a reset is how you recover from an
    // unknown state, so requiring a known state first would be backwards.
    const bool ok = transport_.writeRegister(kRegTrigger, kTriggerSoftResetCode);
    // GAIN is back to its 5.0 V full scale whether or not the frame was
    // acknowledged, so block data writes either way.
    configured_ = false;
    last_code_ = 0;
    return ok;
  }

  // Power the output stage down (output tied to AGND through an internal 1 k)
  // or the internal reference, or both.
  bool setPowerDown(bool dac_powered_down, bool reference_powered_down) {
    uint16_t value = kConfigReset;
    if (dac_powered_down) {
      value |= kConfigDacPwdwn;
    }
    if (reference_powered_down) {
      value |= kConfigRefPwdwn;
    }
    return transport_.writeRegister(kRegConfig, value);
  }

  bool configured() const { return configured_; }
  uint16_t gainRegister() const { return config_.gain_register; }
  float fullScale() const {
    return fullScaleVolts(config_.gain_register, config_.reference_volts);
  }
  float voltsPerLsb() const {
    return voltsPerCode(config_.gain_register, config_.reference_volts);
  }
  uint16_t lastCode() const { return last_code_; }

private:
  Transport &transport_;
  Config config_{};
  bool configured_ = false;
  uint16_t last_code_ = 0;
};

} // namespace dac80501
