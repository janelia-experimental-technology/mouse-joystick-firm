// ADS8689 device logic: transport-agnostic, Arduino-free, header-only.
//
// Split from the Arduino transport for the same reason DAC80501Device.hpp is:
// the property worth pinning by a host test is the ORDERING -- RANGE_SEL
// written and read back before any conversion is believed, a register read
// taking exactly two frames -- and the fake transport is the only coverage the
// readback logic has until a board is on the bench.
//
// Nothing here touches a pin, a bus or a clock. `Transport` is the whole
// hardware surface; the Arduino implementation lives in ADS8689.hpp.
//
// Read ADS8689Registers.hpp first for the frame format, the two-frame read and
// the conversion that every frame starts.

#pragma once

#include <stdint.h>

#include "ADS8689Registers.hpp"

namespace ads8689 {

// ---------------------------------------------------------------------------
// Transport
// ---------------------------------------------------------------------------
class Transport {
public:
  virtual ~Transport() = default;

  // One data transfer frame: CONVST/CS low, 32 SCLKs shifting `out` MSB first
  // while capturing the device's 32-bit output word, CONVST/CS high. The
  // rising edge starts a conversion, so the transport ALSO waits t_conv_max
  // before returning; a caller may issue the next frame immediately.
  virtual uint32_t transferFrame(uint32_t out) = 0;

  // A CONVST/CS pulse with no clocks: starts a conversion with no transfer
  // (a frame shorter than 32 clocks is a NOP). Also waits t_conv_max.
  virtual void pulseConvst() = 0;
};

// What begin() found. Not a bool, because "nothing answered", "SDO is stuck"
// and "the range did not take" call for different hands on the board, and the
// raw read-backs are what a bring-up log wants.
enum class BeginResult : uint8_t {
  kOk,
  kRangeReadbackMismatch,   // RANGE_SEL did not read back as written: no part,
                            // no bus, SDO stuck, or a wrong SPI mode
  kLoopbackOnesFailed,      // DATA_VAL all-ones did not arrive as 0xFFFF
  kLoopbackZerosFailed,     // DATA_VAL all-zeros did not arrive as 0x0000
  kDataoutReadbackMismatch, // DATAOUT_CTL did not return to conversion data
};

struct BeginReport {
  BeginResult result = BeginResult::kOk;
  uint16_t range_written = 0;
  uint16_t range_readback = 0;
  uint16_t loopback_ones = 0;
  uint16_t loopback_zeros = 0;
  uint16_t dataout_readback = 0;
  uint16_t device_id_readback = 0;  // DEVICE_ID[15:0], reads 0 on this part
  uint16_t rst_pwrctl_readback = 0; // RST_PWRCTL[15:0], 0 after power-up
};

struct Config {
  // RANGE_SEL. The board's signal is 0-5 V, so 0 to 1.25 x VREF.
  uint16_t range = kRangeUnipolar1p25;
  float reference_volts = kInternalReferenceVolts;
  // Prove SDO with the DATA_VAL fixed patterns before trusting a number. Costs
  // five frames at start-up and nothing after.
  bool loopback_check = true;
};

class Ads8689 {
public:
  explicit Ads8689(Transport &transport) : transport_(transport) {}

  // Put the protocol registers at their reset values, set the range, read it
  // back, and -- if asked -- run the SDO loopback. Only then are conversions
  // reported. Returns false and leaves `configured()` false on any mismatch;
  // `report` says which.
  //
  // There is no software reset on this part: RST is a pin, held high on
  // mouse-joystick-pcb by R2011 to +3.3V_SW, so the device resets whenever the
  // switched rail does. begin() therefore re-asserts what it needs rather
  // than assuming a fresh part.
  bool begin(const Config &config, BeginReport *report = nullptr) {
    configured_ = false;
    config_ = config;
    BeginReport local;
    BeginReport &r = (report != nullptr) ? *report : local;
    r = BeginReport{};

    // Protocol registers at reset values, written in the protocol the device
    // speaks after reset. A part left in another SDI mode by earlier firmware
    // ignores these, and then fails the read-back below -- which is the right
    // outcome, since the rail cycle that follows is what resets it.
    writeRegister(kRegSdiCtl, kSdiModeSpi00);
    writeRegister(kRegSdoCtl, kSdoCtlDefault);
    writeRegister(kRegDataoutCtl, kDataValConversion);

    // The range, before anything is believed.
    r.range_written =
        static_cast<uint16_t>(config_.range & (kRangeSelMask | kRangeIntrefDis));
    writeRegister(kRegRangeSel, r.range_written);
    r.range_readback = readRegister(kRegRangeSel);
    if ((r.range_readback & (kRangeSelMask | kRangeIntrefDis)) !=
        r.range_written) {
      r.result = BeginResult::kRangeReadbackMismatch;
      return false;
    }

    r.device_id_readback = readRegister(kRegDeviceId);
    r.rst_pwrctl_readback = readRegister(kRegRstPwrctl);

    if (config_.loopback_check) {
      // The pattern selected in frame F is what frame F+1 carries, so each
      // write is followed by one read of the output word.
      writeRegister(kRegDataoutCtl, kDataValAllOnes);
      r.loopback_ones = readLast();
      writeRegister(kRegDataoutCtl, kDataValAllZeros);
      r.loopback_zeros = readLast();
      writeRegister(kRegDataoutCtl, kDataValConversion);
      if (r.loopback_ones != 0xFFFFu) {
        r.result = BeginResult::kLoopbackOnesFailed;
        return false;
      }
      if (r.loopback_zeros != 0x0000u) {
        r.result = BeginResult::kLoopbackZerosFailed;
        return false;
      }
    }

    r.dataout_readback = readRegister(kRegDataoutCtl);
    if ((r.dataout_readback & kDataValMask) != kDataValConversion) {
      r.result = BeginResult::kDataoutReadbackMismatch;
      return false;
    }

    configured_ = true;
    return true;
  }

  // Forget the configuration WITHOUT touching the bus. Call this when the
  // switched rail goes down: the part resets with it (RST follows +3.3V_SW),
  // so RANGE_SEL is back to +/-12.288 V and nothing it says can be scaled
  // correctly until begin() runs again. Must not write anything -- clocking a
  // dead part drives its input clamps and back-powers the rail.
  void invalidate() { configured_ = false; }

  bool configured() const { return configured_; }

  // Read a 16-bit register half-word. Two frames, by construction of the part.
  uint16_t readRegister(uint8_t reg) {
    transport_.transferFrame(readFrame(reg));
    return upperHalf(transport_.transferFrame(kFrameNop));
  }

  // Write a 16-bit register half-word. One frame.
  void writeRegister(uint8_t reg, uint16_t value) {
    transport_.transferFrame(writeFrame(reg, value));
  }

  // Write RST_PWRCTL with the key the part demands.
  void writePowerControl(uint16_t bits) {
    writeRegister(kRegRstPwrctl,
                  static_cast<uint16_t>(kRstPwrctlKey | (bits & 0x00FFu)));
  }

  // The output word of one NOP frame: the latest conversion the device holds,
  // which is the one started by the PREVIOUS frame or CONVST pulse. One frame;
  // in a periodic loop this returns the sample taken at the previous period.
  uint16_t readLast() { return upperHalf(transport_.transferFrame(kFrameNop)); }

  // A conversion started now and returned now: a CONVST pulse, the wait, then
  // one NOP frame. The frame's own rising edge starts a further conversion
  // that nobody reads.
  uint16_t sample() {
    transport_.pulseConvst();
    return readLast();
  }

  float codeToVolts(uint16_t code) const {
    return ads8689::codeToVolts(code, config_.range, config_.reference_volts);
  }
  float voltsPerCode() const {
    return ads8689::voltsPerCode(config_.range, config_.reference_volts);
  }
  float fullScaleVolts() const {
    return ads8689::fullScaleVolts(config_.range, config_.reference_volts);
  }
  uint16_t range() const { return config_.range; }

private:
  Transport &transport_;
  Config config_{};
  bool configured_ = false;
};

} // namespace ads8689
