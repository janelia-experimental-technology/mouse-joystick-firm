// DAC80501 / DAC70501 / DAC60501 register map and output arithmetic.
//
// Arduino-free on purpose, so the host tests can reach all of it. Anything that
// needs SPI, Wire or a pin lives in DAC80501.hpp instead.
//
// Every number here was read out of TI SBAS794E (November 2018, revised August
// 2023), the DACx0501 family datasheet, tables 8-1 through 8-15. A local copy is
// at kicad-libs/tmp/dac60501.pdf -- the filename says 60501 but it is the family
// datasheet and covers the 80501 throughout.
//
// ---------------------------------------------------------------------------
// THE PART IS WRITE-ONLY OVER SPI. THIS IS NOT A FIRMWARE LIMITATION.
// ---------------------------------------------------------------------------
// The VSSOP-10 pinout is VDD, VOUT, NC, AGND, SPI2C, SCLK, SYNC, SDIN, NC,
// VREFIO. There is no SDO pin, `SDIN` is typed *Input* in SPI mode, and the
// string "SDO" does not appear anywhere in the datasheet. Register readback --
// DEVID, GAIN, and STATUS.REF-ALARM -- exists ONLY in I2C mode, which is
// selected by the SPI2C strap pin at power-up and is therefore a board
// decision rather than a firmware one.
//
// That matters beyond convenience. mouse-joystick-pcb's
// docs/encoder-analog-output.md requires that `STATUS` report the output as
// invalid when it reads 0 V for either of two reasons: an unpowered output
// section, or a reference alarm from a misconfigured REF-DIV. Both look exactly
// like a legitimate end of travel in a recorded dataset. Over SPI the reference
// alarm is undetectable, so that requirement cannot be met. Over I2C it can.
//
// ---------------------------------------------------------------------------
// THE SAFETY PROPERTY: GAIN MUST BE WRITTEN BEFORE ANY DAC DATA.
// ---------------------------------------------------------------------------
// GAIN resets to 0x0001 -- REF-DIV 0, BUFF-GAIN 1 -- which with the internal
// 2.5 V reference is a FULL SCALE OF 5.0 V. A board that wants a 2.5 V full
// scale and writes data before writing GAIN presents double the intended
// voltage at its output. On mouse-joystick-pcb v0.3 the DAC runs from +5V_SW
// and the output stage has a gain of 1.32, so a full-scale code at the reset
// GAIN asks for 6.6 V and delivers whatever the 5 V rail allows.
//
// The `Z` orderable saves this from being a live hazard at power-up, because it
// resets the DAC data register to zero scale (the `M` variant resets to
// midscale, which would be 2.5 V at the reset gain). Do not rely on that:
// Dac80501::begin() writes GAIN first and refuses data writes until it has.

#pragma once

#include <stdint.h>

namespace dac80501 {

// ---------------------------------------------------------------------------
// Command byte (datasheet table 8-4)
// ---------------------------------------------------------------------------
// The command byte is DB[23:16] and carries the register address in DB[19:16]
// with DB[23:20] zero, so the byte value is simply the register offset. It is
// the same byte in both SPI and I2C mode -- in SPI it is the first of three
// bytes shifted in, in I2C it follows the address byte.
//
// Offset 6h is absent from the map. It is not a gap this code should paper
// over, so there is no constant for it.
constexpr uint8_t kRegNoop = 0x00;
constexpr uint8_t kRegDevid = 0x01;
constexpr uint8_t kRegSync = 0x02;
constexpr uint8_t kRegConfig = 0x03;
constexpr uint8_t kRegGain = 0x04;
constexpr uint8_t kRegTrigger = 0x05;
constexpr uint8_t kRegStatus = 0x07;
constexpr uint8_t kRegDacData = 0x08;

// ---------------------------------------------------------------------------
// Reset values (datasheet section 8.6)
// ---------------------------------------------------------------------------
// DEVID is read-only and its reset value identifies both the resolution and the
// power-on-reset scale, so it is the whole presence-and-part check in one word
// -- when a transport can read it at all.
constexpr uint16_t kDevidReset80501Z = 0x0115; // 16-bit, zero-scale reset
constexpr uint16_t kDevidReset80501M = 0x0195; // 16-bit, midscale reset
constexpr uint16_t kDevidReset70501Z = 0x1115; // 14-bit
constexpr uint16_t kDevidReset60501Z = 0x2115; // 12-bit

constexpr uint16_t kSyncReset = 0x0000;
constexpr uint16_t kConfigReset = 0x0000;
constexpr uint16_t kGainReset = 0x0001; // REF-DIV 0, BUFF-GAIN 1 -> 5.0 V FS
constexpr uint16_t kTriggerReset = 0x0000;
constexpr uint16_t kStatusReset = 0x0000;
constexpr uint16_t kDacDataResetZ = 0x0000;
constexpr uint16_t kDacDataResetM = 0x8000;

// ---------------------------------------------------------------------------
// Fields
// ---------------------------------------------------------------------------
// SYNC (2h)
constexpr uint16_t kSyncDacSyncEn = 1u << 0; // 1 = update on LDAC, 0 = immediate

// CONFIG (3h)
constexpr uint16_t kConfigRefPwdwn = 1u << 8; // 1 = internal reference disabled
constexpr uint16_t kConfigDacPwdwn = 1u << 0; // 1 = output tied to AGND via 1k

// GAIN (4h)
constexpr uint16_t kGainRefDiv = 1u << 8;    // 1 = reference divided by 2
constexpr uint16_t kGainBuffGain = 1u << 0;  // 1 = output buffer gain of 2

// TRIGGER (5h). Both fields are write-only and self-clearing.
constexpr uint16_t kTriggerLdac = 1u << 4;
constexpr uint16_t kTriggerSoftResetCode = 0x000A; // the reserved code 1010b

// STATUS (7h)
constexpr uint16_t kStatusRefAlarm = 1u << 0;

// DEVID (1h) decode
constexpr uint16_t kDevidResolutionMask = 0x7000u; // DB[14:12]
constexpr uint16_t kDevidRstselMidscale = 1u << 7;

// ---------------------------------------------------------------------------
// I2C addressing (datasheet tables 8-2 and 8-3)
// ---------------------------------------------------------------------------
// The upper four bits are factory-preset to 1001; the low three come from where
// the A0 pin is tied. These are 7-bit addresses, so pass them to Wire as-is --
// the Arduino API adds the R/W bit.
constexpr uint8_t kI2cAddrA0Agnd = 0x48; // 1001000
constexpr uint8_t kI2cAddrA0Vdd = 0x49;  // 1001001
constexpr uint8_t kI2cAddrA0Sda = 0x4A;  // 1001010
constexpr uint8_t kI2cAddrA0Scl = 0x4B;  // 1001011

// ---------------------------------------------------------------------------
// Output arithmetic
// ---------------------------------------------------------------------------
// Full scale is the reference, halved if REF-DIV is set, then doubled if
// BUFF-GAIN is set. With the internal 2.5 V reference that gives 5.0 V, 2.5 V
// or 1.25 V -- the three ranges the datasheet advertises.
//
// Note that 0x0000 (REF-DIV 0, BUFF-GAIN 0) and 0x0101 (REF-DIV 1, BUFF-GAIN 1)
// both produce a 2.5 V full scale by different routes. They are NOT equivalent:
// the second runs the output buffer at a gain of two, which doubles that stage's
// offset and noise contribution, while the first keeps the DAC core at the full
// 2.5 V reference. Prefer 0x0000 unless a supply-headroom argument says
// otherwise -- and there is one at 3.3 V VDD, where a 2.5 V reference leaves
// little room and REF-DIV pulls the core reference down to 1.25 V.
constexpr float kInternalReferenceVolts = 2.5f;

constexpr float fullScaleVolts(uint16_t gain_register,
                               float reference_volts = kInternalReferenceVolts) {
  const float divided = (gain_register & kGainRefDiv) ? reference_volts * 0.5f
                                                      : reference_volts;
  return (gain_register & kGainBuffGain) ? divided * 2.0f : divided;
}

// Codes are MSB-aligned straight binary. The 16-bit part uses all 16 bits; the
// 14- and 12-bit parts left-align into the same field, which is why this scales
// by 65536 regardless of resolution rather than by (1 << resolution).
constexpr uint32_t kCodeCount = 65536u;
constexpr uint16_t kCodeMax = 0xFFFFu;

// The output the datasheet's transfer function predicts for a code. Full scale
// is approached but never reached: code 0xFFFF gives 65535/65536 of it.
constexpr float codeToVolts(uint16_t code, uint16_t gain_register,
                            float reference_volts = kInternalReferenceVolts) {
  return fullScaleVolts(gain_register, reference_volts) *
         static_cast<float>(code) / static_cast<float>(kCodeCount);
}

// Nearest code for a requested voltage, clamped to the representable range.
// Clamps rather than wrapping or erroring: this sits under an angle output
// where a silent wrap would be a catastrophic misreport, and where asking for
// slightly more than full scale is a rounding artefact rather than a bug.
inline uint16_t voltsToCode(float volts, uint16_t gain_register,
                            float reference_volts = kInternalReferenceVolts) {
  const float full_scale = fullScaleVolts(gain_register, reference_volts);
  if (!(full_scale > 0.0f) || !(volts > 0.0f)) {
    return 0;
  }
  const float exact = volts * static_cast<float>(kCodeCount) / full_scale;
  if (exact >= static_cast<float>(kCodeMax)) {
    return kCodeMax;
  }
  return static_cast<uint16_t>(exact + 0.5f);
}

// One LSB at the given gain setting, which is what a caller needs to decide
// whether a span is worth preserving.
constexpr float voltsPerCode(uint16_t gain_register,
                             float reference_volts = kInternalReferenceVolts) {
  return fullScaleVolts(gain_register, reference_volts) /
         static_cast<float>(kCodeCount);
}

// ---------------------------------------------------------------------------
// DEVID decode, for transports that can read
// ---------------------------------------------------------------------------
enum class Resolution : uint8_t {
  kBits16 = 0, // DAC80501
  kBits14 = 1, // DAC70501
  kBits12 = 2, // DAC60501
  kUnknown = 0xFF,
};

constexpr Resolution resolutionFromDevid(uint16_t devid) {
  const uint16_t field = (devid & kDevidResolutionMask) >> 12;
  return field == 0 ? Resolution::kBits16
                    : (field == 1 ? Resolution::kBits14
                                  : (field == 2 ? Resolution::kBits12
                                                : Resolution::kUnknown));
}

// True when the part resets its data register to midscale (the `M` orderable)
// rather than zero scale (`Z`). Worth knowing at bring-up: an `M` part reaches
// half of a possibly-wrong full scale before firmware writes anything.
constexpr bool devidResetsToMidscale(uint16_t devid) {
  return (devid & kDevidRstselMidscale) != 0;
}

// A DEVID that could plausibly have come from this family. The reserved fields
// are fixed -- DB15 = 0, DB[11:8] = 1h, DB[6:0] = 15h -- so a bus that is
// floating, shorted or reading the wrong device fails this.
constexpr bool devidIsPlausible(uint16_t devid) {
  return (devid & 0x8000u) == 0 &&        // DB15 reserved, reads 0
         (devid & 0x0F00u) == 0x0100u &&  // DB[11:8] reserved, reads 1h
         (devid & 0x007Fu) == 0x0015u &&  // DB[6:0] reserved, reads 15h
         resolutionFromDevid(devid) != Resolution::kUnknown;
}

} // namespace dac80501
