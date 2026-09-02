// TMC5130A register addresses and field decoding.
//
// Arduino-free on purpose, so the decoding can be tested on the host under
// `pixi run test`. Addresses and the version constants match TMC51X0's
// `Registers::RegisterAddress` exactly, so the two cannot disagree.

#pragma once

#include <stdint.h>

namespace mouse_joystick {
namespace tmc {

constexpr uint8_t kRwRead = 0;
constexpr uint8_t kRwWrite = 1;

constexpr uint8_t kGconf = 0x00;
constexpr uint8_t kGstat = 0x01;
constexpr uint8_t kIoin = 0x04;
constexpr uint8_t kIholdIrun = 0x10;
constexpr uint8_t kTstep = 0x12;
constexpr uint8_t kXactual = 0x21;
constexpr uint8_t kSwMode = 0x34;
constexpr uint8_t kRampStat = 0x35;
constexpr uint8_t kEncmode = 0x38;
constexpr uint8_t kXenc = 0x39;
constexpr uint8_t kChopconf = 0x6C;
constexpr uint8_t kDrvStatus = 0x6F;
constexpr uint8_t kLostSteps = 0x73;

// IOIN version field, bits 31:24.
constexpr uint8_t kVersionTmc5130 = 0x11;
constexpr uint8_t kVersionTmc5160 = 0x30;

// CHOPCONF bit 17. Resets to 0, and with the 0.15 ohm shunts this board fits
// that means roughly 1.8x the motor's rated current. Stage 2 only reports it;
// stage 3 must set and read it back before it lowers ~{TMC_DRV_EN}.
constexpr uint8_t kChopconfVsenseBit = 17;

struct IoinDecode {
  bool refl_step;
  bool refr_dir;
  bool encb_dcen_cfg4;
  bool enca_dcin_cfg5;
  bool drv_enn;
  bool enc_n_dco_cfg6;
  bool sd_mode;
  bool swcomp_in;
  uint8_t version;
};

// Named bitIsSet, not bitSet: Arduino.h defines bitSet() as a macro, and a
// function with that name is unusable once this header is included after it.
constexpr bool bitIsSet(uint32_t raw, uint8_t bit) {
  return ((raw >> bit) & 0x1u) != 0;
}

constexpr IoinDecode decodeIoin(uint32_t raw) {
  return IoinDecode{bitIsSet(raw, 0),
                    bitIsSet(raw, 1),
                    bitIsSet(raw, 2),
                    bitIsSet(raw, 3),
                    bitIsSet(raw, 4),
                    bitIsSet(raw, 5),
                    bitIsSet(raw, 6),
                    bitIsSet(raw, 7),
                    static_cast<uint8_t>((raw >> 24) & 0xFFu)};
}

constexpr bool vsenseSet(uint32_t chopconf_raw) {
  return bitIsSet(chopconf_raw, kChopconfVsenseBit);
}

// What IOIN should read on this board with the 36 V rail up, nothing plugged
// into the motor, encoder or limit switch connectors, and ~{TMC_DRV_EN} held
// high by firmware:
//
//   bit 0 REFL_STEP      0   LIMIT_SW held low by R1405 10k to GND
//   bit 1 REFR_DIR       0   REFR tied to GND on this board
//   bit 2 ENCB_DCEN_CFG4 0   ENC_B held low by R806 100k to GND
//   bit 3 ENCA_DCIN_CFG5 0   ENC_A held low by R805 100k to GND
//   bit 4 DRV_ENN        1   firmware holds ~{TMC_DRV_EN} high, driver disabled
//   bit 5 ENC_N_DCO      0   ENCN terminated by R1201 10k to GND
//   bit 6 SD_MODE        0   tied to GND, motion-controller mode
//   bit 7 SWCOMP_IN      ?   comparator output, not asserted here
//   31:24 VERSION     0x11   TMC5130A
//
// So the expected value is 0x11000010, with bit 7 free. Bit 4 is the valuable
// one: it is the only read-back that proves firmware's GP3 actually reaches the
// driver through the bus switch.
constexpr uint32_t kIoinExpectedMask = 0xFF0000FFu & ~(1u << 7);
constexpr uint32_t kIoinExpectedValue = 0x11000010u;

constexpr bool ioinMatchesExpected(uint32_t raw) {
  return (raw & kIoinExpectedMask) == (kIoinExpectedValue & kIoinExpectedMask);
}

} // namespace tmc
} // namespace mouse_joystick
