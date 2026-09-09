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

// What IOIN can be checked against, and what it cannot.
//
// REWRITTEN 2026-09-09. This used to compare the whole low byte against a
// single expected value, 0x11000010, and that check is wrong by construction:
// five of those eight bits are driven by whatever is plugged into the motor,
// encoder and limit-switch connectors, so no fixed value can be right. It
// produced a failure on every board that was actually wired up correctly, and
// two v0.2 units "disagreeing" was chased as a possible pair of assembly
// defects for a week before v0.3 settled it against the copper.
//
//   bit 0 REFL_STEP      CONNECTOR-DEPENDENT. Empty it reads 1: U1301's IN pin
//                        has no external pull, so the MAX6816 supplies its own
//                        pull-up, drives OUT high, and nothing can override a
//                        driven output. There is no LIMIT_SW pull-down on v0.3.
//                        Fitted and at rest it reads 0, because the switch is
//                        normally closed; actuating it gives 1.
//   bit 1 REFR_DIR       0   REFR is tied to GND. A hard tie, not a connector,
//                            so this one IS assertable.
//   bit 2 ENCB_DCEN_CFG4 CONNECTOR-DEPENDENT. Floating CMOS input when empty;
//                        a real quadrature state when the encoder is fitted.
//   bit 3 ENCA_DCIN_CFG5 CONNECTOR-DEPENDENT, same reasoning.
//   bit 4 DRV_ENN        1   firmware holds ~{TMC_DRV_EN} high, driver disabled
//   bit 5 ENC_N_DCO      CONNECTOR-DEPENDENT, same reasoning.
//   bit 6 SD_MODE        0   tied to GND, motion-controller mode
//   bit 7 SWCOMP_IN      comparator output, never assertable
//   31:24 VERSION     0x11   TMC5130A
//
// So assert only the four board-invariant fields. Bit 4 is the valuable one: it
// is the only read-back that proves firmware's GP3 actually reaches the driver
// through the bus switch. The connector-dependent bits are still worth
// PRINTING -- they are how you see a switch actuate or an encoder turn -- they
// are just not a pass/fail.
constexpr uint32_t kIoinInvariantMask =
    0xFF000000u | (1u << 1) | (1u << 4) | (1u << 6);
constexpr uint32_t kIoinInvariantValue = 0x11000010u;

// Bits whose value is a property of what is plugged in, not of the board.
constexpr uint32_t kIoinConnectorMask =
    (1u << 0) | (1u << 2) | (1u << 3) | (1u << 5) | (1u << 7);

constexpr bool ioinInvariantsMatch(uint32_t raw) {
  return (raw & kIoinInvariantMask) ==
         (kIoinInvariantValue & kIoinInvariantMask);
}

} // namespace tmc
} // namespace mouse_joystick
