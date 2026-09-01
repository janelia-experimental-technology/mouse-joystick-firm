// Scaling between raw ADC counts and board-level quantities.
//
// Header-only and free of Arduino dependencies on purpose, so the arithmetic
// can be tested on the host under `pixi run test` without hardware.

#pragma once

#include <stdint.h>

namespace mouse_joystick {
namespace conversions {

// The RP2040's ADC reference is ADC_AVDD, which on this board is tied straight
// to +3.3V_MCU (U501 pad 43). There is no separate reference and no filter, so
// absolute accuracy is only as good as that rail.
constexpr float kAdcReferenceVolts = 3.3f;

// The self test runs the ADC at 12 bits, the RP2040's native width.
constexpr uint16_t kAdcFullScale = 4095;

// R401 1k / R402 1k inside the mcu sheet.
constexpr float kSwRailDividerRatio = 2.0f;

// R1907 on the TPS1HTC100-Q1's SNS output.
constexpr float kHvSenseResistorOhms = 249.0f;

// Nominal switched 3.3 V rail, for judging a rail-sense reading.
constexpr float kSwRailNominalVolts = 3.3f;

constexpr float countsToVolts(uint16_t counts,
                              uint16_t full_scale = kAdcFullScale,
                              float reference_volts = kAdcReferenceVolts) {
  return (full_scale == 0) ? 0.0f
                           : (static_cast<float>(counts) * reference_volts /
                              static_cast<float>(full_scale));
}

// Raw +3.3V_SW, undoing the divider.
constexpr float swRailVolts(uint16_t counts,
                            uint16_t full_scale = kAdcFullScale,
                            float reference_volts = kAdcReferenceVolts) {
  return countsToVolts(counts, full_scale, reference_volts) *
         kSwRailDividerRatio;
}

// Voltage across R1907. Converting this to load current needs the
// TPS1HTC100-Q1's current-sense ratio, which this firmware does not use yet:
// stage 1 runs with the 36 V input absent, so the only expected reading is
// zero. Add the ratio from `tps1htc100-q1.pdf` when a stage that actually
// energises the rail needs amps rather than volts.
constexpr float hvSenseVolts(uint16_t counts,
                             uint16_t full_scale = kAdcFullScale,
                             float reference_volts = kAdcReferenceVolts) {
  return countsToVolts(counts, full_scale, reference_volts);
}

} // namespace conversions
} // namespace mouse_joystick
