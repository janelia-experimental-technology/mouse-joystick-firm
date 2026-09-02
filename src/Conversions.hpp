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

// The RP2040's native ADC width. The Arduino core defaults to 10 bits, so
// every read path must set this explicitly -- a reader that forgets divides by
// the wrong full scale and reports a quarter of the real voltage. These two
// constants must agree; a host test asserts it.
constexpr uint8_t kAdcResolutionBits = 12;
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

// TPS1HTC100-Q1 current-sense ratio, `KSNS` = 800 A/A, constant across
// temperature and supply voltage (datasheet Section 5.5). With R207 = 249 ohm
// that makes the sense chain 0.311 V/A, so the ADC's 0.806 mV LSB is 2.6 mA of
// load current.
constexpr float kHvSenseRatio = 800.0f;

// The switch reports faults on the same pin by forcing `ISNSFLT`, 5.2-6.4 mA,
// instead of KSNS-divided load current. Into 249 ohm that is 1.29-1.59 V, and
// measured parts sit a little above the datasheet maximum (6.3-6.5 mA has been
// seen on this board). Anything above this threshold is the fault current
// source, NOT a load reading -- converting it to amps yields a fictitious 4 A.
constexpr float kHvSenseFaultVolts = 1.20f;

// Measured quiet-state floor on this board: 18-22 counts with HV_EN low and no
// load at all, i.e. ADC offset plus leakage. One count is 2.6 mA of load
// current, so at the ~10-15 mA idle draw the sense chain resolves nothing.
// Printing a reading this small as amps is exactly how HV_SNS has been
// misread before -- twice.
constexpr uint16_t kHvSenseFloorCounts = 40;

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

// Load current through the high-side switch, from the SNS voltage. Only
// meaningful below kHvSenseFaultVolts -- check hvSenseIsFault() first.
constexpr float hvLoadAmps(uint16_t counts, uint16_t full_scale = kAdcFullScale,
                           float reference_volts = kAdcReferenceVolts) {
  return countsToVolts(counts, full_scale, reference_volts) * kHvSenseRatio /
         kHvSenseResistorOhms;
}

constexpr bool hvSenseIsFault(uint16_t counts,
                              uint16_t full_scale = kAdcFullScale,
                              float reference_volts = kAdcReferenceVolts) {
  return countsToVolts(counts, full_scale, reference_volts) >=
         kHvSenseFaultVolts;
}

} // namespace conversions
} // namespace mouse_joystick
