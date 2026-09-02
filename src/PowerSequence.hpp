// Bringing the switched power domain up and down, in the only order that is
// safe.
//
// This board has two power domains and a bus switch between them. The ordering
// rules below are not stylistic -- they are the reason the bus switch exists:
//
//   power up:    HV_EN high -> wait for +3.3V_SW -> settle -> close bus switch
//   power down:  open bus switch -> HV_EN low
//
// Close the bus switch before the rail is up, or open it after the rail is
// down, and MCU pins drive into an unpowered TMC5130A. That forward-biases its
// input ESD clamps and back-powers a dead rail through them, which is exactly
// what the switch is there to prevent.
//
// This is the only file in the firmware that raises HV_EN.

#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace mouse_joystick {

struct RailState {
  uint16_t sense_counts; // raw GP26 reading
  float divided_volts;   // at the ADC pin
  float rail_volts;      // undivided +3.3V_SW
  bool up;               // above the "rail is live" threshold
  bool off;              // below the "rail is dead" threshold
};

struct HvDiagnostics {
  uint16_t sns_counts;
  float sns_volts; // across R207 249 ohm; load current needs the switch's
                   // sense ratio, which this firmware does not carry yet
  bool fault_n;    // true = no latched fault
  bool diag_enabled_during_read;
};

// What the rail actually did while HV_EN was high. Recorded even on failure:
// "the rail never moved" and "the rail came up but not far enough" are
// different faults, and without the peak you cannot tell them apart after the
// fact.
struct PowerUpReport {
  uint32_t rise_time_ms; // HV_EN high to crossing kRailUpVolts
  uint16_t peak_counts;  // highest raw reading seen during the window
  float peak_rail_volts; // that reading, undivided
  uint32_t samples;      // how many times the rail was polled
};

enum class PowerUpResult {
  kOk,
  kRailAlreadyLive, // refused to start: the rail was up before we asked
  kRailNeverCameUp, // HV_EN raised but +3.3V_SW never reached threshold
  kFaultLatched,    // the 36 V switch reported a fault
};

class PowerSequence {
public:
  // +3.3V_SW is considered live above this and dead below the "off" figure.
  // Nominal is 3.3 V. The gap between the two is deliberate: a reading in
  // between means something is wrong, not that the rail is merely settling.
  static constexpr float kRailUpVolts = 3.00f;
  static constexpr float kRailOffVolts = 0.30f;

  // Generous. Charging +36V_SW's ~208 uF through the switch's ~1.2 A current
  // limit alone accounts for ~6 ms, then the buck and the LDO have their own
  // soft-start. The measured figure is what matters; this is only a give-up
  // point.
  static constexpr uint32_t kRailRiseTimeoutMs = 1000;

  // After the rail is up, before touching the bus. The TMC5130A needs its
  // internal reset and charge pump, and its 16 MHz clock source X1201 is itself
  // on +3.3V_SW and has only just been powered.
  static constexpr uint32_t kRailSettleMs = 100;

  static RailState readRail();

  // One sample of the switched rail and the switch's current sense.
  struct ProfileSample {
    uint16_t t_ms;
    uint16_t rail_counts;
    uint16_t sns_counts;
  };

  // Raise HV_EN and record what happens, at roughly 1 ms intervals, then power
  // down unconditionally. This exists because a rail that comes up and then
  // collapses looks identical to a rail that never came up if you only sample
  // before and after.
  //
  // HV_DIAG_EN is held high for the duration so HV_SNS means something, and
  // lowered again at the end -- leaving it high with HV_EN low is what causes
  // the leakage creep that breaks power-cycle-as-reset.
  //
  // Returns the number of samples written. Always ends powered down, which also
  // clears any latched fault.
  static uint16_t profilePowerUp(ProfileSample *samples, uint16_t capacity);

  // One attempt of a software retry sequence.
  struct RetryAttempt {
    uint16_t peak_counts;  // highest rail reading during the on-phase
    uint16_t final_counts; // rail at the end of the on-phase
    uint16_t held_ms;      // how long the rail stayed at or above kRailUpVolts
    bool up_at_end;
  };

  // Software soft-start. The TPS1HTC100-Q1 latches off on thermal shutdown
  // because R203 ties LATCH high, and dropping HV_EN is what clears the latch
  // -- so firmware can do its own retry even though the device will not.
  //
  // This is a diagnostic, not a shipping power-up path. If the rail holds for
  // progressively longer across attempts, or eventually stays up, then charging
  // 208 uF is what trips the switch and the fault is a design margin problem
  // rather than a defect. If every attempt looks identical, it is not charging.
  //
  // Bounded on purpose: each on-phase puts real dissipation into the switch,
  // and this deliberately provokes its protection.
  static uint16_t softStartRetry(RetryAttempt *attempts, uint16_t max_attempts,
                                 uint16_t on_ms, uint16_t off_ms);

  // Raise HV_EN and wait for +3.3V_SW. Does NOT touch the bus switch: the
  // caller closes it after this returns kOk, so the ordering stays visible at
  // the call site. Drops HV_EN again on any failure.
  static PowerUpResult powerUp(PowerUpReport &report);

  // Raise HV_DIAG_EN, read the switch's current sense and fault line, then
  // lower it again. Never leave DIAG_EN high: held high it keeps the switch's
  // OFF-state open-load detection enabled, trickling ~240 uA into +36V_SW,
  // which makes the rail creep up and the switched domain hiccup instead of
  // staying off.
  static HvDiagnostics readHvDiagnostics();

  // Open the bus switch, then drop HV_EN and HV_DIAG_EN. Safe to call at any
  // time, including from a fault path, and safe to call twice.
  static void powerDown();

  // Close the bus switch. Only legal once the rail is up and settled; the
  // function checks and refuses otherwise.
  static bool connectBus();
  static void disconnectBus();
  static bool busConnected();

private:
  static bool bus_connected_;
};

} // namespace mouse_joystick
