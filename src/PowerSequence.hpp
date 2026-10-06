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

// How the rail was actually brought up. Recorded so a bring-up log says which
// mechanism a given board needed, rather than leaving it to be re-derived.
enum class PowerUpMethod {
  kNone,
  kLatchGpio, // LATCH driven low for the ramp, raised after -- the real fix
  kStaircase, // firmware burst-charges +36V_SW -- the workaround for a board
              // whose LATCH is strapped to +3.3V_MCU through R203
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
  PowerUpMethod method;  // which mechanism brought it up
  // LATCH-GPIO method only: HV_EN high to HV_SNS leaving the switch's fault
  // band for kBulkChargeQuietMs, i.e. +36V_SW's bulk charged and latch-off
  // armed. 0 when the method did not measure it. `charge_timed_out` means the
  // sense never left the band within kBulkChargeTimeoutMs: the switch stayed
  // in current limit, which is a short, not a slow charge.
  uint32_t charge_ms;
  bool charge_timed_out;
};

enum class PowerUpResult {
  kOk,
  kRailAlreadyLive, // refused to start: the rail was up before we asked
  kRailNeverCameUp, // HV_EN raised but +3.3V_SW never reached threshold
  kFaultLatched,    // the 36 V switch reported a fault
};

// What a deliberate single-shot HV_EN assertion did. This is the test for
// "has the LATCH fix landed on this revision?" -- on a board with LATCH
// strapped high it fails, on one with LATCH low or on a GPIO it succeeds.
struct LatchProbe {
  bool rail_came_up; // reached kRailUpVolts at all
  bool rail_held;    // still up at the end of the window
  uint16_t held_ms;  // how long it stayed up
  uint16_t peak_counts;
  bool fault_n;     // ~{HV_FAULT} at the end; false = latched fault
  bool fix_present; // rail_held && no latched fault
  bool vs_absent;   // no fault AND no rail AND no sense current: no 36 V in

  // Whether the probe managed to start from a genuinely discharged +36V_SW,
  // and how long that took. THIS QUALIFIES EVERY OTHER FIELD. A probe that
  // starts with the bulk still charged has no inrush to supply, so the switch
  // never trips and `fix_present` reads true on a board that does need the
  // workaround. Observed on v0.3 board E4654C159F171E34 on 2026-09-09: PASS
  // when run straight after the stage 2 sequence, FAIL when run cold, same
  // board, same firmware, minutes apart.
  bool started_cold;
  uint16_t discharge_ms;
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
  // LATCH-GPIO method: how long to keep LATCH low (auto-retry) after HV_EN
  // before giving up on the bulk ever charging, and how many consecutive
  // milliseconds HV_SNS must read below the fault level before latch-off is
  // armed. The rail reading cannot stand in for this: the bucks start from a
  // few volts of +36V_SW, so +3.3V_SW is live long before the switch has
  // finished charging 208 uF through its current limit. Measured on pcb v1.0
  // unit E4654C159F274534, 2026-10-05: PG_5V 6.6 ms and PG_12V 10.9 ms after
  // HV_EN, latch-off armed at ~20 ms, the switch tripped at ~37 ms with
  // latch-off armed and stayed off.
  static constexpr uint32_t kBulkChargeTimeoutMs = 1000;
  static constexpr uint32_t kBulkChargeQuietMs = 20;

  // After the rail is up, before touching the bus. The TMC5130A needs its
  // internal reset and charge pump, and its 16 MHz clock source X1201 is itself
  // on +3.3V_SW and has only just been powered.
  static constexpr uint32_t kRailSettleMs = 100;

  // How long to allow +36V_SW's bulk to bleed down before giving up on the
  // switched rail being dead. Measured at a few seconds; this is a give-up
  // point, not an expected duration.
  static constexpr uint32_t kRailOffTimeoutMs = 10000;

  // After +3.3V_SW reads off, before treating +36V_SW as discharged. The rail
  // sense watches +3.3V_SW, and the buck holds that up until its input falls
  // below ~3.6 V, so "off" already means the bulk is nearly empty -- this is
  // margin for the last of it, not the bulk of the wait.
  static constexpr uint32_t kBulkBleedMarginMs = 200;

  static RailState readRail();

  // Wait for +3.3V_SW to actually fall after HV_EN has been dropped.
  //
  // Dropping HV_EN does not make the switched rail go away: +36V_SW's ~208 uF
  // keeps the buck running until it bleeds below the buck's minimum input, and
  // with only board leakage as a load that takes seconds. Measured on board
  // E4654C159F4D1334 on 2026-09-02: still 3.31 V immediately after HV_EN
  // dropped, 0.034 V a few seconds later.
  //
  // So anything that requires a dead rail as a precondition has to wait for it
  // rather than sample once. Returns true if the rail reached kRailOffVolts.
  static bool waitForRailOff(uint32_t timeout_ms);

  // Assert HV_EN once, plainly, and see whether the rail survives. Costs one
  // thermal trip on a board that still needs the workaround, so this is a
  // bring-up diagnostic, not something to run on every power-up.
  //
  // `vs_absent` distinguishes "no 36 V connected" from "the switch tripped":
  // below V_S,UVLOR (5.0-5.75 V) and VS_ISNS (5.9 V at DIAG_EN = 3.3 V) the
  // device does not operate and reports nothing at all, so HV_SNS stays at its
  // offset floor instead of pegging at ISNSFLT within 1 ms.
  //
  // Waits for waitForRailOff() before asserting, and reports whether it got
  // there in `started_cold`. It must: the whole test is whether the switch
  // survives charging +36V_SW's ~208 uF, so a run that begins with that bulk
  // already charged asks nothing of the switch and passes regardless.
  static LatchProbe probeLatchBehaviour(uint16_t window_ms);

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

  // What a staircase soft-start did.
  struct StaircaseReport {
    uint16_t pulses_issued;
    uint16_t rail_after_train; // rail counts immediately after the last pulse
    uint16_t peak_counts;      // highest rail reading during the hold window
    uint16_t final_counts;     // rail at the end of the hold window
    uint16_t held_ms;          // how long it stayed at or above kRailUpVolts
    uint16_t sns_counts;       // HV_SNS at the end, read with DIAG_EN high
    bool fault_n;              // ~{HV_FAULT} at the end; false = latched fault
    bool up_at_end;
  };

  // Charge +36V_SW in short current-limited bursts, then hold HV_EN high.
  //
  // Why this works where a single turn-on does not. The switch trips its own
  // relative thermal shutdown about 1-2 ms into charging 208 uF, having only
  // reached ~10-12 V, and R203 ties LATCH high so it latches instead of
  // retrying. Dropping HV_EN clears the latch. Crucially the bulk KEEPS its
  // charge between bursts: once the buck drops out the only load left on
  // +36V_SW is board leakage, measured at 0.87 Mohm, so tau is ~180 s. Each
  // burst therefore starts where the last one stopped and the rail walks up.
  //
  // This is the datasheet's own sanctioned arrangement for a capacitive load
  // with high parallel impedance -- "hitting thermal shutdown and trying again
  // is acceptable" -- implemented in firmware because the board's LATCH
  // strapping forbids the device from doing it itself.
  //
  // Total energy into the FET is unchanged (it is 1/2*C*V^2 either way), but
  // spread over the whole train instead of one 6.5 ms burst, which is the
  // gentler way to deliver it.
  //
  // off_us must stay short. Give the buck long enough and it bleeds the bulk
  // back down to its own dropout, every burst starts from the same place, and
  // the staircase never climbs.
  //
  // Leaves the rail UP on success, so the caller can close the bus and talk to
  // the driver. Powers down on failure.
  static bool staircaseStart(StaircaseReport &report, uint16_t pulses,
                             uint16_t on_us, uint16_t off_us, uint16_t hold_ms);

  // A self-zeroed reading of the 36 V input current.
  //
  // HV_SNS on its own cannot resolve the idle draw: the chain is 0.311 V/A, so
  // one 12-bit ADC count is 2.6 mA, and the quiet-state floor is 19-23 counts
  // of ADC offset and leakage that drifts a few counts between reads. The idle
  // load sits 2-6 counts above it. Averaging many samples and subtracting a
  // zero captured moments earlier, with HV_EN low, recovers a real number.
  struct HvCurrent {
    float zero_counts; // averaged, HV_EN low
    float live_counts; // averaged, HV_EN high
    float delta_counts;
    float milliamps;
    bool zero_was_fault; // the zero itself read the fault level -- suspect
    bool live_was_fault;
    bool rail_up;
    bool rail_discharged;  // +3.3V_SW read off before the zero was taken; if
                           // not, nothing below it was measured
    uint32_t discharge_ms; // how long that took (bulk bleed, ~700 ms)
  };

  // Averages `samples` reads at each point. DIAG_EN is high for both, which is
  // required for SNS to mean anything; the OFF-state open-load pull-up it also
  // enables trickles 240 uA into 208 uF, i.e. 1.15 V/s, so a few tens of ms of
  // zeroing costs a few tens of mV of creep. Leaves the rail UP on success.
  static HvCurrent measureHvCurrent(uint16_t samples);

  // Raise HV_EN and leave it raised, bus open and diagnostics off. For bench
  // work where the supply, not firmware, controls the rail: ramp the supply
  // from 0 V with HV_EN already high and the switch never sees any V_DS, so
  // there is no inrush event to trip it.
  static void holdEnable();

  // Bring the switched domain up. Does NOT touch the bus switch: the caller
  // closes it after this returns kOk, so the ordering stays visible at the
  // call site. Drops HV_EN again on any failure.
  //
  // A plain HV_EN assertion does NOT work on pcb v0.1-v0.3. `R203` straps the
  // 36 V switch's LATCH to +3.3V_MCU, so it latches off 1-2 ms into charging
  // 208 uF at only ~10-12 V, and the rail collapses 34-68 ms later when the
  // stored charge runs out. So this uses the staircase by default, and the
  // direct path only when MJ_HAS_LATCH_GPIO says the revision can hold LATCH
  // low itself. `report.method` says which ran.
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
