#include <Arduino.h>

#include "BoardPins.hpp"
#include "Conversions.hpp"
#include "PowerSequence.hpp"

namespace mouse_joystick {

bool PowerSequence::bus_connected_ = false;

// Staircase defaults, used by powerUp() on a revision whose LATCH is strapped.
// 1200/800 us is well inside the switch's t_ON_pw (60-134 us) and t_OFF_pw
// (82-175 us). 30 bursts is ~5x the 5-6 the arithmetic says are needed, and the
// whole train costs 60 ms. The off-time is the parameter that matters: give the
// buck long enough and it bleeds the bulk back to its own dropout between
// bursts, and the staircase never climbs.
constexpr uint16_t kStaircasePulses = 30;
constexpr uint16_t kStaircaseOnUs = 1200;
constexpr uint16_t kStaircaseOffUs = 800;
constexpr uint16_t kStaircaseHoldMs = 20;

RailState PowerSequence::readRail() {
  RailState state;
  // Set the resolution here, not in an init function. This was a real bug: the
  // Arduino core defaults to 10 bits, conversions assume 12, and the resulting
  // 4x error made the rail-up threshold unreachable while every reading still
  // looked plausible. Self-contained beats an ordering dependency.
  analogReadResolution(conversions::kAdcResolutionBits);
  state.sense_counts = analogRead(pins::kSwRailSenseDiv2);
  state.divided_volts = conversions::countsToVolts(state.sense_counts);
  state.rail_volts = conversions::swRailVolts(state.sense_counts);
  state.up = state.rail_volts >= kRailUpVolts;
  state.off = state.rail_volts <= kRailOffVolts;
  return state;
}

bool PowerSequence::waitForRailOff(uint32_t timeout_ms) {
  const uint32_t deadline = millis() + timeout_ms;
  while (millis() < deadline) {
    if (readRail().off) {
      return true;
    }
    delay(10);
  }
  return readRail().off;
}

PowerUpResult PowerSequence::powerUp(PowerUpReport &report) {
  report.rise_time_ms = 0;
  report.peak_counts = 0;
  report.peak_rail_volts = 0.0f;
  report.samples = 0;
  report.method = PowerUpMethod::kNone;
  report.charge_ms = 0;
  report.charge_timed_out = false;

  // Refuse to start from anything but a known-dead rail. If +3.3V_SW is already
  // live with HV_EN low, the switch or its enable network is not doing what
  // this sequence assumes, and raising HV_EN would tell us nothing.
  const RailState before = readRail();
  if (!before.off) {
    return PowerUpResult::kRailAlreadyLive;
  }

  // The bus stays open across the whole power-up. Nothing may drive into the
  // TMC5130A until its VCC_IO rail is established.
  disconnectBus();

  const uint32_t start_ms = millis();

#if MJ_HAS_LATCH_GPIO
  // The real fix. Hold LATCH low so the switch retries its own way through the
  // capacitive charge, then raise it once the rail is up so that a genuine
  // fault latches off instead of retrying forever.
  report.method = PowerUpMethod::kLatchGpio;
  pinMode(pins::kHvLatch, OUTPUT);
  digitalWrite(pins::kHvLatch, LOW);

  pinMode(pins::kHvEn, OUTPUT);
  digitalWrite(pins::kHvEn, HIGH);

  bool up = false;
  while ((millis() - start_ms) < kRailRiseTimeoutMs) {
    const RailState now = readRail();
    ++report.samples;
    if (now.sense_counts > report.peak_counts) {
      report.peak_counts = now.sense_counts;
      report.peak_rail_volts = now.rail_volts;
    }
    if (now.up) {
      report.rise_time_ms = millis() - start_ms;
      up = true;
      break;
    }
    delay(1);
  }
  if (up) {
    // Arm latch-off only once the bulk is charged, and +3.3V_SW being live
    // does not say that: the bucks run from a few volts of +36V_SW, so the
    // rail reads up while the switch is still in current limit. Watch HV_SNS
    // instead -- it sits at the fault level (V_SNSFH) for as long as the
    // switch limits -- and arm when it has read below the fault level for
    // kBulkChargeQuietMs. Taking the retry away earlier let the switch trip
    // on the remaining inrush with latch-off armed (v1.0 unit
    // E4654C159F274534, 2026-10-05: tripped at ~37 ms, armed at ~20 ms).
    pinMode(pins::kHvDiagEn, OUTPUT);
    digitalWrite(pins::kHvDiagEn, HIGH);
    delay(2);
    uint32_t quiet_ms = 0;
    while ((millis() - start_ms) < kBulkChargeTimeoutMs) {
      const uint16_t sns = analogRead(pins::kHvSns);
      if (conversions::hvSenseVolts(sns) >= conversions::kHvSenseFaultVolts) {
        quiet_ms = 0;
      } else {
        ++quiet_ms;
      }
      if (quiet_ms >= kBulkChargeQuietMs) {
        break;
      }
      delay(1);
    }
    digitalWrite(pins::kHvDiagEn, LOW);
    report.charge_ms = millis() - start_ms;
    report.charge_timed_out = quiet_ms < kBulkChargeQuietMs;
    if (report.charge_timed_out) {
      // Still limiting after a second is a short, not a slow charge. Leave
      // the retry on and let the fault check below report it.
      up = false;
    } else {
      digitalWrite(pins::kHvLatch, HIGH);
    }
  }
#else
  // The workaround. R203 straps LATCH high on this revision, so the switch
  // cannot retry and firmware has to burst-charge the bulk instead.
  report.method = PowerUpMethod::kStaircase;
  StaircaseReport staircase;
  const bool up = staircaseStart(staircase, kStaircasePulses, kStaircaseOnUs,
                                 kStaircaseOffUs, kStaircaseHoldMs);
  report.samples = kStaircaseHoldMs;
  report.peak_counts = staircase.peak_counts;
  report.peak_rail_volts = conversions::swRailVolts(staircase.peak_counts);
  report.rise_time_ms = millis() - start_ms;
#endif

  if (!up) {
    powerDown();
    return PowerUpResult::kRailNeverCameUp;
  }

  delay(kRailSettleMs);

  // ~{HV_FAULT} is open drain with a 10k pull-up to +3.3V_MCU, so low means a
  // latched fault. On a revision with LATCH strapped high, faults latch rather
  // than auto-retrying -- dropping HV_EN is how they clear.
  //
  // DIAG_EN must be high to read it. That is not optional and it was a bug
  // here: the switch's fault *reporting* is gated by DIAG_EN even though its
  // protection is not, so reading ~{HV_FAULT} with DIAG_EN low always returns
  // high and the check silently passed a latched fault.
  pinMode(pins::kHvDiagEn, OUTPUT);
  digitalWrite(pins::kHvDiagEn, HIGH);
  delay(2);
  pinMode(pins::kHvFaultN, INPUT);
  const bool faulted = digitalRead(pins::kHvFaultN) == LOW;
  digitalWrite(pins::kHvDiagEn, LOW);
  if (faulted) {
    powerDown();
    return PowerUpResult::kFaultLatched;
  }

  return PowerUpResult::kOk;
}

LatchProbe PowerSequence::probeLatchBehaviour(uint16_t window_ms) {
  LatchProbe probe{};

  // Wait for the bulk to actually bleed down, do not just pause. A fixed 50 ms
  // delay used to stand here, and it is not enough: +36V_SW's ~208 uF keeps the
  // buck alive for on the order of a second with no load, so a probe run soon
  // after the rail was last up finds the bulk still charged, presents the
  // switch with no inrush, and reports the LATCH fix as present on a board that
  // does not have it. That is a false PASS on the one command whose entire
  // purpose is to answer this question for a new revision.
  powerDown();
  const uint32_t discharge_start_ms = millis();
  probe.started_cold = waitForRailOff(kRailOffTimeoutMs);
  probe.discharge_ms = static_cast<uint16_t>(millis() - discharge_start_ms);
  delay(kBulkBleedMarginMs);

  analogReadResolution(conversions::kAdcResolutionBits);
  pinMode(pins::kSwRailSenseDiv2, INPUT);
  pinMode(pins::kHvSns, INPUT);

  // Diagnostics on from the start, so a fault -- or its absence -- is visible
  // from the first millisecond.
  pinMode(pins::kHvDiagEn, OUTPUT);
  digitalWrite(pins::kHvDiagEn, HIGH);
  delay(2);

#if MJ_HAS_LATCH_GPIO
  // Probe the *strapped* behaviour even on a board that can drive LATCH, so
  // the answer means the same thing on every revision.
  pinMode(pins::kHvLatch, OUTPUT);
  digitalWrite(pins::kHvLatch, HIGH);
#endif

  pinMode(pins::kHvEn, OUTPUT);
  const uint32_t start_ms = millis();
  digitalWrite(pins::kHvEn, HIGH);

  bool saw_fault_current = false;
  while ((millis() - start_ms) < window_ms) {
    const RailState now = readRail();
    if (now.sense_counts > probe.peak_counts) {
      probe.peak_counts = now.sense_counts;
    }
    if (now.up) {
      probe.rail_came_up = true;
      probe.held_ms = static_cast<uint16_t>(millis() - start_ms);
    }
    if (conversions::hvSenseIsFault(analogRead(pins::kHvSns))) {
      saw_fault_current = true;
    }
    delay(1);
  }
  probe.rail_held = readRail().up;

  pinMode(pins::kHvFaultN, INPUT);
  probe.fault_n = digitalRead(pins::kHvFaultN) == HIGH;
  digitalWrite(pins::kHvDiagEn, LOW);

  // A run that could not reach a discharged rail proves nothing, so it must not
  // be allowed to claim the fix is present.
  probe.fix_present = probe.rail_held && probe.fault_n && probe.started_cold;

  // Nothing at all happened: no rail, and the switch never even reported. Below
  // its UVLO the device does not operate, so this is an absent input supply
  // rather than a tripped switch.
  probe.vs_absent = !probe.rail_came_up && !saw_fault_current;

  powerDown();
  return probe;
}

uint16_t PowerSequence::profilePowerUp(ProfileSample *samples,
                                       uint16_t capacity) {
  if (samples == nullptr || capacity == 0) {
    return 0;
  }

  // Same precondition as probeLatchBehaviour: profiling a power-up that starts
  // with the bulk already charged profiles nothing.
  powerDown();
  waitForRailOff(kRailOffTimeoutMs);
  delay(kBulkBleedMarginMs);

  analogReadResolution(conversions::kAdcResolutionBits);
  pinMode(pins::kSwRailSenseDiv2, INPUT);
  pinMode(pins::kHvSns, INPUT);

  // Diagnostics on before the switch turns on, so HV_SNS is valid from the
  // first sample rather than from whenever we got round to enabling it.
  pinMode(pins::kHvDiagEn, OUTPUT);
  digitalWrite(pins::kHvDiagEn, HIGH);
  delay(2);

  pinMode(pins::kHvEn, OUTPUT);
  const uint32_t start_ms = millis();
  digitalWrite(pins::kHvEn, HIGH);

  uint16_t count = 0;
  while (count < capacity) {
    const uint32_t now = millis() - start_ms;
    samples[count].t_ms = static_cast<uint16_t>(now);
    samples[count].rail_counts = analogRead(pins::kSwRailSenseDiv2);
    samples[count].sns_counts = analogRead(pins::kHvSns);
    ++count;
    delay(1);
  }

  powerDown();
  return count;
}

uint16_t PowerSequence::softStartRetry(RetryAttempt *attempts,
                                       uint16_t max_attempts, uint16_t on_ms,
                                       uint16_t off_ms) {
  if (attempts == nullptr || max_attempts == 0) {
    return 0;
  }

  powerDown();
  delay(50);
  analogReadResolution(conversions::kAdcResolutionBits);
  pinMode(pins::kSwRailSenseDiv2, INPUT);

  // Diagnostics stay OFF for this test. Raising DIAG_EN while EN is low trips
  // the switch's OFF-state open-load detection, which is what made the fault
  // indication useless in the earlier profile.
  pinMode(pins::kHvDiagEn, OUTPUT);
  digitalWrite(pins::kHvDiagEn, LOW);

  pinMode(pins::kHvEn, OUTPUT);

  uint16_t n = 0;
  for (; n < max_attempts; ++n) {
    RetryAttempt &a = attempts[n];
    a.peak_counts = 0;
    a.final_counts = 0;
    a.held_ms = 0;
    a.up_at_end = false;

    const uint32_t start_ms = millis();
    digitalWrite(pins::kHvEn, HIGH);
    while ((millis() - start_ms) < on_ms) {
      const RailState now = readRail();
      if (now.sense_counts > a.peak_counts) {
        a.peak_counts = now.sense_counts;
      }
      if (now.up) {
        a.held_ms = static_cast<uint16_t>(millis() - start_ms);
      }
      a.final_counts = now.sense_counts;
      delay(1);
    }
    a.up_at_end = conversions::swRailVolts(a.final_counts) >= kRailUpVolts;

    // If the rail survived the whole on-phase, the switch is no longer tripping
    // and there is nothing left to learn from more attempts.
    if (a.up_at_end) {
      ++n;
      break;
    }

    digitalWrite(pins::kHvEn, LOW);
    delay(off_ms);
  }

  powerDown();
  return n;
}

bool PowerSequence::staircaseStart(StaircaseReport &report, uint16_t pulses,
                                   uint16_t on_us, uint16_t off_us,
                                   uint16_t hold_ms) {
  report = StaircaseReport{};

  powerDown();
  delay(50);

  analogReadResolution(conversions::kAdcResolutionBits);
  pinMode(pins::kSwRailSenseDiv2, INPUT);
  pinMode(pins::kHvSns, INPUT);

  // Diagnostics OFF for the train. EN is low between bursts, and DIAG_EN high
  // with EN low enables the switch's OFF-state open-load pull-up -- 150 kohm
  // from VS straight onto the node we are trying to charge. That both pollutes
  // the measurement and makes the switch report an open load instead of the
  // real fault.
  pinMode(pins::kHvDiagEn, OUTPUT);
  digitalWrite(pins::kHvDiagEn, LOW);

  pinMode(pins::kHvEn, OUTPUT);

  for (uint16_t i = 0; i < pulses; ++i) {
    digitalWrite(pins::kHvEn, HIGH);
    delayMicroseconds(on_us);
    digitalWrite(pins::kHvEn, LOW);
    delayMicroseconds(off_us);
    ++report.pulses_issued;
  }

  // Now hold. This is the actual test: the failure this works around collapses
  // the rail within 34-48 ms, so a rail still up after hold_ms has made it.
  digitalWrite(pins::kHvEn, HIGH);
  report.rail_after_train = analogRead(pins::kSwRailSenseDiv2);

  const uint32_t start_ms = millis();
  while ((millis() - start_ms) < hold_ms) {
    const RailState now = readRail();
    if (now.sense_counts > report.peak_counts) {
      report.peak_counts = now.sense_counts;
    }
    if (now.up) {
      report.held_ms = static_cast<uint16_t>(millis() - start_ms);
    }
    report.final_counts = now.sense_counts;
    delay(1);
  }
  report.up_at_end =
      conversions::swRailVolts(report.final_counts) >= kRailUpVolts;

  // Ask the switch for its own opinion, briefly, and only at the end.
  digitalWrite(pins::kHvDiagEn, HIGH);
  delay(2);
  report.sns_counts = analogRead(pins::kHvSns);
  pinMode(pins::kHvFaultN, INPUT);
  report.fault_n = digitalRead(pins::kHvFaultN) == HIGH;
  digitalWrite(pins::kHvDiagEn, LOW);

  if (!report.up_at_end) {
    powerDown();
  }
  return report.up_at_end;
}

PowerSequence::HvCurrent PowerSequence::measureHvCurrent(uint16_t samples) {
  HvCurrent out{};
  if (samples == 0) {
    samples = 1;
  }

  powerDown();
  delay(50);
  analogReadResolution(conversions::kAdcResolutionBits);
  pinMode(pins::kHvSns, INPUT);
  pinMode(pins::kSwRailSenseDiv2, INPUT);

  pinMode(pins::kHvDiagEn, OUTPUT);
  digitalWrite(pins::kHvDiagEn, HIGH);
  delay(2);

  uint32_t acc = 0;
  for (uint16_t i = 0; i < samples; ++i) {
    acc += analogRead(pins::kHvSns);
  }
  out.zero_counts = static_cast<float>(acc) / static_cast<float>(samples);
  out.zero_was_fault =
      conversions::hvSenseVolts(static_cast<uint16_t>(out.zero_counts)) >=
      conversions::kHvSenseFaultVolts;

  // Bring the rail up the same way everything else does. A raw HV_EN assertion
  // here was a bug: on a revision whose LATCH is strapped high it latches off,
  // so the "live" average read ISNSFLT and the delta was a fictitious 5 A.
  // powerUp() raises DIAG_EN itself for its fault check and lowers it again.
  digitalWrite(pins::kHvDiagEn, LOW);
  PowerUpReport up_report;
  const PowerUpResult result = powerUp(up_report);
  out.rail_up = (result == PowerUpResult::kOk);
  if (!out.rail_up) {
    powerDown();
    return out;
  }
  digitalWrite(pins::kHvDiagEn, HIGH);
  delay(2);

  acc = 0;
  for (uint16_t i = 0; i < samples; ++i) {
    acc += analogRead(pins::kHvSns);
  }
  out.live_counts = static_cast<float>(acc) / static_cast<float>(samples);
  out.live_was_fault =
      conversions::hvSenseVolts(static_cast<uint16_t>(out.live_counts)) >=
      conversions::kHvSenseFaultVolts;

  digitalWrite(pins::kHvDiagEn, LOW);

  out.delta_counts = out.live_counts - out.zero_counts;
  out.milliamps = 1000.0f * out.delta_counts * conversions::kAdcReferenceVolts /
                  static_cast<float>(conversions::kAdcFullScale) *
                  conversions::kHvSenseRatio /
                  conversions::kHvSenseResistorOhms;

  if (!out.rail_up) {
    powerDown();
  }
  return out;
}

void PowerSequence::holdEnable() {
  // Bus stays open: nothing may drive the driver until a caller has checked
  // the rail and closed the bus deliberately.
  disconnectBus();
  pinMode(pins::kHvDiagEn, OUTPUT);
  digitalWrite(pins::kHvDiagEn, LOW);
  pinMode(pins::kHvEn, OUTPUT);
  digitalWrite(pins::kHvEn, HIGH);
}

HvDiagnostics PowerSequence::readHvDiagnostics() {
  HvDiagnostics diag;

  pinMode(pins::kHvDiagEn, OUTPUT);
  digitalWrite(pins::kHvDiagEn, HIGH);
  // The switch needs a moment with diagnostics enabled before SNS is valid.
  delay(2);

  analogReadResolution(conversions::kAdcResolutionBits);
  diag.sns_counts = analogRead(pins::kHvSns);
  diag.sns_volts = conversions::hvSenseVolts(diag.sns_counts);

  pinMode(pins::kHvFaultN, INPUT);
  diag.fault_n = digitalRead(pins::kHvFaultN) == HIGH;
  diag.diag_enabled_during_read = true;

  // Back down immediately. See the header.
  digitalWrite(pins::kHvDiagEn, LOW);
  return diag;
}

bool PowerSequence::connectBus() {
  const RailState now = readRail();
  if (!now.up) {
    return false;
  }
  pinMode(pins::kBusSwEnN, OUTPUT);
  digitalWrite(pins::kBusSwEnN, LOW);
  bus_connected_ = true;
  return true;
}

void PowerSequence::disconnectBus() {
  pinMode(pins::kBusSwEnN, OUTPUT);
  digitalWrite(pins::kBusSwEnN, HIGH);
  bus_connected_ = false;
}

bool PowerSequence::busConnected() { return bus_connected_; }

void PowerSequence::powerDown() {
  // Bus first, always. Dropping the rail with the bus still closed is the
  // failure this ordering exists to prevent.
  disconnectBus();

  pinMode(pins::kHvEn, OUTPUT);
  digitalWrite(pins::kHvEn, LOW);
  pinMode(pins::kHvDiagEn, OUTPUT);
  digitalWrite(pins::kHvDiagEn, LOW);
#if MJ_HAS_LATCH_GPIO
  // Back to the auto-retry default, so a later plain assertion (holdEnable,
  // the profile) does not inherit latch-off from a probe that drove it high.
  pinMode(pins::kHvLatch, OUTPUT);
  digitalWrite(pins::kHvLatch, LOW);
#endif
}

} // namespace mouse_joystick
