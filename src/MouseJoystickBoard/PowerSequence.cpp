#include <Arduino.h>

#include "BoardPins.hpp"
#include "Conversions.hpp"
#include "PowerSequence.hpp"

namespace mouse_joystick {

bool PowerSequence::bus_connected_ = false;

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

PowerUpResult PowerSequence::powerUp(PowerUpReport &report) {
  report.rise_time_ms = 0;
  report.peak_counts = 0;
  report.peak_rail_volts = 0.0f;
  report.samples = 0;

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

  pinMode(pins::kHvEn, OUTPUT);
  const uint32_t start_ms = millis();
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

  if (!up) {
    digitalWrite(pins::kHvEn, LOW);
    return PowerUpResult::kRailNeverCameUp;
  }

  delay(kRailSettleMs);

  // ~{HV_FAULT} is open drain with a 10k pull-up to +3.3V_MCU, so low means a
  // latched fault. LATCH is tied through R203 to +3.3V_MCU on this board, which
  // means faults latch off rather than auto-retrying -- dropping HV_EN is how
  // they clear.
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
    digitalWrite(pins::kHvEn, LOW);
    return PowerUpResult::kFaultLatched;
  }

  return PowerUpResult::kOk;
}

uint16_t PowerSequence::profilePowerUp(ProfileSample *samples,
                                       uint16_t capacity) {
  if (samples == nullptr || capacity == 0) {
    return 0;
  }

  powerDown();
  delay(50);

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
}

} // namespace mouse_joystick
