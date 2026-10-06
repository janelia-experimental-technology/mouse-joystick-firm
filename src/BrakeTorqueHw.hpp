// pcb v1.0's brake sink and torque bridge behind the `BrakeTorqueBackend`
// seam: the DAC80501Z at I2C 0x49 that sets the coil current, the sink's own
// BRAKE_ISENSE monitor and ~{BRAKE_FAULT} line, and the ADS8689 on SPI1 that
// reads the bridge.
//
// FIRMWARE ONLY. Needs Arduino, so like TmcMotion.hpp this header is not
// reachable from MouseJoystickBoard.hpp or the native test; the protocol
// layer and the corpus stay host-testable because they never include it.
//
// WHAT THE BENCH DECIDED HERE (v1.0 unit E4654C159F274534, 2026-10-06;
// numbers in ../mouse-joystick-pcb/docs/bringup-log.md, A4/A5):
//
//   * A code step to full scale overshoots the coil current to 127 % of the
//     setpoint for ~5 ms (90 %: +26 %, 75 %: +15 %, 60 %: +1 %): the coil's
//     L di/dt against the sink's drain headroom slews at 8-14 mA/ms, the
//     command filter asks faster, the loop integrator winds up. So this
//     backend RAMPS the code -- full scale in `ramp_full_scale_us`, below the
//     slew the coil can follow -- and reports `brakeSettled()` only
//     `settle_us` after the ramp arrives. The drive is released then.
//   * ~{BRAKE_FAULT} stayed high through that 62 mA overshoot, above the
//     56.4 mA ceiling. Whether the ceiling amplifier is slow, its indication
//     is blind, or the monitor exaggerates is a board question; until it is
//     answered the fault that matters is bit 10, BRAKE_ISENSE against the
//     setpoint, and bit 9 is reported when the pin does speak.
//   * The shunt-calibration switch is inert on the as-built board, so
//     `torqueCal()` drives TQ_CAL_EN faithfully and changes nothing.
//   * The torque reading at rest sat 69 mV below the ideal zero; the tare
//     (`torqueZero()`) is in ADC codes and the nominal 0.2715 V per N.cm
//     stands until the per-sensor certificate constant replaces it.
//
// BRING-UP IS THE BSP'S, in ticks. The parts live on the switched domain and
// share `Wire` with the angle DAC, so the BSP's rail-up phase machine calls
// brakeReset(), brakeConfigure() and torqueBegin() one tick apart once the
// rail is up and the bus closed, and end() when the rail goes down. Nothing
// here touches a bus unless the corresponding `*_up_` flag says so.

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <pico/time.h> // time_us_64
#include <stdint.h>

#include "ADS8689.hpp"
#include "BoardPins.hpp"
#include "BrakeTorqueBackend.hpp"
#include "Conversions.hpp"
#include "DAC80501.hpp"
#include "ProtocolBoard.hpp" // the fault bits

#if MJ_BOARD_REV < 2
#error "BrakeTorqueHw is pcb v1.0 (MJ_BOARD_REV 2) hardware; earlier revisions have no brake sink or torque bridge"
#endif

namespace mouse_joystick {

class BrakeTorqueHw : public BrakeTorqueBackend {
public:
  struct Config {
    // The code ramp: 0 to full scale in this long. 8 ms is 6.1 mA/ms, under
    // the 8-14 mA/ms the coil was seen to follow, and inside the brake's own
    // 6 ms mechanical response plus the settle.
    uint32_t ramp_full_scale_us = 8000;
    // After the ramp reaches the target, before `brakeSettled()`: the command
    // filter's ~2 ms and the coil's lag behind it. A full-scale step settled
    // by 16-17 ms from a 6 ms t90, a ramp lags less; 6 ms is margin.
    uint32_t settle_us = 6000;
    // Bit 10: BRAKE_ISENSE away from the setpoint by more than this, for this
    // long, once settled. The sweep agreed within 0.5 mA, so 15 % of full
    // scale (7.3 mA) catches an open coil or a dead pass device, not drift;
    // the floor covers the monitor's 0.55 mA-equivalent floor near zero.
    float current_tolerance_fraction = 0.15f;
    float current_tolerance_floor_amps = 0.002f;
    uint32_t current_fault_us = 50000;
    // Bit 9: ~{BRAKE_FAULT} low for this long while the brake is engaged.
    uint32_t fault_pin_us = 5000;
    // Bit 12: the ADC within this many codes of either rail.
    uint16_t torque_saturation_codes = 64;
    // SPI1 clock. 2 MHz decodes on a 10 MSa/s capture; a frame is 16 us.
    uint32_t adc_spi_hz = 2000000;
  };

  // `spi` is the RP2040 core's SPI1 object (its pin setters are not on the
  // Arduino base class); the transport sees it as a plain SPIClass.
  BrakeTorqueHw(TwoWire &wire, SPIClassRP2040 &spi, const Config &config)
      : brake_transport_(wire, kBrakeDacI2cAddress),
        brake_dac_(brake_transport_),
        adc_transport_(spi, pins::kTqConvstCsN, config.adc_spi_hz),
        adc_(adc_transport_), spi_(spi), cfg_(config) {}
  BrakeTorqueHw(TwoWire &wire, SPIClassRP2040 &spi)
      : BrakeTorqueHw(wire, spi, Config()) {}

  // ------------------------------------------------------- bring-up (BSP)

  // Step 1: the brake DAC's soft reset frame. Wire must be begun. The part
  // wants ~1 ms before the next frame, which is why the BSP waits a tick.
  bool brakeReset() { return brake_dac_.softReset(); }

  // Step 2: CONFIG, GAIN 0x0101, SYNC, and GAIN read back. The DAC80501Z
  // resets to code 0, so the coil is at 0 mA from rail-up until written.
  bool brakeConfigure() {
    dac80501::Config c = dac80501::arduinoConfig();
    c.gain_register = conversions::kBrakeDacGainRegister;
    c.soft_reset_first = false; // done, and settled, a tick ago
    c.delay_ms = nullptr;
    if (!brake_dac_.begin(c)) {
      return false;
    }
    if (brake_dac_.verifyConfiguration() != dac80501::VerifyResult::kVerified) {
      brake_dac_.invalidate();
      return false;
    }
    // The fault line is the brake block's output and high-Z with its supply
    // down; a weak pull-up makes an undriven line read "not limiting".
    pinMode(pins::kBrakeFaultN, INPUT_PULLUP);
    analogReadResolution(conversions::kAdcResolutionBits);
    brake_code_ = 0;
    target_code_ = 0;
    brake_up_ = true;
    return true;
  }

  // Step 3: SPI1 claimed, the ADS8689's range written and read back, SDO
  // proven by the loopback patterns. About 20 frames, ~0.5 ms.
  bool torqueBegin() {
    spi_.setSCK(pins::kTqSpiSck);
    spi_.setTX(pins::kTqSpiCopi);
    spi_.setRX(pins::kTqSpiCipo);
    spi_.begin();
    adc_transport_.begin();
    ads8689::Config c;
    c.range = ads8689::kRangeUnipolar1p25;
    c.loopback_check = true;
    torque_up_ = adc_.begin(c, &adc_report_);
    if (!torque_up_) {
      pendFault(fault::kTorqueAdc, "torque_adc_absent");
    }
    torque_saturated_ = false;
    return torque_up_;
  }
  const ads8689::BeginReport &adcReport() const { return adc_report_; }

  // The rail went down: both parts reset with it. Forget them without a
  // frame (clocking a dead part back-powers its rail) and release the bus.
  void end() {
    brake_dac_.invalidate();
    adc_.invalidate();
    if (torque_up_) {
      spi_.end();
    }
    pinMode(pins::kBrakeFaultN, INPUT);
    digitalWrite(pins::kTqCalEn, LOW);
    cal_on_ = false;
    brake_up_ = false;
    torque_up_ = false;
    engaged_ = false;
    ramping_ = false;
    brake_code_ = 0;
    target_code_ = 0;
    // The last conversion before the rail went was of a dying bridge (one
    // read -6.1 N.cm on 2026-10-06): the field reads zero while down, and
    // a saturation that the collapse itself produced is not a fault.
    torque_code_ = zero_code_;
    torque_saturated_ = false;
    fault_pending_ = false;
  }

  bool brakeUp() const { return brake_up_; }
  bool torqueUp() const { return torque_up_; }

  // ------------------------------------------------- BrakeTorqueBackend

  void poll(uint64_t now_us) override {
    // Once per advance of the protocol clock and never twice within half a
    // millisecond of wall-clock time, as TmcMotion::poll() (which explains
    // both guards): the other stops of the same advance reuse the last
    // conversion and monitor reading.
    const uint64_t wall_us = time_us_64();
    if (polled_this_advance_ ||
        (last_poll_wall_us_ != 0 &&
         (wall_us - last_poll_wall_us_) < kPollMinIntervalUs)) {
      now_us_ = now_us;
      return;
    }
    polled_this_advance_ = true;
    last_poll_wall_us_ = wall_us;
    const uint32_t poll_t0 = micros();
    pollParts(now_us);
    const uint32_t cost = micros() - poll_t0;
    ++poll_calls_;
    if (cost > poll_cost_max_us_) {
      poll_cost_max_us_ = cost;
    }
  }

  // The worst `poll()` since the last read and how many there were (POLL_STAT),
  // and inside it the worst ADS8689 sample and the worst BRAKE_ISENSE read.
  uint32_t takePollCostMaxUs(uint32_t *calls, uint32_t *sample_max_us = nullptr,
                             uint32_t *isense_max_us = nullptr) {
    const uint32_t m = poll_cost_max_us_;
    if (calls != nullptr) {
      *calls = poll_calls_;
    }
    if (sample_max_us != nullptr) {
      *sample_max_us = sample_max_us_;
    }
    if (isense_max_us != nullptr) {
      *isense_max_us = isense_max_us_;
    }
    poll_cost_max_us_ = 0;
    poll_calls_ = 0;
    sample_max_us_ = 0;
    isense_max_us_ = 0;
    return m;
  }

  static constexpr uint64_t kPollMinIntervalUs = 500u;
  void beginAdvance() override { polled_this_advance_ = false; }

private:
  uint32_t poll_cost_max_us_ = 0;
  uint32_t poll_calls_ = 0;
  uint32_t sample_max_us_ = 0;
  uint32_t isense_max_us_ = 0;
  uint64_t last_poll_wall_us_ = 0; // time_us_64() of the last real poll
  bool polled_this_advance_ = false;

  void pollParts(uint64_t now_us) {
    now_us_ = now_us;

    if (torque_up_) {
      // One conversion per tick: a CONVST pulse, the wait, one frame (~37 us
      // at 2 MHz measured). The reading is this tick's, not the previous one's.
      const uint32_t s0 = micros();
      torque_code_ = adc_.sample();
      const uint32_t s_cost = micros() - s0;
      if (s_cost > sample_max_us_) {
        sample_max_us_ = s_cost;
      }
      const bool saturated =
          torque_code_ <= cfg_.torque_saturation_codes ||
          torque_code_ >= static_cast<uint16_t>(0xFFFFu - cfg_.torque_saturation_codes);
      if (saturated && !torque_saturated_) {
        pendFault(fault::kTorqueRange, "torque_saturated");
      }
      torque_saturated_ = saturated;
    }

    if (!brake_up_) {
      return;
    }

    // The ramp: one code write per tick toward the target, at the slew the
    // configuration names, from wherever the code was.
    if (ramping_) {
      const uint64_t dt = now_us - last_ramp_us_;
      last_ramp_us_ = now_us;
      uint32_t step = static_cast<uint32_t>(
          (static_cast<uint64_t>(dac80501::kCodeMax) * dt) / cfg_.ramp_full_scale_us);
      if (step == 0) {
        step = 1;
      }
      int32_t next = brake_code_;
      if (target_code_ > brake_code_) {
        next = brake_code_ + static_cast<int32_t>(step);
        if (next > target_code_) {
          next = target_code_;
        }
      } else {
        next = brake_code_ - static_cast<int32_t>(step);
        if (next < target_code_) {
          next = target_code_;
        }
      }
      brake_code_ = next;
      static_cast<void>(brake_dac_.setCode(static_cast<uint16_t>(brake_code_)));
      if (brake_code_ == target_code_) {
        ramping_ = false;
        settled_at_us_ = now_us + cfg_.settle_us;
      }
    }

    // The monitor and the fault line, every tick.
    const uint32_t a0 = micros();
    isense_counts_ = static_cast<uint16_t>(analogRead(pins::kBrakeIsense));
    const uint32_t a_cost = micros() - a0;
    if (a_cost > isense_max_us_) {
      isense_max_us_ = a_cost;
    }
    fault_pin_low_ = digitalRead(pins::kBrakeFaultN) == LOW;

    if (!engaged_) {
      fault_pin_since_us_ = 0;
      current_bad_since_us_ = 0;
      return;
    }
    if (fault_pin_low_) {
      if (fault_pin_since_us_ == 0) {
        fault_pin_since_us_ = now_us;
      } else if (!fault_pin_reported_ &&
                 (now_us - fault_pin_since_us_) >= cfg_.fault_pin_us) {
        fault_pin_reported_ = true;
        pendFault(fault::kBrakeFault, "brake_fault_pin");
      }
    } else {
      fault_pin_since_us_ = 0;
    }
    if (brakeSettled()) {
      const float expected = conversions::brakeCodeToAmps(static_cast<uint16_t>(target_code_));
      const float measured = conversions::brakeIsenseCountsToAmps(isense_counts_);
      float tolerance = cfg_.current_tolerance_fraction * conversions::kBrakeFullScaleAmps;
      if (tolerance < cfg_.current_tolerance_floor_amps) {
        tolerance = cfg_.current_tolerance_floor_amps;
      }
      if (fabsf(measured - expected) > tolerance) {
        if (current_bad_since_us_ == 0) {
          current_bad_since_us_ = now_us;
        } else if (!current_reported_ &&
                   (now_us - current_bad_since_us_) >= cfg_.current_fault_us) {
          current_reported_ = true;
          pendFault(fault::kBrakeCurrent, "brake_current_off_setpoint");
        }
      } else {
        current_bad_since_us_ = 0;
      }
    }
  }

public:
  bool torqueValid() const override { return torque_up_ && !torque_saturated_; }
  double torqueNcm() const override {
    return torque_up_ ? conversions::torqueAdcCodeToNcm(torque_code_, zero_code_)
                      : 0.0;
  }
  double torqueRawNcm() const override {
    return torque_up_ ? conversions::torqueAdcCodeToNcm(torque_code_) : 0.0;
  }
  void torqueZero() override { zero_code_ = torque_code_; }
  void torqueCal(bool on) override {
    cal_on_ = on;
    digitalWrite(pins::kTqCalEn, on ? HIGH : LOW);
  }
  bool torqueCalOn() const override { return cal_on_; }

  bool brakeEngage(double torque_ncm) override {
    if (!brake_up_) {
      return false;
    }
    // The nominal line, clamped at the sink's full scale (99.6 % of the
    // brake's rating). The per-unit curve (rig plan C4) replaces it later.
    const float amps = static_cast<float>(torque_ncm) / conversions::kBrakeRatedTorqueNcm *
                       conversions::kBrakeRatedAmps;
    target_code_ = conversions::brakeAmpsToCode(amps < 0.0f ? 0.0f : amps);
    engaged_ = true;
    fault_pin_reported_ = false;
    current_reported_ = false;
    fault_pin_since_us_ = 0;
    current_bad_since_us_ = 0;
    last_ramp_us_ = now_us_;
    ramping_ = (target_code_ != brake_code_);
    if (!ramping_) {
      settled_at_us_ = now_us_ + cfg_.settle_us;
    }
    return true;
  }
  bool brakeSettled() const override {
    return engaged_ && !ramping_ && now_us_ >= settled_at_us_;
  }
  void brakeRelease() override {
    engaged_ = false;
    ramping_ = false;
    target_code_ = 0;
    brake_code_ = 0;
    if (brake_up_) {
      static_cast<void>(brake_dac_.setCode(0)); // the fall is clean: no ramp down
    }
  }
  double brakeMeasuredNcm() const override {
    return conversions::brakeAmpsToTorqueNcm(
        conversions::brakeIsenseCountsToAmps(isense_counts_));
  }

  bool consumeFault(uint8_t *bit, const char **detail) override {
    if (!fault_pending_) {
      return false;
    }
    fault_pending_ = false;
    if (bit != nullptr) {
      *bit = fault_bit_;
    }
    if (detail != nullptr) {
      *detail = fault_detail_;
    }
    return true;
  }

  // For the trace and the bench.
  bool brakeEngaged() const { return engaged_; }
  bool brakeRamping() const { return ramping_; }
  uint16_t brakeCode() const { return static_cast<uint16_t>(brake_code_); }
  uint16_t isenseCounts() const { return isense_counts_; }
  bool faultPinLow() const { return fault_pin_low_; }
  uint16_t torqueCode() const { return torque_code_; }
  uint16_t torqueZeroCode() const { return zero_code_; }

private:
  void pendFault(uint8_t bit, const char *detail) {
    // The first fault wins until the protocol layer has consumed it.
    if (!fault_pending_) {
      fault_pending_ = true;
      fault_bit_ = bit;
      fault_detail_ = detail;
    }
  }

  dac80501::I2cTransport brake_transport_;
  dac80501::Dac80501 brake_dac_;
  ads8689::SpiTransport adc_transport_;
  ads8689::Ads8689 adc_;
  ads8689::BeginReport adc_report_;
  SPIClassRP2040 &spi_;
  Config cfg_;

  uint64_t now_us_ = 0;
  bool brake_up_ = false;
  bool torque_up_ = false;

  bool engaged_ = false;
  bool ramping_ = false;
  int32_t brake_code_ = 0;
  int32_t target_code_ = 0;
  uint64_t last_ramp_us_ = 0;
  uint64_t settled_at_us_ = 0;
  uint16_t isense_counts_ = 0;
  bool fault_pin_low_ = false;
  uint64_t fault_pin_since_us_ = 0;
  uint64_t current_bad_since_us_ = 0;
  bool fault_pin_reported_ = false;
  bool current_reported_ = false;

  uint16_t torque_code_ = conversions::kTorqueZeroCodeNominal;
  uint16_t zero_code_ = conversions::kTorqueZeroCodeNominal;
  bool torque_saturated_ = false;
  bool cal_on_ = false;

  bool fault_pending_ = false;
  uint8_t fault_bit_ = 0;
  const char *fault_detail_ = "";
};

} // namespace mouse_joystick
