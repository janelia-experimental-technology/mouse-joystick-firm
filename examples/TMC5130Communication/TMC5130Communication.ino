// Stage 2 bring-up: prove this board can talk to its TMC5130A.
//
// REQUIRES 36 V on the input terminal -- J201 at v0.1/v0.2, J1901 at v0.3.
// Stage 1 (examples/RP2040SelfTest) must have passed
// first; this program assumes the MCU, its clocks, its flash and its ADC are
// already known good and does not re-test them.
//
// What it does, in this order and for these reasons:
//
//   1. Safe state, and confirm +3.3V_SW is genuinely dead before starting.
//   2. Raise HV_EN. Wait for +3.3V_SW, measure how long it took.
//   3. Read the 36 V switch's own diagnostics: current sense and fault line.
//   4. Only now close the GPIO isolation bus switch.
//   5. Read TMC5130A registers over SPI0 with the driver still disabled.
//
// What it deliberately does NOT do:
//
//   * It never lowers ~{TMC_DRV_EN}. The driver's output stage stays off. No
//     current reaches the motor, and there is no code path here that could
//     change that -- enabling the driver requires the current-sense decision
//     (vsense = 1 with the 0.15 ohm shunts) which belongs to stage 3.
//   * It writes no registers. A TMC5130A read already exercises MOSI, SCK, CS
//     and MISO together, so reads alone prove the bus. Writes buy nothing here
//     and cost a way to leave the chip misconfigured.
//
// Watching it with a logic analyser: FW_TRIG1 goes high for the duration of
// each register-read burst, so trigger on LA5032 channel 17 rising and capture
// channels 2, 3, 4, 5 -- SCK, COPI, CIPO, CS. sigrok's `spi` decoder will then
// give you the datagrams to compare against the bytes printed here.

#include <MouseJoystickBoard.hpp>

using mouse_joystick::HvDiagnostics;
using mouse_joystick::PowerSequence;
using mouse_joystick::PowerUpReport;
using mouse_joystick::PowerUpResult;
using mouse_joystick::RailState;
using mouse_joystick::SafeState;
using mouse_joystick::SelfTest;

namespace tmc = mouse_joystick::tmc;

// Designators for the 36 V input block rotate between revisions: the whole
// block moved from the R2xx/U2xx range at v0.1/v0.2 into R19xx/U1901 at v0.3.
// Printing the wrong one sends whoever is probing to a part that is not on the
// board in front of them, which has already cost time on this project. Values
// and nets are stable; designators are not. Mapping read from the v0.3
// schematic and kicad_pcb, not inferred.
#if MJ_BOARD_REV == 0
#define MJ_REF_HV_CONN "J201"
#define MJ_REF_HV_SWITCH "U201"
#define MJ_REF_HV_DIODE "D201"
#define MJ_REF_HV_EN_SERIES "R201"
#define MJ_REF_HV_EN_PULLDOWN "R204"
#define MJ_REF_HV_LATCH_STRAP "R203"
#define MJ_REF_HV_ICL "R206"
#define MJ_REF_HV_SNS_BOTTOM "R207"
#else
#define MJ_REF_HV_CONN "J1901"
#define MJ_REF_HV_SWITCH "U1901"
#define MJ_REF_HV_DIODE "D1901"
#define MJ_REF_HV_EN_SERIES "R1901"
#define MJ_REF_HV_EN_PULLDOWN "R1904"
#define MJ_REF_HV_LATCH_STRAP "R1903"
#define MJ_REF_HV_ICL "R1906"
#define MJ_REF_HV_SNS_BOTTOM "R1907"
#endif

constexpr uint32_t kSerialBaudRate = 115200;
constexpr uint32_t kSerialWaitMs = 5000;

// The board is left powered down until asked. Stage 2 energises 36 V, so it
// does not do that merely because someone plugged in USB.
bool sequence_run = false;

void printMenu(Print &out) {
  out.println(F("commands"));
  out.println(F("  ?  this menu"));
  out.println(F("  s  run the full stage 2 sequence (raises HV_EN)"));
  out.println(F("  r  read and print the register set"));
  out.println(F("  i  read IOIN once"));
  out.println(F("  v  rail state now"));
  out.println(F("  d  36 V switch diagnostics (raises HV_DIAG_EN briefly)"));
  out.println(F("  p  profile a power-up attempt, 1 ms resolution"));
  out.println(F("  c  staircase soft-start: burst-charge +36V_SW, then hold"));
  out.println(F("  b  close the GPIO bus and start SPI0 (rail must be up)"));
  out.println(F("  m  measure 36 V input current, self-zeroed and averaged"));
  out.println(F("  l  LATCH probe: does this revision still need the"));
  out.println(F("     firmware workaround? (costs one thermal trip if it does)"));
  out.println(F("  h  raise HV_EN and HOLD it (for supply-ramp bench tests)"));
  out.println(F("  w  software soft-start: retry HV_EN, 10 attempts"));
  out.println(F("  x  power down: open the bus, drop HV_EN"));
  out.println(F("  I  beacon the board serial on the InstrumentPort"));
  out.println(F("     triggers (no 36 V, no bus)"));
  out.println(F("  B  reboot into BOOTSEL"));
  out.println(F("  R  reboot"));
  out.println();
  out.println(
      F("There is no command to enable the driver. Stage 2 cannot put"));
  out.println(F("current through the motor."));
  out.println();
}

void printDatagram(Print &out, const char *label, const tmc::Datagram &d) {
  out.print(F("      "));
  out.print(label);
  for (size_t i = 0; i < tmc::kDatagramSize; ++i) {
    out.print(F(" "));
    if (d.bytes[i] < 0x10) {
      out.print(F("0"));
    }
    out.print(d.bytes[i], HEX);
  }
  out.println();
}

void printHex32(Print &out, uint32_t v) {
  out.print(F("0x"));
  for (int shift = 28; shift >= 0; shift -= 4) {
    out.print((v >> shift) & 0xF, HEX);
  }
}

void printRail(Print &out, const RailState &rail) {
  out.print(F("    +3.3V_SW: "));
  out.print(rail.rail_volts, 4);
  out.print(F(" V ("));
  out.print(rail.sense_counts);
  out.print(F(" counts, "));
  out.print(rail.divided_volts, 4);
  out.print(F(" V divided) -- "));
  if (rail.up) {
    out.println(F("live"));
  } else if (rail.off) {
    out.println(F("off"));
  } else {
    out.println(F("INDETERMINATE, neither live nor off"));
  }
}

void printHvSense(Print &out, uint16_t counts) {
  namespace conv = mouse_joystick::conversions;
  out.print(F("    HV_SNS: "));
  out.print(conv::hvSenseVolts(counts), 4);
  out.print(F(" V across " MJ_REF_HV_SNS_BOTTOM " 249R ("));
  out.print(counts);
  out.print(F(" counts) -- "));
  if (conv::hvSenseIsFault(counts)) {
    out.println(F("FAULT LEVEL (ISNSFLT), not a load reading"));
    out.print(F("      "));
    out.print(1000.0f * conv::hvSenseVolts(counts) /
                  conv::kHvSenseResistorOhms,
              2);
    out.println(F(" mA of sense current; datasheet ISNSFLT is 5.2-6.4 mA"));
  } else if (counts <= conv::kHvSenseFloorCounts) {
    out.println(F("at the offset floor, no measurable load"));
    out.println(F("      Quiet state with HV_EN low reads 18-22 counts and one"));
    out.println(F("      count is 2.6 mA, so idle draw is unresolvable. All"));
    out.println(F("      this says is 'far below the 1.15 A limit'."));
  } else {
    out.print(1000.0f * conv::hvLoadAmps(counts), 1);
    out.println(F(" mA of load current (KSNS 800 A/A)"));
  }
}

void printHvDiagnostics(Print &out, const HvDiagnostics &diag) {
  printHvSense(out, diag.sns_counts);
  out.print(F("    ~{HV_FAULT}: "));
  out.println(diag.fault_n ? F("high, no latched fault")
                           : F("LOW, FAULT LATCHED"));
}

bool readAndReport(Print &out, const char *name, uint8_t address,
                   uint32_t &data_out) {
  const tmc::ReadResult r = tmc::Spi::read(address);
  data_out = r.data;

  out.print(F("    "));
  out.print(name);
  out.print(F(" (0x"));
  if (address < 0x10) {
    out.print(F("0"));
  }
  out.print(address, HEX);
  out.print(F(") = "));
  printHex32(out, r.data);
  out.print(F("  status=0x"));
  if (r.status < 0x10) {
    out.print(F("0"));
  }
  out.println(r.status, HEX);

  printDatagram(out, "sent 1:", r.sent_first);
  printDatagram(out, "got  1:", r.got_first);
  printDatagram(out, "sent 2:", r.sent_second);
  printDatagram(out, "got  2:", r.got_second);

  if (r.all_zero) {
    out.println(F("    [FAIL] every received byte was 0x00."));
    out.println(
        F("      CIPO is not being driven. On this board TMC_SPI_CIPO has"));
    out.println(
        F("      a 10k pull-down (R1403), so an unpowered or unselected"));
    out.println(
        F("      driver reads as all zeros. Suspect, in order: the bus"));
    out.println(
        F("      switch enable, +3.3V_SW at the driver's VCC_IO, then CS."));
    return false;
  }
  if (r.all_ones) {
    out.println(F("    [FAIL] every received byte was 0xFF."));
    out.println(
        F("      CIPO stuck high, which the board's 10k pull-down should"));
    out.println(
        F("      prevent. Suspect a short or a bus switch channel held on."));
    return false;
  }
  return true;
}

void reportRegisters(Print &out) {
  if (!PowerSequence::busConnected()) {
    out.println(F("Bus is open. Run 's' first."));
    return;
  }

  // Mark the burst for the logic analyser: CH17 high for the whole read.
  SafeState::setInstrumentationEnabled(true);
  SafeState::setTrigger1(true);

  out.println(F("[5] TMC5130A registers, driver disabled throughout"));

  uint16_t checks = 0;
  uint16_t failures = 0;

  uint32_t ioin = 0;
  const bool ioin_ok = readAndReport(out, "IOIN     ", tmc::kIoin, ioin);
  ++checks;
  if (!ioin_ok) {
    ++failures;
  }

  if (ioin_ok) {
    const tmc::IoinDecode d = tmc::decodeIoin(ioin);

    out.print(F("    VERSION = 0x"));
    out.print(d.version, HEX);
    ++checks;
    if (d.version == tmc::kVersionTmc5130) {
      out.println(F("  [ ok ] TMC5130A"));
    } else if (d.version == tmc::kVersionTmc5160) {
      ++failures;
      out.println(F("  [FAIL] that is a TMC5160, not the fitted part"));
    } else {
      ++failures;
      out.println(F("  [FAIL] expected 0x11"));
    }

    // The valuable bit: DRV_ENN reads back the level firmware drives on GP3,
    // through the bus switch. Nothing else in this test proves that path.
    out.print(F("    DRV_ENN = "));
    out.print(d.drv_enn ? F("1") : F("0"));
    ++checks;
    if (d.drv_enn) {
      out.println(F("  [ ok ] driver disabled, and GP3 reaches the chip"));
    } else {
      ++failures;
      out.println(F("  [FAIL] driver reads ENABLED. Powering down."));
    }

    out.print(F("    SD_MODE = "));
    out.print(d.sd_mode ? F("1") : F("0"));
    ++checks;
    if (!d.sd_mode) {
      out.println(F("  [ ok ] motion-controller mode, tied to GND"));
    } else {
      ++failures;
      out.println(F("  [FAIL] step/dir mode; SD_MODE should be tied low"));
    }

    out.print(F("    inputs: REFL_STEP="));
    out.print(d.refl_step ? F("1") : F("0"));
    out.print(F(" REFR_DIR="));
    out.print(d.refr_dir ? F("1") : F("0"));
    out.print(F(" ENCA="));
    out.print(d.enca_dcin_cfg5 ? F("1") : F("0"));
    out.print(F(" ENCB="));
    out.print(d.encb_dcen_cfg4 ? F("1") : F("0"));
    out.print(F(" ENCN="));
    out.print(d.enc_n_dco_cfg6 ? F("1") : F("0"));
    out.print(F(" SWCOMP="));
    out.println(d.swcomp_in ? F("1") : F("0"));
    out.println(
        F("      With nothing plugged into the motor, encoder or limit"));
    out.println(
        F("      switch connectors these should all read 0, held there by"));
#if MJ_BOARD_REV == 0
    out.println(F("      R1405, the GND tie on REFR, R805/R806 and R1201."));
#else
    out.println(
        F("      That list is v0.2's and does not hold on v0.3, where"));
    out.println(
        F("      REFL_STEP = 1 is CORRECT with the switch unplugged and"));
    out.println(
        F("      the check below is simply wrong: U1301's IN pin has no"));
    out.println(
        F("      external pull -- it reaches only J1301 pad 2 -- so the"));
    out.println(
        F("      MAX6816 supplies its own pull-up, drives OUT high, and"));
    out.println(
        F("      nothing can override a driven output. There is no"));
    out.println(F("      LIMIT_SW pull-down anywhere on v0.3."));
#endif

    ++checks;
    if (tmc::ioinInvariantsMatch(ioin)) {
      out.println(F("    [ ok ] IOIN board-invariant fields correct:"));
      out.println(F("      VERSION, DRV_ENN, SD_MODE and REFR_DIR"));
    } else {
      ++failures;
      out.println(F("    [FAIL] an IOIN board-invariant field is wrong."));
      out.println(F("      Only VERSION, DRV_ENN, SD_MODE and REFR_DIR are"));
      out.println(F("      checked; the switch and encoder bits are reported"));
      out.println(F("      above but never asserted, because their value is"));
      out.println(F("      a property of what is plugged in."));
    }

    if (!d.drv_enn) {
      out.println(F("    Driver reported enabled. Dropping the rail now."));
      PowerSequence::powerDown();
      SafeState::setTrigger1(false);
      return;
    }
  }

  uint32_t gstat = 0;
  readAndReport(out, "GSTAT    ", tmc::kGstat, gstat);
  out.print(F("      reset="));
  out.print(tmc::bitIsSet(gstat, 0) ? F("1") : F("0"));
  out.print(F(" drv_err="));
  out.print(tmc::bitIsSet(gstat, 1) ? F("1") : F("0"));
  out.print(F(" uv_cp="));
  out.println(tmc::bitIsSet(gstat, 2) ? F("1") : F("0"));
  out.println(
      F("      reset=1 is expected on the first read after a power-up,"));
  out.println(F("      and reading GSTAT is what clears it."));

  uint32_t gconf = 0;
  readAndReport(out, "GCONF    ", tmc::kGconf, gconf);

  uint32_t chopconf = 0;
  readAndReport(out, "CHOPCONF ", tmc::kChopconf, chopconf);
  out.print(F("      vsense = "));
  out.println(tmc::vsenseSet(chopconf) ? F("1") : F("0"));
  out.println(
      F("      This board fits 0.15 ohm shunts. vsense resets to 0, and"));
  out.println(
      F("      0 with 0.15 ohm is roughly 1.8x the motor's rated current."));
  out.println(
      F("      Stage 3 must set it to 1 and read it back BEFORE it lowers"));
  out.println(F("      ~{TMC_DRV_EN}. Stage 2 only reports it."));

  uint32_t drv_status = 0;
  readAndReport(out, "DRV_STATUS", tmc::kDrvStatus, drv_status);

  uint32_t xactual = 0;
  readAndReport(out, "XACTUAL  ", tmc::kXactual, xactual);

  uint32_t xenc = 0;
  readAndReport(out, "X_ENC    ", tmc::kXenc, xenc);
  out.println(
      F("      Position comes from here and XACTUAL, never from a GPIO."));
  out.println(F("      The encoder does not reach the MCU on any revision."));

  SafeState::setTrigger1(false);

  out.println();
  out.print(F("    register checks: "));
  out.print(checks);
  out.print(F(", "));
  out.print(failures);
  out.println(failures == 0 ? F(" failed") : F(" FAILED"));
}

// Millisecond-resolution trace of a power-up attempt. Ends powered down.
void profilePowerUp(Print &out) {
  constexpr uint16_t kCapacity = 250;
  static PowerSequence::ProfileSample samples[kCapacity];

  out.println(
      F("Profiling a power-up attempt. HV_DIAG_EN is held high for the"));
  out.println(
      F("duration so HV_SNS is meaningful, and lowered again at the end."));
  out.println(F("Ends powered down, which clears any latched fault."));
  out.println();

  const uint16_t n = PowerSequence::profilePowerUp(samples, kCapacity);

  out.println(F("  t_ms  rail_cnt  +3.3V_SW      sns_cnt   HV_SNS"));
  uint16_t peak_rail = 0;
  uint16_t peak_sns = 0;
  for (uint16_t i = 0; i < n; ++i) {
    const PowerSequence::ProfileSample &s = samples[i];
    if (s.rail_counts > peak_rail) {
      peak_rail = s.rail_counts;
    }
    if (s.sns_counts > peak_sns) {
      peak_sns = s.sns_counts;
    }
    out.print(F("  "));
    out.print(s.t_ms);
    out.print(F("\t"));
    out.print(s.rail_counts);
    out.print(F("\t  "));
    out.print(mouse_joystick::conversions::swRailVolts(s.rail_counts), 3);
    out.print(F(" V\t"));
    out.print(s.sns_counts);
    out.print(F("\t  "));
    out.print(mouse_joystick::conversions::hvSenseVolts(s.sns_counts), 3);
    out.println(F(" V"));
  }

  out.println();
  out.print(F("  samples: "));
  out.println(n);
  out.print(F("  peak +3.3V_SW: "));
  out.print(mouse_joystick::conversions::swRailVolts(peak_rail), 3);
  out.println(F(" V"));
  out.print(F("  peak HV_SNS:   "));
  out.print(mouse_joystick::conversions::hvSenseVolts(peak_sns), 3);
  out.println(F(" V"));
  out.println(
      F("  HV_SNS in the 1.3-1.6 V band is the switch's ISNSFLT fault"));
  out.println(F("  current into " MJ_REF_HV_SNS_BOTTOM
                 " 249R, not a load-current reading."));
  out.println(F("  Powered down."));
}

// Diagnostic: does the rail come up if firmware retries after each latch-off?
void softStartRetry(Print &out) {
  constexpr uint16_t kAttempts = 10;
  constexpr uint16_t kOnMs = 60;
  constexpr uint16_t kOffMs = 250;
  static PowerSequence::RetryAttempt attempts[kAttempts];

  out.print(F("Software soft-start: up to "));
  out.print(kAttempts);
  out.print(F(" attempts of "));
  out.print(kOnMs);
  out.print(F(" ms on / "));
  out.print(kOffMs);
  out.println(F(" ms off. HV_DIAG_EN stays low."));
  out.println(F("Stops early if the rail survives a whole on-phase."));
  out.println(
      F("Reading it: held_ms growing across attempts means charging the"));
  out.println(F("208 uF bulk capacitance is what trips the switch. Identical"));
  out.println(
      F("attempts mean it is not charging, and the cause is elsewhere."));
  out.println();

  const uint16_t n =
      PowerSequence::softStartRetry(attempts, kAttempts, kOnMs, kOffMs);

  out.println(F("  attempt  peak_V   final_V  held_ms  up_at_end"));
  for (uint16_t i = 0; i < n; ++i) {
    const PowerSequence::RetryAttempt &a = attempts[i];
    out.print(F("  "));
    out.print(i + 1);
    out.print(F("\t   "));
    out.print(mouse_joystick::conversions::swRailVolts(a.peak_counts), 3);
    out.print(F("\t   "));
    out.print(mouse_joystick::conversions::swRailVolts(a.final_counts), 3);
    out.print(F("\t   "));
    out.print(a.held_ms);
    out.print(F("\t   "));
    out.println(a.up_at_end ? F("YES") : F("no"));
  }
  out.println();
  if (n > 0 && attempts[n - 1].up_at_end) {
    out.print(F("  Rail survived attempt "));
    out.print(n);
    out.println(F(". Firmware soft-start is a viable workaround, and the"));
    out.println(F("  diagnosis is capacitive-charging thermal shutdown."));
  } else {
    out.println(
        F("  Rail never survived an on-phase. Retrying does not help,"));
    out.println(
        F("  so charging the bulk capacitance is probably not the limit."));
  }
  out.println(F("  Powered down."));
}

// The workaround for the latched turn-on fault, and the test that proves it.
void staircase(Print &out) {
  constexpr uint16_t kPulses = 30;
  constexpr uint16_t kOnUs = 1200;
  constexpr uint16_t kOffUs = 800;
  constexpr uint16_t kHoldMs = 1000;

  out.println(F("Staircase soft-start."));
  out.print(F("  "));
  out.print(kPulses);
  out.print(F(" bursts of "));
  out.print(kOnUs);
  out.print(F(" us on / "));
  out.print(kOffUs);
  out.print(F(" us off, then HV_EN held high for "));
  out.print(kHoldMs);
  out.println(F(" ms."));
  out.println(F("  HV_DIAG_EN stays low for the train."));
  out.println();

  PowerSequence::StaircaseReport r;
  const bool ok = PowerSequence::staircaseStart(r, kPulses, kOnUs, kOffUs,
                                                kHoldMs);

  namespace conv = mouse_joystick::conversions;
  out.print(F("  bursts issued:   "));
  out.println(r.pulses_issued);
  out.print(F("  rail after train:"));
  out.print(conv::swRailVolts(r.rail_after_train), 3);
  out.println(F(" V"));
  out.print(F("  rail peak:       "));
  out.print(conv::swRailVolts(r.peak_counts), 3);
  out.println(F(" V"));
  out.print(F("  rail final:      "));
  out.print(conv::swRailVolts(r.final_counts), 3);
  out.println(F(" V"));
  out.print(F("  held up for:     "));
  out.print(r.held_ms);
  out.print(F(" ms of "));
  out.print(kHoldMs);
  out.println(F(" ms"));
  printHvSense(out, r.sns_counts);
  out.print(F("    ~{HV_FAULT}: "));
  out.println(r.fault_n ? F("high, no latched fault")
                        : F("LOW, FAULT LATCHED"));
  out.println();

  if (ok) {
    out.println(F("  [ ok ] the rail came up and STAYED up. The single-shot"));
    out.println(F("  turn-on failure is an inrush-energy problem, not a load"));
    out.println(F("  fault. Rail left live -- send 'b' to close the bus."));
  } else {
    out.println(F("  [FAIL] the rail did not survive the hold. Powered down."));
    out.println(F("  If held_ms is near the 34-48 ms single-shot figure the"));
    out.println(F("  train achieved nothing, so the bulk is being bled"));
    out.println(F("  between bursts or the cause is not inrush at all."));
  }
  out.println();
}

void measureCurrent(Print &out) {
  constexpr uint16_t kSamples = 2048;
  out.print(F("Measuring 36 V input current: "));
  out.print(kSamples);
  out.println(F(" samples at each point."));
  out.println(F("  Zero is taken with HV_EN low, moments before raising it,"));
  out.println(F("  so it cancels ADC offset and leakage rather than assuming"));
  out.println(F("  a floor. HV_DIAG_EN is high for both reads."));
  out.println();

  const PowerSequence::HvCurrent c = PowerSequence::measureHvCurrent(kSamples);

  out.print(F("  zero (HV_EN low):  "));
  out.print(c.zero_counts, 2);
  out.println(F(" counts"));
  out.print(F("  live (HV_EN high): "));
  out.print(c.live_counts, 2);
  out.println(F(" counts"));
  out.print(F("  delta:             "));
  out.print(c.delta_counts, 2);
  out.println(F(" counts"));
  out.print(F("  36 V input current: "));
  out.print(c.milliamps, 1);
  out.println(F(" mA   (KSNS 800 A/A, " MJ_REF_HV_SNS_BOTTOM
                 " 249R, 2.6 mA per count)"));
  out.print(F("  rail: "));
  out.println(c.rail_up ? F("live") : F("NOT UP -- the delta means nothing"));
  if (c.zero_was_fault || c.live_was_fault) {
    out.println(F("  [warn] a reading sat at the fault level (ISNSFLT), so it"));
    out.println(F("  is not a current measurement at all."));
  }
  if (c.rail_up) {
    out.println(F("  Rail left live -- send 'b' to close the bus, 'x' to stop."));
  }
  out.println();
}

// The one command to run on a new board revision.
void latchProbe(Print &out) {
  constexpr uint16_t kWindowMs = 250;

  out.println(F("LATCH probe: a plain single-shot HV_EN assertion."));
  out.println(F("  On pcb v0.1-v0.3 " MJ_REF_HV_LATCH_STRAP
                 " straps the 36 V switch's LATCH to"));
  out.println(F("  +3.3V_MCU, so it latches off 1-2 ms into charging 208 uF at"));
  out.println(F("  only ~10-12 V and the rail collapses when the stored charge"));
  out.println(F("  runs out. Expect FAIL on those, and PASS once LATCH reaches"));
  out.println(F("  a GPIO or is tied low."));
  out.println();

  const mouse_joystick::LatchProbe probe =
      PowerSequence::probeLatchBehaviour(kWindowMs);

  namespace conv = mouse_joystick::conversions;
  out.print(F("  rail came up:  "));
  out.println(probe.rail_came_up ? F("yes") : F("no"));
  out.print(F("  rail held:     "));
  out.println(probe.rail_held ? F("yes") : F("no"));
  out.print(F("  held for:      "));
  out.print(probe.held_ms);
  out.print(F(" ms of "));
  out.print(kWindowMs);
  out.println(F(" ms"));
  out.print(F("  rail peak:     "));
  out.print(conv::swRailVolts(probe.peak_counts), 3);
  out.println(F(" V"));
  out.print(F("  ~{HV_FAULT}:   "));
  out.println(probe.fault_n ? F("high, no latched fault")
                            : F("LOW, FAULT LATCHED"));
  out.print(F("  started cold:  "));
  out.print(probe.started_cold ? F("yes") : F("NO"));
  out.print(F(", +36V_SW bulk bled down in "));
  out.print(probe.discharge_ms);
  out.println(F(" ms"));
  out.println();

  if (!probe.started_cold) {
    out.println(F("  [!!] COULD NOT DISCHARGE +36V_SW. Every field above is"));
    out.println(F("  meaningless: with the bulk still charged there is no"));
    out.println(F("  inrush, the switch is asked nothing, and this probe"));
    out.println(F("  would report the fix as present on a board without it."));
    out.println(F("  Something is holding the rail up. Check 'v'."));
    return;
  }

  if (probe.vs_absent) {
    out.println(F("  [!!] NO 36 V INPUT. The rail never moved AND the switch"));
    out.println(F("  never reported: HV_SNS stayed at its offset floor instead"));
    out.println(F("  of pegging at ISNSFLT within 1 ms. Below V_S,UVLOR"));
    out.println(F("  (5.0-5.75 V) the TPS1HTC100-Q1 does not operate at all."));
    out.println(F("  Check " MJ_REF_HV_CONN
                   " is seated and the supply is on. Pin 1 is"));
    out.println(F("  GND, pin 2 is positive -- reversed leads are blocked by"));
    out.println(F("  " MJ_REF_HV_DIODE " and look exactly like this."));
  } else if (probe.fix_present) {
    out.println(F("  [PASS] LATCH fix is present on this board. A plain"));
    out.println(F("  HV_EN assertion works, so the firmware staircase is no"));
    out.println(F("  longer needed. Build with -DMJ_HAS_LATCH_GPIO=1"));
    out.println(F("  -DMJ_LATCH_GPIO=<n> if the revision routes LATCH to a"));
    out.println(F("  GPIO, so powerUp() can arm latch-off after the ramp."));
  } else {
    out.println(F("  [FAIL] this board still needs the workaround, as"));
    out.println(F("  expected on pcb v0.1-v0.3. powerUp() uses the staircase"));
    out.println(F("  automatically -- nothing to do."));
  }
  out.println();
}

void runSequence(Print &out) {
  out.println();
  out.println(
      F("================================================================"));
  out.println(F(" mouse-joystick-pcb  stage 2: TMC5130A communication"));
  out.println(
      F("================================================================"));
  out.println(F(" Raises HV_EN. The driver output stage stays disabled."));
  out.println();

  // 1. Known-dead start.
  out.println(F("[1] before: rail must be off"));
  PowerSequence::powerDown();
  delay(50);
  RailState rail = PowerSequence::readRail();
  printRail(out, rail);
  if (!rail.off) {
    out.println(F("    [FAIL] +3.3V_SW is not off with HV_EN low."));
    out.println(F("      Refusing to continue. Either 36 V is reaching the"));
    out.println(F("      switched domain around the high-side switch, or the"));
    out.println(
        F("      switch is passing with EN low. Check " MJ_REF_HV_EN_SERIES
          " 1k and"));
    out.println(F("      " MJ_REF_HV_EN_PULLDOWN " 10k at " MJ_REF_HV_SWITCH
                  "'s EN pin before going further."));
    return;
  }
  out.println(F("    [ ok ] switched domain dead, as it must be before HV_EN"));
  out.println();

  // 2. Rail up.
  out.println(F("[2] raising HV_EN"));
  PowerUpReport report;
  const PowerUpResult result = PowerSequence::powerUp(report);
  switch (result) {
  case PowerUpResult::kRailAlreadyLive:
    out.println(F("    [FAIL] rail was already live. Aborted."));
    return;
  case PowerUpResult::kRailNeverCameUp:
    out.print(F("    [FAIL] +3.3V_SW never reached "));
    out.print(PowerSequence::kRailUpVolts, 2);
    out.println(F(" V. HV_EN dropped again."));
    out.print(F("    peak seen while HV_EN was high: "));
    out.print(report.peak_rail_volts, 4);
    out.print(F(" V ("));
    out.print(report.peak_counts);
    out.print(F(" counts) over "));
    out.print(report.samples);
    out.println(F(" samples"));
    out.println(
        F("      A peak near zero means the rail never moved at all. A"));
    out.println(
        F("      peak well above zero but short of the threshold means it"));
    out.println(F("      came up partway, which is a different fault."));
    printRail(out, PowerSequence::readRail());
    out.println(
        F("      Is 36 V actually connected to " MJ_REF_HV_CONN
          "? If the supply"));
    out.println(
        F("      went into constant-current, stop and find the short."));
    out.println(
        F("      Otherwise work down the chain: " MJ_REF_HV_SWITCH
          " high-side switch,"));
    out.println(F("      the TPSM365R6 buck to +5V_SW, the TPS73733 to"));
    out.println(F("      +3.3V_SW."));
    return;
  case PowerUpResult::kFaultLatched:
    out.println(
        F("    [FAIL] the 36 V switch latched a fault during power-up."));
    out.println(
        F("      LATCH is tied high on this board, so faults latch off"));
    out.println(
        F("      rather than auto-retrying. HV_EN has been dropped, which"));
    out.println(F("      is what clears it. I_CL is about 1.2 A ("
                   MJ_REF_HV_ICL " 40k)."));
    return;
  case PowerUpResult::kOk:
    break;
  }
  out.print(F("    method: "));
  out.println(report.method == mouse_joystick::PowerUpMethod::kLatchGpio
                  ? F("LATCH held low, direct HV_EN assertion")
                  : F("firmware staircase (" MJ_REF_HV_LATCH_STRAP
                      " straps LATCH high)"));
  out.print(F("    [ ok ] +3.3V_SW came up in "));
  out.print(report.rise_time_ms);
  out.print(F(" ms, peak "));
  out.print(report.peak_rail_volts, 4);
  out.println(F(" V"));
  printRail(out, PowerSequence::readRail());
  out.println();

  // 3. Switch diagnostics.
  out.println(F("[3] 36 V switch diagnostics"));
  const HvDiagnostics diag = PowerSequence::readHvDiagnostics();
  printHvDiagnostics(out, diag);
  if (!diag.fault_n) {
    out.println(F("    [FAIL] the 36 V switch has latched a fault. Stopping."));
    out.println(F("      Powering down, which is what clears the latch."));
    out.println(F("      Run 'p' for a millisecond-resolution profile of the"));
    out.println(
        F("      rail and the current sense through a power-up attempt."));
    PowerSequence::powerDown();
    return;
  }
  const RailState after_diag = PowerSequence::readRail();
  if (!after_diag.up) {
    out.println(
        F("    [FAIL] the rail is no longer live. It came up and then"));
    out.println(F("      collapsed. Run 'p' for the profile."));
    printRail(out, after_diag);
    PowerSequence::powerDown();
    return;
  }
  out.println();

  // 4. Bus.
  out.println(F("[4] closing the GPIO isolation bus switch"));
  out.println(
      F("    Only now, with the driver's VCC_IO rail established. Before"));
  out.println(
      F("    this point every MCU pin on the switched side was isolated."));
  if (!PowerSequence::connectBus()) {
    out.println(F("    [FAIL] refused: the rail is not live."));
    PowerSequence::powerDown();
    return;
  }
  delay(10);
  out.println(F("    [ ok ] bus closed, ~{TMC_DRV_EN} still high"));
  out.println();

  // 5. Talk to the driver.
  tmc::Spi::begin();
  reportRegisters(out);

  out.println();
  out.println(
      F("----------------------------------------------------------------"));
  out.println(
      F(" The rail is still up and the bus is still closed, so 'r' and"));
  out.println(F(" 'i' work now. Send 'x' to power down when you are done."));
  out.println(
      F("----------------------------------------------------------------"));
  out.println();
  sequence_run = true;
}

void setup() {
  SafeState::apply();

  Serial.begin(kSerialBaudRate);
  const uint32_t deadline = millis() + kSerialWaitMs;
  while (!Serial && millis() < deadline) {
    delay(10);
  }
  delay(100);

  Serial.println();
  Serial.println(F("mouse-joystick-pcb stage 2, idle and powered down."));
  Serial.println(F("36 V must be connected to " MJ_REF_HV_CONN
                   ". Send 's' to begin."));
  Serial.println();
  printMenu(Serial);
}

void loop() {
  if (!Serial.available()) {
    delay(10);
    return;
  }

  const int command = Serial.read();
  switch (command) {
  case '?':
    printMenu(Serial);
    break;
  case 's':
    runSequence(Serial);
    break;
  case 'r':
    reportRegisters(Serial);
    break;
  case 'i': {
    if (!PowerSequence::busConnected()) {
      Serial.println(F("Bus is open. Run 's' first."));
      break;
    }
    uint32_t ioin = 0;
    readAndReport(Serial, "IOIN     ", tmc::kIoin, ioin);
    break;
  }
  case 'v':
    printRail(Serial, PowerSequence::readRail());
    break;
  case 'd':
    printHvDiagnostics(Serial, PowerSequence::readHvDiagnostics());
    break;
  case 'p':
    profilePowerUp(Serial);
    break;
  case 'c':
    staircase(Serial);
    break;
  case 'm':
    measureCurrent(Serial);
    break;
  case 'l':
    latchProbe(Serial);
    break;
  case 'b': {
    if (!PowerSequence::connectBus()) {
      Serial.println(F("Refused: +3.3V_SW is not live. Run 'c' or 's' first."));
      break;
    }
    delay(10);
    tmc::Spi::begin();
    sequence_run = true;
    Serial.println(F("Bus closed, SPI0 up, ~{TMC_DRV_EN} still high."));
    Serial.println(F("'r' and 'i' work now."));
    break;
  }
  case 'h':
    PowerSequence::holdEnable();
    Serial.println(F("HV_EN HELD high. Bus open, HV_DIAG_EN low."));
    Serial.println(F("Nothing will drop it but 'x'. Use 'v' to watch the rail."));
    break;
  case 'w':
    softStartRetry(Serial);
    break;
  case 'x':
    tmc::Spi::end();
    PowerSequence::powerDown();
    sequence_run = false;
    Serial.println(F("Powered down: bus open, HV_EN low, HV_DIAG_EN low."));
    break;
  case 'I':
    SelfTest::emitIdentityBeacon(Serial, 3);
    break;
  case 'B':
    PowerSequence::powerDown();
    Serial.println(F("Powered down. Rebooting into BOOTSEL."));
    Serial.flush();
    delay(100);
    rp2040.rebootToBootloader();
    break;
  case 'R':
    PowerSequence::powerDown();
    Serial.println(F("Powered down. Rebooting."));
    Serial.flush();
    delay(100);
    rp2040.reboot();
    break;
  case '\r':
  case '\n':
    break;
  default:
    Serial.print(F("unknown command: '"));
    Serial.print(static_cast<char>(command));
    Serial.println(F("'. Send ? for the menu."));
    break;
  }
}
