// Stage 2 bring-up: prove this board can talk to its TMC5130A.
//
// REQUIRES 36 V on J201. Stage 1 (examples/RP2040SelfTest) must have passed
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

namespace tmc = mouse_joystick::tmc;

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
  out.println(F("  w  software soft-start: retry HV_EN, 10 attempts"));
  out.println(F("  x  power down: open the bus, drop HV_EN"));
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

void printHvDiagnostics(Print &out, const HvDiagnostics &diag) {
  out.print(F("    HV_SNS: "));
  out.print(diag.sns_volts, 4);
  out.print(F(" V across R207 249R ("));
  out.print(diag.sns_counts);
  out.println(F(" counts)"));
  out.println(
      F("      load current needs the TPS1HTC100-Q1 sense ratio, which"));
  out.println(F("      this firmware does not carry yet. Volts only for now."));
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
    out.println(F("      R1405, the GND tie on REFR, R805/R806 and R1201."));

    ++checks;
    if (tmc::ioinMatchesExpected(ioin)) {
      out.println(F("    [ ok ] IOIN matches 0x11000010, bit 7 free"));
    } else {
      ++failures;
      out.println(F("    [FAIL] IOIN does not match the expected 0x11000010"));
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
  out.println(F("  current into R207 249R, not a load-current reading."));
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
        F("      switch is passing with EN low. Check R201 1k and R204"));
    out.println(F("      10k at U201's EN pin before going further."));
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
        F("      Is 36 V actually connected to J201? If the bench supply"));
    out.println(
        F("      went into constant-current, stop and find the short."));
    out.println(
        F("      Otherwise work down the chain: U201 high-side switch,"));
    out.println(F("      U901 TPSM365R6 buck to +5V_SW, U1001 TPS73733 to"));
    out.println(F("      +3.3V_SW."));
    return;
  case PowerUpResult::kFaultLatched:
    out.println(
        F("    [FAIL] the 36 V switch latched a fault during power-up."));
    out.println(
        F("      LATCH is tied high on this board, so faults latch off"));
    out.println(
        F("      rather than auto-retrying. HV_EN has been dropped, which"));
    out.println(F("      is what clears it. I_CL is about 1.2 A (R206 40k)."));
    return;
  case PowerUpResult::kOk:
    break;
  }
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
  Serial.println(F("36 V must be connected to J201. Send 's' to begin."));
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
  case 'w':
    softStartRetry(Serial);
    break;
  case 'x':
    tmc::Spi::end();
    PowerSequence::powerDown();
    sequence_run = false;
    Serial.println(F("Powered down: bus open, HV_EN low, HV_DIAG_EN low."));
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
