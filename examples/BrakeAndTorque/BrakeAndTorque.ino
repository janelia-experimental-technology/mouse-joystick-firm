// Stages A4 and A5 bring-up: the brake current sink and the torque bridge.
//
// REQUIRES 36 V and a pcb v1.0 board, built as env:mouse_joystick_rev2.
// Stages 1 (RP2040SelfTest), 2 (TMC5130Communication) and A3 (AngleOutput)
// must have passed on the unit first; this program re-proves none of them.
//
// v1.0 is the first revision with the brake sink -- a DAC80501Z at I2C 0x49
// setting a linear current sink from +36V_SW through the B5Z brake coil,
// 0-48.78 mA -- and the torque bridge -- 10 V excitation, an INA821 at
// G = 191.7, and an ADS8689 reading the conditioned signal on SPI1. This
// console puts commanded currents through the coil and reads what the board
// says came back, and reads the torque channel at rest, under a known step
// and at 1 kHz. The design values it compares against are in
// src/Conversions.hpp; the measurements go to the PCB bring-up log.
//
// ---------------------------------------------------------------------------
// THE MOTOR DRIVER IS NEVER TOUCHED.
// ---------------------------------------------------------------------------
// ~{TMC_DRV_EN} stays high from SafeState to power-down, SPI0 is never
// claimed, and nothing moves. A brake that engages holds a stationary
// joystick, which is all the mechanism experiences.
//
// ---------------------------------------------------------------------------
// ORDER OF OPERATIONS, as in every stage before it
// ---------------------------------------------------------------------------
//   1. Rail up through PowerSequence. The sink, both DACs, the bridge's
//      excitation and the ADC all live on the switched domain.
//   2. Bus switch closed. GP10-12, GP15-18 and GP16/17 reach nothing before.
//   3. I2C0 claimed; the brake DAC's GAIN written (0x0101) and READ BACK
//      before any data write. The DAC80501Z resets to code 0, so the coil is
//      at 0 mA from rail-up until this program says otherwise.
//   4. SPI1 claimed; the ADS8689's RANGE_SEL written (0-5.12 V) and read
//      back, and SDO proven with the device's fixed loopback patterns,
//      before any conversion is believed. The reset range is +/-12.288 V,
//      which would read a 0-5 V signal as plausible numbers 4.8x too small.
//
// ---------------------------------------------------------------------------
// WHAT THIS PROGRAM CAN SEE
// ---------------------------------------------------------------------------
// Unlike stage 5, v1.0 feeds both outputs back: BRAKE_ISENSE (GP27) is the
// sink's own current monitor, 20.625 V/A against the RP2040's 3.3 V-referenced
// 12-bit ADC (39 uA per count), and the ADS8689 sits on the torque node with
// its own 4.096 V reference (78.125 uV per code). So the brake numbers printed
// are MEASURED, within the RP2040 ADC's accuracy, and the torque numbers are
// the ADC's. A meter in series with the coil, and a known weight on the
// lever arm, are still what calibrates either chain; this console tells you
// whether the chains work and how they behave in time.
//
// The torque bridge's shunt-calibration switch (TQ_CAL_EN) is exercised but
// not expected to work on the as-built v1.0: the 2026-10-04 design review
// found its N-channel MOSFET's source at ~5 V against a 3.3 V gate. The `k`
// command measures the step, whatever it is, and says what it expected.
//
// Commands are LINES: a letter, optionally a space and a number, then Enter.
// `c 32768` sets the brake DAC code; `T 2000` takes 2000 torque samples.
// The host harness is tools/board_console.py.

#include <ADS8689.hpp>
#include <DAC80501.hpp>
#include <MouseJoystickBoard.hpp>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#if MJ_BOARD_REV < 2
// Keep on one line: a continuation inside an #error string warns.
#error "The brake sink and the torque bridge exist from pcb v1.0 (MJ_BOARD_REV 2) -- build env:mouse_joystick_rev2"
#endif

using mouse_joystick::PowerSequence;
using mouse_joystick::PowerUpReport;
using mouse_joystick::PowerUpResult;
using mouse_joystick::RailState;
using mouse_joystick::SafeState;
using mouse_joystick::SelfTest;

namespace pins = mouse_joystick::pins;
namespace conv = mouse_joystick::conversions;

constexpr uint32_t kSerialBaudRate = 115200;
constexpr uint32_t kSerialWaitMs = 5000;

// I2C at 400 kHz, as the QP build and the other rev 2 consoles since
// 2026-10-05. SPI1 at 2 MHz so a 10 MSa/s capture decodes it; the part takes
// 66 MHz and a 32-bit frame is still only 16 us.
constexpr uint32_t kI2cClockHz = 400000;
constexpr uint32_t kAdcSpiClockHz = 2000000;

// How long to let the coil current settle after a code write before reading
// it. The command filter is two ~300 Hz poles (about 2 ms to 90 %) and the
// coil's L/R against the sink's headroom adds of order 1 ms; the brake's own
// mechanical response is 6 ms. 100 ms is settled by any reading.
constexpr uint32_t kBrakeSettleMs = 100;
constexpr uint16_t kIsenseSamples = 64;
constexpr uint32_t kTorqueDefaultSamples = 1000;
constexpr uint32_t kTorqueSamplePeriodUs = 1000;
constexpr uint32_t kLiveLoopDefaultSeconds = 10;

dac80501::I2cTransport brake_transport(Wire, mouse_joystick::kBrakeDacI2cAddress);
dac80501::Dac80501 brake_dac(brake_transport);
ads8689::SpiTransport adc_transport(SPI1, pins::kTqConvstCsN, kAdcSpiClockHz);
ads8689::Ads8689 torque_adc(adc_transport);

bool rail_up = false;
bool bus_closed = false;
bool i2c_seen_0x48 = false;
bool i2c_seen_0x49 = false;
uint16_t brake_code = 0;
// A measured tare replaces the nominal zero once `z` has run.
uint16_t torque_zero_code = conv::kTorqueZeroCodeNominal;
bool torque_tared = false;
bool cal_switch_on = false;

// Types first: the Arduino builder hoists every function prototype above
// the first function, so anything a prototype names must already exist.

struct IsenseReading {
  float counts;
  float volts;
  float amps;
  bool fault_n; // true = ~{BRAKE_FAULT} high = not limiting
};

struct Stats {
  uint32_t n = 0;
  double mean = 0.0;
  double m2 = 0.0;
  uint16_t min = 0xFFFF;
  uint16_t max = 0;
  void add(uint16_t x) {
    ++n;
    const double d = static_cast<double>(x) - mean;
    mean += d / static_cast<double>(n);
    m2 += d * (static_cast<double>(x) - mean);
    if (x < min) {
      min = x;
    }
    if (x > max) {
      max = x;
    }
  }
  float sd() const {
    return n > 1 ? static_cast<float>(sqrt(m2 / static_cast<double>(n - 1)))
                 : 0.0f;
  }
};

// --------------------------------------------------------------------- menu

void printMenu(Print &out) {
  out.println(F("commands (a letter, an optional number, Enter)"));
  out.println(F("  ?         this menu"));
  out.println(F("  b         rail up, bus closed, brake DAC 0x49 configured and"));
  out.println(F("            read back, SPI1 + ADS8689 range set, read back, SDO"));
  out.println(F("            loopback proven. Both I2C addresses probed."));
  out.println(F("  i         status: rail, brake code / commanded / measured, torque"));
  out.println(F("brake (coil current, measured at BRAKE_ISENSE)"));
  out.println(F("  c <code>  brake DAC code 0..65535 (48.78 mA full scale)"));
  out.println(F("  a <uA>    brake current in microamps, 0..48780"));
  out.println(F("  0         brake off (code 0)"));
  out.println(F("  r         read BRAKE_ISENSE and ~BRAKE_FAULT now"));
  out.println(F("  s         sweep 0,10,...,100 %, each approached from zero;"));
  out.println(F("            current-vs-code table, back to 0 at the end"));
  out.println(F("  S <code>  step 0 -> code -> 0 with BRAKE_ISENSE at 5 kHz:"));
  out.println(F("            rise and fall times (default: full scale)"));
  out.println(F("torque (ADS8689 on AMP_VOUT, before the /2 to the BNC)"));
  out.println(F("  t         one conversion: code, volts, BNC volts, N.cm"));
  out.println(F("  T [n]     n samples at 1 kHz (default 1000): mean, sd, min, max"));
  out.println(F("  z         tare: the current mean becomes zero torque"));
  out.println(F("  k         shunt-cal step: TQ_CAL_EN low / high / low, 500 each"));
  out.println(F("  d         ADS8689 register dump"));
  out.println(F("  L [s]     live 1 kHz loop for s seconds (default 10): torque"));
  out.println(F("            sample + ISENSE read per tick, achieved rate, worst"));
  out.println(F("            iteration; FW_TRIG0 high while a tick works"));
  out.println(F("  X         coupling: torque stats with the brake at 0, 50 %,"));
  out.println(F("            100 %, 0 -- brake current into the torque reading"));
  out.println(F("  M         micro-benchmark: us per ADS8689 sample, per ISENSE"));
  out.println(F("            read, per fault-pin read, per brake DAC write"));
  out.println(F("  x         power down: brake 0, bus open, HV_EN low"));
  out.println(F("  I         identity beacon on FW_TRIG0/1"));
  out.println(F("  B / R     reboot into BOOTSEL / reboot"));
  out.println();
}

// ------------------------------------------------------------------ helpers

IsenseReading readIsense(uint16_t samples = kIsenseSamples) {
  analogReadResolution(conv::kAdcResolutionBits);
  uint32_t acc = 0;
  for (uint16_t i = 0; i < samples; ++i) {
    acc += analogRead(pins::kBrakeIsense);
  }
  IsenseReading r;
  r.counts = static_cast<float>(acc) / static_cast<float>(samples);
  r.volts = conv::countsToVolts(static_cast<uint16_t>(r.counts + 0.5f));
  r.amps = conv::brakeIsenseVoltsToAmps(r.volts);
  r.fault_n = digitalRead(pins::kBrakeFaultN) == HIGH;
  return r;
}

void printIsense(Print &out, const IsenseReading &r) {
  out.print(F("    BRAKE_ISENSE "));
  out.print(r.volts, 4);
  out.print(F(" V ("));
  out.print(r.counts, 1);
  out.print(F(" counts) = "));
  out.print(r.amps * 1000.0f, 2);
  out.print(F(" mA   ~BRAKE_FAULT "));
  out.println(r.fault_n ? F("high (not limiting)") : F("LOW (limiting)"));
}

void printCommanded(Print &out, uint16_t code) {
  out.print(F("    code "));
  out.print(code);
  out.print(F(" (0x"));
  out.print(code, HEX);
  out.print(F(") -> DAC "));
  out.print(dac80501::codeToVolts(code, conv::kBrakeDacGainRegister), 4);
  out.print(F(" V -> "));
  out.print(conv::brakeCodeToAmps(code) * 1000.0f, 3);
  out.print(F(" mA commanded, "));
  out.print(conv::brakeAmpsToTorqueNcm(conv::brakeCodeToAmps(code)), 3);
  out.println(F(" N.cm nominal"));
}

// n conversions on a fixed period, each a CONVST pulse plus one frame.
Stats torqueStats(uint32_t n, uint32_t period_us = kTorqueSamplePeriodUs) {
  Stats s;
  uint32_t next = micros();
  for (uint32_t i = 0; i < n; ++i) {
    while (static_cast<int32_t>(micros() - next) < 0) {
    }
    next += period_us;
    s.add(torque_adc.sample());
  }
  return s;
}

void printTorqueCode(Print &out, float code) {
  const float volts = code * conv::kTorqueAdcVoltsPerCode;
  out.print(code, 1);
  out.print(F(" codes = "));
  out.print(volts, 5);
  out.print(F(" V at AMP_VOUT, "));
  out.print(volts * conv::kTorqueBncDivider, 5);
  out.print(F(" V at the BNC, "));
  out.print(conv::torqueVoltsToNcm(volts, conv::torqueAdcCodeToVolts(torque_zero_code)), 4);
  out.print(F(" N.cm"));
  out.print(torque_tared ? F(" (tared)") : F(" (nominal zero)"));
}

void printTorqueStats(Print &out, const Stats &s) {
  out.print(F("    n "));
  out.print(s.n);
  out.print(F("  mean "));
  printTorqueCode(out, static_cast<float>(s.mean));
  out.println();
  out.print(F("    sd "));
  out.print(s.sd(), 2);
  out.print(F(" codes = "));
  out.print(s.sd() * conv::kTorqueAdcVoltsPerCode * 1e6f, 1);
  out.print(F(" uV rms = "));
  out.print(s.sd() * conv::kTorqueAdcVoltsPerCode / conv::kTorqueVoltsPerNcm * 1000.0f, 3);
  out.print(F(" mN.cm rms   min "));
  out.print(s.min);
  out.print(F("  max "));
  out.print(s.max);
  out.print(F("  span "));
  out.print(s.max - s.min);
  out.println(F(" codes"));
}

bool probeI2c(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool requireBrake(Print &out) {
  if (!brake_dac.configured()) {
    out.println(F("Run 'b' first: the brake DAC is not configured."));
    return false;
  }
  return true;
}

bool requireTorque(Print &out) {
  if (!torque_adc.configured()) {
    out.println(F("Run 'b' first: the torque ADC is not configured."));
    return false;
  }
  return true;
}

// -------------------------------------------------------------- bring-up

bool bringUp(Print &out) {
  out.println(F("[1] rail"));
  PowerUpReport report{};
  const PowerUpResult result = PowerSequence::powerUp(report);
  if (result != PowerUpResult::kOk) {
    out.print(F("    [FAIL] the rail did not come up ("));
    switch (result) {
    case PowerUpResult::kRailAlreadyLive:
      out.print(F("already live"));
      break;
    case PowerUpResult::kRailNeverCameUp:
      out.print(F("never came up: 36 V absent, or the switch tripped"));
      break;
    case PowerUpResult::kFaultLatched:
      out.print(F("fault latched"));
      break;
    default:
      break;
    }
    out.println(F("). Run stage 2 (examples/TMC5130Communication) to"));
    out.println(F("      diagnose it; this program does not duplicate that."));
    return false;
  }
  out.print(F("    [ ok ] +3.3V_SW up in "));
  out.print(report.rise_time_ms);
  out.print(F(" ms, peak "));
  out.print(report.peak_rail_volts, 4);
  out.print(F(" V, bulk charged / latch-off armed at "));
  out.print(report.charge_ms);
  out.println(F(" ms"));
  rail_up = true;

  out.println(F("[2] bus switch"));
  if (!PowerSequence::connectBus()) {
    out.println(F("    [FAIL] the bus switch did not close."));
    return false;
  }
  bus_closed = true;
  out.println(F("    [ ok ] closed. The DACs, the brake fault line and the"));
  out.println(F("           torque ADC are now reachable."));
  // The brake block's fault output has Ioff and is high-Z with the switched
  // domain down; a weak pull-up makes the undriven case read "not limiting"
  // rather than noise. With the rail up U1803 drives it either way.
  pinMode(pins::kBrakeFaultN, INPUT_PULLUP);

  out.println(F("[3] I2C0"));
  Wire.setSCL(pins::kDacScl);
  Wire.setSDA(pins::kDacSda);
  brake_transport.begin();
  Wire.setClock(kI2cClockHz);
  i2c_seen_0x48 = probeI2c(mouse_joystick::kAngleDacI2cAddress);
  i2c_seen_0x49 = probeI2c(mouse_joystick::kBrakeDacI2cAddress);
  out.print(F("    angle DAC 0x48: "));
  out.print(i2c_seen_0x48 ? F("ACK") : F("no ACK"));
  out.print(F("   brake DAC 0x49: "));
  out.println(i2c_seen_0x49 ? F("ACK") : F("no ACK"));
  if (!i2c_seen_0x49) {
    out.println(F("    [FAIL] the brake DAC does not answer. Check +3.3V_FILT"));
    out.println(F("      and the A0 strap (R1807 to +3.3V_FILT)."));
    return false;
  }

  out.println(F("[4] brake DAC configuration"));
  dac80501::Config config = dac80501::arduinoConfig();
  config.gain_register = conv::kBrakeDacGainRegister;
  config.reference_volts = dac80501::kInternalReferenceVolts;
  config.synchronous = false;
  config.soft_reset_first = true;
  if (!brake_dac.begin(config)) {
    out.println(F("    [FAIL] a configuration frame was not acknowledged."));
    return false;
  }
  brake_code = 0;
  const dac80501::VerifyResult verified = brake_dac.verifyConfiguration();
  out.print(F("    GAIN 0x"));
  out.print(conv::kBrakeDacGainRegister, HEX);
  out.print(F(" written: "));
  switch (verified) {
  case dac80501::VerifyResult::kVerified:
    out.println(F("[ ok ] read back and matched"));
    break;
  case dac80501::VerifyResult::kMismatch:
    out.println(F("[FAIL] read back and did NOT match"));
    return false;
  default:
    out.println(F("[FAIL] read did not complete"));
    return false;
  }
  uint16_t devid = 0;
  if (brake_dac.readDevid(devid)) {
    out.print(F("    DEVID 0x"));
    out.print(devid, HEX);
    out.print(dac80501::devidIsPlausible(devid) ? F(" plausible, resets to ")
                                                : F(" NOT plausible, resets to "));
    out.println(dac80501::devidResetsToMidscale(devid) ? F("MIDSCALE -- wrong part for a brake")
                                                       : F("zero scale (coil at 0 mA until written)"));
  }
  bool alarm = false;
  if (brake_dac.readReferenceAlarm(alarm)) {
    out.print(F("    REF-ALARM "));
    out.println(alarm ? F("ACTIVE") : F("clear"));
  }
  out.print(F("    full scale "));
  out.print(conv::kBrakeFullScaleAmps * 1000.0f, 3);
  out.print(F(" mA, one code "));
  out.print(conv::brakeCodeToAmps(1) * 1e6f, 3);
  out.println(F(" uA. Coil at code 0."));
  delay(20);
  printIsense(out, readIsense());

  out.println(F("[5] SPI1 and the ADS8689"));
  SPI1.setSCK(pins::kTqSpiSck);
  SPI1.setTX(pins::kTqSpiCopi);
  SPI1.setRX(pins::kTqSpiCipo);
  SPI1.begin();
  adc_transport.begin();
  out.print(F("    SCK GP"));
  out.print(pins::kTqSpiSck);
  out.print(F(", COPI GP"));
  out.print(pins::kTqSpiCopi);
  out.print(F(", CIPO GP"));
  out.print(pins::kTqSpiCipo);
  out.print(F(", CONVST/CS GP"));
  out.print(pins::kTqConvstCsN);
  out.print(F(", "));
  out.print(kAdcSpiClockHz / 1000000);
  out.println(F(" MHz, mode 0"));
  // The reference needs ~20 ms after power-up with 10 uF on REFCAP; the rail
  // has been up for the settle plus the DAC work, but say so explicitly.
  delay(ads8689::kReferenceTurnOnTypMs);

  ads8689::Config adc_config;
  adc_config.range = ads8689::kRangeUnipolar1p25;
  adc_config.loopback_check = true;
  ads8689::BeginReport adc_report;
  const bool adc_ok = torque_adc.begin(adc_config, &adc_report);
  out.print(F("    RANGE_SEL written 0x"));
  out.print(adc_report.range_written, HEX);
  out.print(F(" (0 to 1.25 x VREF = 0-5.12 V), read back 0x"));
  out.print(adc_report.range_readback, HEX);
  out.println(adc_report.result == ads8689::BeginResult::kRangeReadbackMismatch
                  ? F("  [FAIL] MISMATCH: nothing answered, or SDO is stuck")
                  : F("  [ ok ]"));
  if (adc_report.result != ads8689::BeginResult::kRangeReadbackMismatch) {
    out.print(F("    loopback all-ones 0x"));
    out.print(adc_report.loopback_ones, HEX);
    out.print(F(", all-zeros 0x"));
    out.print(adc_report.loopback_zeros, HEX);
    out.println((adc_report.loopback_ones == 0xFFFFu &&
                 adc_report.loopback_zeros == 0x0000u)
                    ? F("  [ ok ] SDO proven")
                    : F("  [FAIL] a bit did not come back"));
    out.print(F("    DATAOUT_CTL back to 0x"));
    out.print(adc_report.dataout_readback, HEX);
    out.print(F(", DEVICE_ID 0x"));
    out.print(adc_report.device_id_readback, HEX);
    out.print(F(", RST_PWRCTL 0x"));
    out.println(adc_report.rst_pwrctl_readback, HEX);
  }
  if (!adc_ok) {
    out.println(F("    [FAIL] the torque ADC is not configured; torque"));
    out.println(F("      commands are refused. The brake commands still work."));
    return false;
  }
  out.print(F("    one code "));
  out.print(torque_adc.voltsPerCode() * 1e6f, 3);
  out.print(F(" uV; nominal zero "));
  out.print(conv::kTorqueZeroVolts, 4);
  out.print(F(" V = code "));
  out.print(conv::kTorqueZeroCodeNominal);
  out.print(F("; "));
  out.print(conv::kTorqueVoltsPerNcm, 4);
  out.println(F(" V per N.cm nominal"));
  out.print(F("    first reading: "));
  printTorqueCode(out, static_cast<float>(torque_adc.sample()));
  out.println();
  out.println();
  return true;
}

// ---------------------------------------------------------------- brake

void setBrakeCode(Print &out, uint16_t code) {
  if (!brake_dac.setCode(code)) {
    out.println(F("  [FAIL] write refused or not acknowledged."));
    return;
  }
  brake_code = code;
  printCommanded(out, code);
  delay(kBrakeSettleMs);
  printIsense(out, readIsense());
}

void sweep(Print &out) {
  if (!requireBrake(out)) {
    return;
  }
  out.println(F("Brake sweep, every point approached from code 0 (the firmware"));
  out.println(F("contract: the brake's torque-vs-current has two curves, one per"));
  out.println(F("approach direction). 100 ms settle per point, 64 ISENSE samples."));
  out.println(F("  pct   code   cmd_mA   isense_V   meas_mA   err_%FS  fault"));
  const uint8_t steps[] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
  for (unsigned i = 0; i < sizeof(steps) / sizeof(steps[0]); ++i) {
    brake_dac.setCode(0);
    delay(kBrakeSettleMs);
    const uint32_t code32 = static_cast<uint32_t>(steps[i]) * 65535u / 100u;
    const uint16_t code = static_cast<uint16_t>(code32);
    brake_dac.setCode(code);
    brake_code = code;
    delay(kBrakeSettleMs);
    const IsenseReading r = readIsense();
    const float cmd_ma = conv::brakeCodeToAmps(code) * 1000.0f;
    const float meas_ma = r.amps * 1000.0f;
    out.print(F("  "));
    if (steps[i] < 100) {
      out.print(F(" "));
    }
    if (steps[i] < 10) {
      out.print(F(" "));
    }
    out.print(steps[i]);
    out.print(F("  "));
    out.print(code);
    out.print(F("  "));
    out.print(cmd_ma, 3);
    out.print(F("   "));
    out.print(r.volts, 4);
    out.print(F("    "));
    out.print(meas_ma, 3);
    out.print(F("   "));
    out.print((meas_ma - cmd_ma) / (conv::kBrakeFullScaleAmps * 1000.0f) * 100.0f, 2);
    out.print(F("    "));
    out.println(r.fault_n ? F("-") : F("LIMITING"));
  }
  brake_dac.setCode(0);
  brake_code = 0;
  delay(kBrakeSettleMs);
  out.print(F("  back at 0:"));
  printIsense(out, readIsense());
  out.println(F("Full scale should read about 1.006 V; short of it, stable and"));
  out.println(F("repeatable, means the gate drive ran out before the current did."));
  out.println();
}

void stepResponse(Print &out, uint16_t code) {
  if (!requireBrake(out)) {
    return;
  }
  constexpr uint32_t kPeriodUs = 200;
  constexpr uint16_t kSamples = 125; // 25 ms
  static uint16_t rise[kSamples];
  static uint16_t fall[kSamples];
  static bool rise_fault[kSamples]; // ~BRAKE_FAULT low (limiting) at the sample
  static bool fall_fault[kSamples];
  analogReadResolution(conv::kAdcResolutionBits);

  brake_dac.setCode(0);
  brake_code = 0;
  delay(kBrakeSettleMs);
  const IsenseReading before = readIsense();

  out.println(F("Step 0 -> code -> 0, BRAKE_ISENSE every 200 us for 25 ms each way."));
  out.println(F("FW_TRIG1 is high from the rising code write to the falling one."));
  printCommanded(out, code);

  SafeState::setTrigger1(true);
  brake_dac.setCode(code);
  brake_code = code;
  uint32_t next = micros();
  for (uint16_t i = 0; i < kSamples; ++i) {
    while (static_cast<int32_t>(micros() - next) < 0) {
    }
    next += kPeriodUs;
    rise[i] = analogRead(pins::kBrakeIsense);
    rise_fault[i] = digitalRead(pins::kBrakeFaultN) == LOW;
  }
  delay(kBrakeSettleMs);
  const IsenseReading held = readIsense();
  brake_dac.setCode(0);
  brake_code = 0;
  SafeState::setTrigger1(false);
  next = micros();
  for (uint16_t i = 0; i < kSamples; ++i) {
    while (static_cast<int32_t>(micros() - next) < 0) {
    }
    next += kPeriodUs;
    fall[i] = analogRead(pins::kBrakeIsense);
    fall_fault[i] = digitalRead(pins::kBrakeFaultN) == LOW;
  }
  delay(kBrakeSettleMs);
  const IsenseReading after = readIsense();

  auto report = [&](const char *name, const uint16_t *trace, const bool *fault,
                    float from_counts, float to_counts) {
    out.print(F("  "));
    out.print(name);
    out.print(F(": from "));
    out.print(conv::brakeIsenseCountsToAmps(static_cast<uint16_t>(from_counts + 0.5f)) * 1000.0f, 2);
    out.print(F(" mA to "));
    out.print(conv::brakeIsenseCountsToAmps(static_cast<uint16_t>(to_counts + 0.5f)) * 1000.0f, 2);
    out.println(F(" mA"));
    const float span = to_counts - from_counts;
    const float marks[] = {0.10f, 0.50f, 0.90f};
    for (unsigned m = 0; m < 3; ++m) {
      const float threshold = from_counts + span * marks[m];
      int32_t t_us = -1;
      for (uint16_t i = 0; i < kSamples; ++i) {
        const bool crossed = span >= 0 ? trace[i] >= threshold : trace[i] <= threshold;
        if (crossed) {
          t_us = static_cast<int32_t>(i) * kPeriodUs;
          break;
        }
      }
      out.print(F("    t"));
      out.print(static_cast<int>(marks[m] * 100.0f));
      out.print(F(" "));
      if (t_us < 0) {
        out.println(F("not reached within 25 ms"));
      } else {
        out.print(t_us);
        out.println(F(" us"));
      }
    }
    // The peak, because a full-scale step was seen to overshoot (unit A,
    // 2026-10-06: 62 mA for ~5 ms, above the 56.4 mA ceiling), and whether
    // the ceiling amplifier said so on ~{BRAKE_FAULT} while it happened.
    uint16_t peak = 0;
    uint16_t peak_i = 0;
    uint16_t fault_count = 0;
    int32_t fault_first_us = -1;
    int32_t fault_last_us = -1;
    for (uint16_t i = 0; i < kSamples; ++i) {
      if (trace[i] > peak) {
        peak = trace[i];
        peak_i = i;
      }
      if (fault[i]) {
        ++fault_count;
        if (fault_first_us < 0) {
          fault_first_us = static_cast<int32_t>(i) * kPeriodUs;
        }
        fault_last_us = static_cast<int32_t>(i) * kPeriodUs;
      }
    }
    out.print(F("    peak "));
    out.print(conv::brakeIsenseCountsToAmps(peak) * 1000.0f, 2);
    out.print(F(" mA at "));
    out.print(static_cast<uint32_t>(peak_i) * kPeriodUs);
    out.print(F(" us ("));
    out.print(conv::brakeIsenseCountsToAmps(peak) / conv::kBrakeFullScaleAmps * 100.0f, 1);
    out.print(F(" % of full scale; the ceiling is "));
    out.print(conv::kBrakeCeilingAmps * 1000.0f, 1);
    out.println(F(" mA)"));
    out.print(F("    ~BRAKE_FAULT low in "));
    out.print(fault_count);
    out.print(F(" of "));
    out.print(kSamples);
    out.print(F(" samples"));
    if (fault_count > 0) {
      out.print(F(", from "));
      out.print(fault_first_us);
      out.print(F(" to "));
      out.print(fault_last_us);
      out.print(F(" us"));
    }
    out.println();
    out.print(F("    mA at 1 ms steps:"));
    for (uint16_t i = 0; i < kSamples; i += 5) {
      out.print(F(" "));
      out.print(conv::brakeIsenseCountsToAmps(trace[i]) * 1000.0f, 1);
    }
    out.println();
  };
  report("rise", rise, rise_fault, before.counts, held.counts);
  report("fall", fall, fall_fault, held.counts, after.counts);
  out.println(F("  The command filter alone is ~2 ms to 90 %; the coil's L/R"));
  out.println(F("  against the sink's headroom adds the rest. The brake's"));
  out.println(F("  mechanical response is 6 ms (datasheet)."));
  out.println();
}

// --------------------------------------------------------------- torque

void torqueOnce(Print &out) {
  if (!requireTorque(out)) {
    return;
  }
  out.print(F("  "));
  printTorqueCode(out, static_cast<float>(torque_adc.sample()));
  out.println();
}

void torqueMany(Print &out, uint32_t n) {
  if (!requireTorque(out)) {
    return;
  }
  if (n == 0) {
    n = kTorqueDefaultSamples;
  }
  out.print(F("  "));
  out.print(n);
  out.println(F(" conversions at 1 kHz:"));
  printTorqueStats(out, torqueStats(n));
  out.println();
}

void tare(Print &out) {
  if (!requireTorque(out)) {
    return;
  }
  const Stats s = torqueStats(kTorqueDefaultSamples);
  torque_zero_code = static_cast<uint16_t>(s.mean + 0.5);
  torque_tared = true;
  out.print(F("  tared at code "));
  out.print(torque_zero_code);
  out.print(F(" = "));
  out.print(conv::torqueAdcCodeToVolts(torque_zero_code), 5);
  out.print(F(" V (nominal zero "));
  out.print(conv::kTorqueZeroCodeNominal);
  out.print(F(", "));
  out.print(static_cast<int32_t>(torque_zero_code) - conv::kTorqueZeroCodeNominal);
  out.print(F(" codes = "));
  out.print((static_cast<int32_t>(torque_zero_code) - conv::kTorqueZeroCodeNominal) *
                conv::kTorqueAdcVoltsPerCode * 1000.0f,
            2);
  out.println(F(" mV off; the sensor's zero balance is +/-1 % R.O. = +/-9.6 mV here)"));
  out.println();
}

void shuntCal(Print &out) {
  if (!requireTorque(out)) {
    return;
  }
  out.println(F("Shunt calibration: 500 samples with TQ_CAL_EN low, 500 high, 500 low."));
  out.print(F("Design step: R2003 75k across one 350 ohm arm = "));
  out.print(conv::kTorqueShuntCalVoltsPerVolt * 1000.0f, 3);
  out.print(F(" mV/V = "));
  out.print(conv::kTorqueShuntCalStepVolts, 3);
  out.print(F(" V at AMP_VOUT = "));
  out.print(conv::kTorqueShuntCalStepVolts / conv::kTorqueAdcVoltsPerCode, 0);
  out.println(F(" codes. The 2026-10-04 review expects the switch NOT to turn on."));
  const Stats low1 = torqueStats(500);
  digitalWrite(pins::kTqCalEn, HIGH);
  cal_switch_on = true;
  delay(50);
  const Stats high = torqueStats(500);
  digitalWrite(pins::kTqCalEn, LOW);
  cal_switch_on = false;
  delay(50);
  const Stats low2 = torqueStats(500);
  out.println(F("  TQ_CAL_EN low: "));
  printTorqueStats(out, low1);
  out.println(F("  TQ_CAL_EN high:"));
  printTorqueStats(out, high);
  out.println(F("  TQ_CAL_EN low again:"));
  printTorqueStats(out, low2);
  const float step_codes = static_cast<float>(high.mean - (low1.mean + low2.mean) / 2.0);
  out.print(F("  step "));
  out.print(step_codes, 1);
  out.print(F(" codes = "));
  out.print(step_codes * conv::kTorqueAdcVoltsPerCode * 1000.0f, 2);
  out.print(F(" mV = "));
  out.print(step_codes * conv::kTorqueAdcVoltsPerCode / conv::kTorqueShuntCalStepVolts * 100.0f, 1);
  out.println(F(" % of the design step"));
  out.println();
}

void registerDump(Print &out) {
  if (!requireTorque(out)) {
    return;
  }
  struct Entry {
    const char *name;
    uint8_t address;
  };
  const Entry regs[] = {
      {"DEVICE_ID   ", ads8689::kRegDeviceId},   {"RST_PWRCTL  ", ads8689::kRegRstPwrctl},
      {"SDI_CTL     ", ads8689::kRegSdiCtl},     {"SDO_CTL     ", ads8689::kRegSdoCtl},
      {"DATAOUT_CTL ", ads8689::kRegDataoutCtl}, {"RANGE_SEL   ", ads8689::kRegRangeSel},
      {"ALARM       ", ads8689::kRegAlarm},      {"ALARM_H_TH  ", ads8689::kRegAlarmHTh},
      {"ALARM_L_TH  ", ads8689::kRegAlarmLTh},
  };
  for (unsigned i = 0; i < sizeof(regs) / sizeof(regs[0]); ++i) {
    out.print(F("  "));
    out.print(regs[i].name);
    out.print(F(" 0x"));
    if (regs[i].address < 0x10) {
      out.print(F("0"));
    }
    out.print(regs[i].address, HEX);
    out.print(F(": 0x"));
    const uint16_t v = torque_adc.readRegister(regs[i].address);
    if (v < 0x1000) {
      out.print(F("0"));
    }
    if (v < 0x100) {
      out.print(F("0"));
    }
    if (v < 0x10) {
      out.print(F("0"));
    }
    out.println(v, HEX);
  }
  out.println();
}

void liveLoop(Print &out, uint32_t seconds) {
  if (!requireTorque(out) || !requireBrake(out)) {
    return;
  }
  if (seconds == 0) {
    seconds = kLiveLoopDefaultSeconds;
  }
  out.print(F("Live loop: "));
  out.print(seconds);
  out.println(F(" s at 1 kHz, each tick one ADS8689 conversion and one"));
  out.println(F("BRAKE_ISENSE read. FW_TRIG0 high while a tick works."));
  analogReadResolution(conv::kAdcResolutionBits);
  Stats torque;
  Stats isense;
  uint32_t worst_us = 0;
  uint32_t late = 0;
  const uint32_t ticks = seconds * 1000u;
  const uint32_t t_start = micros();
  uint32_t next = t_start;
  for (uint32_t i = 0; i < ticks; ++i) {
    while (static_cast<int32_t>(micros() - next) < 0) {
    }
    next += kTorqueSamplePeriodUs;
    const uint32_t t0 = micros();
    SafeState::setTrigger0(true);
    torque.add(torque_adc.sample());
    isense.add(analogRead(pins::kBrakeIsense));
    SafeState::setTrigger0(false);
    const uint32_t cost = micros() - t0;
    if (cost > worst_us) {
      worst_us = cost;
    }
    if (static_cast<int32_t>(micros() - next) > 0) {
      ++late;
    }
  }
  const uint32_t elapsed_us = micros() - t_start;
  out.print(F("  "));
  out.print(ticks);
  out.print(F(" ticks in "));
  out.print(elapsed_us / 1000);
  out.print(F(" ms = "));
  out.print(static_cast<float>(ticks) * 1e6f / static_cast<float>(elapsed_us), 1);
  out.print(F(" Hz achieved; worst iteration "));
  out.print(worst_us);
  out.print(F(" us; "));
  out.print(late);
  out.println(F(" ticks overran"));
  out.println(F("  torque:"));
  printTorqueStats(out, torque);
  out.print(F("  brake ISENSE mean "));
  out.print(conv::brakeIsenseCountsToAmps(static_cast<uint16_t>(isense.mean + 0.5)) * 1000.0f, 2);
  out.print(F(" mA (code "));
  out.print(brake_code);
  out.print(F(" commanded "));
  out.print(conv::brakeCodeToAmps(brake_code) * 1000.0f, 2);
  out.print(F(" mA), sd "));
  out.print(isense.sd(), 2);
  out.println(F(" counts"));
  out.println();
}

void coupling(Print &out) {
  if (!requireTorque(out) || !requireBrake(out)) {
    return;
  }
  out.println(F("Brake-to-torque coupling: 500 torque samples at each brake level,"));
  out.println(F("joystick untouched. A mean that moves with the brake current is"));
  out.println(F("electrical pickup (shared return, supply sag) or the brake"));
  out.println(F("preloading the sensor through the coupling; sd is the noise."));
  const uint16_t codes[] = {0, 0x8000, 0xFFFF, 0};
  for (unsigned i = 0; i < 4; ++i) {
    brake_dac.setCode(codes[i]);
    brake_code = codes[i];
    delay(kBrakeSettleMs);
    const IsenseReading r = readIsense();
    out.print(F("  brake code "));
    out.print(codes[i]);
    out.print(F(" = "));
    out.print(r.amps * 1000.0f, 2);
    out.println(F(" mA measured:"));
    printTorqueStats(out, torqueStats(500));
  }
  out.println();
}

// The cost of each primitive the protocol build pays per 1 kHz tick. The
// first QP run with the torque sample in the tick (2026-10-06) cost 1.2 ms a
// tick where ~40 us was expected; this is how to see which primitive it is.
void microBenchmark(Print &out) {
  if (!requireTorque(out) || !requireBrake(out)) {
    return;
  }
  constexpr uint16_t kN = 1000;
  uint32_t t0 = micros();
  for (uint16_t i = 0; i < kN; ++i) {
    static_cast<void>(torque_adc.sample());
  }
  const uint32_t t_sample = micros() - t0;
  t0 = micros();
  for (uint16_t i = 0; i < kN; ++i) {
    static_cast<void>(torque_adc.readLast());
  }
  const uint32_t t_frame = micros() - t0;
  analogReadResolution(conv::kAdcResolutionBits);
  t0 = micros();
  for (uint16_t i = 0; i < kN; ++i) {
    static_cast<void>(analogRead(pins::kBrakeIsense));
  }
  const uint32_t t_adc = micros() - t0;
  t0 = micros();
  for (uint16_t i = 0; i < kN; ++i) {
    static_cast<void>(digitalRead(pins::kBrakeFaultN));
  }
  const uint32_t t_pin = micros() - t0;
  t0 = micros();
  for (uint16_t i = 0; i < 100; ++i) {
    static_cast<void>(brake_dac.setCode(0));
  }
  const uint32_t t_dac = micros() - t0;
  out.print(F("  ADS8689 sample (pulse + wait + frame): "));
  out.print(static_cast<float>(t_sample) / kN, 1);
  out.println(F(" us"));
  out.print(F("  ADS8689 frame alone (readLast):        "));
  out.print(static_cast<float>(t_frame) / kN, 1);
  out.println(F(" us"));
  out.print(F("  analogRead BRAKE_ISENSE:               "));
  out.print(static_cast<float>(t_adc) / kN, 1);
  out.println(F(" us"));
  out.print(F("  digitalRead ~BRAKE_FAULT:              "));
  out.print(static_cast<float>(t_pin) / kN, 1);
  out.println(F(" us"));
  out.print(F("  brake DAC setCode over I2C at 400 kHz: "));
  out.print(static_cast<float>(t_dac) / 100.0f, 1);
  out.println(F(" us"));
  out.println();
}

// ---------------------------------------------------------------- status

void status(Print &out) {
  const RailState rail = PowerSequence::readRail();
  out.print(F("  rail: "));
  out.print(rail.rail_volts, 3);
  out.print(F(" V ("));
  out.print(rail.up ? F("up") : (rail.off ? F("off") : F("between")));
  out.print(F("), bus "));
  out.println(bus_closed ? F("closed") : F("open"));
  out.print(F("  I2C: 0x48 "));
  out.print(i2c_seen_0x48 ? F("ACK") : F("-"));
  out.print(F(", 0x49 "));
  out.println(i2c_seen_0x49 ? F("ACK") : F("-"));
  out.print(F("  brake DAC: "));
  out.println(brake_dac.configured() ? F("configured") : F("not configured"));
  if (brake_dac.configured()) {
    printCommanded(out, brake_code);
    printIsense(out, readIsense());
  }
  out.print(F("  torque ADC: "));
  out.println(torque_adc.configured() ? F("configured") : F("not configured"));
  if (torque_adc.configured()) {
    out.print(F("    "));
    printTorqueCode(out, static_cast<float>(torque_adc.sample()));
    out.println();
    out.print(F("    TQ_CAL_EN "));
    out.println(cal_switch_on ? F("HIGH") : F("low"));
  }
  out.println();
}

void powerDown(Print &out) {
  // Coil to zero first, so a later rail-up does not present whatever was
  // last written; then the bus, then the rail.
  if (brake_dac.configured()) {
    brake_dac.setCode(0);
  }
  brake_code = 0;
  digitalWrite(pins::kTqCalEn, LOW);
  cal_switch_on = false;
  PowerSequence::powerDown();
  rail_up = false;
  bus_closed = false;
  // Both parts lost their supply: the DAC's GAIN is back to a 5.0 V span
  // and the ADC's RANGE_SEL back to +/-12.288 V. Neither may be written
  // into while dead (their input clamps would back-power the rail), so this
  // forgets without a frame.
  brake_dac.invalidate();
  torque_adc.invalidate();
  pinMode(pins::kBrakeFaultN, INPUT);
  out.println(F("Powered down: brake at code 0, bus open, HV_EN low."));
  out.println(F("+36V_SW takes about 700 ms to bleed down. The DAC and the ADC"));
  out.println(F("lost their supplies; 'b' configures both again."));
  out.println();
}

// ------------------------------------------------------------- line input

char line_buf[48];
size_t line_len = 0;

// Returns true when a complete line is in line_buf.
bool readLine() {
  while (Serial.available()) {
    const int c = Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      line_buf[line_len] = '\0';
      line_len = 0;
      return true;
    }
    if (line_len < sizeof(line_buf) - 1) {
      line_buf[line_len++] = static_cast<char>(c);
    }
  }
  return false;
}

void dispatch(const char *line) {
  while (*line == ' ') {
    ++line;
  }
  if (*line == '\0') {
    return;
  }
  const char command = *line++;
  while (*line == ' ') {
    ++line;
  }
  const bool has_number = (*line >= '0' && *line <= '9');
  const uint32_t number = has_number ? strtoul(line, nullptr, 10) : 0;

  switch (command) {
  case '?':
    printMenu(Serial);
    break;
  case 'b':
    bringUp(Serial);
    break;
  case 'i':
    status(Serial);
    break;
  case 'c':
    if (requireBrake(Serial)) {
      if (!has_number) {
        Serial.println(F("  usage: c <code 0..65535>"));
      } else if (number > dac80501::kCodeMax) {
        Serial.println(F("  above 65535, refused."));
      } else {
        setBrakeCode(Serial, static_cast<uint16_t>(number));
      }
    }
    break;
  case 'a':
    if (requireBrake(Serial)) {
      if (!has_number) {
        Serial.println(F("  usage: a <microamps 0..48780>"));
      } else if (number > 48780u) {
        Serial.println(F("  above the 48.78 mA full scale, refused."));
      } else {
        setBrakeCode(Serial, conv::brakeAmpsToCode(static_cast<float>(number) * 1e-6f));
      }
    }
    break;
  case '0':
    if (requireBrake(Serial)) {
      setBrakeCode(Serial, 0);
    }
    break;
  case 'r':
    if (bus_closed) {
      printIsense(Serial, readIsense());
    } else {
      Serial.println(F("  bus open: the brake block is not powered."));
    }
    break;
  case 's':
    sweep(Serial);
    break;
  case 'S':
    stepResponse(Serial, has_number ? static_cast<uint16_t>(number > dac80501::kCodeMax
                                                                ? dac80501::kCodeMax
                                                                : number)
                                    : dac80501::kCodeMax);
    break;
  case 't':
    torqueOnce(Serial);
    break;
  case 'T':
    torqueMany(Serial, number);
    break;
  case 'z':
    tare(Serial);
    break;
  case 'k':
    shuntCal(Serial);
    break;
  case 'd':
    registerDump(Serial);
    break;
  case 'L':
    liveLoop(Serial, number);
    break;
  case 'X':
    coupling(Serial);
    break;
  case 'M':
    microBenchmark(Serial);
    break;
  case 'x':
    powerDown(Serial);
    break;
  case 'I':
    SelfTest::emitIdentityBeacon(Serial, 3);
    break;
  case 'B':
    Serial.println(F("Rebooting into BOOTSEL..."));
    powerDown(Serial);
    Serial.flush();
    delay(100);
    rp2040.rebootToBootloader();
    break;
  case 'R':
    Serial.println(F("Rebooting..."));
    powerDown(Serial);
    Serial.flush();
    delay(100);
    rp2040.reboot();
    break;
  default:
    Serial.print(F("unknown command: '"));
    Serial.print(command);
    Serial.println(F("'. Send ? for the menu."));
    break;
  }
}

void setup() {
  SafeState::apply();
  // Proof of life on FW_TRIG0/FW_TRIG1 before USB, as every stage program:
  // three 10 ms pulses on TRIG1 then eight 1 ms pulses on TRIG0. On a capture
  // this also marks the boot instant, so a reboot's effect on the other taps
  // (the bus switch enable, CH27) can be placed in time.
  SelfTest::emitBootSignature();
  Serial.begin(kSerialBaudRate);
  const uint32_t deadline = millis() + kSerialWaitMs;
  while (!Serial && millis() < deadline) {
    delay(10);
  }
  Serial.println();
  Serial.println(F("======================================================"));
  Serial.println(F(" mouse-joystick-pcb  stages A4/A5: brake sink, torque"));
  Serial.println(F("======================================================"));
  Serial.println(F(" Requires 36 V and a v1.0 board. The motor driver is"));
  Serial.println(F(" never enabled. Send 'b' to begin."));
  Serial.println();
  printMenu(Serial);
}

void loop() {
  if (readLine()) {
    dispatch(line_buf);
  }
}
