// Stage 5 bring-up: the analog angle output.
//
// REQUIRES 36 V, and a pcb revision that fits the DAC -- v0.3 or later, built
// as env:mouse_joystick_rev1. Stages 1 and 2 must have passed first.
//
// This is the program the v0.3 board was ordered for. It does not compute an
// angle and does not touch the motor: it puts commanded voltages on the output
// so they can be measured, which is what the gain and offset fit, the output
// floor and the filter group delay all need.
//
// ---------------------------------------------------------------------------
// YOU NEED A METER. THIS PROGRAM CANNOT SEE ITS OWN OUTPUT.
// ---------------------------------------------------------------------------
// `ANGLE_OUT` reaches no RP2040 ADC on v0.3 -- the three ADC pins are the rail
// sense, `HV_SNS` and a spare pad -- and over SPI the DAC cannot even be read
// back. So every number this program prints is what it ASKED FOR, never what
// the board produced. Measure the BNC with a DMM on DC volts, centre to shell.
//
// A scope is the wrong instrument for the DC points: the expected floor is tens
// of millivolts, and a probe-attenuation or coupling mismatch moves a reading by
// exactly the factor under test. That already happened once on this bench, on
// 2026-09-09, when a channel's own baseline was read as a 150 mV output floor
// and was still 150 mV with the cable unplugged. Use the scope for the group
// delay and the chopper alias hunt, where its time axis is the point.
//
// ---------------------------------------------------------------------------
// ORDER OF OPERATIONS, AND WHY IT IS NOT NEGOTIABLE
// ---------------------------------------------------------------------------
//   1. Rail up through PowerSequence. The DAC and its OPA197 run from +5V_SW.
//   2. Bus switch closed. GP10/GP11/GP13 are on the MCU side of the
//      SN74CB3Q3245 and reach the DAC only once it is closed.
//   3. SPI pins configured, and only then a chip select driven high.
//      Driving ~{DAC_CS} high while the DAC's VDD is dead pushes roughly
//      260 uA through the DAC's input clamp into +5V_SW and back-powers the
//      switched domain. Rail first, always.
//   4. GAIN written before any DAC data. GAIN resets to a 5.0 V full scale and
//      this board wants 2.5 V, so a data write before a GAIN write asks for
//      double. Dac80501::begin() enforces this; the driver refuses data writes
//      until it has run.

#include <DAC80501.hpp>
#include <MouseJoystickBoard.hpp>

using mouse_joystick::PowerSequence;
using mouse_joystick::PowerUpReport;
using mouse_joystick::PowerUpResult;
using mouse_joystick::RailState;
using mouse_joystick::SafeState;

namespace pins = mouse_joystick::pins;

constexpr uint32_t kSerialBaudRate = 115200;
constexpr uint32_t kSerialWaitMs = 5000;

#if MJ_BOARD_REV == 0
// A backslash continuation inside a #error string makes the preprocessor warn
// about an unterminated literal, so keep this on one line.
#error "No angle-output DAC on pcb v0.1/v0.2 -- build env:mouse_joystick_rev1"
#endif

// The board's analog chain, kept here rather than in the driver because none of
// it is a property of the converter.
//
// GAIN 0x0101 is REF-DIV 1 with BUFF-GAIN 1: the 2.5 V internal reference is
// halved to 1.25 V and the output buffer doubles it, giving a 2.5 V full scale.
// 0x0000 reaches the same 2.5 V span with the core at the full reference and the
// buffer at unity, which contributes less offset and noise -- worth trying
// against 0x0101 once there is a meter on the output, since the two are
// identical in span and differ only in noise. The pcb plan specifies 0x0101.
constexpr uint16_t kGainRegister =
    dac80501::kGainRefDiv | dac80501::kGainBuffGain;

// The OPA197 stage after the filter. 2.5 V DAC full scale becomes 3.3 V at the
// BNC on v0.3. v1.0 halves everything after this with a divider.
constexpr float kOutputStageGain = 1.32f;

// SPI1 on this board. RX is not used -- the DAC has no SDO -- but the core wants
// a pin assigned, and GP12 is SPI1's RX candidate that is routed nowhere on
// v0.3, so it is the safe choice. At v1.0 GP12 becomes the torque bridge ADC's
// SDO on this same bus.
constexpr uint8_t kSpi1Rx = 12;

// Transport selection, same shape as examples/MotionAndHoming -- deliberately
// duplicated rather than shared, because each example is meant to read on its
// own. Build with -DMJ_DAC_I2C=1, or use env:mouse_joystick_rev1_i2c.
//
// NO BOARD CAN RUN THE I2C PATH YET: pcb v0.3 ties `SPI2C` (U2001 pad 5)
// directly to GND in copper.
#ifndef MJ_DAC_I2C
#define MJ_DAC_I2C 0
#endif

#if MJ_DAC_I2C
// A0 gives four addresses by what it is tied to -- AGND 0x48, VDD 0x49,
// SDA 0x4A, SCL 0x4B -- so v1.0's two DACs share one bus. Angle output is 0x48.
constexpr uint8_t kDacI2cAddress = dac80501::kI2cAddrA0Agnd;
dac80501::I2cTransport transport(Wire, kDacI2cAddress);
#else
dac80501::SpiTransport transport(SPI1, pins::kDacCsN);
#endif
dac80501::Dac80501 dac(transport);

bool rail_up = false;
bool bus_closed = false;

void printMenu(Print &out) {
  out.println(F("commands"));
  out.println(F("  ?  this menu"));
  out.println(F("  b  rail up, bus closed, DAC configured"));
  out.println(F("  z  output zero scale (code 0)"));
  out.println(F("  m  output midscale"));
  out.println(F("  f  output full scale (code 0xFFFF)"));
  out.println(F("  v  set an output voltage, digits then Enter (mV)"));
  out.println(F("  c  set a raw code, digits then Enter"));
  out.println(F("  s  the three-point fit: 0, midscale, full scale, with a"));
  out.println(F("     pause at each for a meter reading"));
  out.println(F("  t  step 0 -> full scale, for the filter group delay"));
  out.println(F("  q  what the driver knows: gain, full scale, LSB"));
  out.println(F("  d  DEVID and STATUS -- I2C builds only, see below"));
  out.println(F("  r  soft reset the DAC (blocks data writes until 'b')"));
  out.println(F("  x  power down: bus open, HV_EN low"));
  out.println(F("  B  reboot into BOOTSEL      R  reboot"));
  out.println();
  out.println(F("Every value printed is COMMANDED, never measured. This"));
  out.println(F("board cannot read its own analog output, and over SPI it"));
  out.println(F("cannot read the DAC's registers either. Use a DMM."));
  out.println();
}

void printExpected(Print &out, uint16_t code) {
  const float dac_volts = dac80501::codeToVolts(code, kGainRegister);
  out.print(F("    code "));
  out.print(code);
  out.print(F(" (0x"));
  out.print(code, HEX);
  out.print(F(")  -> DAC "));
  out.print(dac_volts, 6);
  out.print(F(" V  -> BNC "));
  out.print(dac_volts * kOutputStageGain, 6);
  out.println(F(" V expected"));
}

bool bringUp(Print &out) {
  out.println(F("[1] rail"));
  PowerUpReport report{};
  const PowerUpResult result = PowerSequence::powerUp(report);
  if (result != PowerUpResult::kOk) {
    out.println(F("    [FAIL] the rail did not come up. Run stage 2"));
    out.println(F("      (examples/TMC5130Communication) to diagnose it;"));
    out.println(F("      this program does not duplicate that."));
    return false;
  }
  out.print(F("    [ ok ] +3.3V_SW up in "));
  out.print(report.rise_time_ms);
  out.println(F(" ms"));
  rail_up = true;

  out.println(F("[2] bus switch"));
  if (!PowerSequence::connectBus()) {
    out.println(F("    [FAIL] the bus switch did not close."));
    return false;
  }
  bus_closed = true;
  out.println(F("    [ ok ] closed. GP10/GP11/GP13 now reach the DAC."));

  // Only now. See the back-power note in the header.
#if MJ_DAC_I2C
  out.println(F("[3] I2C"));
  // The strap changes what the pins are: SCLK becomes SCL, SDIN becomes SDA,
  // and SYNC stops being a chip select and becomes the static A0 strap.
  Wire.setSCL(pins::kDacSpiSck);
  Wire.setSDA(pins::kDacSpiCopi);
  transport.begin();
  out.print(F("    [ ok ] SCL GP"));
  out.print(pins::kDacSpiSck);
  out.print(F(", SDA GP"));
  out.print(pins::kDacSpiCopi);
  out.print(F(", address 0x"));
  out.println(kDacI2cAddress, HEX);
#else
  out.println(F("[3] SPI1"));
  SPI1.setSCK(pins::kDacSpiSck);
  SPI1.setTX(pins::kDacSpiCopi);
  SPI1.setRX(kSpi1Rx);
  SPI1.begin();
  transport.begin();
  out.print(F("    [ ok ] SCK GP"));
  out.print(pins::kDacSpiSck);
  out.print(F(", TX GP"));
  out.print(pins::kDacSpiCopi);
  out.print(F(", SYNC GP"));
  out.println(pins::kDacCsN);
#endif

  out.println(F("[4] DAC configuration"));
  // arduinoConfig() rather than a bare Config: it fills in the injected delay
  // that the Arduino-free device class cannot supply for itself.
  dac80501::Config config = dac80501::arduinoConfig();
  config.gain_register = kGainRegister;
  config.reference_volts = dac80501::kInternalReferenceVolts;
  config.synchronous = false;
  config.soft_reset_first = true;
  if (!dac.begin(config)) {
    out.println(F("    [FAIL] a configuration frame failed. Over SPI that"));
    out.println(F("      means the transport itself reported an error, which"));
    out.println(F("      it almost never can -- suspect the bus switch."));
    return false;
  }
  out.print(F("    [ ok ] GAIN 0x"));
  out.print(kGainRegister, HEX);
  out.print(F(", full scale "));
  out.print(dac.fullScale(), 4);
  out.print(F(" V at the DAC, "));
  out.print(dac.fullScale() * kOutputStageGain, 4);
  out.println(F(" V at the BNC"));

  const dac80501::VerifyResult verified = dac.verifyConfiguration();
  switch (verified) {
  case dac80501::VerifyResult::kVerified:
    out.println(F("    [ ok ] GAIN read back and matched"));
    break;
  case dac80501::VerifyResult::kMismatch:
    out.println(F("    [FAIL] GAIN read back and did NOT match"));
    break;
  case dac80501::VerifyResult::kReadFailed:
    out.println(F("    [FAIL] GAIN read did not complete"));
    break;
  case dac80501::VerifyResult::kNotSupported:
    out.println(F("    [note] NOT VERIFIED, and cannot be: this transport"));
    out.println(F("      is SPI and the DAC80501 has no SDO pin. The only"));
    out.println(F("      check on GAIN is measuring the output against the"));
    out.println(F("      expected full scale above. Strapping SPI2C for I2C"));
    out.println(F("      would make this a real read -- see the pcb plan."));
    break;
  }
  out.println();
  return true;
}

bool requireReady(Print &out) {
  if (!dac.configured()) {
    out.println(F("Run 'b' first."));
    return false;
  }
  return true;
}

// Read decimal digits until Enter, with a timeout. Same shape as stage 3's 'n'.
bool readNumber(Print &out, uint32_t &value) {
  out.println(F("digits then Enter:"));
  value = 0;
  bool any = false;
  const uint32_t deadline = millis() + 15000;
  while (millis() < deadline) {
    if (!Serial.available()) {
      delay(5);
      continue;
    }
    const int c = Serial.read();
    if (c == '\r' || c == '\n') {
      break;
    }
    if (c < '0' || c > '9') {
      continue;
    }
    value = value * 10 + static_cast<uint32_t>(c - '0');
    any = true;
  }
  if (!any) {
    out.println(F("nothing entered."));
  }
  return any;
}

void setCodeAndReport(Print &out, uint16_t code) {
  if (!dac.setCode(code)) {
    out.println(F("  [FAIL] write refused or failed."));
    return;
  }
  printExpected(out, code);
}

void threePointFit(Print &out) {
  if (!requireReady(out)) {
    return;
  }
  out.println(F("Three-point fit. Measure the BNC at each point with a DMM."));
  out.println(F("15 s per point. Record volts against the expected value."));
  const uint16_t points[] = {0x0000, 0x8000, 0xFFFF};
  for (unsigned i = 0; i < 3; ++i) {
    out.print(F("  point "));
    out.print(i + 1);
    out.println(F(" of 3:"));
    setCodeAndReport(out, points[i]);
    out.println(F("    measuring window open..."));
    delay(15000);
  }
  out.println(F("Done. Zero scale is the output FLOOR, not 0 V -- the"));
  out.println(F("OPA197 sits about 25 mV off its negative rail at light"));
  out.println(F("load, which is what reserves a driven 0 V as an"));
  out.println(F("out-of-band unhomed/faulted level."));
  out.println();
}

void stepForGroupDelay(Print &out) {
  if (!requireReady(out)) {
    return;
  }
  out.println(F("Step 0 -> full scale on FW_TRIG1's rising edge, then back"));
  out.println(F("after 100 ms. Scope the BNC against InstrumentPort TRIG1"));
  out.println(F("(J1501 pin 28) and read the delay to 50% of the step."));
  out.println(F("Expect about 266 us of group delay from the 2-pole 987 Hz"));
  out.println(F("Sallen-Key, and roughly 2 ms to settle to one OneBox LSB."));
  out.println(F("That delay is the FILTER, not firmware latency."));
  SafeState::setInstrumentationEnabled(true);
  dac.setCode(0);
  delay(50);
  SafeState::setTrigger1(true);
  dac.setCode(0xFFFF);
  delay(100);
  dac.setCode(0);
  SafeState::setTrigger1(false);
  out.println(F("  stepped."));
  out.println();
}

void reportDriverState(Print &out) {
  out.print(F("  configured:   "));
  out.println(dac.configured() ? F("yes") : F("no"));
  out.print(F("  GAIN written: 0x"));
  out.println(dac.gainRegister(), HEX);
  out.print(F("  REF-DIV "));
  out.print((dac.gainRegister() & dac80501::kGainRefDiv) ? F("1") : F("0"));
  out.print(F(", BUFF-GAIN "));
  out.println((dac.gainRegister() & dac80501::kGainBuffGain) ? F("1") : F("0"));
  out.print(F("  DAC full scale: "));
  out.print(dac.fullScale(), 6);
  out.println(F(" V"));
  out.print(F("  BNC full scale: "));
  out.print(dac.fullScale() * kOutputStageGain, 6);
  out.println(F(" V"));
  out.print(F("  one LSB:        "));
  out.print(dac.voltsPerLsb() * 1e6f, 2);
  out.print(F(" uV at the DAC, "));
  out.print(dac.voltsPerLsb() * kOutputStageGain * 1e6f, 2);
  out.println(F(" uV at the BNC"));
  out.print(F("  last code:      "));
  out.println(dac.lastCode());
  out.println();
}

void reportIdentity(Print &out) {
  uint16_t devid = 0;
  if (!dac.readDevid(devid)) {
    out.println(F("  DEVID unreadable. Over SPI this is expected and is not"));
    out.println(F("  a fault: the DAC80501 has no SDO pin, so there is no"));
    out.println(F("  presence check and no reference-alarm visibility at"));
    out.println(F("  all. STATUS.REF-ALARM drives the output to 0 V while"));
    out.println(F("  leaving the DAC codes intact, so a reference alarm and"));
    out.println(F("  a legitimate zero are indistinguishable on this board."));
    out.println();
    return;
  }
  out.print(F("  DEVID 0x"));
  out.println(devid, HEX);
  out.print(F("  plausible: "));
  out.println(dac80501::devidIsPlausible(devid) ? F("yes") : F("NO"));
  out.print(F("  resets to: "));
  out.println(dac80501::devidResetsToMidscale(devid) ? F("midscale")
                                                    : F("zero scale"));
  bool alarm = false;
  if (dac.readReferenceAlarm(alarm)) {
    out.print(F("  REF-ALARM: "));
    out.println(alarm ? F("ACTIVE -- output is 0 V, codes unaffected")
                      : F("clear"));
  }
  out.println();
}

void powerDown(Print &out) {
  // Codes first, then the bus, then the rail: leave the DAC at zero scale so a
  // later rail-up does not present whatever was last written.
  if (dac.configured()) {
    dac.setCode(0);
  }
  PowerSequence::powerDown();
  rail_up = false;
  bus_closed = false;
  // The DAC just lost VDD, so its registers are back to reset values and GAIN
  // is a 5.0 V full scale again. Tell the driver, or it keeps accepting data
  // writes against a span that no longer exists -- which on this board is a
  // full-scale code asking for 6.6 V at the BNC.
  dac.invalidate();
  out.println(F("Powered down: DAC at zero scale, bus open, HV_EN low."));
  out.println(F("+36V_SW takes about 700 ms to bleed down."));
  out.println(F("The DAC lost VDD, so its GAIN is back to a 5.0 V full"));
  out.println(F("scale. Data writes are blocked again until 'b'."));
  out.println();
}

void setup() {
  SafeState::apply();
  Serial.begin(kSerialBaudRate);
  const uint32_t deadline = millis() + kSerialWaitMs;
  while (!Serial && millis() < deadline) {
    delay(10);
  }
  Serial.println();
  Serial.println(F("================================================"));
  Serial.println(F(" mouse-joystick-pcb  stage 5: analog angle output"));
  Serial.println(F("================================================"));
  Serial.println(F(" Requires 36 V and a v0.3 or later board."));
  Serial.println(F(" The motor is not touched. Send 'b' to begin."));
  Serial.println();
  printMenu(Serial);
}

void loop() {
  if (!Serial.available()) {
    return;
  }
  const int command = Serial.read();
  uint32_t entered = 0;
  switch (command) {
  case '?':
    printMenu(Serial);
    break;
  case 'b':
    bringUp(Serial);
    break;
  case 'z':
    if (requireReady(Serial)) {
      setCodeAndReport(Serial, 0x0000);
    }
    break;
  case 'm':
    if (requireReady(Serial)) {
      setCodeAndReport(Serial, 0x8000);
    }
    break;
  case 'f':
    if (requireReady(Serial)) {
      setCodeAndReport(Serial, 0xFFFF);
    }
    break;
  case 'v':
    if (requireReady(Serial) && readNumber(Serial, entered)) {
      const float volts = static_cast<float>(entered) / 1000.0f;
      Serial.print(F("  requested "));
      Serial.print(volts, 4);
      Serial.println(F(" V at the DAC"));
      if (!dac.setVolts(volts)) {
        Serial.println(F("  [FAIL] write refused or failed."));
      } else {
        printExpected(Serial, dac.lastCode());
      }
    }
    break;
  case 'c':
    if (requireReady(Serial) && readNumber(Serial, entered)) {
      if (entered > dac80501::kCodeMax) {
        Serial.println(F("  above 65535, refused."));
      } else {
        setCodeAndReport(Serial, static_cast<uint16_t>(entered));
      }
    }
    break;
  case 's':
    threePointFit(Serial);
    break;
  case 't':
    stepForGroupDelay(Serial);
    break;
  case 'q':
    reportDriverState(Serial);
    break;
  case 'd':
    if (requireReady(Serial)) {
      reportIdentity(Serial);
    }
    break;
  case 'r':
    if (dac.softReset()) {
      Serial.println(F("Soft reset sent. GAIN is back to its 5.0 V full"));
      Serial.println(F("scale, so data writes are blocked until 'b'."));
    } else {
      Serial.println(F("Soft reset frame failed."));
    }
    break;
  case 'x':
    powerDown(Serial);
    break;
  case 'B':
    Serial.println(F("Rebooting into BOOTSEL..."));
    delay(100);
    powerDown(Serial);
    rp2040.rebootToBootloader();
    break;
  case 'R':
    Serial.println(F("Rebooting..."));
    delay(100);
    powerDown(Serial);
    rp2040.reboot();
    break;
  case '\r':
  case '\n':
    break;
  default:
    Serial.print(F("unknown command: '"));
    Serial.print(static_cast<char>(command));
    Serial.println(F("'"));
    break;
  }
}
