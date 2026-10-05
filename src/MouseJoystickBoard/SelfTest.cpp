#include <Arduino.h>
#include <hardware/clocks.h>
#include <hardware/watchdog.h>
#include <math.h>
#include <new>
#include <pico/unique_id.h>

#include "BoardPins.hpp"
#include "Conversions.hpp"
#include "SafeState.hpp"
#include "SelfTest.hpp"

namespace mouse_joystick {

namespace {

constexpr uint32_t kXipBase = 0x10000000u;

// Expected clk_sys for this build. The `generic` board runs the RP2040 at its
// datasheet default.
constexpr uint32_t kExpectedSysClockKhz = 133000;

// The frequency counter measures against clk_ref, which is the 12 MHz crystal,
// so a correct PLL lands within a few counts. 1% is loose enough to never
// false-alarm and tight enough to catch a misconfigured divider.
constexpr float kSysClockTolerance = 0.01f;

// With the 36 V input absent the switched rail is held at 0 V by R401/R402 and
// the sense divider should read essentially nothing. Thresholds are on the
// undivided rail, so they compare directly against +3.3V_SW. Allow a little for
// ADC offset and leakage.
constexpr float kSwRailOffMaxVolts = 0.30f;
constexpr float kHvSenseOffMaxVolts = 0.10f;

// Boot signature. The 10 ms cadence on TRIG1 appears nowhere else in this
// firmware, so it is unambiguous in a capture; the faster TRIG0 burst marks the
// end so a capture can be triggered on either edge density.
constexpr uint32_t kSignatureSlowPulses = 3;
constexpr uint32_t kSignatureSlowHalfMs = 10;
constexpr uint32_t kSignatureFastPulses = 8;
constexpr uint32_t kSignatureFastHalfMs = 1;

// Heartbeat half period. 500 ms gives a 1 Hz square wave: slow enough to spot
// by eye in a long capture, fast enough not to wait for.
constexpr uint32_t kHeartbeatHalfPeriodMs = 500;

// Identity beacon. FW_TRIG0 is the clock and FW_TRIG1 the data, which makes the
// frame decodable by sampling TRIG1 at each TRIG0 rising edge -- no timing
// tolerance beyond sampling faster than the clock, and no assumption that the
// analyser and this board agree on rate.
//
// The preamble is one long FW_TRIG1 high with the clock parked low. 50 ms is
// chosen to be unmistakable against the two other things that drive these
// pins: the boot signature's 10 ms FW_TRIG1 pulses and the 500 ms heartbeat,
// which is on FW_TRIG0 and never touches FW_TRIG1.
constexpr uint32_t kBeaconPreambleMs = 50;
constexpr uint32_t kBeaconGapMs = 10;
constexpr uint32_t kBeaconHalfBitUs = 500;
constexpr uint32_t kBeaconIdBits = 8u * PICO_UNIQUE_BOARD_ID_SIZE_BYTES;

// A plausible junction temperature for a board sitting on a bench. Mostly this
// proves the ADC and ADC_AVDD work at all.
constexpr float kTempMinCelsius = 5.0f;
constexpr float kTempMaxCelsius = 60.0f;

// v1.0 analog monitors with the switched domain down: both nodes are held at
// 0 V by 100 kohm-class pull-downs, so anything above this is a short to a
// live rail or a part fitted wrong. ADC offset on this part is a few mV.
constexpr float kMonitorOffMaxVolts = 0.10f;

void report(Print &out, const char *label, bool ok, uint16_t &checks,
            uint16_t &failures) {
  ++checks;
  if (!ok) {
    ++failures;
  }
  out.print(ok ? F("    [ ok ] ") : F("    [FAIL] "));
  out.println(label);
}

void printKeyValue(Print &out, const char *key, const char *value) {
  out.print(F("    "));
  out.print(key);
  out.print(F(": "));
  out.println(value);
}

void printClock(Print &out, const char *name, uint32_t hz) {
  out.print(F("    "));
  out.print(name);
  out.print(F(": "));
  out.print(hz / 1000);
  out.println(F(" kHz"));
}

// 32 bytes of the XIP window at a given offset.
void readXip(uint32_t offset, uint8_t *dest, size_t length) {
  const volatile uint8_t *src =
      reinterpret_cast<const volatile uint8_t *>(kXipBase + offset);
  for (size_t i = 0; i < length; ++i) {
    dest[i] = src[i];
  }
}

bool sameBytes(const uint8_t *a, const uint8_t *b, size_t length) {
  for (size_t i = 0; i < length; ++i) {
    if (a[i] != b[i]) {
      return false;
    }
  }
  return true;
}

bool heartbeat_level = false;
uint32_t heartbeat_due_ms = 0;

const char *resetReasonName(int reason) {
  // Order taken from the core's RP2040 class enum resetReason_t.
  static const char *const kNames[] = {"unknown",  "power-on", "run pin",
                                       "software", "watchdog", "debug",
                                       "glitch",   "brownout"};
  constexpr int kCount = static_cast<int>(sizeof(kNames) / sizeof(kNames[0]));
  return (reason >= 0 && reason < kCount) ? kNames[reason] : "out of range";
}

} // namespace

void SelfTest::beginAdc() {
  analogReadResolution(conversions::kAdcResolutionBits);
  pinMode(pins::kSwRailSenseDiv2, INPUT);
  pinMode(pins::kHvSns, INPUT);
#if MJ_BOARD_REV >= 2
  pinMode(pins::kBrakeIsense, INPUT);
  pinMode(pins::kAngleOutMon, INPUT);
#endif
}

void SelfTest::reportIdentity(Print &out) {
  out.println(F("[1] identity"));

  char id[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1] = {0};
  pico_get_unique_board_id_string(id, sizeof(id));
  printKeyValue(out, "flash unique id", id);
  out.println(
      F("    ^ this is the board serial. Calibration belongs to it, not"));
  out.println(F("      to a joystick position, and two boards in one rig are"));
  out.println(F("      independently suspect."));

#if defined(USB_MANUFACTURER) && defined(USB_PRODUCT)
  printKeyValue(out, "usb manufacturer", USB_MANUFACTURER);
  printKeyValue(out, "usb product", USB_PRODUCT);
#endif
  printKeyValue(out, "built", __DATE__ " " __TIME__);

  out.print(F("    targets board revision: rev "));
  out.print(MJ_BOARD_REV);
#if MJ_BOARD_REV == 0
  out.println(F("  (pcb tags v0.1 and v0.2 ONLY)"));
  out.println(
      F("    GP10, GP11 and GP13 are unrouted on this revision, so they"));
  out.println(
      F("      are tested as spare pads. From v0.3 they carry the DAC,"));
  out.println(
      F("      so a v0.3 board needs env:mouse_joystick_rev1 instead."));
#elif MJ_BOARD_REV == 1
  out.println(F("  (pcb v0.3, angle output DAC on SPI1)"));
  out.println(
      F("    GP10, GP11 and GP13 carry the DAC bus, so they are NOT"));
  out.println(F("      exercised as spare pads: 9 pads here against 12 on"));
  out.println(F("      rev 0, and 24 checks against 27. pcb HEAD is v1.0,"));
  out.println(F("      which is rev 2, not this."));
#else
  out.println(F("  (pcb v1.0: I2C DACs, torque ADC, brake, LATCH on GP24)"));
  out.println(
      F("    Only GP20 and GP22 are spare. GP10-13, GP15-18 reach the"));
  out.println(
      F("      torque, brake and DAC blocks through the bus switch and"));
  out.println(F("      are left alone: 2 pads, 20 checks. ~{HV_FAULT} is"));
  out.println(F("      GP23 here; GP29 is an ADC input. Confirmed on"));
  out.println(F("      hardware 2026-10-05: 20 checks on E4654C159F274534."));
#endif
}

void SelfTest::reportResetReason(Print &out) {
  out.println(F("[2] reset"));
  const int reason = static_cast<int>(rp2040.getResetReason());
  out.print(F("    reason: "));
  out.print(resetReasonName(reason));
  out.print(F(" ("));
  out.print(reason);
  out.println(F(")"));
  out.print(F("    watchdog caused reboot: "));
  out.println(watchdog_caused_reboot() ? F("yes") : F("no"));
}

void SelfTest::reportClocks(Print &out, uint16_t &checks, uint16_t &failures) {
  out.println(F("[3] clocks"));
  printClock(out, "clk_ref  (xosc)", clock_get_hz(clk_ref));
  printClock(out, "clk_sys        ", clock_get_hz(clk_sys));
  printClock(out, "clk_peri       ", clock_get_hz(clk_peri));
  printClock(out, "clk_usb        ", clock_get_hz(clk_usb));
  printClock(out, "clk_adc        ", clock_get_hz(clk_adc));

  const uint32_t measured_khz =
      frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
  out.print(F("    clk_sys measured against clk_ref: "));
  out.print(measured_khz);
  out.println(F(" kHz"));

  const float tolerance = kExpectedSysClockKhz * kSysClockTolerance;
  const float error = fabsf(static_cast<float>(measured_khz) -
                            static_cast<float>(kExpectedSysClockKhz));
  report(out, "clk_sys within 1% of 133000 kHz", error <= tolerance, checks,
         failures);

  out.println(F("    note: the counter measures clk_sys *against* the 12 MHz"));
  out.println(F("      crystal, so it proves the PLL, not the crystal. The"));
  out.println(
      F("      crystal's own absolute proof is that you are reading this"));
  out.println(
      F("      at all: USB full speed needs 48 MHz within 0.25%, derived"));
  out.println(
      F("      from XOSC. Use the 'k' command to measure it against your"));
  out.println(F("      host clock."));
}

void SelfTest::reportFlash(Print &out, uint16_t &checks, uint16_t &failures) {
  out.println(F("[4] qspi flash (U502 W25Q128JVZPIQ, 16 MiB expected)"));

  pico_unique_board_id_t raw;
  pico_get_unique_board_id(&raw);
  bool all_zero = true;
  bool all_ones = true;
  for (size_t i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; ++i) {
    if (raw.id[i] != 0x00) {
      all_zero = false;
    }
    if (raw.id[i] != 0xFF) {
      all_ones = false;
    }
  }
  // Reading the unique id issues a real 0x4B command over QSPI, so a plausible
  // answer means the flash is present, selected, and responding.
  report(out, "unique id read back plausible (flash responds)",
         !all_zero && !all_ones, checks, failures);

#if defined(PICO_FLASH_SIZE_BYTES)
  out.print(F("    declared size: "));
  out.print(static_cast<uint32_t>(PICO_FLASH_SIZE_BYTES) / (1024u * 1024u));
  out.println(F(" MiB"));
  report(out, "declared size is 16 MiB",
         PICO_FLASH_SIZE_BYTES == 16u * 1024u * 1024u, checks, failures);
#endif

  // Address aliasing probe. A flash smaller than the build assumes wraps its
  // address space, so a read at offset N would return the same bytes as offset
  // 0 -- and offset 0 holds boot2, which is certainly not erased. Distinct
  // bytes at each offset mean the part really is at least that large.
  uint8_t base[32];
  uint8_t probe[32];
  readXip(0, base, sizeof(base));
  const uint32_t offsets_mib[] = {2, 4, 8};
  for (uint32_t mib : offsets_mib) {
    readXip(mib * 1024u * 1024u, probe, sizeof(probe));
    const bool distinct = !sameBytes(base, probe, sizeof(base));
    out.print(F("    offset +"));
    out.print(mib);
    out.println(F(" MiB:"));
    report(out,
           distinct ? "does not alias offset 0"
                    : "ALIASES offset 0 - flash is smaller",
           distinct, checks, failures);
  }
}

void SelfTest::reportRam(Print &out, uint16_t &checks, uint16_t &failures) {
  out.println(F("[5] ram"));
  out.print(F("    heap total/used/free: "));
  out.print(rp2040.getTotalHeap());
  out.print('/');
  out.print(rp2040.getUsedHeap());
  out.print('/');
  out.println(rp2040.getFreeHeap());

  // Non-destructive: a heap block we own, walked with three patterns.
  constexpr size_t kBlockWords = 4096; // 16 KiB
  uint32_t *block = new (std::nothrow) uint32_t[kBlockWords];
  if (block == nullptr) {
    report(out, "16 KiB heap allocation", false, checks, failures);
    return;
  }
  report(out, "16 KiB heap allocation", true, checks, failures);

  const uint32_t patterns[] = {0x00000000u, 0xFFFFFFFFu, 0xA5A5A5A5u,
                               0x5A5A5A5Au};
  bool ok = true;
  for (uint32_t pattern : patterns) {
    for (size_t i = 0; i < kBlockWords; ++i) {
      block[i] = pattern ^ static_cast<uint32_t>(i);
    }
    for (size_t i = 0; i < kBlockWords; ++i) {
      if (block[i] != (pattern ^ static_cast<uint32_t>(i))) {
        ok = false;
        break;
      }
    }
    if (!ok) {
      break;
    }
  }
  delete[] block;
  report(out, "pattern walk over 16 KiB", ok, checks, failures);
}

void SelfTest::reportStraps(Print &out, const ResetStateReads &reads,
                            uint16_t &checks, uint16_t &failures) {
  out.println(F("[6] resistor-defined levels"));
  out.println(
      F("    Captured at boot, internal pulls off, before anything drove"));
  out.println(
      F("      a pin, so each level comes from a board resistor. A failure"));
#if MJ_BOARD_REV == 0
  out.println(F("      here is a missing part, not firmware. v0.1 refdes."));
#elif MJ_BOARD_REV == 1
  out.println(F("      here is a missing part, not firmware. v0.3 refdes."));
#else
  out.println(F("      here is a missing part, not firmware. v1.0 refdes."));
#endif

  if (!reads.valid) {
    report(out, "reset state was captured at boot", false, checks, failures);
    return;
  }

  out.print(F("    GP9  ~{BUS_SW_EN}    = "));
  out.println(reads.bus_sw_en_n ? F("high") : F("low"));
#if MJ_BOARD_REV < 2
  report(out, "R601 7.87k holds the GPIO bus isolated", reads.bus_sw_en_n,
         checks, failures);
#else
  report(out, "R607 7.87k holds the GPIO bus isolated", reads.bus_sw_en_n,
         checks, failures);
#endif

#if MJ_BOARD_REV < 2
  out.print(F("    GP24 ~{INSTR_BUF_OE} = "));
  out.println(reads.instr_buf_oe_n ? F("high") : F("low"));
  report(out, "buffer OE pull-ups hold the instrument port silent",
         reads.instr_buf_oe_n, checks, failures);
#else
  // Not a check. The TPS1HTC100-Q1 is unpowered in stage 1 (36 V absent), so
  // this is its unpowered LATCH pin seen through R503 10k. Recorded because
  // the level at reset is the configuration the switch wakes up in, and
  // because a board that reads high here with 36 V absent deserves a look.
  out.print(F("    GP24 HV_LATCH        = "));
  out.print(reads.hv_latch ? F("high") : F("low"));
  out.println(F("  (informational: switch unpowered; low = auto-retry)"));

  out.print(F("    GP16 DAC_SDA         = "));
  out.println(reads.dac_sda ? F("high") : F("low"));
  report(out, "R403 4.7k holds the I2C SDA tap idle high", reads.dac_sda,
         checks, failures);
  out.print(F("    GP17 DAC_SCL         = "));
  out.println(reads.dac_scl ? F("high") : F("low"));
  report(out, "R402 4.7k holds the I2C SCL tap idle high", reads.dac_scl,
         checks, failures);
#endif

  out.print(F("    GP1  PYC_UART_RX     = "));
  out.println(reads.pyc_uart_rx ? F("high") : F("low"));
#if MJ_BOARD_REV < 2
  report(out, "R403 10k holds the pyControl RX idle high", reads.pyc_uart_rx,
         checks, failures);
#else
  report(out, "R401 10k holds the pyControl RX idle high", reads.pyc_uart_rx,
         checks, failures);
#endif

  // The 36 V block moved from the R2xx range to R19xx between v0.2 and v0.3
  // and to R5xx at v1.0, so this pull-up has three names. Naming the wrong
  // one sends whoever is probing to a part that does not exist on the board
  // in front of them. At v1.0 the pin moved too: GP29 became an ADC input.
#if MJ_BOARD_REV == 0
  out.print(F("    GP29 ~{HV_FAULT}     = "));
  out.println(reads.hv_fault_n ? F("high") : F("low"));
  report(out, "R205 10k present and no latched 36 V fault", reads.hv_fault_n,
         checks, failures);
#elif MJ_BOARD_REV == 1
  out.print(F("    GP29 ~{HV_FAULT}     = "));
  out.println(reads.hv_fault_n ? F("high") : F("low"));
  report(out, "R1905 10k present and no latched 36 V fault", reads.hv_fault_n,
         checks, failures);
#else
  out.print(F("    GP23 ~{HV_FAULT}     = "));
  out.println(reads.hv_fault_n ? F("high") : F("low"));
  report(out, "R507 10k present and no latched 36 V fault", reads.hv_fault_n,
         checks, failures);
#endif

  // Nothing to restore: these levels were captured at boot and this section
  // reads no pin.
}

void SelfTest::reportSparePads(Print &out, uint16_t &checks,
                               uint16_t &failures) {
  out.println(F("[7] spare pads"));
  out.println(F("    Routed nowhere, so an internal pull must win in both"));
  out.println(
      F("      directions. A pad that will not move is shorted; one that"));
  out.println(
      F("      reads wrong either way is an open or a bridged neighbour."));

  for (uint8_t i = 0; i < pins::kSpareCount; ++i) {
    const uint8_t pin = pins::kSpares[i];

    pinMode(pin, INPUT_PULLUP);
    delayMicroseconds(50);
    const bool pulled_up = digitalRead(pin) == HIGH;

    pinMode(pin, INPUT_PULLDOWN);
    delayMicroseconds(50);
    const bool pulled_down = digitalRead(pin) == LOW;

    pinMode(pin, INPUT);

    ++checks;
    const bool ok = pulled_up && pulled_down;
    if (!ok) {
      ++failures;
    }
    out.print(ok ? F("    [ ok ] GP") : F("    [FAIL] GP"));
    out.print(pin);
    out.print(F(" pull-up->"));
    out.print(pulled_up ? F("1") : F("0"));
    out.print(F(" pull-down->"));
    out.println(pulled_down ? F("0") : F("1"));
  }
}

void SelfTest::reportAdc(Print &out, uint16_t &checks, uint16_t &failures) {
  out.println(F("[8] adc"));
  beginAdc();

  const uint16_t rail_counts = analogRead(pins::kSwRailSenseDiv2);
  const float rail_divided = conversions::countsToVolts(rail_counts);
  const float rail_volts = conversions::swRailVolts(rail_counts);
  out.print(F("    GP26 SW_RAIL_SENSE_DIV_2: "));
  out.print(rail_counts);
  out.print(F(" counts = "));
  out.print(rail_divided, 4);
  out.print(F(" V divided = "));
  out.print(rail_volts, 4);
  out.println(F(" V on +3.3V_SW"));
  report(out, "+3.3V_SW is off (below 0.30 V)",
         rail_volts <= kSwRailOffMaxVolts, checks, failures);

  const uint16_t hv_counts = analogRead(pins::kHvSns);
  const float hv_volts = conversions::hvSenseVolts(hv_counts);
  out.print(F("    GP28 HV_SNS: "));
  out.print(hv_counts);
  out.print(F(" counts = "));
  out.print(hv_volts, 4);
#if MJ_BOARD_REV == 0
  out.println(F(" V across R207 249R"));
#elif MJ_BOARD_REV == 1
  out.println(F(" V across R1907 249R"));
#else
  out.println(F(" V across R509 249R (behind R505 1k)"));
#endif
  report(out, "no 36 V load current (HV_SNS below 0.10 V)",
         hv_volts <= kHvSenseOffMaxVolts, checks, failures);
  out.println(
      F("    note: HV_SNS reads zero regardless while HV_DIAG_EN is low."));
  out.println(
      F("      Stage 1 keeps it low on purpose, so this check confirms the"));
  out.println(F("      quiet state rather than the sense chain."));

#if MJ_BOARD_REV >= 2
  // The two v1.0 output monitors. With the switched domain down both nodes
  // are pulled to 0 V (R1818 110k, R805 100k); the rail off is the only thing
  // this can confirm. The live-chain numbers belong to stages A3 and A4 with
  // the rail up and a meter on the BNC.
  const uint16_t brake_counts = analogRead(pins::kBrakeIsense);
  const float brake_volts = conversions::countsToVolts(brake_counts);
  out.print(F("    GP27 BRAKE_ISENSE: "));
  out.print(brake_counts);
  out.print(F(" counts = "));
  out.print(brake_volts, 4);
  out.println(F(" V  (0 to 1.006 V is 0 to 48.78 mA with the rail up)"));
  report(out, "brake current monitor at 0 V with the rail off",
         brake_volts <= kMonitorOffMaxVolts, checks, failures);

  const uint16_t angle_counts = analogRead(pins::kAngleOutMon);
  const float angle_volts = conversions::countsToVolts(angle_counts);
  out.print(F("    GP29 ANGLE_OUT_MON: "));
  out.print(angle_counts);
  out.print(F(" counts = "));
  out.print(angle_volts, 4);
  out.println(F(" V  (the BNC node, 0 to 1.650 V with the rail up)"));
  report(out, "angle output monitor at 0 V with the rail off",
         angle_volts <= kMonitorOffMaxVolts, checks, failures);
#endif

  const float temp_c = analogReadTemp();
  out.print(F("    internal temperature: "));
  out.print(temp_c, 1);
  out.println(F(" C"));
  report(out, "die temperature plausible (ADC and ADC_AVDD alive)",
         temp_c >= kTempMinCelsius && temp_c <= kTempMaxCelsius, checks,
         failures);
}

void SelfTest::reportAdcOnce(Print &out) {
  uint16_t checks = 0;
  uint16_t failures = 0;
  reportAdc(out, checks, failures);
}

uint16_t SelfTest::run(Print &out, const ResetStateReads &reads) {
  uint16_t checks = 0;
  uint16_t failures = 0;

  out.println();
  out.println(
      F("================================================================"));
  out.println(F(" mouse-joystick-pcb  RP2040 self test"));
  out.println(
      F("================================================================"));
  out.println(F(" 36 V input absent, switched domain off, GPIO bus isolated."));
  out.println(F(" This test cannot reach the TMC5130A and does not try."));
  out.println();

  reportIdentity(out);
  out.println();
  reportResetReason(out);
  out.println();
  reportClocks(out, checks, failures);
  out.println();
  reportFlash(out, checks, failures);
  out.println();
  reportRam(out, checks, failures);
  out.println();
  reportStraps(out, reads, checks, failures);
  out.println();
  reportSparePads(out, checks, failures);
  out.println();
  reportAdc(out, checks, failures);
  out.println();

  out.println(
      F("----------------------------------------------------------------"));
  out.print(F(" result: "));
  out.print(checks);
  out.print(F(" checks, "));
  out.print(failures);
  out.println(failures == 0 ? F(" failed") : F(" FAILED"));
  out.println(
      F("----------------------------------------------------------------"));
  out.println(
      F(" Record these numbers in mouse-joystick-pcb/docs/bringup-log.md"));
  out.println(
      F(" against this board's serial. Numbers with units, not pass marks."));
  out.println();

  return failures;
}

void SelfTest::emitBootSignature() {
  // Enable the port first: with the buffers off these edges reach nothing, and
  // the whole point of the signature is to be visible without USB.
  SafeState::setInstrumentationEnabled(true);

  for (uint32_t i = 0; i < kSignatureSlowPulses; ++i) {
    SafeState::setTrigger1(true);
    delay(kSignatureSlowHalfMs);
    SafeState::setTrigger1(false);
    delay(kSignatureSlowHalfMs);
  }
  for (uint32_t i = 0; i < kSignatureFastPulses; ++i) {
    SafeState::setTrigger0(true);
    delay(kSignatureFastHalfMs);
    SafeState::setTrigger0(false);
    delay(kSignatureFastHalfMs);
  }

  // Leave the port enabled. Nothing on this board can observe firmware
  // otherwise, and the port is passive and unidirectional by design, so leaving
  // it on cannot affect board behaviour. The `i` command turns it off.
  heartbeat_level = false;
  heartbeat_due_ms = millis() + kHeartbeatHalfPeriodMs;
}

void SelfTest::serviceHeartbeat() {
  if (!SafeState::instrumentationEnabled()) {
    return;
  }
  if (static_cast<int32_t>(millis() - heartbeat_due_ms) < 0) {
    return;
  }
  heartbeat_level = !heartbeat_level;
  SafeState::setTrigger0(heartbeat_level);
  heartbeat_due_ms = millis() + kHeartbeatHalfPeriodMs;
}

void SelfTest::pulseTriggers(Print &out, uint32_t half_period_us,
                             uint32_t cycles) {
  out.print(F("Pulsing FW_TRIG0 (GP25) and FW_TRIG1 (GP21): "));
  out.print(cycles);
  out.print(F(" cycles at "));
  out.print(half_period_us * 2);
  out.println(F(" us period."));
#if MJ_BOARD_REV < 2
  out.println(
      F("Watch InstrumentPort-40 TRIG0 and TRIG1. FW_TRIG1 runs at half"));
#else
  out.println(
      F("Watch Digital-50 DIG0 and DIG1 (LA CH0/CH1). FW_TRIG1 runs at half"));
#endif
  out.println(
      F("the rate of FW_TRIG0, so a capture that shows one without the"));
  out.println(
      F("other has a buffer or port problem rather than a pin problem."));

  const bool was_enabled = SafeState::instrumentationEnabled();
  SafeState::setInstrumentationEnabled(true);

  for (uint32_t i = 0; i < cycles; ++i) {
    SafeState::setTrigger0(true);
    SafeState::setTrigger1((i % 2) == 0);
    delayMicroseconds(half_period_us);
    SafeState::setTrigger0(false);
    delayMicroseconds(half_period_us);
  }
  SafeState::setTrigger0(false);
  SafeState::setTrigger1(false);

  SafeState::setInstrumentationEnabled(was_enabled);
  out.println(F("Done."));
}

void SelfTest::emitIdentityBeacon(Print &out, uint32_t repeats) {
  pico_unique_board_id_t raw;
  pico_get_unique_board_id(&raw);

  char id[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1] = {0};
  pico_get_unique_board_id_string(id, sizeof(id));

  const uint32_t frame_ms = kBeaconPreambleMs + kBeaconGapMs +
                            (kBeaconIdBits * 2u * kBeaconHalfBitUs) / 1000u +
                            kBeaconGapMs;

  out.print(F("Beaconing board identity on InstrumentPort-40: "));
  out.println(id);
  out.println(F("  TRIG1 (ch17) high 50 ms starts a frame; TRIG0 (ch16) then"));
  out.println(F("  clocks 64 bits MSB first, each valid on TRIG1 at the"));
  out.println(F("  rising edge. Trigger the capture on ch17 rising."));
  out.print(F("  "));
  out.print(repeats);
  out.print(F(" frames, "));
  out.print(frame_ms);
  out.println(F(" ms each."));
  out.println(F("  No 36 V and no bus needed: U1801 runs from +3.3V_MCU, so"));
  out.println(F("  this reaches the port with the switched domain dark."));

  const bool was_enabled = SafeState::instrumentationEnabled();
  SafeState::setInstrumentationEnabled(true);

  for (uint32_t frame = 0; frame < repeats; ++frame) {
    // Frame start, with the clock parked low so the preamble cannot be read as
    // data by a decoder that only counts clock edges.
    SafeState::setTrigger0(false);
    SafeState::setTrigger1(true);
    delay(kBeaconPreambleMs);
    SafeState::setTrigger1(false);
    delay(kBeaconGapMs);

    for (uint32_t bit = 0; bit < kBeaconIdBits; ++bit) {
      // MSB first, so the clocked bits arrive in the same order as the hex
      // string printed above and as the USB serial in /dev/serial/by-id.
      const uint8_t mask = static_cast<uint8_t>(0x80u >> (bit % 8u));
      const bool level = (raw.id[bit / 8u] & mask) != 0;

      // Data set up before the edge, held across it: the decoder samples
      // TRIG1 on the rising edge of TRIG0 and needs no phase guess.
      SafeState::setTrigger1(level);
      delayMicroseconds(kBeaconHalfBitUs);
      SafeState::setTrigger0(true);
      delayMicroseconds(kBeaconHalfBitUs);
      SafeState::setTrigger0(false);
    }

    SafeState::setTrigger1(false);
    delay(kBeaconGapMs);
  }

  SafeState::setTrigger0(false);
  SafeState::setTrigger1(false);
  SafeState::setInstrumentationEnabled(was_enabled);
  out.println(F("Done."));
}

void SelfTest::streamClockTicks(Print &out, uint32_t seconds) {
  out.print(F("Emitting one line per second for "));
  out.print(seconds);
  out.println(F(" s. Timestamp these at the host and compare."));
  out.println(F("A 12 MHz crystal off by 1% shows up as 10 ms of drift per"));
  out.println(F("second; the on-chip counter cannot see that error at all."));

  const uint32_t start_us = micros();
  for (uint32_t s = 1; s <= seconds; ++s) {
    const uint32_t target_us = start_us + s * 1000000u;
    while (static_cast<int32_t>(micros() - target_us) < 0) {
      // Spin. Nothing else needs the CPU during this test.
    }
    out.print(F("tick "));
    out.print(s);
    out.print(F("  rp2040 micros elapsed: "));
    out.println(micros() - start_us);
  }
  out.println(F("Done."));
}

bool SelfTest::pycUartLoopback(Print &out, uint32_t baud) {
  out.print(F("pyControl UART loopback at "));
  out.print(baud);
  out.println(F(" baud on GP0 (TX) / GP1 (RX)."));
  out.println(F("Needs DIO-A shorted to DIO-B on the RJ45. With no loopback"));
  out.println(F("fitted, zero bytes received is the correct result."));

  Serial1.end();
  Serial1.setTX(pins::kPycUartTx);
  Serial1.setRX(pins::kPycUartRx);
  Serial1.begin(baud);

  // Drain anything the line noise left behind.
  const uint32_t drain_deadline = millis() + 20;
  while (millis() < drain_deadline) {
    if (Serial1.available()) {
      (void)Serial1.read();
    }
  }

  static const uint8_t pattern[] = {0x00, 0xFF, 0x55, 0xAA,
                                    0x5A, 0xA5, 0x01, 0x80};
  constexpr size_t kLength = sizeof(pattern);
  Serial1.write(pattern, kLength);
  Serial1.flush();

  uint8_t received[kLength] = {0};
  size_t count = 0;
  const uint32_t deadline = millis() + 200;
  while (count < kLength && millis() < deadline) {
    if (Serial1.available()) {
      received[count++] = static_cast<uint8_t>(Serial1.read());
    }
  }
  Serial1.end();

  out.print(F("    sent "));
  out.print(static_cast<uint32_t>(kLength));
  out.print(F(" bytes, received "));
  out.println(static_cast<uint32_t>(count));

  if (count != kLength) {
    out.println(F("    [ -- ] no loopback detected"));
    return false;
  }

  bool ok = true;
  for (size_t i = 0; i < kLength; ++i) {
    if (received[i] != pattern[i]) {
      ok = false;
      out.print(F("    byte "));
      out.print(static_cast<uint32_t>(i));
      out.print(F(": sent 0x"));
      out.print(pattern[i], HEX);
      out.print(F(" received 0x"));
      out.println(received[i], HEX);
    }
  }
  out.println(ok ? F("    [ ok ] loopback pattern matched")
                 : F("    [FAIL] loopback corrupted"));
  return ok;
}

} // namespace mouse_joystick
