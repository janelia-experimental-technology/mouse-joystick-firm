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

// A plausible junction temperature for a board sitting on a bench. Mostly this
// proves the ADC and ADC_AVDD work at all.
constexpr float kTempMinCelsius = 5.0f;
constexpr float kTempMaxCelsius = 60.0f;

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
  analogReadResolution(12);
  pinMode(pins::kSwRailSenseDiv2, INPUT);
  pinMode(pins::kHvSns, INPUT);
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
  out.println(F("[4] qspi flash (W25Q128JVPIQ, 16 MiB expected)"));

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

void SelfTest::reportStraps(Print &out, uint16_t &checks, uint16_t &failures) {
  out.println(F("[6] resistor-defined levels"));
  out.println(
      F("    Read with internal pulls off, so each level comes from a"));
  out.println(
      F("      board resistor. A failure here is a missing or misplaced"));
  out.println(F("      part, not firmware."));

  const ResetStateReads reads = SafeState::readResetState();

  out.print(F("    GP9  ~{BUS_SW_EN}    = "));
  out.println(reads.bus_sw_en_n ? F("high") : F("low"));
  report(out, "R601 7.87k holds the GPIO bus isolated", reads.bus_sw_en_n,
         checks, failures);

  out.print(F("    GP24 ~{INSTR_BUF_OE} = "));
  out.println(reads.instr_buf_oe_n ? F("high") : F("low"));
  report(out, "buffer OE pull-ups hold the instrument port silent",
         reads.instr_buf_oe_n, checks, failures);

  out.print(F("    GP1  PYC_UART_RX     = "));
  out.println(reads.pyc_uart_rx ? F("high") : F("low"));
  report(out, "R403 10k holds the pyControl RX idle high", reads.pyc_uart_rx,
         checks, failures);

  out.print(F("    GP29 ~{HV_FAULT}     = "));
  out.println(reads.hv_fault_n ? F("high") : F("low"));
  report(out, "R1905 10k present and no latched 36 V fault", reads.hv_fault_n,
         checks, failures);

  // Re-establish the safe state: readResetState() left these four pins as
  // inputs. That window is safe because it is exactly the reset condition --
  // R601 alone holds the bus isolated, which is how the board comes up.
  SafeState::apply();
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
  out.println(F(" V across R1907 249R"));
  report(out, "no 36 V load current (HV_SNS below 0.10 V)",
         hv_volts <= kHvSenseOffMaxVolts, checks, failures);
  out.println(
      F("    note: HV_SNS reads zero regardless while HV_DIAG_EN is low."));
  out.println(
      F("      Stage 1 keeps it low on purpose, so this check confirms the"));
  out.println(F("      quiet state rather than the sense chain."));

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

uint16_t SelfTest::run(Print &out) {
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
  reportStraps(out, checks, failures);
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

void SelfTest::pulseTriggers(Print &out, uint32_t half_period_us,
                             uint32_t cycles) {
  out.print(F("Pulsing FW_TRIG0 (GP25) and FW_TRIG1 (GP21): "));
  out.print(cycles);
  out.print(F(" cycles at "));
  out.print(half_period_us * 2);
  out.println(F(" us period."));
  out.println(
      F("Watch InstrumentPort-40 TRIG0 and TRIG1. FW_TRIG1 runs at half"));
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
