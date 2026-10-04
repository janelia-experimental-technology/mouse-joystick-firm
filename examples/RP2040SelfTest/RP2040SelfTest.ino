// Stage 1 bring-up: prove the RP2040 subsystem of mouse-joystick-pcb.
//
// Run this with the 36 V input UNPLUGGED. The board's MCU domain is fed from
// USB VBUS or pyControl pin 6 through two ORing diodes, never from the 36 V
// chain, so USB alone is enough to run this firmware.
//
// This sketch never enables HV_EN and never enables the GPIO isolation bus
// switch, so it cannot reach the TMC5130A. Driver communication is stage 2 and
// belongs in its own example.
//
// There are no LEDs and no test points on this board, and rev 0 has no SWD
// header. That leaves two feedback channels, and the second one matters:
//
//   * USB CDC, for everything the test measures. Open the monitor and read.
//   * FW_TRIG0 and FW_TRIG1 on the instrument port, carrying a boot signature
//     and a 1 Hz heartbeat. These are the only evidence of firmware execution
//     that does not depend on USB working, which is exactly what you want when
//     USB is one of the things under test. InstrumentPort-40 (rev 0/1): TRIG0
//     and TRIG1, LA5032 channels 16 and 17. InstrumentPort-Digital-50 (v1.0):
//     DIG0 and DIG1, LA5032 channels 0 and 1 through the v2.0 adapter.

#include <MouseJoystickBoard.hpp>

using mouse_joystick::ResetStateReads;
using mouse_joystick::SafeState;
using mouse_joystick::SelfTest;

// Captured before anything drives a pin, so the resistor-defined levels
// reported later are genuinely the reset state rather than whatever firmware
// last left.
ResetStateReads reset_reads;

constexpr uint32_t kSerialBaudRate = 115200;

// How long to wait for a host to open the CDC port before giving up and running
// the test anyway. Output is buffered, so a monitor attached later still sees
// most of it -- but the safe state is applied long before this wait starts.
constexpr uint32_t kSerialWaitMs = 5000;

// pyControl link parameters, for the on-demand loopback test.
constexpr uint32_t kPycUartBaudRate = 115200;

void printMenu(Print &out) {
  out.println(F("commands"));
  out.println(F("  ?  this menu"));
  out.println(F("  r  re-run the full self test"));
  out.println(F("  a  read the ADC channels once"));
  out.println(F("  t  pulse FW_TRIG0/FW_TRIG1 into InstrumentPort-40"));
  out.println(F("  I  beacon the board serial on the triggers, so a"));
  out.println(F("     capture identifies which board it came from"));
#if MJ_BOARD_REV < 2
  out.println(F("  i  toggle the instrumentation buffers, which silences"));
  out.println(F("     or restores the 1 Hz FW_TRIG0 heartbeat"));
#else
  out.println(F("  i  (no effect on v1.0: the Digital-50 buffers have no"));
  out.println(F("     enable and are always on)"));
#endif
  out.println(F("  k  10 s clock tick stream, for host-side crystal check"));
  out.println(F("  u  pyControl UART loopback (needs DIO-A shorted to DIO-B)"));
  out.println(F("  B  reboot into BOOTSEL for reflashing"));
  out.println(F("  R  reboot"));
  out.println();
  out.println(
      F("There is deliberately no command to enable HV_EN or the GPIO"));
  out.println(F("isolation bus switch. Stage 1 cannot energise anything."));
  out.println();
}

void setup() {
  // Order matters here.
  //
  // 1. Capture the resistor-defined pin levels. This has to happen before
  //    anything drives a pin, and the boot signature below drives
  //    ~{INSTR_BUF_OE}, so there is exactly one chance to do it.
  reset_reads = SafeState::readResetState();

  // 2. Safe state, before Serial: a host that never attaches still leaves the
  //    board with the 36 V switch off, the GPIO bus isolated, and the driver
  //    disabled.
  SafeState::apply();

  // 3. Boot signature on the InstrumentPort triggers. No LEDs, no test points
  //    and no SWD header on this board, so if USB does not come up this is the
  //    only thing that can say whether the RP2040 runs your code at all.
  //    Watch LA5032 channel 16 (TRIG0, J1501 pin 26) and channel 17 (TRIG1,
  //    J1501 pin 28). It leaves the port enabled so the heartbeat is visible.
  SelfTest::emitBootSignature();

  Serial.begin(kSerialBaudRate);
  const uint32_t deadline = millis() + kSerialWaitMs;
  while (!Serial && millis() < deadline) {
    delay(10);
  }
  delay(100);

  SelfTest::run(Serial, reset_reads);
  printMenu(Serial);
}

void loop() {
  // 1 Hz square wave on FW_TRIG0 whenever the instrumentation port is enabled.
  // A capture started long after boot still shows the firmware is alive and its
  // timebase is sane.
  SelfTest::serviceHeartbeat();

  if (!Serial.available()) {
    delay(10);
    return;
  }

  const int command = Serial.read();
  switch (command) {
  case '?':
    printMenu(Serial);
    break;
  case 'r':
    SelfTest::run(Serial, reset_reads);
    break;
  case 'a':
    SelfTest::reportAdcOnce(Serial);
    break;
  case 't':
    // 100 us half period, 1000 cycles: a 5 kHz burst lasting 200 ms, long
    // enough to catch on a scope without a trigger hunt.
    SelfTest::pulseTriggers(Serial, 100, 1000);
    break;
  case 'i': {
    const bool enabled = !SafeState::instrumentationEnabled();
    SafeState::setInstrumentationEnabled(enabled);
    Serial.print(F("instrumentation buffers "));
    Serial.println(enabled ? F("enabled") : F("disabled"));
    break;
  }
  case 'I':
    SelfTest::emitIdentityBeacon(Serial, 3);
    break;
  case 'k':
    SelfTest::streamClockTicks(Serial, 10);
    break;
  case 'u':
    SelfTest::pycUartLoopback(Serial, kPycUartBaudRate);
    break;
  case 'B':
    Serial.println(F("Rebooting into BOOTSEL. The board will appear as an"));
    Serial.println(F("RPI-RP2 mass storage device."));
    Serial.flush();
    delay(100);
    rp2040.rebootToBootloader();
    break;
  case 'R':
    Serial.println(F("Rebooting."));
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
