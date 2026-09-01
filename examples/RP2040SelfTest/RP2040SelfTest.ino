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
// header, so USB CDC is the only feedback channel. Open the monitor and read.

#include <MouseJoystickBoard.hpp>

using mouse_joystick::SafeState;
using mouse_joystick::SelfTest;

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
  out.println(F("  i  toggle the instrumentation buffers"));
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
  // Safe state before anything else, and before Serial: a host that never
  // attaches still leaves the board with the 36 V switch off, the GPIO bus
  // isolated, and the driver disabled.
  //
  // The resistor-defined pin levels are read later, by reportStraps(). Reading
  // them there loses nothing: it samples each pin as an input with the
  // internal pulls disabled, so the level always comes from the board rather
  // than from whatever firmware last drove.
  SafeState::apply();

  Serial.begin(kSerialBaudRate);
  const uint32_t deadline = millis() + kSerialWaitMs;
  while (!Serial && millis() < deadline) {
    delay(10);
  }
  delay(100);

  SelfTest::run(Serial);
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
  case 'r':
    SelfTest::run(Serial);
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
