// RP2040 subsystem self test for mouse-joystick-pcb.
//
// Scope: everything on the always-on +3.3V_MCU domain, with the 36 V input
// absent and the switched domain at 0 V. It never enables HV_EN and never
// enables the GPIO isolation bus switch, so it cannot reach the TMC5130A. That
// boundary is the point -- prove the MCU before energising anything.
//
// This board has no LEDs and no test points, and rev 0 has no SWD header, so
// USB CDC is the whole feedback channel. Every check therefore reports a number
// or a level, not a bare pass mark.

#pragma once

// Arduino.h rather than a forward declaration of Print: the core's Print lives
// in namespace arduino and is only visible globally through a using directive,
// so a hand-written `class Print;` declares a different, ambiguous type.
#include <Arduino.h>
#include <stdint.h>

namespace mouse_joystick {

class SelfTest {
public:
  // Run every section and print a summary. Returns the number of failed checks.
  static uint16_t run(Print &out);

  // Sections, individually useful from the interactive menu.
  static void reportIdentity(Print &out);
  static void reportResetReason(Print &out);
  static void reportClocks(Print &out, uint16_t &checks, uint16_t &failures);
  static void reportFlash(Print &out, uint16_t &checks, uint16_t &failures);
  static void reportRam(Print &out, uint16_t &checks, uint16_t &failures);
  static void reportStraps(Print &out, uint16_t &checks, uint16_t &failures);
  static void reportSparePads(Print &out, uint16_t &checks, uint16_t &failures);
  static void reportAdc(Print &out, uint16_t &checks, uint16_t &failures);

  // On-demand tests that need either an instrument or a fixture.

  // Square wave on FW_TRIG0 and FW_TRIG1, with the instrumentation buffers
  // enabled so the edges reach InstrumentPort-40 TRIG0/TRIG1. Needs a scope or
  // logic analyser on the port; there is no other way to see these pins.
  static void pulseTriggers(Print &out, uint32_t half_period_us,
                            uint32_t cycles);

  // Emit one line per second for `seconds`, each carrying the RP2040's own
  // microsecond count. Comparing the drift against the host's wall clock is the
  // only absolute check on the 12 MHz crystal available on this board -- the
  // on-chip frequency counter measures everything *against* that crystal and so
  // cannot see an error in it.
  static void streamClockTicks(Print &out, uint32_t seconds);

  // Round trip a byte pattern out PYC_UART_TX and back in PYC_UART_RX at the
  // pyControl link's parameters. Requires an external loopback: DIO-A shorted
  // to DIO-B on the RJ45. Without one this reports no bytes received, which is
  // the correct answer, not a fault.
  static bool pycUartLoopback(Print &out, uint32_t baud);

  // ADC channels only, for watching a value change.
  static void reportAdcOnce(Print &out);

private:
  static void beginAdc();
};

} // namespace mouse_joystick
