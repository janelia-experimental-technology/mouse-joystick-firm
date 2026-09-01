// The state this board must be in before it is allowed to do anything else.
//
// From `mouse-joystick-pcb/.agents/AGENTS.md`: on boot, motor rail off, stepper
// outputs disabled, wait for an explicit ARM. This file is the "on boot" half.
// It is deliberately the only place in the firmware that touches HV_EN.

#pragma once

#include <stdint.h>

namespace mouse_joystick {

// Reset-state readings of the four pins whose idle level is set by a board
// resistor rather than by firmware. Capturing them is only meaningful before
// anything is driven, so `SafeState::apply()` refuses to run until this has
// been read.
struct ResetStateReads {
  bool bus_sw_en_n;    // expect high: R601 7.87k. The bus must be isolated.
  bool instr_buf_oe_n; // expect high: R1601/R1701/R1801 10k. Port silent.
  bool pyc_uart_rx;    // expect high: R403 10k, and an idle UART TX also idles
                       // high.
  bool hv_fault_n;     // expect high: R1905 10k, and no latched fault.
  bool valid;
};

class SafeState {
public:
  // Sample the four resistor-defined pins with all internal pulls off, so the
  // value reflects the board and not the pad. Call this first, before anything
  // drives a pin.
  //
  // At RP2040 reset each pad has its internal pull-down enabled (~50-80k).
  // Every pull-up above is 10k or less, so the board wins by a wide margin and
  // these levels hold even in the window before firmware runs.
  static ResetStateReads readResetState();

  // Drive every allocated output to its safe level:
  //
  //   ~{BUS_SW_EN}    high  bus isolated
  //   HV_EN           low   36 V switch off, switched domain at 0 V
  //   HV_DIAG_EN      low   switch diagnostics off, no VS-to-VOUT leakage
  //   ~{INSTR_BUF_OE} high  instrumentation port silent
  //   ~{TMC_CS}       high  driver deselected
  //   ~{TMC_DRV_EN}   high  driver outputs disabled
  //   FW_TRIG0/1      low   markers idle
  //
  // The two driver pins sit behind an open bus switch and reach nothing yet.
  // Setting them now is the point: whenever the bus is later enabled, the
  // driver cannot see a glitch on either.
  static void apply();

  // Instrumentation buffers. Enabling them makes FW_TRIG0/FW_TRIG1 and the
  // tapped driver bus visible on InstrumentPort-40; it cannot affect board
  // behaviour, which is why the port is safe to use during experiments.
  static void setInstrumentationEnabled(bool enabled);
  static bool instrumentationEnabled();

  // Firmware timing markers. The only board-level outputs stage 1 drives.
  static void setTrigger0(bool level);
  static void setTrigger1(bool level);

private:
  static bool instrumentation_enabled_;
};

} // namespace mouse_joystick
