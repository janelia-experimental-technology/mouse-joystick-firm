// The state this board must be in before it is allowed to do anything else.
//
// From `mouse-joystick-pcb/.agents/AGENTS.md`: on boot, motor rail off, stepper
// outputs disabled, wait for an explicit ARM. This file is the "on boot" half.
// It is deliberately the only place in the firmware that touches HV_EN.

#pragma once

#include <stdint.h>

#include "BoardPins.hpp" // MJ_BOARD_REV, MJ_HAS_LATCH_GPIO

namespace mouse_joystick {

// Reset-state readings of the pins whose idle level is set by a board resistor
// rather than by firmware. Capturing them is only meaningful before anything
// is driven, so `SafeState::apply()` refuses to run until this has been read.
//
// Which pins exist depends on the revision (BoardPins.hpp). Designators are
// v0.3's for rev 0/1 and v1.0's for rev 2.
struct ResetStateReads {
  bool bus_sw_en_n; // expect high: 7.87k pull-up (R601 v0.3, R607 v1.0). The
                    // bus must be isolated.
#if MJ_BOARD_REV < 2
  bool instr_buf_oe_n; // expect high: R1601/R1701/R1801 10k. Port silent.
#else
  bool hv_latch; // v1.0 only. The 36 V switch's LATCH through R503 10k.
                 // Informational: the TPS1HTC100-Q1 is unpowered in stage 1
                 // (36 V absent), so the level is whatever its unpowered pin
                 // and the 10k present, not a resistor's verdict.
  bool dac_sda;  // expect high: R403 4.7k to +3.3V_MCU on the MCU-side tap.
  bool dac_scl;  // expect high: R402 4.7k to +3.3V_MCU on the MCU-side tap.
#endif
  bool pyc_uart_rx; // expect high: 10k pull-up (R403 v0.3, R401 v1.0), and an
                    // idle UART TX also idles high.
  bool hv_fault_n;  // expect high: 10k pull-up (R1905 v0.3, R507 v1.0), and
                    // no latched fault.
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
  //   HV_LATCH        low   (v1.0) auto-retry, the reset default; powerUp()
  //                         raises it once the rail is up
  //   ~{INSTR_BUF_OE} high  (rev 0/1) instrumentation port silent
  //   ~{TMC_CS}       high  driver deselected
  //   ~{TMC_DRV_EN}   high  driver outputs disabled
  //   TQ_CAL_EN       low   (v1.0) torque shunt-calibration switch off
  //   TQ_CONVST_CS    high  (v1.0) torque ADC deselected
  //   FW_TRIG0/1      low   markers idle
  //
  // The driver, torque and brake pins sit behind an open bus switch and reach
  // nothing yet. Setting them now is the point: whenever the bus is later
  // enabled, no chip on the switched domain sees a glitch on any of them.
  // The I2C pair and the two SPI buses stay inputs until the stage that owns
  // them claims them with the rail up.
  static void apply();

  // Instrumentation buffers. On rev 0/1 enabling them makes FW_TRIG0/FW_TRIG1
  // and the tapped driver bus visible on InstrumentPort-40; it cannot affect
  // board behaviour, which is why the port is safe to use during experiments.
  // On v1.0 the Digital-50 port has no enable and is always on: the setter is
  // a no-op and the getter always answers true.
  static void setInstrumentationEnabled(bool enabled);
  static bool instrumentationEnabled();

  // Firmware timing markers. The only board-level outputs stage 1 drives.
  static void setTrigger0(bool level);
  static void setTrigger1(bool level);

private:
  static bool instrumentation_enabled_;
};

} // namespace mouse_joystick
