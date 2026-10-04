#include <Arduino.h>

#include "BoardPins.hpp"
#include "SafeState.hpp"

namespace mouse_joystick {

namespace {

// Read a pin with no internal pull, so the level comes from the board.
bool readBare(uint8_t pin) {
  pinMode(pin, INPUT);
  return digitalRead(pin) == HIGH;
}

void driveOutput(uint8_t pin, bool level) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, level ? HIGH : LOW);
}

} // namespace

bool SafeState::instrumentation_enabled_ = false;

ResetStateReads SafeState::readResetState() {
  ResetStateReads reads;
  reads.bus_sw_en_n = readBare(pins::kBusSwEnN);
#if MJ_BOARD_REV < 2
  reads.instr_buf_oe_n = readBare(pins::kInstrBufOeN);
#else
  reads.hv_latch = readBare(pins::kHvLatch);
  reads.dac_sda = readBare(pins::kDacSda);
  reads.dac_scl = readBare(pins::kDacScl);
#endif
  reads.pyc_uart_rx = readBare(pins::kPycUartRx);
  reads.hv_fault_n = readBare(pins::kHvFaultN);
  reads.valid = true;
  return reads;
}

void SafeState::apply() {
  // Isolation first. The 7.87k pull-up already holds this high, but an
  // explicit drive means the level is ours rather than a pull-up's, and it
  // stays high if a later stage reconfigures the pad.
  driveOutput(pins::kBusSwEnN, HIGH);

  // 36 V switch off and its diagnostics off. Order matters only in that
  // HV_DIAG_EN must not be left floating high while HV_EN is low.
  driveOutput(pins::kHvEn, LOW);
  driveOutput(pins::kHvDiagEn, LOW);
#if MJ_HAS_LATCH_GPIO
  // LATCH low is the switch's auto-retry configuration: the reset default, and
  // the only one in which the rail can charge this board's bulk capacitance.
  // PowerSequence::powerUp() raises it once the rail is established so that a
  // real fault latches off; with HV_EN low the level has no effect either way.
  driveOutput(pins::kHvLatch, LOW);
#endif

#if MJ_BOARD_REV < 2
  // Instrumentation port silent.
  driveOutput(pins::kInstrBufOeN, HIGH);
  instrumentation_enabled_ = false;
#else
  // The Digital-50 port has no enable; its buffers are on from power-up.
  instrumentation_enabled_ = true;
#endif

  // Driver deselected and disabled behind the open bus switch.
  driveOutput(pins::kTmcCsN, HIGH);
  driveOutput(pins::kTmcDrvEnN, HIGH);

#if MJ_BOARD_REV >= 2
  // Torque and brake blocks, behind the same open switch, same reasoning: the
  // calibration shunt stays out of the bridge and the ADC stays deselected
  // from the instant the bus closes. ~{BRAKE_FAULT} is the brake block's
  // output and must never be driven from this side.
  driveOutput(pins::kTqCalEn, LOW);
  driveOutput(pins::kTqConvstCsN, HIGH);
  pinMode(pins::kBrakeFaultN, INPUT);
#endif

  // Timing markers idle low.
  driveOutput(pins::kFwTrig0, LOW);
  driveOutput(pins::kFwTrig1, LOW);

  // SPI0 and the driver diagnostics stay inputs. Stage 2 configures SPI0 when
  // it is ready to talk to the TMC5130A; leaving SCK and COPI undriven until
  // then keeps the B side of the bus switch quiet.
  pinMode(pins::kTmcSpiSck, INPUT);
  pinMode(pins::kTmcSpiCopi, INPUT);
  pinMode(pins::kTmcSpiCipo, INPUT);
  pinMode(pins::kTmcDiag0, INPUT);
  pinMode(pins::kTmcDiag1, INPUT);

#if MJ_BOARD_REV >= 2
  // Likewise the torque ADC's SPI1 and the DAC I2C pair: inputs until the
  // stage that owns them claims them with the rail up. R402/R403 hold the I2C
  // pair high meanwhile.
  pinMode(pins::kTqSpiSck, INPUT);
  pinMode(pins::kTqSpiCopi, INPUT);
  pinMode(pins::kTqSpiCipo, INPUT);
  pinMode(pins::kDacSda, INPUT);
  pinMode(pins::kDacScl, INPUT);
#endif
}

void SafeState::setInstrumentationEnabled(bool enabled) {
#if MJ_BOARD_REV < 2
  driveOutput(pins::kInstrBufOeN, enabled ? LOW : HIGH);
  instrumentation_enabled_ = enabled;
#else
  (void)enabled; // no enable pin on InstrumentPort-Digital-50
  instrumentation_enabled_ = true;
#endif
}

bool SafeState::instrumentationEnabled() { return instrumentation_enabled_; }

void SafeState::setTrigger0(bool level) {
  digitalWrite(pins::kFwTrig0, level ? HIGH : LOW);
}

void SafeState::setTrigger1(bool level) {
  digitalWrite(pins::kFwTrig1, level ? HIGH : LOW);
}

} // namespace mouse_joystick
