// DAC80501 / DAC70501 / DAC60501 driver, SPI or I2C -- Arduino transports.
//
// Reusable across boards on purpose. mouse-joystick-pcb places this part twice
// at v1.0 -- once as the angle output in `analog_output_0_3v3_dac80501` and once
// as the brake current setpoint in `brake_current_sink_0_49ma_dac80501` -- on
// different chip selects and in different roles. Nothing board-specific belongs
// in here.
//
// In particular these stay OUT: the 1.32 output-stage gain, the ~25 mV output
// floor, true 0 V reserved as an out-of-band unhomed/faulted level, the
// per-board gain and offset fit, and anything that mentions an angle. Those are
// properties of a board's analog chain, not of the converter.
//
// Three headers, split by what they need:
//
//   DAC80501Registers.hpp  register map and output arithmetic. Arduino-free,
//                          host tested. READ ITS HEADER COMMENT FIRST -- it
//                          explains why this part cannot be read back over SPI
//                          and why GAIN must be written before any DAC data.
//   DAC80501Device.hpp     the Transport interface and the device class,
//                          including the ordering interlock. Arduino-free and
//                          host tested against a fake transport.
//   DAC80501.hpp           this file: the two Arduino transports, and a helper
//                          that fills in the injected delay.
//
// ---------------------------------------------------------------------------
// WHY BOTH TRANSPORTS
// ---------------------------------------------------------------------------
// The interface is chosen by the SPI2C strap pin at power-up, so it is a board
// decision, and the two are not equivalent:
//
//   SPI  50 MHz, three wires, and NO READBACK WHATSOEVER.
//   I2C  1 MHz in fast-mode-plus, two wires plus pull-ups, and full readback of
//        DEVID, GAIN and STATUS.REF-ALARM.
//
// Bandwidth does not usually decide it. A 24-bit SPI frame at even 1 MHz is
// 24 us and an I2C update is 36 clocks, so both clear a kHz-scale update rate
// comfortably. What decides it is whether firmware needs to see the reference
// alarm -- because a reference alarm drives the output to 0 V while leaving the
// DAC codes intact, which is indistinguishable from a legitimate zero unless
// STATUS can be read.
//
// Build either. `verifyConfiguration()` reports honestly which one you have
// rather than pretending an SPI build verified anything.

#pragma once

// Both bus headers, not just the one a given board uses. `SPIClass` and
// `TwoWire` are declared in SPI.h and Wire.h rather than Arduino.h, and this
// header names both transports, so it cannot compile without them. Pulling in
// the unused one costs a header parse and keeps this file usable standalone.
//
// `lib_ldf_mode = off` in platformio.ini means nothing is discovered by
// scanning includes, so BOTH `SPI` and `Wire` have to be named in lib_deps for
// this to resolve. The symptom otherwise is `Wire.h: No such file or directory`
// out of a library that plainly includes it.
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <stdint.h>

#include "DAC80501Device.hpp"
#include "DAC80501Registers.hpp"

namespace dac80501 {

// ---------------------------------------------------------------------------
// SPI transport
// ---------------------------------------------------------------------------
// Three wires: SYNC (chip select, active low), SCLK, SDIN. The device samples
// SDIN on each FALLING edge of SCLK and latches the shift register into the
// addressed register on the RISING edge of SYNC.
//
// SPI mode: the datasheet specifies the sampling edge, not a mode number, and a
// falling-edge sample is satisfied by both mode 1 (CPOL 0, CPHA 1) and mode 2
// (CPOL 1, CPHA 0). This uses MODE1, which is the conventional choice for the
// family and keeps SCLK idling low. If a bench capture ever shows the first bit
// mistimed, mode 2 is the other candidate -- it is a two-character change, and
// the symptom would be every written value shifted by one bit.
//
// SYNC also acts as an abort: raising it before the 24th falling edge discards
// the frame and updates nothing. That is a useful property rather than a hazard,
// but it does mean the chip select must be held for the whole transfer, so this
// drives it manually rather than relying on any hardware CS.
class SpiTransport : public Transport {
public:
  // `sync_pin` is the DAC's SYNC pin -- this driver's chip select. `spi` is the
  // bus, already given its pins by the caller: which SPI instance and which
  // pads it uses is a board fact, not a converter fact.
  SpiTransport(SPIClass &spi, uint8_t sync_pin, uint32_t clock_hz = 8000000u);

  // Idles SYNC high before anything can drive SCLK, so a partially clocked
  // frame cannot be latched by a stray edge during start-up.
  void begin();

  bool writeRegister(uint8_t reg, uint16_t value) override;

  // Always false, and not because of anything this code does. See the header
  // comment in DAC80501Registers.hpp.
  bool readRegister(uint8_t reg, uint16_t &out) override;
  bool canRead() const override { return false; }

private:
  SPIClass &spi_;
  uint8_t sync_pin_;
  uint32_t clock_hz_;
};

// ---------------------------------------------------------------------------
// I2C transport
// ---------------------------------------------------------------------------
// Address byte, command byte, MSDB, LSDB -- 36 clocks for an update. Readback is
// a write of the command byte followed by a repeated START and two bytes in.
class I2cTransport : public Transport {
public:
  // `address` is the 7-bit address, which the A0 pin selects; use the
  // kI2cAddrA0* constants rather than a literal.
  I2cTransport(TwoWire &wire, uint8_t address = kI2cAddrA0Agnd);

  void begin();

  bool writeRegister(uint8_t reg, uint16_t value) override;
  bool readRegister(uint8_t reg, uint16_t &out) override;
  bool canRead() const override { return true; }

private:
  TwoWire &wire_;
  uint8_t address_;
};

// A Config with the injected delay already pointing at Arduino's `delay`.
// Config defaults `delay_ms` to null so the device class stays Arduino-free,
// but on hardware the post-reset settle is wanted, and leaving each caller to
// remember that is how it gets forgotten.
inline Config arduinoConfig() {
  Config config;
  config.delay_ms = [](uint32_t ms) { delay(ms); };
  return config;
}

} // namespace dac80501
