#include <SPI.h>
#include <Wire.h>

#include "DAC80501.hpp"

namespace dac80501 {

// ---------------------------------------------------------------------------
// SpiTransport
// ---------------------------------------------------------------------------

SpiTransport::SpiTransport(SPIClass &spi, uint8_t sync_pin, uint32_t clock_hz)
    : spi_(spi), sync_pin_(sync_pin), clock_hz_(clock_hz) {}

void SpiTransport::begin() {
  // SYNC high before it is an output, so the pin never presents a low -- a
  // falling edge is what starts a frame, and one during start-up would begin a
  // transfer nobody asked for.
  digitalWrite(sync_pin_, HIGH);
  pinMode(sync_pin_, OUTPUT);
  digitalWrite(sync_pin_, HIGH);
}

bool SpiTransport::writeRegister(uint8_t reg, uint16_t value) {
  // MODE1: SCLK idles low and the device samples SDIN on the falling edge. See
  // the mode discussion in DAC80501.hpp.
  spi_.beginTransaction(SPISettings(clock_hz_, MSBFIRST, SPI_MODE1));
  digitalWrite(sync_pin_, LOW);
  spi_.transfer(reg);
  spi_.transfer(static_cast<uint8_t>(value >> 8));
  spi_.transfer(static_cast<uint8_t>(value & 0xFFu));
  // The rising edge is what latches the shift register into the addressed
  // register, so it has to happen before the transaction ends.
  digitalWrite(sync_pin_, HIGH);
  spi_.endTransaction();

  // No acknowledgement exists to check. A write either happened or the bus is
  // broken in a way this transport cannot see, which is exactly why
  // verifyConfiguration() reports kNotSupported here.
  return true;
}

bool SpiTransport::readRegister(uint8_t reg, uint16_t &out) {
  (void)reg;
  (void)out;
  // Not "unimplemented" -- impossible. The part has no SDO pin in SPI mode.
  return false;
}

// ---------------------------------------------------------------------------
// I2cTransport
// ---------------------------------------------------------------------------

I2cTransport::I2cTransport(TwoWire &wire, uint8_t address)
    : wire_(wire), address_(address) {}

void I2cTransport::begin() { wire_.begin(); }

bool I2cTransport::writeRegister(uint8_t reg, uint16_t value) {
  wire_.beginTransmission(address_);
  wire_.write(reg);
  wire_.write(static_cast<uint8_t>(value >> 8));
  wire_.write(static_cast<uint8_t>(value & 0xFFu));
  // endTransmission returns 0 only if every byte was acknowledged, so unlike
  // the SPI path this genuinely confirms the device is present and listening.
  return wire_.endTransmission() == 0;
}

bool I2cTransport::readRegister(uint8_t reg, uint16_t &out) {
  // Point at the register, then read it back. `false` keeps the bus for a
  // repeated START rather than releasing it, which is what the datasheet's read
  // sequence expects.
  wire_.beginTransmission(address_);
  wire_.write(reg);
  if (wire_.endTransmission(false) != 0) {
    return false;
  }
  if (wire_.requestFrom(address_, static_cast<uint8_t>(2)) != 2) {
    return false;
  }
  const uint16_t msb = static_cast<uint16_t>(wire_.read());
  const uint16_t lsb = static_cast<uint16_t>(wire_.read());
  out = static_cast<uint16_t>((msb << 8) | lsb);
  return true;
}

} // namespace dac80501
