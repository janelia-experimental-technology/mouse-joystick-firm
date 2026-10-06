// ADS8689 driver -- the Arduino SPI transport.
//
// Reusable across boards on purpose: nothing in here knows about a torque
// bridge, an excitation voltage or a divider. Those belong to the consuming
// firmware (`Conversions.hpp` on mouse-joystick-pcb).
//
// Three headers, split by what they need:
//
//   ADS8689Registers.hpp  register map, command framing and the transfer
//                         function. Arduino-free, host tested. READ ITS HEADER
//                         COMMENT FIRST: it explains the two-frame register
//                         read and the conversion every frame starts.
//   ADS8689Device.hpp     the Transport interface and the device class, with
//                         the range-before-trust ordering. Arduino-free, host
//                         tested against a fake transport.
//   ADS8689.hpp           this file: the SPI transport over an SPIClass and a
//                         plain-GPIO CONVST/CS pin.
//
// CONVST/CS is plain GPIO, never hardware chip select. The RP2040's PL022
// cannot hold CS low across a 32-bit frame delivered as four byte transfers,
// and the rising edge of this pin is the sampling instant, so when it happens
// is the measurement.

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>

#include "ADS8689Device.hpp"
#include "ADS8689Registers.hpp"

namespace ads8689 {

class SpiTransport : public Transport {
public:
  // `spi` is the bus, already given its pins by the caller -- which SPI
  // instance and which pads is a board fact. `convst_cs_pin` is the device's
  // CONVST/CS. `clock_hz` defaults to 2 MHz: the part takes 66 MHz, but a
  // logic analyser at 10 MSa/s decodes 2 MHz cleanly and a 32-bit frame is
  // still only 16 us. `conversion_wait_us` is the pause after every rising
  // edge; the datasheet maximum is 5 us and RVS is not wired to the MCU.
  SpiTransport(SPIClass &spi, uint8_t convst_cs_pin,
               uint32_t clock_hz = 2000000u, uint32_t conversion_wait_us = 6u);

  // Make CONVST/CS an output idling HIGH with no intermediate low: a falling
  // edge opens a frame and a rising edge starts a conversion, and neither is
  // wanted before the caller asks. Does not touch the SPI peripheral; the
  // caller calls `spi.begin()` after assigning pins.
  void begin();

  uint32_t transferFrame(uint32_t out) override;
  void pulseConvst() override;

  uint32_t clockHz() const { return clock_hz_; }

private:
  SPIClass &spi_;
  uint8_t convst_cs_pin_;
  uint32_t clock_hz_;
  uint32_t conversion_wait_us_;
};

} // namespace ads8689
