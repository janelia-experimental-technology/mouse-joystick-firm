// Minimal raw SPI access to the TMC5130A, for proving the bus.
//
// This is deliberately NOT the TMC51X0 driver library. Stage 2 asks one
// question -- does this board's SPI path reach the driver chip -- and answering
// it with a 40-bit datagram nobody has to trust is worth more than answering it
// with a library that would then be a second unknown. Bringing TMC51X0 up on
// this board is its own step, and it starts from a bus already known to work.
//
// Byte order matches TMC51X0's SpiProtocol.hpp exactly:
//
//   [0] address | (rw << 7)
//   [1] data[31:24]
//   [2] data[23:16]
//   [3] data[15:8]
//   [4] data[7:0]
//
// SPI mode 3, MSB first, 1 MHz -- the same settings and default clock rate
// TMC51X0 uses.
//
// Reads take two transfers because the chip is a shift register: each datagram
// returns the status byte plus the data of the register requested in the
// *previous* datagram. That property is useful rather than annoying. Sending a
// correct address requires MOSI, clocking it requires SCK, framing it requires
// CS, and receiving the answer requires MISO -- so a successful read exercises
// the entire bus, and stage 2 needs no writes at all.

#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

#include "TmcRegisters.hpp"

namespace mouse_joystick {
namespace tmc {

constexpr size_t kDatagramSize = 5;

struct Datagram {
  uint8_t bytes[kDatagramSize];
};

struct ReadResult {
  uint8_t address;
  uint32_t data;
  uint8_t status;      // SPI_STATUS byte from the second transfer
  Datagram sent_first; // all four datagrams are kept so the serial log can be
  Datagram got_first;  // compared byte for byte against a logic analyser
  Datagram sent_second;
  Datagram got_second;
  bool all_zero; // every received byte 0x00 across both transfers
  bool all_ones; // every received byte 0xFF across both transfers
};

class Spi {
public:
  // Configure SPI0 on this board's pins. Does not touch the bus switch, and
  // never touches ~{TMC_DRV_EN}: stage 2 does not enable the driver.
  static void begin();
  static void end();

  // Two transfers, returning the second one's data.
  static ReadResult read(uint8_t address);

private:
  static void transfer(const Datagram &out, Datagram &in);
};

} // namespace tmc
} // namespace mouse_joystick
