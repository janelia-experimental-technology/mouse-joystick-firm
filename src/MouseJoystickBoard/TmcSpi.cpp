#include <Arduino.h>
#include <SPI.h>

#include "BoardPins.hpp"
#include "TmcSpi.hpp"

namespace mouse_joystick {
namespace tmc {

namespace {

// Same as TMC51X0's SpiParameters default. Conservative for a first contact,
// and slow enough that a logic analyser at 1 MSa/s still resolves every edge
// comfortably at 10x oversampling.
constexpr uint32_t kClockRateHz = 1000000;

// The TMC5130A wants CS stable around the frame. At 1 MHz the datasheet minima
// are nanoseconds, so single microseconds are generous rather than tight -- and
// they make the frame boundaries obvious in a capture.
constexpr uint32_t kCsSetupUs = 2;
constexpr uint32_t kCsHoldUs = 2;
constexpr uint32_t kInterFrameUs = 10;

SPISettings settings() {
  return SPISettings(kClockRateHz, MSBFIRST, SPI_MODE3);
}

void packRead(uint8_t address, Datagram &out) {
  out.bytes[0] = (address & 0x7F) | ((kRwRead & 0x01) << 7);
  out.bytes[1] = 0;
  out.bytes[2] = 0;
  out.bytes[3] = 0;
  out.bytes[4] = 0;
}

uint32_t unpackData(const Datagram &in) {
  return (static_cast<uint32_t>(in.bytes[1]) << 24) |
         (static_cast<uint32_t>(in.bytes[2]) << 16) |
         (static_cast<uint32_t>(in.bytes[3]) << 8) |
         (static_cast<uint32_t>(in.bytes[4]) << 0);
}

} // namespace

void Spi::begin() {
  // Chip select is plain GPIO, never the PL022's hardware CSn: the RP2040's
  // hardware chip select does not hold CS low across a 40-bit multi-byte
  // datagram, and the TMC5130A requires that it does.
  pinMode(pins::kTmcCsN, OUTPUT);
  digitalWrite(pins::kTmcCsN, HIGH);

  SPI.setSCK(pins::kTmcSpiSck);
  SPI.setTX(pins::kTmcSpiCopi);
  SPI.setRX(pins::kTmcSpiCipo);
  SPI.begin();
}

void Spi::end() {
  SPI.end();
  digitalWrite(pins::kTmcCsN, HIGH);
}

void Spi::transfer(const Datagram &out, Datagram &in) {
  SPI.beginTransaction(settings());
  digitalWrite(pins::kTmcCsN, LOW);
  delayMicroseconds(kCsSetupUs);
  for (size_t i = 0; i < kDatagramSize; ++i) {
    in.bytes[i] = SPI.transfer(out.bytes[i]);
  }
  delayMicroseconds(kCsHoldUs);
  digitalWrite(pins::kTmcCsN, HIGH);
  SPI.endTransaction();
}

ReadResult Spi::read(uint8_t address) {
  ReadResult result;
  result.address = address;

  packRead(address, result.sent_first);
  packRead(address, result.sent_second);

  transfer(result.sent_first, result.got_first);
  delayMicroseconds(kInterFrameUs);
  transfer(result.sent_second, result.got_second);

  result.data = unpackData(result.got_second);
  result.status = result.got_second.bytes[0];

  result.all_zero = true;
  result.all_ones = true;
  for (size_t i = 0; i < kDatagramSize; ++i) {
    const uint8_t a = result.got_first.bytes[i];
    const uint8_t b = result.got_second.bytes[i];
    if (a != 0x00 || b != 0x00) {
      result.all_zero = false;
    }
    if (a != 0xFF || b != 0xFF) {
      result.all_ones = false;
    }
  }
  return result;
}

} // namespace tmc
} // namespace mouse_joystick
