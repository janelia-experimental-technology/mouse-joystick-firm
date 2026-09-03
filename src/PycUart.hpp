// The pyControl link: UART0 on the RJ-45, plus the clock the protocol needs.
//
// FIRMWARE ONLY -- needs Arduino. The protocol layer stays free of it, which is
// what keeps the shared corpus runnable on the host.

#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "BoardPins.hpp"
#include "ProtocolBoard.hpp"

namespace mouse_joystick {

// 64-bit microseconds accumulated from a 32-bit `micros()`.
//
// This is not defensive padding. `micros()` wraps every 71.6 minutes, which is
// shorter than a session, and every event on this link carries `t_us` into a
// recorded dataset used for alignment against the cameras and the OneBox. A
// wrap would not fail loudly -- it would produce timestamps that go backwards
// in the middle of an experiment, and the first sign of it would be an
// alignment that quietly stopped working.
//
// Must be sampled more often than once per wrap. Any `loop()` satisfies that by
// a margin of several orders of magnitude; a firmware that blocks for over an
// hour has a different problem.
class MonotonicMicros {
public:
  uint64_t now() {
    const uint32_t raw = micros();
    if (raw < last_raw_) {
      high_ += 0x100000000ull;
    }
    last_raw_ = raw;
    return high_ + raw;
  }

private:
  uint32_t last_raw_ = 0;
  uint64_t high_ = 0;
};

class PycUart {
public:
  // 115200 8N1, newline-terminated: the lab's existing pyControl peripheral
  // idiom, which is the whole reason this protocol looks the way it does.
  static constexpr uint32_t kBaudRate = 115200;

  void begin(uint32_t baud = kBaudRate) {
    // `Serial1` is UART0 on this core. Never `Serial2`: its defaults land on
    // GPIO 8/9, which here are `TMC_DIAG1_` and `~{BUS_SW_EN}`.
    Serial1.end();
    Serial1.setTX(pins::kPycUartTx);
    Serial1.setRX(pins::kPycUartRx);
    Serial1.begin(baud);
  }

  // Pump the link once. Feed whatever arrived, then write whatever the board
  // produced. Call every loop.
  //
  // `mirror` optionally echoes both directions to USB CDC, which on this board
  // is the only way a person at the bench can watch the link without a logic
  // analyser on the InstrumentPort. It is a diagnostic, not a second protocol
  // channel: nothing reads it back.
  void service(ProtocolBoard &board, Print *mirror = nullptr) {
    size_t count = 0;
    while (Serial1.available() > 0 && count < sizeof(rx_) - 1) {
      const int byte = Serial1.read();
      if (byte < 0) {
        break;
      }
      rx_[count++] = static_cast<char>(byte);
    }
    if (count > 0) {
      rx_[count] = '\0';
      if (mirror != nullptr) {
        mirror->print(rx_);
      }
      board.feed(rx_);
    }

    for (uint8_t i = 0; i < board.outputCount(); ++i) {
      Serial1.print(board.output(i));
      Serial1.write('\n');
      if (mirror != nullptr) {
        mirror->print(F("< "));
        mirror->println(board.output(i));
      }
    }
    if (board.outputOverflowed() && mirror != nullptr) {
      // Worth shouting about: a dropped reply leaves the host waiting for an
      // ack that will never arrive, and it means the loop is not draining
      // often enough or something produced more events than the queue holds.
      mirror->println(F("!! output queue overflowed -- a reply was dropped"));
    }
    board.clearOutput();
  }

private:
  // One loop's worth of bytes. The board buffers partial lines itself, so this
  // only has to be big enough to keep up, not big enough for a whole line.
  char rx_[64] = {0};
};

}  // namespace mouse_joystick
