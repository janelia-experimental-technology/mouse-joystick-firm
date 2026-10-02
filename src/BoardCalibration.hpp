// Per-board calibration that the protocol or the application needs and the
// board cannot measure for itself, keyed by the RP2040 flash unique id that
// `ID` reports as `serial`. Arduino-free. Every value here cites the bench
// evidence it came from; a board that is not listed gets the defaults, which
// are stated as unmeasured.

#pragma once

#include <string.h>

namespace mouse_joystick {
namespace calibration {

// Which encoder direction is the animal's pull. +1.0: increasing angle. The
// protocol's WATCH classifies PULL on the pull-positive excursion from the
// release angle, so a wrong sign reports a pull as PUSH (uart-protocol.md,
// WATCH).
struct PullSign {
  const char *serial;
  double pull_sign;
};

constexpr PullSign kPullSigns[] = {
    // Board 4, pcb v0.3, the RIGHT joystick: a hand pull from the parked end
    // was classified PUSH with pull_sign +1 on 2026-10-02
    // (qp-arduino-example/evidence/2026-10-02-1245-board4-rail-up/,
    // pilot/c_pilot.txt: ev;PUSH, a_peak=-5.041 from a0=-0.724), so its pull
    // is the decreasing-angle direction.
    {"E4654C159F171E34", -1.0},
};

constexpr double kPullSignUnmeasured = 1.0;

inline double pullSignForSerial(const char *serial) {
  if (serial != nullptr) {
    for (const PullSign &entry : kPullSigns) {
      if (strcmp(entry.serial, serial) == 0) {
        return entry.pull_sign;
      }
    }
  }
  return kPullSignUnmeasured;
}

} // namespace calibration
} // namespace mouse_joystick
