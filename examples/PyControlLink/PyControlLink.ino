// Stage 4 on hardware: the pyControl UART protocol, driving the real motor.
//
// This is the first firmware in this repository that is not a bench console.
// Stages 1 to 3 were menu-driven: type a letter, read a paragraph. This one
// answers a peer, and the peer is `pycontrol/devices/mouse_joystick.py` running
// on a pyControl Breakout 1.2.
//
// WHAT IS WIRED TO WHAT
//
//   Serial1 (UART0, GP0/GP1) -> RJ-45 J1901 -> pyControl port  the protocol
//   Serial  (USB CDC)                                          a mirror, for a
//                                                              person watching
//   ProtocolBoard  framing, state machine, watchdog, retransmission cache
//   TmcMotion      the 36 V rail, SPI, the driver, and real motion
//
// `ProtocolBoard` is byte-for-byte the implementation the shared corpus checks
// on the host (`test/native/test_protocol/`). Nothing about the protocol is
// re-decided here -- this file only supplies a clock, a UART and a motion
// backend. That is the point of the seam: the thing the corpus proved is the
// thing that runs.
//
// !! FIRST RUN: SUPERVISED, ON THE BENCH, WITH A HAND ON THE SUPPLY.
//
//    `TmcMotion` has never been executed. Stage 3 validated the *blocking*
//    motion in `MotionSetup`; this drives the same primitives without waiting,
//    which is a faithful rearrangement and not a measurement. Specifically
//    unproven: the non-blocking homing state machine, and the travel limits
//    below.
//
//    Homing here is SINGLE-STAGE and that is deliberate -- see
//    `TmcMotion::startHome()`. The bench measured two-stage and single-stage
//    repeatability as the same 2-5 encoder counts; the slow creep buys datum
//    accuracy, not precision. Two-stage homing over the protocol is a recorded
//    follow-up.
//
// The board answers `id` before anything is energised and `hv` decides the rail,
// so a host can discover and identify this board with the 36 V supply switched
// off entirely. Start there.

#include <pico/unique_id.h>

#include <MotionSetup.hpp>
#include <MouseJoystickBoard.hpp>
#include <PycUart.hpp>
#include <TMC51X0.hpp>
#include <TmcMotion.hpp>

using mouse_joystick::MonotonicMicros;
using mouse_joystick::MotionSetup;
using mouse_joystick::PowerSequence;
using mouse_joystick::ProtocolBoard;
using mouse_joystick::PycUart;
using mouse_joystick::SafeState;
using mouse_joystick::SelfTest;
using mouse_joystick::TmcMotion;
namespace conv = mouse_joystick::conversions;

constexpr uint32_t kUsbBaudRate = 115200;

// Set false to stop echoing the link to USB. Leave it on at the bench: on a
// board with no LEDs and no test points, this is the only way to watch the
// protocol without a logic analyser.
constexpr bool kMirrorToUsb = true;

TMC51X0 stepper;
MonotonicMicros clock_us;
PycUart link;

// ---------------------------------------------------------------------------
// Mechanism facts. Copied from examples/MotionAndHoming, measured on the rig
// 2026-09-02 on board E4654C159F4D1334: the switch is at the positive end of
// travel, the encoder counts up with XACTUAL, the motor is not reversed.
//
// A correctly built LEFT-joystick harness makes the left station look identical
// from the connector inwards, so these do not change per side. See
// ../mouse-joystick-pcb/docs/harness-pinouts.md.
// ---------------------------------------------------------------------------
mouse_joystick::MotionConfig motionConfig() {
  mouse_joystick::MotionConfig config;
  config.run_current_scale = conv::kCurrentScaleBringUp;  // 8 = 0.211 A RMS
  config.hold_current_scale = conv::kCurrentScaleHold;    // 2 = 0.070 A RMS
  config.motor_reversed = false;
  config.encoder_inverted = false;
  config.switch_active_high = true;
  config.switch_end = mouse_joystick::HomeEnd::kPositive;
  return config;
}

// ---------------------------------------------------------------------------
// Travel limits, and why they are not the corpus's numbers.
//
// The corpus uses +/-20 deg because they divide evenly at its placeholder
// velocity. Those are a TEST FIXTURE, not a property of this mechanism. The
// bench measured ~4000 microsteps (~28 deg at the motor shaft) of travel with
// the switch at the positive end and a hard mechanical stop at the negative
// end.
//
// The datum is the switch, so travel runs negative from zero. Both ends keep a
// jog step of margin: at the positive end because homing parks one clearance
// step off the switch and driving back into it is what the switch is there to
// prevent, and at the negative end because a hard stop is not a sensor and
// hitting it is a stall, not a limit.
//
// !! UNCONFIRMED. Derived from bench geometry, never exercised through the
//    protocol. The first supervised run should walk to each end and check that
//    MOVETO refuses one microstep further, before anything relies on it.
// ---------------------------------------------------------------------------
// Zero is now the PARKED position -- homing zeroes both frames there -- so
// travel runs from 0 down. That is a better zero than the switch edge was: it is
// where homing leaves the joystick, it is commandable, and `ev;HOMED` reports
// `angle=0.0` in agreement with the shared corpus.
//
// The park sits one clearance step off the switch, so going positive from zero
// heads back toward the switch and is simply not allowed. Going negative, the
// mechanism has the remaining travel less a clearance step of margin at the
// hard stop -- a stop is not a sensor and reaching it is a stall.
constexpr float kTravelHiDeg = 0.0f;
constexpr float kTravelLoDeg = conv::microstepsToDegrees(
    -(conv::kMeasuredTravelMicrosteps - 2 * conv::kHomeBackoffMicrosteps));

TmcMotion::Config tmcConfig() { return TmcMotion::Config(); }

ProtocolBoard::Config boardConfig() {
  ProtocolBoard::Config config;
  config.travel_lo = kTravelLoDeg;
  config.travel_hi = kTravelHiDeg;
  // v_max is what LIMITS reports and it must describe what the mechanism does.
  // The bench jogged at 0.100 rev/s; a host that believed the corpus's 180
  // deg/s would time out on every move.
  config.v_max = conv::kJogRevsPerSecond * 360.0f;
  config.a_max = 3600.0;
  config.i_max = 0.421;  // IRUN 17, the conservative reading of the 0.6 A spec
  return config;
}

TmcMotion motion(stepper, motionConfig(), tmcConfig());
ProtocolBoard board(motion, boardConfig());

// The USB mirror, handed over only when it is safe to write to.
//
// A diagnostic must never be able to wedge the protocol. USB CDC writes block
// when the host is connected but not draining -- a terminal scrolled back, a
// script that opened the port and stopped reading -- and a blocked write in
// this loop stalls the link, the motion poll and the host-loss watchdog
// together. So the mirror is dropped rather than waited on: losing a
// diagnostic line is free, losing the link is not.
Print *usbMirror() {
  if (!kMirrorToUsb || !Serial) {
    return nullptr;
  }
  if (Serial.availableForWrite() < 128) {
    return nullptr;
  }
  return &Serial;
}

void setup() {
  // Safe idle FIRST, and read the resistor-defined pins before anything drives
  // them. HV_EN low, bus isolated, driver disabled, instrument port silent.
  const mouse_joystick::ResetStateReads reset_reads = SafeState::readResetState();
  SafeState::apply();
  (void)reset_reads;

  Serial.begin(kUsbBaudRate);
  const uint32_t waited_at = millis();
  while (!Serial && (millis() - waited_at) < 2000) {
    delay(10);
  }

  // The instrument port carries both directions of this link on DIG7/DIG8 and
  // the firmware markers on TRIG0/TRIG1. Enabling it here means a capture
  // started at any point in a session sees the protocol in hardware, which is
  // the only account of the link that does not come from one of its two ends.
  SafeState::setInstrumentationEnabled(true);

  // The board serial, from the flash unique id. This is what `ID` reports and
  // what a task checks before trusting a calibration, so it has to be THIS
  // board's -- not a default. Set before boot(), because READY carries it too.
  static char serial[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1] = {0};
  pico_get_unique_board_id_string(serial, sizeof(serial));
  board.setSerial(serial);

  link.begin();

  Serial.println();
  Serial.println(F("mouse-joystick-pcb -- pyControl protocol link"));
  SelfTest::reportIdentity(Serial);
  Serial.println(F("UART0 on the RJ-45 at 115200 8N1. The 36 V rail is OFF:"));
  Serial.println(F("the host turns it on with `hv;<seq>;on`."));
  Serial.println(F("Mirrored below: `>` is from the host, `<` is from us."));
  Serial.println();

  // READY, once, after boot. The draft: the host should reconnect and re-ID.
  board.advanceTo(clock_us.now());
  board.boot();
  link.service(board, usbMirror());
}

void loop() {
  // READ THE LINK BEFORE JUDGING TIME. The order matters and the original was
  // wrong.
  //
  // The host-loss watchdog is evaluated inside `advanceTo()`. With the link
  // read afterwards, a heartbeat that had already ARRIVED but not yet been
  // parsed did not count, so the board could fault on a ping sitting in the
  // UART's receive buffer. Measured on hardware 2026-09-03: at 40 ms pings
  // against a 100 ms timeout the board faulted `host_timeout` 21 ms after a
  // move completed; at 20 ms pings the same sequence ran clean. That is a
  // margin, not a fix -- the race is closed by reading first.
  //
  // Serviced twice: the first call feeds what arrived and refreshes the
  // watchdog, the second emits whatever advancing time produced, so an event is
  // not held back a whole loop.
  link.service(board, usbMirror());
  board.advanceTo(clock_us.now());
  link.service(board, usbMirror());

  // Deliberately no delay. The host-loss watchdog, the motion poll and the
  // link all run off this loop, and the protocol's whole safety argument is
  // that they are serviced promptly. `TmcMotion::poll()` does one SPI snapshot
  // per pass, which is the cost worth paying.
}
