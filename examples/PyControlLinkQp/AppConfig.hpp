// The mechanism and protocol configuration of this board, shared by the real
// and the simulated backend builds. Copied from examples/PyControlLink, which
// explains each number; the facts behind them were measured on the rig
// 2026-09-02 and live in ../mouse-joystick-pcb/docs/bringup-log.md.

#pragma once

#include <BoardCalibration.hpp>
#include <Conversions.hpp>
#include <FirmwareVersion.hpp>
#include <MotionSetup.hpp>
#include <MouseJoystickBoard.hpp> // pins, SafeState, PowerSequence, ProtocolBoard
#include <ProtocolBoard.hpp>

#ifndef MJ_SIM_MOTION
#define MJ_SIM_MOTION 0
#endif

#if !MJ_SIM_MOTION
#include <TmcMotion.hpp>
#endif

namespace app {

namespace conv = mouse_joystick::conversions;

inline mouse_joystick::MotionConfig motionConfig() {
  mouse_joystick::MotionConfig config;
  config.run_current_scale = conv::kCurrentScaleBringUp; // 8 = 0.211 A RMS
  config.hold_current_scale = conv::kCurrentScaleHold;   // 2 = 0.070 A RMS
  config.motor_reversed = false;
  config.encoder_inverted = false;
  config.switch_active_high = true;
  config.switch_end = mouse_joystick::HomeEnd::kPositive;
  return config;
}

#if !MJ_SIM_MOTION
inline mouse_joystick::TmcMotion::Config tmcConfig() {
  return mouse_joystick::TmcMotion::Config();
}
#endif

// Zero is the parked position, one clearance step off the switch at the
// positive end; travel runs negative from there, less a clearance step of
// margin at the hard stop. Unconfirmed through the protocol on the real
// mechanism, as in PyControlLink.
constexpr float kTravelHiDeg = 0.0f;
constexpr float kTravelLoDeg = conv::microstepsToDegrees(
    -(conv::kMeasuredTravelMicrosteps - 2 * conv::kHomeBackoffMicrosteps));

inline mouse_joystick::ProtocolBoard::Config boardConfig() {
  mouse_joystick::ProtocolBoard::Config config;
  config.fw_version = mouse_joystick::kFirmwareVersion; // git describe
  config.hw_rev = mouse_joystick::kHwRevName;           // from MJ_BOARD_REV
  // Protocol v1 capabilities of this revision: rev 1 has the angle DAC; no
  // board yet has the brake or the torque bridge.
  config.has_angout = (MJ_BOARD_REV != 0);
  config.has_brake = false;
  config.has_torque = false;
  // Which encoder direction is "pull": per board, from BoardCalibration.hpp,
  // applied by the application once the serial is known (board 4 is -1,
  // measured 2026-10-02). This default is the unmeasured one.
  config.pull_sign = mouse_joystick::calibration::kPullSignUnmeasured;
#if MJ_SIM_MOTION
  // The simulator's placeholder limits, so expected durations are the
  // corpus's: 9 deg at 180 deg/s is 50,000 us, homing is 50,000 us.
#else
  config.travel_lo = kTravelLoDeg;
  config.travel_hi = kTravelHiDeg;
  config.v_max = conv::kJogRevsPerSecond * 360.0f;
  config.a_max = 3600.0;
  config.i_max = 0.421; // IRUN 17, the conservative reading of the 0.6 A spec
#endif
  return config;
}

} // namespace app
