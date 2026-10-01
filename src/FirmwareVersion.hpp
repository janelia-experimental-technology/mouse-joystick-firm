// What `ID` reports as `fw`. The build defines MJ_FW_VERSION from
// `git describe --tags --always --dirty` (tools/git_version.py, run by
// PlatformIO before every build); a build that did not run it reports
// "unversioned" rather than a plausible number. `hw` comes from
// `BoardPins.hpp`, the single source of truth for the board revision.
// Before 2026-10-01 both were the protocol layer's placeholders, `0.0.0` and
// `rev0`, on every board (review bug 5).

#pragma once

#ifndef MJ_FW_VERSION
#define MJ_FW_VERSION "unversioned"
#endif

namespace mouse_joystick {

constexpr char kFirmwareVersion[] = MJ_FW_VERSION;

} // namespace mouse_joystick
