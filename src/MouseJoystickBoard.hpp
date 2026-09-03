// mouse-joystick-pcb board support.
//
// One PCB, one joystick axis, one pyControl UART. There is no axis argument
// anywhere in this firmware and there should never be one -- the twin-axis
// board is a separate design in a separate repository.
//
// Include this and you get the pin map, the safe-state helpers, the RP2040
// self test, and the pyControl UART protocol layer.

#pragma once

#include "BoardPins.hpp"
#include "Conversions.hpp"
#include "PowerSequence.hpp"
#include "Protocol.hpp"
#include "ProtocolBoard.hpp"
#include "SafeState.hpp"
#include "SelfTest.hpp"
#include "TmcRegisters.hpp"
#include "TmcSpi.hpp"
