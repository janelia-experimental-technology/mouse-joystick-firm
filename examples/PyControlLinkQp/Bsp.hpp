// Board support for the QP/C++ link application. Everything hardware-specific
// lives behind this interface: the tick, the markers, the trace transport, the
// protocol line transport and the safe state. `LinkAo.cpp` sees nothing else.

#pragma once

#include <stdint.h>

class BSP {
public:
  enum { TICKS_PER_SEC = 100 };

  // Safe state first, then the ports, the boot signature, the identity beacon
  // and the trace.
  static void init();

  // Application QS dictionaries, re-emitted periodically in the trace profile.
  static void setDictionaryHook(void (*hook)());

  // 64-bit microseconds since boot: `ProtocolBoard`'s clock must not wrap.
  static uint64_t nowUs64();

  // The flash unique id, what `ID` reports.
  static const char *boardSerial();

  // Queue one reply or event line (no terminator) for the pyControl port.
  // Never blocks: the idle loop drains a ring into the port as it has room.
  // Returns false, and counts it, when the ring is full.
  static bool linkWrite(const char *line);

  // "Operation in progress" on FW_TRIG0, visible at the InstrumentPort.
  static void opMarker(bool on);
};
