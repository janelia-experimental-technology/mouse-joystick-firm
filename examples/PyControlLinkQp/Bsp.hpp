// Board support for the QP/C++ link application. Everything hardware-specific
// lives behind this interface: the tick, the markers, the trace transport, the
// protocol line transport and the safe state. `LinkAo.cpp` sees nothing else.

#pragma once

#include <stdint.h>

class BSP {
public:
  // 1 kHz: the sampler rate. TICK_SIG is posted by the alarm interrupt,
  // coalesced to one in flight; the AO acknowledges each with tickHandled().
  enum { TICKS_PER_SEC = 1000 };

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

  // The InstrumentPort buffers, driven by the protocol's INSTR verb. Off also
  // silences the trace, which leaves on FW_TRIG1 through the same buffers.
  static void instrumentation(bool on);
  static bool instrumentationEnabled();

  // The 1 kHz sampler's bookkeeping: the AO acknowledges a tick so the next
  // one may be posted, and reports what the tick cost (SAMPLE_STAT).
  static void tickHandled();
  static void reportTickCost(uint32_t us);

  // The angle output (rev 1): map `angle_deg` from [lo, hi] onto the BNC's
  // 0.050-3.300 V while `valid`, park at the fault level otherwise. The DAC
  // sits on the switched domain, so it is configured after the rail is up
  // (`hv_on`) and forgotten when it goes down. No-op on rev 0.
  static void angleOutput(bool valid, float angle_deg, float lo, float hi,
                          bool hv_on);
};
