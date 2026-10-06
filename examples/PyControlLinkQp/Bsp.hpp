// Board support for the QP/C++ link application. Everything hardware-specific
// lives behind this interface: the tick, the markers, the trace transport, the
// protocol line transport and the safe state. `LinkAo.cpp` sees nothing else.

#pragma once

#include <stdint.h>

namespace mouse_joystick {
class BrakeTorqueBackend;
}

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
  // Overrun bookkeeping (2026-10-02): a tick whose work was skipped because
  // the previous handler ended less than 200 us earlier (close=true) or
  // because polling is decimated to every other tick; and the decimation
  // state itself. All three go out in SAMPLE_STAT.
  static void reportTickSkipped(bool decimated);
  static void reportPollMode(bool decimated);
  // MJ_TICK_LEVEL builds: FW_TRIG0 high for the whole TICK handler, so the
  // analyzer shows the handler's duration directly (back-to-back levels are
  // an overrun, a level that never falls a stuck poll). The operation marker
  // is then a trace record only. No-op in other builds.
  static void tickLevel(bool on);

  // Bring-up breadcrumb (2026-10-02): checkpoint `n` as a train of n pulses
  // of 25 us high / 25 us low on FW_TRIG0 (an operation marker pulse is
  // 500 us or longer, so the two cannot be confused), and `n` into the
  // watchdog scratch register, where it survives the watchdog's reset and is
  // reported at the next boot (BOOT_INFO record, banner). Counts: 1-8 the
  // rail-up sequence in TmcMotion::setHvEnabled, 9 the hv ack queued,
  // 11-17 the rev 1 DAC bring-up and release, see Bsp.cpp.
  static void crumb(uint8_t n);
  // Captured once in init(), before any crumb(): why the chip booted ("por",
  // "wdt" for the 2 s watchdog, "reboot" for watchdog_reboot) and the last
  // breadcrumb scratch 0 held. ID reports them as boot= and crumb=.
  static const char *bootReason();
  static uint32_t bootCrumb();

  // The angle output (rev 1): map `angle_deg` from [lo, hi] onto the BNC's
  // 0.050-3.300 V while `valid`, park at the fault level otherwise. The DAC
  // sits on the switched domain, so it is configured after the rail is up
  // (`hv_on`) and forgotten when it goes down. No-op on rev 0.
  static void angleOutput(bool valid, float angle_deg, float lo, float hi,
                          bool hv_on);

  // pcb v1.0's brake sink and torque bridge (rev 2), configured by the same
  // rail-up phase machine as the angle DAC one tick after it and forgotten
  // with the rail; null on earlier revisions. The protocol layer polls it.
  static mouse_joystick::BrakeTorqueBackend *brakeTorque();
  // Its worst poll since the last call and the number of polls (POLL_STAT);
  // zeros before pcb v1.0.
  static uint32_t takeBrakeTorquePollCost(uint32_t *calls, uint32_t *sample_max_us,
                                          uint32_t *isense_max_us);
};
