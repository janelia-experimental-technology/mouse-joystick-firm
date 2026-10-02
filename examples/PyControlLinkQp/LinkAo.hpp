// The link application's interface: signals, the line event, the QS records
// and the active object. Board-independent.

#pragma once

#include "qpcpp.hpp"

enum LinkSignals {
  LINE_SIG = QP::Q_USER_SIG, // one complete protocol line arrived (LineEvt)
  TICK_SIG,                  // periodic: advance the protocol clock
  MAX_SIG
};

// One received line, newline included, NUL terminated. Sized for the
// protocol's 120-byte maximum plus a margin, so an over-length line still
// reaches ProtocolBoard, which answers it with nak;?;badframe.
struct LineEvt : public QP::QEvt {
  uint8_t len; // bytes in text, including the trailing '\n'
  char text[126 + 2];
};

// QS user records of the application (QS_USER + 0..7; the BSP uses +8..).
enum LinkRecords {
  LINK_RX = QP::QS_USER + 0, // line dispatched to the AO: seq, verb, length
  LINK_TX = QP::QS_USER +
            1, // reply/event queued for the port: sent?, ready time, prefix
  LINK_STATE = QP::QS_USER + 2, // ProtocolBoard state changed: name
  OP_START = QP::QS_USER + 3, // operation started (home?, angle); FW_TRIG0 high
  OP_DONE = QP::QS_USER + 4, // operation completed (home?, angle); FW_TRIG0 low
  LINK_WORK =
      QP::QS_USER + 5, // per line: clock-advance, feed, output cost in us
  ENABLE_SETTLE = QP::QS_USER + 6 // after a driver enable: settle us, snap
                                  // microsteps, enable count (TmcMotion)
};

extern QP::QActive *const AO_Link;
void Link_dictionaries();     // QS dictionaries, re-emitted by the BSP
uint16_t Link_queueMinFree(); // the AO's event queue minimum free count
