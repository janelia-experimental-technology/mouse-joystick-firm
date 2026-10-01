// The link active object: the stage-4 protocol state machine (ProtocolBoard)
// and a motion backend, unchanged, with a 64-bit clock, lines in and lines
// out. That is exactly what PyControlLink.ino's loop() supplied; here it is
// an event-driven run-to-completion step, and every step leaves a trace
// record saying what it established: a line reached the AO (LINK_RX), the
// protocol state changed (LINK_STATE), an operation started or completed
// (OP_START/OP_DONE, mirrored on FW_TRIG0), a reply was queued (LINK_TX).
//
// The backend is chosen at build time: `TmcMotion` over the TMC5130A, or
// `SimulatedMotion` (MJ_SIM_MOTION=1) for a board with no 36 V, which is how
// this application was first run on the rev 0 fixture. ProtocolBoard does not
// know which it has; neither does this file beyond construction.

#include "qpcpp.hpp"

#include "AppConfig.hpp"
#include "Bsp.hpp"
#include "LinkAo.hpp"

#include <Arduino.h> // micros()
#include <stdlib.h>
#include <string.h>

using namespace QP;

Q_DEFINE_THIS_FILE

namespace {

// The backend first, then the AO that refers to it: within one translation
// unit static objects are constructed in definition order.
#if MJ_SIM_MOTION
mouse_joystick::SimulatedMotion l_motion;
#else
TMC51X0 l_stepper;
mouse_joystick::TmcMotion l_motion(l_stepper, app::motionConfig(),
                                   app::tmcConfig());
#endif

class Link : public QActive {
private:
  QTimeEvt m_tick;
  mouse_joystick::ProtocolBoard m_board;
  const char *m_lastState;
  bool m_moving;   // an operation is in progress
  bool m_home;     // ... and it is a homing
  bool m_lastHome; // the last dispatched verb was "home"

public:
  static Link instance;
  Link();
  static void dictionaries();
  static uint16_t queueMinFree() {
    return static_cast<uint16_t>(instance.m_eQueue.getNMin());
  }

protected:
  Q_STATE_DECL(initial);
  Q_STATE_DECL(running);

private:
  void afterService();
  void recordRx(const LineEvt *e);
};

Link Link::instance;

Link::Link()
    : QActive(Q_STATE_CAST(&Link::initial)), m_tick(this, TICK_SIG, 0U),
      m_board(l_motion, app::boardConfig()), m_lastState(nullptr),
      m_moving(false), m_home(false), m_lastHome(false) {}

void Link::dictionaries() {
#ifdef QS_ON
  QS_OBJ_DICTIONARY(&Link::instance);
  QS_OBJ_DICTIONARY(&Link::instance.m_tick);
  QS_SIG_DICTIONARY(LINE_SIG, &Link::instance);
  QS_SIG_DICTIONARY(TICK_SIG, &Link::instance);
  QS_FUN_DICTIONARY(&Link::running);
  QS_USR_DICTIONARY(LINK_RX);
  QS_USR_DICTIONARY(LINK_TX);
  QS_USR_DICTIONARY(LINK_STATE);
  QS_USR_DICTIONARY(OP_START);
  QS_USR_DICTIONARY(OP_DONE);
  QS_USR_DICTIONARY(LINK_WORK);
#endif
}

// A trace label, parsed loosely: verb and sequence number. ProtocolBoard does
// the real parsing and answers bad frames itself.
void Link::recordRx(const LineEvt *e) {
  char verb[16];
  uint8_t n = 0U;
  const char *p = e->text;
  if (*p == '\r') {
    ++p;
  }
  while ((*p != '\0') && (*p != ';') && (*p != '\n') &&
         (n < sizeof(verb) - 1U)) {
    verb[n++] = *p++;
  }
  verb[n] = '\0';
  m_lastHome = (strcmp(verb, "home") == 0);
  uint16_t seq = 0U;
  if (*p == ';') {
    seq = static_cast<uint16_t>(strtoul(p + 1, nullptr, 10));
  }
  QS_BEGIN_ID(LINK_RX, m_prio)
  QS_U16(0U, seq);
  QS_STR(verb);
  QS_U8(0U, e->len);
  QS_END()
}

// After every feed or clock advance: queue the replies and record what
// changed.
void Link::afterService() {
  for (uint8_t i = 0U; i < m_board.outputCount(); ++i) {
    const char *line = m_board.output(i);
    const uint32_t ready_us = micros();
    const bool sent = BSP::linkWrite(line);
#ifdef QS_ON
    char prefix[21];
    strncpy(prefix, line, sizeof(prefix) - 1U);
    prefix[sizeof(prefix) - 1U] = '\0';
    QS_BEGIN_ID(LINK_TX, m_prio)
    QS_U8(0U, sent ? 1U : 0U);
    QS_U32(0U, ready_us);
    QS_STR(prefix);
    QS_END()
#else
    (void)sent;
#endif
  }
  m_board.clearOutput();

  const char *state = m_board.state();
  if (state != m_lastState) {
    m_lastState = state;
    QS_BEGIN_ID(LINK_STATE, m_prio)
    QS_STR(state);
    QS_END()
  }

  const bool moving = l_motion.isMoving();
  if (moving && !m_moving) {
    m_home = m_lastHome; // a move that starts right after "home" is a homing
    BSP::opMarker(true);
    QS_BEGIN_ID(OP_START, m_prio)
    QS_U8(0U, m_home ? 1U : 0U);
    QS_F32(2U, static_cast<float>(l_motion.angleDeg()));
    QS_END()
  } else if (!moving && m_moving) {
    BSP::opMarker(false);
    QS_BEGIN_ID(OP_DONE, m_prio)
    QS_U8(0U, m_home ? 1U : 0U);
    QS_F32(2U, static_cast<float>(l_motion.angleDeg()));
    QS_END()
  }
  m_moving = moving;
}

Q_STATE_DEF(Link, initial) {
  (void)e;
  dictionaries();
  m_board.setSerial(BSP::boardSerial());
  m_board.advanceTo(BSP::nowUs64());
  m_board.boot(); // READY, once
  afterService();
  m_tick.armX(1U, 1U); // every tick: watchdog and the motion poll
  return tran(&running);
}

Q_STATE_DEF(Link, running) {
  QState status_;
  switch (e->sig) {
  case TICK_SIG: {
    m_board.advanceTo(BSP::nowUs64());
    afterService();
    status_ = Q_RET_HANDLED;
    break;
  }
  case LINE_SIG: {
    const LineEvt *const le = static_cast<const LineEvt *>(e);
    recordRx(le);
    const uint32_t t0 = micros();
    m_board.advanceTo(BSP::nowUs64());
    const uint32_t t1 = micros();
    m_board.feed(le->text);
    const uint32_t t2 = micros();
    afterService();
    const uint32_t t3 = micros();
    QS_BEGIN_ID(LINK_WORK, m_prio)
    QS_U16(0U, static_cast<uint16_t>(t1 - t0)); // clock advance (motion poll)
    QS_U16(0U,
           static_cast<uint16_t>(t2 - t1)); // the line: parse, dispatch, reply
    QS_U16(0U, static_cast<uint16_t>(t3 - t2)); // output handling and records
    QS_END()
    status_ = Q_RET_HANDLED;
    break;
  }
  default: {
    status_ = super(&top);
    break;
  }
  }
  return status_;
}

} // namespace

QActive *const AO_Link = &Link::instance;

void Link_dictionaries() { Link::dictionaries(); }

uint16_t Link_queueMinFree() { return Link::queueMinFree(); }
