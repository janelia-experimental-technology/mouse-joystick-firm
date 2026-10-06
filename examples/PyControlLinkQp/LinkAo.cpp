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
#include <BoardCalibration.hpp>

#include <Arduino.h> // micros()
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
  static constexpr uint32_t kIdleGapUs = 200U; // idle must have had this much
  static constexpr uint32_t kTickBudgetUs = 400U; // the declared sampler budget
  mouse_joystick::ProtocolBoard m_board;
  uint32_t m_lastTickEnd_us = 0U;
  bool m_tickEndValid = false;
  bool m_pollDecimated = false;
  bool m_tickParity = false;
  uint8_t m_overBudgetStreak = 0U;
  uint32_t m_enableCount = 0U;
  const char *m_lastState;
  bool m_brakeEngaging = false;
  float m_brakeLevel = 0.0f;
  uint32_t m_lastPollStat_ms = 0U;
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
#if MJ_SIM_MOTION
  void simStimulus(const char *rest);
#endif
  void recordRx(const LineEvt *e);
};

Link Link::instance;

Link::Link()
    : QActive(Q_STATE_CAST(&Link::initial)),
      m_board(l_motion, app::boardConfig(), BSP::brakeTorque()),
      m_lastState(nullptr),
      m_moving(false), m_home(false), m_lastHome(false) {}

void Link::dictionaries() {
#ifdef QS_ON
  QS_OBJ_DICTIONARY(&Link::instance);
  QS_SIG_DICTIONARY(LINE_SIG, &Link::instance);
  QS_SIG_DICTIONARY(TICK_SIG, &Link::instance);
  QS_USR_DICTIONARY(ENABLE_SETTLE);
  QS_FUN_DICTIONARY(&Link::running);
  QS_USR_DICTIONARY(LINK_RX);
  QS_USR_DICTIONARY(LINK_TX);
  QS_USR_DICTIONARY(LINK_STATE);
  QS_USR_DICTIONARY(OP_START);
  QS_USR_DICTIONARY(OP_DONE);
  QS_USR_DICTIONARY(LINK_WORK);
  QS_USR_DICTIONARY(BRAKE_STATE);
  QS_USR_DICTIONARY(POLL_STAT);
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
  if (*p == ';') { // the digits that follow, modulo 2^16; no newlib here
    uint32_t v = 0U;
    for (const char *d = p + 1; (*d >= '0') && (*d <= '9'); ++d) {
      v = (v * 10U + static_cast<uint32_t>(*d - '0')) % 65536U;
    }
    seq = static_cast<uint16_t>(v);
  }
  QS_BEGIN_ID(LINK_RX, m_prio)
  QS_U16(0U, seq);
  QS_STR(verb);
  QS_U8(0U, e->len);
  QS_END()
}

#if MJ_SIM_MOTION
// `<seq>;angle;<deg>`: answer ack;<seq> or nak;<seq>;badargs on the port.
void Link::simStimulus(const char *rest) {
  namespace protocol = mouse_joystick::protocol;
  char seq_text[8];
  size_t n = 0U;
  while ((rest[n] >= '0') && (rest[n] <= '9') && (n + 1U < sizeof(seq_text))) {
    seq_text[n] = rest[n];
    ++n;
  }
  seq_text[n] = '\0';
  uint16_t seq = 0U;
  const bool seq_ok = protocol::normalizeSeq(seq_text, &seq);
  // The event text keeps its terminator; parseNumber, like the protocol
  // parser after feed() has split the line, wants the bare field (the first
  // rerun of 2026-10-01 answered badargs to every stimulus for this reason).
  char number[16];
  size_t m = 0U;
  const char *num = rest + n + 7;
  while ((num[m] != '\0') && (num[m] != '\n') && (num[m] != '\r') &&
         (m + 1U < sizeof(number))) {
    number[m] = num[m];
    ++m;
  }
  number[m] = '\0';
  protocol::Line reply;
  double deg = 0.0;
  if (seq_ok && strncmp(rest + n, ";angle;", 7U) == 0 &&
      protocol::parseNumber(number, &deg)) {
    m_board.advanceTo(BSP::nowUs64());
    l_motion.setEncoderAngleDeg(deg);
    m_board.advanceTo(BSP::nowUs64()); // the classifier sees it now
    reply.beginAck(seq);
  } else {
    reply.nak(seq, protocol::kNakBadArgs);
  }
  static_cast<void>(BSP::linkWrite(reply.c_str()));
}
#endif

// After every feed or clock advance: queue the replies and record what
// changed.
void Link::afterService() {
  // A FETCH chunk is 32 `cap;k;...` lines and `cap;end` queued at once: one
  // LINK_TX record per line saturated the 115200 baud trace (2026-10-01 run),
  // so the chunk's cap lines share one record, `cap;<first>-<last>` plus
  // `;end`, with the sent flag false if any of them was dropped.
  uint16_t cap_lines = 0U;
  bool cap_sent = true;
  bool cap_end = false;
  uint32_t cap_ready_us = 0U;
  char cap_first[12] = "";
  char cap_last[12] = "";
  for (uint8_t i = 0U; i < m_board.outputCount(); ++i) {
    const char *line = m_board.output(i);
    const uint32_t ready_us = micros();
    const bool sent = BSP::linkWrite(line);
#ifdef QS_ON
    if (strncmp(line, "cap;", 4U) == 0) {
      if (cap_lines == 0U) {
        cap_ready_us = ready_us;
      }
      ++cap_lines;
      cap_sent = cap_sent && sent;
      if (strcmp(line + 4, "end") == 0) {
        cap_end = true;
      } else {
        size_t k = 0U;
        while ((line[4U + k] >= '0') && (line[4U + k] <= '9') &&
               (k + 1U < sizeof(cap_last))) {
          cap_last[k] = line[4U + k];
          ++k;
        }
        cap_last[k] = '\0';
        if (cap_first[0] == '\0') {
          strcpy(cap_first, cap_last);
        }
      }
      continue;
    }
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
#ifdef QS_ON
  if (cap_lines != 0U) {
    char prefix[32];
    size_t n = 0U;
    const char *parts[] = {"cap;", cap_first, "-", cap_last,
                           cap_end ? ";end" : ""};
    for (const char *part : parts) {
      for (const char *c = part; (*c != '\0') && (n + 1U < sizeof(prefix));
           ++c) {
        prefix[n++] = *c;
      }
    }
    prefix[n] = '\0';
    QS_BEGIN_ID(LINK_TX, m_prio)
    QS_U8(0U, cap_sent ? 1U : 0U);
    QS_U32(0U, cap_ready_us);
    QS_STR(prefix);
    QS_END()
  }
#endif
  m_board.clearOutput();

  // `instr on/off` drives the InstrumentPort buffers (review bug 3).
  if (m_board.instrumentationOn() != BSP::instrumentationEnabled()) {
    BSP::instrumentation(m_board.instrumentationOn());
  }

  const char *state = m_board.state();
  if (state != m_lastState) {
    m_lastState = state;
    QS_BEGIN_ID(LINK_STATE, m_prio)
    QS_STR(state);
    QS_END()
  }

  // Where the tick's time goes, once a second: the two backends' worst polls.
  if ((millis() - m_lastPollStat_ms) >= 1000U) {
    m_lastPollStat_ms = millis();
    uint32_t motion_calls = 0U;
    uint32_t motion_enc_max = 0U;
    uint32_t motion_snap_max = 0U;
    uint32_t bt_calls = 0U;
    uint32_t bt_sample_max = 0U;
    uint32_t bt_isense_max = 0U;
#if MJ_SIM_MOTION
    const uint32_t motion_max = 0U;
#else
    const uint32_t motion_max =
        l_motion.takePollCostMaxUs(&motion_calls, &motion_enc_max, &motion_snap_max);
#endif
    const uint32_t bt_max =
        BSP::takeBrakeTorquePollCost(&bt_calls, &bt_sample_max, &bt_isense_max);
    QS_BEGIN_ID(POLL_STAT, m_prio)
    QS_U32(0U, motion_max);
    QS_U32(0U, motion_calls);
    QS_U32(0U, motion_enc_max);  // the X_ENC read (every poll)
    QS_U32(0U, motion_snap_max); // the five-register snapshot (every tenth)
    QS_U32(0U, bt_max);
    QS_U32(0U, bt_calls);
    QS_U32(0U, bt_sample_max); // the ADS8689 conversion
    QS_U32(0U, bt_isense_max); // the BRAKE_ISENSE analogRead
    QS_END()
  }

  // The brake (pcb v1.0): engaging, or the commanded level, changed.
  const bool engaging = m_board.brakeEngaging();
  const float level = static_cast<float>(m_board.brakeLevel());
  if (engaging != m_brakeEngaging || level != m_brakeLevel) {
    m_brakeEngaging = engaging;
    m_brakeLevel = level;
    QS_BEGIN_ID(BRAKE_STATE, m_prio)
    QS_U8(0U, engaging ? 1U : 0U);
    QS_F32(3U, level);
    QS_F32(3U, static_cast<float>(m_board.torque()));
    QS_END()
  }

#if !MJ_SIM_MOTION
  if (l_motion.enableCount() != m_enableCount) { // a driver enable settled
    m_enableCount = l_motion.enableCount();
    QS_BEGIN_ID(ENABLE_SETTLE, m_prio)
    QS_U32(0U, l_motion.enableSettleUs());
    QS_I32(0U, l_motion.enableSnapMicrosteps());
    QS_U16(0U, static_cast<uint16_t>(m_enableCount));
    QS_END()
  }
#endif
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
#if !MJ_SIM_MOTION
  l_motion.setCheckpointHook(&BSP::crumb); // rail-up breadcrumbs on FW_TRIG0
#endif
  m_board.setSerial(BSP::boardSerial());
  m_board.setPullSign(
      mouse_joystick::calibration::pullSignForSerial(BSP::boardSerial()));
  m_board.setBootInfo(BSP::bootReason(), BSP::bootCrumb());
  m_board.setInstrumentationOn(
      BSP::instrumentationEnabled()); // enabled at boot
  m_board.advanceTo(BSP::nowUs64());
  m_board.boot(); // READY, once
  afterService();
  // The 1 kHz tick arrives as TICK_SIG from the BSP's alarm, coalesced (one
  // in flight at a time), not from a QTimeEvt that would queue up behind a
  // long step such as the rail's soft start.
  return tran(&running);
}

Q_STATE_DEF(Link, running) {
  QState status_;
  switch (e->sig) {
  case TICK_SIG: {
    // The sampler: one backend poll (encoder, status) at 1 kHz, the classifier
    // and capture inside advanceTo(), then the analog output. Its cost is
    // reported per second as SAMPLE_STAT; the budget is 400 us.
    BSP::tickHandled();
    const uint32_t t0 = micros();
    // Overrun guard (2026-10-02, after board 4 never idled once the rail was
    // up): if the previous handler ended less than kIdleGapUs ago the AO is
    // running back to back and the idle loop has had no turn, so this tick's
    // work is skipped (counted) and idle gets one. If the cost exceeds the
    // budget three ticks running, poll every other tick (counted, flagged)
    // until the rail goes down.
    if (m_tickEndValid && (t0 - m_lastTickEnd_us) < kIdleGapUs) {
      BSP::reportTickSkipped(false);
      status_ = Q_RET_HANDLED;
      break;
    }
    if (m_pollDecimated) {
      m_tickParity = !m_tickParity;
      if (m_tickParity) {
        BSP::reportTickSkipped(true);
        status_ = Q_RET_HANDLED;
        break;
      }
    }
    BSP::tickLevel(true);
    m_board.advanceTo(BSP::nowUs64());
    afterService();
    BSP::angleOutput(
        m_board.angleOutputValid(), static_cast<float>(l_motion.angleDeg()),
        static_cast<float>(m_board.angleScaleLo()),
        static_cast<float>(m_board.angleScaleHi()), m_board.hvOn());
    BSP::tickLevel(false);
    const uint32_t cost = micros() - t0;
    BSP::reportTickCost(cost);
    if (cost > kTickBudgetUs) {
      if (m_overBudgetStreak < 255U) {
        ++m_overBudgetStreak;
      }
      if (m_overBudgetStreak >= 3U && !m_pollDecimated) {
        m_pollDecimated = true;
        BSP::reportPollMode(true);
      }
    } else {
      m_overBudgetStreak = 0U;
    }
    if (m_pollDecimated && !m_board.hvOn()) { // the rail went down: start over
      m_pollDecimated = false;
      m_overBudgetStreak = 0U;
      BSP::reportPollMode(false);
    }
    m_lastTickEnd_us = micros();
    m_tickEndValid = true;
    status_ = Q_RET_HANDLED;
    break;
  }
  case LINE_SIG: {
    const LineEvt *const le = static_cast<const LineEvt *>(e);
    recordRx(le);
#if MJ_SIM_MOTION
    // Bench stimulus, simulated-motion builds only: `sim;<seq>;angle;<deg>`
    // sets the simulated encoder, as the corpus's ANGLE step does, so a WATCH
    // outcome can be driven by hand on a fixture with no joystick. It never
    // reaches ProtocolBoard, is not a protocol verb, and the delivery build
    // answers it `badverb` like any unknown verb.
    if (strncmp(le->text, "sim;", 4U) == 0) {
      simStimulus(le->text + 4);
      afterService();
      status_ = Q_RET_HANDLED;
      break;
    }
#endif
    const uint32_t t0 = micros();
    m_board.advanceTo(BSP::nowUs64());
    const uint32_t t1 = micros();
    m_board.feed(le->text);
    const uint32_t t2 = micros();
    afterService();
#if !MJ_SIM_MOTION
    if (strncmp(le->text, "hv;", 3U) == 0) {
      BSP::crumb(9U); // the hv reply is in the port ring
    }
#endif
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
