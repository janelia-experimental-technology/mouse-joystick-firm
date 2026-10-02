// Board support for the QP/C++ link application on mouse-joystick-pcb, Earle
// Philhower arduino-pico core, QP/C++ v7.3.4 QV kernel. Derived from the
// qp-arduino RP2040 BSPs and the joystick-link pilot in qp-arduino-example:
// hardware-alarm tick with watchdog, bounded transmit-only QS transport on a
// PIO UART, non-blocking line transport. Board facts come from BoardPins.hpp
// and SafeState; no GPIO number appears here.
//
// Compile-time configuration (build_flags in platformio.ini):
//
//   MJ_SIM_MOTION (default 0)
//       1 replaces TmcMotion with SimulatedMotion: no 36 V, no SPI, the
//       protocol's "hv" switches nothing. The first build run on the fixture.
//   MJ_LINK_USB (default 0)
//       1 carries the protocol over USB CDC instead of UART0, for a bench with
//       no pyControl host: the board waits MJ_LINK_HOST_WAIT_MS (10 s) for the
//       port to be opened so READY is not discarded. 0 is the delivery
//       configuration: UART0 on the pyControl RJ-45, USB CDC prints a banner.
//   MJ_TICK_PROBE (default 0)
//       1 pulses the UART0 TX pad on every tick, allowed only with
//       MJ_LINK_USB=1 where that pad is otherwise unused (DIG8, LA5032 ch 8).
//   QS_ON
//       QS trace on a transmit-only PIO UART driving FW_TRIG1 (InstrumentPort
//       TRIG1), MJ_QS_BAUD (115200) 8N1, dictionaries every
//       MJ_QS_DICT_PERIOD_MS (10000), TRACE_STAT and LINK_STAT every
//       MJ_QS_STAT_PERIOD_MS (1000). No receive side exists: nothing on the
//       trace link can reset, command or poke the board. Without QS_ON the
//       application runs untraced and FW_TRIG1 stays idle.
//   MJ_DAC_I2C (default 0)
//       Rev 1 only: the angle DAC on I2C instead of SPI1 (v1.0 straps I2C;
//       untestable on v0.3, whose SPI2C pin is tied to ground).
//   LINK_QS_FULL (default 0)
//       1 keeps every QP record; 0, the observation set, drops the per-event
//       dispatch/internal-transition/post/get records so a 100 Hz tick does not
//       consume the 115200 baud line.
//
// Proof of life without USB: the boot signature and the identity beacon on
// FW_TRIG0/FW_TRIG1 at start-up, then the trace itself on FW_TRIG1 (a
// TRACE_STAT record every second) and FW_TRIG0 high during every operation.
// FW_TRIG1 is a UART line from the moment QS starts; SafeState::setTrigger1()
// must not be used after BSP::init().

#include "qpcpp.hpp"

#include "Bsp.hpp"
#include "LinkAo.hpp"

#include <Arduino.h>
#include <BoardPins.hpp>
#include <Conversions.hpp>
#include <SafeState.hpp>
#include <SelfTest.hpp>
#if MJ_BOARD_REV != 0
#include <DAC80501.hpp>
#include <SPI.h>
#include <Wire.h>
#endif
#include <hardware/timer.h>    // hardware_alarm_*()
#include <hardware/watchdog.h> // watchdog_reboot()
#include <pico/time.h>         // time_us_64(), from_us_since_boot()
#include <pico/unique_id.h>    // pico_get_unique_board_id_string()
#include <string.h>

using namespace QP;
using mouse_joystick::SafeState;
using mouse_joystick::SelfTest;
namespace pins = mouse_joystick::pins;

Q_DEFINE_THIS_FILE

#ifndef MJ_SIM_MOTION
#define MJ_SIM_MOTION 0
#endif
#ifndef MJ_LINK_USB
#define MJ_LINK_USB 0
#endif
#ifndef MJ_TICK_PROBE
#define MJ_TICK_PROBE 0
#endif
#if MJ_TICK_PROBE && !MJ_LINK_USB
#error "MJ_TICK_PROBE needs MJ_LINK_USB=1: the probe pad is UART0 TX"
#endif
#ifndef MJ_QS_BAUD
#define MJ_QS_BAUD 115200U
#endif
#ifndef MJ_QS_SERVICE_US
#define MJ_QS_SERVICE_US 1000U
#endif
#ifndef MJ_QS_SERVICE_BYTES
#define MJ_QS_SERVICE_BYTES 64U
#endif
#ifndef MJ_QS_DICT_PERIOD_MS
#define MJ_QS_DICT_PERIOD_MS 10000U
#endif
#ifndef MJ_QS_STAT_PERIOD_MS
#define MJ_QS_STAT_PERIOD_MS 1000U
#endif
#ifndef MJ_LINK_HOST_WAIT_MS
#define MJ_LINK_HOST_WAIT_MS 10000U
#endif
#ifndef MJ_LINK_POLL_US
#define MJ_LINK_POLL_US 1000U
#endif
#ifndef MJ_LINK_POLL_BYTES
#define MJ_LINK_POLL_BYTES 128
#endif
#ifndef LINK_QS_FULL
#define LINK_QS_FULL 0
#endif

#if MJ_LINK_USB
#define LINK_PORT Serial
#else
#define LINK_PORT Serial1
#endif

enum BspRecords { // BSP-level QS user records (the application uses +0..+7)
  TRACE_STAT = QP::QS_USER + 8,
  TICK_LOST = QP::QS_USER + 9,
  LINK_STAT = QP::QS_USER + 10,
  SAMPLE_STAT =
      QP::QS_USER +
      11, // per second: ticks posted, coalesced, dropped, max/last cost
  DAC_STATE = QP::QS_USER + 12 // rev 1 angle DAC phase changed: phase (0 down,
                               // 1 reset, 2 up), begin failures
};

namespace {

// A Print that discards, for SelfTest reports when no USB host is attached.
class NullPrint : public Print {
public:
  size_t write(uint8_t) override { return 1; }
  size_t write(const uint8_t *, size_t size) override { return size; }
};
NullPrint l_nullPrint;

// Transmit-only PIO UART for the QS trace on FW_TRIG1. SerialPIO reports the
// free space of its joined 8-entry PIO FIFO through availableForWrite() and
// blocks in write() only when that FIFO is full, which qsTransmitSome() never
// lets happen.
SerialPIO l_qsPioSerial(pins::kFwTrig1, SerialPIO::NOPIN);

QP::QSpyId const l_TICK_ID = {0U};       // QS source of the tick
QP::QSpyId const l_linkTransport = {0U}; // QS sender of LineEvt posts

// The sampler's tick event: one static immutable event, posted by the alarm
// interrupt when none is in flight. A periodic QTimeEvt would queue one event
// per tick behind a long run-to-completion step (the rail's 60 ms soft
// start) and overflow the queue; coalescing loses the sample instead, and
// counts it.
QP::QEvt const l_tickEvt(TICK_SIG);
bool volatile l_tickPending = false;
uint32_t volatile l_tickCoalesced = 0U; // alarm fired with a tick still pending
uint32_t volatile l_tickDropped = 0U;   // the AO queue refused the tick
uint32_t l_tickCostMax_us = 0U; // from the AO, since the last SAMPLE_STAT
uint32_t l_tickCostLast_us = 0U;
uint32_t l_tickHandled = 0U;

#if MJ_TICK_PROBE
inline void tickProbe(PinStatus level) {
  digitalWrite(pins::kPycUartTx, level);
}
#else
inline void tickProbe(PinStatus) {}
#endif

//----------------------------------------------------------------------------
// System clock tick: a hardware timer alarm owned by this BSP, re-armed from
// its own interrupt and watched from the idle loop. Not the pico-sdk repeating
// timer: that lives in the alarm pool shared with the core's USB task and
// stopped silently under load (qp-arduino, 2026-09-30).
constexpr uint32_t kTickPeriodUs = 1000000U / BSP::TICKS_PER_SEC;
int l_tickAlarm = -1;
uint64_t l_tickTarget = 0U;
uint32_t volatile l_tickCount = 0U;
uint32_t volatile l_tickCatchUps = 0U;
uint32_t volatile l_tickRearms = 0U;
uint32_t volatile l_lastTick_us = 0U;

void armTick() {
  while (hardware_alarm_set_target(static_cast<uint>(l_tickAlarm),
                                   from_us_since_boot(l_tickTarget))) {
    l_tickTarget += kTickPeriodUs;
    ++l_tickCatchUps;
  }
}
void tickAlarmCallback(uint) {
  tickProbe(HIGH);
  l_tickTarget += kTickPeriodUs;
  armTick();
  ++l_tickCount;
  l_lastTick_us = micros();
  QTimeEvt::tick(
      0U, &l_TICK_ID); // no QP time events are armed today; kept for them
  if (l_tickPending) {
    ++l_tickCoalesced;
  } else {
    l_tickPending = true;
    if (!AO_Link->POST_X(&l_tickEvt, 0U, &l_TICK_ID)) { // queue full
      l_tickPending = false;
      ++l_tickDropped;
    }
  }
  tickProbe(LOW);
}
void startTick() {
  l_tickAlarm = hardware_alarm_claim_unused(true);
  hardware_alarm_set_callback(static_cast<uint>(l_tickAlarm),
                              &tickAlarmCallback);
  l_tickTarget = time_us_64() + kTickPeriodUs;
  l_lastTick_us = micros();
  armTick();
}
void tickWatchdog() {
  const uint32_t last =
      l_lastTick_us; // read before the clock (race, 2026-09-30)
  const int32_t elapsed = static_cast<int32_t>(micros() - last);
  if (elapsed > static_cast<int32_t>(5U * kTickPeriodUs)) {
    ++l_tickRearms;
    QS_BEGIN_ID(TICK_LOST, 0U)
    QS_U32(0U, l_tickCount);
    QS_U32(0U, l_tickRearms);
    QS_U32(0U, l_tickCatchUps);
    QS_END()
    l_tickTarget = time_us_64() + kTickPeriodUs;
    l_lastTick_us = micros();
    armTick();
  }
}

//----------------------------------------------------------------------------
// Protocol line transport. Lines in: read in the idle loop, posted as LineEvt.
// Lines out: a ring the idle loop drains into the port as the port has room,
// so a reply never blocks the active object (the hardware UART reports room
// one byte at a time; USB CDC in chunks).
char l_rxLine[sizeof(LineEvt::text)];
uint8_t l_rxLen = 0U;
bool l_rxOverlong = false;
uint32_t l_linkLastPoll_us = 0U;
uint32_t l_linesRx = 0U;           // lines posted to the AO
uint16_t l_linesDroppedPool = 0U;  // no LineEvt left in the pool
uint16_t l_linesDroppedQueue = 0U; // AO queue full
uint16_t l_repliesDropped = 0U;    // ring full
uint16_t l_linesOverlong = 0U;     // truncated before posting
uint16_t l_ringHighWater = 0U;
char l_boardSerial[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1] = {0};

constexpr uint16_t kRingSize = 1024U; // eight full replies
char l_ring[kRingSize];
uint16_t volatile l_ringHead = 0U; // next byte to write (AO context)
uint16_t volatile l_ringTail = 0U; // next byte to send (idle context)

uint16_t ringUsed() {
  return static_cast<uint16_t>((l_ringHead + kRingSize - l_ringTail) %
                               kRingSize);
}

void linkDrain() {
  while (l_ringTail != l_ringHead) {
    const int room = LINK_PORT.availableForWrite();
    if (room <= 0) {
      return;
    }
    const uint16_t head = l_ringHead;
    uint16_t n = (head > l_ringTail)
                     ? static_cast<uint16_t>(head - l_ringTail)
                     : static_cast<uint16_t>(kRingSize - l_ringTail);
    if (n > static_cast<uint16_t>(room)) {
      n = static_cast<uint16_t>(room);
    }
    const size_t written = LINK_PORT.write(
        reinterpret_cast<const uint8_t *>(&l_ring[l_ringTail]), n);
    if (written == 0U) {
      return;
    }
    l_ringTail = static_cast<uint16_t>((l_ringTail + written) % kRingSize);
  }
}

void linkPoll() {
  const uint32_t now = micros();
  if ((now - l_linkLastPoll_us) < MJ_LINK_POLL_US) {
    return;
  }
  l_linkLastPoll_us = now;
  int budget = MJ_LINK_POLL_BYTES;
  while ((budget-- > 0) && (LINK_PORT.available() > 0)) {
    const int c = LINK_PORT.read();
    if (c < 0) {
      break;
    }
    if (c == '\n') {
      LineEvt *e =
          Q_NEW_X(LineEvt, 0U, LINE_SIG); // nullptr if the pool is empty
      if (e == nullptr) {
        ++l_linesDroppedPool;
      } else {
        memcpy(e->text, l_rxLine, l_rxLen);
        e->text[l_rxLen] = '\n';
        e->text[l_rxLen + 1U] = '\0';
        e->len = static_cast<uint8_t>(l_rxLen + 1U);
        if (l_rxOverlong) {
          ++l_linesOverlong;
        }
        if (!AO_Link->POST_X(e, 0U, &l_linkTransport)) { // queue full
          QF::gc(e);
          ++l_linesDroppedQueue;
        } else {
          ++l_linesRx;
        }
      }
      l_rxLen = 0U;
      l_rxOverlong = false;
    } else if (l_rxLen < static_cast<uint8_t>(sizeof(l_rxLine) - 2U)) {
      l_rxLine[l_rxLen++] = static_cast<char>(c);
    } else {
      l_rxOverlong = true;
    }
  }
}

void linkBegin() {
#if MJ_LINK_USB
  // Serial.begin() already ran for the banner; nothing else to do.
#else
  // UART0 on the pyControl RJ-45, as PycUart::begin(): never Serial2, whose
  // defaults land on TMC_DIAG1_ and ~{BUS_SW_EN}.
  Serial1.end();
  Serial1.setTX(pins::kPycUartTx);
  Serial1.setRX(pins::kPycUartRx);
  Serial1.begin(115200);
#endif
}

//----------------------------------------------------------------------------
// QS transport helpers (always compiled; the port forces Q_SPY)
void (*l_dictionaryHook)() = nullptr;
uint32_t l_qsHighWater = 0U;
uint32_t l_qsSaturations = 0U;
bool l_qsNoFlush =
    false; // set while re-emitting dictionaries from the idle loop

bool qsServiceDue() {
  static uint32_t last_us = 0U;
  const uint32_t now = micros();
  const uint32_t pending = QP::QS::priv_.used;
  if (pending > l_qsHighWater) {
    l_qsHighWater = pending;
  }
  if (pending >= QP::QS::priv_.end) {
    ++l_qsSaturations;
  }
  if ((pending < MJ_QS_SERVICE_BYTES) && ((now - last_us) < MJ_QS_SERVICE_US)) {
    return false;
  }
  last_us = now;
  return true;
}

// QS::getBlock() changes the ring buffer's shared state and must run with
// interrupts disabled: records written from interrupts race with it otherwise
// (found 2026-10-01 on the pilot as corrupt records and resets).
const uint8_t *qsTakeBlock(uint16_t *len) {
  QF_INT_DISABLE();
  const uint8_t *const buf = QS::getBlock(len);
  QF_INT_ENABLE();
  return buf;
}

void qsTransmitSome() {
  const int room = l_qsPioSerial.availableForWrite();
  if (room > 0) {
    uint16_t len = (room > 0xFFFF) ? 0xFFFFU : static_cast<uint16_t>(room);
    const uint8_t *const buf = qsTakeBlock(&len);
    if (buf != nullptr) {
      static_cast<void>(l_qsPioSerial.write(buf, len));
    }
  }
}

void qsFlushBounded(uint32_t timeout_us) {
  uint32_t start = micros();
  for (;;) {
    const int room = l_qsPioSerial.availableForWrite();
    if (room > 0) {
      uint16_t len = (room > 0xFFFF) ? 0xFFFFU : static_cast<uint16_t>(room);
      const uint8_t *const buf = qsTakeBlock(&len);
      if (buf == nullptr) {
        return;
      }
      static_cast<void>(l_qsPioSerial.write(buf, len));
      start = micros();
    } else if ((micros() - start) > timeout_us) {
      return;
    }
  }
}

void bspDictionaries() {
  QS_OBJ_DICTIONARY(&l_TICK_ID);
  QS_OBJ_DICTIONARY(&l_linkTransport);
  QS_USR_DICTIONARY(TRACE_STAT);
  QS_USR_DICTIONARY(TICK_LOST);
  QS_USR_DICTIONARY(LINK_STAT);
  QS_USR_DICTIONARY(SAMPLE_STAT);
  QS_USR_DICTIONARY(DAC_STATE);
}

void qsPeriodic() {
#if (MJ_QS_DICT_PERIOD_MS > 0U)
  static uint32_t lastDict_ms = 0U;
  if ((millis() - lastDict_ms) >= MJ_QS_DICT_PERIOD_MS) {
    lastDict_ms = millis();
    // Every QS dictionary function ends with QS::onFlush(); a synchronous
    // flush here held the idle loop for about 30 ms per period on the pilot.
    l_qsNoFlush = true;
    bspDictionaries();
    if (l_dictionaryHook != nullptr) {
      l_dictionaryHook();
    }
    l_qsNoFlush = false;
  }
#endif
#if (MJ_QS_STAT_PERIOD_MS > 0U)
  static uint32_t lastStat_ms = 0U;
  if ((millis() - lastStat_ms) >= MJ_QS_STAT_PERIOD_MS) {
    lastStat_ms = millis();
    QS_BEGIN_ID(TRACE_STAT, 0U)
    QS_U32(0U, l_qsHighWater);
    QS_U32(0U, l_qsSaturations);
    QS_U16(0U, QP::QS::priv_.end);
    QS_U32(0U, l_tickCount);
    QS_END()
    l_qsHighWater = 0U;
    l_qsSaturations = 0U;
    // Query the budgets before opening the record: QF::getPoolMin() has its
    // own critical section and this port's critical sections do not nest.
    const uint16_t queueMin = Link_queueMinFree();
    const uint16_t poolMin = static_cast<uint16_t>(QF::getPoolMin(1U));
    QS_BEGIN_ID(SAMPLE_STAT, 0U)
    QS_U32(0U, l_tickCount);      // alarm ticks since boot
    QS_U32(0U, l_tickHandled);    // dispatched to the AO since boot
    QS_U32(0U, l_tickCoalesced);  // samples lost to a tick still in flight
    QS_U32(0U, l_tickDropped);    // samples lost to a full queue
    QS_U32(0U, l_tickCostMax_us); // worst tick handler since the last record
    QS_U32(0U, l_tickCostLast_us);
    QS_END()
    l_tickCostMax_us = 0U;
    QS_BEGIN_ID(LINK_STAT, 0U)
    QS_U32(0U, l_linesRx);
    QS_U16(0U, l_linesDroppedPool);
    QS_U16(0U, l_linesDroppedQueue);
    QS_U16(0U, l_repliesDropped);
    QS_U16(0U, l_linesOverlong);
    QS_U16(0U, queueMin);
    QS_U16(0U, poolMin);
    QS_U16(0U, l_ringHighWater);
    QS_END()
    l_ringHighWater = 0U;
  }
#endif
}

} // namespace

//----------------------------------------------------------------------------
// BSP functions
void BSP::init() {
  // Safe idle FIRST, and read the resistor-defined pins before anything
  // drives them: HV_EN low, bus isolated, driver disabled, port silent.
  const mouse_joystick::ResetStateReads reset_reads =
      SafeState::readResetState();
  SafeState::apply();
  (void)reset_reads;

  Serial.begin(115200);
#if MJ_LINK_USB
  // USB CDC is the protocol port on this bench: wait for a host so READY is
  // not written into a port nobody has opened.
  {
    const uint32_t start = millis();
    while (!Serial && ((millis() - start) < MJ_LINK_HOST_WAIT_MS)) {
      delay(10);
    }
  }
#endif
  pico_get_unique_board_id_string(l_boardSerial, sizeof(l_boardSerial));

  // Proof of life that needs no USB, on the InstrumentPort: the boot
  // signature, then the identity beacon (three frames), both as GPIO on
  // FW_TRIG0/FW_TRIG1 before the trace claims FW_TRIG1.
  SafeState::setInstrumentationEnabled(true);
  SelfTest::emitBootSignature();
  SelfTest::emitIdentityBeacon(Serial ? static_cast<Print &>(Serial)
                                      : static_cast<Print &>(l_nullPrint),
                               3);

  if (Serial) {
    Serial.println();
    Serial.println(
        F("mouse-joystick-pcb -- pyControl protocol link over QP/C++"));
    SelfTest::reportIdentity(Serial);
#if MJ_LINK_USB
    Serial.println(
        F("Protocol on THIS port (bench build). The 36 V rail is OFF."));
#else
    Serial.println(F("Protocol on UART0 at 115200 8N1. The 36 V rail is OFF:"));
    Serial.println(F("the host turns it on with `hv;<seq>;on`."));
#endif
#if MJ_SIM_MOTION
    Serial.println(F("Motion is SIMULATED in this build."));
#endif
    Serial.println(F("QS trace: FW_TRIG1, transmit only."));
    Serial.println();
  }

  linkBegin();
#if MJ_TICK_PROBE
  // UART0 is unused in the USB bench build; its TX pad becomes the tick probe
  // (InstrumentPort DIG8). Not configured by SafeState, so do it here.
  digitalWrite(pins::kPycUartTx, LOW);
  pinMode(pins::kPycUartTx, OUTPUT);
#endif

#ifdef QS_ON
  QS_INIT(nullptr);
  bspDictionaries();
  QS_GLB_FILTER(QP::QS_SM_RECORDS);
  QS_GLB_FILTER(QP::QS_AO_RECORDS);
  QS_GLB_FILTER(QP::QS_UA_RECORDS);
#if !LINK_QS_FULL
  QS_GLB_FILTER(-QP::QS_QEP_DISPATCH);
  QS_GLB_FILTER(-QP::QS_QEP_INTERN_TRAN);
  QS_GLB_FILTER(-QP::QS_QF_ACTIVE_POST);
  QS_GLB_FILTER(-QP::QS_QF_ACTIVE_GET_LAST);
  QS_GLB_FILTER(-QP::QS_QF_ACTIVE_GET);
#endif
#endif
}
//............................................................................
void BSP::setDictionaryHook(void (*hook)()) { l_dictionaryHook = hook; }
//............................................................................
uint64_t BSP::nowUs64() { return time_us_64(); }
//............................................................................
const char *BSP::boardSerial() { return l_boardSerial; }
//............................................................................
bool BSP::linkWrite(const char *line) {
  const size_t len = strlen(line);
  const uint16_t need = static_cast<uint16_t>(len + 1U);
  const uint16_t used = ringUsed();
  if (static_cast<uint16_t>(kRingSize - 1U - used) < need) {
    ++l_repliesDropped;
    return false;
  }
  uint16_t head = l_ringHead;
  for (size_t i = 0U; i < len; ++i) {
    l_ring[head] = line[i];
    head = static_cast<uint16_t>((head + 1U) % kRingSize);
  }
  l_ring[head] = '\n';
  l_ringHead = static_cast<uint16_t>((head + 1U) % kRingSize);
  const uint16_t now_used = static_cast<uint16_t>(used + need);
  if (now_used > l_ringHighWater) {
    l_ringHighWater = now_used;
  }
  return true;
}
//............................................................................
void BSP::opMarker(bool on) { SafeState::setTrigger0(on); }
//............................................................................
void BSP::tickHandled() {
  l_tickPending = false;
  ++l_tickHandled;
}
//............................................................................
void BSP::reportTickCost(uint32_t us) {
  l_tickCostLast_us = us;
  if (us > l_tickCostMax_us) {
    l_tickCostMax_us = us;
  }
}
//............................................................................
#if MJ_BOARD_REV != 0
namespace {
#ifndef MJ_DAC_I2C
#define MJ_DAC_I2C 0
#endif
#if MJ_DAC_I2C
dac80501::I2cTransport l_dacTransport(Wire, dac80501::kI2cAddrA0Agnd);
#else
dac80501::SpiTransport l_dacTransport(SPI1, pins::kDacCsN);
#endif
dac80501::Dac80501 l_dac(l_dacTransport);

// The DAC runs from the switched domain (MotionAndHoming's gate [4]: driving
// its chip select before the rail is up back-powers that domain through the
// input clamps), so the bus is claimed only while the protocol reports the
// rail up, and released again when it goes down. Bring-up is two ticks, not
// one blocking call: the driver's default begin() sleeps 1 ms after the soft
// reset, which is the whole sampler budget.
enum class DacPhase : uint8_t { kDown, kReset, kUp };
DacPhase l_dacPhase = DacPhase::kDown;
void dacPhase(DacPhase phase); // sets l_dacPhase and records it
uint32_t l_dacResetAt_us = 0U;
bool l_dacParked = false;
uint32_t l_dacBeginFailures = 0U;
constexpr uint32_t kDacResetSettleUs = 2000U; // the driver's 1 ms, doubled

void dacBusUp() {
#if MJ_DAC_I2C
  Wire.setSCL(pins::kDacSpiSck);
  Wire.setSDA(pins::kDacSpiCopi);
  l_dacTransport.begin();
  Wire.setClock(400000U); // 100 kHz would cost ~400 us per update
#else
  SPI1.setSCK(pins::kDacSpiSck);
  SPI1.setTX(pins::kDacSpiCopi);
  SPI1.setRX(pins::kSpi1RxUnused);
  SPI1.begin();
  l_dacTransport.begin();
#endif
}

void dacBusDown() {
  l_dac.invalidate(); // its GAIN is about to reset with the domain
#if MJ_DAC_I2C
  Wire.end();
#else
  SPI1.end();
  pinMode(pins::kDacCsN, INPUT); // nothing drives an unpowered part
#endif
}

void dacPhase(DacPhase phase) {
  l_dacPhase = phase;
  QS_BEGIN_ID(DAC_STATE, 0U)
  QS_U8(0U, static_cast<uint8_t>(phase));
  QS_U32(0U, l_dacBeginFailures);
  QS_END()
}

// Tick 1 after the rail: claim the bus and soft-reset the part. Tick N: write
// CONFIG, GAIN 0x0101 and SYNC (begin() without its own reset and sleep), as
// the bench console does before its first data write.
void dacStep(uint32_t now_us) {
  switch (l_dacPhase) {
  case DacPhase::kDown:
    dacBusUp();
    static_cast<void>(l_dac.softReset());
    l_dacResetAt_us = now_us;
    dacPhase(DacPhase::kReset);
    break;
  case DacPhase::kReset:
    if ((now_us - l_dacResetAt_us) >= kDacResetSettleUs) {
      dac80501::Config cfg = dac80501::arduinoConfig();
      cfg.gain_register = dac80501::kGainRefDiv | dac80501::kGainBuffGain;
      cfg.soft_reset_first = false; // done, and settled, in the previous phase
      cfg.delay_ms = nullptr;
      if (l_dac.begin(cfg)) {
        dacPhase(DacPhase::kUp);
        l_dacParked = false;
      } else {
        ++l_dacBeginFailures;
        l_dacResetAt_us = now_us; // try again after another settle
      }
    }
    break;
  case DacPhase::kUp:
    break;
  }
}
} // namespace

void BSP::angleOutput(bool valid, float angle_deg, float lo, float hi,
                      bool hv_on) {
  namespace conv = mouse_joystick::conversions;
  if (!hv_on) {
    if (l_dacPhase != DacPhase::kDown) {
      dacBusDown();
      dacPhase(DacPhase::kDown);
    }
    return;
  }
  if (l_dacPhase != DacPhase::kUp) {
    dacStep(micros());
    return;
  }
  if (!valid) {
    if (!l_dacParked) {
      static_cast<void>(l_dac.setCode(conv::kAngleOutFaultCode));
      l_dacParked = true;
    }
    return;
  }
  l_dacParked = false;
  float fraction = (hi > lo) ? (angle_deg - lo) / (hi - lo) : 0.0f;
  if (fraction < 0.0f) {
    fraction = 0.0f;
  } else if (fraction > 1.0f) {
    fraction = 1.0f;
  }
  const float bnc_volts =
      conv::kAngleOutMinVolts +
      fraction * (conv::kAngleOutMaxVolts - conv::kAngleOutMinVolts);
  // One data frame per tick, whether or not the code changed, so the cost the
  // SAMPLE_STAT record reports is the steady one. The DAC produces the BNC
  // voltage divided by the output stage's gain.
  static_cast<void>(l_dac.setVolts(bnc_volts / conv::kAngleOutStageGain));
}
#else
void BSP::angleOutput(bool, float, float, float, bool) {} // rev 0: no DAC
#endif
//............................................................................
void BSP::instrumentation(bool on) { SafeState::setInstrumentationEnabled(on); }
//............................................................................
bool BSP::instrumentationEnabled() {
  return SafeState::instrumentationEnabled();
}

//----------------------------------------------------------------------------
// QF callbacks
void QF::onStartup() { startTick(); }
//............................................................................
void QF::onCleanup() {}
//............................................................................
void QV::onIdle() { // called with interrupts DISABLED, see NOTE in qv.hpp
  QF_INT_ENABLE();
  tickWatchdog();
  linkPoll();  // protocol lines in
  linkDrain(); // replies out
#ifdef QS_ON
  qsPeriodic();
  if (qsServiceDue()) {
    qsTransmitSome();
  }
#else
  // No trace: sleep until the next interrupt (tick, UART or USB).
  QF_INT_DISABLE();
  QV_CPU_SLEEP();
#endif
}
//............................................................................
extern "C" Q_NORETURN Q_onError(char const *const module, int const id) {
#ifdef QS_ON
  QS_ASSERTION(module, id, 0U);
#else
  (void)module;
  (void)id;
#endif
  __asm volatile("cpsid i" ::: "memory");
  // The board's rule for any fault: rail off, driver disabled, bus isolated.
  // SafeState::apply() is the only place allowed to touch HV_EN.
  SafeState::apply();
  // Analyzer signature: FW_TRIG0 solid high for two seconds, then a watchdog
  // reset (rev 0 has no reset button and no SWD).
  SafeState::setInstrumentationEnabled(true);
  SafeState::setTrigger0(true);
  const uint32_t start = micros();
  while ((micros() - start) < 2000000U) {
  }
  watchdog_reboot(0U, 0U, 10U);
  for (;;) {
  }
}

//----------------------------------------------------------------------------
// QS callbacks (always defined; the port forces Q_SPY)
bool QS::onStartup(void const *arg) {
  (void)arg;
  static uint8_t qsTxBuf[2048]; // declared budget
  static uint8_t qsRxBuf[128];  // required by QS; never fed
  initBuf(qsTxBuf, sizeof(qsTxBuf));
  rxInitBuf(qsRxBuf, sizeof(qsRxBuf));
  l_qsPioSerial.begin(MJ_QS_BAUD);
  return true;
}
//............................................................................
void QS::onCleanup() {}
//............................................................................
QSTimeCtr QS::onGetTime() {
  return static_cast<QSTimeCtr>(micros()); // 1 us resolution, wraps at 32 bits
}
//............................................................................
void QS::onFlush() {
  if (!l_qsNoFlush) {
    qsFlushBounded(200000U);
  }
}
//............................................................................
void QS::onReset() {
  watchdog_reboot(0U, 0U, 10U);
  for (;;) {
  }
}
//............................................................................
void QS::onCommand(uint8_t cmdId, uint32_t param1, uint32_t param2,
                   uint32_t param3) {
  (void)cmdId;
  (void)param1;
  (void)param2;
  (void)param3;
}
