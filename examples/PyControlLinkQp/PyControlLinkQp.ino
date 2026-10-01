// Stage 4 over QP/C++: the pyControl UART protocol inside one active object,
// traced over FW_TRIG1. See README.md. This file only wires the framework up;
// keep it free of functions and of Q_DEFINE_THIS_FILE (the Arduino sketch
// preprocessor duplicates both).

#include "qpcpp.hpp"

#include "Bsp.hpp"
#include "LinkAo.hpp"

using namespace QP;

static QF_MPOOL_EL(LineEvt) l_lineSto[8]; // declared budget: 8 lines in flight
static QEvt const *l_linkQueueSto[16];    // declared budget: 16 events queued

void setup() {
  QF::init();
  BSP::init();
  BSP::setDictionaryHook(&Link_dictionaries);
  QF::poolInit(l_lineSto, sizeof(l_lineSto), sizeof(l_lineSto[0]));
  AO_Link->start(1U, l_linkQueueSto, Q_DIM(l_linkQueueSto), nullptr, 0U);
}

void loop() {
  QF::run(); // never returns
}
