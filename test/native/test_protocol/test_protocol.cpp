// Host tests for the board's protocol layer, and the firmware half of Gate C.
//
// Gate C asks that "a host simulator or protocol fixture and the firmware agree
// on framing and versions". `pycontrol/tests/test_fixture_corpus.py` proves the
// Python simulator reproduces `pycontrol/tests/fixtures/protocol_cases.txt`.
// THIS file is the other half. Until both existed the corpus proved nothing: a
// simulator agreeing with a fixture only it has ever been checked against is a
// tautology, and the corpus only becomes evidence once two independent
// implementations are held to it.
//
// The corpus parser here is a deliberate direct translation of the Python one,
// not a reinterpretation. If the two parsers disagree about the fixture, the
// fixture stops being a shared contract and becomes two different files that
// happen to share a name.
//
// STL is used freely below. This is a host test; the firmware headers it
// exercises allocate nothing.

#include <unity.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <set>
#include <string>
#include <vector>

#include "BrakeTorqueBackend.hpp"
#include "MotionBackend.hpp"
#include "Protocol.hpp"
#include "ProtocolBoard.hpp"

using mouse_joystick::ProtocolBoard;
using mouse_joystick::SimulatedBrakeTorque;
using mouse_joystick::SimulatedMotion;
namespace P = mouse_joystick::protocol;

// The board holds its backends by reference, so the three have to be owned
// together. Declaration order matters: the backends are constructed first.
// `caps` are the corpus's CAPS tokens (brake, torque, angout): the board
// boots with those hardware facts, as the Python simulator does.
struct Rig {
  SimulatedMotion motion;
  SimulatedBrakeTorque brake_torque;
  ProtocolBoard board;

  Rig() : board(motion, corpusConfig(std::set<std::string>()), &brake_torque) {
    board.boot();
    board.clearOutput();  // READY drained, as the corpus assumes
  }
  explicit Rig(const std::set<std::string> &caps)
      : board(motion, corpusConfig(caps), &brake_torque) {
    board.boot();
    board.clearOutput();
  }

  static ProtocolBoard::Config corpusConfig() {
    return corpusConfig(std::set<std::string>());
  }
  static ProtocolBoard::Config corpusConfig(const std::set<std::string> &caps) {
    ProtocolBoard::Config cfg;
    cfg.serial = "0000000000000000";  // the corpus header's stated default
    cfg.has_brake = caps.count("brake") != 0;
    cfg.has_torque = caps.count("torque") != 0;
    cfg.has_angout = caps.count("angout") != 0;
    return cfg;
  }
};

void setUp(void) {}
void tearDown(void) {}

// ============================================================ codec unit tests

// `str(float)` is not guaranteed to agree between CPython and MicroPython, let
// alone with printf. These are the exact spellings the corpus depends on.
void test_number_formatting_matches_the_python_side(void) {
  char out[32];

  P::formatNumber(0.0, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("0.0", out);

  P::formatNumber(20.0, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("20.0", out);

  P::formatNumber(-20.0, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("-20.0", out);

  P::formatNumber(3600.0, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("3600.0", out);

  P::formatNumber(0.421, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("0.421", out);

  P::formatNumber(-12.5, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("-12.5", out);

  P::formatNumber(-3.1, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("-3.1", out);
}

void test_negative_zero_is_never_emitted(void) {
  // Not hypothetical: any move that lands on zero from below produces it, and
  // it is a valid float that reads as a different value to a human.
  char out[32];
  P::formatNumber(-0.0, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("0.0", out);

  P::formatNumber(-0.0001, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("0.0", out);
}

// The formatter rounds the exact binary value with ties to even, as `%.3f`
// does in newlib and CPython, without calling either: these expectations were
// produced by `MouseJoystickProtocol.format_number` on CPython and include the
// cases that only exact arithmetic gets right (0.9995 is a hair above, 1.0005
// a hair below, 0.0625 and 0.1875 are exact ties).
void test_number_formatting_is_exact_on_the_binary_value(void) {
  struct Case {
    double value;
    const char *text;
  };
  static const Case cases[] = {
      {0.0, "0.0"},
      {-0.0, "0.0"},
      {6.0, "6.0"},
      {-9.0, "-9.0"},
      {12.5, "12.5"},
      {20.0, "20.0"},
      {-20.0, "-20.0"},
      {180.0, "180.0"},
      {3600.0, "3600.0"},
      {0.421, "0.421"},
      {0.5, "0.5"},
      {0.05, "0.05"},
      {0.001, "0.001"},
      {0.0005, "0.001"},
      {-0.0005, "-0.001"},
      {0.00049, "0.0"},
      {0.0625, "0.062"},
      {0.1875, "0.188"},
      {0.0015, "0.002"},
      {0.0025, "0.003"},
      {2.0005, "2.001"},
      {1.0005, "1.0"},
      {1234.5678, "1234.568"},
      {-3.691, "-3.691"},
      {0.1, "0.1"},
      {0.2, "0.2"},
      {0.3, "0.3"},
      {1e-07, "0.0"},
      {-1e-07, "0.0"},
      {123456.789, "123456.789"},
      {0.9995, "1.0"},
      {0.99949, "0.999"},
      {9.9995, "9.999"},
      {359.99951, "360.0"},
      {1.0 / 3.0, "0.333"},
      {2.0 / 3.0, "0.667"},
      {1e15, "1000000000000000.0"},
      {4294967296.5, "4294967296.5"},
      {100.0 / 3.0, "33.333"},
      {7.0 / 9.0, "0.778"},
  };
  char out[32];
  for (const Case &c : cases) {
    P::formatNumber(c.value, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING_MESSAGE(c.text, out, "formatNumber");
  }
}

void test_integer_formatting_without_printf(void) {
  char out[24];
  P::formatInt(0, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("0", out);
  P::formatInt(65535, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("65535", out);
  P::formatInt(-1, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("-1", out);
  P::formatInt(-2147483647L - 1L, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("-2147483648", out);
  P::formatInt(2147483647L, out, sizeof(out));
  TEST_ASSERT_EQUAL_STRING("2147483647", out);
}

// Decimal digits scaled by one exact power of ten give the same double as
// `strtod`/`float()` for everything the protocol carries; rejected inputs are
// the ones `float()` rejects too, plus "inf"/"nan"/underscores, which no case
// uses.
void test_number_parsing_matches_float(void) {
  struct Case {
    const char *text;
    double value;
  };
  static const Case accepted[] = {
      {"0", 0.0},
      {"6", 6.0},
      {"-9.0", -9.0},
      {"12.5", 12.5},
      {"+3", 3.0},
      {"0.05", 0.05},
      {"3.691", 3.691},
      {"-3.691", -3.691},
      {".5", 0.5},
      {"5.", 5.0},
      {" 4 ", 4.0},
      {"1e3", 1000.0},
      {"2.5E-3", 0.0025},
      {"0.1", 0.1},
      {"5000", 5000.0},
      {"0.000499999999999", 0.000499999999999}, // 12 digits: exact path
      {"180.0", 180.0},
      {"-0", -0.0},
      {"1234.5678", 1234.5678},
  };
  for (const Case &c : accepted) {
    double v = 123.0;
    TEST_ASSERT_TRUE_MESSAGE(P::parseNumber(c.text, &v), c.text);
    TEST_ASSERT_TRUE_MESSAGE(v == c.value, c.text);
  }
  static const char *const rejected[] = {"",     "abc",   "1.2.3", "1e",  "e3",
                                         "-",    ".",     "12x",   "nan", "inf",
                                         "0x10", "1_000", "1 2"};
  for (const char *text : rejected) {
    double v = 0.0;
    TEST_ASSERT_FALSE_MESSAGE(P::parseNumber(text, &v), text);
  }
}

void test_integer_parsing_matches_int(void) {
  long v = 0;
  TEST_ASSERT_TRUE(P::parseInt("32", &v));
  TEST_ASSERT_EQUAL(32, v);
  TEST_ASSERT_TRUE(P::parseInt("-7", &v));
  TEST_ASSERT_EQUAL(-7, v);
  TEST_ASSERT_TRUE(P::parseInt(" +0 ", &v));
  TEST_ASSERT_EQUAL(0, v);
  TEST_ASSERT_TRUE(P::parseInt("99999999999", &v)); // clamps, as strtol did
  TEST_ASSERT_EQUAL(2147483647L, v);
  TEST_ASSERT_FALSE(P::parseInt("12.5", &v));
  TEST_ASSERT_FALSE(P::parseInt("", &v));
  TEST_ASSERT_FALSE(P::parseInt("-", &v));
  TEST_ASSERT_FALSE(P::parseInt("3a", &v));
}

// A tick that arrives late fills the missed grid points with the current
// angle and FETCH reports them as `lost`; a tick on time reports none.
void test_a_late_tick_fills_the_grid_and_fetch_counts_it(void) {
  mouse_joystick::TrialWatch w;
  w.setWindow(10, 2);
  w.release(1000000, 0.0, 1.0);
  mouse_joystick::TrialWatch::Params p;
  p.pull = 5.0;
  p.push = 5.0;
  p.timeout_us = 1000000;
  w.arm(1000000, p, 0.5);
  for (uint32_t k = 0; k < 5; ++k) { // five on-time samples
    w.sample(1000000 + k * 1000, 0.0);
    w.evaluate(1000000 + k * 1000, 0.0);
  }
  // the next tick comes 3 ms late: grid points +5, +6, +7 ms are fills
  w.sample(1008000, 6.0);
  TEST_ASSERT_EQUAL(mouse_joystick::TrialWatch::Outcome::kPull,
                    w.evaluate(1008000, 6.0));
  for (uint32_t k = 9; k <= 10; ++k) {
    w.sample(1000000 + k * 1000, 6.0);
  }
  TEST_ASSERT_TRUE(w.hasWindow());
  TEST_ASSERT_EQUAL(8, w.windowPre()); // samples 0..7 precede the outcome
  TEST_ASSERT_EQUAL(11, w.windowCount());
  TEST_ASSERT_EQUAL(3, w.windowLost());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, static_cast<float>(w.windowSample(4)));
  TEST_ASSERT_EQUAL_FLOAT(6.0f, static_cast<float>(w.windowSample(5)));
}

void test_sequence_numbers_wrap_the_way_python_does(void) {
  uint16_t seq = 0;

  TEST_ASSERT_TRUE(P::normalizeSeq("0", &seq));
  TEST_ASSERT_EQUAL_UINT16(0, seq);

  TEST_ASSERT_TRUE(P::normalizeSeq("65535", &seq));
  TEST_ASSERT_EQUAL_UINT16(65535, seq);

  TEST_ASSERT_TRUE(P::normalizeSeq("65536", &seq));
  TEST_ASSERT_EQUAL_UINT16(0, seq);

  TEST_ASSERT_TRUE(P::normalizeSeq("65537", &seq));
  TEST_ASSERT_EQUAL_UINT16(1, seq);

  // Python's `%` floors, so a negative sequence number wraps to the top of the
  // range rather than staying negative. C's `%` would give -1 here.
  TEST_ASSERT_TRUE(P::normalizeSeq("-1", &seq));
  TEST_ASSERT_EQUAL_UINT16(65535, seq);

  // A digit run longer than 64 bits must still reduce, as Python's arbitrary
  // precision would. Accumulating modulo the range is what makes that true.
  TEST_ASSERT_TRUE(P::normalizeSeq("99999999999999999999999999", &seq));
}

void test_sequence_numbers_reject_what_python_rejects(void) {
  uint16_t seq = 0;
  TEST_ASSERT_FALSE(P::normalizeSeq("", &seq));
  TEST_ASSERT_FALSE(P::normalizeSeq("notanumber", &seq));
  TEST_ASSERT_FALSE(P::normalizeSeq("12.5", &seq));   // int("12.5") raises
  TEST_ASSERT_FALSE(P::normalizeSeq("12x", &seq));
  TEST_ASSERT_FALSE(P::normalizeSeq("-", &seq));
}

void test_decode_recovers_verb_seq_and_args(void) {
  P::Command cmd;
  TEST_ASSERT_TRUE(P::decodeCommand("cal;419;set;enc_offset;2048", &cmd));
  TEST_ASSERT_EQUAL_STRING("cal", cmd.verb);
  TEST_ASSERT_EQUAL_UINT16(419, cmd.seq);
  TEST_ASSERT_EQUAL_UINT8(3, cmd.arg_count);
  TEST_ASSERT_EQUAL_STRING("set", cmd.arg(0));
  TEST_ASSERT_EQUAL_STRING("enc_offset", cmd.arg(1));
  TEST_ASSERT_EQUAL_STRING("2048", cmd.arg(2));
}

void test_verbs_decode_case_insensitively(void) {
  // The draft's table is uppercase and every concrete example is lowercase.
  P::Command cmd;
  TEST_ASSERT_TRUE(P::decodeCommand("PING;3", &cmd));
  TEST_ASSERT_EQUAL_STRING("ping", cmd.verb);
}

void test_carriage_returns_are_stripped_from_both_ends(void) {
  // The draft says a *leading* `\r` is tolerated. A CRLF peer's `\r` actually
  // arrives trailing, so reading the draft literally leaves the common case
  // broken. Both ends are stripped here and on the Python side.
  P::Command cmd;
  TEST_ASSERT_TRUE(P::decodeCommand("ping;4\r", &cmd));
  TEST_ASSERT_EQUAL_UINT16(4, cmd.seq);
  TEST_ASSERT_TRUE(P::decodeCommand("\rping;5", &cmd));
  TEST_ASSERT_EQUAL_UINT16(5, cmd.seq);
}

void test_bad_frames_report_a_sequence_number_when_one_is_readable(void) {
  P::Command cmd;

  // No seq field at all: nothing to correlate against, so `?`.
  TEST_ASSERT_FALSE(P::decodeCommand("ping", &cmd));
  TEST_ASSERT_FALSE(cmd.has_seq);

  // Unreadable seq: also `?`, because the number was not recoverable.
  TEST_ASSERT_FALSE(P::decodeCommand("ping;notanumber", &cmd));
  TEST_ASSERT_FALSE(cmd.has_seq);

  // An empty verb is a bad frame; `";"` splits to two empty fields.
  TEST_ASSERT_FALSE(P::decodeCommand(";", &cmd));
  TEST_ASSERT_FALSE(cmd.has_seq);

  TEST_ASSERT_FALSE(P::decodeCommand("", &cmd));
  TEST_ASSERT_FALSE(P::decodeCommand("\r\n", &cmd));
}

void test_over_length_lines_are_rejected(void) {
  std::string long_line = "ping;1;";
  long_line.append(P::kMaxLine, 'x');
  P::Command cmd;
  TEST_ASSERT_FALSE(P::decodeCommand(long_line.c_str(), &cmd));
}

void test_retransmission_classification_matches_the_python_side(void) {
  // Reads: never cached.
  TEST_ASSERT_FALSE(P::changesState("ping", ""));
  TEST_ASSERT_FALSE(P::changesState("id", ""));
  TEST_ASSERT_FALSE(P::changesState("status", ""));
  TEST_ASSERT_FALSE(P::changesState("limits", ""));
  TEST_ASSERT_FALSE(P::changesState("faults", ""));

  // State-changing: cached.
  TEST_ASSERT_TRUE(P::changesState("moveto", "1.0"));
  TEST_ASSERT_TRUE(P::changesState("move", "1.0"));
  TEST_ASSERT_TRUE(P::changesState("home", ""));
  TEST_ASSERT_TRUE(P::changesState("stop", ""));
  TEST_ASSERT_TRUE(P::changesState("clrfault", ""));

  // Dual-mode: the whole reason classification is on the parsed command and not
  // on the verb name. A verb table gets all three of these wrong.
  TEST_ASSERT_FALSE(P::changesState("cal", "get"));
  TEST_ASSERT_TRUE(P::changesState("cal", "set"));
  TEST_ASSERT_FALSE(P::changesState("instr", "state"));
  TEST_ASSERT_TRUE(P::changesState("instr", "on"));
  TEST_ASSERT_FALSE(P::changesState("hv", "state"));
  TEST_ASSERT_TRUE(P::changesState("hv", "on"));
  TEST_ASSERT_FALSE(P::changesState("hv", "STATE"));  // case-insensitive

  // Unrecognised defaults to cached: over-caching costs a stale ack,
  // under-caching moves the joystick twice.
  TEST_ASSERT_TRUE(P::changesState("wibble", ""));
  TEST_ASSERT_TRUE(P::changesState("cal", ""));
}

// ==================================================== board behaviour, direct

static std::string sendOne(ProtocolBoard &board, const std::string &line) {
  board.clearOutput();
  board.feed((line + "\n").c_str());
  return board.outputCount() > 0 ? std::string(board.output(0)) : std::string();
}

void test_partial_lines_are_buffered(void) {
  Rig rig;
  ProtocolBoard &board = rig.board;
  board.feed("pi");
  TEST_ASSERT_EQUAL_UINT8(0, board.outputCount());
  board.feed("ng;");
  TEST_ASSERT_EQUAL_UINT8(0, board.outputCount());
  board.feed("7\n");
  TEST_ASSERT_EQUAL_UINT8(1, board.outputCount());
  TEST_ASSERT_EQUAL_STRING("ack;7", board.output(0));
}

void test_a_repeated_relative_move_is_not_re_executed(void) {
  // MOVE is the only non-idempotent command in the set, and the sole reason the
  // retransmission cache exists.
  Rig rig;
  ProtocolBoard &board = rig.board;
  sendOne(board, "hv;1;on");
  sendOne(board, "home;2");
  board.advance(50000);
  sendOne(board, "arm;3");

  TEST_ASSERT_EQUAL_STRING("ack;4", sendOne(board, "move;4;4.0").c_str());
  board.advance(50000);
  double once = board.angle();

  TEST_ASSERT_EQUAL_STRING("ack;4", sendOne(board, "move;4;4.0").c_str());
  board.advance(50000);
  // Float, not double: Unity is built here without double-precision asserts,
  // which is also what test_conversions.cpp uses.
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, static_cast<float>(once),
                           static_cast<float>(board.angle()));
}

void test_reads_are_re_executed_and_return_fresh_state(void) {
  Rig rig;
  ProtocolBoard &board = rig.board;
  std::string before = sendOne(board, "status;9");
  sendOne(board, "hv;1;on");
  std::string after = sendOne(board, "status;9");  // same seq, same content
  TEST_ASSERT_TRUE(before != after);
}

void test_any_well_formed_command_resets_the_watchdog(void) {
  // Not just PING: liveness is proven by a frame arriving. This is what makes a
  // host polling STATUS during a long move survivable.
  Rig rig;
  ProtocolBoard &board = rig.board;
  sendOne(board, "hv;1;on");
  sendOne(board, "home;2");
  board.advance(50000);
  sendOne(board, "arm;3");

  for (int i = 0; i < 5; ++i) {
    board.advance(90000);
    char line[32];
    snprintf(line, sizeof(line), "status;%d", 10 + i);
    sendOne(board, line);
  }
  TEST_ASSERT_EQUAL_UINT32(0, board.faultMask());
  TEST_ASSERT_EQUAL_STRING("ARMED", board.state());
}

void test_malformed_frames_do_not_hold_the_watchdog_open(void) {
  // A host with a wedged serial driver is not usefully alive. The refresh sits
  // after the badframe branch deliberately.
  Rig rig;
  ProtocolBoard &board = rig.board;
  sendOne(board, "hv;1;on");
  sendOne(board, "home;2");
  board.advance(50000);
  sendOne(board, "arm;3");

  // The margins here are what make this discriminating. Armed at t=50000 with
  // a 100 ms timeout, the deadline is 150000. Advance to just inside it, send
  // garbage, then step just past it: if garbage refreshed the watchdog the
  // deadline would have moved to 240000 and no fault would fire.
  //
  // An earlier version of this test advanced a further full timeout at the end,
  // which tripped the watchdog either way and so passed against a board that
  // did refresh on garbage. Found by mutation-testing this file 2026-09-03.
  board.advance(90000);  // t = 140000, still inside the deadline
  TEST_ASSERT_EQUAL_UINT32(0, board.faultMask());

  std::string reply = sendOne(board, "!!! not a command");
  TEST_ASSERT_EQUAL_STRING("nak;?;badframe", reply.c_str());

  board.advance(20000);  // t = 160000, past the original 150000 deadline
  TEST_ASSERT_TRUE_MESSAGE((board.faultMask() & 1u) != 0u,
                           "a malformed frame refreshed the host-loss watchdog");
}

void test_reported_limits_agree_with_the_backend(void) {
  // LIMITS is "what the board will enforce", so it is a protocol fact and lives
  // in ProtocolBoard::Config. How fast the mechanism moves is an implementation
  // fact and lives in the backend. They are two places holding one number, so
  // the invariant is asserted rather than hoped for: a board reporting a v_max
  // it cannot achieve would make every host-side duration estimate wrong.
  ProtocolBoard::Config board_cfg;
  SimulatedMotion::Config motion_cfg;
  TEST_ASSERT_EQUAL_FLOAT(static_cast<float>(board_cfg.v_max),
                          static_cast<float>(motion_cfg.v_max_deg_s));
}

void test_arm_is_refused_when_the_driver_refuses_to_enable(void) {
  // The vsense read-back path at ARM specifically.
  //
  // Since HOME now enables the driver too, reaching ARM at all means enabling
  // worked once. So the case still worth testing is a driver that succeeded for
  // homing and then refuses -- a transient, which on real hardware is what a
  // CHOPCONF read-back that comes back wrong after a recovery looks like.
  class FailsSecondEnable : public SimulatedMotion {
  public:
    bool enableDriver() override {
      calls += 1;
      return calls < 2;
    }
    int calls = 0;
  };
  FailsSecondEnable motion;
  ProtocolBoard board(motion, Rig::corpusConfig());
  board.boot();
  board.clearOutput();

  sendOne(board, "hv;1;on");
  TEST_ASSERT_EQUAL_STRING("ack;2", sendOne(board, "home;2").c_str());
  board.advance(50000);

  board.clearOutput();
  board.feed("arm;3\n");
  // The nak comes first, then the FAULT event: the refusal is the answer to the
  // command and the fault is what the board did about it.
  TEST_ASSERT_EQUAL_UINT8(2, board.outputCount());
  TEST_ASSERT_EQUAL_STRING("nak;3;faulted", board.output(0));
  TEST_ASSERT_EQUAL_STRING("ev;FAULT;50000;code=7;detail=vsense_not_1",
                           board.output(1));
  TEST_ASSERT_EQUAL_STRING("FAULTED", board.state());
}

void test_an_animal_pull_diverges_the_two_positions(void) {
  // The whole reason STATUS reports `angle` and `xactual` separately. In
  // FREEWHEEL the encoder moves and the ramp generator does not, and the resync
  // happens only on the way out.
  Rig rig;
  ProtocolBoard &board = rig.board;
  sendOne(board, "hv;1;on");
  sendOne(board, "home;2");
  board.advance(50000);
  sendOne(board, "arm;3");
  sendOne(board, "brake;4;0"); // no brake on this board: release only

  rig.motion.setEncoderAngleDeg(-7.5);  // the animal pulls
  TEST_ASSERT_EQUAL_STRING("ack;5;state=RELEASED;angle=-7.5;xactual=0.0;torque="
                           ";brake=;homed=1;limit=0;faults=0;hv=1;t=50000",
                           sendOne(board, "status;5").c_str());

  // Leaving compliance reconciles them, and only then.
  sendOne(board, "stop;6");
  TEST_ASSERT_EQUAL_STRING("ack;7;state=ARMED;angle=-7.5;xactual=-7.5;torque=;"
                           "brake=;homed=1;limit=0;faults=0;hv=1;t=50000",
                           sendOne(board, "status;7").c_str());
}

// A backend that fails the way real motion does: asynchronously, with no host
// command in flight.
class FaultingMotion : public SimulatedMotion {
public:
  void injectFault(uint8_t bit, const char *detail) {
    bit_ = bit;
    detail_ = detail;
    pending_ = true;
  }
  bool consumeFault(uint8_t *bit, const char **detail) override {
    if (!pending_) {
      return false;
    }
    pending_ = false;
    *bit = bit_;
    *detail = detail_;
    return true;
  }

private:
  bool pending_ = false;
  uint8_t bit_ = 0;
  const char *detail_ = "";
};

void test_a_backend_fault_becomes_a_latched_fault_and_an_event(void) {
  FaultingMotion motion;
  ProtocolBoard board(motion, Rig::corpusConfig());
  board.boot();
  board.clearOutput();

  sendOne(board, "hv;1;on");
  sendOne(board, "home;2");
  board.advance(50000);
  sendOne(board, "arm;3");
  sendOne(board, "moveto;4;9.0");

  motion.injectFault(mouse_joystick::fault::kDriverFault, "s2ga");
  board.clearOutput();
  board.advance(1000);

  TEST_ASSERT_EQUAL_UINT8(1, board.outputCount());
  TEST_ASSERT_EQUAL_STRING("ev;FAULT;51000;code=1;detail=s2ga", board.output(0));
  TEST_ASSERT_EQUAL_STRING("FAULTED", board.state());
  // Faults drop HV_EN, per the board's safety rules.
  TEST_ASSERT_FALSE(board.hvOn());

  // Motion is refused while faulted...
  TEST_ASSERT_EQUAL_STRING("nak;5;faulted", sendOne(board, "moveto;5;0.0").c_str());

  // ...but CLRFAULT must succeed. This test previously asserted `nak;latched`,
  // encoding as intended behaviour what turned out on hardware to be a deadlock:
  // the remedy for a motion fault is to move, and a fault that refuses to clear
  // makes that impossible without a reset. If the cause is still present the
  // next command that touches it faults again, which is recoverable and just as
  // loud.
  TEST_ASSERT_EQUAL_STRING("ack;6", sendOne(board, "clrfault;6").c_str());
  TEST_ASSERT_EQUAL_STRING("IDLE", board.state());
}

void test_a_backend_fault_outranks_a_completion(void) {
  // If the driver reports a short to ground on the same poll that the move
  // finishes, the fault is what happened -- a TARGET event would be a lie.
  FaultingMotion motion;
  ProtocolBoard board(motion, Rig::corpusConfig());
  board.boot();
  board.clearOutput();

  sendOne(board, "hv;1;on");
  sendOne(board, "home;2");
  board.advance(50000);
  sendOne(board, "arm;3");
  sendOne(board, "moveto;4;9.0");

  motion.injectFault(mouse_joystick::fault::kOverTravel, "over_travel");
  board.clearOutput();
  board.advance(50000);  // the move would otherwise complete here

  // Exactly one line, and it is the FAULT rather than the TARGET. The
  // timestamp is the motion deadline because the scheduler steps to it, which
  // is precisely the poll where both things wanted to happen at once.
  TEST_ASSERT_EQUAL_UINT8(1, board.outputCount());
  TEST_ASSERT_EQUAL_STRING("ev;FAULT;100000;code=2;detail=over_travel",
                           board.output(0));
}

void test_a_rail_that_does_not_come_up_is_a_fault_not_a_nak(void) {
  // Asking for the rail and not getting it is not a precondition failure -- the
  // attempt was made and the hardware answered. On these boards this is the
  // live failure mode, not a hypothetical: `R203` is fitted on every unit and a
  // single HV_EN assertion cannot charge the 208 uF bulk.
  class NoRail : public SimulatedMotion {
  public:
    bool setHvEnabled(bool on) override { return !on; }  // off works, on does not
  };
  NoRail motion;
  ProtocolBoard board(motion, Rig::corpusConfig());
  board.boot();
  board.clearOutput();

  board.feed("hv;1;on\n");
  TEST_ASSERT_EQUAL_UINT8(2, board.outputCount());
  TEST_ASSERT_EQUAL_STRING("nak;1;faulted", board.output(0));
  TEST_ASSERT_EQUAL_STRING("ev;FAULT;0;code=6;detail=hv_did_not_come_up",
                           board.output(1));
  TEST_ASSERT_FALSE(board.hvOn());

  // And HV OFF is still permitted while faulted: the board never requires a
  // message in order to become safe, and this is the message that makes it
  // safer.
  TEST_ASSERT_EQUAL_STRING("ack;2", sendOne(board, "hv;2;off").c_str());
}

void test_the_default_serial_cannot_be_mistaken_for_a_real_one(void) {
  // Regression, found on hardware 2026-09-03: the first READY this firmware
  // emitted carried `serial=0000000000000000`, the corpus placeholder, because
  // the example never called setSerial(). Two boards would have reported the
  // same value and a host's identity check would have PASSED against the wrong
  // one -- silently, which is the worst way for an identity check to fail.
  //
  // So the default must be impossible to mistake for a flash unique id: 16 hex
  // characters. Anything that fails that test is fine; a plausible one is not.
  ProtocolBoard::Config cfg;
  const char *serial = cfg.serial;
  bool looks_like_a_real_id = strlen(serial) == 16;
  for (const char *p = serial; looks_like_a_real_id && *p != '\0'; ++p) {
    const bool hex = (*p >= '0' && *p <= '9') || (*p >= 'A' && *p <= 'F') ||
                     (*p >= 'a' && *p <= 'f');
    if (!hex) {
      looks_like_a_real_id = false;
    }
  }
  TEST_ASSERT_FALSE_MESSAGE(looks_like_a_real_id,
                            "the default serial looks like a real board id, so "
                            "forgetting setSerial() would fail silently");
}

void test_status_reports_the_limit_switch_from_the_backend(void) {
  // Regression, found 2026-09-03 while reading `status` on hardware: this field
  // was a member that nothing ever assigned, so `limit` was a hardcoded 0. A
  // host could have used it to decide homing was safe to start. A status field
  // that cannot change is worse than an absent one.
  class SwitchClosed : public SimulatedMotion {
  public:
    bool limitAsserted() const override { return true; }
  };
  SwitchClosed motion;
  ProtocolBoard board(motion, Rig::corpusConfig());
  board.boot();
  board.clearOutput();

  TEST_ASSERT_EQUAL_STRING("ack;1;state=IDLE;angle=0.0;xactual=0.0;torque=;"
                           "brake=;homed=0;limit=1;faults=0;hv=0;t=0",
                           sendOne(board, "status;1").c_str());
}

// Records what the protocol layer asked of the hardware, so safety rules can be
// asserted rather than assumed.
class RecordingMotion : public SimulatedMotion {
public:
  bool setHvEnabled(bool on) override {
    hv_calls += 1;
    hv_last = on;
    return SimulatedMotion::setHvEnabled(on);
  }
  bool enableDriver() override {
    enable_calls += 1;
    return SimulatedMotion::enableDriver();
  }
  int hv_calls = 0;
  bool hv_last = false;
  int enable_calls = 0;
};

void test_home_energises_the_driver_it_is_about_to_use(void) {
  // Found on hardware 2026-09-03: ARM is refused unless homed, and ARM is what
  // enables the driver -- so homing commanded a ramp with the driver disabled
  // and the shaft never moved. HOME must energise for itself.
  RecordingMotion motion;
  ProtocolBoard board(motion, Rig::corpusConfig());
  board.boot();
  board.clearOutput();

  sendOne(board, "hv;1;on");
  TEST_ASSERT_EQUAL_INT(0, motion.enable_calls);
  TEST_ASSERT_EQUAL_STRING("ack;2", sendOne(board, "home;2").c_str());
  TEST_ASSERT_EQUAL_INT_MESSAGE(1, motion.enable_calls,
                                "HOME did not enable the driver");
}

void test_a_fault_drops_the_rail_and_not_just_the_flag(void) {
  // `status` reporting hv=0 while the 36 V rail is still up is worse than
  // reporting the unsafe state honestly. Found on hardware 2026-09-03.
  RecordingMotion motion;
  ProtocolBoard board(motion, Rig::corpusConfig());
  board.boot();
  board.clearOutput();

  sendOne(board, "hv;1;on");
  TEST_ASSERT_TRUE(motion.hv_last);
  sendOne(board, "home;2");
  board.advance(50000);
  sendOne(board, "arm;3");

  const int before = motion.hv_calls;
  board.advance(200000);  // let the host-loss watchdog expire

  TEST_ASSERT_EQUAL_STRING("FAULTED", board.state());
  TEST_ASSERT_TRUE_MESSAGE(motion.hv_calls > before,
                           "the fault never asked the backend to drop the rail");
  TEST_ASSERT_FALSE_MESSAGE(motion.hv_last,
                            "the fault left the rail enabled");
  TEST_ASSERT_FALSE(board.hvOn());
}

void test_home_is_refused_when_the_driver_will_not_enable(void) {
  SimulatedMotion::Config motion_cfg;
  motion_cfg.driver_enable_ok = false;
  SimulatedMotion motion(motion_cfg);
  ProtocolBoard board(motion, Rig::corpusConfig());
  board.boot();
  board.clearOutput();

  sendOne(board, "hv;1;on");
  board.clearOutput();
  board.feed("home;2\n");
  TEST_ASSERT_EQUAL_UINT8(2, board.outputCount());
  TEST_ASSERT_EQUAL_STRING("nak;2;faulted", board.output(0));
  TEST_ASSERT_EQUAL_STRING("ev;FAULT;0;code=7;detail=vsense_not_1",
                           board.output(1));
}

void test_leaving_released_re_energises_the_driver(void) {
  // Found on hardware 2026-09-03: `freewheel` releases the driver and every
  // path out of it resynced and set ARMED without re-enabling. The ramp then
  // advanced with no torque and the follow check faulted `not_following`.
  //
  // Checked for all three exits, because they were three separate omissions.
  const char *exits[] = {"moveto;30;-3.0", "stop;30", "disarm;30"};
  for (const char *exit : exits) {
    RecordingMotion motion;
    ProtocolBoard board(motion, Rig::corpusConfig());
    board.boot();
    board.clearOutput();

    sendOne(board, "hv;1;on");
    sendOne(board, "home;2");
    board.advance(50000);
    sendOne(board, "arm;3");
    TEST_ASSERT_EQUAL_STRING("ack;4", sendOne(board, "brake;4;0").c_str());

    const int before = motion.enable_calls;
    sendOne(board, exit);
    TEST_ASSERT_EQUAL_INT_MESSAGE(before + 1, motion.enable_calls, exit);
    TEST_ASSERT_FALSE_MESSAGE(board.faultMask() != 0, exit);
  }
}

static void armAndRelease(ProtocolBoard &board) {
  sendOne(board, "cfg;1;wdt_ms=500");
  sendOne(board, "hv;2;on");
  sendOne(board, "home;3");
  board.advance(50000);
  sendOne(board, "arm;4");
  sendOne(board, "brake;5;0"); // no brake on this board: release only
}

void test_a_pull_past_the_threshold_emits_one_pull_event(void) {
  // WATCH (protocol v1): thresholds are excursions from the angle at RELEASED,
  // pull-positive; one WATCH yields exactly one outcome.
  Rig rig;
  ProtocolBoard &board = rig.board;
  armAndRelease(board);
  TEST_ASSERT_EQUAL_STRING(
      "ack;6", sendOne(board, "watch;6;pull=5;push=5;timeout=300").c_str());

  // Not yet past it.
  rig.motion.setEncoderAngleDeg(4.0);
  board.clearOutput();
  board.advance(1000);
  TEST_ASSERT_EQUAL_UINT8(0, board.outputCount());

  // The animal pulls past.
  rig.motion.setEncoderAngleDeg(6.5);
  board.clearOutput();
  board.advance(1000);
  TEST_ASSERT_EQUAL_UINT8(1, board.outputCount());
  TEST_ASSERT_EQUAL_STRING("ev;PULL;52000;angle=6.5;torque=", board.output(0));

  // Latched: one WATCH yields one outcome, not a stream while it sits past.
  rig.motion.setEncoderAngleDeg(7.5);
  board.clearOutput();
  board.advance(1000);
  TEST_ASSERT_EQUAL_UINT8(0, board.outputCount());
}

void test_the_pull_direction_follows_the_configured_sign(void) {
  // Travel is mirrored between the left and right joysticks. The board's
  // `pull_sign` hides that: with -1 a pull is a decreasing angle, and the
  // excursions the trace reports stay pull-positive.
  SimulatedMotion motion;
  ProtocolBoard::Config cfg = Rig::corpusConfig();
  cfg.pull_sign = -1.0;
  ProtocolBoard board(motion, cfg);
  board.boot();
  board.clearOutput();
  armAndRelease(board);
  sendOne(board, "watch;6;pull=5;push=5;timeout=300");

  motion.setEncoderAngleDeg(6.0); // increasing angle: a push on this side
  board.clearOutput();
  board.advance(1000); // the first service after WATCH is at its own time
  TEST_ASSERT_EQUAL_UINT8(1, board.outputCount());
  TEST_ASSERT_EQUAL_STRING("ev;PUSH;50000;angle=6.0;torque=", board.output(0));

  board.clearOutput();
  sendOne(board, "stop;7"); // at 51000: the summary carries the outcome's times
  TEST_ASSERT_EQUAL_STRING(
      "ev;TRIAL;51000;outcome=PUSH;t_rel=50000;t_touch=;t_move=50000;t_out="
      "50000;a0=0.0;a_peak=-6.0;a_end=-6.0;tq_peak=;tq_out=;dur_thr=0",
      board.output(0));
  TEST_ASSERT_EQUAL_STRING("ack;7", board.output(1));
}

void test_a_commanded_move_across_the_threshold_is_not_a_pull(void) {
  // Reporting the board's own motion as a pull would put a fabricated event
  // into a behavioural dataset: the classifier only runs in RELEASED, and
  // leaving RELEASED disarms it (TRIAL outcome=NONE).
  Rig rig;
  ProtocolBoard &board = rig.board;
  armAndRelease(board);
  sendOne(board, "watch;6;pull=5;push=5;timeout=300");
  board.clearOutput();
  board.feed("moveto;7;-9.0\n");
  TEST_ASSERT_EQUAL_UINT8(2, board.outputCount());
  TEST_ASSERT_EQUAL_STRING(
      "ev;TRIAL;50000;outcome=NONE;t_rel=50000;t_touch=;t_move=;t_out=;a0=0.0;"
      "a_peak=0.0;a_end=0.0;tq_peak=;tq_out=;dur_thr=0",
      board.output(0));
  TEST_ASSERT_EQUAL_STRING("ack;7", board.output(1));
  board.clearOutput();
  board.advance(60000);

  // Exactly one event, and it is TARGET rather than PULL.
  TEST_ASSERT_EQUAL_UINT8(1, board.outputCount());
  TEST_ASSERT_EQUAL_STRING("ev;TARGET;100000;angle=-9.0", board.output(0));
}

void test_angle_output_verbs_work_when_the_board_has_the_output(void) {
  // ANGOUT/ANGSCALE answer badverb on a board without the output (the corpus
  // pins that); with `has_angout` they are real, and the application drives
  // the DAC from the accessors.
  SimulatedMotion motion;
  ProtocolBoard::Config cfg = Rig::corpusConfig();
  cfg.has_angout = true;
  ProtocolBoard board(motion, cfg);
  board.boot();
  board.clearOutput();
  TEST_ASSERT_EQUAL_STRING(
      "ack;1;hw=rev0;serial=0000000000000000;fw=0.0.0;proto=2;axes=1;caps="
      "motion+home+instr+hv+watch+fetch+stream+cfg+angout;rxbuf=200;boot=por;"
      "crumb=0",
      sendOne(board, "id;1").c_str());
  TEST_ASSERT_FALSE(board.angoutOn());
  TEST_ASSERT_EQUAL_STRING("ack;2", sendOne(board, "angout;2;on").c_str());
  TEST_ASSERT_TRUE(board.angoutOn());
  TEST_ASSERT_FALSE(
      board.angleOutputValid()); // not homed: parked at the fault level
  TEST_ASSERT_EQUAL_STRING("nak;3;outofrange",
                           sendOne(board, "angscale;3;20;-20").c_str());
  TEST_ASSERT_EQUAL_STRING("ack;4",
                           sendOne(board, "angscale;4;-15;15").c_str());
  TEST_ASSERT_EQUAL_FLOAT(-15.0f, static_cast<float>(board.angleScaleLo()));
  TEST_ASSERT_EQUAL_FLOAT(15.0f, static_cast<float>(board.angleScaleHi()));
  sendOne(board, "hv;5;on");
  sendOne(board, "home;6");
  board.advance(50000);
  TEST_ASSERT_TRUE(board.angleOutputValid());
}

void test_every_fault_this_board_raises_can_be_cleared(void) {
  // Found on hardware 2026-09-03: a `not_following` fault at the mechanical stop
  // refused CLRFAULT with `latched` forever, and the remedy for being against a
  // stop is to move -- which is refused while faulted. Only a reset recovered
  // it, and it silently invalidated two measurements that looked like they ran.
  //
  // An unclearable fault whose only remedy is the action it blocks is a
  // deadlock, so nothing here may latch permanently.
  FaultingMotion motion;
  ProtocolBoard board(motion, Rig::corpusConfig());
  board.boot();
  board.clearOutput();

  sendOne(board, "hv;1;on");
  sendOne(board, "home;2");
  board.advance(50000);
  sendOne(board, "arm;3");

  motion.injectFault(mouse_joystick::fault::kOverTravel, "not_following");
  board.advance(1000);
  TEST_ASSERT_EQUAL_STRING("FAULTED", board.state());

  TEST_ASSERT_EQUAL_STRING_MESSAGE(
      "ack;4", sendOne(board, "clrfault;4").c_str(),
      "a motion fault could not be cleared, so recovery needs a reset");
  TEST_ASSERT_EQUAL_STRING("IDLE", board.state());
  TEST_ASSERT_EQUAL_UINT32(0, board.faultMask());

  // And the board is usable again from there.
  TEST_ASSERT_EQUAL_STRING("ack;5", sendOne(board, "hv;5;on").c_str());
  TEST_ASSERT_EQUAL_STRING("ack;6", sendOne(board, "home;6").c_str());
}

void test_a_reply_is_never_silently_truncated(void) {
  // A host cannot tell a truncated reply from a different one, and this
  // protocol is compared byte for byte.
  P::Line line;
  line.clear();
  for (int i = 0; i < 40; ++i) {
    line.kvStr("key", "value");
  }
  TEST_ASSERT_TRUE(line.overflowed());
  TEST_ASSERT_TRUE(line.length() <= P::kMaxLine);
}

// ================================================= 2026-10-01 review bugs

static std::string lastOutput(const ProtocolBoard &board) {
  const uint8_t n = board.outputCount();
  return n == 0 ? std::string() : std::string(board.output(n - 1));
}

namespace {
uint64_t g_fake_wall_us = 0;
uint64_t fakeWallClock() { return g_fake_wall_us; }
} // namespace

void test_a_blocking_hv_on_does_not_trip_the_watchdog_it_arms(void) {
  // On pcb v1.0 `hv on` blocks 185 ms in the rail's soft start, longer than
  // the 100 ms host timeout. The deadline armed by it must count from the
  // end of the call, not from the line's arrival (Gate E, 2026-10-06 11:16).
  Rig rig;
  rig.board.setWallClock(&fakeWallClock);
  g_fake_wall_us = 185000; // the wall when the backend call returns
  rig.board.feed("hv;1;on\n");
  TEST_ASSERT_EQUAL_STRING("ack;1", rig.board.output(0));
  rig.board.clearOutput();
  rig.board.advanceTo(190000); // the firmware's advance after the line
  TEST_ASSERT_EQUAL_UINT8(0, rig.board.outputCount());
  rig.board.advanceTo(284999);
  TEST_ASSERT_EQUAL_UINT8(0, rig.board.outputCount());
  rig.board.advanceTo(285000); // 100 ms after the rail came up
  TEST_ASSERT_EQUAL_STRING("ev;FAULT;285000;code=0;detail=host_timeout",
                           lastOutput(rig.board).c_str());
}

void test_time_blocked_in_a_command_is_credited_to_the_watchdog(void) {
  // Gate E 11:25: `home` blocked ~130 ms with the watchdog running from
  // `hv on`; the heartbeats that arrived meanwhile were read only after the
  // deadline check. The blocked time moves the deadline.
  Rig rig;
  rig.board.setWallClock(&fakeWallClock);
  g_fake_wall_us = 185000;
  rig.board.feed("hv;1;on\n"); // deadline 285000
  rig.board.advanceTo(190000);
  g_fake_wall_us = 340000; // `home` returns 150 ms after its arrival
  rig.board.feed("home;2\n"); // refreshed to 290000 on arrival, +150000
  rig.board.clearOutput();
  rig.board.advanceTo(345000);
  rig.board.clearOutput(); // HOMED lands here; not the point
  rig.board.advanceTo(439999);
  TEST_ASSERT_EQUAL_UINT8(0, rig.board.outputCount());
  rig.board.advanceTo(440000);
  TEST_ASSERT_EQUAL_STRING("ev;FAULT;440000;code=0;detail=host_timeout",
                           lastOutput(rig.board).c_str());
}

void test_event_timestamps_do_not_wrap_at_32_bits(void) {
  // Bug 1: `t` was cast to uint32_t and wrapped at 71.6 minutes, inside a
  // session. The protocol carries the board's 64-bit microsecond clock whole.
  Rig rig;
  rig.board.advance(4294967296ULL); // 2^32 us: 71.6 minutes into the session,
                                    // rail off (the watchdog runs with the rail)
  rig.board.feed("hv;1;on\n");
  rig.board.clearOutput();
  rig.board.feed("home;2\n");
  rig.board.clearOutput();
  rig.board.advance(50000);
  TEST_ASSERT_EQUAL_STRING("ev;HOMED;4295017296;angle=0.0",
                           lastOutput(rig.board).c_str());
}

void test_a_home_timeout_shorter_than_the_home_faults_when_it_expires(void) {
  // Bug 2: the request used to be compared with the backend's bound up front,
  // which faulted immediately. It is a deadline on the running home.
  Rig rig;
  rig.board.feed("hv;1;on\n");
  rig.board.clearOutput();
  rig.board.feed("home;2;30\n"); // 30 ms; the simulated home takes 50 ms
  TEST_ASSERT_EQUAL_UINT8(1, rig.board.outputCount());
  TEST_ASSERT_EQUAL_STRING("ack;2", rig.board.output(0));
  rig.board.clearOutput();
  rig.board.advance(29999);
  TEST_ASSERT_EQUAL_UINT8(0, rig.board.outputCount()); // still homing
  rig.board.advance(1);
  TEST_ASSERT_EQUAL_STRING("ev;FAULT;30000;code=3;detail=home_timeout",
                           lastOutput(rig.board).c_str());
  TEST_ASSERT_EQUAL_STRING("FAULTED", rig.board.state());
  TEST_ASSERT_FALSE(rig.motion.isMoving());
}

void test_a_home_timeout_longer_than_the_home_is_not_a_fault(void) {
  Rig rig;
  // The rail stays on for two seconds, so the host keeps the heartbeat
  // (V1.12: the watchdog runs with the rail), widened first to 500 ms.
  rig.board.feed("cfg;1;wdt_ms=500\n");
  rig.board.feed("hv;2;on\n");
  rig.board.clearOutput();
  rig.board.feed("home;3;2000\n"); // under the old rule this faulted at once
  TEST_ASSERT_EQUAL_STRING("ack;3", rig.board.output(0));
  rig.board.clearOutput();
  rig.board.advance(50000);
  TEST_ASSERT_EQUAL_STRING("ev;HOMED;50000;angle=0.0",
                           lastOutput(rig.board).c_str());
  rig.board.clearOutput();
  for (unsigned n = 4; n < 9; ++n) { // 2 s: the deadline was disarmed by completion
    rig.board.advance(400000);
    TEST_ASSERT_EQUAL_UINT8(0, rig.board.outputCount());
    char ping[16];
    snprintf(ping, sizeof(ping), "ping;%u\n", n);
    rig.board.feed(ping);
    rig.board.clearOutput();
  }
  TEST_ASSERT_EQUAL_STRING("HOMED", rig.board.state());
}

void test_a_home_timeout_is_clipped_to_the_firmware_maximum(void) {
  Rig rig;
  rig.board.feed("hv;1;on\n");
  rig.board.clearOutput();
  rig.board.feed("home;2;999999\n"); // 1000 s requested, 8 s is the maximum
  TEST_ASSERT_EQUAL_STRING("ack;2", rig.board.output(0));
  rig.board.clearOutput();
  rig.board.advance(50000); // completes normally; the clip is only visible
  TEST_ASSERT_EQUAL_STRING("ev;HOMED;50000;angle=0.0",
                           lastOutput(rig.board).c_str());
  TEST_ASSERT_EQUAL_UINT32(8000000u, rig.motion.homeTimeoutMaxUs());
}

void test_instrumentation_state_is_seeded_and_driven_by_the_verb(void) {
  // Bug 3: the application drives the buffers from this flag and seeds it with
  // the boot state, so `instr state` reports the port and not the last request.
  Rig rig;
  TEST_ASSERT_TRUE(rig.board.instrumentationOn()); // on at boot, like the port
  rig.board.feed("instr;1;state\n");
  TEST_ASSERT_EQUAL_STRING("ack;1;instr=1", lastOutput(rig.board).c_str());
  rig.board.feed("instr;2;off\n");
  TEST_ASSERT_FALSE(rig.board.instrumentationOn());
}

// ================================================================= the corpus

struct Step {
  std::string kind; // send | advance | angle | torque | expect
  std::string text;
  long value = 0;
  double angle = 0.0;
};

struct Case {
  std::string name;
  int line_no = 0;
  std::set<std::string> caps;  // CAPS: the hardware the board boots with
  std::vector<Step> steps;
};

static std::string trim(const std::string &in) {
  size_t b = in.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) {
    return "";
  }
  size_t e = in.find_last_not_of(" \t\r\n");
  return in.substr(b, e - b + 1);
}

static std::string replaceAll(std::string s, const std::string &from,
                              const std::string &to) {
  size_t at = 0;
  while ((at = s.find(from, at)) != std::string::npos) {
    s.replace(at, from.size(), to);
    at += to.size();
  }
  return s;
}

// The corpus lives next to the Python side, and the test binary's working
// directory depends on how PlatformIO was invoked. Try the plausible roots and
// allow an override, but NEVER skip: a corpus runner that passes when it cannot
// find the corpus is worse than no runner at all.
static std::string findCorpus(void) {
  const char *override_path = getenv("MJ_PROTOCOL_CASES");
  std::vector<std::string> candidates;
  if (override_path != nullptr) {
    candidates.push_back(override_path);
  }
  candidates.push_back("pycontrol/tests/fixtures/protocol_cases.txt");
  candidates.push_back("../pycontrol/tests/fixtures/protocol_cases.txt");
  candidates.push_back("../../pycontrol/tests/fixtures/protocol_cases.txt");
  candidates.push_back("../../../pycontrol/tests/fixtures/protocol_cases.txt");
  candidates.push_back(
      "../../../../pycontrol/tests/fixtures/protocol_cases.txt");

  for (const std::string &path : candidates) {
    std::ifstream probe(path.c_str());
    if (probe.good()) {
      return path;
    }
  }
  return "";
}

static std::vector<Case> g_cases;
static std::string g_corpus_path;

static void loadCorpus(void) {
  g_corpus_path = findCorpus();
  if (g_corpus_path.empty()) {
    return;
  }
  std::ifstream in(g_corpus_path.c_str());
  std::string raw;
  int line_no = 0;
  bool open_case = false;
  Case current;

  while (std::getline(in, raw)) {
    ++line_no;
    std::string line = trim(raw);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    size_t space = line.find(' ');
    std::string keyword = line.substr(0, space);
    std::string rest = space == std::string::npos ? "" : trim(line.substr(space + 1));

    if (keyword == "CASE") {
      current = Case();
      current.name = rest;
      current.line_no = line_no;
      open_case = true;
    } else if (keyword == "END") {
      g_cases.push_back(current);
      open_case = false;
    } else if (!open_case) {
      continue;  // reported as a structural failure below
    } else if (keyword == "CAPS") {
      // Space-separated capability tokens the board boots with.
      size_t pos = 0;
      while (pos < rest.size()) {
        size_t end = rest.find(' ', pos);
        if (end == std::string::npos) {
          end = rest.size();
        }
        if (end > pos) {
          current.caps.insert(rest.substr(pos, end - pos));
        }
        pos = end + 1;
      }
    } else if (keyword == "SEND") {
      Step step;
      step.kind = "send";
      // `\r` is written as an escape so the corpus stays one line per step and
      // survives an editor that strips trailing whitespace.
      step.text = replaceAll(rest, "\\r", "\r");
      current.steps.push_back(step);
    } else if (keyword == "ADVANCE") {
      Step step;
      step.kind = "advance";
      step.value = strtol(rest.c_str(), nullptr, 10);
      current.steps.push_back(step);
    } else if (keyword == "ANGLE") {
      // The animal moves the joystick: the simulated encoder reads this from
      // now on, and the board services once so the classifier sees it.
      Step step;
      step.kind = "angle";
      step.angle = strtod(rest.c_str(), nullptr);
      current.steps.push_back(step);
    } else if (keyword == "TORQUE") {
      // The torque the bridge reads from now on (a board with CAPS torque);
      // the board services once so the classifier sees it.
      Step step;
      step.kind = "torque";
      step.angle = strtod(rest.c_str(), nullptr);
      current.steps.push_back(step);
    } else if (keyword == "SLIP") {
      // The next move lands this far short on the encoder (V1.13); no
      // service step, the stimulus is consumed by the move.
      Step step;
      step.kind = "slip";
      step.angle = strtod(rest.c_str(), nullptr);
      current.steps.push_back(step);
    } else if (keyword == "EXPECT") {
      Step step;
      step.kind = "expect";
      step.text = rest;
      current.steps.push_back(step);
    }
  }
}

void test_the_corpus_was_found(void) {
  TEST_ASSERT_MESSAGE(!g_corpus_path.empty(),
                      "protocol_cases.txt not found; set MJ_PROTOCOL_CASES. "
                      "A corpus runner that passes without the corpus is "
                      "worse than no runner.");
}

void test_the_corpus_is_not_empty(void) {
  // A corpus that silently parsed to nothing would make every case vacuous.
  TEST_ASSERT_TRUE_MESSAGE(g_cases.size() >= 15, "corpus parsed to too few cases");
  for (const Case &c : g_cases) {
    TEST_ASSERT_TRUE_MESSAGE(!c.steps.empty(), c.name.c_str());
  }
}

void test_every_case_has_a_unique_name(void) {
  // Failures are reported by name, so duplicates would be ambiguous.
  std::set<std::string> seen;
  for (const Case &c : g_cases) {
    TEST_ASSERT_TRUE_MESSAGE(seen.insert(c.name).second, c.name.c_str());
  }
}

void test_every_case_expects_something(void) {
  // A case with no EXPECT asserts nothing and would pass against any board.
  for (const Case &c : g_cases) {
    bool has_expect = false;
    for (const Step &s : c.steps) {
      if (s.kind == "expect") {
        has_expect = true;
      }
    }
    TEST_ASSERT_TRUE_MESSAGE(has_expect, c.name.c_str());
  }
}

void test_the_firmware_reproduces_every_corpus_case(void) {
  for (const Case &c : g_cases) {
    Rig rig(c.caps);
    ProtocolBoard &board = rig.board;

    std::vector<std::string> pending;
    for (size_t index = 0; index < c.steps.size(); ++index) {
      const Step &step = c.steps[index];
      char where[192];
      snprintf(where, sizeof(where), "%s step %zu", c.name.c_str(), index);

      if (step.kind == "send") {
        board.clearOutput();
        board.feed((step.text + "\n").c_str());
        for (uint8_t i = 0; i < board.outputCount(); ++i) {
          pending.push_back(board.output(i));
        }
        TEST_ASSERT_FALSE_MESSAGE(board.outputOverflowed(), where);
      } else if (step.kind == "advance") {
        board.clearOutput();
        board.advance(static_cast<uint64_t>(step.value));
        for (uint8_t i = 0; i < board.outputCount(); ++i) {
          pending.push_back(board.output(i));
        }
        TEST_ASSERT_FALSE_MESSAGE(board.outputOverflowed(), where);
      } else if (step.kind == "angle") {
        // The animal moves the joystick; one service step at the current time
        // so the classifier sees the sample.
        board.clearOutput();
        rig.motion.setEncoderAngleDeg(step.angle);
        board.advance(0);
        for (uint8_t i = 0; i < board.outputCount(); ++i) {
          pending.push_back(board.output(i));
        }
        TEST_ASSERT_FALSE_MESSAGE(board.outputOverflowed(), where);
      } else if (step.kind == "torque") {
        // The bridge reads this from now on; one service step at the current
        // time so the classifier sees the sample.
        board.clearOutput();
        rig.brake_torque.setTorqueNcm(step.angle);
        board.advance(0);
        for (uint8_t i = 0; i < board.outputCount(); ++i) {
          pending.push_back(board.output(i));
        }
        TEST_ASSERT_FALSE_MESSAGE(board.outputOverflowed(), where);
      } else if (step.kind == "slip") {
        rig.motion.setNextMoveSlipDeg(step.angle);
      } else if (step.kind == "expect") {
        TEST_ASSERT_FALSE_MESSAGE(pending.empty(), where);
        std::string actual = pending.front();
        pending.erase(pending.begin());
        TEST_ASSERT_EQUAL_STRING_MESSAGE(step.text.c_str(), actual.c_str(), where);
      }
    }

    char trailing[256];
    snprintf(trailing, sizeof(trailing), "%s produced %zu unexpected trailing line(s): %s",
             c.name.c_str(), pending.size(),
             pending.empty() ? "" : pending.front().c_str());
    TEST_ASSERT_TRUE_MESSAGE(pending.empty(), trailing);
  }
}

int main(int, char **) {
  loadCorpus();

  UNITY_BEGIN();

  RUN_TEST(test_number_formatting_matches_the_python_side);
  RUN_TEST(test_negative_zero_is_never_emitted);
  RUN_TEST(test_number_formatting_is_exact_on_the_binary_value);
  RUN_TEST(test_integer_formatting_without_printf);
  RUN_TEST(test_number_parsing_matches_float);
  RUN_TEST(test_integer_parsing_matches_int);
  RUN_TEST(test_a_late_tick_fills_the_grid_and_fetch_counts_it);
  RUN_TEST(test_sequence_numbers_wrap_the_way_python_does);
  RUN_TEST(test_sequence_numbers_reject_what_python_rejects);
  RUN_TEST(test_decode_recovers_verb_seq_and_args);
  RUN_TEST(test_verbs_decode_case_insensitively);
  RUN_TEST(test_carriage_returns_are_stripped_from_both_ends);
  RUN_TEST(test_bad_frames_report_a_sequence_number_when_one_is_readable);
  RUN_TEST(test_over_length_lines_are_rejected);
  RUN_TEST(test_retransmission_classification_matches_the_python_side);

  RUN_TEST(test_partial_lines_are_buffered);
  RUN_TEST(test_a_repeated_relative_move_is_not_re_executed);
  RUN_TEST(test_reads_are_re_executed_and_return_fresh_state);
  RUN_TEST(test_any_well_formed_command_resets_the_watchdog);
  RUN_TEST(test_a_blocking_hv_on_does_not_trip_the_watchdog_it_arms);
  RUN_TEST(test_time_blocked_in_a_command_is_credited_to_the_watchdog);
  RUN_TEST(test_malformed_frames_do_not_hold_the_watchdog_open);
  RUN_TEST(test_reported_limits_agree_with_the_backend);
  RUN_TEST(test_arm_is_refused_when_the_driver_refuses_to_enable);
  RUN_TEST(test_an_animal_pull_diverges_the_two_positions);
  RUN_TEST(test_a_backend_fault_becomes_a_latched_fault_and_an_event);
  RUN_TEST(test_a_backend_fault_outranks_a_completion);
  RUN_TEST(test_a_rail_that_does_not_come_up_is_a_fault_not_a_nak);
  RUN_TEST(test_the_default_serial_cannot_be_mistaken_for_a_real_one);
  RUN_TEST(test_status_reports_the_limit_switch_from_the_backend);
  RUN_TEST(test_home_energises_the_driver_it_is_about_to_use);
  RUN_TEST(test_a_fault_drops_the_rail_and_not_just_the_flag);
  RUN_TEST(test_home_is_refused_when_the_driver_will_not_enable);
  RUN_TEST(test_leaving_released_re_energises_the_driver);
  RUN_TEST(test_a_pull_past_the_threshold_emits_one_pull_event);
  RUN_TEST(test_the_pull_direction_follows_the_configured_sign);
  RUN_TEST(test_angle_output_verbs_work_when_the_board_has_the_output);
  RUN_TEST(test_a_commanded_move_across_the_threshold_is_not_a_pull);
  RUN_TEST(test_every_fault_this_board_raises_can_be_cleared);
  RUN_TEST(test_a_reply_is_never_silently_truncated);
  RUN_TEST(test_event_timestamps_do_not_wrap_at_32_bits);
  RUN_TEST(test_a_home_timeout_shorter_than_the_home_faults_when_it_expires);
  RUN_TEST(test_a_home_timeout_longer_than_the_home_is_not_a_fault);
  RUN_TEST(test_a_home_timeout_is_clipped_to_the_firmware_maximum);
  RUN_TEST(test_instrumentation_state_is_seeded_and_driven_by_the_verb);

  RUN_TEST(test_the_corpus_was_found);
  RUN_TEST(test_the_corpus_is_not_empty);
  RUN_TEST(test_every_case_has_a_unique_name);
  RUN_TEST(test_every_case_expects_something);
  RUN_TEST(test_the_firmware_reproduces_every_corpus_case);

  return UNITY_END();
}
