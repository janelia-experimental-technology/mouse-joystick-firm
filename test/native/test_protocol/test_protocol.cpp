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

#include "Protocol.hpp"
#include "ProtocolBoard.hpp"

using mouse_joystick::ProtocolBoard;
namespace P = mouse_joystick::protocol;

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

static ProtocolBoard freshBoard(void) {
  ProtocolBoard::Config cfg;
  cfg.serial = "0000000000000000";  // the corpus header's stated default
  ProtocolBoard board(cfg);
  board.boot();
  board.clearOutput();  // READY drained, as the corpus assumes
  return board;
}

static std::string sendOne(ProtocolBoard &board, const std::string &line) {
  board.clearOutput();
  board.feed((line + "\n").c_str());
  return board.outputCount() > 0 ? std::string(board.output(0)) : std::string();
}

void test_partial_lines_are_buffered(void) {
  ProtocolBoard board = freshBoard();
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
  ProtocolBoard board = freshBoard();
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
  ProtocolBoard board = freshBoard();
  std::string before = sendOne(board, "status;9");
  sendOne(board, "hv;1;on");
  std::string after = sendOne(board, "status;9");  // same seq, same content
  TEST_ASSERT_TRUE(before != after);
}

void test_any_well_formed_command_resets_the_watchdog(void) {
  // Not just PING: liveness is proven by a frame arriving. This is what makes a
  // host polling STATUS during a long move survivable.
  ProtocolBoard board = freshBoard();
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
  ProtocolBoard board = freshBoard();
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

// ================================================================= the corpus

struct Step {
  std::string kind;  // send | advance | expect
  std::string text;
  long value = 0;
};

struct Case {
  std::string name;
  int line_no = 0;
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
    ProtocolBoard::Config cfg;
    cfg.serial = "0000000000000000";
    ProtocolBoard board(cfg);
    board.boot();
    board.clearOutput();  // READY drained

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
  RUN_TEST(test_malformed_frames_do_not_hold_the_watchdog_open);
  RUN_TEST(test_a_reply_is_never_silently_truncated);

  RUN_TEST(test_the_corpus_was_found);
  RUN_TEST(test_the_corpus_is_not_empty);
  RUN_TEST(test_every_case_has_a_unique_name);
  RUN_TEST(test_every_case_expects_something);
  RUN_TEST(test_the_firmware_reproduces_every_corpus_case);

  return UNITY_END();
}
