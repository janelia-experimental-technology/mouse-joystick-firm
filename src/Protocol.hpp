// pyControl UART application protocol — framing codec.
//
// The board half of `../mouse-joystick-pcb/docs/uart-protocol.md`. The host
// half is `pycontrol/devices/mouse_joystick_protocol.py`, and the two are held
// to a shared corpus, `pycontrol/tests/fixtures/protocol_cases.txt`. Until both
// exist the corpus proves nothing: one implementation agreeing with a fixture
// only it has ever been checked against is a tautology.
//
// TWO CONSTRAINTS SHAPE EVERY DECISION HERE.
//
// 1. **Arduino-free.** `platformio.ini`'s `env:native` sets
//    `test_build_src = no` and reaches `src/` only through `-Isrc`, so this is
//    header-only and includes nothing from the core. That is what makes it
//    testable on the host at all, and the host test is the only place the
//    corpus can be run.
//
// 2. **No dynamic allocation.** Every buffer here is fixed and every write is
//    bounds-checked. This board has no way to report a memory problem: no LEDs,
//    no test points, rev 0 has no SWD header, and a heap that fragments during
//    an experiment presents as a joystick that stopped answering. `MAX_LINE` is
//    120 bytes and the deepest structure is a handful of fields, so there is
//    nothing here that needs a heap.
//
// Byte-exactness is the contract. `formatNumber` in particular exists because
// `str(float)` is not guaranteed to agree between CPython and MicroPython, let
// alone with `printf`; all three have to produce the same bytes or the corpus
// comparison means nothing.

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace mouse_joystick {
namespace protocol {

// ------------------------------------------------------------------ constants

constexpr uint8_t kMaxLine = 120;      // MAX_LINE; also reported as `rxbuf`
constexpr uint16_t kProtocolVersion = 1;
constexpr uint8_t kAxisCount = 1;
constexpr uint32_t kSeqModulus = 65536u;
constexpr char kDelimiter = ';';
constexpr char kTerminator = '\n';

// Field budget: verb + seq + args. `cal;9;set;enc_offset;2048` is the longest
// command in the draft at five fields; the margin is for a longer `cal set`.
constexpr uint8_t kMaxFields = 8;
constexpr uint8_t kMaxArgs = kMaxFields - 2;

constexpr char kAck[] = "ack";
constexpr char kNak[] = "nak";
constexpr char kEvent[] = "ev";
constexpr char kSeqUnknown[] = "?";

// Nak reasons, spelled exactly as the draft's table.
constexpr char kNakBadFrame[] = "badframe";
constexpr char kNakBadVerb[] = "badverb";
constexpr char kNakOutOfRange[] = "outofrange";
constexpr char kNakLatched[] = "latched";
constexpr char kNakBadArgs[] = "badargs";
constexpr char kNakNotArmed[] = "notarmed";
constexpr char kNakNotHomed[] = "nothomed";
constexpr char kNakFaulted[] = "faulted";
constexpr char kNakBusy[] = "busy";
constexpr char kNakNoHv[] = "nohv";

// -------------------------------------------------------------- number format
//
// Mirrors `MouseJoystickProtocol.format_number`: `%.3f`, strip trailing zeros,
// keep at least one decimal place, and never emit `-0.0`.
//
// The `-0.0` case is not hypothetical. Any move that lands on zero from the
// negative side produces it, and it is a valid float that reads as a different
// value to a human and to a naive parser.

inline void formatNumber(double value, char *out, size_t n) {
  if (n == 0) {
    return;
  }
  snprintf(out, n, "%.3f", value);

  char *dot = strchr(out, '.');
  if (dot != nullptr) {
    size_t len = strlen(out);
    while (len > 0 && out[len - 1] == '0') {
      out[--len] = '\0';
    }
    if (len > 0 && out[len - 1] == '.') {
      // Python re-appends a zero rather than leaving a bare point, so
      // `20.0` stays `20.0` and never becomes `20.`.
      if (len + 1 < n) {
        out[len] = '0';
        out[len + 1] = '\0';
      }
    }
  }
  if (strcmp(out, "-0.0") == 0) {
    // A commanded zero reached from below. Same value, one spelling.
    snprintf(out, n, "0.0");
  }
}

inline void formatInt(long value, char *out, size_t n) {
  snprintf(out, n, "%ld", value);
}

inline const char *formatBool(bool value) { return value ? "1" : "0"; }

// ------------------------------------------------------------- seq handling
//
// `int(field) % 65536`, and Python's `%` floors, so a negative sequence number
// wraps to the top of the range rather than staying negative. Whitespace is
// tolerated because `int()` tolerates it; a trailing non-digit is not, because
// `int()` rejects it -- `int("12.5")` raises, and so must this.

inline bool normalizeSeq(const char *text, uint16_t *out) {
  if (text == nullptr) {
    return false;
  }
  const char *p = text;
  while (*p == ' ' || *p == '\t') {
    ++p;
  }
  bool negative = false;
  if (*p == '+' || *p == '-') {
    negative = (*p == '-');
    ++p;
  }
  if (*p < '0' || *p > '9') {
    return false;  // `int("")` and `int("notanumber")` both raise.
  }

  // Accumulate modulo the range so a long digit run cannot overflow. Python
  // has arbitrary precision here and would happily reduce a 40-digit number.
  uint32_t value = 0;
  while (*p >= '0' && *p <= '9') {
    value = (value * 10u + static_cast<uint32_t>(*p - '0')) % kSeqModulus;
    ++p;
  }
  while (*p == ' ' || *p == '\t') {
    ++p;
  }
  if (*p != '\0') {
    return false;
  }

  if (negative && value != 0) {
    value = kSeqModulus - value;  // floor modulo, as Python does
  }
  *out = static_cast<uint16_t>(value);
  return true;
}

// Strict float parse, matching `float(arg)`: no trailing garbage allowed.
inline bool parseNumber(const char *text, double *out) {
  if (text == nullptr || *text == '\0') {
    return false;
  }
  char *end = nullptr;
  double value = strtod(text, &end);
  if (end == text) {
    return false;
  }
  while (*end == ' ' || *end == '\t') {
    ++end;
  }
  if (*end != '\0') {
    return false;
  }
  *out = value;
  return true;
}

// Strict integer parse, matching `int(arg)`.
inline bool parseInt(const char *text, long *out) {
  uint16_t unused = 0;
  (void)unused;
  if (text == nullptr || *text == '\0') {
    return false;
  }
  const char *p = text;
  while (*p == ' ' || *p == '\t') {
    ++p;
  }
  if (*p == '+' || *p == '-') {
    ++p;
  }
  if (*p < '0' || *p > '9') {
    return false;
  }
  char *end = nullptr;
  long value = strtol(text, &end, 10);
  while (*end == ' ' || *end == '\t') {
    ++end;
  }
  if (*end != '\0') {
    return false;
  }
  *out = value;
  return true;
}

// ------------------------------------------------------------------- decoding

// A decoded host->board line.
//
// Argument pointers are stored as offsets into `storage` rather than as
// pointers, so a `Command` can be copied without dangling. That is not
// theoretical tidiness: the dispatcher passes it by value in places.
struct Command {
  bool ok = false;         // false => bad frame; answer `nak;<seq>;badframe`
  bool has_seq = false;    // a bad frame may still carry a readable seq
  uint16_t seq = 0;
  char verb[24] = {0};     // lowercased
  uint8_t arg_count = 0;
  char storage[kMaxLine + 1] = {0};
  uint8_t arg_offset[kMaxArgs] = {0};

  const char *arg(uint8_t index) const {
    return index < arg_count ? storage + arg_offset[index] : "";
  }
};

inline char lowerAscii(char c) {
  return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c;
}

inline char upperAscii(char c) {
  return (c >= 'a' && c <= 'z') ? static_cast<char>(c - 'a' + 'A') : c;
}

// The draft says "a leading `\r` is tolerated and ignored". A CRLF peer's `\r`
// actually arrives *trailing*, before the `\n`, so both ends are stripped --
// reading the draft literally would leave the common case broken. Flagged as a
// draft wording bug in the Python side too.
inline bool decodeCommand(const char *line, Command *out) {
  *out = Command();
  if (line == nullptr) {
    return false;
  }

  const char *begin = line;
  const char *end = line + strlen(line);
  while (begin < end && (*begin == '\r' || *begin == '\n')) {
    ++begin;
  }
  while (end > begin && (end[-1] == '\r' || end[-1] == '\n')) {
    --end;
  }

  size_t length = static_cast<size_t>(end - begin);
  if (length == 0 || length > kMaxLine) {
    return false;  // empty or over-length: bad frame, no seq recoverable
  }

  memcpy(out->storage, begin, length);
  out->storage[length] = '\0';

  // Split in place. `";"` splits to two empty fields in Python, and an empty
  // verb is a bad frame -- that is the `SEND ;` case in the corpus.
  const char *fields[kMaxFields] = {nullptr};
  uint8_t count = 0;
  char *cursor = out->storage;
  fields[count++] = cursor;
  while (*cursor != '\0') {
    if (*cursor == kDelimiter) {
      *cursor = '\0';
      if (count >= kMaxFields) {
        return false;  // more fields than this protocol can produce
      }
      fields[count++] = cursor + 1;
    }
    ++cursor;
  }

  size_t verb_len = strlen(fields[0]);
  if (verb_len == 0 || verb_len >= sizeof(out->verb)) {
    return false;
  }
  for (size_t i = 0; i < verb_len; ++i) {
    out->verb[i] = lowerAscii(fields[0][i]);
  }
  out->verb[verb_len] = '\0';

  if (count < 2) {
    return false;  // `ping` with no seq
  }
  uint16_t seq = 0;
  if (!normalizeSeq(fields[1], &seq)) {
    return false;  // `ping;notanumber`
  }
  out->seq = seq;
  out->has_seq = true;

  for (uint8_t i = 2; i < count; ++i) {
    out->arg_offset[out->arg_count++] =
        static_cast<uint8_t>(fields[i] - out->storage);
  }
  out->ok = true;
  return true;
}

// ---------------------------------------------------- retransmission policy
//
// Resolved 2026-09-03 in `../mouse-joystick-pcb/docs/uart-protocol.md`, open
// question 1: the cache holds state-changing commands only. Reads and
// heartbeats are re-executed, because a retried STATUS returning fresh state
// beats a replayed stale one and a retried CLRFAULT re-evaluates whether the
// cause has gone.
//
// The watchdog does NOT depend on this. It is reset by the arrival of a
// well-formed frame, before dispatch, so a cache hit cannot starve it.
//
// Classification is on the PARSED command, not the verb: `cal`, `instr` and
// `hv` are each a read or a write depending on their first argument, and a
// verb-name table gets all three wrong.
inline bool changesState(const char *verb, const char *first_arg) {
  struct DualMode {
    const char *verb;
    const char *read_arg;
  };
  static const DualMode kDualMode[] = {
      {"cal", "get"}, {"instr", "state"}, {"hv", "state"}};

  for (const DualMode &entry : kDualMode) {
    if (strcmp(verb, entry.verb) == 0) {
      if (first_arg == nullptr || *first_arg == '\0') {
        return true;
      }
      // Case-insensitive: `HV STATE` and `hv state` are the same command.
      const char *a = first_arg;
      const char *b = entry.read_arg;
      while (*a != '\0' && *b != '\0' && lowerAscii(*a) == *b) {
        ++a;
        ++b;
      }
      return !(*a == '\0' && *b == '\0');
    }
  }

  static const char *const kReadVerbs[] = {"id", "ping", "status", "limits",
                                           "faults"};
  for (const char *read : kReadVerbs) {
    if (strcmp(verb, read) == 0) {
      return false;
    }
  }
  // Default to cached. Over-caching costs a stale ack; under-caching moves the
  // joystick twice, so a verb added carelessly should be merely suboptimal.
  return true;
}

// ------------------------------------------------------------------- encoding
//
// A bounded line builder. Every append is checked and an overflow is sticky, so
// a caller can build a whole reply and test once at the end rather than at each
// field -- and a truncated line is never silently emitted.

class Line {
public:
  void clear() {
    len_ = 0;
    buf_[0] = '\0';
    overflow_ = false;
  }

  bool overflowed() const { return overflow_; }
  const char *c_str() const { return buf_; }
  uint8_t length() const { return len_; }
  bool empty() const { return len_ == 0; }

  void append(const char *text) {
    if (text == nullptr) {
      return;
    }
    size_t add = strlen(text);
    if (len_ + add > kMaxLine) {
      overflow_ = true;
      return;
    }
    memcpy(buf_ + len_, text, add);
    len_ = static_cast<uint8_t>(len_ + add);
    buf_[len_] = '\0';
  }

  void appendChar(char c) {
    if (len_ + 1u > kMaxLine) {
      overflow_ = true;
      return;
    }
    buf_[len_++] = c;
    buf_[len_] = '\0';
  }

  void appendInt(long value) {
    char scratch[24];
    formatInt(value, scratch, sizeof(scratch));
    append(scratch);
  }

  void appendNumber(double value) {
    char scratch[32];
    formatNumber(value, scratch, sizeof(scratch));
    append(scratch);
  }

  // key=value fields. The draft shows one data-carrying ack and it is
  // `key=value`, so every data-carrying reply here is.
  void kvStr(const char *key, const char *value) {
    appendChar(kDelimiter);
    append(key);
    appendChar('=');
    append(value);
  }

  void kvInt(const char *key, long value) {
    appendChar(kDelimiter);
    append(key);
    appendChar('=');
    appendInt(value);
  }

  void kvNumber(const char *key, double value) {
    appendChar(kDelimiter);
    append(key);
    appendChar('=');
    appendNumber(value);
  }

  void kvBool(const char *key, bool value) {
    kvStr(key, formatBool(value));
  }

  // ---- message heads -------------------------------------------------------

  void beginAck(uint16_t seq) {
    clear();
    append(kAck);
    appendChar(kDelimiter);
    appendInt(seq);
  }

  void nak(uint16_t seq, const char *reason) {
    clear();
    append(kNak);
    appendChar(kDelimiter);
    appendInt(seq);
    appendChar(kDelimiter);
    append(reason);
  }

  // For a frame whose sequence number could not be recovered. The draft writes
  // this position as `<seq or ?>`; answering `?` when the seq was readable
  // would cost the host its correlation, so the caller decides which to use.
  void nakUnknown(const char *reason) {
    clear();
    append(kNak);
    appendChar(kDelimiter);
    append(kSeqUnknown);
    appendChar(kDelimiter);
    append(reason);
  }

  void beginEvent(const char *name, uint32_t t_us) {
    clear();
    append(kEvent);
    appendChar(kDelimiter);
    for (const char *p = name; *p != '\0'; ++p) {
      appendChar(upperAscii(*p));  // uppercase on the wire, in every example
    }
    appendChar(kDelimiter);
    appendInt(static_cast<long>(t_us));
  }

private:
  char buf_[kMaxLine + 2] = {0};
  uint8_t len_ = 0;
  bool overflow_ = false;
};

}  // namespace protocol
}  // namespace mouse_joystick
