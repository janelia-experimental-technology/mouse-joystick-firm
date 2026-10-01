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
//
// 3. **No newlib on the line path** (2026-10-01, measured on the fixture).
//    `snprintf("%.3f")` with software double arithmetic cost about 0.5 ms per
//    number on the Cortex-M0+, which put every protocol line past the 1 kHz
//    sampler's tick (`qp-arduino-example/evidence/2026-10-01-rp2040-joystick-
//    ring-capture/`). Formatting and parsing here are integer fixed-point,
//    exact on the binary value, so <stdio.h> and <stdlib.h> are not included.

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace mouse_joystick {
namespace protocol {

// ------------------------------------------------------------------ constants

constexpr uint8_t kMaxLine =
    200; // MAX_LINE; also reported as `rxbuf` (v1, 2026-10-01: was 120)
constexpr uint16_t kProtocolVersion =
    2; // v1 proposal of 2026-10-01: FREEWHEEL/ARMPULL removed
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
constexpr char kNakNotReleased[] = "notreleased"; // WATCH outside RELEASED

// -------------------------------------------------------------- number format
//
// Mirrors `MouseJoystickProtocol.format_number`: `%.3f`, strip trailing zeros,
// keep at least one decimal place, and never emit `-0.0`.
//
// The `-0.0` case is not hypothetical. Any move that lands on zero from the
// negative side produces it, and it is a valid float that reads as a different
// value to a human and to a naive parser.

// Unsigned 64-bit decimal, for event timestamps. `long` is 32 bits on the
// RP2040, so the board clock (microseconds, 64-bit) must not pass through
// `formatInt`: it wrapped at 71.6 minutes, inside a normal session (found in
// the 2026-10-01 review).
inline void formatU64(uint64_t value, char *out, size_t n) {
  if (n == 0) {
    return;
  }
  char tmp[21];
  size_t len = 0;
  do {
    tmp[len++] = static_cast<char>('0' + (value % 10u));
    value /= 10u;
  } while (value != 0 && len < sizeof(tmp));
  size_t i = 0;
  while (len > 0 && i + 1 < n) {
    out[i++] = tmp[--len];
  }
  out[i] = '\0';
}

// Whole-string copy: the whole of `src` or nothing, never a truncated line
// (a host cannot tell a truncated reply from a different one). Returns false
// and writes "" when `src` does not fit.
inline bool copyText(char *dst, size_t n, const char *src) {
  if (dst == nullptr || n == 0) {
    return false;
  }
  const size_t len = (src != nullptr) ? strlen(src) : 0;
  if (len + 1 > n) {
    dst[0] = '\0';
    return false;
  }
  memcpy(dst, src, len + 1);
  return true;
}

inline void formatInt(long value, char *out, size_t n) {
  if (n == 0) {
    return;
  }
  unsigned long magnitude = (value < 0)
                                ? (0ul - static_cast<unsigned long>(value))
                                : static_cast<unsigned long>(value);
  char tmp[21];
  size_t len = 0;
  do {
    tmp[len++] = static_cast<char>('0' + (magnitude % 10u));
    magnitude /= 10u;
  } while (magnitude != 0 && len < sizeof(tmp));
  size_t i = 0;
  if (value < 0 && i + 1 < n) {
    out[i++] = '-';
  }
  while (len > 0 && i + 1 < n) {
    out[i++] = tmp[--len];
  }
  out[i] = '\0';
}

// The exact decimal rounding `%.3f` performs, without `printf`: the double is
// taken apart into mantissa and binary exponent, scaled by 1000 in integers
// and rounded to nearest with ties to even on the exact binary value, which
// is what newlib's and CPython's `%.3f` both do (so 0.0625 -> "0.062",
// 0.9995 -> "1.0" because the double 0.9995 is a hair above, 1.0005 -> "1.0"
// because it is a hair below). Returns false for nan, inf and magnitudes of
// 2^64 and more, which are not protocol values.
inline bool roundToThousandths(double value, bool *negative,
                               uint64_t *integer_part, uint32_t *thousandths) {
  uint64_t bits = 0;
  memcpy(&bits, &value, sizeof(bits));
  *negative = (bits >> 63) != 0;
  const uint32_t exp_field = static_cast<uint32_t>((bits >> 52) & 0x7FFu);
  uint64_t mant = bits & ((1ull << 52) - 1u);
  if (exp_field == 0x7FFu) {
    return false; // inf or nan
  }
  int32_t exp2 = 0;
  if (exp_field == 0) {
    exp2 = -1074; // subnormal
  } else {
    mant |= (1ull << 52);
    exp2 = static_cast<int32_t>(exp_field) - 1075;
  }
  if (mant == 0) {
    *integer_part = 0;
    *thousandths = 0;
    return true;
  }
  if (exp2 >= 0) {
    // 2^52 and above: an integer. 1000 * mant << exp2 could overflow, so the
    // fraction is simply zero.
    if (exp2 > 11) {
      return false;
    }
    *integer_part = mant << exp2;
    *thousandths = 0;
    return true;
  }
  const uint32_t shift = static_cast<uint32_t>(-exp2);
  const uint64_t scaled = mant * 1000u; // < 2^63: mant < 2^53
  uint64_t q = 0;
  if (shift < 64) {
    q = scaled >> shift;
    const uint64_t remainder = scaled - (q << shift);
    const uint64_t half = 1ull << (shift - 1);
    if (remainder > half || (remainder == half && (q & 1u) != 0)) {
      ++q;
    }
  } // else the value is below 2^-11 and rounds to 0.000
  *integer_part = q / 1000u;
  *thousandths = static_cast<uint32_t>(q % 1000u);
  return true;
}

inline void formatNumber(double value, char *out, size_t n) {
  if (n == 0) {
    return;
  }
  bool negative = false;
  uint64_t integer_part = 0;
  uint32_t thousandths = 0;
  if (!roundToThousandths(value, &negative, &integer_part, &thousandths)) {
    copyText(out, n, (value != value) ? "nan" : (negative ? "-inf" : "inf"));
    return;
  }
  // `-0.0` and anything that rounds to it: same value, one spelling.
  if (integer_part == 0 && thousandths == 0) {
    negative = false;
  }
  char tmp[32];
  size_t len = 0;
  if (negative) {
    tmp[len++] = '-';
  }
  char digits[21];
  formatU64(integer_part, digits, sizeof(digits));
  for (const char *d = digits; *d != '\0'; ++d) {
    tmp[len++] = *d;
  }
  tmp[len++] = '.';
  // Three places, trailing zeros stripped, at least one kept: `%.3f` then
  // Python's rstrip("0") with a zero re-appended after a bare point.
  char frac[4] = {static_cast<char>('0' + thousandths / 100u),
                  static_cast<char>('0' + (thousandths / 10u) % 10u),
                  static_cast<char>('0' + thousandths % 10u), '\0'};
  size_t keep = 3;
  while (keep > 1 && frac[keep - 1] == '0') {
    --keep;
  }
  for (size_t i = 0; i < keep; ++i) {
    tmp[len++] = frac[i];
  }
  tmp[len] = '\0';
  copyText(out, n, tmp);
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

// Strict float parse, matching `float(arg)` for what the protocol carries:
// optional blanks, sign, digits with an optional point, an optional exponent,
// optional blanks, nothing else. Decimal digits are accumulated as an integer
// and scaled by one exact power of ten, which is correctly rounded (the same
// double `strtod` and `float()` give) for up to 15 significant digits and
// exponents within 10^+-22; longer inputs are within an ulp. Not accepted,
// unlike `float()`: "inf", "nan", "1_000"; unlike `strtod`: hex.
inline bool parseNumber(const char *text, double *out) {
  if (text == nullptr || *text == '\0') {
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
  uint64_t mant = 0;
  int32_t exp10 = 0;
  uint8_t significant = 0;
  bool any_digit = false;
  while (*p >= '0' && *p <= '9') {
    any_digit = true;
    if (significant < 15) {
      mant = mant * 10u + static_cast<uint64_t>(*p - '0');
      if (mant != 0) {
        ++significant;
      }
    } else {
      ++exp10; // dropped digit before the point keeps its weight
    }
    ++p;
  }
  if (*p == '.') {
    ++p;
    while (*p >= '0' && *p <= '9') {
      any_digit = true;
      if (significant < 15) {
        mant = mant * 10u + static_cast<uint64_t>(*p - '0');
        if (mant != 0) {
          ++significant;
        }
        --exp10;
      }
      ++p;
    }
  }
  if (!any_digit) {
    return false;
  }
  if (*p == 'e' || *p == 'E') {
    ++p;
    bool exp_negative = false;
    if (*p == '+' || *p == '-') {
      exp_negative = (*p == '-');
      ++p;
    }
    if (*p < '0' || *p > '9') {
      return false;
    }
    int32_t e = 0;
    while (*p >= '0' && *p <= '9') {
      if (e < 100000) {
        e = e * 10 + (*p - '0');
      }
      ++p;
    }
    exp10 += exp_negative ? -e : e;
  }
  while (*p == ' ' || *p == '\t') {
    ++p;
  }
  if (*p != '\0') {
    return false;
  }
  static const double kPow10[] = {
      1e0,  1e1,  1e2,  1e3,  1e4,  1e5,  1e6,  1e7,  1e8,  1e9,  1e10, 1e11,
      1e12, 1e13, 1e14, 1e15, 1e16, 1e17, 1e18, 1e19, 1e20, 1e21, 1e22};
  double value = static_cast<double>(mant); // exact: at most 15 digits
  if (mant != 0) {
    while (exp10 > 22) {
      value *= 1e22;
      exp10 -= 22;
    }
    while (exp10 < -22) {
      value /= 1e22;
      exp10 += 22;
    }
    if (exp10 > 0) {
      value *= kPow10[exp10];
    } else if (exp10 < 0) {
      value /= kPow10[-exp10];
    }
  }
  *out = negative ? -value : value;
  return true;
}

// Strict integer parse, matching `int(arg)`: optional blanks, sign, digits,
// optional blanks. Out-of-range magnitudes clamp to LONG_MAX/LONG_MIN, as
// `strtol` did.
inline bool parseInt(const char *text, long *out) {
  if (text == nullptr || *text == '\0') {
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
    return false;
  }
  const unsigned long limit =
      negative ? (static_cast<unsigned long>(0x7FFFFFFFul) + 1ul)
               : 0x7FFFFFFFul; // 32-bit long on the RP2040; clamped below on
                               // hosts with a wider long as well
  unsigned long magnitude = 0;
  bool clamped = false;
  while (*p >= '0' && *p <= '9') {
    const unsigned long digit = static_cast<unsigned long>(*p - '0');
    if (!clamped) {
      if (magnitude > (limit - digit) / 10ul) {
        clamped = true;
        magnitude = limit;
      } else {
        magnitude = magnitude * 10ul + digit;
      }
    }
    ++p;
  }
  while (*p == ' ' || *p == '\t') {
    ++p;
  }
  if (*p != '\0') {
    return false;
  }
  *out = negative ? -static_cast<long>(magnitude - 1ul) - 1l
                  : static_cast<long>(magnitude);
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
  static const DualMode kDualMode[] = {{"cal", "get"},
                                       {"instr", "state"},
                                       {"hv", "state"},
                                       {"cfg", "get"},
                                       {"tq", "get"}};

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

  static const char *const kReadVerbs[] = {"id",     "ping",   "status",
                                           "limits", "faults", "fetch"};
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

  void appendU64(uint64_t value) {
    char scratch[24];
    formatU64(value, scratch, sizeof(scratch));
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

  void kvU64(const char *key, uint64_t value) {
    appendChar(kDelimiter);
    append(key);
    appendChar('=');
    appendU64(value);
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

  // `t_us` is the board's 64-bit microsecond clock, carried whole: a 32-bit
  // `t` wraps at 71.6 minutes, which is shorter than a session.
  void beginEvent(const char *name, uint64_t t_us) {
    clear();
    append(kEvent);
    appendChar(kDelimiter);
    for (const char *p = name; *p != '\0'; ++p) {
      appendChar(upperAscii(*p));  // uppercase on the wire, in every example
    }
    appendChar(kDelimiter);
    appendU64(t_us);
  }

private:
  char buf_[kMaxLine + 2] = {0};
  uint8_t len_ = 0;
  bool overflow_ = false;
};

}  // namespace protocol
}  // namespace mouse_joystick
