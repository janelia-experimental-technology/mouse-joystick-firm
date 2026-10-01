// The WATCH classifier, the 1 kHz capture ring and the per-trial summary.
//
// Protocol v1 (`../mouse-joystick-pcb/docs/uart-protocol.md`, "Protocol v1
// Proposal — 2026-10-01"). Arduino-free and allocation-free like the rest of
// the protocol layer, so the corpus can hold the simulator and this to the
// same bytes. Nothing here knows about time sources: `ProtocolBoard` feeds it
// one (time, angle) pair per service step, which is the firmware's 1 kHz
// tick and the simulator's every scheduled stop, and asks it for the next
// deadline so the clock can be stepped to it exactly.
//
// Angles handed in are absolute encoder degrees; everything the classifier
// decides on is the pull-positive excursion from the angle at RELEASED,
// `(angle - a0) * pull_sign`.

#pragma once

#include <math.h>
#include <stdint.h>

namespace mouse_joystick {

class TrialWatch {
public:
  static constexpr uint16_t kCaptureMax = 2048;             // the ring, 1 kHz
  static constexpr uint16_t kWindowMaxMs = kCaptureMax - 1; // pre + post bound
  static constexpr uint32_t kSampleUs = 1000;               // dt_us in FETCH
  static constexpr uint32_t kTrialDelayUs = 1000000u; // TRIAL after an outcome
  static constexpr uint16_t kDefaultPreMs = 500;      // CFG pre_ms
  static constexpr uint16_t kDefaultPostMs = 500;     // CFG post_ms

  enum class Outcome : uint8_t { kNone, kPull, kPush, kNoMove, kAbort };

  struct Params {
    double pull = 0.0;       // deg, pull-positive excursion
    double push = 0.0;       // deg, positive in the push direction
    uint32_t timeout_us = 0; // NOMOVE if no first motion by then
    uint32_t hold_us = 0;    // PULL needs this long continuously past `pull`
    bool has_maxdur = false;
    uint32_t maxdur_us = 0; // ABORT if motion but no outcome by then
  };

  struct Summary {
    Outcome outcome = Outcome::kNone;
    uint64_t t_rel = 0;
    bool has_move = false;
    uint64_t t_move = 0;
    bool has_out = false;
    uint64_t t_out = 0;
    double a0 = 0.0;
    double a_peak = 0.0;     // signed excursion of largest magnitude
    double a_end = 0.0;      // excursion at the outcome, or at the exit
    uint32_t dur_thr_ms = 0; // whole ms at or past `pull` before the outcome
  };

  static const char *name(Outcome o) {
    switch (o) {
    case Outcome::kPull:
      return "PULL";
    case Outcome::kPush:
      return "PUSH";
    case Outcome::kNoMove:
      return "NOMOVE";
    case Outcome::kAbort:
      return "ABORT";
    default:
      return "NONE";
    }
  }

  // ------------------------------------------------------------ lifecycle

  // RELEASED entered: thresholds will be relative to `a0`, and the capture
  // ring starts, so that a WATCH that follows has history behind it. The
  // previous window is gone from here: the host fetches before the next trial.
  void release(uint64_t now_us, double a0, double pull_sign) {
    released_ = true;
    a0_ = a0;
    pull_sign_ = pull_sign;
    t_rel_ = now_us;
    watched_ = false;
    armed_ = false;
    summarized_ = false;
    trial_due_ = false;
    outcome_ = Outcome::kNone;
    cap_watched_ = false;
    win_valid_ = false;
    startRing(now_us);
  }

  // RELEASED left. The caller takes the summary first (if `trialPending()`).
  // The window freezes here with what exists.
  void leave() {
    if (ring_running_) {
      freezeWindow();
    }
    released_ = false;
    armed_ = false;
    trial_due_ = false;
  }

  bool released() const { return released_; }
  bool armed() const { return armed_; }

  // WATCH: arm the one-shot classifier. A WATCH while one is armed replaces
  // it and the ring keeps running; a WATCH after a frozen window restarts the
  // ring, so its history starts here.
  void arm(uint64_t now_us, const Params &p, double deadband_deg) {
    params_ = p;
    deadband_ = deadband_deg;
    watched_ = true;
    armed_ = true;
    summarized_ = false;
    trial_due_ = false;
    outcome_ = Outcome::kNone;
    t_watch_ = now_us;
    first_motion_ = false;
    pull_valid_ = false;
    above_prev_ = false;
    last_eval_us_ = now_us;
    dur_thr_us_ = 0;
    peak_ = 0.0;
    a_end_ = 0.0;
    cap_watched_ = true;
    trig_set_ = false;
    if (!ring_running_) {
      win_valid_ = false;
      startRing(now_us);
    }
  }

  // CFG pre_ms / post_ms: the window around the outcome. The caller bounds
  // them (each <= kWindowMaxMs, sum <= kWindowMaxMs).
  void setWindow(uint16_t pre_ms, uint16_t post_ms) {
    pre_ms_ = pre_ms;
    post_ms_ = post_ms;
  }
  uint16_t preMs() const { return pre_ms_; }
  uint16_t postMs() const { return post_ms_; }

  // ------------------------------------------------------------- per step

  // Record the 1 ms-grid sample into the ring while it runs. Idempotent at
  // one time. A caller that arrives late (a coalesced tick) fills the missed
  // grid points with the current angle so sample k stays at t0 + k*dt; a gap
  // longer than the whole ring resynchronises the grid instead.
  void sample(uint64_t now_us, double angle) {
    if (!ring_running_) {
      return;
    }
    if (now_us >=
        next_sample_us_ + static_cast<uint64_t>(kCaptureMax) * kSampleUs) {
      next_sample_us_ = now_us;
    }
    while (ring_running_ && now_us >= next_sample_us_) {
      cap_[written_ % kCaptureMax] = static_cast<float>(angle);
      ++written_;
      next_sample_us_ += kSampleUs;
      if (trig_set_ && written_ >= trig_index_ + 1u + post_ms_) {
        freezeWindow();
      }
    }
  }

  // Classify. Returns the outcome to announce, once; kNone otherwise.
  Outcome evaluate(uint64_t now_us, double angle) {
    if (!armed_) {
      return Outcome::kNone;
    }
    const double rel = excursion(angle);
    if (!first_motion_ && fabs(rel) > deadband_) {
      first_motion_ = true;
      t_move_ = now_us;
    }
    if (fabs(rel) > fabs(peak_)) {
      peak_ = rel;
    }
    if (above_prev_) {
      dur_thr_us_ += static_cast<uint32_t>(now_us - last_eval_us_);
    }
    last_eval_us_ = now_us;
    const bool above = rel >= params_.pull;
    above_prev_ = above;

    Outcome result = Outcome::kNone;
    if (rel <= -params_.push) {
      result = Outcome::kPush;
    } else if (above) {
      if (!pull_valid_) {
        pull_valid_ = true;
        pull_since_us_ = now_us;
      }
      if (now_us - pull_since_us_ >= params_.hold_us) {
        result = Outcome::kPull;
      }
    } else {
      pull_valid_ = false;
    }
    if (result == Outcome::kNone && !first_motion_ &&
        now_us >= t_watch_ + params_.timeout_us) {
      result = Outcome::kNoMove;
    }
    if (result == Outcome::kNone && first_motion_ && params_.has_maxdur &&
        now_us >= t_watch_ + params_.maxdur_us) {
      result = Outcome::kAbort;
    }
    if (result != Outcome::kNone) {
      outcome_ = result;
      t_out_ = now_us;
      a_end_ = rel;
      armed_ = false;
      trial_due_ = true;
      trial_due_at_us_ = now_us + kTrialDelayUs;
      // The outcome sample is the last one written (sample() runs first in
      // the same service step); the window freezes post_ms samples later.
      if (ring_running_) {
        trig_set_ = true;
        trig_index_ = (written_ > 0) ? written_ - 1u : 0u;
        if (written_ >= trig_index_ + 1u + post_ms_) {
          freezeWindow();
        }
      }
    }
    return result;
  }

  // The earliest time something scheduled here happens, or 0 for none.
  uint64_t nextStopUs(uint64_t now_us) const {
    uint64_t best = 0;
    auto consider = [&](uint64_t t) {
      if (t < now_us) {
        t = now_us;
      }
      if (best == 0 || t < best) {
        best = t;
      }
    };
    if (ring_running_) {
      consider(next_sample_us_);
    }
    if (armed_) {
      if (!first_motion_) {
        consider(t_watch_ + params_.timeout_us);
      }
      if (pull_valid_) {
        consider(pull_since_us_ + params_.hold_us);
      }
      if (first_motion_ && params_.has_maxdur) {
        consider(t_watch_ + params_.maxdur_us);
      }
    }
    if (trial_due_ && !summarized_) {
      consider(trial_due_at_us_);
    }
    return best;
  }

  // ---------------------------------------------------------------- TRIAL

  // A summary is owed: a WATCH happened in this RELEASED period and has not
  // been reported yet.
  bool trialPending() const { return released_ && watched_ && !summarized_; }

  // The one-second rule after an outcome.
  bool trialDue(uint64_t now_us) const {
    return trialPending() && trial_due_ && now_us >= trial_due_at_us_;
  }

  // Take the summary (once). `angle` is the current angle, for `a_end` when
  // there was no outcome.
  Summary takeSummary(double angle) {
    Summary s;
    s.outcome = outcome_;
    s.t_rel = t_rel_;
    s.has_move = first_motion_;
    s.t_move = t_move_;
    s.has_out = outcome_ != Outcome::kNone;
    s.t_out = t_out_;
    s.a0 = a0_;
    s.a_peak = peak_;
    s.a_end = s.has_out ? a_end_ : excursion(angle);
    s.dur_thr_ms = dur_thr_us_ / 1000u;
    summarized_ = true;
    watched_ = false;
    armed_ = false;
    trial_due_ = false;
    return s;
  }

  // -------------------------------------------------------------- capture

  // FETCH is refused while the capture is live: a WATCH armed, or the post
  // window still filling after its outcome.
  bool captureBusy() const { return ring_running_ && (armed_ || trig_set_); }
  // A frozen window exists (from a RELEASED period in which a WATCH armed).
  bool hasWindow() const { return win_valid_; }
  uint32_t windowCount() const { return win_valid_ ? win_count_ : 0u; }
  uint32_t windowPre() const { return win_valid_ ? win_pre_ : 0u; }
  // Board time of window sample k (k may be past the end: FETCH echoes it).
  uint64_t windowSampleTimeUs(uint32_t k) const {
    return ring_t0_us_ + static_cast<uint64_t>(win_start_ + k) * kSampleUs;
  }
  double windowSample(uint32_t k) const {
    return cap_[(win_start_ + k) % kCaptureMax];
  }

private:
  double excursion(double angle) const { return (angle - a0_) * pull_sign_; }

  void startRing(uint64_t now_us) {
    ring_t0_us_ = now_us;
    next_sample_us_ = now_us;
    written_ = 0;
    trig_set_ = false;
    ring_running_ = true;
  }

  // Freeze the window: the last pre_ms samples before the outcome sample, the
  // outcome sample and what follows it (post_ms samples when it ran to the
  // end, fewer if RELEASED was left first); without an outcome, the last
  // pre_ms samples before the exit. `pre` is what the ring actually holds.
  void freezeWindow() {
    ring_running_ = false;
    const uint32_t oldest =
        (written_ > kCaptureMax) ? written_ - kCaptureMax : 0u;
    if (trig_set_) {
      const uint32_t have = trig_index_ - oldest;
      win_pre_ = (have < pre_ms_) ? have : pre_ms_;
      win_start_ = trig_index_ - win_pre_;
      win_count_ = written_ - win_start_;
    } else {
      const uint32_t have = written_ - oldest;
      win_pre_ = (have < pre_ms_) ? have : pre_ms_;
      win_start_ = written_ - win_pre_;
      win_count_ = win_pre_;
    }
    win_valid_ = cap_watched_;
  }

  bool released_ = false;
  bool watched_ = false;    // a WATCH happened since RELEASED
  bool armed_ = false;      // classifier active, no outcome yet
  bool summarized_ = false; // TRIAL already emitted
  bool trial_due_ = false;
  uint64_t trial_due_at_us_ = 0;
  double a0_ = 0.0;
  double pull_sign_ = 1.0;
  double deadband_ = 0.5;
  Params params_;
  uint64_t t_rel_ = 0;
  uint64_t t_watch_ = 0;
  bool first_motion_ = false;
  uint64_t t_move_ = 0;
  bool pull_valid_ = false;
  uint64_t pull_since_us_ = 0;
  bool above_prev_ = false;
  uint64_t last_eval_us_ = 0;
  uint32_t dur_thr_us_ = 0;
  double peak_ = 0.0;
  Outcome outcome_ = Outcome::kNone;
  uint64_t t_out_ = 0;
  double a_end_ = 0.0;
  // the ring
  bool ring_running_ = false;
  bool cap_watched_ = false; // a WATCH armed since this ring started
  uint64_t ring_t0_us_ = 0;  // time of absolute sample 0
  uint64_t next_sample_us_ = 0;
  uint32_t written_ = 0; // absolute samples written; slot = written % size
  bool trig_set_ = false;
  uint32_t trig_index_ = 0; // absolute index of the outcome sample
  uint16_t pre_ms_ = kDefaultPreMs;
  uint16_t post_ms_ = kDefaultPostMs;
  // the frozen window
  bool win_valid_ = false;
  uint32_t win_start_ = 0; // absolute index of window sample 0
  uint32_t win_count_ = 0;
  uint32_t win_pre_ = 0; // index of the outcome sample, or the count if none
  float cap_[kCaptureMax] = {0};
};

} // namespace mouse_joystick
