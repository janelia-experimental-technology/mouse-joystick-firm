#!/usr/bin/env python3
"""The gauge's reading in the slack windows of a run: before the first hold (after the 'start' cue,
hands off), and between each 'hold L end' and the next 'hold start' (hands off, the lever moving
back). With the string slack these are the gauge's zero at that moment, and their trend is the
zero's drift. Usage: gauge_slack_windows.py LOG [LOG ...]"""
import os, statistics, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "tools"))
import torque_cal_reduce as R
def med(xs): return statistics.median(xs) if xs else float("nan")
for path in sys.argv[1:]:
    print("===", os.path.basename(path))
    stream, gauge, holds = R.parse(path)
    t0 = gauge[0][0] if gauge else 0
    starts = [h[1] for h in holds]
    ends = [h[2] for h in holds]
    # before the first hold: from the first stream sample (rail up, homing) to the first hold start
    first_stream = stream[0][0] if stream else starts[0]
    w = [g[1] for g in gauge if first_stream <= g[0] < starts[0]]
    print("  pre-hold window (%.0f s): n=%d median %+.1f gF p25 %+.1f p75 %+.1f" % ((starts[0]-first_stream)/1e9, len(w), med(w), statistics.quantiles(w, n=4)[0] if len(w) > 3 else 0, statistics.quantiles(w, n=4)[2] if len(w) > 3 else 0))
    for i, (L, a, b) in enumerate(holds):
        if b is None: continue
        nxt = starts[i + 1] if i + 1 < len(starts) else b + 3e9
        w = [g[1] for g in gauge if b <= g[0] < nxt]
        print("  after hold %.1f (t=%.0f s): slack window %.1f s n=%d median %+.1f gF p25 %+.1f p75 %+.1f" % (
            L, (b - t0)/1e9, (nxt - b)/1e9, len(w), med(w), statistics.quantiles(w, n=4)[0] if len(w) > 3 else 0, statistics.quantiles(w, n=4)[2] if len(w) > 3 else 0))
