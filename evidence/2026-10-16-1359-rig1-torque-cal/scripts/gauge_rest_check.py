#!/usr/bin/env python3
"""Per hold: the gauge's raw reading (gF) while the lever is still and the sensor unloaded, i.e. the
gauge's zero during that hold, and the count of such samples. Usage: gauge_rest_check.py LOG [LOG ...]"""
import os, statistics, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "tools"))
import torque_cal_reduce as R
for path in sys.argv[1:]:
    print("===", os.path.basename(path))
    stream, gauge, holds = R.parse(path)
    vel = {}
    for i in range(1, len(stream) - 1):
        dt = (stream[i + 1][0] - stream[i - 1][0]) / 1e9
        vel[stream[i][0]] = (stream[i + 1][1] - stream[i - 1][1]) / dt if dt > 0 else 0.0
    first_hold = holds[0][1] if holds else None
    pre = [g[1] for g in gauge if first_hold and g[0] < first_hold]
    print("before the first hold (lever untouched): n=%d median %+.1f gF, min %+.1f max %+.1f" % (len(pre), statistics.median(pre) if pre else float('nan'), min(pre) if pre else 0, max(pre) if pre else 0))
    for L, a, b in holds:
        if b is None: b = stream[-1][0]
        seg = [s for s in stream if a <= s[0] <= b]
        if len(seg) < 20: continue
        ts = [s[0] for s in seg]
        base = statistics.median([s[2] for s in seg if s[0] >= b - 3e9])
        rest = []
        for g in gauge:
            if not (a <= g[0] <= b): continue
            s = R.nearest(seg, ts, g[0])
            if s and abs(vel.get(s[0], 0.0)) < 0.15 and abs(s[2] - base) < 0.05:
                rest.append(g[1])
        print("level %.1f: gauge while lever still & sensor at baseline: n=%3d median %+.1f gF (p25 %+.1f p75 %+.1f)" % (
            L, len(rest), statistics.median(rest) if rest else float('nan'),
            statistics.quantiles(rest, n=4)[0] if len(rest) > 3 else 0, statistics.quantiles(rest, n=4)[2] if len(rest) > 3 else 0))
