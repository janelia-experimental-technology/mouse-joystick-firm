#!/usr/bin/env python3
"""torque_cal_reduce.py with the gauge's drifting zero removed.

The Mark-10's zero drifted about 2 gF/min while hand-held at Rig 1 (2026-10-16). The zero is known
at the run's start (the gauge was zeroed just before the 'ready') and in every slack window between
'hold L end' and the next 'hold start' (hands off, the lever moving back, the string slack). This
script takes the zero as 0 at the first stream sample and the slack-window median after each hold,
interpolates linearly in between, subtracts it from every gauge reading, and then runs the same
reduction as torque_cal_reduce.py (slides by lever velocity, pull fit across levels). A re-zero by
the person during a hold is not modelled; such a hold is flagged when its two bounding slack medians
differ by more than --jump gF.

Usage: reduce_zero_corrected.py --radius-mm 72.5 LOG [LOG ...] [--json OUT] [--zero-at-start 0]"""
import argparse, json, os, statistics, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "tools"))
import torque_cal_reduce as R

def med(xs): return statistics.median(xs) if xs else float("nan")

def zero_knots(stream, gauge, holds, z0):
    knots = [(stream[0][0], z0)]
    starts = [h[1] for h in holds]
    for i, (L, a, b) in enumerate(holds):
        if b is None: continue
        nxt = starts[i + 1] if i + 1 < len(starts) else b + 3e9
        w = [g[1] for g in gauge if b <= g[0] < nxt]
        if len(w) >= 4:
            knots.append(((b + min(nxt, b + 3e9)) / 2, med(w)))
    return knots

def zero_at(knots, t):
    if t <= knots[0][0]: return knots[0][1]
    for (t0, z0), (t1, z1) in zip(knots, knots[1:]):
        if t0 <= t <= t1:
            return z0 + (z1 - z0) * (t - t0) / (t1 - t0)
    return knots[-1][1]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+"); ap.add_argument("--radius-mm", type=float, required=True)
    ap.add_argument("--json", default=None); ap.add_argument("--zero-at-start", type=float, default=0.0)
    ap.add_argument("--jump", type=float, default=4.0)
    args = ap.parse_args()
    r_cm = args.radius_mm / 10.0
    summaries = {}
    for path in args.logs:
        print("=== %s (gauge zero interpolated through the slack windows) ===" % os.path.basename(path))
        stream, gauge, holds = R.parse(path)
        knots = zero_knots(stream, gauge, holds, args.zero_at_start)
        t0 = stream[0][0]
        print("  zero knots (s, gF): " + ", ".join("(%.0f, %+.1f)" % ((t - t0) / 1e9, z) for t, z in knots))
        gauge_c = [(t, g - zero_at(knots, t)) for t, g in gauge]
        # same reduction as torque_cal_reduce.reduce(), on the corrected gauge
        vel = {}
        for i in range(1, len(stream) - 1):
            dt = (stream[i + 1][0] - stream[i - 1][0]) / 1e9
            vel[stream[i][0]] = (stream[i + 1][1] - stream[i - 1][1]) / dt if dt > 0 else 0.0
        pts = []; summ = {"radius_mm": args.radius_mm, "zero_knots_gf": [((t - t0) / 1e9, z) for t, z in knots], "levels": []}
        for L, a, b in holds:
            if b is None: b = stream[-1][0]
            seg = [s for s in stream if a <= s[0] <= b]
            if len(seg) < 20: continue
            ts = [s[0] for s in seg]
            base = med([s[2] for s in seg if s[0] >= b - 3e9])
            za, zb = zero_at(knots, a), zero_at(knots, b)
            pairs = []
            for g in gauge_c:
                if not (a <= g[0] <= b): continue
                s = R.nearest(seg, ts, g[0])
                if s is None: continue
                pairs.append(dict(tg=-g[1] * R.GF * r_cm, exc=s[2] - base, ang=s[1], v=vel.get(s[0], 0.0)))
            pull = [p for p in pairs if p["v"] < -0.5]; push = [p for p in pairs if p["v"] > 0.5]
            flag = "  RE-ZERO SUSPECTED (zero %+.1f -> %+.1f gF across the hold)" % (za, zb) if abs(zb - za) > args.jump else ""
            print("level %.1f (%.0f s) baseline %+.3f zero %+.1f..%+.1f gF | pull slide n=%3d gauge %.3f sensor %+.3f | push slide n=%3d gauge %+.3f sensor %+.3f%s"
                  % (L, (b - a) / 1e9, base, za, zb, len(pull), med([p["tg"] for p in pull]), med([p["exc"] for p in pull]),
                     len(push), med([p["tg"] for p in push]), med([p["exc"] for p in push]), flag))
            summ["levels"].append(dict(level=L, hold_s=(b - a) / 1e9, baseline=base, zero_gf=[za, zb],
                pull=dict(n=len(pull), gauge=med([p["tg"] for p in pull]), sensor=med([p["exc"] for p in pull])),
                push=dict(n=len(push), gauge=med([p["tg"] for p in push]), sensor=med([p["exc"] for p in push]))))
            if len(pull) >= 10: pts.append((L, med([p["tg"] for p in pull]), med([p["exc"] for p in pull])))
        if len(pts) >= 3:
            a_, b_, rms = R.fit([t for _, t, _ in pts], [e for _, _, e in pts]); f = -b_ / a_
            print("  pull slides across levels: sensor = %+.3f x (gauge - %.3f), rms %.3f -> |sensor/true| %.3f, friction %.3f N.cm" % (a_, f, rms, abs(a_), f))
            summ["pull_fit"] = dict(a=a_, f=f, rms=rms)
            lvl0 = [p for p in pts if p[0] == 0]
            if lvl0:
                f0 = lvl0[0][1] - lvl0[0][2] / a_
                print("  friction from the brake-off slide alone: %.3f N.cm; per-level |sensor/true| with it: %s"
                      % (f0, ["%.1f: %.2f" % (L, abs(e / (t - f0))) for L, t, e in pts if abs(t - f0) > 0.05]))
                summ["friction_level0"] = f0
        summaries[os.path.basename(path)] = summ
    if args.json:
        with open(args.json, "w") as fh: json.dump(summaries, fh, indent=1)
        print("wrote", args.json)

if __name__ == "__main__":
    main()
