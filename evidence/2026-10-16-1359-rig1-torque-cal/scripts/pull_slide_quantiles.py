#!/usr/bin/env python3
"""Per level: quantiles of the gauge torque, the sensor excursion and the lever velocity over the
pull-slide and push-slide samples of torque_cal_reduce.py, to see whether a level's median hides
a bimodal pull. Usage: pull_slide_quantiles.py --radius-mm 72.5 LOG [LOG ...]"""
import argparse, os, statistics, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "tools"))
import torque_cal_reduce as R
ap = argparse.ArgumentParser(); ap.add_argument("logs", nargs="+"); ap.add_argument("--radius-mm", type=float, required=True)
a = ap.parse_args()
def q(xs):
    if len(xs) < 4: return "n=%d" % len(xs)
    qs = statistics.quantiles(xs, n=10)
    return "p10 %+.2f p25 %+.2f p50 %+.2f p75 %+.2f p90 %+.2f" % (qs[0], qs[1], qs[4], qs[6], qs[8])
for path in a.logs:
    print("===", os.path.basename(path))
    for h in R.reduce(path, a.radius_mm / 10.0):
        for name in ("pull", "push"):
            s = h[name]
            print("level %.1f %s n=%3d | gauge %s | sensor %s | |v| %s | angle %s" % (
                h["level"], name, len(s), q([p["tg"] for p in s]), q([p["exc"] for p in s]),
                q([abs(p["v"]) for p in s]), q([p["ang"] for p in s])))
