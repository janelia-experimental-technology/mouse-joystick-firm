#!/usr/bin/env python3
"""Reduce torque_cal_run2.py logs by the lever's motion rather than by cue phase.

For each hold: baseline = median board torque over the last 3 s (hands off); samples are
paired gauge reading (tau_g = -gF x 9.80665e-3 x r_cm, pull positive) to the nearest stream
sample within 120 ms. Moving samples (|angular velocity| > 0.5 deg/s) are split by the
velocity sign: negative is the pull direction on both stations. Static loaded samples
(|velocity| < 0.15 deg/s, |tau_g| > 0.3) get a straight-line fit. Across levels the pull
slides are fitted as sensor = a x (gauge - f): a is the sensor's reading relative to the
torque through the brake, f the kinetic friction the gauge pays and the sensor never sees.

Usage: torque_cal_reduce.py --radius-mm 72 LOG [LOG ...]
"""
import argparse, bisect, math, statistics
GF = 9.80665e-3


def parse(path):
    stream, gauge, holds, cur = [], [], [], None
    for line in open(path):
        p = line.rstrip("\n").split(" ", 2)
        if len(p) < 3:
            continue
        tag, ns, rest = p[0], int(p[1]), p[2]
        if tag == "G":
            try:
                gauge.append((ns, float(rest.split()[0])))
            except (ValueError, IndexError):
                pass
        elif tag == "B":
            f = rest.split(" ", 2)
            if len(f) < 3:
                continue
            if f[1] == "<" and f[2].startswith("ev;S;"):
                d = dict(kv.split("=") for kv in f[2].split(";")[3:] if "=" in kv)
                stream.append((ns, float(d["angle"]), float(d["torque"])))
            elif f[1] == "H" and f[2].startswith("hold "):
                t = f[2].split()
                if t[2] == "start":
                    cur = [float(t[1]), ns, None]
                    holds.append(cur)
                else:
                    cur[2] = ns
    return stream, gauge, holds


def nearest(seg, ts, ns):
    j = bisect.bisect_left(ts, ns)
    c = [seg[i] for i in (j - 1, j) if 0 <= i < len(seg)]
    c = [s for s in c if abs(s[0] - ns) <= 120e6]
    return min(c, key=lambda s: abs(s[0] - ns)) if c else None


def med(xs):
    return statistics.median(xs) if xs else float("nan")


def fit(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    b = my - a * mx
    rms = math.sqrt(sum((y - a * x - b) ** 2 for x, y in zip(xs, ys)) / n)
    return a, b, rms


def reduce(path, r_cm):
    stream, gauge, holds = parse(path)
    vel = {}
    for i in range(1, len(stream) - 1):
        dt = (stream[i + 1][0] - stream[i - 1][0]) / 1e9
        vel[stream[i][0]] = (stream[i + 1][1] - stream[i - 1][1]) / dt if dt > 0 else 0.0
    out = []
    for L, a, b in holds:
        if b is None:  # a hold still running, or a run cut short
            b = stream[-1][0] if stream else a
        seg = [s for s in stream if a <= s[0] <= b]
        if len(seg) < 20:
            continue
        ts = [s[0] for s in seg]
        base = med([s[2] for s in seg if s[0] >= b - 3e9])
        pairs = []
        for g in gauge:
            if not (a <= g[0] <= b):
                continue
            s = nearest(seg, ts, g[0])
            if s is None:
                continue
            pairs.append(dict(tg=-g[1] * GF * r_cm, exc=s[2] - base, ang=s[1], v=vel.get(s[0], 0.0)))
        pull = [p for p in pairs if p["v"] < -0.5]
        push = [p for p in pairs if p["v"] > 0.5]
        static = [p for p in pairs if abs(p["v"]) < 0.15 and abs(p["tg"]) > 0.3]
        out.append(dict(level=L, base=base, pull=pull, push=push, static=static, hold_s=(b - a) / 1e9))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--radius-mm", type=float, required=True)
    args = ap.parse_args()
    for path in args.logs:
        print("=== %s ===" % path)
        levels = reduce(path, args.radius_mm / 10.0)
        pts = []
        for h in levels:
            pull, push, static = h["pull"], h["push"], h["static"]
            print("level %.1f (%.0f s) baseline %+.3f | pull slide n=%3d gauge %.3f sensor %+.3f | push slide n=%3d gauge %+.3f sensor %+.3f | static n=%d"
                  % (h["level"], h["hold_s"], h["base"], len(pull), med([p["tg"] for p in pull]), med([p["exc"] for p in pull]),
                     len(push), med([p["tg"] for p in push]), med([p["exc"] for p in push]), len(static)))
            if len(static) >= 8:
                a, b, rms = fit([p["tg"] for p in static], [p["exc"] for p in static])
                print("    static fit: sensor = %+.3f x gauge %+.3f, rms %.3f, gauge %.2f..%.2f" % (a, b, rms, min(p["tg"] for p in static), max(p["tg"] for p in static)))
            if len(pull) >= 10:
                pts.append((h["level"], med([p["tg"] for p in pull]), med([p["exc"] for p in pull])))
        if len(pts) >= 3:
            a, b, rms = fit([t for _, t, _ in pts], [e for _, _, e in pts])
            f = -b / a if a else float("nan")
            print("  pull slides across levels: sensor = %+.3f x (gauge - %.3f), rms %.3f  -> |sensor/true| %.3f, true/sensor %.3f, friction %.3f N.cm"
                  % (a, f, rms, abs(a), 1 / abs(a), f))
            lvl0 = [p for p in pts if p[0] == 0]
            if lvl0:
                f0 = lvl0[0][1] - lvl0[0][2] / a
                print("  friction from the brake-off slide alone: %.3f N.cm; per-level |sensor/true| with it: %s"
                      % (f0, ["%.1f: %.2f" % (L, abs(e / (t - f0))) for L, t, e in pts if abs(t - f0) > 0.05]))


if __name__ == "__main__":
    main()
