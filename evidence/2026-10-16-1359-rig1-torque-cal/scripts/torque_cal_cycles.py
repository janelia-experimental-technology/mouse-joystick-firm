#!/usr/bin/env python3
"""Reduce torque_cal_run2.py logs by pull cycle, without needing the lever's angle.

With the pinion off the encoder sits on the free motor shaft and the stream's angle says
nothing about the lever, so this reduction finds the pull cycles in the gauge trace
instead. A cycle is a stretch where |gauge torque| exceeds --on (N.cm) for at least
--min-s seconds; its plateau is the median of its top quartile of |gauge torque| (where
the brake slips and the lever slides) and the board torque at the same instants; its rest
is the median of both over the --rest-s seconds before it began. Excursions are plateau
minus rest.

Level 0 cycles give the friction between the gauge and the brake: the gauge excursion
while the free lever slides, less the sensor's excursion (the brake's drag, which does pass
through the sensor). For the other levels, ratio = sensor excursion / (gauge excursion -
friction) is the sensor's reading relative to the torque through the brake. Pulls read
negative on the gauge; tau_g = -gF x 9.80665e-3 x r_cm, pull positive.

Usage: torque_cal_cycles.py --radius-mm 72.5 LOG [LOG ...] [--friction F]
"""
import argparse, bisect, statistics
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


def med(xs):
    return statistics.median(xs) if xs else float("nan")


def torque_at(stream, ts, ns):
    j = bisect.bisect_left(ts, ns)
    c = [stream[i] for i in (j - 1, j) if 0 <= i < len(stream)]
    c = [s for s in c if abs(s[0] - ns) <= 120e6]
    return min(c, key=lambda s: abs(s[0] - ns))[2] if c else None


def cycles(pairs, on, min_ns):
    """pairs: (ns, tau_g, tq). Returns [(i0, i1)] index ranges where |tau_g| > on."""
    out, i, n = [], 0, len(pairs)
    while i < n:
        if abs(pairs[i][1]) > on:
            j = i
            while j < n and abs(pairs[j][1]) > on * 0.5:
                j += 1
            if pairs[j - 1][0] - pairs[i][0] >= min_ns:
                out.append((i, j))
            i = j
        else:
            i += 1
    return out


def reduce(path, r_cm, on, min_s, rest_s):
    stream, gauge, holds = parse(path)
    ts = [s[0] for s in stream]
    result = []
    for L, a, b in holds:
        if b is None:
            b = stream[-1][0] if stream else a
        pairs = []
        for g in gauge:
            if a <= g[0] <= b:
                tq = torque_at(stream, ts, g[0])
                if tq is not None:
                    pairs.append((g[0], -g[1] * GF * r_cm, tq))
        cyc = []
        for i0, i1 in cycles(pairs, on, int(min_s * 1e9)):
            seg = pairs[i0:i1]
            sign = 1.0 if med([p[1] for p in seg]) > 0 else -1.0
            mags = sorted(abs(p[1]) for p in seg)
            q = mags[int(len(mags) * 0.75)] if len(mags) > 4 else mags[0]
            top = [p for p in seg if abs(p[1]) >= q]
            t_start = pairs[i0][0]
            pre = [p for p in pairs if t_start - int(rest_s * 1e9) <= p[0] < t_start and abs(p[1]) < on * 0.5]
            if len(pre) < 3 or len(top) < 3:
                continue
            cyc.append(dict(sign=sign, dur_s=(pairs[i1 - 1][0] - t_start) / 1e9,
                            g_plateau=med([p[1] for p in top]) - med([p[1] for p in pre]),
                            s_plateau=med([p[2] for p in top]) - med([p[2] for p in pre]),
                            g_rest=med([p[1] for p in pre]), s_rest=med([p[2] for p in pre])))
        result.append(dict(level=L, hold_s=(b - a) / 1e9, cycles=cyc))
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--radius-mm", type=float, required=True)
    ap.add_argument("--on", type=float, default=0.25, help="N.cm of gauge torque that starts a cycle")
    ap.add_argument("--min-s", type=float, default=1.0)
    ap.add_argument("--rest-s", type=float, default=1.5)
    ap.add_argument("--friction", type=float, default=None, help="N.cm, override the level-0 friction (pull direction)")
    args = ap.parse_args()
    for path in args.logs:
        print("=== %s ===" % path)
        levels = reduce(path, args.radius_mm / 10.0, args.on, args.min_s, args.rest_s)
        fr = {}
        for lv in levels:
            if lv["level"] == 0:
                for d, name in ((1.0, "pull"), (-1.0, "push")):
                    cs = [c for c in lv["cycles"] if c["sign"] == d]
                    if cs:
                        fr[name] = med([abs(c["g_plateau"]) - abs(c["s_plateau"]) for c in cs])
        if args.friction is not None:
            fr["pull"] = args.friction
            fr.setdefault("push", args.friction)
        print("friction between gauge and brake, from the level-0 cycles: %s" % {k: round(v, 3) for k, v in fr.items()})
        for lv in levels:
            for d, name in ((1.0, "pull"), (-1.0, "push")):
                cs = [c for c in lv["cycles"] if c["sign"] == d]
                if not cs:
                    continue
                gp = [abs(c["g_plateau"]) for c in cs]
                sp = [abs(c["s_plateau"]) for c in cs]
                line = "level %.1f %-4s %2d cycles: gauge plateau %.3f (sd %.3f), sensor plateau %.3f (sd %.3f), sensor rest %+.3f" % (
                    lv["level"], name, len(cs), med(gp), statistics.pstdev(gp) if len(gp) > 1 else 0.0, med(sp),
                    statistics.pstdev(sp) if len(sp) > 1 else 0.0, med([c["s_rest"] for c in cs]))
                f = fr.get(name)
                if lv["level"] != 0 and f is not None and med(gp) - f > 0.05:
                    ratios = [s / (g - f) for g, s in zip(gp, sp) if g - f > 0.05]
                    line += "  -> through brake %.3f, sensor/true %.3f (sd %.3f)" % (med(gp) - f, med(ratios), statistics.pstdev(ratios) if len(ratios) > 1 else 0.0)
                print(line)


if __name__ == "__main__":
    main()
