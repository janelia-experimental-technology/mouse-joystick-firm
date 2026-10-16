#!/usr/bin/env python3
"""Reduce torque_cal_run2.py logs as pull/push hysteresis loops, with no rest and no angle needed.

Per hold, each gauge reading is paired with the board torque at the same instant. The pull
plateau is the median of the readings in the top --tail fraction of gauge torque, the push
plateau the median of the bottom tail (a pull reads negative on the gauge; tau_g = -gF x
9.80665e-3 x r_cm, pull positive). Differences between the two plateaus cancel every rest
offset, the hook's pre-tension and the sensor's wandering baseline:

  gauge span   G = g_pull - g_push = 2 T_brake + 2 f        (f: friction the gauge pays, the sensor never sees)
  sensor span  S = s_pull - s_push = 2 a T_brake            (a: the sensor's reading over the torque through it)

Level 0 gives f directly, since there T_brake is only the brake's drag D as the sensor sees it:
f = G0/2 - S0/(2a); with a from the full-brake level the two equations close in one iteration.
For every level: T_brake = G/2 - f, a_level = S / (2 T_brake), and the brake's delivered
torque is T_brake. The loop width in the sensor, measured at zero gauge torque, is the
hysteresis of the sensor path itself.

Usage: torque_cal_loops.py --radius-mm 72 LOG [LOG ...] [--full 3.5] [--tail 0.15]
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


def loops(path, r_cm, tail):
    """Plateaus are the medians of the samples within `tail` of each extreme of gauge torque,
    so idle time in a hold does not dilute them; the rest is the sensor's median where the
    gauge is unloaded."""
    stream, gauge, holds = parse(path)
    ts = [s[0] for s in stream]
    out = []
    for L, a, b in holds:
        if b is None:
            b = stream[-1][0] if stream else a
        pairs = []
        for g in gauge:
            if a <= g[0] <= b:
                tq = torque_at(stream, ts, g[0])
                if tq is not None:
                    pairs.append((-g[1] * GF * r_cm, tq))
        if len(pairs) < 40:
            continue
        gmax, gmin = max(p[0] for p in pairs), min(p[0] for p in pairs)
        pull = [p for p in pairs if p[0] >= gmax * (1 - tail)] if gmax > 0.3 else []
        push = [p for p in pairs if p[0] <= gmin * (1 - tail)] if gmin < -0.3 else []
        near0 = [p for p in pairs if abs(p[0]) < 0.08]
        r = dict(level=L, n=len(pairs), n_pull=len(pull), n_push=len(push),
                 s_mid=med([p[1] for p in near0]) if near0 else float("nan"),
                 s_spread0=(max(p[1] for p in near0) - min(p[1] for p in near0)) if len(near0) > 5 else float("nan"))
        r["g_pull"], r["s_pull"] = (med([p[0] for p in pull]), med([p[1] for p in pull])) if len(pull) >= 5 else (None, None)
        r["g_push"], r["s_push"] = (med([p[0] for p in push]), med([p[1] for p in push])) if len(push) >= 5 else (None, None)
        out.append(r)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--radius-mm", type=float, required=True)
    ap.add_argument("--full", type=float, default=3.5, help="the level whose pull anchors a when solving for f")
    ap.add_argument("--tail", type=float, default=0.15, help="fraction below each extreme that counts as its plateau")
    ap.add_argument("--friction", type=float, default=None, help="N.cm, use this f for both directions instead of solving")
    args = ap.parse_args()
    for path in args.logs:
        print("=== %s ===" % path)
        res = loops(path, args.radius_mm / 10.0, args.tail)
        by = {r["level"]: r for r in res}

        def exc(r, d):
            g, sv = r["g_" + d], r["s_" + d]
            if g is None:
                return None, None
            return abs(g), abs(sv - r["s_mid"])

        # Solve a (full-brake pull) and f per direction (level 0) together.
        fr = {"pull": args.friction, "push": args.friction}
        a = None
        if args.full in by and 0.0 in by and args.friction is None:
            gF_, sF_ = exc(by[args.full], "pull")
            if gF_:
                a = sF_ / gF_
                for _ in range(30):
                    for d in ("pull", "push"):
                        g0, s0 = exc(by[0.0], d)
                        fr[d] = (g0 - s0 / a) if g0 else None
                    if fr["pull"] is not None:
                        a = sF_ / (gF_ - fr["pull"])
        print("  friction at the gauge: pull %s, push %s N.cm%s" % (
            "%.3f" % fr["pull"] if fr["pull"] is not None else "n/a",
            "%.3f" % fr["push"] if fr["push"] is not None else "n/a",
            "" if a is None else "; a from the %.1f pull = %.3f" % (args.full, a)))
        for r in res:
            parts = ["level %.1f n=%4d rest %+.3f (spread %.3f)" % (r["level"], r["n"], r["s_mid"], r["s_spread0"])]
            for d in ("pull", "push"):
                g, sv = exc(r, d)
                if g is None:
                    parts.append("%s: none" % d)
                    continue
                f = fr[d]
                if f is None or g - f <= 0.05:
                    parts.append("%s(n=%d): gauge %.3f sensor %.3f" % (d, r["n_" + d], g, sv))
                else:
                    parts.append("%s(n=%d): gauge %.3f sensor %.3f -> brake %.3f, sensor/true %.3f" % (d, r["n_" + d], g, sv, g - f, sv / (g - f)))
            print("  " + " | ".join(parts))


if __name__ == "__main__":
    main()
