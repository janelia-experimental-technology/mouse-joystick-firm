#!/usr/bin/env python3
"""Reduce a torque_cal_run.py log: gauge force times radius against the board's torque.

For each hold (between the pyboard program's `hold L start` and `hold L end`
notes) the gauge readings (gF, host-stamped) are paired with the board's
`ev;S` torque at the nearest stream sample (within --pair-ms). The applied
torque is F x r: gF x 9.80665e-3 N/gF x r_cm, N.cm, sign flipped with --invert
so that a pull reads positive like the board's pull-positive convention on the
right station (use --invert for a station or gauge orientation that reads the
other way). Samples are split into "holding" (the angle within --slip-deg of
where the hold began) and "slipping" (moved further). Over the holding samples
with |F x r| above --min-ncm a straight line torque = a x applied + b is fitted;
a is the ratio of the board's nominal scale to the truth, b the untared offset.
Over the slipping samples the medians of both are the brake's slip torque as
the sensor and as the gauge see it (the gauge also carries the motor-side
friction and detent).

Usage: torque_cal_analyze.py LOG --radius-mm 75 [--invert] [--json OUT]
"""
import argparse, json, math, statistics, sys

GF_TO_N = 9.80665e-3


def parse(path):
    stream, gauge, holds = [], [], []
    cur = None
    for line in open(path):
        parts = line.rstrip("\n").split(" ", 2)
        if len(parts) < 3:
            continue
        tag, ns, rest = parts[0], int(parts[1]), parts[2]
        if tag == "G":
            toks = rest.split()
            try:
                gauge.append((ns, float(toks[0]), toks[1] if len(toks) > 1 else ""))
            except (ValueError, IndexError):
                pass
        elif tag == "B":
            f = rest.split(" ", 2)
            if len(f) < 3:
                continue
            kind, text = f[1], f[2]
            if kind == "<" and text.startswith("ev;S;"):
                d = {}
                for kvp in text.split(";")[3:]:
                    k, _, v = kvp.partition("=")
                    try:
                        d[k] = float(v)
                    except ValueError:
                        pass
                if "angle" in d and "torque" in d:
                    stream.append((ns, d["angle"], d["torque"]))
            elif kind == "H" and text.startswith("hold "):
                toks = text.split()
                level = float(toks[1])
                if toks[2] == "start":
                    rest_tq = None
                    for t in toks[3:]:
                        if t.startswith("rest_tq="):
                            try:
                                rest_tq = float(t.split("=", 1)[1])
                            except ValueError:
                                pass
                    cur = dict(level=level, start_ns=ns, end_ns=None, rest_tq=rest_tq)
                    holds.append(cur)
                elif toks[2] == "end" and cur is not None:
                    cur["end_ns"] = ns
                    cur["end_how"] = toks[3] if len(toks) > 3 else ""
                    cur = None
    return stream, gauge, holds


def nearest(stream, ns, pair_ns):
    lo, hi = 0, len(stream) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if stream[mid][0] < ns:
            lo = mid + 1
        else:
            hi = mid
    best = None
    for i in (lo - 1, lo):
        if 0 <= i < len(stream):
            d = abs(stream[i][0] - ns)
            if d <= pair_ns and (best is None or d < abs(best[0] - ns)):
                best = stream[i]
    return best


def fit(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return None
    a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    b = my - a * mx
    res = [y - (a * x + b) for x, y in zip(xs, ys)]
    rms = math.sqrt(sum(r * r for r in res) / n)
    return a, b, rms


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("log")
    ap.add_argument("--radius-mm", type=float, required=True)
    ap.add_argument("--invert", action="store_true", help="flip the sign of the gauge torque")
    ap.add_argument("--pair-ms", type=float, default=120.0)
    ap.add_argument("--slip-deg", type=float, default=0.5)
    ap.add_argument("--min-ncm", type=float, default=0.15, help="ignore gauge torques smaller than this in the fit")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    stream, gauge, holds = parse(args.log)
    r_cm = args.radius_mm / 10.0
    sign = -1.0 if args.invert else 1.0
    print("stream samples %d, gauge readings %d, holds %d, radius %.1f mm" % (len(stream), len(gauge), len(holds), args.radius_mm))
    results = []
    for h in holds:
        end = h["end_ns"] or (stream[-1][0] if stream else h["start_ns"])
        seg = [s for s in stream if h["start_ns"] <= s[0] <= end]
        if not seg:
            print("level %.2f: no stream samples" % h["level"])
            continue
        a0 = statistics.median([s[1] for s in seg[:25]])
        pairs = []
        for g in gauge:
            if not (h["start_ns"] <= g[0] <= end):
                continue
            s = nearest(seg, g[0], int(args.pair_ms * 1e6))
            if s is None:
                continue
            applied = sign * g[1] * GF_TO_N * r_cm
            pairs.append((applied, s[2], s[1], g[1]))
        holding = [p for p in pairs if abs(p[2] - a0) < args.slip_deg]
        slipping = [p for p in pairs if abs(p[2] - a0) >= args.slip_deg]
        loaded = [p for p in holding if abs(p[0]) >= args.min_ncm]
        res = dict(level=h["level"], rest_tq=h["rest_tq"], end_how=h.get("end_how"), angle0=a0,
                   n_pairs=len(pairs), n_holding=len(holding), n_loaded=len(loaded), n_slipping=len(slipping),
                   gauge_max_gf=max((abs(p[3]) for p in pairs), default=0.0),
                   torque_min=min(s[2] for s in seg), torque_max=max(s[2] for s in seg))
        if len(loaded) >= 5:
            f = fit([p[0] for p in loaded], [p[1] for p in loaded])
            if f:
                res.update(fit_a=f[0], fit_b=f[1], fit_rms=f[2])
        if len(slipping) >= 5:
            res.update(slip_sensor_median=statistics.median(p[1] for p in slipping),
                       slip_gauge_median=statistics.median(p[0] for p in slipping),
                       slip_angle_span=(min(p[2] for p in slipping), max(p[2] for p in slipping)))
        results.append(res)
        print("\nlevel %.2f N.cm (hold ended: %s): %d gauge/stream pairs, %d holding (%d loaded), %d slipping; gauge peak %.1f gF; stream torque %.3f..%.3f"
              % (res["level"], res["end_how"], res["n_pairs"], res["n_holding"], res["n_loaded"], res["n_slipping"],
                 res["gauge_max_gf"], res["torque_min"], res["torque_max"]))
        if "fit_a" in res:
            print("  holding fit: board torque = %.4f x (F x r) %+.4f N.cm, rms %.4f N.cm  -> true scale factor %.4f"
                  % (res["fit_a"], res["fit_b"], res["fit_rms"], 1.0 / res["fit_a"] if res["fit_a"] else float("nan")))
        if "slip_sensor_median" in res:
            print("  slipping: sensor median %.3f N.cm, gauge median %.3f N.cm, angle %.2f..%.2f deg"
                  % (res["slip_sensor_median"], res["slip_gauge_median"], res["slip_angle_span"][0], res["slip_angle_span"][1]))
    if args.json:
        with open(args.json, "w") as f:
            json.dump(dict(radius_mm=args.radius_mm, invert=args.invert, results=results), f, indent=1)
        print("\nwrote", args.json)


if __name__ == "__main__":
    main()
