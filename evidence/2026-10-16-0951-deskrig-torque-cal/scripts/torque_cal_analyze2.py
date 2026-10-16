#!/usr/bin/env python3
"""Reduce a torque_cal_run2.py log: gauge force x radius against the board's torque, by cue phase.

Gauge torque: a pull on the hook reads negative on the Mark-10, so
  tau_g = -gF x 9.80665e-3 N/gF x r_cm   (N.cm, pull positive)
Board torque: the `ev;S` stream, raw about the nominal zero. Each hold's baseline is the
median of the last --baseline-s seconds before `hold end` (hands off), and the excursion is
torque minus that baseline.

Within each hold the host's cue lines (`C`) split the time into phases. For the static
phases (ramp_up, steady, ramp_down) the lever must not have moved more than --static-deg
from the hold's start angle; a line excursion = a x tau_g + b is fitted over the samples
with |tau_g| > --min-ncm. For the slide phases (slide_pull, slide_push) only samples where
the lever is moving (|angular velocity| > --moving-dps) count; their medians give the
gauge torque, the excursion and the angle span. The level-0 slides give the kinetic
friction per direction (the gauge torque while the free lever moves, less the sensor's
drag). For every other level's slide, ratio = excursion / (tau_g - friction_dir) is the
sensor's reading relative to the torque actually passing through it.

Usage: torque_cal_analyze2.py LOG --radius-mm 72 [--json OUT]
"""
import argparse, json, math, statistics

GF_TO_N = 9.80665e-3


def parse(path):
    stream, gauge, holds, cues = [], [], [], []
    cur = None
    for line in open(path):
        parts = line.rstrip("\n").split(" ", 2)
        if len(parts) < 3:
            continue
        tag, ns, rest = parts[0], int(parts[1]), parts[2]
        if tag == "G":
            toks = rest.split()
            try:
                gauge.append((ns, float(toks[0])))
            except (ValueError, IndexError):
                pass
        elif tag == "C":
            toks = rest.split(" ", 3)
            try:
                level = float(toks[0])
            except ValueError:
                level = None
            cues.append((ns, level, toks[1], toks[3] if len(toks) > 3 else ""))
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
                    cur = dict(level=level, start_ns=ns, end_ns=None, end_how="")
                    holds.append(cur)
                elif toks[2] == "end" and cur is not None:
                    cur["end_ns"] = ns
                    cur["end_how"] = toks[3] if len(toks) > 3 else ""
                    cur = None
    return stream, gauge, holds, cues


def nearest(seg, ns, pair_ns):
    lo, hi = 0, len(seg) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if seg[mid][0] < ns:
            lo = mid + 1
        else:
            hi = mid
    best = None
    for i in (lo - 1, lo):
        if 0 <= i < len(seg):
            d = abs(seg[i][0] - ns)
            if d <= pair_ns and (best is None or d < abs(best[0] - ns)):
                best = seg[i]
    return best


def fit(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return None
    a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    b = my - a * mx
    rms = math.sqrt(sum((y - (a * x + b)) ** 2 for x, y in zip(xs, ys)) / n)
    return a, b, rms


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("log")
    ap.add_argument("--radius-mm", type=float, required=True)
    ap.add_argument("--pair-ms", type=float, default=120.0)
    ap.add_argument("--baseline-s", type=float, default=3.0)
    ap.add_argument("--static-deg", type=float, default=0.4)
    ap.add_argument("--moving-dps", type=float, default=0.5)
    ap.add_argument("--min-ncm", type=float, default=0.3)
    ap.add_argument("--friction-pull", type=float, default=None, help="N.cm, kinetic friction at the gauge for the pull slides, if this log has no level 0")
    ap.add_argument("--friction-push", type=float, default=None, help="N.cm, the same for the push slides")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    stream, gauge, holds, cues = parse(args.log)
    r_cm = args.radius_mm / 10.0
    print("stream %d, gauge %d, holds %d, cues %d, radius %.1f mm" % (len(stream), len(gauge), len(holds), len(cues), args.radius_mm))

    # angular velocity per stream sample, deg/s, from neighbours
    vel = {}
    for i in range(1, len(stream) - 1):
        dt = (stream[i + 1][0] - stream[i - 1][0]) / 1e9
        vel[stream[i][0]] = (stream[i + 1][1] - stream[i - 1][1]) / dt if dt > 0 else 0.0

    results = []
    friction = {}  # direction -> kinetic friction torque at the gauge, N.cm, signed like tau_g
    if args.friction_pull is not None:
        friction["slide_pull"] = args.friction_pull
    if args.friction_push is not None:
        friction["slide_push"] = args.friction_push
    for h in holds:
        end = h["end_ns"] or (stream[-1][0] if stream else h["start_ns"])
        seg = [s for s in stream if h["start_ns"] <= s[0] <= end]
        if len(seg) < 20:
            print("level %.2f: too few samples" % h["level"])
            continue
        a0 = statistics.median([s[1] for s in seg[:25]])
        base_from = end - int(args.baseline_s * 1e9)
        base = statistics.median([s[2] for s in seg if s[0] >= base_from]) if any(s[0] >= base_from for s in seg) else seg[-1][2]
        # phases of this hold from the cue lines
        hc = [c for c in cues if c[1] == h["level"] and h["start_ns"] - int(2e9) <= c[0] <= end]
        phases = []
        for i, c in enumerate(hc):
            p_end = hc[i + 1][0] if i + 1 < len(hc) else end
            phases.append((c[2], c[0], p_end))
        pairs = []
        for g in gauge:
            if not (h["start_ns"] <= g[0] <= end):
                continue
            s = nearest(seg, g[0], int(args.pair_ms * 1e6))
            if s is None:
                continue
            tau_g = -g[1] * GF_TO_N * r_cm
            phase = next((p[0] for p in phases if p[1] <= g[0] < p[2]), "none")
            pairs.append(dict(ns=g[0], tau_g=tau_g, exc=s[2] - base, angle=s[1], vel=vel.get(s[0], 0.0), phase=phase, gf=g[1]))
        res = dict(level=h["level"], end_how=h["end_how"], angle0=a0, baseline=base,
                   n_pairs=len(pairs), gauge_peak_pull_gf=min((p["gf"] for p in pairs), default=0.0),
                   gauge_peak_push_gf=max((p["gf"] for p in pairs), default=0.0),
                   exc_min=min(p["exc"] for p in pairs) if pairs else 0.0, exc_max=max(p["exc"] for p in pairs) if pairs else 0.0,
                   angle_min=min(s[1] for s in seg), angle_max=max(s[1] for s in seg))
        static = [p for p in pairs if p["phase"] in ("ramp_up", "steady", "ramp_down") and abs(p["angle"] - a0) < args.static_deg and abs(p["tau_g"]) > args.min_ncm]
        res["n_static"] = len(static)
        if len(static) >= 6:
            f = fit([p["tau_g"] for p in static], [p["exc"] for p in static])
            if f:
                res.update(static_a=f[0], static_b=f[1], static_rms=f[2])
        for direction in ("slide_pull", "slide_push"):
            mv = [p for p in pairs if p["phase"] == direction and abs(p["vel"]) > args.moving_dps]
            if len(mv) >= 5:
                res[direction] = dict(n=len(mv), tau_g=statistics.median(p["tau_g"] for p in mv),
                                      exc=statistics.median(p["exc"] for p in mv),
                                      vel=statistics.median(p["vel"] for p in mv),
                                      angle=(min(p["angle"] for p in mv), max(p["angle"] for p in mv)))
                if h["level"] == 0:
                    friction[direction] = res[direction]["tau_g"] - res[direction]["exc"]
        results.append(res)

    for res in results:
        print("\nlevel %.2f (hold ended %s): baseline %.3f N.cm (hands off), gauge peaks %.1f gF pull / %.1f gF push, excursion %.3f..%.3f, angle %.2f..%.2f"
              % (res["level"], res["end_how"], res["baseline"], res["gauge_peak_pull_gf"], res["gauge_peak_push_gf"], res["exc_min"], res["exc_max"], res["angle_min"], res["angle_max"]))
        if "static_a" in res:
            print("  static (%d samples): excursion = %.4f x tau_g %+.4f, rms %.4f  -> sensor/applied %.3f (friction indeterminate up to the kinetic value)"
                  % (res["n_static"], res["static_a"], res["static_b"], res["static_rms"], res["static_a"]))
        for direction in ("slide_pull", "slide_push"):
            d = res.get(direction)
            if not d:
                continue
            fr = friction.get(direction)
            line = "  %s (%d moving samples, %.1f deg/s, %.2f..%.2f deg): gauge %.3f, sensor %.3f" % (
                direction, d["n"], d["vel"], d["angle"][0], d["angle"][1], d["tau_g"], d["exc"])
            if res["level"] == 0:
                line += "  -> kinetic friction at the gauge %.3f N.cm" % (d["tau_g"] - d["exc"])
            elif fr is not None and abs(d["tau_g"] - fr) > 0.05:
                d["ratio"] = d["exc"] / (d["tau_g"] - fr)
                d["through_sensor"] = d["tau_g"] - fr
                line += "  -> through the brake %.3f, sensor/true %.3f" % (d["through_sensor"], d["ratio"])
            print(line)
    if args.json:
        with open(args.json, "w") as f:
            json.dump(dict(radius_mm=args.radius_mm, friction=friction, results=results), f, indent=1)
        print("\nwrote", args.json)


if __name__ == "__main__":
    main()
