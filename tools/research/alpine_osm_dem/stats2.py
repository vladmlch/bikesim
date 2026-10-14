# /// script
# requires-python = ">=3.9"
# dependencies = ["numpy", "scipy"]
# ///
"""Grade statistics over trail profiles — the canonical stats emitter.

Reads <profiles-dir>/<region>_profiles.json (or .json.gz), writes
stats2_out.json. This is a verbatim port of the audit's stats2.py
(.scratch/track-research/a-slope-profile/src/stats2.py) with an argparse
wrapper; the algorithm is unchanged and reproduces the reference output
bitwise on the pinned inputs (see README "Verification").

Definitions (fixed):
  STEP = 2 m profile spacing.
  win_slope_ls(s, z, w): least-squares dz/ds over a w-metre window centred
    on each sample (k = w/STEP samples each side; NaN at the edges).
  runs: consecutive samples of |g5| in bands <8 %, 8-15 %, 15-25 %, >25 %;
    run length = samples x STEP.
  reversals: local max-min-max on gaussian-filtered z (sigma = 2 m);
    a "dip" counts if drop >= 0.30 m and length in 1.5-30 m.
  switchbacks: heading change > 120 deg over a 15 m window of the 2 m
    resampled polyline.
  treeline mask: z > 2100 m (zermatt) / z > 2200 m (engadin); the
    flat_control class uses z < treeline - 300 m AND |g20| < 5 %.
  segments: contiguous above/below-mask runs; >= 16 samples (32 m) to count.
"""
import argparse
import gzip
import json
import os

import numpy as np
from scipy.ndimage import gaussian_filter1d

STEP = 2.0


def win_slope_ls(s, z, w):
    n = len(s)
    k = max(1, int(round(w / STEP)))
    g = np.full(n, np.nan)
    for i in range(k, n - k):
        xs = s[i - k:i + k + 1]
        ys = z[i - k:i + k + 1]
        g[i] = np.polyfit(xs, ys, 1)[0]
    return g


def reversals(s, z, sigma_m=2.0, depth_min=0.30, min_len=1.5, max_len=30.):
    ks = sigma_m / STEP
    zs = gaussian_filter1d(z, ks)
    ext = []
    for i in range(1, len(zs) - 1):
        if (zs[i] >= zs[i - 1] and zs[i] > zs[i + 1]) or (zs[i] <= zs[i - 1] and zs[i] < zs[i + 1]):
            ext.append(i)
    dips = []
    i = 0
    while i < len(ext) - 2:
        a, b, c = ext[i], ext[i + 1], ext[i + 2]
        if zs[a] > zs[b] and zs[c] > zs[b]:
            drop = zs[a] - zs[b]
            length = s[b] - s[a]
            if drop >= depth_min and min_len <= length <= max_len:
                dips.append((length, drop))
                i += 2
                continue
        i += 1
    return dips


def switchbacks(x, y, win_m=15., ang_min=120.):
    hdg = np.arctan2(np.diff(y), np.diff(x))
    n = len(hdg)
    k = max(1, int(round(win_m / STEP)))
    turns = []
    i = 0
    while i < n - k:
        tot = abs(np.arctan2(np.sin(hdg[i + k] - hdg[i]), np.cos(hdg[i + k] - hdg[i])))
        if np.degrees(tot) > ang_min:
            turns.append(i + k // 2)
            i += k * 2
        else:
            i += 1
    return turns


CLASSES = {
    "T2_mountain": lambda t: t.get("sac_scale") == "mountain_hiking",
    "T3_demanding": lambda t: t.get("sac_scale") == "demanding_mountain_hiking",
    "T45_alpine": lambda t: t.get("sac_scale") in ("alpine_hiking", "demanding_alpine_hiking", "difficult_alpine_hiking"),
    "mtb_tagged": lambda t: "mtb:scale" in t or "mtb:scale:uphill" in t,
    "flat_control": lambda t: True,   # filtered later by terrain slope
}
TREELINE = {"zermatt": 2100., "engadin": 2200.}


def load_profiles(path):
    op = gzip.open if path.endswith(".gz") else open
    with op(path, "rt") as f:
        return json.load(f)


def run(regions, profiles_dir, out_path):
    out = {}
    for region in regions:
        prof_path = os.path.join(profiles_dir, f"{region}_profiles.json")
        if not os.path.exists(prof_path):
            prof_path += ".gz"
        profs = load_profiles(prof_path)
        tl = TREELINE[region]
        for cname, pred in CLASSES.items():
            g5 = []; g10 = []; g20 = []; g2 = []
            revs = []; total = 0; sw = 0
            runs = {"flat": [], "mod": [], "steep": [], "wall": []}
            for p in profs:
                if not pred(p["tags"]):
                    continue
                s = np.array(p["s"]); z = np.array(p["z"]); x = np.array(p["x"]); y = np.array(p["y"])
                if cname == "flat_control":
                    m = z < tl - 300.
                else:
                    m = z > tl
                if m.sum() < 12:
                    continue
                idx = np.where(m)[0]
                for seg in np.split(idx, np.where(np.diff(idx) > 1)[0] + 1):
                    if len(seg) < 16:
                        continue
                    ss = s[seg] - s[seg][0]; zz = z[seg]; xx = x[seg]; yy = y[seg]
                    g20v = win_slope_ls(ss, zz, 20.)
                    if cname == "flat_control":
                        keep = np.isfinite(g20v) & (np.abs(g20v) < 0.05)
                        if keep.sum() < 8:
                            continue
                    total += ss[-1]
                    for w, acc in ((2., g2), (5., g5), (10., g10), (20., g20)):
                        acc.extend(np.abs(win_slope_ls(ss, zz, w)).tolist())
                    revs += reversals(ss, zz)
                    sw += len(switchbacks(xx, yy))
                    gv = win_slope_ls(ss, zz, 5.)
                    band = np.digitize(np.abs(gv), [0.08, 0.15, 0.25])
                    band[~np.isfinite(gv)] = -1
                    i = 0
                    while i < len(band):
                        j = i
                        while j < len(band) and band[j] == band[i]:
                            j += 1
                        if band[i] >= 0:
                            runs[["flat", "mod", "steep", "wall"][band[i]]].append((j - i) * STEP)
                        i = j
            if total == 0:
                continue

            def q(a, qs=(5, 25, 50, 75, 90, 95, 99)):
                a = np.array(a)
                a = a[np.isfinite(a)]
                return [float(np.percentile(a, x)) for x in qs] if len(a) else None
            rl = [d[0] for d in revs]
            rd = [d[1] for d in revs]
            out[f"{region}_{cname}"] = {
                "km": total / 1000,
                "g2": q(g2), "g5": q(g5), "g10": q(g10), "g20": q(g20),
                "frac20_5": float(np.nanmean(np.array(g5) > 0.20)),
                "frac25_5": float(np.nanmean(np.array(g5) > 0.25)),
                "frac30_5": float(np.nanmean(np.array(g5) > 0.30)),
                "frac20_10": float(np.nanmean(np.array(g10) > 0.20)),
                "frac25_10": float(np.nanmean(np.array(g10) > 0.25)),
                "frac30_10": float(np.nanmean(np.array(g10) > 0.30)),
                "frac20_20": float(np.nanmean(np.array(g20) > 0.20)),
                "frac25_20": float(np.nanmean(np.array(g20) > 0.25)),
                "frac30_20": float(np.nanmean(np.array(g20) > 0.30)),
                "rev_per_km": len(revs) / (total / 1000),
                "rev_len": q(rl, (25, 50, 75, 90)),
                "rev_dep": q(rd, (25, 50, 75, 90)),
                "sw_per_km": sw / (total / 1000),
                "run_flat": q(runs["flat"], (25, 50, 75, 90)),
                "run_mod": q(runs["mod"], (25, 50, 75, 90)),
                "run_steep": q(runs["steep"], (25, 50, 75, 90)),
                "run_wall": q(runs["wall"], (25, 50, 75, 90)),
            }
    json.dump(out, open(out_path, "w"), indent=1)
    for k, v in out.items():
        print(k, "km=%.1f" % v["km"])
        print("  g5 :", [round(x, 3) for x in v["g5"]])
        print("  g10:", [round(x, 3) for x in v["g10"]])
        print("  g20:", [round(x, 3) for x in v["g20"]])
        print("  frac>20/25/30 (10m): %.3f/%.3f/%.3f  (20m): %.3f/%.3f/%.3f" % (
            v["frac20_10"], v["frac25_10"], v["frac30_10"],
            v["frac20_20"], v["frac25_20"], v["frac30_20"]))
        print("  rev/km=%.2f len_q=%s dep_q=%s sw/km=%.2f" % (
            v["rev_per_km"],
            [round(x, 1) for x in v["rev_len"]] if v["rev_len"] else None,
            [round(x, 2) for x in v["rev_dep"]] if v["rev_dep"] else None,
            v["sw_per_km"]))
        print("  runs flat=%s mod=%s steep=%s wall=%s" % (
            [round(x, 1) for x in v["run_flat"]] if v["run_flat"] else None,
            [round(x, 1) for x in v["run_mod"]] if v["run_mod"] else None,
            [round(x, 1) for x in v["run_steep"]] if v["run_steep"] else None,
            [round(x, 1) for x in v["run_wall"]] if v["run_wall"] else None))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--regions", nargs="+", default=["zermatt", "engadin"])
    ap.add_argument("--profiles-dir", default="osm")
    ap.add_argument("--out", default="stats2_out.json")
    args = ap.parse_args()
    run(args.regions, args.profiles_dir, args.out)


if __name__ == "__main__":
    main()
