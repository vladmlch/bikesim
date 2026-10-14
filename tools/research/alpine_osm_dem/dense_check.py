# /// script
# requires-python = ">=3.9"
# dependencies = ["numpy", "scipy"]
# ///
"""Dense-geometry control: switchback rate on ways with median node
spacing <= 6 m — supports the audit's A7 underestimate correction
(T2: ~2.2-3.7/km on all ways -> 7.0-8.0/km on dense-node ways).

Needs <dir>/<region>_ways_lv.json[.gz] (full LV95 node lists from
prepare.py) and <dir>/<region>_profiles.json[.gz]. Both ship compressed in
data/. Verbatim port of the audit's dense_check.py.
"""
import argparse
import gzip
import json
import math
import os

import numpy as np

from stats2 import win_slope_ls, reversals, switchbacks, STEP  # noqa: F401

TREELINE = {"zermatt": 2100., "engadin": 2200.}
DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def load_json(base):
    """base may be given with or without .json suffix; .gz fallback."""
    for cand in (base, base + ".gz"):
        if os.path.exists(cand):
            op = gzip.open if cand.endswith(".gz") else open
            with op(cand, "rt") as f:
                return json.load(f)
    raise FileNotFoundError(base)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--regions", nargs="+", default=["zermatt", "engadin"])
    ap.add_argument("--osmdir", "--profiles-dir", dest="osmdir", default=DATA)
    args = ap.parse_args()

    for region in args.regions:
        profs = load_json(os.path.join(args.osmdir, f"{region}_profiles.json"))
        ways = {w["id"]: w for w in load_json(os.path.join(args.osmdir, f"{region}_ways_lv.json"))}
        tl = TREELINE[region]
        sw = 0; tot = 0; revs = []
        for p in profs:
            w = ways[p["id"]]
            if w["tags"].get("sac_scale") != "mountain_hiking":
                continue
            lv = w["lv"]
            sp = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(lv, lv[1:])]
            if not sp or np.median(sp) > 6.:
                continue  # dense GPS geometry only
            s = np.array(p["s"]); z = np.array(p["z"]); x = np.array(p["x"]); y = np.array(p["y"])
            m = z > tl
            idx = np.where(m)[0]
            for seg in np.split(idx, np.where(np.diff(idx) > 1)[0] + 1):
                if len(seg) < 16:
                    continue
                ss = s[seg]; zz = z[seg]; xx = x[seg]; yy = y[seg]
                tot += ss[-1] - ss[0]
                sw += len(switchbacks(xx, yy, win_m=10., ang_min=100.))
                revs += reversals(ss, zz)
        print(region, "T2 dense subset: km=%.1f switchbacks/km=%.2f rev/km=%.2f" % (tot / 1000, sw / (tot / 1000), len(revs) / (tot / 1000)))


if __name__ == "__main__":
    main()
