# /// script
# requires-python = ">=3.9"
# dependencies = ["numpy", "scipy"]
# ///
"""Length share of each grade band (|g5|) per class — audit table A4.

Run from a directory containing <profiles-dir>/<region>_profiles.json[.gz].
Verbatim port of the audit's bandfrac.py.
"""
import argparse
import gzip
import json
import os

import numpy as np

from stats2 import win_slope_ls

TREELINE = {"zermatt": 2100., "engadin": 2200.}
CLASSES = (
    ("T2", lambda t: t.get("sac_scale") == "mountain_hiking"),
    ("T3", lambda t: t.get("sac_scale") == "demanding_mountain_hiking"),
    ("T45", lambda t: t.get("sac_scale") in ("alpine_hiking", "demanding_alpine_hiking", "difficult_alpine_hiking")),
    ("mtb", lambda t: "mtb:scale" in t or "mtb:scale:uphill" in t),
)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--regions", nargs="+", default=["zermatt", "engadin"])
    ap.add_argument("--profiles-dir", default="osm")
    args = ap.parse_args()

    for region in args.regions:
        path = os.path.join(args.profiles_dir, f"{region}_profiles.json")
        if not os.path.exists(path):
            path += ".gz"
        op = gzip.open if path.endswith(".gz") else open
        with op(path, "rt") as f:
            profs = json.load(f)
        tl = TREELINE[region]
        for cname, pred in CLASSES:
            bands = np.zeros(4)
            for p in profs:
                if not pred(p["tags"]):
                    continue
                s = np.array(p["s"]); z = np.array(p["z"])
                m = z > tl
                idx = np.where(m)[0]
                if len(idx) < 16:
                    continue
                for seg in np.split(idx, np.where(np.diff(idx) > 1)[0] + 1):
                    if len(seg) < 16:
                        continue
                    ss = s[seg]; zz = z[seg]
                    g = win_slope_ls(ss, zz, 5.)
                    b = np.digitize(np.abs(g), [0.08, 0.15, 0.25])
                    ok = np.isfinite(g)
                    for i in range(4):
                        bands[i] += np.sum((b == i) & ok)
            print(region, cname, "band share <8/8-15/15-25/>25%:", [round(x / bands.sum(), 3) for x in bands])


if __name__ == "__main__":
    main()