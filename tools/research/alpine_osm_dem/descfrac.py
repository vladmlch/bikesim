# /// script
# requires-python = ">=3.9"
# dependencies = ["numpy", "scipy"]
# ///
"""Share of descending length along the climb direction — audit table A6.

For each segment the climb direction is the sign of its net elevation gain;
the metric is the fraction of 10 m-window samples with g10 < -2 % along that
direction. Verbatim port of the audit's descfrac.py.
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
            dn = 0; tt = 0
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
                    dirn = np.sign(zz[-1] - zz[0]) or 1.0
                    g10 = win_slope_ls(ss, zz, 10.) * dirn
                    m2 = np.isfinite(g10)
                    dn += np.sum((g10 < -0.02) & m2); tt += np.sum(m2)
            print(region, cname, "descending share along climb direction (g10<-2%%): %.3f" % (dn / tt))


if __name__ == "__main__":
    main()