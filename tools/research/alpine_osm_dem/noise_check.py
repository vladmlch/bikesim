# /// script
# requires-python = ">=3.9"
# dependencies = ["numpy", "scipy"]
# ///
"""Negative control: |g| quantiles of valley-floor trail samples (z < 1700 m
in Zermatt). If the pipeline invented steep grades, they would appear here
too; the measured values stay far below the above-treeline classes.

Verbatim port of the audit's noise_check.py.
"""
import argparse
import gzip
import json
import os

import numpy as np

from stats2 import win_slope_ls


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--profiles-dir", default="osm")
    args = ap.parse_args()

    path = os.path.join(args.profiles_dir, "zermatt_profiles.json")
    if not os.path.exists(path):
        path += ".gz"
    op = gzip.open if path.endswith(".gz") else open
    with op(path, "rt") as f:
        profs = json.load(f)
    g5 = []; g20 = []; zlow = []
    for p in profs:
        s = np.array(p["s"]); z = np.array(p["z"])
        m = (z < 1700.)  # valley floor, flat terrain around Zermatt/Täsch
        if m.sum() < 12:
            continue
        idx = np.where(m)[0]
        for seg in np.split(idx, np.where(np.diff(idx) > 1)[0] + 1):
            if len(seg) < 12:
                continue
            ss = s[seg]; zz = z[seg]
            g5.extend(np.abs(win_slope_ls(ss, zz, 5.)).tolist())
            g20.extend(np.abs(win_slope_ls(ss, zz, 20.)).tolist())
            zlow.extend(zz.tolist())
    g5 = np.array(g5); g20 = np.array(g20)
    g5 = g5[np.isfinite(g5)]; g20 = g20[np.isfinite(g20)]
    print("valley-floor (<1700m) samples:", len(g5))
    print("|g|5m quantiles 5/25/50/75/90/95/99:", [round(float(np.percentile(g5, q)), 3) for q in (5, 25, 50, 75, 90, 95, 99)])
    print("|g|20m quantiles:", [round(float(np.percentile(g20, q)), 3) for q in (5, 25, 50, 75, 90, 95, 99)])


if __name__ == "__main__":
    main()
