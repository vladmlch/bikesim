# /// script
# requires-python = ">=3.9"
# dependencies = ["numpy", "scipy", "rasterio"]
# ///
"""Paired 2 m vs 0.5 m swissALTI3D check (quantifies the audit's DEM
resolution erratum: findings.md says 0.5 m, the run used 2 m assets).

For each class, takes the profile segments that fall entirely inside the
1 km LV95 cells where BOTH resolutions were downloaded, samples the 0.5 m
DEM at the SAME 2 m-spaced profile points, median-filters the same way,
and compares |g2|/|g5|/|g10| quantiles and the >25 % share.

Prepare:
  uv run download_tiles.py --region zermatt --res 05 --demdir dem

  (writes dem/zermatt_05m/*.tif; restrict --cells below to what you fetched)

Usage:
  uv run dem_resolution_check.py --demdir05 dem/zermatt_05m
"""
import argparse
import json
import os

import numpy as np
import rasterio
from scipy.ndimage import median_filter

from stats2 import win_slope_ls

# default: the two blocks fetched for the 2026-10-14 check
# (Gornergrat 2623-2625 x 1094-1095 + Matterhorn/T45 2629-2630 x 1089 and
#  2632-2633 x 1109-1110)
DEFAULT_CELLS = [(2623, 1094), (2624, 1094), (2625, 1094), (2623, 1095), (2624, 1095), (2625, 1095),
                 (2629, 1089), (2630, 1089), (2632, 1109), (2633, 1109), (2632, 1110), (2633, 1110)]


class DEM:
    def __init__(self, dirpath):
        self.cache = {}
        self.dir = dirpath
        self.files = {f[:-4]: f for f in os.listdir(self.dir)}

    def get(self, x, y):
        ex, ny = int(x // 1000), int(y // 1000)
        tid = None
        for f in self.files:
            if f.endswith(f"_{ex}-{ny}"):
                tid = f
                break
        if tid is None:
            return np.nan
        if tid not in self.cache:
            ds = rasterio.open(os.path.join(self.dir, self.files[tid]))
            self.cache[tid] = (ds.read(1), ds.transform)
            ds.close()
        arr, tr = self.cache[tid]
        col = (x - tr.c) / tr.a - 0.5
        row = (y - tr.f) / tr.e - 0.5
        if row < 0 or col < 0 or row > arr.shape[0] - 1 or col > arr.shape[1] - 1:
            return np.nan
        r0, c0 = int(row), int(col)
        fr, fc = row - r0, col - c0
        r1, c1 = min(r0 + 1, arr.shape[0] - 1), min(c0 + 1, arr.shape[1] - 1)
        return float(arr[r0, c0] * (1 - fr) * (1 - fc) + arr[r1, c0] * fr * (1 - fc)
                     + arr[r0, c1] * (1 - fr) * fc + arr[r1, c1] * fr * fc)


TREELINE = {"zermatt": 2100., "engadin": 2200.}
CLS = {"T2": lambda t: t.get("sac_scale") == "mountain_hiking",
       "T3": lambda t: t.get("sac_scale") == "demanding_mountain_hiking",
       "T45": lambda t: t.get("sac_scale") in ("alpine_hiking", "demanding_alpine_hiking", "difficult_alpine_hiking"),
       "mtb": lambda t: "mtb:scale" in t or "mtb:scale:uphill" in t}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--region", default="zermatt")
    ap.add_argument("--profiles", default=None, help="profiles.json[.gz]; default data/<region>_profiles.json.gz")
    ap.add_argument("--demdir05", required=True)
    ap.add_argument("--cells", nargs="*", type=int, default=None,
                    help="flat list e1 n1 e2 n2 ...; default = the two 2026-10-14 blocks")
    args = ap.parse_args()

    cells = DEFAULT_CELLS
    if args.cells:
        cells = [(args.cells[i], args.cells[i + 1]) for i in range(0, len(args.cells), 2)]
    cells = set(cells)

    path = args.profiles
    if path is None:
        path = os.path.join(os.path.dirname(__file__), "data", f"{args.region}_profiles.json.gz")
    import gzip
    op = gzip.open if path.endswith(".gz") else open
    with op(path, "rt") as f:
        profs = json.load(f)

    dem = DEM(args.demdir05)
    tl = TREELINE[args.region]
    for cname, pred in CLS.items():
        accs = {w: ([], []) for w in (2., 5., 10.)}
        tot = 0
        for p in profs:
            if not pred(p["tags"]):
                continue
            s = np.array(p["s"]); z = np.array(p["z"]); x = np.array(p["x"]); y = np.array(p["y"])
            m = (z > tl) & np.array([(int(xi // 1000), int(yi // 1000)) in cells for xi, yi in zip(x, y)])
            idx = np.where(m)[0]
            if len(idx) < 12:
                continue
            for seg in np.split(idx, np.where(np.diff(idx) > 1)[0] + 1):
                if len(seg) < 16:
                    continue
                xx = x[seg]; yy = y[seg]
                z05 = np.array([dem.get(xi, yi) for xi, yi in zip(xx, yy)])
                if not np.isfinite(z05).all():
                    continue
                z05 = median_filter(z05, 3)
                ss = s[seg] - s[seg][0]; zz = z[seg]
                tot += ss[-1]
                for w in accs:
                    accs[w][0].extend(np.abs(win_slope_ls(ss, zz, w)))
                    accs[w][1].extend(np.abs(win_slope_ls(ss, z05, w)))
        if tot == 0:
            print(cname, "no paired segments")
            continue

        def qq(a):
            a = np.array(a); a = a[np.isfinite(a)]
            return [round(float(np.percentile(a, q)), 3) for q in (25, 50, 75, 90, 95)]
        print(f"{cname}: paired km={tot/1000:.1f}")
        for w in accs:
            a, b = accs[w]
            fa = float(np.nanmean(np.array(a) > 0.25)); fb = float(np.nanmean(np.array(b) > 0.25))
            print(f"  |g{int(w)}| 2m :", qq(a), " frac>25:", round(fa, 3))
            print(f"  |g{int(w)}| 0.5m:", qq(b), " frac>25:", round(fb, 3))


if __name__ == "__main__":
    main()
