# /// script
# requires-python = ">=3.9"
# dependencies = ["numpy", "scipy", "rasterio"]
# ///
"""Sample trail elevation profiles z(s) from swissALTI3D tiles.

For every way in osm/<region>_ways_lv.json: resample the LV95 polyline to
2 m spacing, bilinear-sample the DEM, split at NaN runs (uncovered cells),
median-filter z over 3 samples, and emit one record per contiguous covered
segment to osm/<region>_profiles.json.

Ported verbatim from the audit pipeline (/tmp/alpslope/analyze.py); the only
change is an argparse wrapper. The profile JSON is the input of stats2.py.
"""
import argparse
import json
import math
import os

import numpy as np
import rasterio
from scipy.ndimage import median_filter

STEP = 2.0  # m resample


class DEM:
    """Bilinear sampler over a directory of swissALTI3D GeoTIFF tiles."""

    def __init__(self, dirpath):
        self.cache = {}
        self.dir = dirpath
        self.files = {f[:-4]: f for f in os.listdir(self.dir) if f.endswith(".tif")}

    def get(self, x, y):
        # bilinear sample at LV95 coords; returns z or nan
        ex, ny = int(x // 1000), int(y // 1000)
        key = f"{ex}-{ny}"
        tid = None
        for f in self.files:
            if f.endswith(f"_{key}"):
                tid = f
                break
        if tid is None:
            return np.nan
        if tid not in self.cache:
            ds = rasterio.open(os.path.join(self.dir, self.files[tid]))
            self.cache[tid] = (ds.read(1), ds.transform)
            ds.close()
        arr, tr = self.cache[tid]
        # tr: x = tr.c + col*tr.a ; y = tr.f + row*tr.e (e negative)
        col = (x - tr.c) / tr.a - 0.5
        row = (y - tr.f) / tr.e - 0.5
        if row < 0 or col < 0 or row > arr.shape[0] - 1 or col > arr.shape[1] - 1:
            return np.nan
        r0, c0 = int(row), int(col)
        fr, fc = row - r0, col - c0
        r1, c1 = min(r0 + 1, arr.shape[0] - 1), min(c0 + 1, arr.shape[1] - 1)
        z = (arr[r0, c0] * (1 - fr) * (1 - fc) + arr[r1, c0] * fr * (1 - fc) +
             arr[r0, c1] * (1 - fr) * fc + arr[r1, c1] * fr * fc)
        return float(z)


def resample(lvpts, step=STEP):
    pts = [lvpts[0]]
    px, py = lvpts[0]
    acc = 0.0
    for x, y in lvpts[1:]:
        d = math.hypot(x - px, y - py)
        while acc + d >= step:
            f = (step - acc) / d
            px, py = px + f * (x - px), py + f * (y - py)
            pts.append((px, py))
            d = math.hypot(x - px, y - py)
            acc = 0.0
        acc += d
        px, py = x, y
    return np.array(pts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--regions", nargs="+", default=["zermatt", "engadin"])
    ap.add_argument("--osmdir", default="osm")
    ap.add_argument("--demdir", default="dem")
    args = ap.parse_args()

    for region in args.regions:
        ways = json.load(open(os.path.join(args.osmdir, f"{region}_ways_lv.json")))
        dem = DEM(os.path.join(args.demdir, region))
        out = []
        for w in ways:
            lv = [tuple(p) for p in w["lv"]]
            if len(lv) < 2:
                continue
            pts = resample(lv)
            if len(pts) < 8:
                continue
            z = np.array([dem.get(x, y) for x, y in pts])
            ok = np.isfinite(z)
            s = np.arange(len(pts)) * STEP
            idx = np.where(ok)[0]
            if len(idx) == 0:
                continue
            splits = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1)
            for seg in splits:
                if len(seg) < 8:
                    continue
                sseg = s[seg]
                zseg = z[seg]
                pseg = pts[seg]
                # median-filter elevation over 3 samples to kill spikes
                zs = median_filter(zseg, 3)
                rec = {
                    "id": w["id"], "region": region, "tags": w["tags"],
                    "s": sseg.tolist(), "z": zs.tolist(),
                    "x": pseg[:, 0].tolist(), "y": pseg[:, 1].tolist(),
                }
                out.append(rec)
        json.dump(out, open(os.path.join(args.osmdir, f"{region}_profiles.json"), "w"))
        print(region, "profiles:", len(out), "samples:", sum(len(r["s"]) for r in out))


if __name__ == "__main__":
    main()
