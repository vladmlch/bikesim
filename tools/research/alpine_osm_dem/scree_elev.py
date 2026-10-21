# /// script
# requires-python = ">=3.9"
# dependencies = ["numpy", "pillow"]
# ///
"""Elevation analysis of scree crossings on Terrarium z13 tiles.

Per crossing event (from the crossings JSON):
  - elevation profile resampled to 12 m spacing;
  - net grade, median/mean |local grade| (segments > 3 m);
  - TSA (trail-vs-slope angle): angle between the crossing's overall heading
    and the DEM downslope azimuth (3x3 Sobel at the crossing midpoint);
    0 deg = fall line, 90 deg = contour-parallel traverse;
  - terrain slope under the crossing midpoint;
  - pos_frac: elevation fraction of the crossing midpoint within the
    scree polygon's sampled min-max range;
  - way_med_grade: median |grade| of the whole parent way (15 m profile);
  - elev_min/elev_max of the crossing.

Writes crossing_elev JSON (list of dicts). Ported from /tmp/scree_elev.py —
identical logic, parameterised paths.
"""
import argparse
import json
import math
import os

import numpy as np
from PIL import Image

Z = 13
R = 6371000.0


def seg_len(a, b):
    dx = math.radians(b[0] - a[0]) * math.cos(math.radians((a[1] + b[1]) / 2))
    dy = math.radians(b[1] - a[1])
    return math.hypot(dx, dy) * R


class Terrarium:
    def __init__(self, tiles_dir):
        self.dir = tiles_dir
        self.tiles = {}

    def grid(self, z, x, y):
        key = (z, x, y)
        if key not in self.tiles:
            fn = f"{self.dir}/{z}_{x}_{y}.png"
            if not os.path.exists(fn):
                self.tiles[key] = None
            else:
                im = np.array(Image.open(fn))
                self.tiles[key] = (im[:, :, 0].astype(float) * 256
                                   + im[:, :, 1].astype(float)
                                   + im[:, :, 2].astype(float) / 256 - 32768)
        return self.tiles[key]


def lonlat2px(lon, lat):
    n = 2 ** Z * 256
    x = (lon + 180.0) / 360.0 * n
    lat_r = math.radians(lat)
    y = (1.0 - math.log(math.tan(lat_r) + 1 / math.cos(lat_r)) / math.pi) / 2.0 * n
    return x, y


def px2tile(px, py):
    return (Z, int(px // 256), int(py // 256)), (px % 256, py % 256)


def m_per_px(lat):
    return 40075016.686 / (2 ** Z * 256) * math.cos(math.radians(lat))


def heading_deg(a, b):
    dx = math.radians(b[0] - a[0]) * math.cos(math.radians((a[1] + b[1]) / 2))
    dy = math.radians(b[1] - a[1])
    return (math.degrees(math.atan2(dx, dy)) + 360) % 360


def line_profile(pts, step=12.0):
    if len(pts) < 2:
        return []
    out = [(0.0, pts[0][0], pts[0][1])]
    acc = 0.0
    for a, b in zip(pts, pts[1:]):
        L = seg_len(a, b)
        while acc + L >= step:
            frac = (step - acc) / L
            nx = a[0] + (b[0] - a[0]) * frac
            ny = a[1] + (b[1] - a[1]) * frac
            s_new = out[-1][0] + step
            out.append((s_new, nx, ny))
            a = (nx, ny)
            L = seg_len(a, b)
            acc = 0.0
        acc += L
    total = out[-1][0] + acc
    out.append((total, pts[-1][0], pts[-1][1]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--crossings", required=True)
    ap.add_argument("--data", required=True, help="scree_data JSON (for parent ways)")
    ap.add_argument("--tiles-dir", default="/tmp/tiles13")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    terr = Terrarium(args.tiles_dir)
    d = json.load(open(args.crossings))
    crossings = d["crossings"]
    d2 = json.load(open(args.data))
    coords = {int(k): tuple(v) for k, v in d2["coords"].items()}
    trails = {int(k): v for k, v in d2["trails"].items()}

    def elev(lon, lat):
        px, py = lonlat2px(lon, lat)
        (t, (lx, ly)) = px2tile(px, py)
        g = terr.grid(*t)
        if g is None:
            return None
        xi, yi = int(lx), int(ly)
        xi = min(max(xi, 0), 254); yi = min(max(yi, 0), 254)
        fx, fy = lx - xi, ly - yi
        h = (g[yi, xi] * (1 - fx) * (1 - fy) + g[yi, xi + 1] * fx * (1 - fy)
             + g[yi + 1, xi] * (1 - fx) * fy + g[yi + 1, xi + 1] * fx * fy)
        return h

    def aspect_deg(lon, lat):
        """Downslope azimuth (deg from north, 0-360) from 3x3 px Sobel."""
        px, py = lonlat2px(lon, lat)
        (t, (lx, ly)) = px2tile(px, py)
        g = terr.grid(*t)
        if g is None:
            return None, None
        xi, yi = int(lx), int(ly)
        if xi < 1 or yi < 1 or xi > 254 or yi > 254:
            return None, None
        win = g[yi - 1:yi + 2, xi - 1:xi + 2].astype(float)
        mpp = m_per_px(lat)
        gx = (win[0, 2] + 2 * win[1, 2] + win[2, 2] - win[0, 0] - 2 * win[1, 0] - win[2, 0]) / (8 * mpp)
        gy = (win[2, 0] + 2 * win[2, 1] + win[2, 2] - win[0, 0] - 2 * win[0, 1] - win[0, 2]) / (8 * mpp)
        slope = math.degrees(math.atan(math.hypot(gx, gy)))
        az_down = math.degrees(math.atan2(-gx, gy))
        az_down = (az_down + 360) % 360
        return az_down, slope

    results = []
    for i, c in enumerate(crossings):
        pts = [tuple(p) for p in c["coords"]]
        if len(pts) < 2:
            continue
        prof = line_profile(pts, step=12.0)
        hs = [elev(lon, lat) for _, lon, lat in prof]
        n_ok = sum(h is not None for h in hs)
        if n_ok < 2:
            continue
        hs_clean = [h for h in hs if h is not None]
        ss = [s for (s, _, _), h in zip(prof, hs) if h is not None]
        net_grade = ((hs_clean[-1] - hs_clean[0]) / max(ss[-1] - ss[0], 1e-6)
                     if ss[-1] - ss[0] > 1 else None)
        local = [abs((b - a) / (s2 - s1)) for a, b, s1, s2 in zip(hs_clean, hs_clean[1:], ss, ss[1:]) if s2 - s1 > 3]
        med_grade = float(np.median(local)) if local else None
        mean_grade = float(np.mean(local)) if local else None
        mid = prof[len(prof) // 2]
        az, terr_slope = aspect_deg(mid[1], mid[2])
        hd = heading_deg(pts[0], pts[-1])
        tsa = None
        if az is not None and c["len_m"] > 8:
            diff = abs(hd - az) % 180
            tsa = min(diff, 180 - diff)
        tw = trails.get(c["trail_id"])
        approach_grade = None
        if tw is not None:
            wpts = [coords.get(r) for r in tw["refs"]]
            wpts = [p for p in wpts if p]
            if len(wpts) >= 2:
                wprof = line_profile(wpts, step=15.0)
                whs = [elev(lon, lat) for _, lon, lat in wprof]
                wss = [p[0] for p in wprof]
                pairs = [(s1, s2, a, b) for s1, s2, a, b in zip(wss, wss[1:], whs, whs[1:]) if a is not None and b is not None and s2 - s1 > 5]
                if pairs:
                    approach_grade = float(np.median([abs((b - a) / (s2 - s1)) for s1, s2, a, b in pairs]))
        bpts = c["scree_boundary"]
        if len(bpts) > 4:
            idx = np.linspace(0, len(bpts) - 1, min(12, len(bpts))).astype(int)
            bh = [elev(*bpts[j]) for j in idx]
            bh = [h for h in bh if h is not None]
            if len(bh) >= 3 and max(bh) - min(bh) > 3:
                hmid = hs_clean[len(hs_clean) // 2]
                frac = (hmid - min(bh)) / (max(bh) - min(bh))
                relief = max(bh) - min(bh)
            else:
                frac = None; relief = None
        else:
            frac = None; relief = None
        results.append({
            "len": c["len_m"], "hw": c["hw"], "sac": c["sac"], "mtb": c["mtb"],
            "net_grade": net_grade, "med_grade": med_grade, "mean_grade": mean_grade,
            "tsa": tsa, "terr_slope": terr_slope, "pos_frac": frac, "scree_relief": relief,
            "way_med_grade": approach_grade,
            "elev_min": min(hs_clean), "elev_max": max(hs_clean),
        })
        if i % 3000 == 0:
            print(i, flush=True)

    json.dump(results, open(args.out, "w"))
    r = np.array([[x["len"], x["med_grade"] or np.nan, x["tsa"] or np.nan, x["terr_slope"] or np.nan,
                   x["pos_frac"] if x["pos_frac"] is not None else np.nan,
                   x["way_med_grade"] or np.nan, x["elev_min"]] for x in results])
    print("n:", len(results))
    print("crossing med_grade quantiles:", np.nanpercentile(r[:, 1], [10, 25, 50, 75, 90]))
    print("way med_grade quantiles:", np.nanpercentile(r[:, 5], [10, 25, 50, 75, 90]))
    print("tsa quantiles:", np.nanpercentile(r[:, 2], [10, 25, 50, 75, 90]))
    print("terrain slope at crossing:", np.nanpercentile(r[:, 3], [10, 25, 50, 75, 90]))
    print("pos_frac quantiles:", np.nanpercentile(r[:, 4], [10, 25, 50, 75, 90]))
    print("elev_min quantiles:", np.nanpercentile(r[:, 6], [5, 25, 50, 75, 95]))


if __name__ == "__main__":
    main()