# /// script
# requires-python = ">=3.9"
# dependencies = ["numpy"]
# ///
"""Fetch the Terrarium z13 elevation tiles needed by the scree crossings.

For every crossing in the crossings JSON: the tile under every polyline
vertex plus tiles under ~12 sampled scree-polygon boundary vertices (for the
polygon's elevation range). Downloads PNG tiles from the AWS
elevation-tiles-prod Terrarium bucket into a cache dir.

Terrarium encoding: elev_m = r*256 + g + b/256 - 32768.
z13 tile is ~4.8 x 4.8 km at 46 deg latitude, ~19 m/px native posting —
the audit findings call it "25-30 m tiles" loosely; pixel size at lat 46 is
19.0 m E-W x 23.9 m N-S effective.

Ported from /tmp/scree_tiles.py — identical logic, parameterised paths.
"""
import argparse
import json
import math
import os
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import urllib.request

Z = 13


def tile_of(lon, lat, z=Z):
    n = 2 ** z
    x = int((lon + 180.0) / 360.0 * n)
    lat_r = math.radians(lat)
    y = int((1.0 - math.log(math.tan(lat_r) + 1 / math.cos(lat_r)) / math.pi) / 2.0 * n)
    return (z, x, y)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--crossings", required=True)
    ap.add_argument("--tiles-dir", default="/tmp/tiles13")
    ap.add_argument("--workers", type=int, default=24)
    args = ap.parse_args()

    d = json.load(open(args.crossings))
    crossings = d["crossings"]

    tiles = set()
    for c in crossings:
        for lon, lat in c["coords"]:
            tiles.add(tile_of(lon, lat))
        b = c["scree_boundary"]
        if len(b) > 4:
            for idx in np.linspace(0, len(b) - 1, min(12, len(b))).astype(int):
                tiles.add(tile_of(*b[idx]))

    print("unique z13 tiles:", len(tiles))
    os.makedirs(args.tiles_dir, exist_ok=True)

    def fetch(t):
        z, x, y = t
        fn = f"{args.tiles_dir}/{z}_{x}_{y}.png"
        if os.path.exists(fn):
            return t, True
        url = f"https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"
        try:
            urllib.request.urlretrieve(url, fn)
            return t, True
        except Exception:  # noqa: BLE001
            return t, False

    tl = list(tiles)
    done = 0
    failed = 0
    with ThreadPoolExecutor(args.workers) as ex:
        for t, ok in ex.map(fetch, tl):
            done += 1
            failed += 0 if ok else 1
            if done % 200 == 0:
                print(f"{done}/{len(tl)}", flush=True)
    print("fetched; failed:", failed)


if __name__ == "__main__":
    main()