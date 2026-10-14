# /// script
# requires-python = ">=3.9"
# dependencies = ["pyproj"]
# ///
"""Prepare LV95-projected way geometry and the needed swissALTI3D tile list.

Reads osm/<region>_paths.json (raw Overpass `out geom` response), writes
osm/<region>_ways_lv.json (ways with `lv` = EPSG:2056 node coordinates and
`pts` = 1 m resampled polyline) and osm/<region>_tiles.json (list of
1x1 km LV95 cells the resampled ways touch — the DEM tiles to download).

Run from the work directory that contains osm/. Ported verbatim from the
audit pipeline (/tmp/alpslope/prepare.py); only an argparse thin-wrapper
was added.
"""
import argparse
import json
import math
import os

from pyproj import Transformer

T = Transformer.from_crs("EPSG:4326", "EPSG:2056", always_xy=True)


def lv95(lon, lat):
    return T.transform(lon, lat)


def load_ways(path):
    d = json.load(open(path))
    out = []
    for e in d["elements"]:
        if e.get("type") != "way" or "geometry" not in e:
            continue
        coords = [(g["lon"], g["lat"]) for g in e["geometry"]]
        out.append({"id": e["id"], "tags": e.get("tags", {}), "coords": coords})
    return out


def resample(coords_lv, step=1.0):
    pts = []
    acc = 0.0
    px, py = coords_lv[0]
    pts.append((px, py))
    for (x, y) in coords_lv[1:]:
        d = math.hypot(x - px, y - py)
        while acc + d >= step:
            f = (step - acc) / d
            px, py = px + f * (x - px), py + f * (y - py)
            pts.append((px, py))
            d = math.hypot(x - px, y - py)
            acc = 0.0
        acc += d
        px, py = x, y
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--regions", nargs="+", default=["zermatt", "engadin"])
    ap.add_argument("--osmdir", default="osm")
    args = ap.parse_args()

    for region in args.regions:
        ways = load_ways(os.path.join(args.osmdir, f"{region}_paths.json"))
        tiles = set()
        nsel = 0
        for w in ways:
            w["lv"] = [lv95(lon, lat) for lon, lat in w["coords"]]
            w["pts"] = resample(w["lv"])
            if len(w["pts"]) < 6:
                continue
            nsel += 1
            for x, y in w["pts"]:
                tiles.add((int(x // 1000), int(y // 1000)))
        json.dump(ways, open(os.path.join(args.osmdir, f"{region}_ways_lv.json"), "w"))
        json.dump(sorted(tiles), open(os.path.join(args.osmdir, f"{region}_tiles.json"), "w"))
        print(region, "ways selected:", nsel, "tiles needed:", len(tiles))


if __name__ == "__main__":
    main()
