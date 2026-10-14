# /// script
# requires-python = ">=3.9"
# dependencies = []
# ///
"""List swissALTI3D tiles covering each study region via the swisstopo STAC API.

STAC collection: ch.swisstopo.swissalti3d
Endpoint (v0.9):   https://data.geo.admin.ch/api/stac/v0.9/collections/
                   ch.swisstopo.swissalti3d/items?bbox=<w,s,e,n>&limit=100

Writes dem/tiles_<region>_full.json mapping tile id -> {"05": url, "2": url}
for the 0.5 m and 2 m GeoTIFF assets (EPSG:2056, height datum 5728).

NOTE (erratum vs the original audit text): the 2026-10-14 run downloaded the
2 m assets (`*_2_2056_5728.tif`), although `a-slope-profile/findings.md`
says "GSD 0.5 m". A paired check on ~25 km of segments found |g5| quantiles
and the >25 % share unchanged within ~1 pp — see dem_resolution_check.py and
docs/reference/alpine-trail-statistics.md.
"""
import argparse
import json
import os
import urllib.request

BBOXES = {
    "zermatt": "7.65,45.95,7.95,46.15",   # w,s,e,n (STAC order)
    "engadin": "9.70,46.30,9.95,46.52",
}

STAC = "https://data.geo.admin.ch/api/stac/v0.9/collections/ch.swisstopo.swissalti3d/items"


def items_for_bbox(bbox, out):
    url = f"{STAC}?bbox={bbox}&limit=100"
    tiles = {}
    while url:
        with urllib.request.urlopen(url, timeout=60) as r:
            d = json.load(r)
        for f in d.get("features", []):
            tid = f["id"]
            rec = {}
            for k, v in f.get("assets", {}).items():
                if k.endswith("_0.5_2056_5728.tif"):
                    rec["05"] = v["href"]
                elif k.endswith("_2_2056_5728.tif"):
                    rec["2"] = v["href"]
            tiles[tid] = rec
        url = None
        for l in d.get("links", []):
            if l.get("rel") == "next":
                url = l["href"]
    os.makedirs(os.path.dirname(out), exist_ok=True)
    json.dump(tiles, open(out, "w"))
    print(out, len(tiles),
          "with 2m:", sum(1 for t in tiles.values() if "2" in t),
          "with 0.5m:", sum(1 for t in tiles.values() if "05" in t))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--regions", nargs="+", default=list(BBOXES))
    ap.add_argument("--demdir", default="dem")
    args = ap.parse_args()
    for region in args.regions:
        items_for_bbox(BBOXES[region], os.path.join(args.demdir, f"tiles_{region}_full.json"))


if __name__ == "__main__":
    main()
