# /// script
# requires-python = ">=3.9"
# dependencies = []
# ///
"""Download the swissALTI3D tiles needed by the OSM ways.

Reads dem/tiles_<region>_full.json (STAC index, see fetch_dem_index.py) and
osm/<region>_tiles.json (needed 1 km cells, see prepare.py), writes
dem/<region>/<tile>.tif files.

`--res 2` selects the 2 m assets (what the 2026-10-14 audit used);
`--res 05` selects the 0.5 m assets (used for the resolution erratum check).
"""
import argparse
import concurrent.futures
import json
import os
import urllib.request


def build_list(region, res, demdir, osmdir):
    index = json.load(open(os.path.join(demdir, f"tiles_{region}_full.json")))
    needed = json.load(open(os.path.join(osmdir, f"{region}_tiles.json")))
    keymap = {}
    for tid in index:
        coord = tid.split("_", 2)[2]          # swissalti3d_2024_2620-1094
        ex, ny = coord.split("-")
        keymap[(int(ex), int(ny))] = tid
    urls, missing = [], []
    for e, n in needed:
        tid = keymap.get((e, n))
        if tid is None or res not in index[tid]:
            missing.append((e, n))
        else:
            urls.append((tid, index[tid][res]))
    print(region, "needed", len(needed), "found", len(urls), "missing", len(missing), missing[:8])
    return urls


def dl(job):
    outdir, tid, url = job
    f = os.path.join(outdir, f"{tid}.tif")
    if os.path.exists(f) and os.path.getsize(f) > 10000:
        return f
    try:
        urllib.request.urlretrieve(url, f + ".tmp")
        os.rename(f + ".tmp", f)
        return f
    except Exception as e:  # noqa: BLE001 - report and continue
        return f"FAIL {tid} {e}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--regions", nargs="+", default=["zermatt", "engadin"])
    ap.add_argument("--res", choices=["2", "05"], default="2")
    ap.add_argument("--demdir", default="dem")
    ap.add_argument("--osmdir", default="osm")
    ap.add_argument("--workers", type=int, default=16)
    args = ap.parse_args()

    jobs = []
    for region in args.regions:
        outdir = os.path.join(args.demdir, region if args.res == "2" else f"{region}_05m")
        os.makedirs(outdir, exist_ok=True)
        for tid, url in build_list(region, args.res, args.demdir, args.osmdir):
            jobs.append((outdir, tid, url))
    print("jobs:", len(jobs))
    with concurrent.futures.ThreadPoolExecutor(args.workers) as ex:
        res = list(ex.map(dl, jobs))
    fails = [r for r in res if r.startswith("FAIL")]
    print("done, fails:", len(fails))
    for f in fails[:10]:
        print(f)


if __name__ == "__main__":
    main()
