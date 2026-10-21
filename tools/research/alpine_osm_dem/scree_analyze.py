# /// script
# requires-python = ">=3.9"
# dependencies = ["numpy", "shapely"]
# ///
"""Intersect trail ways with scree polygons -> crossing events.

Reads the scree_data JSON written by scree_collect.py. Builds shapely
polygons from closed ways and multipolygon relations (member ways stitched
into rings), indexes them with STRtree, and for every trail way computes
the parts lying inside a polygon. Each contiguous part >= 1 m is one
crossing event (len_m on an equirectangular approximation).

Writes crossings JSON: per event {trail_id, scree_id, len_m, hw, surface,
sac, mtb, mtb_up, vis, coords, scree_boundary} plus trail_km totals.

Ported from /tmp/scree_analyze{,_ch}.py — identical logic, parameterised
paths.
"""
import argparse
import json
import math

import numpy as np
from shapely.geometry import Polygon, LineString
from shapely.ops import unary_union
from shapely.validation import make_valid
from shapely.strtree import STRtree

R = 6371000.0


def seg_len(a, b):
    dx = math.radians(b[0] - a[0]) * math.cos(math.radians((a[1] + b[1]) / 2))
    dy = math.radians(b[1] - a[1])
    return math.hypot(dx, dy) * R


def refs_to_ring(refs, coords):
    return [coords[r] for r in refs if r in coords]


def stitch(ways):
    """Stitch way nodelists into closed rings; return list of rings."""
    unused = {i: list(w) for i, w in enumerate(ways)}
    rings = []
    while unused:
        i, chain = unused.popitem()
        closed = chain[0] == chain[-1]
        while not closed:
            extended = False
            for j, w in list(unused.items()):
                if chain[-1] == w[0]:
                    chain += w[1:]
                elif chain[-1] == w[-1]:
                    chain += list(reversed(w[:-1]))
                elif chain[0] == w[-1]:
                    chain = w[:-1] + chain
                elif chain[0] == w[0]:
                    chain = list(reversed(w[1:])) + chain
                else:
                    continue
                del unused[j]
                extended = True
                closed = chain[0] == chain[-1]
                break
            if not extended:
                break
        if closed:
            rings.append(chain)
    return rings


def build_polygons(d, coords):
    polys = []   # (id, shapely geom)
    bad = 0
    for wid, refs in d["scree_ways"].items():
        if refs[0] != refs[-1]:
            bad += 1
            continue
        pts = refs_to_ring(refs, coords)
        if len(pts) < 4:
            bad += 1
            continue
        try:
            p = Polygon(pts)
            if not p.is_valid:
                p = make_valid(p)
            if p.is_empty:
                bad += 1
                continue
            if p.geom_type == "Polygon":
                polys.append((wid, p))
            elif p.geom_type == "MultiPolygon":
                polys += [(wid, g) for g in p.geoms]
        except Exception:  # noqa: BLE001
            bad += 1

    rel_ok = rel_bad = 0
    ml = {int(k): v for k, v in d["member_lists"].items()}
    for rel in d["scree_rel"]:
        outers, inners = [], []
        for role, ref in rel["members"]:
            w = ml.get(ref)
            if w is None:
                continue
            (outers if role == "outer" else inners).append(w)
        outer_rings = stitch(outers)
        inner_rings = stitch(inners)
        shells = [refs_to_ring(ring, coords) for ring in outer_rings]
        shells = [s for s in shells if len(s) >= 4]
        holes = [refs_to_ring(ring, coords) for ring in inner_rings]
        holes = [h for h in holes if len(h) >= 4]
        try:
            if not shells:
                rel_bad += 1
                continue
            # naive: first shell gets all holes (good enough for stats)
            p = Polygon(shells[0], holes if len(shells) == 1 else [])
            for s in shells[1:]:
                p = unary_union([p, Polygon(s)])
            if not p.is_valid:
                p = make_valid(p)
            if p.geom_type == "Polygon":
                polys.append((rel["id"], p))
                rel_ok += 1
            elif p.geom_type == "MultiPolygon":
                polys += [(rel["id"], g) for g in p.geoms]
                rel_ok += 1
            else:
                rel_bad += 1
        except Exception:  # noqa: BLE001
            rel_bad += 1
    return polys, bad, rel_ok, rel_bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="scree_data JSON from scree_collect.py")
    ap.add_argument("--out", required=True, help="crossings JSON")
    args = ap.parse_args()

    d = json.load(open(args.data))
    coords = {int(k): tuple(v) for k, v in d["coords"].items()}

    polys, bad, rel_ok, rel_bad = build_polygons(d, coords)
    print(f"polygons: {len(polys)} (closed-way failures {bad}, rel ok {rel_ok}, rel bad {rel_bad})")

    geoms = [p for _, p in polys]
    pids = [i for i, _ in polys]
    tree = STRtree(geoms)

    crossings = []
    trail_km_total = 0.0
    trail_km_alpine = 0.0   # path/footway only
    n_trails_hit = 0
    for wid_s, info in d["trails"].items():
        refs = info["refs"]
        tags = info["tags"]
        pts = [coords.get(r) for r in refs]
        pts = [p for p in pts if p]
        if len(pts) < 2:
            continue
        L = sum(seg_len(a, b) for a, b in zip(pts, pts[1:]))
        trail_km_total += L / 1000
        if tags.get("highway") in ("path", "footway"):
            trail_km_alpine += L / 1000
        ls = LineString(pts)
        hit = False
        for gi in tree.query(ls):
            p = geoms[gi]
            inter = ls.intersection(p)
            if inter.is_empty:
                continue
            parts = []
            if inter.geom_type == "LineString":
                parts = [inter]
            elif inter.geom_type == "MultiLineString":
                parts = list(inter.geoms)
            elif inter.geom_type == "GeometryCollection":
                parts = [g for g in inter.geoms if g.geom_type in ("LineString",)]
            for part in parts:
                pl = sum(seg_len(a, b) for a, b in zip(part.coords, part.coords[1:]))
                if pl < 1.0:
                    continue
                crossings.append({
                    "trail_id": int(wid_s), "scree_id": pids[gi],
                    "len_m": pl, "hw": tags.get("highway"),
                    "surface": tags.get("surface"), "sac": tags.get("sac_scale"),
                    "mtb": tags.get("mtb:scale"), "mtb_up": tags.get("mtb:scale:uphill"),
                    "vis": tags.get("trail_visibility"),
                    "coords": [list(c) for c in part.coords],
                    "scree_boundary": [list(c) for c in p.exterior.coords][:2000],
                })
                hit = True
        if hit:
            n_trails_hit += 1

    print(f"trail km total {trail_km_total:.0f}, path/footway {trail_km_alpine:.0f}")
    print(f"trail ways crossing scree: {n_trails_hit}, crossings: {len(crossings)}")
    lens = np.array([c["len_m"] for c in crossings])
    print("len quantiles:", np.percentile(lens, [5, 25, 50, 75, 95, 99]).round(1))
    for hw in ("path", "footway", "track", "bridleway"):
        sel = lens[[c["hw"] == hw for c in crossings]]
        if len(sel):
            print(hw, len(sel), "median", np.median(sel).round(1), "km_in_scree", (sel.sum() / 1000).round(1))

    with open(args.out, "w") as f:
        json.dump({"crossings": crossings,
                   "trail_km_total": trail_km_total,
                   "trail_km_path": trail_km_alpine}, f)
    print("saved", args.out)


if __name__ == "__main__":
    main()