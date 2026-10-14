# /// script
# requires-python = ">=3.9"
# dependencies = ["osmium"]
# ///
"""Collect scree polygons and trail ways from a Geofabrik .osm.pbf extract.

Four sequential passes over the PBF (pyosmium SimpleHandler):
  1. collect closed/open ways tagged natural=scree|shingle, multipolygon
     relations with those tags, and all trail ways
     (highway in path|footway|track|bridleway|cycleway|steps) with a kept
     tag subset;
  2. collect coordinates of every referenced node;
  3. collect nodelists of untagged member ways of scree relations;
  4. collect coords of the extra nodes those member ways need.

Writes a single JSON dict:
  scree_ways {way_id: [node refs]}, scree_rel [{id, members}],
  member_lists {way_id: [refs]}, trails {way_id: {refs, tags}},
  coords {node_id: [lon, lat]}, surface_scree_way_ids.

Used in the audit as austria-261013.osm.pbf (812 MB) and
switzerland-latest.osm.pbf (548 MB) fetches of 2026-10-13/14 (Geofabrik
nightly). Ported from /tmp/scree_pass1{,_ch}.py — identical logic,
parameterised paths.
"""
import argparse
import json

import osmium

HWY = {"path", "footway", "track", "bridleway", "cycleway", "steps"}
KEEP_TAGS = ("highway", "surface", "sac_scale", "mtb:scale", "mtb:scale:uphill",
             "trail_visibility", "incline", "tracktype", "name")


class Collector(osmium.SimpleHandler):
    def __init__(self):
        super().__init__()
        self.scree_ways = {}      # way_id -> nodelist
        self.scree_rel = []       # (rel_id, [(role, way_id)])
        self.rel_member_ways = set()
        self.trails = {}          # way_id -> (nodelist, tags)
        self.surface_scree = []

    def way(self, w):
        tags = dict(w.tags)
        nat = tags.get("natural", "")
        if nat in ("scree", "shingle"):
            refs = [nd.ref for nd in w.nodes]
            if len(refs) >= 4 and refs[0] == refs[-1]:
                self.scree_ways[w.id] = refs
            elif len(refs) >= 2:
                self.scree_ways[w.id] = refs  # unclosed -> try later
        hw = tags.get("highway")
        if hw in HWY:
            keep = {k: v for k, v in tags.items() if k in KEEP_TAGS}
            self.trails[w.id] = ([nd.ref for nd in w.nodes], keep)
            if tags.get("surface") == "scree":
                self.surface_scree.append(w.id)

    def relation(self, r):
        tags = dict(r.tags)
        if tags.get("natural") in ("scree", "shingle") and tags.get("type") in ("multipolygon", "boundary"):
            members = [(m.role, m.ref) for m in r.members if m.type == "w"]
            self.scree_rel.append((r.id, members))
            for role, ref in members:
                self.rel_member_ways.add(ref)


class NodeGrab(osmium.SimpleHandler):
    def __init__(self, needed):
        super().__init__()
        self.needed = needed
        self.coords = {}

    def node(self, n):
        if n.id in self.needed:
            self.coords[n.id] = (n.location.lon, n.location.lat)


class MemberGrab(osmium.SimpleHandler):
    def __init__(self, ids):
        super().__init__()
        self.ids = ids
        self.lists = {}

    def way(self, w):
        if w.id in self.ids:
            self.lists[w.id] = [nd.ref for nd in w.nodes]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pbf", required=True, help="Geofabrik .osm.pbf extract")
    ap.add_argument("--out", required=True, help="output scree_data JSON")
    args = ap.parse_args()

    c = Collector()
    c.apply_file(args.pbf)
    print(f"scree ways: {len(c.scree_ways)}, scree relations: {len(c.scree_rel)}, trails: {len(c.trails)}", flush=True)

    needed = set()
    for refs in c.scree_ways.values():
        needed.update(refs)
    for refs, _ in c.trails.values():
        needed.update(refs)
    needed.update(c.rel_member_ways)
    print(f"needed nodes: {len(needed)}", flush=True)

    ng = NodeGrab(needed)
    ng.apply_file(args.pbf)
    print(f"got coords: {len(ng.coords)}", flush=True)

    member_lists = {}
    mg = MemberGrab(c.rel_member_ways)
    mg.apply_file(args.pbf)
    member_lists = mg.lists
    extra = set()
    for refs in member_lists.values():
        extra.update(refs)
    extra -= needed
    if extra:
        ng2 = NodeGrab(extra)
        ng2.apply_file(args.pbf)
        ng.coords.update(ng2.coords)
    print(f"member ways: {len(member_lists)}, total coords: {len(ng.coords)}", flush=True)

    out = {
        "scree_ways": c.scree_ways,
        "scree_rel": [{"id": rid, "members": m} for rid, m in c.scree_rel],
        "member_lists": member_lists,
        "trails": {str(k): {"refs": v[0], "tags": v[1]} for k, v in c.trails.items()},
        "coords": {str(k): v for k, v in ng.coords.items()},
        "surface_scree_way_ids": c.surface_scree,
    }
    with open(args.out, "w") as f:
        json.dump(out, f)
    print("saved", args.out, flush=True)


if __name__ == "__main__":
    main()
