# /// script
# requires-python = ">=3.9"
# dependencies = []
# ///
"""Fetch OSM trail ways for a study region from an Overpass API mirror.

Query (fixed; matches the 2026-10-14 audit run — a same-day re-fetch
returned an identical way-ID set for both regions):

    [out:json][timeout:180];
    (
      way(<S>,<W>,<N>,<E>)["highway"="path"]["sac_scale"];
      way(<S>,<W>,<N>,<E>)["highway"="path"]["mtb:scale"];
      way(<S>,<W>,<N>,<E>)["highway"="path"]["mtb:scale:uphill"];
    );
    out geom;

bbox is (south, west, north, east). Usage:

    uv run fetch_osm_paths.py --region zermatt --outdir osm

Writes osm/<region>_paths.json (raw Overpass JSON with inline geometry).
"""
import argparse
import json
import urllib.parse
import urllib.request

REGIONS = {
    "zermatt": (45.95, 7.65, 46.15, 7.95),
    "engadin": (46.30, 9.70, 46.52, 9.95),
}

OVERPASS_ENDPOINTS = [
    "https://overpass.osm.ch/api/interpreter",
    "https://overpass-api.de/api/interpreter",
]

QUERY_TEMPLATE = """[out:json][timeout:180];
(
  way({s},{w},{n},{e})["highway"="path"]["sac_scale"];
  way({s},{w},{n},{e})["highway"="path"]["mtb:scale"];
  way({s},{w},{n},{e})["highway"="path"]["mtb:scale:uphill"];
);
out geom;
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--region", choices=sorted(REGIONS), required=True)
    ap.add_argument("--outdir", default="osm")
    ap.add_argument("--endpoint", default=OVERPASS_ENDPOINTS[0])
    args = ap.parse_args()

    s, w, n, e = REGIONS[args.region]
    query = QUERY_TEMPLATE.format(s=s, w=w, n=n, e=e)
    body = urllib.parse.urlencode({"data": query}).encode()
    with urllib.request.urlopen(urllib.request.Request(args.endpoint, data=body), timeout=300) as r:
        data = json.load(r)
    import os

    os.makedirs(args.outdir, exist_ok=True)
    out = os.path.join(args.outdir, f"{args.region}_paths.json")
    with open(out, "w") as f:
        json.dump(data, f)
    print(out, "elements:", len(data.get("elements", [])),
          "osm_base:", data.get("osm3s", {}).get("timestamp_osm_base"))


if __name__ == "__main__":
    main()
