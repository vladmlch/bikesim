# /// script
# requires-python = ">=3.9"
# dependencies = ["numpy"]
# ///
"""Final scree-crossing statistics (findings F1-F4) from crossing records.

Reads the slim crossing tables (scree_crossings_{at,ch}.slim.json[.gz]) and
the per-crossing elevation table (scree_crossing_elev_at.json[.gz]) shipped
in data/. Emits a JSON summary + printed table matching the audit's
d-scree-sections findings.

This script did not exist in the audit scratch — the tables were computed
ad hoc. The numbers it reproduces on the pinned inputs:
  AT all-trails n=15211: len p5/25/50/75/95/99 = 3.1/11.4/31.0/97.8/448.6/995.3 m
  AT path n=12390:       3.3/11.7/33.7/109.3/495.1/1044.6 m
  AT alpine path (>1600 m, n=9012): p25/50/75 = 14.4/45.7/146.2 m, p95 = 581 m
  CH all-trails n=11565: 4.2/17.7/57.6/170.3/629.6/1244.8 m; path med 64.4 m
  AT grade in scree (alpine): p10..p90 = 0.06/0.11/0.21/0.32/0.44
  TSA classes (alpine): fall <30 deg 25.9 %, diagonal 27.7 %, traverse 46.4 %
"""
import argparse
import gzip
import json
import os

import numpy as np


def load(path):
    op = gzip.open if path.endswith(".gz") else open
    with op(path, "rt") as f:
        return json.load(f)


def q(vals, ps=(5, 25, 50, 75, 95, 99)):
    a = np.array([v for v in vals if v is not None and np.isfinite(v)])
    return [float(np.percentile(a, p)) for p in ps] if len(a) else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--datadir", default=os.path.join(os.path.dirname(__file__), "data"))
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    def find(base):
        for cand in (base, base + ".gz"):
            if os.path.exists(cand):
                return cand
        raise FileNotFoundError(base)

    summary = {}
    for tag in ("at", "ch"):
        d = load(find(os.path.join(args.datadir, f"scree_crossings_{tag}.slim.json")))
        cr = d["crossings"]
        lens = [c["len_m"] for c in cr]
        rec = {
            "n_crossings": len(cr),
            "trail_km_total": d["trail_km_total"],
            "trail_km_path_footway": d["trail_km_path"],
            "len_quantiles_all": q(lens),
        }
        for hw in ("path", "footway", "track"):
            sel = [c["len_m"] for c in cr if c["hw"] == hw]
            if sel:
                rec[f"n_{hw}"] = len(sel)
                rec[f"len_quantiles_{hw}"] = q(sel)
                rec[f"km_in_scree_{hw}"] = sum(sel) / 1000
        sac = [c["len_m"] for c in cr if c.get("sac")]
        mtb = [c["len_m"] for c in cr if c.get("mtb") or c.get("mtb_up")]
        rec["n_sac"] = len(sac); rec["len_median_sac"] = q(sac, (50,))[0] if sac else None
        rec["n_mtb"] = len(mtb); rec["len_quantiles_mtb"] = q(mtb)
        n_path = rec.get("n_path", 0)
        rec["events_per_100km_path_footway"] = 100 * n_path / d["trail_km_path"]
        rec["share_path_km_in_scree"] = rec.get("km_in_scree_path", 0) / d["trail_km_path"]
        summary[tag] = rec

    elev = load(find(os.path.join(args.datadir, "scree_crossing_elev_at.json")))
    alpine = [x for x in elev if x["hw"] == "path" and x["elev_min"] is not None and x["elev_min"] > 1600]
    rec = {"n_alpine_path_at": len(alpine)}
    rec["len_quantiles_alpine"] = q([x["len"] for x in alpine])
    rec["med_grade_in_scree_q"] = q([x["med_grade"] for x in alpine], (10, 25, 50, 75, 90))
    rec["way_med_grade_q"] = q([x["way_med_grade"] for x in alpine], (10, 25, 50, 75, 90))
    mg = [x["med_grade"] for x in alpine if x["med_grade"] is not None]
    rec["frac_grade_ge15"] = float(np.mean([g >= 0.15 for g in mg]))
    rec["frac_grade_ge20"] = float(np.mean([g >= 0.20 for g in mg]))
    rec["frac_grade_lt10"] = float(np.mean([g < 0.10 for g in mg]))
    short = [x["med_grade"] for x in alpine if x["med_grade"] is not None and x["len"] < 25]
    rec["short_lt25m_median_grade"] = float(np.median(short))
    rec["short_lt25m_frac_ge15"] = float(np.mean([g >= 0.15 for g in short]))
    tsa = [x["tsa"] for x in alpine if x["tsa"] is not None]
    n = len(tsa)
    rec["tsa_n"] = n
    rec["tsa_fall_lt30"] = float(np.mean([t < 30 for t in tsa]))
    rec["tsa_diag_30_60"] = float(np.mean([30 <= t <= 60 for t in tsa]))
    rec["tsa_traverse_gt60"] = float(np.mean([t > 60 for t in tsa]))
    for name, sel in (("fall", [x for x in alpine if x["tsa"] is not None and x["tsa"] < 30]),
                      ("diag", [x for x in alpine if x["tsa"] is not None and 30 <= x["tsa"] <= 60]),
                      ("traverse", [x for x in alpine if x["tsa"] is not None and x["tsa"] > 60])):
        rec[f"med_grade_{name}"] = float(np.median([x["med_grade"] for x in sel if x["med_grade"] is not None]))
        rec[f"terr_slope_{name}"] = float(np.median([x["terr_slope"] for x in sel if x["terr_slope"] is not None]))
    rec["terr_slope_q"] = q([x["terr_slope"] for x in alpine], (10, 50, 90))
    rec["pos_frac_q"] = q([x["pos_frac"] for x in alpine], (10, 50, 90))
    summary["at_alpine"] = rec

    print(json.dumps(summary, indent=1, sort_keys=True))
    if args.out:
        json.dump(summary, open(args.out, "w"), indent=1, sort_keys=True)


if __name__ == "__main__":
    main()
