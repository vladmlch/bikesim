# alpine_osm_dem — reproducible OSM × DEM alpine-trail statistics

> **Snapshot availability (2026-10-21):** This Repomix-based delivery
> includes the expected statistics JSON recovered from the supplied audit
> result and the two tile lists. The nine compressed datasets listed below
> are absent from the supplied snapshots; the offline commands need the
> original pinned files. The dated verification table was not rerun.
> See the [merge availability record](../../../migration/cpp-linux-task103-source-merge.md#inputs-absent-from-the-supplied-snapshots).

Tools that generate the numbers in
[`docs/reference/alpine-trail-statistics.md`](../../../docs/reference/alpine-trail-statistics.md)
(the canonical numeric source for `AlpineGenSpec` v2). The package ports the
2026-10-14 audit pipeline (`.scratch/track-research/a-slope-profile/src/` and
`d-scree-sections/`) into reproducible form and ships the pinned intermediate
datasets in `data/` so the published statistics can be regenerated offline.

Every script carries a PEP-723 header — run with `uv run <script>.py …`
from inside this directory; `uv` resolves dependencies automatically.

## What is reproduced

| Claim (audit D2/D3, master plan §3) | Reproduced | Where |
|---|---|---|
| T3 technical trail: ~35–38 % of length at >25 % grade | 37.5 % (Zermatt) / 33.3 % (Engadin) at 5 m window; band-fraction path: 38.2 % / 35.1 % | `stats2_out`, `bandfrac` |
| Median constant-grade run 4–8 m | 4–8 m medians across all bands/classes (run length = n_samples × 2 m) | `stats2_out` `run_*` |
| Scree crossing median 34–64 m | AT path 33.7 m / CH path 64.4 m | `scree_stats` |

`expected/stats2_out.json` is the byte-for-byte reference from the audit;
`compare_stats.py` is the gate.

## Data snapshots (`data/`)

| File | Contents | Source |
|---|---|---|
| `{zermatt,engadin}_paths.json.gz` | Raw Overpass `out geom` responses (976 / 951 `highway=path` ways) | Overpass fetch 2026-10-14, query below |
| `{zermatt,engadin}_ways_lv.json.gz` | Ways projected to LV95 + 1 m resampled `pts` | `prepare.py` output |
| `{zermatt,engadin}_tiles.json` | 1 km LV95 cells touched (DEM tile list) | `prepare.py` output |
| `{zermatt,engadin}_profiles.json.gz` | Per-segment `s`, `z`, `x`, `y`, `tags` at 2 m step on the 2 m swissALTI3D DEM | `analyze_profiles.py` output |
| `scree_crossings_{at,ch}.slim.json.gz` | Crossing events `{trail_id, scree_id, len_m, hw, surface, sac, mtb, mtb_up, vis}` + `trail_km_total`, `trail_km_path` | `scree_collect.py` + `scree_analyze.py` outputs (slimmed: coords dropped) |
| `scree_crossing_elev_at.json.gz` | Per-crossing Terrarium metrics (grades, TSA, terrain slope, pos_frac, elev range) | `scree_elev.py` output |
| `expected/stats2_out.json` | Reference stats output of the audit | bit-identical to `.scratch/track-research/a-slope-profile/src/stats2_out.json` |

The slope snapshots let all profile statistics be regenerated without
Overpass/swisstopo access. The scree *raw* collections (`scree_data.json` ≈ 1 GB
each, full crossing geometry ≈ 44–75 MB) were too large to commit; the PBF
inputs (Geofabrik `austria-261013.osm.pbf`, `switzerland-latest.osm.pbf`,
fetched 2026-10-13/14) are re-downloadable and the collector output was
verified count-identical on re-run (see Verification).

## Slope pipeline (per region)

```sh
# 1. OSM ways — fixed query, regions zermatt|engadin
uv run fetch_osm_paths.py --region zermatt --outdir work/osm

# 2. project to LV95, resample 1 m, list DEM cells
uv run prepare.py --regions zermatt --osmdir work/osm

# 3. swissALTI3D tile index (STAC v0.9) and download (2 m = audit choice)
uv run fetch_dem_index.py --regions zermatt --demdir work/dem
uv run download_tiles.py --regions zermatt --res 2 --demdir work/dem --osmdir work/osm

# 4. profiles z(s) at 2 m step + statistics
uv run analyze_profiles.py --regions zermatt --osmdir work/osm --demdir work/dem
uv run stats2.py --regions zermatt --profiles-dir work/osm --out stats2_out.json
uv run bandfrac.py   --profiles-dir work/osm
uv run descfrac.py   --profiles-dir work/osm
uv run dense_check.py --osmdir work/osm
uv run noise_check.py --profiles-dir work/osm
```

Or regenerate the published numbers directly from the pinned snapshots:

```sh
uv run stats2.py --profiles-dir data --out /tmp/stats2_out.json
uv run compare_stats.py /tmp/stats2_out.json      # gate vs expected/
uv run bandfrac.py   --profiles-dir data
uv run descfrac.py   --profiles-dir data
uv run dense_check.py                            # uses data/
uv run noise_check.py --profiles-dir data
uv run scree_stats.py --datadir data             # F1–F4 tables
```

### Fixed methodology (slope)

- **Regions / bboxes** (S,W,N,E): Zermatt `45.95,7.65,46.15,7.95`;
  Engadin `46.30,9.70,46.52,9.95`.
- **Overpass query** (`fetch_osm_paths.py`, `timeout 180`, `out geom`):
  `way(bbox)["highway"="path"]["sac_scale"]` ∪ `["mtb:scale"]` ∪
  `["mtb:scale:uphill"]`. Fetch date: **2026-10-14** (audit run). The saved
  response's `osm3s.timestamp_osm_base` field reads `"117722"` — an anomalous
  value in the API response itself; the fetch date is taken from the audit
  record, not from that field.
- **Classes**: T2 = `sac_scale=mountain_hiking`; T3 =
  `demanding_mountain_hiking`; T45 = `alpine/demanding/difficult_alpine_hiking`;
  `mtb` = any `mtb:scale*`; `flat_control` = proxy class
  (z < treeline−300 m **and** |g20| < 5 %) — it filters *trail-profile*
  flatness, it is **not** a true terrain-slope control.
- **DEM**: swisstopo `swissALTI3D`, STAC `ch.swisstopo.swissalti3d`,
  assets `*_2_2056_5728.tif` → **2 m posting** (500×500 px per 1 km cell),
  EPSG:2056, height datum LN02-ish (5728 code). *Erratum:* the audit text
  says 0.5 m; the downloaded run used 2 m. `dem_resolution_check.py`
  quantifies the difference on paired cells (negligible vs OSM line error;
  see the reference doc §Limitations).
- **Profile**: way → LV95 (pyproj 4326→2056) → resample 2 m → bilinear DEM
  sample → split at NaN → median-filter z (3 samples) → contiguous segments
  ≥ 16 samples kept by stats.
- **Grade**: `win_slope_ls` — least-squares dz/ds over centred ±w/2 window,
  w ∈ {2, 5, 10, 20 m}; tables use |g5|.
- **Treeline mask**: z > 2100 m (Zermatt), z > 2200 m (Engadin).
- **Runs**: consecutive samples of |g5| inside one band
  (<8 / 8–15 / 15–25 / >25 %); run length = samples × 2 m. Quantiles
  p25/50/75/90.
- **Reversals**: gaussian σ = 2 m, local max–min–max, dip counts if drop
  ≥ 0.30 m and length 1.5–30 m.
- **Switchbacks**: heading change > 120° over 15 m window (dense_check uses
  100°/10 m on ways with median node spacing ≤ 6 m).

## Scree pipeline (Austria + Switzerland)

```sh
uv run scree_collect.py --pbf austria.osm.pbf --out work/scree_data_at.json
uv run scree_analyze.py --data work/scree_data_at.json --out work/crossings_at.json
uv run scree_tiles.py   --crossings work/crossings_at.json --tiles-dir work/tiles13
uv run scree_elev.py    --crossings work/crossings_at.json --data work/scree_data_at.json \
                        --tiles-dir work/tiles13 --out work/crossing_elev_at.json
uv run scree_stats.py   --datadir data      # publishes F1–F4 tables
```

- **Inputs**: Geofabrik nightly extracts `austria-261013.osm.pbf` (812 MB,
  snapshot dated 2026-10-13 per filename convention) and
  `switzerland-latest.osm.pbf` (548 MB, fetched 2026-10-14).
- **Polygons**: `natural=scree|shingle` closed ways + multipolygon/boundary
  relations (member ways stitched into rings; invalid rings repaired with
  `make_valid`). AT: 24,863 ways + 2,025 relations collected; polygons built
  26,974 / 19,428 incl. relation geometry.
- **Trails**: `highway ∈ {path, footway, track, bridleway, cycleway, steps}`
  (1,240,171 ways AT).
- **Crossing event**: contiguous part of a trail line inside a polygon,
  equirectangular length ≥ 1 m.
- **Elevation layer**: AWS Terrarium z13 PNG tiles
  (`https://s3.amazonaws.com/elevation-tiles-prod/terrarium/13/x/y.png`,
  decode `R*256 + G + B/256 − 32768`, ~19–24 m/px at 46–47° N).
  Per crossing: 12 m-resampled profile → median |local grade|; TSA = angle
  between crossing heading and DEM downslope azimuth (3×3 Sobel at midpoint);
  terrain slope at midpoint; `pos_frac` = midpoint elevation fraction within
  the polygon's sampled min–max; `way_med_grade` = median |grade| of the
  whole parent way at 15 m sampling.
- **Alpine subset**: `hw == "path"` and `elev_min > 1600 m`.

## Verification log (this port, 2026-10-14)

| Check | Result |
|---|---|
| `stats2.py --profiles-dir data` vs `expected/stats2_out.json` | **identical** (`compare_stats.py`: all fields within tolerance; max rel diff 0) |
| `bandfrac`/`descfrac`/`dense_check`/`noise_check` on `data/` | all values match the audit findings tables |
| `scree_collect.py` + `scree_analyze.py` on `austria-261013.osm.pbf` | object counts identical to the audit run (24,863 scree ways; 2,025 relations; 10,380 member nodelists; 1,240,171 trails; 16,836,545 coords; 41 `surface=scree` ways; identical key sets) → 26,974 polygons, **15,211 crossings**, path median 33.7 m — identical |
| `scree_collect.py` + `scree_analyze.py` on `switzerland-latest.osm.pbf` | 18,410 scree ways; 875 relations; 4,956 member nodelists; 1,124,892 trails; 11,918,261 coords → 19,428 polygons, **11,565 crossings**, path median 64.4 m — identical |
| `scree_stats.py` on pinned slim data | AT 33.7 m / CH 64.4 m path medians and all F1–F4 numbers match the findings |
| Known deviations | CH path+footway total: findings text 68,511 km vs surviving artifact 78,039 km (−12 % on the denominator; event counts identical) — documented in the reference doc §8 |

## Limitations (short form — full text in the reference doc)

- OSM `sac_scale`/`mtb:scale` are volunteer tags; coverage is selective and
  biased — classes are *tag populations*, not random samples.
- Median OSM node spacing ≈ 9.6 m: sub-10 m geometry is under-resolved;
  switchbacks are undercounted (dense-subset check: 8.0 vs 3.7 per km).
- The profile follows the *line*, not the tread: where the line cuts a
  switchback corner the slope under the line is steeper than the tread.
- 2 m DEM (actually used) + median filter smooth short features; a paired
  0.5 m check showed ≤ ~1 pp change in the >25 % share.
- `natural=scree` polygons are geomorphological map units, not loose-tread
  measurements: crossing length overestimates physically loose tread.
- Terrarium z13 (~20 m/px) is much coarser than swissALTI3D; crossing-grade
  and TSA numbers inherit that resolution.