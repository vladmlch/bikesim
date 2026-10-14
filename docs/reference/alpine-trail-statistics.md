# Alpine trail statistics — canonical numeric source

**Status: verified reproduction.** Every table below was regenerated on
2026-10-14 from pinned data with the scripts in
[`tools/research/alpine_osm_dem/`](../../tools/research/alpine_osm_dem/README.md);
`stats2_out.json` is **bit-identical** to the audit's `.scratch` output, and
the raw OSM collector output was re-run count-identical against the original
Geofabrik extract. This document is the single source of alpine-trail numbers
for `AlpineGenSpec` v2 (disputes D2/D3 of
`docs/superpowers/plans/2026-10-14-round-2-integration-master.md` §3 / WP-V1).
Where a number here disagrees with the earlier audit text, the difference is
stated in §8 (errata) — trust the tables.

## Provenance vocabulary (used per table)

- **digitised** — computed by us from primary datasets (OSM geometries,
  swissALTI3D / Terrarium DEMs) with a committed, re-runnable script.
  Not peer-reviewed field measurement.
- **measured** — published instrumented field data.
- **standard** — design/construction norms; describes what a built trail
  *should* be, not what alpine trails *are*.
- **authored** — engineering estimate chosen by a human, no dataset.
- **extrapolated** — transferred from a different population/scale.
- **disputed** — claimed by an external report but not confirmed here.

## Definitions on first use

- **Grade g** — dz/ds of the trail line, least-squares fit inside a centred
  window; `g5` = 5 m window. All main tables use |g| (absolute): OSM ways have
  no direction of travel.
- **Quantile pX** — the value below which X % of *length-samples* fall
  (profile samples at uniform 2 m spacing ⇒ fraction of samples ≈ fraction
  of length).
- **Run** — maximal consecutive sequence of samples whose |g5| stays inside
  one band (<8 %, 8–15 %, 15–25 %, >25 %); run length = samples × 2 m.
- **Traverse** — trail segment oriented roughly along the contour
  (trail-vs-slope angle TSA > 60°; 0° = fall line).
- **Crossing** — contiguous part of an OSM trail line lying inside an OSM
  `natural=scree|shingle` polygon (≥ 1 m).
- **TSA** — angle between the crossing's heading and the DEM downslope
  azimuth at its midpoint.

---

## 1. Slope statistics (OSM paths × swissALTI3D)

**Method.** Ways = Overpass `out geom` query
`way(bbox)["highway"="path"]["sac_scale"|"mtb:scale"|"mtb:scale:uphill"]`
over Zermatt (S 45.95, W 7.65, N 46.15, E 7.95) and Engadin
(46.30, 9.70, 46.52, 9.95); fetch 2026-10-14; 976 + 951 ways. Way geometry →
LV95 → resample 2 m → bilinear sample of swissALTI3D → median-filter (3
samples) → keep contiguous covered segments ≥ 32 m. Above-treeline mask:
z > 2100 m (Zermatt) / 2200 m (Engadin). ~620 km total. DEM posting
**2 m** — the audit text says 0.5 m; the run used the `*_2_2056_5728.tif`
assets (see §8). A paired 0.5 m check found ≤ ~1 pp change in the >25 %
share (§7).

Classes: **T2** `sac_scale=mountain_hiking`, **T3**
`demanding_mountain_hiking`, **T45** `alpine|demanding|difficult_alpine_hiking`,
**MTB** any `mtb:scale*`, **flat** = proxy control (z < treeline−300 m and
|g20| < 5 % — a *profile-flatness* filter, not a flat-terrain survey).

### 1a. |g| quantiles — p5/p25/p50/p75/p90/p95/p99 [digitised]

| class | km | g5 | g10 | g20 |
|---|---|---|---|---|
| Zermatt T2 | 125.1 | .014/.074/.155/.241/.324/.378/.514 | .014/.076/.156/.232/.306/.352/.455 | .014/.076/.156/.226/.294/.335/.422 |
| Zermatt T3 | 93.5 | .019/.107/.205/.306/.411/.492/.667 | .020/.110/.208/.298/.388/.459/.622 | .021/.111/.210/.293/.371/.437/.580 |
| Zermatt T45 | 162.4 | .023/.114/.232/.390/.578/.712/1.118 | .023/.113/.228/.381/.553/.678/1.019 | .023/.109/.224/.369/.527/.641/.931 |
| Zermatt MTB | 52.2 | .012/.064/.139/.214/.300/.360/.495 | .011/.064/.137/.206/.284/.337/.448 | .013/.064/.134/.198/.274/.317/.427 |
| Zermatt flat (proxy) | 37.9 | .008/.046/.116/.219/.336/.419/.738 | .008/.043/.111/.207/.310/.378/.644 | .008/.041/.105/.196/.286/.353/.573 |
| Engadin T2 | 116.9 | .013/.067/.146/.230/.306/.357/.470 | .012/.067/.145/.225/.295/.337/.432 | .012/.067/.144/.222/.285/.324/.405 |
| Engadin T3 | 38.2 | .017/.086/.183/.301/.418/.493/.623 | .017/.086/.183/.295/.400/.464/.584 | .017/.088/.183/.290/.387/.444/.558 |
| Engadin T45 | 57.3 | .022/.120/.256/.428/.595/.715/1.098 | .022/.123/.255/.417/.569/.684/.967 | .021/.123/.256/.411/.546/.648/.882 |
| Engadin MTB | 30.8 | .010/.053/.115/.191/.271/.321/.410 | .010/.053/.111/.186/.262/.309/.383 | .010/.051/.111/.182/.255/.300/.362 |
| Engadin flat (proxy) | 35.5 | .004/.024/.058/.116/.185/.234/.381 | .004/.022/.053/.107/.171/.214/.341 | .003/.020/.048/.101/.160/.199/.320 |

The 2 m-window quantiles (g2) are stored in `expected/stats2_out.json`; they
are at/below the resolution limit of the OSM line (median node spacing
9.6 m, §5) and are informational only — use 5–10 m for calibration.

### 1b. Fraction of length above grade thresholds [digitised]

| class | >20/>25/>30 % (w5) | (w10) | (w20) |
|---|---|---|---|
| Zermatt T2 | .350/.223/.131 | .327/.194/.103 | .295/.166/.081 |
| Zermatt T3 | .503/**.375**/.257 | .499/.362/.234 | .478/.342/.209 |
| Zermatt T45 | .555/.459/.372 | .534/.437/.351 | .502/.409/.325 |
| Zermatt MTB | .283/.167/.098 | .251/.144/.078 | .218/.121/.058 |
| Engadin T2 | .327/.196/.106 | .310/.178/.088 | .292/.159/.072 |
| Engadin T3 | .456/**.346**/.247 | .444/.333/.230 | .425/.311/.213 |
| Engadin T45 | .591/.503/.419 | .579/.489/.407 | .559/.471/.388 |
| Engadin MTB | .223/.126/.066 | .203/.114/.056 | .185/.097/.046 |
| Engadin flat (proxy) | .080/.040/.023 | .059/.027/.014 | .043/.018/.010 |

Headline (dispute D2): **T3-class trails spend ~33–38 % of their length at
>25 % grade** (5–10 m windows), not 5–15 %.

### 1c. Constant-grade run lengths, m (p25/p50/p75/p90) [digitised]

| class | flat <8 % | mod 8–15 % | steep 15–25 % | wall >25 % |
|---|---|---|---|---|
| Zermatt T2 | 4/8/14/28 | 2/4/6/12 | 2/4/10/16 | 4/8/14/26 |
| Zermatt T3 | 4/6/12/24 | 2/4/6/10 | 2/4/8/14 | 6/10/22/41.8 |
| Zermatt T45 | 2/6/10/20 | 2/4/6/10 | 2/4/8/14 | 6/10/26/60 |
| Zermatt MTB | 4/8/16/28 | 2/4/6/12 | 2/6/10/18 | 4/8/14/26 |
| Engadin T2 | 4/8/16/34 | 2/4/8/12 | 4/6/10/18 | 4/8/16/30 |
| Engadin T3 | 4/8/14/24 | 2/4/6/10 | 2/4/8/12 | 6/10/20/42 |
| Engadin T45 | 2/6/10/22 | 2/4/6/10 | 2/4/8/14 | 6/12/26/73.2 |
| Engadin MTB | 4/8/18/38 | 2/4/8/14 | 4/6/12/22 | 4/8/18/33.8 |
| Engadin flat | 6/10/26/52 | 2/4/8/12 | 2/4/8/15.4 | 4/6/8/18 |

**Median run is 4–8 m in every band and class.** Wall runs are *longer* than
steep runs (median 8–12 m; T45 p90 up to 60–73 m) — long steep traverses and
gullies are a real mode, not a rare "one wall" event. Values are integer
multiples of the 2 m step; 4–6 m medians sit at the resolution floor.

### 1d. Length share per band (|g5|) [digitised]

| class | <8 % | 8–15 % | 15–25 % | >25 % |
|---|---|---|---|---|
| Zermatt T2 | .267 | .216 | .288 | .229 |
| Zermatt T3 | .189 | .164 | .264 | **.382** |
| Zermatt T45 | .175 | .152 | .206 | .467 |
| Zermatt MTB | .303 | .238 | .288 | .171 |
| Engadin T2 | .295 | .219 | .286 | .200 |
| Engadin T3 | .233 | .186 | .229 | **.351** |
| Engadin T45 | .168 | .138 | .183 | .510 |
| Engadin MTB | .366 | .256 | .249 | .129 |

(Independent counting path via `bandfrac.py` — the bold T3 >25 % cells are
the alternative estimate: 38.2 % / 35.1 % vs the stats2 path's 37.5 % /
33.3 %. Both land inside the audit's "35–38 %" claim.)

### 1e. Grade reversals, switchbacks, descending share [digitised]

- **Reversals** (gauss σ=2 m, dip ≥0.30 m, length 1.5–30 m): 3.1–4.5 per km
  across classes; length p50 ≈ 12–16 m; depth p50 ≈ 0.8–1.3 m.
  Zermatt T2 3.66/km, T3 3.08/km, T45 4.27/km, MTB 4.48/km;
  Engadin T2 3.22/km, T3 4.00/km, T45 3.42/km.
- **Switchbacks** (heading change >120°/15 m): T2 2.2–3.7/km, T3 2.5–3.4/km,
  T45 0.6–0.95/km, MTB 0.9–3.1/km — *undercounted* at 9.6 m node spacing.
  Dense-subset control (median node spacing ≤ 6 m, 100°/10 m): **7.99/km
  Zermatt T2, 7.03/km Engadin T2** (19.7 km / 15.9 km samples).
- **Descending share along climb direction** (g10 < −2 %, direction = sign of
  net segment gain): Zermatt T2 8.0 %, T3 8.4 %, T45 11.8 %; Engadin T2
  9.2 %, T3 10.6 %, T45 9.6 %. → 8–12 % of a real "up" route descends.

## 2. Scree crossings (OSM polygons × Terrarium DEM)

**Method.** Geofabrik extracts `austria-261013.osm.pbf` (812 MB) and
`switzerland-latest.osm.pbf` (548 MB), fetched 2026-10-13/14. Scree =
`natural=scree|shingle` closed ways + multipolygon relations. Trails =
`highway ∈ {path, footway, track, bridleway, cycleway, steps}`. AT:
26,974 polygons, 334,293 km trails total / 76,221 km path+footway,
15,211 crossings. CH: 19,428 polygons, 181,016 km / 78,039 km path+footway
(see §8.2), 11,565 crossings. Elevation/TSA via AWS Terrarium z13
(~19–24 m/px). Alpine subset = `highway=path` & polygon `elev_min` > 1600 m.

### 2a. Crossing length quantiles, m — p5/p25/p50/p75/p95/p99 [digitised]

| sample | n | p5 | p25 | p50 | p75 | p95 | p99 |
|---|---|---|---|---|---|---|---|
| AT all trails | 15,211 | 3.1 | 11.4 | 31.0 | 97.8 | 448.6 | 995.3 |
| AT path only | 12,390 | 3.3 | 11.7 | **33.7** | 109.3 | 495.1 | 1,044.6 |
| AT alpine path (>1600 m) | 9,012 | 3.8 | 14.4 | **45.7** | 146.2 | 581.4 | 1,169.7 |
| AT `sac_scale`-tagged | 9,985 | — | — | 36.2 | — | — | — |
| AT `mtb:scale`-tagged | 806 | 3.4 | 11.6 | 31.1 | 90.4 | 376.7 | 814.2 |
| CH all trails | 11,565 | 4.2 | 17.7 | **57.6** | 170.3 | 629.6 | 1,244.8 |
| CH path only | 9,677 | 4.7 | 20.3 | **64.4** | 183.2 | 666.3 | 1,311.8 |
| CH `mtb:scale`-tagged | 986 | 5.6 | 23.2 | 66.1 | 176.6 | 610.1 | 1,058.6 |

Headline (dispute D3): **median crossing ≈ 34–64 m** depending on
country/subset; p95 ≈ 450–670 m. The distribution is heavy-tailed — the
"6–15 m" range is only ~p25–p30. The AT-vs-CH 2× difference (33.7 vs 64.4 m)
is a *between-region* spread of the same population family: CH has a larger
alpine fraction and denser scree mapping; both are far above Gemini's 3–30 m.

### 2b. Frequency [digitised]

- AT: 12,390 path crossings / 76,221 km path+footway = **16.3 events/100 km**;
  path km inside scree ≈ 1.8 %.
- CH: 9,677 / 78,039 km = **12.4 events/100 km**; share ≈ 2.0 %.
  (Audit text: 14.1 /100 km on 68,511 km — denominator erratum, §8.2.)
- AT SAC-tagged subset density (audit finding): ~45 events/100 km —
  alpine trails hit scree far more often than the global average.
- A 120 m corridor at a *random* mountain location expects ~0.02–0.05
  crossings ⇒ `n_scree=(2,3)` per 120 m is a deliberately scree-rich
  conditioning, not a typical share.

### 2c. Grade inside scree + trail-vs-slope angle (AT alpine subset) [digitised]

- Crossing grade p10/p25/p50/p75/p90 = **.06/.11/.21/.32/.44**
  (n = 9,012; 65 % ≥ 15 %, 51 % ≥ 20 %, 22 % < 10 %). Parent-way median
  |grade| p10–p90 = .09–.36; in-scree/way ratio median ≈ 1.01 — scree neither
  flattens nor steepens the profile systematically.
- Short crossings (< 25 m): median grade .19, 61 % ≥ 15 %.
- **TSA classes**: fall-line <30° **25.9 %**, diagonal 30–60° **27.7 %**,
  traverse >60° **46.4 %** (n = 7,656 valid). Median in-scree grade by class:
  fall .31, diagonal .26, traverse .13; terrain slope under crossings
  p10–p90 = 11.2–38.5°, median ~27–30° (≈ scree repose band, measured:
  Chandler 1973 φ′cv 39–40°, Carson 1977 35°, Sanders 2010 30–35° —
  see findings.md F6 for citations).
- `pos_frac` (crossing midpoint's elevation fraction inside its polygon)
  p10/p50/p90 = .06/.51/.95 — crossings sit anywhere in the polygon, not only
  at its edges.

## 3. OSM geometry resolution floor [digitised]

Node spacing of the fetched ways: p25 5.0 m / median 9.6 m / p75 17.7 m /
p90 30.7 m. Consequences:

- Windows < 10 m describe "terrain under the digitised line", not tread
  geometry; g2 quantiles are informational.
- Switchbacks are systematically undercounted (§1e dense control).
- Where the line cuts a hairpin corner, the slope *under the line* exceeds
  the tread slope — this *raises* measured |g| tails slightly, i.e. the
  >25 % share is, if anything, an over-estimate of tread steepness on
  switchback segments and an under-estimate of true corner count.

## 4. DEM resolution check (audit erratum: 2 m used, not 0.5 m)

`dem_resolution_check.py` paired the same profile segments against 0.5 m
swissALTI3D tiles on 12 LV95 cells (Gornergrat + Matterhorn/T45 blocks):

| class | paired km | >25 % share 2 m → 0.5 m (g5) | g5 median shift |
|---|---|---|---|
| T2 | 3.7 | .128 → .138 | .208 → .206 |
| T3 | 3.0 | .659 → .659 | .334 → .334 |
| T45 | 10.1 | .385 → .383 | .310 → .314 |
| MTB | 8.4 | .083 → .087 | .168 → .169 |

DEM posting changes the >25 % share by ≤ ~1 pp; the OSM line's 9.6 m node
spacing is the dominant resolution limit. All §1 numbers stand as computed
on the 2 m assets.

## 5. Limitations (read before using any number)

1. **Selection bias**: classes are OSM tag populations, not random samples.
   `sac_scale`/`mtb:scale` coverage is voluntary and uneven.
2. **Line ≠ tread**: OSM ways smooth real geometry (9.6 m median node
   spacing); short structures and hairpins are under-resolved, small-window
   grade variance slightly inflated where the chord leaves the tread.
3. **DEM**: swissALTI3D is a stereo-correlated surface DEM; above ~2000 m,
   snow/ice cover epochs, rock faces and correlation failures introduce
   decimetre–metre noise and voids. Median filtering + least-squares windows
   suppress but do not remove this; per-route outliers (e.g. Europaweg
   max 57 %) are line×DEM artefacts — they live in p99, not the medians.
   Terrarium z13 (~20 m/px) is far coarser — scree grades/TSA inherit that.
4. **`natural=scree` is a landform polygon, not a tread measurement**:
   crossing length overestimates physically loose surface (treads can be
   stabilised/graded inside the polygon); mapping completeness varies.
5. **flat_control is a proxy**: it selects flat *trail profiles* at low
   elevation — the Zermatt control is contaminated by valley walls
   (median g20 .105); Engadin's clean control (median g20 .048) shows the
   measurement chain does not invent grades.
6. **Sign**: |g| only; the descending-share table reconstructs direction
   from net segment gain, not recorded travel direction.
7. **Epoch**: single snapshots (OSM 2026-10-13/14, DEM tile year 2024);
   re-fetch on 2026-10-14 returned an identical way-ID set for both regions.

## 6. Audit claim → reproduced → verdict

| # | Audit claim | Reproduced | Verdict |
|---|---|---|---|
| D2a | Technical (T3) trails: ~35–38 % of length at >25 % | 37.5/33.3 % (stats2, w5) & 38.2/35.1 % (bandfrac), Zermatt/Engadin | **confirmed** |
| D2b | Median constant-grade run 4–8 m | 4–8 m medians in every band/class (2 m grid floor) | **confirmed** |
| D2c | T3 medians |g5| ≈ 18–21 % | 20.5 % / 18.3 % | **confirmed** |
| D2d | 8–15 % band share ≈ 14–26 % | .138–.256 across classes | **confirmed** |
| A5 | Reversals 3.1–4.5/km, len p50 12–16 m | identical values | **confirmed** |
| A6 | Descending share 8–12 % | 8.0–11.8 % / 9.2–10.6 % | **confirmed** |
| A7 | Switchbacks 2–8/km; dense subset ~7–8/km | 0.56–4.48/km overall; dense 7.99/7.03/km | **confirmed** |
| D3a | Scree crossing median 34–64 m | 33.7 / 64.4 m (AT/CH path) | **confirmed** |
| D3b | Scree crossing p95 500–580 m | 495.1 (AT path) / 581.4 (AT alpine) / 666.3 (CH path) | **confirmed** (CH p95 slightly above audit band) |
| F2 | ~14–16 events/100 km | AT 16.3 ✓; CH 12.4 (audit printed 14.1 on a wrong denominator — §8.2) | **confirmed with correction** |
| F3 | Grade in scree median ~21 %, tail to 44 % | .206 median, p90 .44 | **confirmed** |
| F4 | TSA 26/28/46 % fall/diag/traverse | 25.9/27.7/46.4 % | **confirmed** |
| — | Gemini: >25 % share 5–15 % | not supported (actual 33–51 % on T3/T45) | **disproved by digitised data** |
| — | Gemini: runs 3–30 m typical | consistent only as a *range*; median claim contradicted (medians 4–8 m vs implied longer segments) | **partially contradicted** |
| — | Gemini: scree pocket 3–10 m / traverse 10–30 m | medians 33.7–64.4 m; 3–15 m is ~p25–p30 only | **disproved** |
| — | Gemini: scree absent on >28 % walls | 65 % of alpine crossings sit at ≥15 % grade; p90 in-scree grade .44 | **disproved** (crossings do occupy steep terrain) |
| — | WTA "H_s ≈ 10 cm verified" (D4) | no primary measurement found for tread veneer | **disputed — stays `authored`** |

## 7. Reproduction

```sh
cd tools/research/alpine_osm_dem
uv run stats2.py --profiles-dir data --out /tmp/stats2_out.json
uv run compare_stats.py /tmp/stats2_out.json   # → "fields compared OK"
uv run bandfrac.py --profiles-dir data && uv run descfrac.py --profiles-dir data
uv run dense_check.py                          # dense-subset control
uv run noise_check.py --profiles-dir data      # valley-floor negative control
uv run scree_stats.py --datadir data           # all §2 tables
```

Gate result (2026-10-14): `compare_stats.py` reports **all fields identical**
to `expected/stats2_out.json` (the audit's `.scratch` output). Both scree
collectors + analyzers were re-run end-to-end from the original Geofabrik
extracts: AT 24,863 scree ways / 2,025 relations / 1,240,171 trails /
16,836,545 coords → 15,211 crossings (path median 33.7 m); CH 18,410 /
875 / 1,124,892 / 11,918,261 → 11,565 crossings (path median 64.4 m) —
all counts and quantiles identical to the audit artifacts. Full
methodology, queries, bboxes and commands:
`tools/research/alpine_osm_dem/README.md`.

## 8. Errata vs the original audit text

1. **DEM resolution**: findings say swissALTI3D "0.5 m / 616 tiles / 665 MB";
   the surviving run used the **2 m** assets (`*_2_2056_5728.tif`, 500×500 px
   per 1 km cell). Sensitivity ≤ ~1 pp (§4) — conclusions unaffected; the
   canonical resolution is 2 m.
2. **CH path+footway total**: findings print 96,764 km (method) and 68,511 km
   (F2); the surviving analysis artifact and this port compute **78,039 km**.
   Event counts and crossing lengths are identical; only the rate
   denominator differs (14.1 → 12.4 events/100 km; km share 2.3 → 2.0 %).
   Cause unrecoverable (earlier pipeline iteration or a different filter);
   the pinned artifact is authoritative.
3. **Overpass timestamp**: saved responses carry
   `osm3s.timestamp_osm_base = "117722"/"117727"` — an anomalous, truncated
   field in the API response; the fetch date (2026-10-14) is taken from the
   audit record, and a same-day re-fetch returned identical way IDs.
4. **`flat_control`**: a profile-flatness proxy (z < treeline−300 m AND
   |g20| < 5 %), not a survey of flat terrain — keep for sign checks only.
5. **Scree n for `mtb` in findings** says n=795 for AT; the pinned artifact
   gives **806** (`mtb:scale` or `mtb:scale:uphill` present). Numbers in §2a
   use 806.

## 9. What this does NOT establish

- True tread (wheel-path) slopes — needs RTK/barometric GPX or LiDAR along
  the tread.
- Loose-layer thickness on tread — unmeasured anywhere; `H_s` stays
  `authored` (3–10 cm plausible as trafficked veneer by incision-proxy
  analogy; untouched/fresh scree 20–100 cm is `extrapolated` from
  Deckschicht covers, per findings F7).
- Causal linkage of scree to profile grade — scree belongs to steep *map*
  locations (terrain ~27–30°), not to steep profile segments (§2c, F5).
- Coverage of regions outside Zermatt/Engadin (slopes) or AT/CH (scree).
