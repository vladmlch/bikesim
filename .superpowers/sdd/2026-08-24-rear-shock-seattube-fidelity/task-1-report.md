# Task 1 Report: Reference Extraction Tool

## Status: DONE

## What was implemented

Followed the brief's steps verbatim, with Ruling R1 applied to `calibrate()`.

- `docs/reference/bulls_sonic_evo_side.jpg` — copy of the source photo
  (`/Users/vladislav.molchanov/Desktop/side_001_bike-detail-2x copy 4.jpg`)
- `tools/__init__.py` — empty package marker
- `tools/photo_reference.py` — red-cluster segmentation (flood fill), pixel→mm
  calibration, and named-pivot payload builder, exactly as specified in the
  brief, except for `calibrate()`
- `docs/reference/bulls_reference_points.json` — generated output, committed
- `tests/test_photo_reference.py` — the brief's test file, verbatim

### Ruling R1 applied

`calibrate()` now excludes any cluster within `AXLE_EXCLUSION_RADIUS_PX = 150.0`
px of either axle cluster before picking `max(interior, key=px_y)` as the BB:

```python
def near_axle(c: Dict[str, Any]) -> bool:
    for axle in (rear, front):
        if np.hypot(c["px_x"] - axle["px_x"], c["px_y"] - axle["px_y"]) < AXLE_EXCLUSION_RADIUS_PX:
            return True
    return False

interior = [
    c for c in clusters
    if rear["px_x"] < c["px_x"] < front["px_x"] and not near_axle(c)
]
bb = max(interior, key=lambda c: c["px_y"])
```

A comment above the constant explains why it exists (hub hardware —
derailleur-hanger and cassette bolts — sits low in the image near the rear
axle and would otherwise win the naive "lowest interior cluster" rule).

Verified empirically: the derailleur-hanger bolt cluster is at px
`(340.16, 738.42)`, the true BB is at px `(587.38, 733.33)` — the derailleur
bolt is indeed 5 px lower (higher px_y) than the BB. Its distance to the rear
axle cluster (px `(298.06, 728.23)`) is `hypot(42.1, 10.19) = 43.3 px`, well
inside the 150 px exclusion radius, so it is correctly dropped. No other
cluster falls inside the exclusion radius, so R1 affects exactly the one
cluster it was designed to affect. Everything else in the brief's code
(detection, `Calibration`, `build_reference`, `main`) stands as written.

## TDD evidence

**RED** — `uv run --with pillow pytest tests/test_photo_reference.py -v`
(run before `tools/photo_reference.py` existed):

```
ERROR collecting tests/test_photo_reference.py
tests/test_photo_reference.py:8: in <module>
    from tools.photo_reference import (
E   ModuleNotFoundError: No module named 'tools.photo_reference'
```
Expected failure — the module did not exist yet.

**GREEN** — after implementing `tools/photo_reference.py` and generating the
JSON with `uv run --with pillow python -m tools.photo_reference`:

```
uv run --with pillow pytest tests/test_photo_reference.py -v
tests/test_photo_reference.py::test_detects_the_ten_pivot_clusters PASSED
tests/test_photo_reference.py::test_calibration_reproduces_bb_drop PASSED
tests/test_photo_reference.py::test_reference_payload_contains_every_named_pivot PASSED
tests/test_photo_reference.py::test_reference_json_is_committed_and_matches_the_tool PASSED
4 passed in 0.47s
```

Full suite before commit: `uv run --with pillow pytest -q` → `50 passed in 6.14s`
(46 pre-existing + 4 new).

## Detected clusters and millimetre coordinates

Calibration: `origin_px = (587.375, 733.325)`, `mm_per_px = 1.618557642107814`
(brief expects 1.6193 ± 0.005 — matches). BB-drop self-check:
`front_axle Z = 22.2147 mm` (expected 22.5 ± 1.0 — matches).

All ten detected clusters, sorted by descending area, with pixel centroid and
calibrated (X, Z) mm:

| area | px_x | px_y | X (mm) | Z (mm) | identity |
|---|---|---|---|---|---|
| 145 | 559.59 | 705.02 | -44.98 | 45.81 | P0_candidate |
| 133 | 340.16 | 738.42 | -400.14 | -8.25 | P2_candidate (derailleur-hanger bolt) |
| 128 | 563.11 | 615.01 | -39.28 | 191.50 | P4 |
| 122 | 613.30 | 619.24 | 41.95 | 184.66 | P5 |
| 95 | 693.22 | 485.31 | 171.32 | 401.43 | P7 |
| 87 | 543.39 | 605.99 | -71.19 | 206.10 | P3 |
| 87 | 298.06 | 728.23 | -468.28 | 8.25 | rear_axle |
| 51 | 605.75 | 571.86 | 29.73 | 261.34 | P6 |
| 45 | 1089.18 | 719.60 | 812.20 | 22.21 | front_axle |
| 40 | 587.38 | 733.33 | 0.00 | 0.00 | bb (origin) |

All named points fall within a few mm of the brief's expected ground truth
(P3, P4, P5, P6, P7, rear_axle, front_axle) — largest deviation is well under
2 mm (e.g. P3 X: -71.19 vs expected -71.3; P7 Z: 401.43 vs expected 401.7).

The full generated payload is committed at
`docs/reference/bulls_reference_points.json`.

## Files changed

- `docs/reference/bulls_sonic_evo_side.jpg` (new, binary copy of source photo)
- `docs/reference/bulls_reference_points.json` (new, generated)
- `tools/__init__.py` (new, empty)
- `tools/photo_reference.py` (new, 211 lines)
- `tests/test_photo_reference.py` (new, 57 lines, verbatim from brief)

`pyproject.toml` and `uv.lock` are untouched — Pillow was not added as a
project dependency, per the constraint; all runs used
`uv run --with pillow ...`.

## Self-review findings

- Ruling R1 is implemented as a single, named, commented constant and a small
  helper predicate; it changes only `calibrate()`, nothing else in the brief's
  code was touched.
- Verified by hand that exactly one cluster (the derailleur-hanger bolt) is
  affected by the new exclusion — no unintended side effects on other
  clusters (checked distances of all ten clusters to both axles).
- Tests exercise real image data end-to-end (no mocking of detection or
  calibration), so they verify actual behavior, not just interface shape.
- No YAGNI concerns — implementation is exactly the brief's scope plus the
  minimal R1 fix.
- Confirmed `pyproject.toml`/committed dependency set unchanged; only the five
  intended files were staged and committed (checked `git status` before
  `git add`, since `.DS_Store`, `.claude/`, `MUJOCO_LOG.TXT`, and `uv.lock`
  were pre-existing untracked files unrelated to this task and were correctly
  left out of the commit).

## Concerns

None. Numbers reproduce the brief's expected ground truth well within
tolerance, all four tests pass, full suite (50 tests) passes.
