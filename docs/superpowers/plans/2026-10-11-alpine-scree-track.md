# Alpine Scree Track Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A seeded generator for a 120 m alpine scree-trail climb (`AlpineGenSpec` → `TrackSpec`), a `scree` surface, geometric single-contact guards for the `compliant_2d` tyre, a pinned `alpine_scree` preset for `bike-ride`, an `alpine` scenario for `bike-research`, and frozen example files.

**Architecture:** Pure-geometry Python inside `src/bike_sim/terrain/`: shared placement helpers are extracted from `generator.py`/`road.py` into `placement.py` (byte-identical outputs), closed-form tyre guards live in `contact_safety.py`, and `alpine.py` draws grade rhythm, surfaces and obstacles from one `numpy.random.Generator` in a fixed order. The output is an ordinary `TrackSpec`, so heightfield, TOML dump/load, HUD and both backends need no changes. CLI wiring adds a seeded-preset registry and one research scenario.

**Tech Stack:** Python 3.12, numpy, dataclasses, tomllib, pytest; `uv` for every command (`uv run --frozen …`).

**Spec:** `docs/superpowers/specs/2026-10-11-alpine-scree-track-design.md`

## Global Constraints

- Branch: `feat/alpine-scree-track` (already created from `impl/cpp-port-p2` HEAD `d52ed38`).
- Python only: no changes under `native/`, `src/bike_sim/native/`, `docs/RIDE.md`.
- Verification profile: `bash tools/run_tests.sh quick` (see `AGENTS.md`). Run single tests with `uv run --frozen python -m pytest tests/reference/<file>.py -q`.
- Always use `uv run --frozen python …`, never bare `python`.
- Units: metres and dimensionless grade (dz/dx) everywhere in `terrain/`; `.15` means 15 %.
- Positions rounded to 1 cm (`rounded(x, 2)`), sizes to 0.1 mm (`rounded(x)` = 4 digits) so TOML round-trips are exact.
- `generate_track` (old generator) and `road_*` presets must stay byte-identical (Task 1 pins their hashes).
- Code style: match neighbours — module docstring first, compact code, no new comments beyond what the task shows, no emojis.
- Do not add dependencies.
- Commit after every task with the project's commit trailer:

```
Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
```

---

## File Structure

| File | Responsibility |
|---|---|
| Create `src/bike_sim/terrain/placement.py` | Seeded interval placement helpers shared by all procedural generators (`rounded`, `check_range`, `is_free`, `place`, `free_intervals`, `CLEARANCE_M`, `PLACEMENT_TRIES`) |
| Modify `src/bike_sim/terrain/generator.py` | Import helpers from `placement.py` instead of defining them |
| Modify `src/bike_sim/terrain/road.py` | Import `free_intervals` from `placement.py` |
| Modify `src/bike_sim/terrain/surface.py` | Add the `scree` surface |
| Create `src/bike_sim/terrain/contact_safety.py` | Closed-form single-contact guards for the 2D tyre |
| Create `src/bike_sim/terrain/alpine.py` | `AlpineGenSpec`, `generate_alpine_track`, `alpine_scree`, `ALPINE_PRESET_SEED` |
| Modify `src/bike_sim/terrain/presets.py` | Register `alpine_scree`; `SEEDED_PRESETS` |
| Modify `src/bike_sim/terrain/__init__.py` | Exports |
| Modify `src/bike_sim/cli/ride.py` | Seeded presets in `resolve_track`, `track_seed`, `_list_tracks` |
| Modify `src/bike_sim/cli/research.py` | `--scenario alpine` |
| Create `examples/research/alpine_scree.toml` | Frozen dump of the preset |
| Create `examples/research/alpine_rocky_genspec.toml` | Multi-contact-ready generator spec (no preset) |
| Create `tests/reference/test_terrain_placement_regression.py` | T6: old generators byte-identical |
| Create `tests/reference/test_contact_safety.py` | Guard formulas on spike points |
| Create `tests/reference/test_alpine_track.py` | T1–T5 |

---

### Task 1: Pin the old generators, then extract `placement.py`

**Files:**
- Create: `tests/reference/test_terrain_placement_regression.py`
- Create: `src/bike_sim/terrain/placement.py`
- Modify: `src/bike_sim/terrain/generator.py:23-45` and `:115-138`
- Modify: `src/bike_sim/terrain/road.py:127-139`

**Interfaces:**
- Produces: `placement.rounded(value, digits=4) -> float`, `placement.check_range(value, name, *, minimum=0., integer=False) -> tuple`, `placement.is_free(start, end, taken) -> bool`, `placement.place(rng, length, lo, hi, taken, *, shrink=False) -> tuple[float, float] | None`, `placement.free_intervals(span, occupied, gap_m) -> list[tuple[float, float]]`, constants `CLEARANCE_M = 1.`, `PLACEMENT_TRIES = 200`.

- [ ] **Step 1: Write the regression test with the hashes captured before any refactor**

These digests were computed on `d52ed38` with `hashlib.sha256(dump_track(track).encode()).hexdigest()`.

```python
# tests/reference/test_terrain_placement_regression.py
"""The procedural generators must stay byte-identical across the placement refactor."""
import hashlib

import pytest

from bike_sim.terrain.generator import TerrainGenSpec, generate_track
from bike_sim.terrain.presets import get_preset
from bike_sim.terrain.trackfile import dump_track

GENERATED = {
    0: 'b298d8170a8391daf50b671f41b9dc7e33855354fe770d3cb894af70400f544d',
    1: '7ac544f21b89f27a03a176e5d017e0a0e175af8ec80119601dd05a652133848a',
    2: '4837293048585ed234201595833d3e8ed7851b6c6e8a384ab41745840c7f5622',
    3: 'be8c67f11a7961b2bcb012612b6d16ed0b72394edcae09f9b12fc0e1b73857c0',
}
ROADS = {
    'road_smooth': 'd05d8a6b82f179bdb80f66ee5cfa71bdb8c74237135456f673e74dfc74eafb1d',
    'road_worn': '455af2ec2c2665629898e2c05b7ae29cf0b2f766a4cf2d9a1e0e77fd0017288c',
    'road_broken': 'f6eb8883c2f83196328a06cb7eaf2d7eb1736f9b5de49206dc25dffcc1aa029e',
}
SPEC_SHA = '45d11d386fb651dff55d7af3292ea3e2a972d9354e1bef677227f2399bfcf8e7'


def digest(track):
    return hashlib.sha256(dump_track(track).encode()).hexdigest()


def test_default_terrain_spec_hash_is_pinned():
    assert TerrainGenSpec().sha256() == SPEC_SHA


@pytest.mark.parametrize('seed', sorted(GENERATED))
def test_generated_tracks_are_byte_identical(seed):
    assert digest(generate_track(TerrainGenSpec(), seed=seed)) == GENERATED[seed]


@pytest.mark.parametrize('name', sorted(ROADS))
def test_road_presets_are_byte_identical(name):
    assert digest(get_preset(name)) == ROADS[name]
```

- [ ] **Step 2: Run it — it must pass on the untouched code**

Run: `uv run --frozen python -m pytest tests/reference/test_terrain_placement_regression.py -q`
Expected: 8 passed. (If any hash differs, stop: the working tree is not at the pinned baseline.)

- [ ] **Step 3: Create `placement.py`**

```python
# src/bike_sim/terrain/placement.py
"""
Seeded Interval Placement.

Helpers shared by the procedural track generators (:mod:`bike_sim.terrain.generator`,
:mod:`bike_sim.terrain.road`, :mod:`bike_sim.terrain.alpine`): a rounding rule that keeps a
dumped TOML file round-trip exact, range validation, and clearance-aware random placement of
an interval inside a span. Callers pass their own ``numpy`` Generator; nothing here touches
global RNG state.
"""
from typing import List, Sequence, Tuple

from bike_sim.physics.checks import scalar

Interval = Tuple[float, float]

CLEARANCE_M = 1.
"""Minimum clear distance between placed features and sections, in metres."""
PLACEMENT_TRIES = 200


def rounded(value, digits=4):
    """Round so saved TOML files reproduce the track bit-for-bit."""
    return round(float(value), digits)


def check_range(value, name, *, minimum=0., integer=False):
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise ValueError(f'{name} must be a (low, high) pair')
    lo, hi = value
    if integer:
        if any(type(v) is not int for v in (lo, hi)):
            raise ValueError(f'{name} must hold integers')
    else:
        lo, hi = scalar(lo, name, minimum=minimum), scalar(hi, name, minimum=minimum)
    if lo > hi or lo < minimum:
        raise ValueError(f'{name} must satisfy {minimum} <= low <= high')
    return (lo, hi)


def is_free(start, end, taken):
    return all(end+CLEARANCE_M <= a or start >= b+CLEARANCE_M for a, b in taken)


def place(rng, length, lo, hi, taken, *, shrink=False):
    """Random start for an interval of `length` inside [lo, hi] clear of `taken`.

    With shrink, the interval is halved (down to 1 m) when it cannot be placed.
    Returns (start, length) or None.
    """
    while True:
        if length <= hi-lo:
            for _ in range(PLACEMENT_TRIES):
                start = rounded(rng.uniform(lo, hi-length), 2)
                if is_free(start, start+length, taken):
                    return start, length
        if not shrink or length/2. < 1.:
            return None
        length = rounded(length/2., 2)


def free_intervals(span: Interval, occupied: Sequence[Interval], gap_m: float) -> List[Interval]:
    """Sub-intervals of ``span`` at least ``gap_m`` clear of every occupied interval."""
    lo, hi = span
    free: List[Interval] = []
    cursor = lo
    for start, end in sorted(occupied):
        right = min(start - gap_m, hi)
        if right > cursor:
            free.append((cursor, right))
        cursor = max(cursor, end + gap_m)
    if hi > cursor:
        free.append((cursor, hi))
    return free


__all__ = ['CLEARANCE_M', 'PLACEMENT_TRIES', 'Interval', 'rounded', 'check_range', 'is_free', 'place',
           'free_intervals']
```

- [ ] **Step 4: Point `generator.py` at the shared helpers**

In `src/bike_sim/terrain/generator.py`:

1. Replace the import block so it reads:

```python
from dataclasses import asdict, dataclass, fields
import hashlib
import json
import numpy as np
from bike_sim.physics.checks import scalar
from bike_sim.terrain.grade import GradeProfile
from bike_sim.terrain.obstacles import Bump, Drop, RoadRoughness, Roots, SquareEdge
from bike_sim.terrain.placement import (
    CLEARANCE_M, PLACEMENT_TRIES, check_range as _range, is_free as _free, place as _place, rounded as _r,
)
from bike_sim.terrain.profile import TrackSpec
from bike_sim.terrain.surface import SURFACES, SurfaceSection
```

2. Delete the two lines `CLEARANCE_M = 1.` and `PLACEMENT_TRIES = 200` (keep the comment line
   `# Minimum clear distance between placed features and sections, in metres.` — move it above
   the `FEATURE_TYPES` line is NOT required; simply delete the comment together with the constants).
3. Delete the whole `def _range(...)` function (lines 34–45).
4. Delete `def _r(...)`, `def _free(...)` and `def _place(...)` (lines 115–138) including the
   `# Round so saved TOML files reproduce the track bit-for-bit.` comment inside `_r`.

Everything else in the file is untouched; all call sites keep using `_r`, `_range`, `_free`, `_place`.

- [ ] **Step 5: Point `road.py` at `free_intervals`**

In `src/bike_sim/terrain/road.py` add after `from bike_sim.terrain.profile import TrackSpec`:

```python
from bike_sim.terrain.placement import free_intervals as _free_intervals
```

and delete the local `def _free_intervals(...)` (lines 127–139). Call sites keep the name.

- [ ] **Step 6: Run the regression test and the two existing terrain users**

Run: `uv run --frozen python -m pytest tests/reference/test_terrain_placement_regression.py tests/reference/test_savage_track_surface.py -q`
Expected: 9 passed.

Run: `uv run --frozen python -c "import bike_sim.terrain, bike_sim.cli.ride, bike_sim.cli.research"`
Expected: no output, exit 0.

- [ ] **Step 7: Commit**

```bash
git add src/bike_sim/terrain/placement.py src/bike_sim/terrain/generator.py src/bike_sim/terrain/road.py tests/reference/test_terrain_placement_regression.py
git commit -m "$(cat <<'EOF'
refactor(terrain): share seeded placement helpers across generators

Pins the procedural generators' outputs by hash first so the extraction
is provably byte-identical.

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```

---

### Task 2: The `scree` surface

**Files:**
- Modify: `src/bike_sim/terrain/surface.py:96-112`
- Test: `tests/reference/test_alpine_track.py` (first tests of the file)

**Interfaces:**
- Produces: `SURFACES['scree']` with `mu_peak=0.45, mu_slide=0.38, slip_stiffness_per_load=4.5`.

- [ ] **Step 1: Write the failing test**

```python
# tests/reference/test_alpine_track.py
"""Alpine scree climb: generator, preset, frozen files and CLI routing."""
from bike_sim.terrain.surface import SURFACES, get_surface


def test_scree_surface_is_registered_below_loose():
    scree, loose = get_surface('scree'), get_surface('loose')
    assert 'scree' in SURFACES
    assert scree.mu_slide <= scree.mu_peak
    assert scree.mu_peak < loose.mu_peak
    assert scree.mu_slide < loose.mu_slide
    assert scree.slip_stiffness_per_load < loose.slip_stiffness_per_load
    assert scree.stribeck_speed_mps == loose.stribeck_speed_mps
```

- [ ] **Step 2: Run it to verify it fails**

Run: `uv run --frozen python -m pytest tests/reference/test_alpine_track.py -q`
Expected: FAIL with `KeyError: "unknown surface 'scree'; available: asphalt, hardpack, loose, wet"`.

- [ ] **Step 3: Add the surface**

In `src/bike_sim/terrain/surface.py`, inside the `SURFACES` tuple after the `"wet"` entry:

```python
        SurfaceSpec("scree", mu_peak=0.45, mu_slide=0.38, slip_stiffness_per_load=4.5,
                    stribeck_speed_mps=DEFAULT_STRIBECK_SPEED_MPS,
                    description="alpine scree: loose rock debris over bedrock"),
```

- [ ] **Step 4: Run tests**

Run: `uv run --frozen python -m pytest tests/reference/test_alpine_track.py tests/reference/test_terrain_placement_regression.py -q`
Expected: 9 passed (the registry order change must not alter any dumped track: dumps store surface names, not indices).

- [ ] **Step 5: Commit**

```bash
git add src/bike_sim/terrain/surface.py tests/reference/test_alpine_track.py
git commit -m "$(cat <<'EOF'
feat(terrain): add the scree surface

Authored friction below loose gravel, for alpine trail sections.

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```

---

### Task 3: `contact_safety.py` — single-contact guards

**Files:**
- Create: `src/bike_sim/terrain/contact_safety.py`
- Create: `tests/reference/test_contact_safety.py`

**Interfaces:**
- Produces: `min_bump_length_m(height_m, min_radius_m=.45) -> float`, `max_bowl_depth_m(length_m, min_radius_m=.45) -> float`, `concave_span_deg(obstacle, window_m=.25) -> float` (obstacle is a `RockGarden` or subclass), `single_contact_hazards(track, *, min_radius_m=.45, max_kink_deg=17., window_m=.25) -> list[str]`, constants `DEFAULT_MIN_CONTACT_RADIUS_M = .45`, `DEFAULT_MAX_KINK_DEG = 17.`, `DEFAULT_CONTACT_WINDOW_M = .25`.

- [ ] **Step 1: Write the failing tests (values come from the spike, spec §4)**

```python
# tests/reference/test_contact_safety.py
"""Closed-form single-contact guards reproduce the rolling-circle spike (spec section 4)."""
import pytest

from bike_sim.terrain.contact_safety import (
    concave_span_deg, max_bowl_depth_m, min_bump_length_m, single_contact_hazards,
)
from bike_sim.terrain.obstacles import BowlPothole, Bump, RoadRoughness, RockGarden, SlopedPothole, SquareEdge
from bike_sim.terrain.profile import TrackSpec


def test_bump_length_floor_matches_spike():
    assert min_bump_length_m(0.10) == pytest.approx(0.9425, abs=1e-4)
    assert min_bump_length_m(0.05) < 0.67
    assert min_bump_length_m(0.15) < 1.16


def test_bowl_depth_ceiling_matches_spike():
    assert max_bowl_depth_m(0.8) == pytest.approx(0.07205, abs=1e-5)
    assert 0.07 <= max_bowl_depth_m(0.8) < 0.10
    assert 0.15 <= max_bowl_depth_m(1.2)
    assert 0.20 <= max_bowl_depth_m(1.5)


def garden(amp, corr, seed, length=8., start=0.):
    return RockGarden(start_m=start, section_length_m=length, amplitude_m=amp, correlation_length_m=corr, seed=seed)


def test_concave_span_flags_the_dirty_spike_seed_and_passes_clean_ones():
    assert concave_span_deg(garden(0.07, 0.45, 5)) >= 20.
    assert concave_span_deg(garden(0.07, 0.45, 0)) < 20.
    assert max(concave_span_deg(garden(0.03, 0.25, s)) for s in range(6)) < 20.
    assert max(concave_span_deg(garden(0.05, 0.30, s)) for s in range(6)) < 20.
    assert max(concave_span_deg(garden(0.10, 0.15, s)) for s in range(6)) >= 20.


def test_concave_span_ignores_convex_crests():
    # This seed's sharpest kink (20.46 deg between adjacent facets) is a crest; the rolling
    # spike found it clean, so the concave-only span must stay below 20 deg.
    assert concave_span_deg(garden(0.05, 0.30, 3)) < 20.


def test_hazards_report_every_unsafe_obstacle_kind():
    track = TrackSpec('t', 60., obstacles=[
        Bump(start_m=5., height_m=.10, bump_length_m=.66),
        Bump(start_m=8., height_m=.10, bump_length_m=.95),
        BowlPothole(start_m=12., depth_m=.10, hole_length_m=.8),
        BowlPothole(start_m=16., depth_m=.07, hole_length_m=.8),
        SlopedPothole(start_m=20., depth_m=.05, hole_length_m=.8),
        SquareEdge(start_m=24., height_m=.05, ledge_length_m=.3),
        garden(0.07, 0.45, 5, start=30.),
        RoadRoughness(start_m=40., section_length_m=8., amplitude_m=.01, correlation_length_m=.10, seed=0),
    ])
    hazards = single_contact_hazards(track)
    labels = [h.split(':')[0] for h in hazards]
    assert labels == ['Bump@5m', 'BowlPothole@12m', 'SlopedPothole@20m', 'SquareEdge@24m', 'RockGarden@30m']


def test_clean_track_has_no_hazards():
    track = TrackSpec('t', 30., obstacles=[Bump(start_m=5., height_m=.05, bump_length_m=.7),
                                          garden(0.03, 0.25, 2, start=10.)])
    assert single_contact_hazards(track) == []
```

- [ ] **Step 2: Run to verify failure**

Run: `uv run --frozen python -m pytest tests/reference/test_contact_safety.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'bike_sim.terrain.contact_safety'`.

- [ ] **Step 3: Implement the module**

```python
# src/bike_sim/terrain/contact_safety.py
"""
Single-Contact Geometry Guards for the ``compliant_2d`` Tyre.

The 2D tyre rests on one support. Two significant supports whose normals differ by more
than ``distinct_normal_deg`` (20 degrees) end a strict run as ``multi_support``. Two
closed-form guards reproduce where that happens
(docs/superpowers/specs/2026-10-11-alpine-scree-track-design.md, section 4):

- smooth raised-cosine shapes (bump, bowl) must keep their radius of curvature at or above
  ``min_radius_m`` -- a crest or trough tighter than the wheel is bridged by two rim points;
- faceted shapes (RockGarden, RoadRoughness) must keep the concave slope increase across any
  ``window_m`` stretch below ``max_kink_deg`` -- the chord the contact patch covers,
  ``2 sqrt(2 R delta)``, is about 0.24 m at 20 mm penetration.

Obstacles with vertical or linear faces are hazards by construction. Pure geometry: no MuJoCo.
"""
from math import pi, sqrt
from typing import List

import numpy as np

from bike_sim.terrain.obstacles import BowlPothole, Bump, GOut, RockGarden
from bike_sim.terrain.profile import TrackSpec

DEFAULT_MIN_CONTACT_RADIUS_M = .45
DEFAULT_MAX_KINK_DEG = 17.
DEFAULT_CONTACT_WINDOW_M = .25


def min_bump_length_m(height_m: float, min_radius_m: float = DEFAULT_MIN_CONTACT_RADIUS_M) -> float:
    """Shortest raised-cosine bump of ``height_m`` whose crest radius stays at or above ``min_radius_m``.

    The crest curvature of ``h/2 (1 - cos(2 pi s / L))`` is ``2 pi^2 h / L^2``.
    """
    return pi * sqrt(2. * height_m * min_radius_m)


def max_bowl_depth_m(length_m: float, min_radius_m: float = DEFAULT_MIN_CONTACT_RADIUS_M) -> float:
    """Deepest raised-cosine bowl of ``length_m`` whose trough radius stays at or above ``min_radius_m``."""
    return length_m ** 2 / (2. * pi ** 2 * min_radius_m)


def concave_span_deg(obstacle: RockGarden, window_m: float = DEFAULT_CONTACT_WINDOW_M) -> float:
    """Largest slope increase between two facets at most ``window_m`` apart, in degrees.

    The joins to flat ground at both ends are included. Convex crests do not count: a wheel
    on a crest has a single support.
    """
    s, z = obstacle._nodes()
    s = np.r_[s[0] - 1., s, s[-1] + 1.]
    z = np.r_[0., z, 0.]
    mid = (s[:-1] + s[1:]) / 2.
    slopes = np.degrees(np.arctan(np.diff(z) / np.diff(s)))
    worst = 0.
    for i in range(len(slopes) - 1):
        ahead = slopes[(mid > mid[i]) & (mid - mid[i] <= window_m)]
        if len(ahead):
            worst = max(worst, float(np.max(ahead) - slopes[i]))
    return worst


def single_contact_hazards(track: TrackSpec, *, min_radius_m: float = DEFAULT_MIN_CONTACT_RADIUS_M,
                           max_kink_deg: float = DEFAULT_MAX_KINK_DEG,
                           window_m: float = DEFAULT_CONTACT_WINDOW_M) -> List[str]:
    """Obstacles of ``track`` that can put the 2D tyre on two supports; an empty list means clean."""
    hazards = []
    for o in track.sorted_obstacles:
        if isinstance(o, RockGarden):
            span = concave_span_deg(o, window_m)
            if span >= max_kink_deg:
                hazards.append(f'{o.label}: concave span {span:.1f} deg >= {max_kink_deg:g} deg')
        elif isinstance(o, Bump):
            need = min_bump_length_m(o.height_m, min_radius_m)
            if o.bump_length_m < need - 1e-9:
                hazards.append(f'{o.label}: length {o.bump_length_m:.3f} m < {need:.3f} m')
        elif isinstance(o, (BowlPothole, GOut)):
            length = o.hole_length_m if isinstance(o, BowlPothole) else o.dip_length_m
            limit = max_bowl_depth_m(length, min_radius_m)
            if o.depth_m > limit + 1e-9:
                hazards.append(f'{o.label}: depth {o.depth_m:.3f} m > {limit:.3f} m')
        else:
            hazards.append(f'{o.label}: {type(o).__name__} has faces the 2D tyre cannot ride')
    return hazards


__all__ = [
    'DEFAULT_MIN_CONTACT_RADIUS_M', 'DEFAULT_MAX_KINK_DEG', 'DEFAULT_CONTACT_WINDOW_M',
    'min_bump_length_m', 'max_bowl_depth_m', 'concave_span_deg', 'single_contact_hazards',
]
```

- [ ] **Step 4: Run tests**

Run: `uv run --frozen python -m pytest tests/reference/test_contact_safety.py -q`
Expected: 6 passed. If `test_concave_span_flags_the_dirty_spike_seed_and_passes_clean_ones` fails on the `(0.10, 0.15)` line, the spike data says those gardens flag at 2.5–12 %; check that `_nodes()` is called on the obstacle, not the class.

- [ ] **Step 5: Commit**

```bash
git add src/bike_sim/terrain/contact_safety.py tests/reference/test_contact_safety.py
git commit -m "$(cat <<'EOF'
feat(terrain): closed-form single-contact guards for the 2D tyre

Curvature floor for raised-cosine shapes and a windowed concave slope
span for faceted sections, both calibrated against a rolling-circle
sweep through ProfileQuery.

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```

---

### Task 4: `AlpineGenSpec` with validation and dict/hash round-trip

**Files:**
- Create: `src/bike_sim/terrain/alpine.py` (spec only; the generator comes in Tasks 5–6)
- Test: `tests/reference/test_alpine_track.py`

**Interfaces:**
- Produces: `AlpineGenSpec` frozen dataclass with the fields and defaults of spec §5.1, `AlpineGenSpec.from_dict(data) -> AlpineGenSpec`, `.to_dict() -> dict`, `.sha256() -> str`, `ALPINE_PRESET_SEED = 41`.

- [ ] **Step 1: Write the failing tests (append to `tests/reference/test_alpine_track.py`)**

```python
import tomllib

import pytest

from bike_sim.terrain.alpine import ALPINE_PRESET_SEED, AlpineGenSpec


def test_default_spec_matches_the_design_numbers():
    s = AlpineGenSpec()
    assert (s.length_m, s.lead_in_m, s.lead_out_m, s.transition_m) == (120., 10., 15., 2.)
    assert s.ramp_grade == (.15, .25) and s.wall_grade == (.26, .32) and s.n_walls == 1
    assert s.counter_grade == (-.08, -.03) and s.counter_prob == .3
    assert s.base_surface == 'hardpack' and s.n_scree == (2, 3) and s.scree_ramp_bias == .7
    assert s.n_sharp_edges == (0, 0) and s.single_contact_safe is True
    assert (s.min_contact_radius_m, s.max_kink_deg, s.contact_window_m) == (.45, 17., .25)
    assert isinstance(ALPINE_PRESET_SEED, int)


def test_spec_round_trips_through_dict_and_toml_lists():
    spec = AlpineGenSpec(ramp_grade=[.10, .20], n_boulders=[1, 2])
    assert spec.ramp_grade == (.10, .20) and spec.n_boulders == (1, 2)
    again = AlpineGenSpec.from_dict(tomllib.loads('ramp_grade = [0.10, 0.20]\nn_boulders = [1, 2]\n'))
    assert again == spec and again.sha256() == spec.sha256()
    assert AlpineGenSpec.from_dict(spec.to_dict()) == spec
    assert AlpineGenSpec().sha256() != spec.sha256()


@pytest.mark.parametrize('bad', [
    dict(unknown_key=1),
    dict(ramp_grade=(.25, .15)),
    dict(counter_grade=(-.05, .01)),
    dict(n_boulders=(1.5, 2)),
    dict(counter_prob=1.5),
    dict(n_walls=2),
    dict(wall_grade=(.20, .30)),
    dict(base_surface='ice'),
    dict(length_m=30.),
    dict(n_sharp_edges=(1, 1)),
    dict(single_contact_safe='yes'),
])
def test_spec_rejects_invalid_values(bad):
    with pytest.raises(ValueError):
        AlpineGenSpec.from_dict(bad)


def test_sharp_edges_need_the_multi_contact_opt_in():
    spec = AlpineGenSpec(single_contact_safe=False, n_sharp_edges=(1, 2))
    assert spec.n_sharp_edges == (1, 2)
```

- [ ] **Step 2: Run to verify failure**

Run: `uv run --frozen python -m pytest tests/reference/test_alpine_track.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'bike_sim.terrain.alpine'`.

- [ ] **Step 3: Create `alpine.py` with the spec**

```python
# src/bike_sim/terrain/alpine.py
"""
Seeded Alpine Scree-Trail Generator.

A high-mountain MTB climb above the treeline, built for traction, wheel-slip and
front-wheel-lift research: a trail rhythm of steep ramps and switchback shelves, scree
sections (a low-friction surface plus fine texture), embedded boulders, small rock gardens
and drainage bowls on hardpack.

The result is an ordinary validated TrackSpec, so it runs through the same heightfield,
dumps to TOML and loads back. One numpy Generator drives every draw in a fixed order:
(spec, seed) fully determines the track; nothing here touches global RNG state.

With ``single_contact_safe`` (the default) every obstacle is drawn to satisfy the
``compliant_2d`` tyre's one-support geometry (:mod:`bike_sim.terrain.contact_safety`), so a
strict research run never ends in ``multi_support``. Switching it off removes the guards for
a multi-contact tyre; the ranges are then the caller's responsibility.
Design: docs/superpowers/specs/2026-10-11-alpine-scree-track-design.md.
"""
from dataclasses import asdict, dataclass, fields
import hashlib
import json
from math import ceil

import numpy as np

from bike_sim.physics.checks import scalar
from bike_sim.terrain.contact_safety import concave_span_deg, max_bowl_depth_m, min_bump_length_m
from bike_sim.terrain.grade import GradeProfile
from bike_sim.terrain.obstacles import BowlPothole, Bump, RoadRoughness, RockGarden, SquareEdge
from bike_sim.terrain.placement import PLACEMENT_TRIES, check_range, free_intervals, place, rounded as _r
from bike_sim.terrain.profile import TrackSpec
from bike_sim.terrain.surface import SURFACES, SurfaceSection

ALPINE_PRESET_SEED = 41
MIN_BACKGROUND_CORRELATIONS = 4.
"""Free stretches shorter than this many correlation lengths get no background texture."""

_RANGE_FIELDS = ('ramp_len_m', 'ramp_grade', 'shelf_len_m', 'shelf_grade', 'wall_grade', 'counter_len_m',
                 'scree_len_m', 'scree_amp_m', 'scree_corr_m', 'wet_len_m', 'background_amp_m',
                 'background_corr_m', 'boulder_height_m', 'boulder_len_m', 'rock_len_m', 'rock_amp_m',
                 'rock_corr_m', 'water_bar_depth_m', 'water_bar_len_m', 'sharp_edge_height_m')
_COUNT_FIELDS = ('n_scree', 'n_wet', 'n_boulders', 'n_rock_gardens', 'n_water_bars', 'n_sharp_edges')
_POSITIVE_FIELDS = ('length_m', 'lead_in_m', 'lead_out_m', 'transition_m', 'min_contact_radius_m',
                    'max_kink_deg', 'contact_window_m')


@dataclass(frozen=True)
class AlpineGenSpec:
    """Ranges an alpine scree climb is drawn from (lengths m, heights m, grades dz/dx)."""
    length_m: float = 120.
    lead_in_m: float = 10.
    lead_out_m: float = 15.
    transition_m: float = 2.
    ramp_len_m: tuple = (15., 30.)
    ramp_grade: tuple = (.15, .25)
    shelf_len_m: tuple = (4., 8.)
    shelf_grade: tuple = (.02, .06)
    wall_grade: tuple = (.26, .32)
    n_walls: int = 1
    counter_len_m: tuple = (3., 6.)
    counter_grade: tuple = (-.08, -.03)
    counter_prob: float = .3
    base_surface: str = 'hardpack'
    n_scree: tuple = (2, 3)
    scree_len_m: tuple = (6., 15.)
    scree_ramp_bias: float = .7
    scree_amp_m: tuple = (.010, .020)
    scree_corr_m: tuple = (.15, .20)
    n_wet: tuple = (0, 0)
    wet_len_m: tuple = (2., 5.)
    background_amp_m: tuple = (.005, .010)
    background_corr_m: tuple = (.20, .30)
    n_boulders: tuple = (2, 5)
    boulder_height_m: tuple = (.05, .15)
    boulder_len_m: tuple = (.6, 1.2)
    n_rock_gardens: tuple = (1, 2)
    rock_len_m: tuple = (4., 8.)
    rock_amp_m: tuple = (.03, .06)
    rock_corr_m: tuple = (.30, .45)
    n_water_bars: tuple = (1, 3)
    water_bar_depth_m: tuple = (.08, .15)
    water_bar_len_m: tuple = (1.0, 1.5)
    n_sharp_edges: tuple = (0, 0)
    sharp_edge_height_m: tuple = (.03, .08)
    single_contact_safe: bool = True
    min_contact_radius_m: float = .45
    max_kink_deg: float = 17.
    contact_window_m: float = .25

    def __post_init__(self):
        # JSON/TOML give lists; freeze them so the dataclass stays hashable/immutable.
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, list):
                object.__setattr__(self, f.name, tuple(value))
        for name in _RANGE_FIELDS:
            lo, hi = check_range(getattr(self, name), name)
            object.__setattr__(self, name, (float(lo), float(hi)))
        lo, hi = check_range(self.counter_grade, 'counter_grade', minimum=-1.)
        if hi >= 0.:
            raise ValueError('counter_grade must be negative: it is a short descent')
        object.__setattr__(self, 'counter_grade', (float(lo), float(hi)))
        for name in _COUNT_FIELDS:
            check_range(getattr(self, name), name, integer=True)
        for name in _POSITIVE_FIELDS:
            scalar(getattr(self, name), name, positive=True)
        for name in ('counter_prob', 'scree_ramp_bias'):
            if scalar(getattr(self, name), name, minimum=0.) > 1.:
                raise ValueError(f'{name} must lie in [0, 1]')
        if type(self.n_walls) is not int or not 0 <= self.n_walls <= 1:
            raise ValueError('n_walls must be 0 or 1')
        if type(self.single_contact_safe) is not bool:
            raise ValueError('single_contact_safe must be a bool')
        if self.wall_grade[0] < self.ramp_grade[1]:
            raise ValueError('wall_grade must start at or above the top of ramp_grade')
        if self.base_surface not in SURFACES:
            raise ValueError(f'unknown base_surface {self.base_surface!r}; available: {", ".join(SURFACES)}')
        if self.lead_in_m + self.lead_out_m + 2. * self.transition_m + self.ramp_len_m[0] > self.length_m:
            raise ValueError('the body must hold at least one shortest ramp between lead-in and lead-out')
        if self.single_contact_safe and self.n_sharp_edges[1] > 0:
            raise ValueError('sharp edges always put the 2D tyre on two supports; '
                             'set single_contact_safe = false to opt in')

    @classmethod
    def from_dict(cls, data):
        known = {f.name for f in fields(cls)}
        unknown = sorted(set(data) - known)
        if unknown:
            raise ValueError(f'unknown alpine generator keys: {unknown}')
        return cls(**data)

    def to_dict(self):
        return {k: list(v) if isinstance(v, tuple) else v for k, v in asdict(self).items()}

    def sha256(self):
        return hashlib.sha256(json.dumps(self.to_dict(), sort_keys=True).encode()).hexdigest()
```

(The imports of `ceil`, `np`, `concave_span_deg`, `max_bowl_depth_m`, `min_bump_length_m`, `GradeProfile`, the obstacle classes, `PLACEMENT_TRIES`, `free_intervals`, `place`, `TrackSpec`, `SurfaceSection` and `MIN_BACKGROUND_CORRELATIONS` are used by Tasks 5–6; keep them now so the file is complete in one shape.)

- [ ] **Step 4: Run tests**

Run: `uv run --frozen python -m pytest tests/reference/test_alpine_track.py -q`
Expected: 15 passed (1 surface test + 3 spec tests + 11 parametrized rejections; `n_boulders=(1.5, 2)` is rejected by `check_range(..., integer=True)`).

- [ ] **Step 5: Commit**

```bash
git add src/bike_sim/terrain/alpine.py tests/reference/test_alpine_track.py
git commit -m "$(cat <<'EOF'
feat(terrain): AlpineGenSpec ranges with validation and hashing

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```

---

### Task 5: Grade rhythm `_rhythm`

**Files:**
- Modify: `src/bike_sim/terrain/alpine.py` (append)
- Test: `tests/reference/test_alpine_track.py`

**Interfaces:**
- Produces: `_rhythm(spec, rng) -> tuple[GradeProfile, list[tuple[float, float]]]` — grade profile and the `(start, end)` of every ramp or wall. `_segments(spec, rng) -> list[list]` is the inner builder returning `[kind, start, end, grade]` rows with `kind in ('ramp', 'shelf', 'counter', 'wall')`; tests use it directly.

- [ ] **Step 1: Write the failing tests (append)**

```python
import numpy as np

from bike_sim.terrain.alpine import _rhythm, _segments


def _kinds(seed, **overrides):
    spec = AlpineGenSpec(**overrides)
    return spec, _segments(spec, np.random.default_rng(seed))


@pytest.mark.parametrize('seed', range(8))
def test_rhythm_fills_the_body_with_ramp_shelf_cycles(seed):
    spec, segments = _kinds(seed)
    body_end, t = spec.length_m - spec.lead_out_m, spec.transition_m
    kinds = [s[0] for s in segments]
    assert kinds[0] in ('ramp', 'wall')
    assert kinds.count('wall') == 1
    assert all(k in ('ramp', 'wall', 'shelf', 'counter') for k in kinds)
    for (_, s, e, _g), (_, s2, _e2, _g2) in zip(segments, segments[1:]):
        assert s < e and s2 == pytest.approx(e + t, abs=1e-9)
    assert segments[0][1] == pytest.approx(spec.lead_in_m + t)
    assert segments[-1][2] <= body_end - t + 1e-9
    assert body_end - t - segments[-1][2] < spec.ramp_len_m[0] + t  # no room left for another ramp
    for kind, s, e, g in segments:
        lo, hi = {'ramp': spec.ramp_grade, 'wall': spec.wall_grade, 'shelf': spec.shelf_grade,
                  'counter': spec.counter_grade}[kind]
        assert lo <= g <= hi
        if kind == 'ramp':
            assert spec.ramp_len_m[0] <= e - s <= spec.ramp_len_m[1] + 1e-9


@pytest.mark.parametrize('seed', range(8))
def test_rhythm_grade_profile_is_flat_outside_the_body(seed):
    spec = AlpineGenSpec()
    grade, ramps = _rhythm(spec, np.random.default_rng(seed))
    x = np.linspace(0., spec.length_m, 2401)
    slope = grade.slope(x)
    assert np.all(slope[x <= spec.lead_in_m] == 0.)
    assert np.all(slope[x >= spec.length_m - spec.lead_out_m] == 0.)
    assert slope.max() <= spec.wall_grade[1] + 1e-12
    assert slope.min() >= spec.counter_grade[0] - 1e-12
    assert ramps and all(spec.lead_in_m < s < e < spec.length_m - spec.lead_out_m for s, e in ramps)
    assert grade.elevation(spec.length_m) > 8.  # a climb, not a flat


def test_rhythm_without_walls_or_counter_slopes():
    spec, segments = _kinds(3, n_walls=0, counter_prob=0.)
    assert {s[0] for s in segments} <= {'ramp', 'shelf'}
```

- [ ] **Step 2: Run to verify failure**

Run: `uv run --frozen python -m pytest tests/reference/test_alpine_track.py -q -k rhythm`
Expected: FAIL with `ImportError: cannot import name '_rhythm'`.

- [ ] **Step 3: Append the rhythm builder to `alpine.py`**

```python
def _segments(spec, rng):
    """Fill the body with ramp -> shelf [-> counter-slope] cycles; rows are [kind, start, end, grade].

    Each segment ends ``transition_m`` before the next starts so the grade profile ramps
    linearly between them. When the remaining body is too short for a ramp, the last
    segment runs on to the body end; one ramp becomes the wall when ``n_walls`` is 1.
    """
    body_end = _r(spec.length_m - spec.lead_out_m, 2)
    limit = _r(body_end - spec.transition_m, 2)
    cursor = _r(spec.lead_in_m + spec.transition_m, 2)
    segments = []

    def push(kind, length, grade):
        nonlocal cursor
        end = _r(min(cursor + length, limit), 2)
        segments.append([kind, cursor, end, grade])
        cursor = _r(end + spec.transition_m, 2)
        return end < limit

    while cursor < limit:
        length, grade = _r(rng.uniform(*spec.ramp_len_m), 2), _r(rng.uniform(*spec.ramp_grade))
        if limit - cursor < spec.ramp_len_m[0]:
            segments[-1][2] = limit
            break
        if not push('ramp', length, grade):
            break
        if not push('shelf', _r(rng.uniform(*spec.shelf_len_m), 2), _r(rng.uniform(*spec.shelf_grade))):
            break
        if rng.uniform() < spec.counter_prob and not push(
                'counter', _r(rng.uniform(*spec.counter_len_m), 2), _r(rng.uniform(*spec.counter_grade))):
            break
    if spec.n_walls:
        ramps = [i for i, s in enumerate(segments) if s[0] == 'ramp']
        i = ramps[int(rng.integers(0, len(ramps)))]
        segments[i][0], segments[i][3] = 'wall', _r(rng.uniform(*spec.wall_grade))
    return segments


def _rhythm(spec, rng):
    """Grade profile of the trail rhythm plus the (start, end) of every ramp or wall."""
    segments = _segments(spec, rng)
    body_end = _r(spec.length_m - spec.lead_out_m, 2)
    knots = [(0., 0.), (spec.lead_in_m, 0.)]
    knots += [knot for _, s, e, g in segments for knot in ((s, g), (e, g))]
    knots += [(body_end, 0.), (spec.length_m, 0.)]
    return GradeProfile(tuple(knots)), [(s, e) for kind, s, e, _ in segments if kind in ('ramp', 'wall')]
```

- [ ] **Step 4: Run tests**

Run: `uv run --frozen python -m pytest tests/reference/test_alpine_track.py -q`
Expected: 32 passed. If `test_rhythm_fills_the_body_with_ramp_shelf_cycles` fails on the ramp-length line for a seed, the ramp was truncated by `limit`; truncation is only allowed when `limit - cursor >= ramp_len_m[0]`, which the preceding check guarantees — so a failure means the check and `push` disagree on rounding; compare `limit - cursor` with `_r(..., 2)` applied to both.

- [ ] **Step 5: Commit**

```bash
git add src/bike_sim/terrain/alpine.py tests/reference/test_alpine_track.py
git commit -m "$(cat <<'EOF'
feat(terrain): alpine grade rhythm of ramps, shelves and counter-slopes

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```

---

### Task 6: `generate_alpine_track` — surfaces, obstacles, background, assembly

**Files:**
- Modify: `src/bike_sim/terrain/alpine.py` (append)
- Test: `tests/reference/test_alpine_track.py`

**Interfaces:**
- Consumes: `_rhythm`, `place`, `free_intervals`, `concave_span_deg`, `min_bump_length_m`, `max_bowl_depth_m`.
- Produces: `generate_alpine_track(spec, *, seed, name=None) -> TrackSpec`; `alpine_scree(seed=ALPINE_PRESET_SEED) -> TrackSpec` (named `alpine_scree` at the pinned seed, `alpine_scree_<seed>` otherwise).

- [ ] **Step 1: Write the failing tests (append). T1, T2, T3 of the spec.**

```python
import tomllib as _tomllib

from bike_sim.physics.physical_config import TireBackendConfig
from bike_sim.terrain.alpine import alpine_scree, generate_alpine_track
from bike_sim.terrain.contact_profile import ProfileQuery
from bike_sim.terrain.contact_safety import single_contact_hazards
from bike_sim.terrain.heightfield import build_field_data
from bike_sim.terrain.obstacles import BowlPothole, Bump, RoadRoughness, RockGarden, SquareEdge
from bike_sim.terrain.profile import build_profile
from bike_sim.terrain.trackfile import dump_track, track_from_dict, track_to_dict
from bike_sim.terrain.wheelpath import FRONT_WHEEL_RADIUS_M, REAR_WHEEL_RADIUS_M

SEEDS = range(8)


def test_generate_rejects_bad_arguments():
    with pytest.raises(ValueError):
        generate_alpine_track(object(), seed=0)
    with pytest.raises(ValueError):
        generate_alpine_track(AlpineGenSpec(), seed=-1)
    with pytest.raises(ValueError):
        generate_alpine_track(AlpineGenSpec(), seed=1.0)


@pytest.mark.parametrize('seed', SEEDS)
def test_same_seed_same_bytes_and_toml_round_trip(seed):
    spec = AlpineGenSpec()
    first, second = dump_track(generate_alpine_track(spec, seed=seed)), dump_track(generate_alpine_track(spec, seed=seed))
    assert first == second
    doc = _tomllib.loads(first)
    assert track_to_dict(track_from_dict(doc)) == track_to_dict(generate_alpine_track(spec, seed=seed))


def test_different_seeds_differ():
    dumps = {dump_track(generate_alpine_track(AlpineGenSpec(), seed=s)) for s in SEEDS}
    assert len(dumps) == len(SEEDS)


def _body(track, spec):
    return spec.lead_in_m, spec.length_m - spec.lead_out_m


@pytest.mark.parametrize('seed', SEEDS)
def test_default_track_structure(seed):
    spec = AlpineGenSpec()
    track = generate_alpine_track(spec, seed=seed)
    track.validate()
    lo, hi = _body(track, spec)
    assert track.length_m == 120. and track.surface == 'hardpack' and track.name == f'alpine_{seed}'
    assert f'seed={seed}' in track.description and spec.sha256() in track.description
    assert all(lo <= o.start_m and o.end_m <= hi + 1e-9 for o in track.obstacles)
    assert all(lo <= s.start_m and s.end_m <= hi + 1e-9 for s in track.surface_sections)
    x = np.array([0., 5., 10., 105., 112., 120.])
    assert np.all(track.grade_profile.slope(x) == 0.)

    scree = [s for s in track.surface_sections if s.surface == 'scree']
    rough = [o for o in track.obstacles if isinstance(o, RoadRoughness)]
    assert len(scree) >= 2
    for section in scree:
        matches = [o for o in rough if abs(o.start_m - section.start_m) < 1e-6 and abs(o.end_m - section.end_m) < 1e-6]
        assert len(matches) == 1
        assert spec.scree_amp_m[0] * .8 ** 6 <= matches[0].amplitude_m <= spec.scree_amp_m[1]
    gardens = [o for o in track.obstacles if type(o) is RockGarden]
    boulders = [o for o in track.obstacles if isinstance(o, Bump)]
    bowls = [o for o in track.obstacles if isinstance(o, BowlPothole)]
    assert 1 <= len(gardens) <= spec.n_rock_gardens[1]
    assert 1 <= len(boulders) <= spec.n_boulders[1]
    assert 1 <= len(bowls) <= spec.n_water_bars[1]
    assert not any(isinstance(o, SquareEdge) for o in track.obstacles)
    assert all(spec.boulder_height_m[0] <= o.height_m <= spec.boulder_height_m[1] for o in boulders)
    assert all(o.depth_m <= spec.water_bar_depth_m[1] for o in bowls)
    assert build_field_data(track).shape[0] == 2


@pytest.mark.parametrize('seed', SEEDS)
def test_default_track_has_no_single_contact_hazards(seed):
    assert single_contact_hazards(generate_alpine_track(AlpineGenSpec(), seed=seed)) == []


def roll_multi_support_count(track, radius, *, delta_m=.010, step_m=.005):
    """Ground truth: roll a circle along the 5 mm raster through the production contact query."""
    cfg = TireBackendConfig()
    x = np.arange(0., track.length_m + 2. * radius + step_m / 2., step_m)
    z = build_profile(track, x)
    query = ProfileQuery(np.column_stack([x, z]), significant_delta_m=cfg.significant_delta_m,
                         significance_fraction=cfg.significance_fraction, normal_angle_deg=cfg.distinct_normal_deg)
    window = int(round(radius / step_m))
    count = 0
    for i in range(window + 1, len(x) - window - 1):
        lo, hi = i - window, i + window + 1
        dx = x[i] - x[lo:hi]
        lift = np.sqrt(np.maximum(radius * radius - dx * dx, 0.))
        centre = np.array([x[i], float(np.max(z[lo:hi] + lift)) - delta_m])
        count += query.contact(centre, radius).multi_support
    return count


@pytest.mark.parametrize('seed', range(3))
def test_default_track_rolls_clean_for_both_wheels(seed):
    track = generate_alpine_track(AlpineGenSpec(), seed=seed)
    assert roll_multi_support_count(track, FRONT_WHEEL_RADIUS_M) == 0
    assert roll_multi_support_count(track, REAR_WHEEL_RADIUS_M) == 0


def test_opting_out_of_single_contact_produces_detectable_hazards():
    spec = AlpineGenSpec(single_contact_safe=False, n_sharp_edges=(2, 2), rock_amp_m=(.10, .10), rock_corr_m=(.15, .15))
    track = generate_alpine_track(spec, seed=0)
    assert any(isinstance(o, SquareEdge) for o in track.obstacles)
    assert single_contact_hazards(track)
    assert roll_multi_support_count(track, FRONT_WHEEL_RADIUS_M) > 0


def test_redraw_keeps_a_marginal_rock_garden_clean():
    spec = AlpineGenSpec(rock_amp_m=(.07, .07), rock_corr_m=(.45, .45), n_rock_gardens=(2, 2))
    for seed in SEEDS:
        track = generate_alpine_track(spec, seed=seed)
        assert single_contact_hazards(track) == []
        assert all(o.amplitude_m <= .07 for o in track.obstacles if type(o) is RockGarden)


def test_alpine_scree_preset_builder():
    pinned, other = alpine_scree(), alpine_scree(seed=7)
    assert pinned.name == 'alpine_scree' and other.name == 'alpine_scree_7'
    assert dump_track(pinned) == dump_track(generate_alpine_track(AlpineGenSpec(), seed=ALPINE_PRESET_SEED, name='alpine_scree'))
```

- [ ] **Step 2: Run to verify failure**

Run: `uv run --frozen python -m pytest tests/reference/test_alpine_track.py -q -k "generate or track or roll or redraw or preset_builder"`
Expected: FAIL with `ImportError: cannot import name 'alpine_scree'`.

- [ ] **Step 3: Append the generator to `alpine.py`**

```python
def _count(rng, bounds):
    return int(rng.integers(bounds[0], bounds[1] + 1))


def _faceted(spec, rng, cls, start, length, amp, corr):
    """A seeded faceted section whose concave span respects ``max_kink_deg``.

    New seeds are drawn first; if none of PLACEMENT_TRIES passes, the amplitude shrinks by
    a fifth and the draw repeats. Without ``single_contact_safe`` the first seed is kept.
    """
    while True:
        for _ in range(PLACEMENT_TRIES):
            obstacle = cls(start_m=start, section_length_m=length, amplitude_m=amp,
                           correlation_length_m=corr, seed=int(rng.integers(0, 2**31)))
            if not spec.single_contact_safe or concave_span_deg(obstacle, spec.contact_window_m) < spec.max_kink_deg:
                return obstacle
        amp = _r(amp * .8)


def generate_alpine_track(spec, *, seed, name=None):
    """Draws one alpine scree climb; ``(spec, seed)`` fully determines the result."""
    if not isinstance(spec, AlpineGenSpec):
        raise ValueError('expected an AlpineGenSpec')
    if type(seed) is not int or seed < 0:
        raise ValueError('terrain seed must be a nonnegative integer')
    rng = np.random.default_rng(seed)
    lo, hi = spec.lead_in_m, _r(spec.length_m - spec.lead_out_m, 2)
    grade, ramps = _rhythm(spec, rng)
    taken, used, obstacles, sections = [], [], [], []

    for _ in range(_count(rng, spec.n_rock_gardens)):
        wanted = _r(rng.uniform(*spec.rock_len_m), 2)
        amp, corr = _r(rng.uniform(*spec.rock_amp_m)), _r(rng.uniform(*spec.rock_corr_m), 3)
        placed = place(rng, wanted, lo, hi, taken, shrink=True)
        if placed is not None:
            start, length = placed
            obstacles.append(_faceted(spec, rng, RockGarden, start, length, amp, corr))
            taken.append((start, start + length))

    for _ in range(_count(rng, spec.n_scree)):
        wanted = _r(rng.uniform(*spec.scree_len_m), 2)
        amp, corr = _r(rng.uniform(*spec.scree_amp_m)), _r(rng.uniform(*spec.scree_corr_m), 3)
        a, b = ramps[int(rng.integers(0, len(ramps)))] if rng.uniform() < spec.scree_ramp_bias else (lo, hi)
        placed = place(rng, wanted, a, b, taken, shrink=True)
        if placed is not None:
            start, length = placed
            obstacles.append(_faceted(spec, rng, RoadRoughness, start, length, amp, corr))
            sections.append(SurfaceSection(start, _r(start + length, 2), 'scree'))
            taken.append((start, start + length))
            used.append((start, start + length))

    for _ in range(_count(rng, spec.n_boulders)):
        height, length = _r(rng.uniform(*spec.boulder_height_m)), _r(rng.uniform(*spec.boulder_len_m), 2)
        if spec.single_contact_safe:
            length = max(length, _r(ceil(min_bump_length_m(height, spec.min_contact_radius_m) * 100.) / 100., 2))
        placed = place(rng, length, lo, hi, taken)
        if placed is not None:
            start, _ = placed
            obstacles.append(Bump(start_m=start, height_m=height, bump_length_m=length))
            taken.append((start, start + length))

    for _ in range(_count(rng, spec.n_water_bars)):
        depth, length = _r(rng.uniform(*spec.water_bar_depth_m)), _r(rng.uniform(*spec.water_bar_len_m), 2)
        if spec.single_contact_safe:
            depth = min(depth, _r(max_bowl_depth_m(length, spec.min_contact_radius_m) * .98))
        placed = place(rng, length, lo, hi, taken)
        if placed is not None:
            start, _ = placed
            obstacles.append(BowlPothole(start_m=start, depth_m=depth, hole_length_m=length))
            taken.append((start, start + length))

    for _ in range(_count(rng, spec.n_sharp_edges)):
        height, length = _r(rng.uniform(*spec.sharp_edge_height_m)), _r(rng.uniform(.2, .6), 2)
        placed = place(rng, length, lo, hi, taken)
        if placed is not None:
            start, _ = placed
            obstacles.append(SquareEdge(start_m=start, height_m=height, ledge_length_m=length))
            taken.append((start, start + length))

    for _ in range(_count(rng, spec.n_wet)):
        wanted = _r(rng.uniform(*spec.wet_len_m), 2)
        placed = place(rng, wanted, lo, hi, used, shrink=True)
        if placed is not None:
            start, length = placed
            sections.append(SurfaceSection(start, _r(start + length, 2), 'wet'))
            used.append((start, start + length))

    amp, corr = _r(rng.uniform(*spec.background_amp_m)), _r(rng.uniform(*spec.background_corr_m), 3)
    for a, b in free_intervals((lo, hi), taken, 0.):
        if b - a >= MIN_BACKGROUND_CORRELATIONS * corr:
            # Trim float noise from the segment length without crossing into the neighbour.
            length = round(b - a, 6)
            while a + length > b:
                length = round(length - 1e-6, 6)
            obstacles.append(_faceted(spec, rng, RoadRoughness, a, length, amp, corr))

    track = TrackSpec(name or f'alpine_{seed}', spec.length_m, obstacles=obstacles, surface=spec.base_surface,
                      grade_profile=grade, surface_sections=tuple(sorted(sections, key=lambda s: s.start_m)),
                      description=f'Alpine scree climb; seed={seed}; spec_sha256={spec.sha256()}')
    track.validate()
    return track


def alpine_scree(seed=ALPINE_PRESET_SEED):
    """Builds the ``alpine_scree`` preset: the default spec at the pinned seed, or another seed."""
    name = 'alpine_scree' if seed == ALPINE_PRESET_SEED else f'alpine_scree_{seed}'
    return generate_alpine_track(AlpineGenSpec(), seed=seed, name=name)


__all__ = ['ALPINE_PRESET_SEED', 'AlpineGenSpec', 'generate_alpine_track', 'alpine_scree']
```

- [ ] **Step 4: Run the tests**

Run: `uv run --frozen python -m pytest tests/reference/test_alpine_track.py -q`
Expected: all pass; the three rolling tests take ~5–10 s each. Known ways this can fail and what to do:

- `test_default_track_structure` asserts `>= 1` garden/boulder/bowl: if a seed leaves no room, the defaults are too dense — lower `scree_len_m` to `(6., 12.)` in **both** the dataclass default and spec §5.1 (do not weaken the test).
- `test_default_track_rolls_clean_for_both_wheels` with a non-zero count: print the flagged x positions and the obstacle there; if it is a `RoadRoughness`/`RockGarden`, lower `DEFAULT_MAX_KINK_DEG`/`max_kink_deg` to 15 in `contact_safety.py`, `alpine.py` and the spec; if it is a `Bump`/`BowlPothole`, the curvature floor is being applied with the wrong radius — check `spec.min_contact_radius_m` is passed.
- `test_same_seed_same_bytes_and_toml_round_trip`: if `track_to_dict` differs after the file round trip, a float left `_r` unrounded — every obstacle size must go through `_r`.

- [ ] **Step 5: Run the quick profile**

Run: `bash tools/run_tests.sh quick`
Expected: green.

- [ ] **Step 6: Commit**

```bash
git add src/bike_sim/terrain/alpine.py tests/reference/test_alpine_track.py
git commit -m "$(cat <<'EOF'
feat(terrain): seeded alpine scree climb generator

Scree sections pair a low-friction surface zone with fine texture;
boulders, rock gardens and drainage bowls are drawn inside the 2D
tyre's single-contact envelope unless the spec opts out.

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```

---

### Task 7: Preset registry, package exports, frozen example files

**Files:**
- Modify: `src/bike_sim/terrain/presets.py:18-33`, `:179-191`, `:219-230`
- Modify: `src/bike_sim/terrain/__init__.py`
- Create: `examples/research/alpine_scree.toml`
- Create: `examples/research/alpine_rocky_genspec.toml`
- Test: `tests/reference/test_alpine_track.py`

**Interfaces:**
- Produces: `presets.PRESETS['alpine_scree']`, `presets.SEEDED_PRESETS: Dict[str, Tuple[Callable[..., TrackSpec], int]] = {'alpine_scree': (alpine_scree, ALPINE_PRESET_SEED)}`; package-level exports `AlpineGenSpec, generate_alpine_track, alpine_scree, ALPINE_PRESET_SEED, SEEDED_PRESETS, single_contact_hazards`.

- [ ] **Step 1: Write the failing tests (append)**

```python
from pathlib import Path

from bike_sim.terrain import SEEDED_PRESETS, available_presets, get_preset
from bike_sim.terrain.trackfile import load_track

ROOT = Path(__file__).resolve().parents[2]


def test_preset_is_registered_and_seeded():
    assert 'alpine_scree' in available_presets()
    assert SEEDED_PRESETS['alpine_scree'] == (alpine_scree, ALPINE_PRESET_SEED)
    track = get_preset('alpine_scree')
    assert track.name == 'alpine_scree' and track.length_m == 120.


def test_frozen_example_equals_the_preset_dump():
    path = ROOT / 'examples/research/alpine_scree.toml'
    assert path.read_text(encoding='utf-8') == dump_track(get_preset('alpine_scree'))
    track = load_track(path)
    assert track.surface == 'hardpack'
    assert any(s.surface == 'scree' for s in track.surface_sections)
    assert track.length_m == 120.


def test_rocky_genspec_is_the_multi_contact_opt_in():
    spec = AlpineGenSpec.from_dict(_tomllib.loads((ROOT / 'examples/research/alpine_rocky_genspec.toml').read_text(encoding='utf-8')))
    assert spec.single_contact_safe is False
    assert spec.n_sharp_edges[1] >= 1
    assert spec.rock_amp_m[1] > AlpineGenSpec().rock_amp_m[1]
    assert single_contact_hazards(generate_alpine_track(spec, seed=0))
```

- [ ] **Step 2: Run to verify failure**

Run: `uv run --frozen python -m pytest tests/reference/test_alpine_track.py -q -k "registered or frozen or rocky"`
Expected: FAIL with `ImportError: cannot import name 'SEEDED_PRESETS' from 'bike_sim.terrain'`.

- [ ] **Step 3: Register the preset**

In `src/bike_sim/terrain/presets.py`:

1. Add after `from bike_sim.terrain.road import road_broken, road_smooth, road_worn`:

```python
from bike_sim.terrain.alpine import ALPINE_PRESET_SEED, alpine_scree
```

2. Add `"alpine_scree": alpine_scree,` as the last entry of `PRESETS`.
3. After `DEFAULT_ROAD_PRESET = "road_worn"` add:

```python
# Presets that are generated from a seed and accept a seed override but no length override.
SEEDED_PRESETS: Dict[str, Tuple[Callable[..., TrackSpec], int]] = {
    "alpine_scree": (alpine_scree, ALPINE_PRESET_SEED),
}
```

and extend the typing import to `from typing import Callable, Dict, List, Tuple`.

4. Add `"SEEDED_PRESETS",` and `"alpine_scree",` to `__all__`.
5. Extend the module docstring's last paragraph with one sentence:
   `The ``alpine_scree`` preset is likewise generated (:mod:`bike_sim.terrain.alpine`) from a pinned seed.`

- [ ] **Step 4: Export from the package**

In `src/bike_sim/terrain/__init__.py` add imports:

```python
from bike_sim.terrain.alpine import ALPINE_PRESET_SEED, AlpineGenSpec, alpine_scree, generate_alpine_track
from bike_sim.terrain.contact_safety import single_contact_hazards
```

change the presets import to include `SEEDED_PRESETS`, and add to `__all__`:
`"AlpineGenSpec", "generate_alpine_track", "alpine_scree", "ALPINE_PRESET_SEED", "SEEDED_PRESETS", "single_contact_hazards"`.

- [ ] **Step 5: Generate the frozen example**

Run:

```bash
uv run --frozen python -c "from bike_sim.terrain.presets import get_preset; from bike_sim.terrain.trackfile import save_track; save_track(get_preset('alpine_scree'), 'examples/research/alpine_scree.toml')"
```

Open the file and confirm it starts with the two `# Track file for bike-ride.` comment lines, has `surface = "hardpack"`, a `grade_profile = { knots = [...] }` line with ≥ 10 knots, a `surface_sections = [...]` line containing `"scree"`, and `[[obstacles]]` tables of types `rock_garden`, `roughness`, `bump`, `pothole` (`edge = "bowl"`). Do not hand-edit it: the test compares it byte-for-byte with the dump.

- [ ] **Step 6: Write the multi-contact gen-spec**

```toml
# examples/research/alpine_rocky_genspec.toml
# AlpineGenSpec for bike-research --scenario alpine --gen-spec <this file>.
# Multi-contact opt-in: single_contact_safe = false removes the compliant_2d
# tyre's geometry guards, so rock gardens, scree and sharp ledges take the
# sizes a real alpine trail has. Strict compliant_2d runs WILL end in
# multi_support; use a multi-contact tyre backend or --diagnostic-model-limits.
single_contact_safe = false
rock_amp_m = [0.06, 0.12]
rock_corr_m = [0.15, 0.25]
n_rock_gardens = [2, 3]
scree_amp_m = [0.015, 0.030]
scree_corr_m = [0.10, 0.15]
boulder_height_m = [0.08, 0.20]
boulder_len_m = [0.4, 0.9]
water_bar_depth_m = [0.10, 0.18]
water_bar_len_m = [0.6, 1.0]
n_sharp_edges = [1, 2]
sharp_edge_height_m = [0.04, 0.10]
```

- [ ] **Step 7: Run tests**

Run: `uv run --frozen python -m pytest tests/reference/test_alpine_track.py -q`
Expected: all pass.

- [ ] **Step 8: Commit**

```bash
git add src/bike_sim/terrain/presets.py src/bike_sim/terrain/__init__.py examples/research/alpine_scree.toml examples/research/alpine_rocky_genspec.toml tests/reference/test_alpine_track.py
git commit -m "$(cat <<'EOF'
feat(terrain): alpine_scree preset, seeded-preset registry and frozen examples

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```

---

### Task 8: `bike-ride` routing for seeded presets

**Files:**
- Modify: `src/bike_sim/cli/ride.py:354-390` (`resolve_track`), `:400-430` (`track_seed`), `:564-569` (`_list_tracks`), plus its terrain imports near line 47
- Test: `tests/reference/test_alpine_track.py`

**Interfaces:**
- Consumes: `SEEDED_PRESETS`, `ALPINE_PRESET_SEED`.
- Produces: `resolve_track('alpine_scree', seed=None|int, length_m=None)`; `track_seed('alpine_scree', None) == ALPINE_PRESET_SEED`.

- [ ] **Step 1: Write the failing tests (append)**

```python
from bike_sim.cli import ride as ride_cli


def test_ride_resolves_the_seeded_preset():
    assert dump_track(ride_cli.resolve_track('alpine_scree')) == dump_track(alpine_scree())
    assert dump_track(ride_cli.resolve_track('alpine_scree', seed=7)) == dump_track(alpine_scree(seed=7))
    with pytest.raises(ValueError, match='--length'):
        ride_cli.resolve_track('alpine_scree', length_m=150.)
    assert ride_cli.track_seed('alpine_scree', None) == ALPINE_PRESET_SEED
    assert ride_cli.track_seed('alpine_scree', 9) == 9


def test_ride_lists_the_preset_as_generated(capsys):
    assert ride_cli._list_tracks() == 0
    line = next(l for l in capsys.readouterr().out.splitlines() if l.split() and l.split()[0] == 'alpine_scree')
    assert 'generated' in line and '120 m' in line
```

- [ ] **Step 2: Run to verify failure**

Run: `uv run --frozen python -m pytest tests/reference/test_alpine_track.py -q -k ride`
Expected: `test_ride_resolves_the_seeded_preset` FAILS with `ValueError: --seed/--length only apply to generated roads` (on the `seed=7` call); `test_ride_lists_the_preset_as_generated` FAILS on `'generated' in line`.

- [ ] **Step 3: Wire the registry into `ride.py`**

1. Extend the terrain import (around line 47) so `SEEDED_PRESETS` is imported alongside `available_presets`, `get_preset`, `PRESETS`, `ROAD_LEVEL_SPECS` (follow the existing import style of the file — it imports from `bike_sim.terrain`).

2. In `resolve_track`, insert **before** `if name_or_path not in PRESETS:`:

```python
    if name_or_path in SEEDED_PRESETS:
        if length_m is not None:
            raise ValueError(f"--length does not apply to '{name_or_path}': its length is part of the generator "
                             f"spec; use bike-research --scenario alpine --gen-spec for another length")
        build, default_seed = SEEDED_PRESETS[name_or_path]
        return build(seed=default_seed if seed is None else int(seed))
```

Also update the docstring line `seed: Generator seed override; only meaningful for road presets and files with a` → `seed: Generator seed override; only meaningful for road presets, seeded presets and files with a`.

3. In `track_seed`, insert after the `if name_or_path in ROAD_LEVEL_SPECS:` block:

```python
    if name_or_path in SEEDED_PRESETS:
        return int(SEEDED_PRESETS[name_or_path][1])
```

4. In `_list_tracks` change the kind expression to:

```python
        kind = "generated" if name in ROAD_LEVEL_SPECS or name in SEEDED_PRESETS else "authored"
```

- [ ] **Step 4: Run tests and the preview path by hand**

Run: `uv run --frozen python -m pytest tests/reference/test_alpine_track.py -q`
Expected: all pass.

Run: `uv run --frozen bike-ride --track alpine_scree --preview 2>&1 | head -20` (`--preview` "renders the track profile with effective pothole drops and exits", `ride.py:115`; it opens a figure window — close it).
Expected: a profile figure of a 120 m climb with shelves and ramps, no traceback. Also run `uv run --frozen bike-ride --dump-track alpine_scree | head -12` and check the header lines match `examples/research/alpine_scree.toml`.

- [ ] **Step 5: Commit**

```bash
git add src/bike_sim/cli/ride.py tests/reference/test_alpine_track.py
git commit -m "$(cat <<'EOF'
feat(cli): bike-ride accepts seeded presets with --seed

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```

---

### Task 9: `bike-research --scenario alpine`

**Files:**
- Modify: `src/bike_sim/cli/research.py:23`, `:50-52`, `:97-106`
- Test: `tests/reference/test_alpine_track.py`

**Interfaces:**
- Consumes: `AlpineGenSpec.from_dict`, `generate_alpine_track`.
- Produces: `research.build_track(args)` returning `alpine_<seed>` for `--scenario alpine`.

- [ ] **Step 1: Write the failing tests (append)**

```python
from bike_sim.cli import research as research_cli


def _research_args(*extra):
    return research_cli.parser().parse_args(['--scenario', 'alpine', '--seed', '3', *extra])


def test_research_alpine_scenario_builds_the_generator_track(tmp_path):
    track = research_cli.build_track(_research_args())
    assert track.name == 'alpine_3'
    assert dump_track(track) == dump_track(generate_alpine_track(AlpineGenSpec(), seed=3, name='alpine_3'))
    spec_file = tmp_path / 'gen.toml'
    spec_file.write_text('ramp_grade = [0.10, 0.18]\nwall_grade = [0.20, 0.24]\n', encoding='utf-8')
    custom = research_cli.build_track(_research_args('--gen-spec', str(spec_file)))
    assert custom.grade_profile.slope(np.linspace(0., 120., 1201)).max() <= .24 + 1e-12


def test_research_gen_spec_rejects_unknown_keys_and_wrong_scenarios(tmp_path):
    bad = tmp_path / 'bad.toml'
    bad.write_text('rampgrade = [0.1, 0.2]\n', encoding='utf-8')
    with pytest.raises(ValueError, match='unknown alpine generator keys'):
        research_cli.build_track(_research_args('--gen-spec', str(bad)))
    good = tmp_path / 'good.toml'
    good.write_text('ramp_grade = [0.1, 0.2]\n', encoding='utf-8')
    with pytest.raises(ValueError, match='--gen-spec requires'):
        research_cli.build_track(research_cli.parser().parse_args(['--scenario', 'rough_uphill', '--gen-spec', str(good)]))
```

- [ ] **Step 2: Run to verify failure**

Run: `uv run --frozen python -m pytest tests/reference/test_alpine_track.py -q -k research`
Expected: FAIL with `SystemExit: 2` (argparse: `invalid choice: 'alpine'`).

- [ ] **Step 3: Add the scenario**

In `src/bike_sim/cli/research.py`:

1. Add after line 23 (`from bike_sim.terrain.generator import ...`):

```python
from bike_sim.terrain.alpine import AlpineGenSpec, generate_alpine_track
```

2. Replace the two argument definitions:

```python
    p.add_argument('--scenario', choices=RESEARCH_SCENARIOS+('generated', 'alpine'), default='rough_uphill',
        help="'generated' draws a seeded procedural rough climb, 'alpine' a seeded alpine scree climb "
             "(see --gen-spec); the seed is --seed")
    p.add_argument('--gen-spec', type=Path,
        help='TOML of generator ranges: TerrainGenSpec for --scenario generated, AlpineGenSpec for --scenario alpine')
```

3. Replace `build_track`:

```python
def build_track(args):
    """--track-file wins over --scenario so an authored/frozen track is never regenerated."""
    if args.track_file:
        return load_track(args.track_file)
    if args.gen_spec is not None and args.scenario not in ('generated', 'alpine'):
        raise ValueError('--gen-spec requires --scenario generated or alpine')
    if args.scenario == 'generated':
        spec = TerrainGenSpec.from_dict(tomllib.loads(args.gen_spec.read_text())) if args.gen_spec else TerrainGenSpec()
        return generate_track(spec, seed=args.seed, name=f'generated_{args.seed}')
    if args.scenario == 'alpine':
        spec = AlpineGenSpec.from_dict(tomllib.loads(args.gen_spec.read_text())) if args.gen_spec else AlpineGenSpec()
        return generate_alpine_track(spec, seed=args.seed, name=f'alpine_{args.seed}')
    return build_research_track(args.scenario, seed=args.seed)
```

- [ ] **Step 4: Run tests and a headless smoke run**

Run: `uv run --frozen python -m pytest tests/reference/test_alpine_track.py -q`
Expected: all pass.

Run: `uv run --frozen bike-research --scenario alpine --seed 3 --duration 0.5 --physics-config examples/research/viewer_physics_welded.toml --diagnostic-model-limits --out output/alpine-smoke --overwrite`
Expected: exit 0 and an `episode-0000` directory (or the layout the command prints); `--diagnostic-model-limits` is only there so the platform's documented `energy.constraint_work` baseline violation at t=0.005 s does not end the smoke run — the track itself must not produce `multi_support`. Delete `output/alpine-smoke` afterwards; it is not committed.

- [ ] **Step 5: Commit**

```bash
git add src/bike_sim/cli/research.py tests/reference/test_alpine_track.py
git commit -m "$(cat <<'EOF'
feat(cli): bike-research --scenario alpine with AlpineGenSpec --gen-spec

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```

---

### Task 10: Final verification and spec status

**Files:**
- Modify: `docs/superpowers/specs/2026-10-11-alpine-scree-track-design.md` (status line only)

- [ ] **Step 1: Full quick profile**

Run: `bash tools/run_tests.sh quick`
Expected: green; `test_alpine_track.py` and `test_contact_safety.py` appear in `--durations=10` only via the three rolling tests (each ≤ ~15 s).

- [ ] **Step 2: Interactive eyeball check on the native backend (not a gate)**

Run:

```bash
BIKE_NATIVE_BUILD_PATH="$PWD/native/build/release" uv run --frozen --group native bike-ride --backend native --physics-config examples/research/viewer_physics_welded.toml --track alpine_scree --time-scale 1
```

Expected: the viewer opens on a climb with visible shelves and ramps, scree zones reported in the HUD surface channel as `scree`, no `[FIRST FAILURE] … multi_support` banner. If the native build is missing, build it with the two cmake commands in `docs/RIDE.md` first; do not change any native source.

- [ ] **Step 3: Update the spec status**

Change the spec's status line to: `Статус: реализовано в feat/alpine-scree-track; тесты tests/reference/test_alpine_track.py, test_contact_safety.py, test_terrain_placement_regression.py.`

- [ ] **Step 4: Commit**

```bash
git add docs/superpowers/specs/2026-10-11-alpine-scree-track-design.md
git commit -m "$(cat <<'EOF'
docs: mark the alpine scree track design as implemented

Generated with [Devin](https://devin.ai)

Co-Authored-By: Devin <158243242+devin-ai-integration[bot]@users.noreply.github.com>
EOF
)"
```

---

## Self-review against the spec

- §2 decisions → Task 4 (ranges), Task 5 (rhythm, one wall, counter-slopes), Task 6 (scree composite, boulders, gardens, bowls, background, wet, sharp edges opt-in), Task 2 (`scree`), Task 7 (preset, examples), Tasks 8–9 (CLI), Task 1 (placement extraction + byte-identity).
- §4 spike criteria → Task 3 formulas and tests; §4.2(2) window criterion → `concave_span_deg`.
- §6 tests: T1 (Task 6), T2 (Task 6), T3 (Tasks 3, 6), T4 (Task 7 and Task 2), T5 (Tasks 8–9), T6 (Task 1).
- §7 verification → Task 10.
- No native, no RIDE.md, no trackfile/heightfield changes anywhere in the plan.
- Names are consistent: `rounded/check_range/is_free/place/free_intervals`, `_segments/_rhythm/_faceted/_count`, `generate_alpine_track`, `alpine_scree`, `ALPINE_PRESET_SEED`, `SEEDED_PRESETS` as `name -> (builder, default_seed)`.
