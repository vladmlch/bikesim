# Photo-Derived Frame, Rear Shock & Seat Tube Fidelity — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Re-derive the suspension hardpoints and the front-triangle / shock / seat-tube visuals from a calibrated reference photograph, so the rendered bike matches the real Bulls Sonic EVO, while the published geometry table stays bit-for-bit unchanged.

**Architecture:** A committed offline tool extracts pivot targets from the photo and a second offline tool fits hardpoints to those targets under hard kinematic constraints. The fitted constants — not the optimiser — are committed into `bike_geometry.py` and `HorstLinkageSolver` defaults, which are the single source every downstream artifact (MJCF, `coordinates.json`, mass profile) already reads from. Visual work then happens entirely in `export_mujoco.py`.

**Tech Stack:** Python ≥3.10, numpy, mujoco, matplotlib, pytest + unittest. Run everything through `uv` per `AGENTS.md`. `scipy` is used **only** by the offline fitter via `uv run --with scipy`; it must never become a runtime dependency.

**Spec:** `docs/superpowers/specs/2026-08-24-rear-shock-seattube-fidelity-design.md`

## Global Constraints

- Wheel configuration: Mullet — 29" front (radius 372.0 mm) / 27.5" rear (radius 352.0 mm)
- Rear shock: 205 × 65 mm trunnion (`shock_eye_to_eye = 205.0`, `shock_stroke = 65.0`)
- Suspension travel: 180 mm front (`fork_travel`) / 180 mm rear (`rear_wheel_travel`)
- Wheelbase: 1280.55 mm (published as 1281 mm)
- Reach: 480.0 mm · Stack: 646.0 mm
- Head angle: 64.0° · Effective seat angle: 77.0°
- BB drop: 22.5 mm · Chainstay: 447.5 mm
- Photo calibration: origin = BB red cluster at pixel (587.4, 733.3); scale = 1.6193 mm/px; image +x → bike +X, image +y → bike −Z
- Binding kinematic equality: `solve_trajectory(max_travel=180.0).max_stroke == 65.000 ± 0.01`
- Fidelity tolerance: fitted pivots within 8 mm of photo targets (noise floor is ±6 mm)
- Reference photo lives at `docs/reference/bulls_sonic_evo_side.jpg` (copied in Task 1)
- All Python execution via `uv run`; `scipy` only via `uv run --with scipy`

---

### Task 1: Reference Extraction Tool

Extracts the red pivot clusters from the reference photograph, calibrates pixels to millimetres, and emits an auditable JSON of measurement targets.

**Files:**
- Create: `docs/reference/bulls_sonic_evo_side.jpg` (copy of the source photo)
- Create: `tools/__init__.py` (empty)
- Create: `tools/photo_reference.py`
- Create: `docs/reference/bulls_reference_points.json` (generated)
- Test: `tests/test_photo_reference.py`

**Interfaces:**
- Consumes: nothing (first task)
- Produces:
  - `detect_red_clusters(image_path: str | Path) -> list[dict]` — each dict `{"area": int, "px_x": float, "px_y": float}`, sorted by descending `area`
  - `Calibration` dataclass with fields `origin_px: tuple[float, float]`, `mm_per_px: float`, and method `to_mm(px_x: float, px_y: float) -> tuple[float, float]`
  - `calibrate(clusters: list[dict]) -> Calibration`
  - `build_reference(image_path: str | Path) -> dict` — the JSON payload
  - `BB_DROP_CHECK_MM = 22.5`, `BB_DROP_TOLERANCE_MM = 1.0`

- [ ] **Step 1: Copy the reference photo into the repo**

```bash
cd /Users/vladislav.molchanov/Desktop/Projects_my/mujoco_sim_new2
mkdir -p docs/reference tools
cp "/Users/vladislav.molchanov/Desktop/side_001_bike-detail-2x copy 4.jpg" docs/reference/bulls_sonic_evo_side.jpg
touch tools/__init__.py
```

- [ ] **Step 2: Write the failing test**

Create `tests/test_photo_reference.py`:

```python
"""Tests for calibrated pivot extraction from the reference photograph."""

import json
from pathlib import Path

import pytest

from tools.photo_reference import (
    BB_DROP_CHECK_MM,
    BB_DROP_TOLERANCE_MM,
    build_reference,
    calibrate,
    detect_red_clusters,
)

REPO = Path(__file__).resolve().parent.parent
PHOTO = REPO / "docs" / "reference" / "bulls_sonic_evo_side.jpg"


def test_detects_the_ten_pivot_clusters():
    clusters = detect_red_clusters(PHOTO)
    assert len(clusters) == 10, f"expected 10 red clusters, got {len(clusters)}"
    assert clusters[0]["area"] >= clusters[-1]["area"], "clusters must be sorted by area"


def test_calibration_reproduces_bb_drop():
    """The scale is fitted from wheelbase only; BB drop is an independent check."""
    clusters = detect_red_clusters(PHOTO)
    cal = calibrate(clusters)
    assert cal.mm_per_px == pytest.approx(1.6193, abs=0.005)

    ref = build_reference(PHOTO)
    front_axle_z = ref["points_mm"]["front_axle"][1]
    assert front_axle_z == pytest.approx(
        BB_DROP_CHECK_MM, abs=BB_DROP_TOLERANCE_MM
    ), f"calibration self-check failed: front axle Z={front_axle_z}"


def test_reference_payload_contains_every_named_pivot():
    ref = build_reference(PHOTO)
    for name in (
        "P0_candidate", "P2_candidate", "P3", "P4", "P5", "P6", "P7",
        "rear_axle", "front_axle", "bb",
    ):
        assert name in ref["points_mm"], f"missing {name}"

    assert ref["points_mm"]["P3"][0] == pytest.approx(-71.3, abs=2.0)
    assert ref["points_mm"]["P3"][1] == pytest.approx(206.4, abs=2.0)
    assert ref["points_mm"]["P7"][0] == pytest.approx(171.4, abs=2.0)
    assert ref["points_mm"]["P7"][1] == pytest.approx(401.7, abs=2.0)


def test_reference_json_is_committed_and_matches_the_tool():
    path = REPO / "docs" / "reference" / "bulls_reference_points.json"
    assert path.exists(), "run: uv run python -m tools.photo_reference"
    on_disk = json.loads(path.read_text())
    assert on_disk["points_mm"] == build_reference(PHOTO)["points_mm"]
```

- [ ] **Step 3: Run the test to verify it fails**

Run: `uv run pytest tests/test_photo_reference.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'tools.photo_reference'`

- [ ] **Step 4: Write the implementation**

Create `tools/photo_reference.py`:

```python
"""
Calibrated extraction of suspension pivot coordinates from the reference photograph.

The reference product shot highlights every pivot bolt in red. Those clusters are
segmented, then mapped from pixels to millimetres in the MuJoCo bike frame
(+X forward, +Z up, origin at the bottom bracket).

Calibration uses two anchors only: the BB cluster as origin and the two axle
clusters as a known wheelbase. BB drop is deliberately NOT used to fit the scale,
so reproducing it is an independent correctness check.
"""

from __future__ import annotations

import json
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np
from PIL import Image

# Red-bolt segmentation thresholds (tuned against the reference shot).
RED_MIN = 110
RED_OVER_GREEN = 55
RED_OVER_BLUE = 55
MIN_CLUSTER_AREA = 25

WHEELBASE_MM = 1280.55
BB_DROP_CHECK_MM = 22.5
BB_DROP_TOLERANCE_MM = 1.0

DEFAULT_PHOTO = Path(__file__).resolve().parent.parent / "docs" / "reference" / "bulls_sonic_evo_side.jpg"
DEFAULT_OUTPUT = Path(__file__).resolve().parent.parent / "docs" / "reference" / "bulls_reference_points.json"


@dataclass(frozen=True)
class Calibration:
    """Pixel-to-millimetre mapping for the reference photograph."""

    origin_px: Tuple[float, float]
    mm_per_px: float

    def to_mm(self, px_x: float, px_y: float) -> Tuple[float, float]:
        """Maps a pixel coordinate to (X, Z) in mm, BB origin, +X forward, +Z up."""
        x_mm = (px_x - self.origin_px[0]) * self.mm_per_px
        z_mm = (self.origin_px[1] - px_y) * self.mm_per_px
        return float(x_mm), float(z_mm)


def detect_red_clusters(image_path: str | Path) -> List[Dict[str, Any]]:
    """
    Segments red pivot-bolt markers and returns their centroids, largest first.

    Args:
        image_path: Path to the reference photograph.

    Returns:
        List of {"area": int, "px_x": float, "px_y": float}, sorted by descending area.
    """
    rgb = np.asarray(Image.open(image_path).convert("RGB")).astype(int)
    r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
    mask = (r > RED_MIN) & (r - g > RED_OVER_GREEN) & (r - b > RED_OVER_BLUE)

    height, width = mask.shape
    seen = np.zeros_like(mask)
    clusters: List[Dict[str, Any]] = []

    for y in range(height):
        for x in range(width):
            if not mask[y, x] or seen[y, x]:
                continue
            queue = deque([(y, x)])
            seen[y, x] = True
            pixels: List[Tuple[int, int]] = []
            while queue:
                cy, cx = queue.popleft()
                pixels.append((cy, cx))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        ny, nx = cy + dy, cx + dx
                        if 0 <= ny < height and 0 <= nx < width and mask[ny, nx] and not seen[ny, nx]:
                            seen[ny, nx] = True
                            queue.append((ny, nx))
            if len(pixels) >= MIN_CLUSTER_AREA:
                clusters.append(
                    {
                        "area": len(pixels),
                        "px_x": float(np.mean([p[1] for p in pixels])),
                        "px_y": float(np.mean([p[0] for p in pixels])),
                    }
                )

    clusters.sort(key=lambda c: -c["area"])
    return clusters


def calibrate(clusters: List[Dict[str, Any]]) -> Calibration:
    """
    Derives the pixel-to-millimetre mapping from the BB and axle clusters.

    The BB is the lowest cluster near the crank; the axles are the extreme
    left and right clusters. Their separation is the known wheelbase.
    """
    rear = min(clusters, key=lambda c: c["px_x"])
    front = max(clusters, key=lambda c: c["px_x"])
    span_px = float(np.hypot(front["px_x"] - rear["px_x"], front["px_y"] - rear["px_y"]))
    mm_per_px = WHEELBASE_MM / span_px

    # The BB cluster is the one closest to the crank spindle: among clusters
    # between the axles, it is the lowest in the image (largest px_y).
    interior = [c for c in clusters if rear["px_x"] < c["px_x"] < front["px_x"]]
    bb = max(interior, key=lambda c: c["px_y"])

    return Calibration(origin_px=(bb["px_x"], bb["px_y"]), mm_per_px=mm_per_px)


def build_reference(image_path: str | Path = DEFAULT_PHOTO) -> Dict[str, Any]:
    """
    Extracts, calibrates and names every pivot target.

    Returns:
        Payload with calibration metadata and named (X, Z) targets in mm.

    Raises:
        ValueError: If the BB-drop self-check fails.
    """
    clusters = detect_red_clusters(image_path)
    cal = calibrate(clusters)

    measured = [
        {"area": c["area"], "mm": cal.to_mm(c["px_x"], c["px_y"]), "px": (c["px_x"], c["px_y"])}
        for c in clusters
    ]

    rear_axle = min(measured, key=lambda m: m["mm"][0])
    front_axle = max(measured, key=lambda m: m["mm"][0])

    # Name the linkage pivots by their measured position. Ordering is stable
    # because the pivots are well separated in the (X, Z) plane.
    def nearest(x_mm: float, z_mm: float) -> Dict[str, Any]:
        return min(measured, key=lambda m: (m["mm"][0] - x_mm) ** 2 + (m["mm"][1] - z_mm) ** 2)

    points = {
        "bb": (0.0, 0.0),
        "rear_axle": rear_axle["mm"],
        "front_axle": front_axle["mm"],
        "P0_candidate": nearest(-45.0, 45.9)["mm"],
        "P2_candidate": nearest(-400.4, -8.3)["mm"],
        "P3": nearest(-71.3, 206.4)["mm"],
        "P4": nearest(-39.4, 191.8)["mm"],
        "P5": nearest(41.9, 185.0)["mm"],
        "P6": nearest(29.6, 261.6)["mm"],
        "P7": nearest(171.4, 401.7)["mm"],
    }

    front_axle_z = points["front_axle"][1]
    if abs(front_axle_z - BB_DROP_CHECK_MM) > BB_DROP_TOLERANCE_MM:
        raise ValueError(
            f"Calibration self-check failed: front axle Z={front_axle_z:.2f} mm, "
            f"expected BB drop {BB_DROP_CHECK_MM} +/- {BB_DROP_TOLERANCE_MM} mm"
        )

    return {
        "source_image": str(Path(image_path).name),
        "calibration": {
            "origin_px": list(cal.origin_px),
            "mm_per_px": cal.mm_per_px,
            "wheelbase_mm": WHEELBASE_MM,
            "bb_drop_selfcheck_mm": front_axle_z,
        },
        "note": (
            "Pivot targets are MEASURED from red bolt markers. Front-triangle tube "
            "shapes are NOT measurable from this photograph (white-on-white) and are "
            "authored styling elsewhere."
        ),
        "points_mm": {k: [round(v[0], 4), round(v[1], 4)] for k, v in points.items()},
    }


def main() -> None:
    payload = build_reference()
    DEFAULT_OUTPUT.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"Wrote {DEFAULT_OUTPUT}")
    for name, (x, z) in payload["points_mm"].items():
        print(f"  {name:16s} X={x:9.2f}  Z={z:9.2f}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 5: Add Pillow to dev usage and generate the reference JSON**

Run:
```bash
uv run --with pillow python -m tools.photo_reference
```
Expected: prints ten named points; `front_axle` Z ≈ 22.2.

- [ ] **Step 6: Run the tests to verify they pass**

Run: `uv run --with pillow pytest tests/test_photo_reference.py -v`
Expected: 4 passed

- [ ] **Step 7: Commit**

```bash
git add tools/ tests/test_photo_reference.py docs/reference/
git commit -m "feat(reference): calibrated pivot extraction from reference photo"
```

---

### Task 2: Constrained Hardpoint Fitter

Resolves the P0/P2 identification ambiguity and fits hardpoints to the photo targets under the hard kinematic constraints. Offline only — its output is constants, not a runtime dependency.

**Files:**
- Create: `tools/fit_hardpoints.py`
- Create: `docs/reference/fitted_hardpoints.json` (generated)
- Test: `tests/test_fitted_hardpoints.py`

**Interfaces:**
- Consumes: `tools.photo_reference.build_reference`
- Produces:
  - `CHAINSTAY_SHIFT_MM = 21.0`
  - `evaluate(points: dict[str, np.ndarray]) -> dict` — returns `{"max_stroke", "initial_leverage_ratio", "final_leverage_ratio", "progressivity_pct", "max_link_error"}` or raises
  - `fit(targets: dict[str, tuple[float, float]]) -> dict[str, list[float]]` — fitted `{name: [X, Y, Z]}` for P0, P2, P3, P4, P5, P6, P7, P12
  - `docs/reference/fitted_hardpoints.json` with keys `points_mm`, `kinematics`, `residuals_mm`

- [ ] **Step 1: Write the failing test**

Create `tests/test_fitted_hardpoints.py`:

```python
"""Validates the committed fitted hardpoints, not the optimiser that produced them."""

import json
from pathlib import Path

import numpy as np
import pytest

from bike_geometry import BikeSpecs, compute_rear_axle
from linkage_solver import HorstLinkageSolver

REPO = Path(__file__).resolve().parent.parent
FITTED = REPO / "docs" / "reference" / "fitted_hardpoints.json"
REFERENCE = REPO / "docs" / "reference" / "bulls_reference_points.json"

FIDELITY_TOLERANCE_MM = 8.0
CHAINSTAY_SHIFT_MM = 21.0


@pytest.fixture(scope="module")
def fitted():
    assert FITTED.exists(), "run: uv run --with scipy --with pillow python -m tools.fit_hardpoints"
    return json.loads(FITTED.read_text())


def _solver(points):
    specs = BikeSpecs()
    arr = {k: np.array(v, dtype=float) for k, v in points.items()}
    return HorstLinkageSolver(
        specs=specs,
        p0=arr["P0"], p5=arr["P5"], p7=arr["P7"],
        p1_0=compute_rear_axle(specs),
        p2_0=arr["P2"], p3_0=arr["P3"], p4_0=arr["P4"],
        p6_0=arr["P6"], p12_0=arr["P12"],
    )


def test_binding_equality_stroke_is_exactly_65(fitted):
    traj = _solver(fitted["points_mm"]).solve_trajectory(n_points=101, max_travel=180.0)
    assert traj["max_stroke"] == pytest.approx(65.0, abs=0.01)


def test_four_bar_closes(fitted):
    traj = _solver(fitted["points_mm"]).solve_trajectory(n_points=41, max_travel=180.0)
    assert traj["max_link_error"] < 1e-6


def test_leverage_curve_is_progressive_and_sane(fitted):
    traj = _solver(fitted["points_mm"]).solve_trajectory(n_points=41, max_travel=180.0)
    lr = traj["leverage_ratio"]
    assert traj["initial_leverage_ratio"] > traj["final_leverage_ratio"], "must be progressive"
    assert 2.9 <= traj["initial_leverage_ratio"] <= 3.4
    assert 2.1 <= traj["final_leverage_ratio"] <= 2.6
    assert 15.0 <= traj["progressivity_pct"] <= 32.0
    assert np.all(np.diff(lr) < 1e-6), "leverage ratio must decrease monotonically"


def test_pivots_stay_within_fidelity_tolerance_of_the_photo(fitted):
    """P2 is compared against the deliberately shifted target (chainstay spec wins)."""
    reference = json.loads(REFERENCE.read_text())["points_mm"]
    points = fitted["points_mm"]

    comparisons = [
        ("P0", reference["P0_candidate"], 0.0),
        ("P2", reference["P2_candidate"], CHAINSTAY_SHIFT_MM),
        ("P3", reference["P3"], 0.0),
        ("P4", reference["P4"], 0.0),
        ("P5", reference["P5"], 0.0),
        ("P6", reference["P6"], 0.0),
        ("P7", reference["P7"], 0.0),
    ]
    for name, target, shift in comparisons:
        fx, _, fz = points[name]
        dist = float(np.hypot(fx - (target[0] + shift), fz - target[1]))
        assert dist <= FIDELITY_TOLERANCE_MM, f"{name} drifted {dist:.2f} mm from photo target"


def test_shock_eye_to_eye_matches_spec(fitted):
    p6 = np.array(fitted["points_mm"]["P6"])
    p7 = np.array(fitted["points_mm"]["P7"])
    assert float(np.linalg.norm(p7 - p6)) == pytest.approx(205.0, abs=0.5)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run pytest tests/test_fitted_hardpoints.py -v`
Expected: FAIL — every test errors on the missing `docs/reference/fitted_hardpoints.json`

- [ ] **Step 3: Write the fitter**

Create `tools/fit_hardpoints.py`:

```python
"""
Offline constrained fit of suspension hardpoints to photo-measured targets.

Run with:  uv run --with scipy --with pillow python -m tools.fit_hardpoints

scipy is used here ONLY. Its output is a committed JSON of constants; the runtime
library stays numpy-only, and the test suite validates the constants rather than
re-running this optimiser.

Hard constraints (never traded away):
  - solve_trajectory(max_travel=180).max_stroke == 65.000 mm
  - shock eye-to-eye |P7 - P6| == 205.0 mm
  - chainstay 447.5 mm (spec beats the photo's 468.5 mm; the rear group is
    translated forward CHAINSTAY_SHIFT_MM as a rigid body)

Objective: least squares distance from each fitted pivot to its photo target.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Tuple

import numpy as np
from scipy.optimize import minimize

from bike_geometry import BikeSpecs, compute_rear_axle
from linkage_solver import HorstLinkageSolver
from tools.photo_reference import DEFAULT_OUTPUT as REFERENCE_JSON
from tools.photo_reference import build_reference

CHAINSTAY_SHIFT_MM = 21.0
TARGET_STROKE_MM = 65.0
TARGET_E2E_MM = 205.0
OUTPUT = Path(__file__).resolve().parent.parent / "docs" / "reference" / "fitted_hardpoints.json"

# Order of the free parameters packed into the optimisation vector.
FREE = ("P0", "P2", "P3", "P4", "P5", "P6", "P7", "P12")


def _pack(points: Dict[str, np.ndarray]) -> np.ndarray:
    return np.array([v for name in FREE for v in (points[name][0], points[name][2])], dtype=float)


def _unpack(vec: np.ndarray) -> Dict[str, np.ndarray]:
    return {
        name: np.array([vec[2 * i], 0.0, vec[2 * i + 1]], dtype=float)
        for i, name in enumerate(FREE)
    }


def evaluate(points: Dict[str, np.ndarray]) -> Dict[str, float]:
    """Solves the linkage for a candidate hardpoint set."""
    specs = BikeSpecs()
    solver = HorstLinkageSolver(
        specs=specs,
        p0=points["P0"], p5=points["P5"], p7=points["P7"],
        p1_0=compute_rear_axle(specs),
        p2_0=points["P2"], p3_0=points["P3"], p4_0=points["P4"],
        p6_0=points["P6"], p12_0=points["P12"],
    )
    traj = solver.solve_trajectory(n_points=41, max_travel=180.0)
    return {
        "max_stroke": float(traj["max_stroke"]),
        "initial_leverage_ratio": float(traj["initial_leverage_ratio"]),
        "final_leverage_ratio": float(traj["final_leverage_ratio"]),
        "progressivity_pct": float(traj["progressivity_pct"]),
        "max_link_error": float(traj["max_link_error"]),
    }


def _seed(targets: Dict[str, Tuple[float, float]]) -> Dict[str, np.ndarray]:
    """Photo targets, with the rear group shifted forward to honour chainstay 447.5."""
    def pt(name: str, shift: float = 0.0, lift: float = 0.0) -> np.ndarray:
        x, z = targets[name]
        return np.array([x + shift, 0.0, z + lift], dtype=float)

    seed = {
        "P0": pt("P0_candidate"),
        "P2": pt("P2_candidate", shift=CHAINSTAY_SHIFT_MM, lift=10.8),
        "P3": pt("P3"),
        "P4": pt("P4"),
        "P5": pt("P5"),
        "P6": pt("P6"),
        "P7": pt("P7"),
        "P12": np.array([-435.0 + CHAINSTAY_SHIFT_MM, 0.0, 60.0], dtype=float),
    }
    # Place P6 exactly on the P7 line at the specified eye-to-eye.
    direction = seed["P6"] - seed["P7"]
    seed["P6"] = seed["P7"] + TARGET_E2E_MM * direction / np.linalg.norm(direction)
    return seed


def fit(targets: Dict[str, Tuple[float, float]]) -> Dict[str, Any]:
    """Runs the constrained least-squares fit and returns the full payload."""
    seed = _seed(targets)
    goal = {name: seed[name].copy() for name in FREE}

    def cost(vec: np.ndarray) -> float:
        points = _unpack(vec)
        residual = sum(
            float(np.sum((points[name][[0, 2]] - goal[name][[0, 2]]) ** 2)) for name in FREE
        )
        try:
            metrics = evaluate(points)
        except (ValueError, ZeroDivisionError):
            return 1e9
        stroke_penalty = 4.0e4 * (metrics["max_stroke"] - TARGET_STROKE_MM) ** 2
        e2e = float(np.linalg.norm(points["P7"] - points["P6"]))
        e2e_penalty = 4.0e4 * (e2e - TARGET_E2E_MM) ** 2
        return residual + stroke_penalty + e2e_penalty

    result = minimize(cost, _pack(seed), method="Nelder-Mead",
                      options={"maxiter": 40000, "maxfev": 40000, "xatol": 1e-6, "fatol": 1e-9})
    points = _unpack(result.x)
    metrics = evaluate(points)

    residuals = {
        name: float(np.linalg.norm(points[name][[0, 2]] - goal[name][[0, 2]])) for name in FREE
    }
    return {
        "points_mm": {name: [round(float(v), 6) for v in points[name]] for name in FREE},
        "kinematics": {k: round(v, 6) for k, v in metrics.items()},
        "residuals_mm": {k: round(v, 4) for k, v in residuals.items()},
        "chainstay_shift_mm": CHAINSTAY_SHIFT_MM,
    }


def main() -> None:
    targets = json.loads(REFERENCE_JSON.read_text())["points_mm"] if REFERENCE_JSON.exists() \
        else build_reference()["points_mm"]
    payload = fit({k: tuple(v) for k, v in targets.items()})
    OUTPUT.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"Wrote {OUTPUT}")
    print(json.dumps(payload["kinematics"], indent=2))
    print(json.dumps(payload["residuals_mm"], indent=2))


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run the fitter**

Run:
```bash
uv run --with scipy --with pillow python -m tools.fit_hardpoints
```
Expected: `max_stroke` prints as 65.0 within 0.01, `max_link_error` below 1e-9, every residual under 8 mm.

If `max_stroke` misses by more than 0.01, raise the `stroke_penalty` weight from `4.0e4` to `4.0e5` and re-run. If a residual exceeds 8 mm, lower it to `4.0e3` and re-run. Do not hand-edit the output JSON.

- [ ] **Step 5: Run the tests to verify they pass**

Run: `uv run pytest tests/test_fitted_hardpoints.py -v`
Expected: 5 passed

- [ ] **Step 6: Commit**

```bash
git add tools/fit_hardpoints.py tests/test_fitted_hardpoints.py docs/reference/fitted_hardpoints.json
git commit -m "feat(geometry): constrained fit of hardpoints to photo targets"
```

---

### Task 3: Adopt the Fitted Hardpoints

Moves the fitted constants into the two places the whole codebase reads geometry from, regenerates `coordinates.json`, and hardens the kinematics tests with explicit invariant assertions.

**Files:**
- Modify: `bike_geometry.py:269-295` (`get_fixed_frame_points`)
- Modify: `linkage_solver.py:227-248` (solver seed defaults)
- Modify: `tests/test_kinematics.py:138-197` (`test_fixed_frame_points`), `:419-458` (`test_leverage_ratio_progressive`)
- Modify: `coordinates.json` (regenerated)
- Test: `tests/test_kinematics.py` (new `TestPublishedGeometryInvariants`)

**Interfaces:**
- Consumes: `docs/reference/fitted_hardpoints.json` from Task 2
- Produces: updated `get_fixed_frame_points` returning fitted `P0`, `P5`, `P7`; updated `HorstLinkageSolver` defaults for `p2_0`, `p3_0`, `p4_0`, `p6_0`, `p12_0`

- [ ] **Step 1: Write the failing invariant test**

Append to `tests/test_kinematics.py`:

```python
class TestPublishedGeometryInvariants(unittest.TestCase):
    """
    Locks the published geometry table. These values are user-fixed: a future
    hardpoint refit must never move them silently.
    """

    def setUp(self) -> None:
        self.specs = BikeSpecs()
        self.solver = HorstLinkageSolver(self.specs)

    def test_frame_table_is_unchanged(self) -> None:
        self.assertAlmostEqual(self.specs.reach, 480.0, places=6)
        self.assertAlmostEqual(self.specs.stack, 646.0, places=6)
        self.assertAlmostEqual(self.specs.head_angle_deg, 64.0, places=6)
        self.assertAlmostEqual(self.specs.effective_seat_angle_deg, 77.0, places=6)
        self.assertAlmostEqual(self.specs.bb_drop, 22.5, places=6)
        self.assertAlmostEqual(self.specs.wheelbase, 1280.55, places=6)

    def test_wheel_and_shock_hardware_is_unchanged(self) -> None:
        self.assertAlmostEqual(self.specs.front_wheel_radius, 372.0, places=6)
        self.assertAlmostEqual(self.specs.rear_wheel_radius, 352.0, places=6)
        self.assertAlmostEqual(self.specs.shock_eye_to_eye, 205.0, places=6)
        self.assertAlmostEqual(self.specs.shock_stroke, 65.0, places=6)
        self.assertAlmostEqual(self.specs.fork_travel, 180.0, places=6)
        self.assertAlmostEqual(self.specs.rear_wheel_travel, 180.0, places=6)

    def test_chainstay_length_is_447_5(self) -> None:
        st0 = self.solver.solve_state_from_wheel_travel(0.0)
        chainstay = float(np.linalg.norm(st0["P1"] - np.zeros(3)))
        self.assertAlmostEqual(chainstay, 447.5, delta=0.1)

    def test_full_travel_consumes_exactly_the_65mm_stroke(self) -> None:
        traj = self.solver.solve_trajectory(n_points=101, max_travel=180.0)
        self.assertAlmostEqual(traj["max_stroke"], 65.0, delta=0.01)

    def test_shock_eye_to_eye_is_205_at_full_extension(self) -> None:
        st0 = self.solver.solve_state_from_wheel_travel(0.0)
        e2e = float(np.linalg.norm(st0["P7"] - st0["P6"]))
        self.assertAlmostEqual(e2e, 205.0, delta=0.5)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run pytest tests/test_kinematics.py::TestPublishedGeometryInvariants -v`
Expected: `test_chainstay_length_is_447_5` FAILS (current 447.50 passes) and `test_full_travel_consumes_exactly_the_65mm_stroke` FAILS — current `max_stroke` is 65.0464, outside the 0.01 tolerance.

- [ ] **Step 3: Transfer the fitted constants**

Read `docs/reference/fitted_hardpoints.json`, then edit `bike_geometry.py` `get_fixed_frame_points` — replace the `P0`, `P5`, `P7` literals (and their descriptive aliases `main_pivot`, `rocker_frame_pivot`, `shock_upper_mount`) with the fitted values, keeping full precision:

```python
    points = {
        "P0": np.array([<fitted P0 X>, 0.0, <fitted P0 Z>], dtype=float),
        "P5": np.array([<fitted P5 X>, 0.0, <fitted P5 Z>], dtype=float),
        "P7": np.array([<fitted P7 X>, 0.0, <fitted P7 Z>], dtype=float),
        ...
        "main_pivot": np.array([<fitted P0 X>, 0.0, <fitted P0 Z>], dtype=float),
        "rocker_frame_pivot": np.array([<fitted P5 X>, 0.0, <fitted P5 Z>], dtype=float),
        "shock_upper_mount": np.array([<fitted P7 X>, 0.0, <fitted P7 Z>], dtype=float),
```

Then edit `linkage_solver.py:230-248` — replace the `p2_0`, `p3_0`, `p4_0`, `p6_0`, `p12_0` fallback literals with the fitted values. Keep the existing comment above `p6_0` warning that full precision is required for the 1e-6 m MJCF rounding of the `site_P7` loop closure.

- [ ] **Step 4: Run the invariant tests to verify they pass**

Run: `uv run pytest tests/test_kinematics.py::TestPublishedGeometryInvariants -v`
Expected: 5 passed

- [ ] **Step 5: Regenerate coordinates.json and read the new curve**

Run:
```bash
uv run python main.py --export-json
uv run python -c "
from bike_geometry import BikeSpecs
from linkage_solver import HorstLinkageSolver
t = HorstLinkageSolver(BikeSpecs()).solve_trajectory(n_points=101, max_travel=180.0)
print('LR', round(t['initial_leverage_ratio'],4), '->', round(t['final_leverage_ratio'],4))
print('progressivity', round(t['progressivity_pct'],3))
print('max_stroke', round(t['max_stroke'],4))
"
```
Note the three printed numbers — the next step pins them.

- [ ] **Step 6: Update the existing kinematics expectations**

In `tests/test_kinematics.py`, update `test_leverage_ratio_progressive` (line 419) and `test_fixed_frame_points` (line 138) so their hardcoded expectations match the values printed in Step 5. Pin the leverage figures as a characterization assertion with a ±0.02 window, e.g.:

```python
        self.assertAlmostEqual(traj["initial_leverage_ratio"], <printed LR init>, delta=0.02)
        self.assertAlmostEqual(traj["final_leverage_ratio"], <printed LR final>, delta=0.02)
        self.assertAlmostEqual(traj["progressivity_pct"], <printed progressivity>, delta=0.10)
```

- [ ] **Step 7: Run the whole kinematics suite**

Run: `uv run pytest tests/test_kinematics.py -v`
Expected: all pass. If `test_json_export_structure` fails on a stale value, regenerate with Step 5 first.

- [ ] **Step 8: Commit**

```bash
git add bike_geometry.py linkage_solver.py coordinates.json tests/test_kinematics.py
git commit -m "feat(geometry): adopt photo-fitted hardpoints, lock published geometry table"
```

---

### Task 4: Continuous Seat Tube and Frame Casting

Removes the split-strut cage and replaces the linkage-region structure with a continuous seat tube dying into a casting, as the reference silhouette shows.

**Files:**
- Modify: `export_mujoco.py:755-822` (split seat tube block)
- Test: `tests/test_frame_visuals.py` (create)

**Interfaces:**
- Consumes: fitted `P5`, `P10` from Task 3
- Produces: geoms named `geom_seattube`, `geom_frame_yoke`; removes `geom_seattube_upper`, `geom_seattube_strut_l_top`, `geom_seattube_strut_l_bot`, `geom_seattube_strut_r_top`, `geom_seattube_strut_r_bot`

- [ ] **Step 1: Write the failing test**

Create `tests/test_frame_visuals.py`:

```python
"""Structural assertions on the generated MJCF frame visuals."""

import xml.etree.ElementTree as ET

import mujoco
import pytest

from export_mujoco import generate_mujoco_xml

SEAT_TUBE_TERMINATION_Z_M = 0.258


@pytest.fixture(scope="module")
def root():
    return ET.fromstring(generate_mujoco_xml(mode="stand"))


def _geom_names(root):
    return {g.get("name") for g in root.iter("geom")}


def test_split_seat_tube_cage_is_gone(root):
    names = _geom_names(root)
    for removed in (
        "geom_seattube_strut_l_top", "geom_seattube_strut_l_bot",
        "geom_seattube_strut_r_top", "geom_seattube_strut_r_bot",
        "geom_seattube_upper",
    ):
        assert removed not in names, f"{removed} should have been removed"


def test_seat_tube_is_continuous_and_terminates_at_the_casting(root):
    names = _geom_names(root)
    assert "geom_seattube" in names
    assert "geom_frame_yoke" in names

    seattube = next(g for g in root.iter("geom") if g.get("name") == "geom_seattube")
    coords = [float(v) for v in seattube.get("fromto").split()]
    lower_z = min(coords[2], coords[5])
    assert lower_z == pytest.approx(SEAT_TUBE_TERMINATION_Z_M, abs=0.015)


def test_model_still_compiles(root):
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode="stand"))
    assert model.ngeom > 0
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run pytest tests/test_frame_visuals.py -v`
Expected: FAIL — `geom_seattube_upper` still present, `geom_seattube` and `geom_frame_yoke` missing.

The builder is `generate_mujoco_xml(specs=None, solver=None, mode="standard", mass_specs=None, include_rider=True, bump_scale=1.0, pit_scale=1.0) -> str` at `export_mujoco.py:65`. It returns the MJCF string directly, so the tests parse its output with `ET.fromstring` and need no temporary files.

- [ ] **Step 3: Replace the split seat tube block**

In `export_mujoco.py`, delete lines 755-822 (the `# Split Seat Tube (Shock Tunnel Architecture)` block through `geom_seattube_strut_r_bot`) and substitute:

```python
    # Seat Tube (continuous) + Frame Casting
    #
    # Reference measurement: the real seat tube runs straight from the collar and
    # dies into the frame casting at Z ~= 258 mm, where the silhouette width steps
    # from 29 mm to 68 mm. It never reaches the BB, which is why no split-strut
    # cage is needed to clear the rocker. Shapes below the tube are AUTHORED
    # STYLING - the white-on-white front triangle is not measurable from the photo.
    seattube_end = np.array([-0.030, 0.0, 0.258])
    ET.SubElement(
        frame,
        "geom",
        {
            "name": "geom_seattube",
            "type": "capsule",
            "fromto": _format_fromto(P10, seattube_end),
            "size": "0.020",
            "mass": "0.45",
            "material": "mat_frame",
        },
    )
    # Motor / shock-tunnel casting: carries the rocker pivot P5 and spans forward
    # of the linkage. Sized from the measured silhouette (X -35..+33 at Z = 255).
    ET.SubElement(
        frame,
        "geom",
        {
            "name": "geom_frame_yoke",
            "type": "box",
            "pos": f"{(seattube_end[0] + P5[0]) / 2.0:.6f} 0 {(seattube_end[2] + 0.0) / 2.0:.6f}",
            "size": "0.075 0.036 0.130",
            "mass": "0.55",
            "material": "mat_frame",
        },
    )
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `uv run pytest tests/test_frame_visuals.py -v`
Expected: 3 passed

- [ ] **Step 5: Commit**

```bash
git add export_mujoco.py tests/test_frame_visuals.py
git commit -m "feat(visual): continuous seat tube into frame casting, drop strut cage"
```

---

### Task 5: Trunnion Mount and Damper Proportions

Removes the floating shock bracket, derives the damper proportions from spec instead of literals, and pulls the piggyback back into the frame plane.

**Files:**
- Modify: `export_mujoco.py:848-860` (`geom_shock_tab`)
- Modify: `export_mujoco.py:1765-1776` (damper length literals)
- Modify: `export_mujoco.py:1891-1919` (piggyback offsets)
- Test: `tests/test_frame_visuals.py` (extend)

**Interfaces:**
- Consumes: `specs.shock_eye_to_eye`, `specs.shock_stroke`
- Produces: geom `geom_frame_junction`; removes `geom_shock_tab`; `body_can_len` and `trunnion_overhang` become derived values

- [ ] **Step 1: Write the failing test**

Append to `tests/test_frame_visuals.py`:

```python
BOTTOM_OUT_CLEARANCE_MM = 2.0


def test_floating_shock_bracket_is_replaced_by_frame_material(root):
    names = _geom_names(root)
    assert "geom_shock_tab" not in names, "the thin floating bracket must be gone"
    assert "geom_frame_junction" in names


def test_air_can_clears_the_lower_eyelet_at_bottom_out():
    """
    At full bottom-out the eye-to-eye collapses to e2e - stroke. The can is
    measured back from P7, so it must stay shorter than that by the clearance.
    """
    from bike_geometry import BikeSpecs

    specs = BikeSpecs()
    collapsed = specs.shock_eye_to_eye - specs.shock_stroke
    root_el = ET.fromstring(generate_mujoco_xml(mode="stand"))
    can = next(g for g in root_el.iter("geom") if g.get("name") == "geom_shock_body")
    coords = [float(v) for v in can.get("fromto").split()]
    can_len_mm = 1000.0 * (
        (coords[3] - coords[0]) ** 2 + (coords[4] - coords[1]) ** 2 + (coords[5] - coords[2]) ** 2
    ) ** 0.5
    assert can_len_mm <= collapsed - BOTTOM_OUT_CLEARANCE_MM + 1e-6
    assert can_len_mm == pytest.approx(138.0, abs=0.5)


def test_piggyback_sits_in_the_frame_plane(root):
    piggy = next(g for g in root.iter("geom") if g.get("name") == "geom_shock_piggyback")
    coords = [float(v) for v in piggy.get("fromto").split()]
    lateral = max(abs(coords[1]), abs(coords[4]))
    assert lateral <= 0.012, f"piggyback is {lateral*1000:.0f} mm off-plane; expected <= 12 mm"
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run pytest tests/test_frame_visuals.py -v`
Expected: FAIL — `geom_shock_tab` still present, can length is 133 mm, piggyback lateral offset is 0.038 m.

- [ ] **Step 3: Replace the shock tab with frame material**

In `export_mujoco.py`, delete the `geom_shock_tab` block (lines 848-860) and substitute:

```python
    # Frame casting at the top-tube / seat-tube / down-tube junction. The trunnion
    # at P7 bolts into this material - the real bike has no standoff bracket.
    p_junction_top = P10 + (P8 - P10) * 0.52
    ET.SubElement(
        frame,
        "geom",
        {
            "name": "geom_frame_junction",
            "type": "capsule",
            "fromto": _format_fromto(p_junction_top, P7),
            "size": "0.042",
            "mass": "0.30",
            "material": "mat_frame",
        },
    )
```

- [ ] **Step 4: Derive the damper proportions from spec**

Replace the literals at `export_mujoco.py:1768-1776`:

```python
    shaft_len = 0.145  # m (145 mm shaft; tip hides inside the trunnion overhang at bottom-out)
    shaft_tip_rel_p6 = u_shock * shaft_len
    # Air can length is DERIVED, not chosen: at full bottom-out the eye-to-eye
    # collapses to (e2e - stroke), so a can measured back from P7 must stay
    # shorter than that or it drives through the lower eyelet. 2 mm clearance.
    bottom_out_clearance_m = 0.002
    body_can_len = (specs.shock_eye_to_eye - specs.shock_stroke) / 1000.0 - bottom_out_clearance_m
    body_can_end_rel_p7 = shock_slide_axis * body_can_len
    # Trunnion overhang measured from the reference photo: ~12 mm of damper body
    # continues past the mounting bolt axis, carrying the reservoir cap.
    trunnion_overhang = 0.014
    trunnion_cap_rel_p7 = -shock_slide_axis * trunnion_overhang
    trunnion_half_width = 0.027  # m (54 mm across the two trunnion bosses)
```

- [ ] **Step 5: Move the piggyback into the frame plane**

Replace `export_mujoco.py:1892-1893` and the bridge offsets at `:1906-1907`:

```python
    # Reservoir rides on the damper body in the frame plane. The perpendicular
    # direction is the shock axis rotated 90 degrees in XZ, pointing up-rearward.
    perp = np.array([-shock_slide_axis[2], 0.0, shock_slide_axis[0]])
    piggy_offset = perp * 0.034 + np.array([0.0, 0.008, 0.0])
    piggy_start = (shock_slide_axis * 0.018) + piggy_offset
    piggy_end = (shock_slide_axis * 0.108) + piggy_offset
```

and

```python
    piggy_bridge_1 = shock_slide_axis * 0.030
    piggy_bridge_2 = (shock_slide_axis * 0.030) + piggy_offset
```

- [ ] **Step 6: Run the tests to verify they pass**

Run: `uv run pytest tests/test_frame_visuals.py -v`
Expected: 6 passed

- [ ] **Step 7: Commit**

```bash
git add export_mujoco.py tests/test_frame_visuals.py
git commit -m "feat(visual): trunnion into frame, spec-derived damper proportions"
```

---

### Task 6: Battery-Box Down Tube

Replaces the thin down-tube capsule with the measured battery-box envelope.

**Files:**
- Modify: `export_mujoco.py:706-717` (`geom_downtube`)
- Test: `tests/test_frame_visuals.py` (extend)

**Interfaces:**
- Consumes: `BB`, `P_HT_bot` from the existing hardpoint block
- Produces: `geom_downtube` changes `type` from `capsule` to `box`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_frame_visuals.py`:

```python
def test_down_tube_is_a_battery_box_not_a_thin_tube(root):
    downtube = next(g for g in root.iter("geom") if g.get("name") == "geom_downtube")
    assert downtube.get("type") == "box"
    half_extents = [float(v) for v in downtube.get("size").split()]
    # Measured envelope is ~116 mm across at Z = 255 mm, so the largest cross
    # section half-extent must be well above the old 28 mm capsule radius.
    assert max(half_extents) >= 0.055
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run pytest tests/test_frame_visuals.py::test_down_tube_is_a_battery_box_not_a_thin_tube -v`
Expected: FAIL — `assert 'capsule' == 'box'`

- [ ] **Step 3: Replace the down tube geom**

Replace the `geom_downtube` block at `export_mujoco.py:706-717`:

```python
    # Down tube is a battery box on this e-bike, not a tube. Envelope measured
    # from the reference silhouette: X 150..266 mm at Z = 255, X 208..305 at Z = 323.
    # AUTHORED STYLING - the exact casting shape is not recoverable from the photo.
    downtube_mid = (BB + P_HT_bot) / 2.0
    downtube_vec = P_HT_bot - BB
    downtube_angle = float(np.arctan2(downtube_vec[2], downtube_vec[0]))
    ET.SubElement(
        frame,
        "geom",
        {
            "name": "geom_downtube",
            "type": "box",
            "pos": _format_vec(downtube_mid),
            "size": f"{float(np.linalg.norm(downtube_vec)) / 2.0:.6f} 0.038 0.058",
            "euler": f"0 {-downtube_angle:.6f} 0",
            "mass": "0.60",
            "material": "mat_frame",
        },
    )
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `uv run pytest tests/test_frame_visuals.py -v`
Expected: 7 passed

- [ ] **Step 5: Commit**

```bash
git add export_mujoco.py tests/test_frame_visuals.py
git commit -m "feat(visual): battery-box down tube from measured envelope"
```

---

### Task 7: Debug Livery Toggle

Puts the pivot markers and diagnostic materials behind a flag so hero renders show the bike, not the debug rig.

**Files:**
- Modify: `export_mujoco.py:934-945` (frame markers), `:1652-1665` (rocker markers), `:1758-1759` (yoke markers)
- Modify: `export_mujoco.py:65-72` (`generate_mujoco_xml` signature)
- Modify: `run_playground.py:300-307` (the `load_model` call into `generate_mujoco_xml`)
- Test: `tests/test_frame_visuals.py` (extend)

**Interfaces:**
- Consumes: nothing new
- Produces: MJCF builder gains keyword argument `debug_markers: bool = False`; every `marker_*` geom and every diagnostic `site` is emitted only when it is `True`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_frame_visuals.py`:

```python
def test_markers_are_absent_by_default():
    root_el = ET.fromstring(generate_mujoco_xml(mode="stand"))
    markers = [g.get("name") for g in root_el.iter("geom")
               if (g.get("name") or "").startswith("marker_")]
    assert markers == [], f"hero render must have no debug markers, found {markers}"


def test_markers_return_when_requested():
    root_el = ET.fromstring(generate_mujoco_xml(mode="stand", debug_markers=True))
    markers = {g.get("name") for g in root_el.iter("geom")
               if (g.get("name") or "").startswith("marker_")}
    assert "marker_p5_l" in markers
    assert "marker_p7" in markers


def test_equality_constraint_sites_survive_marker_removal():
    """site_P3_ss / site_P3_rocker / site_P7_shock drive loop closure - never optional."""
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode="stand"))
    site_names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i)
                  for i in range(model.nsite)}
    for required in ("site_P3_ss", "site_P3_rocker", "site_P7_shock", "site_P6_yoke"):
        assert required in site_names
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run pytest tests/test_frame_visuals.py -v`
Expected: FAIL — markers present by default; `debug_markers` is an unexpected keyword.

- [ ] **Step 3: Thread the flag through the builder**

Add `debug_markers: bool = False` to the MJCF builder signature and to any wrapper that forwards to it. Guard every `marker_*` geom with it, for example at `export_mujoco.py:934`:

```python
    # Visual Pivot Markers (debug only - matching engineering sketch)
    if debug_markers:
        ET.SubElement(frame, "geom", {"name": "marker_bb", ...})
        ...
```

Apply the same guard to the rocker markers (`marker_p3_roc_l/r`, `marker_p4_l/r`) and the yoke markers (`marker_p6_l/r`). **Do not guard** `site_P3_ss`, `site_P3_rocker`, `site_P6_yoke` or `site_P7_shock` — the `<equality><connect>` elements reference them and the model will not compile without them.

- [ ] **Step 4: Expose it at the playground call site**

In `run_playground.py`, add `self.debug_markers = False` beside the other display flags in `__init__` (near `self.show_telemetry` at line 288), then forward it in `load_model` at line 300:

```python
        xml_str = generate_mujoco_xml(
            specs=self.specs,
            solver=self.solver,
            mode=mode,
            include_rider=self.include_rider,
            bump_scale=self.bump_scale,
            pit_scale=self.pit_scale,
            debug_markers=self.debug_markers,
        )
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `uv run pytest tests/test_frame_visuals.py tests/test_playground.py -v`
Expected: all pass

- [ ] **Step 6: Commit**

```bash
git add export_mujoco.py run_playground.py tests/test_frame_visuals.py
git commit -m "feat(visual): gate debug markers behind a flag"
```

---

### Task 8: Restore 30 % Rear Sag on the New Leverage Curve

The refit changed the leverage ratio, so the air spring no longer produces the specified sag.

**Files:**
- Modify: `bike_geometry.py:79-84` (rear shock damper settings) or `air_spring.py` pressure default, whichever holds the rear spring rate
- Test: `tests/test_air_spring.py` (extend)

**Interfaces:**
- Consumes: fitted leverage curve from Task 3
- Produces: retuned rear spring pressure constant

- [ ] **Step 1: Write the failing test**

Append to `tests/test_air_spring.py`:

```python
def test_rear_sag_is_30_percent_on_the_fitted_leverage_curve():
    """
    Sag is a design target, not an emergent value. The photo refit moved the
    leverage curve, so the spring rate must be retuned to keep 30 percent.
    """
    from bike_geometry import BikeSpecs
    from linkage_solver import HorstLinkageSolver

    specs = BikeSpecs()
    solver = HorstLinkageSolver(specs)
    target_sag_mm = 0.30 * specs.rear_wheel_travel

    sag_state = solver.solve_state_from_wheel_travel(target_sag_mm)
    assert sag_state["shock_stroke"] > 0.0
    # 30 percent of travel must land near 30 percent of stroke for a curve this
    # progressive; a gross mismatch means the spring will not sit where specified.
    sag_fraction_of_stroke = sag_state["shock_stroke"] / specs.shock_stroke
    assert 0.24 <= sag_fraction_of_stroke <= 0.36, (
        f"30% travel consumes {sag_fraction_of_stroke:.1%} of stroke"
    )
```

- [ ] **Step 2: Run the test**

Run: `uv run pytest tests/test_air_spring.py -v`
Expected: PASS or FAIL depending on the fitted curve. If it PASSES, no retune is required — record that in the commit message and skip to Step 4.

- [ ] **Step 3: Retune if it failed**

Locate the rear spring pressure used by the sag computation and adjust it until the test passes. Change one constant, re-run, repeat. Do not weaken the assertion window.

- [ ] **Step 4: Run the full air-spring and mass suites**

Run: `uv run pytest tests/test_air_spring.py tests/test_mass_distribution.py -v`
Expected: all pass

- [ ] **Step 5: Commit**

```bash
git add bike_geometry.py air_spring.py tests/test_air_spring.py
git commit -m "fix(suspension): restore 30% rear sag on the refitted leverage curve"
```

---

### Task 9: Side-by-Side Render Comparison

Makes similarity judgeable on one frame instead of from memory.

**Files:**
- Create: `tools/render_comparison.py`
- Create: `docs/reference/comparison.png` (generated)

**Interfaces:**
- Consumes: the MJCF builder from Tasks 4-7, `docs/reference/bulls_sonic_evo_side.jpg`
- Produces: `render_comparison(output_path: str | Path) -> Path`

- [ ] **Step 1: Write the tool**

Create `tools/render_comparison.py`:

```python
"""
Renders the simulated bike from the reference viewpoint and pastes it beside
the reference photograph, so fidelity is judged on a single frame.

Run with:  uv run --with pillow python -m tools.render_comparison
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from export_mujoco import generate_mujoco_xml

REPO = Path(__file__).resolve().parent.parent
PHOTO = REPO / "docs" / "reference" / "bulls_sonic_evo_side.jpg"
OUTPUT = REPO / "docs" / "reference" / "comparison.png"

WIDTH, HEIGHT = 1400, 1050


def render_comparison(output_path: str | Path = OUTPUT) -> Path:
    """Renders the model side-on and writes a side-by-side PNG against the photo."""
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode="stand"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.azimuth = 90.0      # side-on, drive side, matching the product shot
    camera.elevation = 0.0
    camera.distance = 2.4
    camera.lookat[:] = np.array([0.15, 0.0, 0.30])

    with mujoco.Renderer(model, height=HEIGHT, width=WIDTH) as renderer:
        renderer.update_scene(data, camera=camera)
        rendered = Image.fromarray(renderer.render())

    reference = Image.open(PHOTO).convert("RGB").resize((WIDTH, HEIGHT))
    canvas = Image.new("RGB", (WIDTH * 2, HEIGHT), "white")
    canvas.paste(reference, (0, 0))
    canvas.paste(rendered, (WIDTH, 0))

    output = Path(output_path)
    canvas.save(output)
    return output


if __name__ == "__main__":
    print(f"Wrote {render_comparison()}")
```

- [ ] **Step 2: Render and inspect**

Run:
```bash
uv run --with pillow python -m tools.render_comparison
```
Expected: `docs/reference/comparison.png` written. Open it and check the seat tube runs unbroken into the casting, the trunnion sits in frame material, and no debug markers appear.

- [ ] **Step 3: Commit**

```bash
git add tools/render_comparison.py docs/reference/comparison.png
git commit -m "feat(tooling): side-by-side render comparison against reference photo"
```

---

### Task 10: Full Suite and Documentation

**Files:**
- Modify: `README.md` (geometry and kinematics tables)

**Interfaces:**
- Consumes: everything above
- Produces: documentation consistent with the shipped numbers

- [ ] **Step 1: Run the entire test suite**

Run: `uv run --with pillow pytest tests/ -v`
Expected: all pass. Fix any failure before continuing — do not proceed on a red suite.

- [ ] **Step 2: Refresh the README numbers**

Update every geometry and kinematics table in `README.md` to the values now in `coordinates.json`: hardpoint coordinates, initial/final leverage ratio, progressivity, transmission angles, link lengths. Add a short subsection recording that hardpoints are photo-derived, naming `tools/photo_reference.py` and `docs/reference/bulls_reference_points.json`, and stating plainly that front-triangle tube shapes are authored styling rather than measurements.

- [ ] **Step 3: Verify the README claims against the artifacts**

Run:
```bash
uv run python -c "
import json
d = json.load(open('coordinates.json'))
print(json.dumps(d['geometry_summary'], indent=2))
"
```
Cross-check each printed figure against what you wrote.

- [ ] **Step 4: Commit**

```bash
git add README.md
git commit -m "docs: refresh geometry and kinematics tables for photo-derived frame"
```

---

## Self-Review

**Spec coverage:**

| Spec section | Task |
|---|---|
| §2 invariants | Task 3 (`TestPublishedGeometryInvariants`) |
| §3 measurement method | Task 1 |
| §4 reference extraction + ambiguity | Tasks 1, 2 |
| §5 constrained refit + chainstay shift | Tasks 2, 3 |
| §6 seat tube / casting | Task 4 |
| §6 trunnion, damper, piggyback | Task 5 |
| §6 down tube | Task 6 |
| §6 debug livery | Task 7 |
| §7 verification, sag, render, README | Tasks 3, 5, 8, 9, 10 |
| §8.1 styling labelled in code | Tasks 4, 6, 10 |

No spec requirement is unassigned.

**Known execution risks:**

1. Task 2 Step 4 may need penalty reweighting; the exact adjustment and its direction are given inline.
2. Task 8 may be a no-op if sag already lands in the window — Step 2 says so explicitly rather than forcing a change.
3. Task 3 Steps 3 and 6 deliberately carry values that do not exist until Task 2's fitter has run. Both steps name the exact file and command to read them from; they are data dependencies, not unfilled blanks.

**Verified against the codebase while writing this plan:** the MJCF builder is `generate_mujoco_xml` at `export_mujoco.py:65` (not a `build_*` name); `run_playground.py` calls it from `load_model` at line 300 (not from the viewer setup at 1296); the JSON regeneration flag is `main.py --export-json`. Task 7's marker guard must skip `site_P3_ss`, `site_P3_rocker`, `site_P6_yoke` and `site_P7_shock`, which `<equality><connect>` depends on.
