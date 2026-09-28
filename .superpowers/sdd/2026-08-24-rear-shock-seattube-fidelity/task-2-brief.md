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

