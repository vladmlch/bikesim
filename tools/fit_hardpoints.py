"""
Offline constrained fit of suspension hardpoints to photo-measured targets.

Run with:  uv run --with scipy --with pillow python -m tools.fit_hardpoints

scipy is used here ONLY. Its output is a committed JSON of constants; the runtime
library stays numpy-only, and the test suite validates the constants rather than
re-running this optimiser.

Hard constraints (never traded away):
  - shock stroke over 0..180 mm of rear travel == 65.000 mm
  - shock eye-to-eye |P7 - P6| == 205.0 mm, with P6 COLLINEAR on P4 -> P7
  - chainstay 447.5 mm (spec beats the photo's 468.5 mm; the rear group is
    translated forward CHAINSTAY_SHIFT_MM as a rigid body)

Objective: least squares distance from each fitted pivot to its photo target.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Dict, Tuple

import numpy as np
from scipy.optimize import minimize

from bike_sim.geometry.hardpoints import compute_rear_axle
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.kinematics.solver import HorstLinkageSolver
from tools.photo_reference import DEFAULT_OUTPUT as REFERENCE_JSON
from tools.photo_reference import build_reference

CHAINSTAY_SHIFT_MM = 21.0
TARGET_STROKE_MM = 65.0
TARGET_E2E_MM = 205.0
MAX_TRAVEL_MM = 180.0
OUTPUT = Path(__file__).resolve().parent.parent / "docs" / "reference" / "fitted_hardpoints.json"

# Free parameters packed into the optimisation vector.
#
# P6 is deliberately ABSENT. The solver re-projects the shock eyelet onto the
# P4 -> P7 line at every step, so a P6 that is not collinear with P4 -> P7 is not
# a geometry the model can represent: the triangle inequality then makes
# shock_stroke non-zero at zero travel, and `max_stroke` (defined as
# shock_stroke[-1], never minus shock_stroke[0]) silently over-reports the real
# excursion. Fitting P6 freely therefore manufactures phantom stroke. Instead P6
# follows the convention the solver itself documents:
#     P6 = P7 - 205 * unit(P7 - P4)
# which pins eye-to-eye exactly and makes stroke at rest identically zero. This
# also removes the need for any eye-to-eye penalty term.
FREE = ("P0", "P2", "P3", "P4", "P5", "P7", "P12")

# Every photo-measured pivot the fit is scored against, including the derived P6
# -- whose reported residual is thus the residual of the pivot the model really
# realises, not of a stored constant the solver would ignore.
SCORED = ("P0", "P2", "P3", "P4", "P5", "P6", "P7", "P12")

# Penalty weight on the one remaining hard equality, quoted per mm^2 of
# violation against a pivot residual objective that is also in mm^2.
STROKE_PENALTY_WEIGHT = 4.0e4

# Nelder-Mead's default initial simplex perturbs each coordinate by 5% of its
# own value, which for hardpoints spanning -400 mm to +400 mm means steps from
# 0.4 mm to 20 mm. A geometry fit wants one uniform length scale, so the simplex
# is built explicitly with this step instead.
SIMPLEX_STEP_MM = 1.5

# Nelder-Mead in double-digit dimensions routinely stalls on a degenerate
# simplex well short of the optimum; restarting from the incumbent with a fresh
# simplex is the standard remedy. Ruling R2's 8000-evaluation cap is lifted now
# that _FitSolver has made evaluations ~100x cheaper, so the budget is set by
# what converges rather than by what is affordable.
MAX_EVALUATIONS_PER_RESTART = 20000
MAX_RESTARTS = 25

# The seed hardpoints already produce a valid four-bar, so an infeasible
# candidate is genuinely off the manifold. A large finite cost (rather than inf)
# keeps Nelder-Mead's shrink steps well defined.
INFEASIBLE_COST = 1e9


class _FitSolver(HorstLinkageSolver):
    """
    Fit-loop solver that skips the constructor's travel <-> stroke lookup table.

    `HorstLinkageSolver.__init__` spends 101 Brent root-finds precomputing that
    table purely so `solve_state_from_shock_stroke` can invert it in O(1). The
    objective here only ever goes the forward direction
    (`solve_state_from_wheel_travel`), so those 101 root-finds are the entire
    per-evaluation cost and none of them is ever read -- skipping them makes an
    evaluation ~100x cheaper without touching any runtime module.

    The table is left filled with zeros, so `solve_state_from_shock_stroke`
    raises rather than silently interpolating garbage.
    """

    def __init__(self, **kwargs: Any) -> None:
        self._skip_table = True
        super().__init__(**kwargs)
        self._skip_table = False

    def solve_state_from_wheel_travel(self, target_travel_z: float) -> Dict[str, Any]:
        if getattr(self, "_skip_table", False):
            # Called only from the base constructor's table-building loop.
            return {"shock_stroke": 0.0}
        return super().solve_state_from_wheel_travel(target_travel_z)

    def solve_state_from_shock_stroke(self, shock_stroke_mm: float) -> Dict[str, Any]:
        raise NotImplementedError(
            "_FitSolver skips the travel<->stroke table; use solve_state_from_wheel_travel"
        )


def _derive_p6(p4: np.ndarray, p7: np.ndarray) -> np.ndarray:
    """Shock lower eyelet: on the P4 -> P7 line, exactly TARGET_E2E_MM below P7."""
    direction = p7 - p4
    return p7 - TARGET_E2E_MM * direction / np.linalg.norm(direction)


def _pack(points: Dict[str, np.ndarray]) -> np.ndarray:
    return np.array([v for name in FREE for v in (points[name][0], points[name][2])], dtype=float)


def _unpack(vec: np.ndarray) -> Dict[str, np.ndarray]:
    """Unpacks the free parameters and appends the derived P6."""
    points = {
        name: np.array([vec[2 * i], 0.0, vec[2 * i + 1]], dtype=float)
        for i, name in enumerate(FREE)
    }
    points["P6"] = _derive_p6(points["P4"], points["P7"])
    return points


def _build_solver(points: Dict[str, np.ndarray], fast: bool = False) -> HorstLinkageSolver:
    specs = BikeSpecs()
    factory = _FitSolver if fast else HorstLinkageSolver
    return factory(
        specs=specs,
        p0=points["P0"], p5=points["P5"], p7=points["P7"],
        p1_0=compute_rear_axle(specs),
        p2_0=points["P2"], p3_0=points["P3"], p4_0=points["P4"],
        p6_0=points["P6"], p12_0=points["P12"],
    )


def _stroke_excursion(points: Dict[str, np.ndarray]) -> float:
    """
    Shock stroke consumed over 0..MAX_TRAVEL_MM of rear wheel travel.

    Ruling R6: the fit objective reads only this one number, so it is solved
    directly instead of building a whole trajectory whose other states are
    discarded. The stroke at zero travel is subtracted explicitly rather than
    assumed zero -- assuming it is what let phantom stroke through before.
    """
    solver = _build_solver(points, fast=True)
    at_rest = solver.solve_state_from_wheel_travel(0.0)["shock_stroke"]
    at_full = solver.solve_state_from_wheel_travel(MAX_TRAVEL_MM)["shock_stroke"]
    return float(at_full - at_rest)


def evaluate(points: Dict[str, np.ndarray]) -> Dict[str, float]:
    """
    Solves the linkage for a candidate hardpoint set.

    Ruling R6: this builds the full trajectory and is therefore called once,
    after the fit has converged, to produce the reported kinematics -- not from
    inside the objective.
    """
    traj = _build_solver(points).solve_trajectory(n_points=41, max_travel=MAX_TRAVEL_MM)
    stroke = traj["shock_stroke"]
    return {
        "max_stroke": float(traj["max_stroke"]),
        "stroke_at_zero_travel": float(stroke[0]),
        "stroke_excursion": float(stroke[-1] - stroke[0]),
        "initial_leverage_ratio": float(traj["initial_leverage_ratio"]),
        "final_leverage_ratio": float(traj["final_leverage_ratio"]),
        "progressivity_pct": float(traj["progressivity_pct"]),
        "max_link_error": float(traj["max_link_error"]),
    }


def _photo_goal(targets: Dict[str, Tuple[float, float]]) -> Dict[str, np.ndarray]:
    """
    The positions the fit is scored against.

    Every linkage pivot is its measured photo position; P2 alone is translated
    forward by CHAINSTAY_SHIFT_MM because the 447.5 mm chainstay spec overrides
    the 468.5 mm the photo implies, and the rear group moves as a rigid body
    (Ruling R10: the shift is X-only and intended). P12 is not a red bolt in the
    photograph, so it keeps its authored position.
    """
    def pt(name: str, shift: float = 0.0) -> np.ndarray:
        x, z = targets[name]
        return np.array([x + shift, 0.0, z], dtype=float)

    return {
        "P0": pt("P0_candidate"),
        "P2": pt("P2_candidate", shift=CHAINSTAY_SHIFT_MM),
        "P3": pt("P3"),
        "P4": pt("P4"),
        "P5": pt("P5"),
        "P6": pt("P6"),
        "P7": pt("P7"),
        "P12": np.array([-435.0 + CHAINSTAY_SHIFT_MM, 0.0, 60.0], dtype=float),
    }


def _seed(targets: Dict[str, Tuple[float, float]]) -> Dict[str, np.ndarray]:
    """Starting point: the goal positions, with P6 derived rather than measured."""
    seed = {name: p.copy() for name, p in _photo_goal(targets).items()}
    seed["P6"] = _derive_p6(seed["P4"], seed["P7"])
    return seed


def _initial_simplex(x0: np.ndarray, step: float = SIMPLEX_STEP_MM) -> np.ndarray:
    simplex = np.repeat(x0[None, :], x0.size + 1, axis=0)
    simplex[1:] += step * np.eye(x0.size)
    return simplex


def fit(targets: Dict[str, Tuple[float, float]], verbose: bool = False) -> Dict[str, Any]:
    """Runs the constrained least-squares fit and returns the full payload."""
    seed = _seed(targets)
    goal = _photo_goal(targets)
    calls = 0

    def cost(vec: np.ndarray) -> float:
        nonlocal calls
        calls += 1
        points = _unpack(vec)
        residual = sum(
            float(np.sum((points[name][[0, 2]] - goal[name][[0, 2]]) ** 2)) for name in SCORED
        )
        try:
            stroke = _stroke_excursion(points)
        except (ValueError, ZeroDivisionError):
            return INFEASIBLE_COST
        return residual + STROKE_PENALTY_WEIGHT * (stroke - TARGET_STROKE_MM) ** 2

    x = _pack(seed)
    started = time.perf_counter()
    result = None
    for restart in range(MAX_RESTARTS):
        result = minimize(
            cost, x, method="Nelder-Mead",
            options={
                "maxiter": MAX_EVALUATIONS_PER_RESTART,
                "maxfev": MAX_EVALUATIONS_PER_RESTART,
                "xatol": 1e-8, "fatol": 1e-12,
                "initial_simplex": _initial_simplex(x),
            },
        )
        x = result.x
        if verbose:
            print(f"  restart {restart}: cost {result.fun:.9f}  "
                  f"evals {result.nfev}  success={result.success}")
        if result.success:
            break
    elapsed = time.perf_counter() - started

    points = _unpack(x)
    metrics = evaluate(points)
    residuals = {
        name: float(np.linalg.norm(points[name][[0, 2]] - goal[name][[0, 2]])) for name in SCORED
    }

    if verbose:
        print(f"  total evaluations {calls}  wall {elapsed:.1f}s  "
              f"({1000.0 * elapsed / max(calls, 1):.3f} ms/eval)  success={result.success}")

    return {
        "points_mm": {name: [round(float(v), 6) for v in points[name]] for name in SCORED},
        "kinematics": {k: round(v, 9) for k, v in metrics.items()},
        "residuals_mm": {k: round(v, 4) for k, v in residuals.items()},
        "chainstay_shift_mm": CHAINSTAY_SHIFT_MM,
        "shock_eye_to_eye_mm": round(
            float(np.linalg.norm(points["P7"] - points["P6"])), 9
        ),
        "optimiser": {
            "method": "Nelder-Mead",
            "dimensions": len(FREE) * 2,
            "evaluations": calls,
            "converged": bool(result.success),
            "wall_seconds": round(elapsed, 1),
        },
    }


def main() -> None:
    targets = json.loads(REFERENCE_JSON.read_text())["points_mm"] if REFERENCE_JSON.exists() \
        else build_reference()["points_mm"]
    payload = fit({k: tuple(v) for k, v in targets.items()}, verbose=True)
    OUTPUT.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"Wrote {OUTPUT}")
    print(json.dumps(payload["kinematics"], indent=2))
    print(json.dumps(payload["residuals_mm"], indent=2))


if __name__ == "__main__":
    main()
