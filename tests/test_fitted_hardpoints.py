"""Validates the committed fitted hardpoints, not the optimiser that produced them."""

import json
from pathlib import Path

import numpy as np
import pytest

from bike_sim.geometry.hardpoints import compute_rear_axle
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.kinematics.solver import HorstLinkageSolver

REPO = Path(__file__).resolve().parent.parent
FITTED = REPO / "docs" / "reference" / "fitted_hardpoints.json"
REFERENCE = REPO / "docs" / "reference" / "bulls_reference_points.json"

FIDELITY_TOLERANCE_MM = 8.0
CHAINSTAY_SHIFT_MM = 21.0


@pytest.fixture(scope="module")
def fitted():
    assert FITTED.exists(), "run: uv run --with scipy --with pillow python -m tools.fit_hardpoints"
    return json.loads(FITTED.read_text())


@pytest.fixture(scope="module")
def fitted_solver(fitted):
    """Module-scoped: the solver is immutable and every test builds it identically."""
    return _solver(fitted["points_mm"])


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


def test_binding_equality_stroke_is_exactly_65(fitted_solver):
    traj = fitted_solver.solve_trajectory(n_points=101, max_travel=180.0)
    assert traj["max_stroke"] == pytest.approx(65.0, abs=0.01)


def test_shock_stroke_is_zero_at_zero_travel(fitted_solver):
    """The invariant Important #1 actually violated: a P6 not collinear with
    P4->P7 manufactures phantom stroke at rest, which silently inflates the
    65 mm excursion measured only at full travel."""
    st0 = fitted_solver.solve_state_from_wheel_travel(0.0)
    assert st0["shock_stroke"] == pytest.approx(0.0, abs=1e-6)


def test_four_bar_closes(fitted_solver):
    traj = fitted_solver.solve_trajectory(n_points=41, max_travel=180.0)
    assert traj["max_link_error"] < 1e-6


def test_leverage_curve_is_progressive_and_sane(fitted_solver):
    traj = fitted_solver.solve_trajectory(n_points=41, max_travel=180.0)
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


def test_shock_eye_to_eye_matches_spec(fitted_solver):
    """Assert against the realised eyelet -- the solver's own re-projection of
    P6 onto P4->P7 -- not the stored constant, since that re-projected value
    is what every downstream consumer (linkage_solver, export_mujoco) actually
    reads. P6 is derived collinear on P4->P7 at exactly 205 mm, so this closes
    to machine precision; abs=0.5 previously let a 204.957 mm real eyelet
    (a 0.043 mm violation, but a phantom 65 mm stroke miss) pass unnoticed."""
    st0 = fitted_solver.solve_state_from_wheel_travel(0.0)
    assert st0["shock_length"] == pytest.approx(205.0, abs=1e-6)
