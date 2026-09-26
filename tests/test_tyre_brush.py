"""Longitudinal relaxation and brush-force behaviour (RIDE.md section 4.1).

The numerical `κ_peak` estimates for the reference patch and the four authored surfaces are
recorded in the contract; these tests assert the peak force band, not those car-derived slip
ratios.
"""

import ast
import math
from pathlib import Path

import numpy as np
import pytest

from bike_sim.sim.ride.tyre.brush import DiscretisedBrush, lumped_brush, relax
from bike_sim.terrain.surface import SURFACES, SurfaceSpec


def _parabolic_loads(total_load_n: float, half_length_m: float, n_elements: int) -> np.ndarray:
    """Cell loads sampled from a parabolic pressure patch, normalised to the given load."""
    dx_m = 2.0 * half_length_m / n_elements
    x_m = -half_length_m + (np.arange(n_elements) + 0.5) * dx_m
    pressure_shape = np.maximum(1.0 - (x_m / half_length_m) ** 2, 0.0)
    return total_load_n * pressure_shape / np.sum(pressure_shape)


def _lumped_curve(surface: SurfaceSpec, speed_mps: float, kappa: np.ndarray,
                  load_n: float, half_length_m: float) -> np.ndarray:
    """Vector form of `lumped_brush` used for the speed sweep."""
    c_px = surface.slip_stiffness_per_load * load_n / (2.0 * half_length_m ** 2)
    sliding_speed_mps = np.abs(kappa * speed_mps)
    mu = surface.mu(sliding_speed_mps)
    sigma_x = kappa / (1.0 + kappa)
    theta = 2.0 * c_px * half_length_m ** 2 / (3.0 * mu * load_n)
    z = theta * sigma_x
    force = 3.0 * mu * load_n * z * (1.0 - np.abs(z) + z * z / 3.0)
    saturated = np.abs(z) >= 1.0
    force[saturated] = mu[saturated] * load_n * np.sign(z[saturated])
    return force


def test_relaxation_is_exact_at_finite_and_zero_hub_speed():
    previous = 0.15
    dt_s = 0.01
    sigma_m = 0.09
    v_x_mps = 4.0
    v_s_mps = -0.4
    target = -v_s_mps / abs(v_x_mps)
    expected = target + (previous - target) * math.exp(-abs(v_x_mps) * dt_s / sigma_m)
    assert relax(previous, v_x_mps, v_s_mps, sigma_m, dt_s) == pytest.approx(expected)
    assert relax(previous, 0.0, 0.0, sigma_m, dt_s) == previous
    assert relax(previous, 0.0, -0.09, sigma_m, dt_s) == pytest.approx(0.16)
    assert relax(previous, 1e-9, -1e-9, sigma_m, dt_s) == pytest.approx(
        previous + 1e-9 * dt_s / sigma_m, rel=1e-6
    )


def test_relaxation_reaches_63_percent_after_rolling_one_sigma():
    sigma_m = 0.09
    v_x_mps = 6.0
    dt_s = sigma_m / v_x_mps / 1000
    kappa = 0.0
    for _ in range(1000):
        kappa = relax(kappa, v_x_mps, -0.1 * v_x_mps, sigma_m, dt_s)
    assert kappa == pytest.approx(0.1 * (1.0 - math.exp(-1.0)), rel=1e-10)


@pytest.mark.parametrize("surface", list(SURFACES.values()), ids=list(SURFACES))
def test_lumped_brush_zero_load_sign_and_lockup(surface):
    free = lumped_brush(0.5, 0.0, 0.06, surface)
    assert free.force_n == 0.0 and not free.fully_sliding

    drive = lumped_brush(0.1, 418.0, 0.06, surface)
    brake = lumped_brush(-0.1, 418.0, 0.06, surface)
    assert drive.force_n > 0.0 and brake.force_n < 0.0
    locked = lumped_brush(-1.0, 418.0, 0.06, surface)
    assert locked.fully_sliding
    assert locked.force_n == pytest.approx(-locked.friction_coefficient * 418.0)


@pytest.mark.parametrize("surface", list(SURFACES.values()), ids=list(SURFACES))
def test_lumped_brush_initial_slope_is_the_surface_slip_stiffness(surface):
    load_n, half_length_m, kappa = 418.0, 0.061, 1e-5
    forward = lumped_brush(kappa, load_n, half_length_m, surface, sliding_speed_mps=0.0)
    backward = lumped_brush(-kappa, load_n, half_length_m, surface, sliding_speed_mps=0.0)
    initial_slope_per_load = (forward.force_n - backward.force_n) / (2.0 * kappa * load_n)
    assert initial_slope_per_load == pytest.approx(surface.slip_stiffness_per_load, rel=0.05)


@pytest.mark.parametrize("surface", list(SURFACES.values()), ids=list(SURFACES))
def test_peak_friction_stays_near_the_surface_peak_from_15_to_45_kmh(surface):
    load_n, half_length_m = 418.0, 0.061
    kappa = np.linspace(-0.999, 1.0, 4001)
    for speed_kmh in (15.0, 30.0, 45.0):
        force_n = _lumped_curve(surface, speed_kmh / 3.6, kappa, load_n, half_length_m)
        kappa_at_peak = float(kappa[np.argmax(np.abs(force_n))])
        peak_mu = float(np.max(np.abs(force_n)) / load_n)
        assert peak_mu == pytest.approx(surface.mu_peak, rel=0.10), (
            surface.name, speed_kmh, peak_mu, kappa_at_peak
        )


@pytest.mark.parametrize("surface", list(SURFACES.values()), ids=list(SURFACES))
def test_discretised_brush_matches_lumped_curve_on_a_uniform_patch(surface):
    load_n, half_length_m, n_elements = 418.0, 0.061, 32
    v_x_mps, dt_s, sigma_m = 4.0, 0.0005, 0.09
    normal_loads_n = _parabolic_loads(load_n, half_length_m, n_elements)
    for kappa in np.linspace(-1.0, 1.0, 41):
        v_s_mps = -float(kappa) * v_x_mps
        brush = DiscretisedBrush(n_elements)
        for _ in range(400):
            result = brush.step(
                normal_loads_n,
                half_length_m=half_length_m,
                v_x_mps=v_x_mps,
                v_s_mps=v_s_mps,
                sigma_m=sigma_m,
                surface=surface,
                dt_s=dt_s,
            )
        reference = lumped_brush(
            float(kappa), load_n, half_length_m, surface,
            sliding_speed_mps=abs(v_s_mps),
        )
        assert abs(result.force_n - reference.force_n) <= 0.03 * result.friction_coefficient * load_n, (
            surface.name, kappa, result.force_n, reference.force_n
        )


def test_parked_untorqued_brush_holds_zero_force_without_drift_for_ten_seconds():
    load_n, half_length_m, n_elements = 500.0, 0.06, 30
    brush = DiscretisedBrush(n_elements)
    loads = _parabolic_loads(load_n, half_length_m, n_elements)
    dt_s = 0.001
    for _ in range(10_000):
        result = brush.step(
            loads,
            half_length_m=half_length_m,
            v_x_mps=0.0,
            v_s_mps=0.0,
            sigma_m=0.09,
            surface=SURFACES["hardpack"],
            dt_s=dt_s,
        )
    assert result.force_n == 0.0
    assert not result.fully_sliding
    assert brush.kappa_prime == 0.0
    assert brush.bristle_deflection_m == pytest.approx(np.zeros(n_elements))


def test_discretised_brush_reports_full_sliding_at_both_slip_limits():
    load_n, half_length_m, n_elements = 418.0, 0.061, 32
    loads = _parabolic_loads(load_n, half_length_m, n_elements)
    surface = SURFACES["asphalt"]
    for kappa in (-1.0, 1.0):
        brush = DiscretisedBrush(n_elements)
        v_x_mps = 4.0
        v_s_mps = -kappa * v_x_mps
        for _ in range(400):
            result = brush.step(
                loads,
                half_length_m=half_length_m,
                v_x_mps=v_x_mps,
                v_s_mps=v_s_mps,
                sigma_m=0.09,
                surface=surface,
                dt_s=0.0005,
            )
        assert result.fully_sliding
        assert abs(result.force_n) == pytest.approx(
            result.friction_coefficient * load_n, rel=1e-12
        )


def test_detailed_brush_remains_finite_at_45_kmh_and_forced_courant_two():
    load_n, half_length_m, n_elements = 418.0, 0.06, 30
    loads = _parabolic_loads(load_n, half_length_m, n_elements)
    dx_m = 2.0 * half_length_m / n_elements
    cases = (
        (45.0 / 3.6, 0.00025, False),
        (8.0, 2.0 * dx_m / (8.0 * 1.1), True),
    )
    for v_x_mps, dt_s, forced_two in cases:
        brush = DiscretisedBrush(n_elements)
        v_s_mps = -0.1 * v_x_mps
        wheel_surface_speed_mps = v_x_mps - v_s_mps
        courant = wheel_surface_speed_mps * dt_s / dx_m
        if forced_two:
            assert courant == pytest.approx(2.0)
        for _ in range(1000):
            result = brush.step(
                loads,
                half_length_m=half_length_m,
                v_x_mps=v_x_mps,
                v_s_mps=v_s_mps,
                sigma_m=0.09,
                surface=SURFACES["asphalt"],
                dt_s=dt_s,
            )
            assert math.isfinite(result.force_n)
            assert abs(result.force_n) <= result.friction_coefficient * load_n + 1e-9


def test_detailed_input_filter_and_bristle_transport_match_lumped_total_relaxation():
    load_n, half_length_m, n_elements = 418.0, 0.06, 30
    v_x_mps, dt_s, sigma_m, target_kappa = 4.0, 0.0005, 0.09, 0.10
    v_s_mps = -target_kappa * v_x_mps
    surface = SURFACES["hardpack"]
    loads = _parabolic_loads(load_n, half_length_m, n_elements)
    detailed = DiscretisedBrush(n_elements)
    lumped_kappa = 0.0
    detailed_force = []
    lumped_force = []
    for _ in range(500):
        detailed_result = detailed.step(
            loads,
            half_length_m=half_length_m,
            v_x_mps=v_x_mps,
            v_s_mps=v_s_mps,
            sigma_m=sigma_m,
            surface=surface,
            dt_s=dt_s,
        )
        lumped_kappa = relax(lumped_kappa, v_x_mps, v_s_mps, sigma_m, dt_s)
        lumped_result = lumped_brush(
            lumped_kappa,
            load_n,
            half_length_m,
            surface,
            sliding_speed_mps=abs(v_s_mps),
        )
        detailed_force.append(detailed_result.force_n)
        lumped_force.append(lumped_result.force_n)

    detailed_force = np.asarray(detailed_force)
    lumped_force = np.asarray(lumped_force)
    detailed_steady = float(detailed_force[-1])
    lumped_steady = float(lumped_force[-1])
    assert detailed_steady == pytest.approx(lumped_steady, rel=0.03)
    detailed_crossing = int(np.flatnonzero(detailed_force >= 0.63 * detailed_steady)[0])
    lumped_crossing = int(np.flatnonzero(lumped_force >= 0.63 * lumped_steady)[0])
    assert abs(detailed_crossing - lumped_crossing) * dt_s <= 0.10 * (sigma_m / v_x_mps)


def test_brush_module_does_not_import_mujoco():
    source = Path(__file__).resolve().parents[1] / "src/bike_sim/sim/ride/tyre/brush.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    imported = {
        alias.name.split(".")[0]
        for node in ast.walk(tree)
        if isinstance(node, ast.Import)
        for alias in node.names
    }
    imported |= {
        node.module.split(".")[0]
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.module
    }
    assert "mujoco" not in imported
