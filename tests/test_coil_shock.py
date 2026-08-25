"""
Unit tests for the coil shock spring and bottom-out bumper model.

Tests include:
- Linearity of the coil term and its agreement with the declared rate.
- Preload offsetting the whole force curve.
- Bumper engagement: exactly zero before it, continuous across it, at peak at full stroke.
- Monotonicity of the total axial force in stroke.
- The shipped defaults matching the repository's documented spring settings.
"""

import numpy as np
import pytest

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.physics.coil_shock import CoilShock, CoilShockSpecs


def test_spring_force_is_linear_in_stroke():
    """The coil rate is constant across the stroke: equal steps give equal force steps."""
    shock = CoilShock()
    strokes = np.linspace(0.0, 65.0, 66)
    forces = np.array([shock.compute_spring_force(s) for s in strokes])

    rate_n_mm = CoilShockSpecs().rate_n_m / 1000.0
    slopes = np.diff(forces) / np.diff(strokes)
    assert slopes == pytest.approx(rate_n_mm)
    assert shock.compute_spring_force(0.0) == pytest.approx(0.0)
    assert shock.compute_spring_force(32.5) == pytest.approx(shock.compute_spring_force(65.0) / 2.0)


def test_preload_offsets_the_whole_curve():
    """Preload adds one constant force at every stroke and does not change the rate."""
    plain = CoilShock()
    preloaded = CoilShock(CoilShockSpecs(preload_mm=5.0))
    offset_n = 5.0 * CoilShockSpecs().rate_n_m / 1000.0

    for stroke_mm in [0.0, 10.0, 32.5, 55.0, 65.0]:
        assert preloaded.compute_spring_force(stroke_mm) == pytest.approx(
            plain.compute_spring_force(stroke_mm) + offset_n
        )


def test_bumper_is_zero_before_engagement():
    """The bumper contributes nothing until the shaft reaches its engagement point."""
    shock = CoilShock()
    engage_mm = CoilShockSpecs().bumper_engage_mm
    assert engage_mm == pytest.approx(55.0)

    for stroke_mm in [0.0, 20.0, 54.0, engage_mm]:
        assert shock.compute_bumper_force(stroke_mm) == 0.0
        assert shock.compute_axial_force(stroke_mm) == pytest.approx(
            shock.compute_spring_force(stroke_mm)
        )


def test_bumper_is_continuous_at_engagement():
    """Force and slope are both continuous where the bumper engages: no step, no kink."""
    specs = CoilShockSpecs()
    shock = CoilShock(specs)
    engage_mm = specs.bumper_engage_mm

    # Quadratic in engagement depth, so the added force is second order in the overlap.
    for depth_mm in [0.01, 0.1, 1.0]:
        assert shock.compute_bumper_force(engage_mm + depth_mm) == pytest.approx(
            specs.bumper_peak_n * (depth_mm / specs.bumper_length_mm) ** 2
        )

    # The slope of the total curve is the coil rate on both sides of engagement.
    eps_mm = 0.0001
    rate_n_mm = specs.rate_n_m / 1000.0
    slope_before = (
        shock.compute_axial_force(engage_mm) - shock.compute_axial_force(engage_mm - eps_mm)
    ) / eps_mm
    slope_after = (
        shock.compute_axial_force(engage_mm + eps_mm) - shock.compute_axial_force(engage_mm)
    ) / eps_mm
    assert slope_before == pytest.approx(rate_n_mm)
    assert slope_after == pytest.approx(rate_n_mm, rel=1e-4)


def test_bumper_reaches_peak_at_full_stroke():
    """The bumper delivers exactly its peak force at full stroke, and less before it."""
    specs = CoilShockSpecs()
    shock = CoilShock(specs)
    assert shock.compute_bumper_force(specs.stroke_mm) == pytest.approx(specs.bumper_peak_n)
    # Half way into the bumper a quadratic has delivered a quarter of its peak.
    assert shock.compute_bumper_force(specs.stroke_mm - specs.bumper_length_mm / 2.0) == (
        pytest.approx(specs.bumper_peak_n / 4.0)
    )


def test_total_force_increases_monotonically_with_stroke():
    """Total axial force rises everywhere: no soft spot where the bumper takes over."""
    shock = CoilShock()
    strokes = np.linspace(0.0, 65.0, 651)
    forces = np.array([shock.compute_axial_force(s) for s in strokes])
    assert np.all(np.diff(forces) > 0.0)


def test_defaults_match_the_shipped_spring_settings():
    """The default rate is the repository's shipped shock stiffness, with no preload."""
    specs = CoilShockSpecs()
    assert specs.rate_n_m == pytest.approx(BikeSpecs().shock_stiffness)
    assert specs.preload_mm == 0.0
    assert specs.stroke_mm == pytest.approx(BikeSpecs().shock_stroke)
