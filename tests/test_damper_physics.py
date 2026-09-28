"""Regression checks for the rear shock's corrected and legacy damping."""

import pytest

from bike_sim.physics.damper import SuperDeluxeDamper


@pytest.mark.parametrize("firm", [False, True])
def test_hbo_adds_force_in_both_modes(firm: bool) -> None:
    damper = SuperDeluxeDamper(lockout_firm=firm)

    assert damper.compute_damping_force(1.0, 65.0) > damper.compute_damping_force(1.0, 20.0)


def test_firm_force_is_continuous() -> None:
    damper = SuperDeluxeDamper(lockout_firm=True)

    left = damper.compute_damping_force(0.03 - 1e-8, 20.0)
    right = damper.compute_damping_force(0.03 + 1e-8, 20.0)

    assert abs(right - left) < 1e-3


@pytest.mark.parametrize("firm", [False, True])
@pytest.mark.parametrize("stroke_mm", [0.0, 20.0, 52.0, 65.0])
@pytest.mark.parametrize("velocity_mps", [-2.0, -0.2, -1e-6, 0.0, 1e-6, 0.03, 0.5, 2.0])
def test_damping_is_passive(firm: bool, stroke_mm: float, velocity_mps: float) -> None:
    damper = SuperDeluxeDamper(lockout_firm=firm)

    assert damper.compute_damping_force(velocity_mps, stroke_mm) * velocity_mps >= 0.0


@pytest.mark.parametrize(
    ("firm", "velocity_mps", "stroke_mm", "expected_n"),
    [
        (False, -0.2, 20.0, -997.6136249253102),
        (False, 0.03, 20.0, 51.022395684534956),
        (False, 1.0, 65.0, 10650.522576967967),
        (True, -0.2, 20.0, -997.6136249253102),
        (True, 0.029, 20.0, 435.0),
        (True, 0.03, 20.0, 480.0),
        (True, 1.0, 20.0, 2052.4916538592547),
        (True, 1.0, 65.0, 2052.4916538592547),
    ],
)
def test_legacy_force_matches_previous_model(
    firm: bool, velocity_mps: float, stroke_mm: float, expected_n: float
) -> None:
    damper = SuperDeluxeDamper(lockout_firm=firm, legacy_behavior=True)

    assert damper.compute_damping_force(velocity_mps, stroke_mm) == pytest.approx(expected_n)
