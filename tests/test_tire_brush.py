"""Synthetic C4 return-map, work, and unrestricted-drive stand checks."""

from itertools import product
from math import inf, nan

import pytest

from bike_sim.physics.tire import brush_step


def assert_passive(args):
    xi, u, v_roll, load, k, mu, length, dt = args
    new_xi, force, loss = brush_step(*args)
    old_energy = 0.5 * k * xi**2
    new_energy = 0.5 * k * new_xi**2
    contact_work = force * u * dt
    assert abs(force) <= mu * load + 1e-10
    assert loss >= 0.0
    scale = max(1.0, abs(contact_work), old_energy, new_energy, loss)
    assert contact_work + new_energy - old_energy + loss == pytest.approx(0.0, abs=1e-12 * scale)
    return new_xi, force, loss


def test_drive_slip_creates_forward_force_within_friction_limit():
    _, force, _ = assert_passive((0.0, -1.0, 5.0, 100.0, 20000.0, 0.5, 0.2, 0.01))
    assert 0.0 < force <= 50.0


def test_static_shear_can_hold_without_wheel_rotation():
    assert brush_step(0.001, 0.0, 0.0, 100.0, 20000.0, 0.5, 0.2, 0.01) == pytest.approx((0.001, -20.0, 0.0))


@pytest.mark.parametrize("xi", [-0.01, 0.0, 0.01])
def test_lift_off_releases_and_accounts_for_all_shear_energy(xi):
    assert brush_step(xi, 12.0, 3.0, 0.0, 20000.0, 0.5, 0.2, 0.01) == pytest.approx((0.0, 0.0, 0.5 * 20000.0 * xi**2))


def test_zero_friction_releases_stored_energy():
    assert brush_step(0.01, -1.0, 2.0, 100.0, 20000.0, 0.0, 0.2, 0.01) == pytest.approx((0.0, 0.0, 1.0))


def test_load_drop_returns_to_new_friction_limit():
    _, force, loss = assert_passive((0.01, 0.0, 0.0, 10.0, 20000.0, 0.5, 0.2, 0.01))
    assert force == pytest.approx(-5.0)
    assert loss > 0.0


def test_reversal_releases_then_rebuilds_shear():
    xi, _, _ = brush_step(0.0, -1.0, 2.0, 100.0, 20000.0, 0.5, 0.2, 0.001)
    for _ in range(30):
        xi, force, _ = assert_passive((xi, 1.0, 2.0, 100.0, 20000.0, 0.5, 0.2, 0.001))
    assert force < 0.0


def test_passivity_across_velocity_load_and_initial_state_grid():
    for xi, u, v_roll, load, mu, dt in product(
        [-0.03, -0.001, 0.0, 0.001, 0.03],
        [-20.0, -0.1, 0.0, 0.1, 20.0],
        [-10.0, 0.0, 10.0],
        [0.0, 1.0, 100.0, 1000.0],
        [0.0, 0.5, 1.2],
        [0.0001, 0.001, 0.01],
    ):
        assert_passive((xi, u, v_roll, load, 20000.0, mu, 0.2, dt))


def test_rolling_relaxation_is_independent_of_direction():
    forward = brush_step(0.001, 0.1, 5.0, 1000.0, 20000.0, 0.5, 0.2, 0.01)
    reverse = brush_step(0.001, 0.1, -5.0, 1000.0, 20000.0, 0.5, 0.2, 0.01)
    assert forward == reverse


def test_return_map_matches_backward_euler_before_saturation():
    xi, force, _ = assert_passive((0.001, 0.02, 2.0, 1000.0, 20000.0, 0.5, 0.2, 0.01))
    expected = (0.001 + 0.01 * 0.02) / (1.0 + 0.01 * 2.0 / 0.2)
    assert xi == pytest.approx(expected)
    assert force == pytest.approx(-20000.0 * expected)


@pytest.mark.parametrize("index", range(8))
@pytest.mark.parametrize("value", [nan, inf, -inf])
def test_nonfinite_brush_inputs_rejected(index, value):
    args = [0.001, 0.1, 2.0, 100.0, 20000.0, 0.5, 0.2, 0.01]
    args[index] = value
    with pytest.raises(ValueError):
        brush_step(*args)


@pytest.mark.parametrize("index,value", [(3, -1.0), (4, 0.0), (4, -1.0), (5, -0.1), (6, 0.0), (6, -1.0), (7, 0.0), (7, -1.0)])
def test_invalid_brush_parameters_rejected(index, value):
    args = [0.001, 0.1, 2.0, 100.0, 20000.0, 0.5, 0.2, 0.01]
    args[index] = value
    with pytest.raises(ValueError):
        brush_step(*args)


def test_invalid_parameters_are_rejected_even_on_lift_off():
    with pytest.raises(ValueError):
        brush_step(0.001, 0.1, 2.0, 0.0, -1.0, 0.5, 0.2, 0.01)


def test_two_callers_do_not_share_shear_state():
    args = (0.001, 0.1, 2.0, 100.0, 20000.0, 0.5, 0.2, 0.01)
    expected = brush_step(*args)
    brush_step(-0.03, -2.0, -5.0, 200.0, 10000.0, 0.8, 0.1, 0.001)
    assert brush_step(*args) == expected


def test_finite_inputs_cannot_silently_produce_infinite_energy():
    with pytest.raises(ArithmeticError):
        brush_step(1e200, 0.0, 0.0, 0.0, 20000.0, 0.5, 0.2, 0.01)


def launch(dt, motor_torque=100.0):
    # Newton/Euler two-DOF stand, not a MuJoCo bike or experimental validation.
    # One freely spinning wheel drives a translating mass. The fixed normal
    # load is a synthetic fixture constraint. Positive +Y spin gives u=v-R*w.
    mass, inertia, radius = 20.0, 0.15, 0.3
    normal_load, mu = 100.0, 0.5
    xi = velocity = omega = max_force = total_loss = 0.0
    for _ in range(round(0.2 / dt)):
        u = velocity - radius * omega
        xi, force, loss = assert_passive((xi, u, radius * omega, normal_load, 20000.0, mu, 0.2, dt))
        velocity += dt * force / mass
        omega += dt * (motor_torque - radius * force) / inertia
        max_force = max(max_force, abs(force))
        total_loss += loss
    return velocity, omega, max_force, total_loss


def test_unlimited_motor_torque_produces_wheelspin():
    velocity, omega, max_force, loss = launch(0.0001)
    assert 0.0 < velocity < 1.0
    assert 0.3 * omega - velocity > 10.0
    assert max_force == pytest.approx(50.0)
    assert loss > 0.0


def test_synthetic_launch_converges_with_timestep_refinement():
    coarse, medium, fine = (launch(dt) for dt in (0.0002, 0.0001, 0.00005))
    for index in (0, 1):
        assert abs(medium[index] - fine[index]) < abs(coarse[index] - fine[index])
        assert medium[index] == pytest.approx(fine[index], rel=0.005)


def test_reverse_launch_has_the_same_passive_response():
    forward = launch(0.0001)
    reverse = launch(0.0001, motor_torque=-100.0)
    assert reverse[0] == pytest.approx(-forward[0])
    assert reverse[1] == pytest.approx(-forward[1])
    assert reverse[2:] == pytest.approx(forward[2:])


@pytest.mark.parametrize("value", [True, "0.001", None, [0.001]])
def test_brush_rejects_non_scalar_inputs(value):
    with pytest.raises(ValueError):
        brush_step(value, 0.1, 2.0, 100.0, 20000.0, 0.5, 0.2, 0.01)
