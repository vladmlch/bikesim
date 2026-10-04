"""Momentum stands must exercise actual internal motor actuation."""
import pytest

from bike_sim.validation.system_momentum import momentum_rig


@pytest.mark.slow
def test_airborne_motor_stand_delivers_work_without_root_actuation():
    metrics, bounds = momentum_rig(.00125, active=True, rider_active=False)
    assert metrics['peak_motor_torque_nm'] > 10.
    assert metrics['motor_shaft_work_j'] > 0.
    assert metrics['joint_positive_work_j'] == 0.
    for name in ('impulse_residual_ratio', 'angular_residual_ratio',
                 'shaft_power_identity_error_w', 'root_actuator_force_n',
                 'ground_contact_count'):
        lo, hi = bounds[name]
        assert lo <= metrics[name] <= hi, (name, metrics[name], bounds[name])


@pytest.mark.slow
def test_passive_airborne_stand_has_no_motor_or_joint_work():
    metrics, _ = momentum_rig(.00125, active=False)
    assert metrics['peak_motor_torque_nm'] == 0.
    assert metrics['motor_shaft_work_j'] == 0.
    assert metrics['joint_positive_work_j'] == 0.
