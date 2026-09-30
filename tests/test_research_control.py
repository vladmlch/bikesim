import dataclasses
import numpy as np
import pytest
from bike_sim.physics.motor import AssistController
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.physical_config import TireBackendConfig
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset


@pytest.mark.parametrize('field', ['motor_torque_nm', 'motor_limit_nm', 'human_torque_nm'])
@pytest.mark.parametrize('bad', [-1., float('nan'), float('inf'), True, '4'])
def test_command_validation(field, bad):
    with pytest.raises(ValueError):
        RideControl(**{field: bad})


def test_direct_motor_keeps_physical_limits_and_can_start_without_pedaling():
    motor = AssistController(max_torque=60., max_power=200., stall_timeout_s=5.)
    for _ in range(300):
        value = motor.step(0., 0., 0., False, .005, torque_request_nm=100.)
    assert value == pytest.approx(60., abs=1e-5)
    assert motor.step(0., 300., 1., False, .005, torque_request_nm=100.) <= 200./(10*np.pi)
    assert motor.step(0., 20., 1., True, .005, torque_request_nm=100.) == 0.
    assert motor.step(0., 20., 10., False, .005, torque_request_nm=100.) == 0.


def test_invalid_direct_motor_request_does_not_mutate_state():
    motor = AssistController()
    state = vars(motor).copy()
    with pytest.raises(ValueError):
        motor.step(0., 0., 0., False, .01, torque_request_nm=float('nan'))
    assert vars(motor) == state


@pytest.fixture
def sim():
    return RideSimulation(track=get_preset('flat'), rider='lumped',
        physics_config=SimulationPhysicsConfig('physical', drive_mode='crank_effort',
            tires=TireBackendConfig(backend='compliant_2d')))


def test_command_is_logged_limited_and_brakes_override_it(sim):
    command = RideControl(motor_torque_nm=40., motor_limit_nm=2., human_torque_nm=0.)
    for _ in range(20):
        sim.step(control=command)
        assert 0. <= sim.physical.drive.last['motor_torque_nm'] <= 2.
    assert sim.physical.sample.channels['control']['motor_torque_nm'] == 40.
    assert sim.physical.drive.last['motor_control_source'] == 'external_request'
    sim.step(front_brake_demand=.1, control=command)
    assert sim.physical.drive.last['motor_torque_nm'] == 0.
    assert abs(sim.physical.energy['electrical_residual_j']) < 1e-6


def test_default_none_preserves_assist_semantics(sim):
    sim.step()
    assert sim.physical.drive.last['motor_control_source'] == 'assist'
    assert sim.physical.drive.last['motor_torque_nm'] == 0.


def test_bad_control_fails_before_state_changes(sim):
    q, v, t = sim.data.qpos.copy(), sim.data.qvel.copy(), sim.time_s
    with pytest.raises(ValueError):
        sim.step(control={'motor_torque_nm': 10.})
    np.testing.assert_array_equal(sim.data.qpos, q)
    np.testing.assert_array_equal(sim.data.qvel, v)
    assert sim.time_s == t


def test_coast_cannot_accept_motor_authority(sim):
    sim.physical.cfg = dataclasses.replace(sim.physical.cfg, drive_mode='coast')
    with pytest.raises(ValueError, match='effort'):
        sim.step(control=RideControl(motor_torque_nm=1.))
