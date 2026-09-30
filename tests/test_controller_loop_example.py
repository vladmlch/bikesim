import importlib.util
from pathlib import Path
from bike_sim.sim.research.sensors import SensorObservation
from bike_sim.sim.ride.control import RideControl

PATH = Path(__file__).resolve().parents[1]/'examples/research/controller_loop.py'


def _module():
    spec = importlib.util.spec_from_file_location('controller_loop_example', PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _obs(valid=True, t=2.):
    return SensorObservation(time_s=t, source_time_s=t, valid=valid, specific_force_body_mps2=(0., 0., 9.8),
        pitch_rate_up_rad_s=0., front_wheel_rad_s=5., rear_wheel_rad_s=5., crank_rad_s=3.,
        motor_torque_nm=0., human_torque_nm=0.)


def test_policy_takes_observation_and_demand():
    policy = _module().policy
    assert policy(_obs(), 60.) == RideControl(motor_torque_nm=60., motor_limit_nm=_module().MOTOR_CAP_NM, human_torque_nm=None)
    capped = policy(_obs(), 500.)
    assert capped.motor_torque_nm == _module().MOTOR_CAP_NM
    assert policy(_obs(valid=False), 60.).motor_torque_nm == 0.
    assert policy(_obs(), None).motor_torque_nm == 0.   # no demand: do not invent one
