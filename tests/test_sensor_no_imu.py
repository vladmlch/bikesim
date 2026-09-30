from dataclasses import replace
import pytest
from bike_sim.sim.research.sensors import SensorConfig, SensorObservation, SensorPipeline


def _raw(t):
    return SensorObservation(time_s=t, source_time_s=t, valid=True,
        specific_force_body_mps2=(1., 2., 9.8), pitch_rate_up_rad_s=.7, front_wheel_rad_s=10.,
        rear_wheel_rad_s=11., crank_rad_s=5., motor_torque_nm=30., human_torque_nm=20.)


def _run(config, seed=3, n=5):
    pipe = SensorPipeline(config, seed=seed)
    pipe.reset(_raw(0.))
    out = [pipe.read(0.)]
    for i in range(1, n):
        pipe.push(_raw(i*.01))
        out.append(pipe.read(i*.01))
    return out


def test_disabled_imu_reports_exact_zeros_including_first_observation():
    for obs in _run(SensorConfig(latency_s=0., imu_enabled=False)):
        assert obs.specific_force_body_mps2 == (0., 0., 0.)
        assert obs.pitch_rate_up_rad_s == 0.
        assert obs.rear_wheel_rad_s != 0. and obs.motor_torque_nm != 0.


def test_enabled_imu_is_unchanged():
    obs = _run(SensorConfig(latency_s=0.))[1]
    assert obs.specific_force_body_mps2 != (0., 0., 0.) and obs.pitch_rate_up_rad_s != 0.


def test_toggling_imu_does_not_shift_other_sensor_noise():
    on = _run(SensorConfig(latency_s=0., imu_enabled=True))
    off = _run(SensorConfig(latency_s=0., imu_enabled=False))
    for a, b in zip(on, off):
        assert (a.front_wheel_rad_s, a.rear_wheel_rad_s, a.crank_rad_s) == (b.front_wheel_rad_s, b.rear_wheel_rad_s, b.crank_rad_s)
        assert (a.motor_torque_nm, a.human_torque_nm) == (b.motor_torque_nm, b.human_torque_nm)


def test_imu_flag_is_validated_and_ideal_still_works():
    with pytest.raises(ValueError, match='imu_enabled'):
        SensorConfig(imu_enabled=1)
    assert SensorConfig.ideal().imu_enabled is True
    assert replace(SensorConfig.ideal(), imu_enabled=False).imu_enabled is False
    with pytest.raises(ValueError):
        SensorConfig(latency_s=-1.)


def test_disabled_imu_in_a_real_ride_is_zero(tmp_path):
    from bike_sim.physics.model_config import SimulationPhysicsConfig
    from bike_sim.physics.physical_config import TireBackendConfig
    from bike_sim.sim.research.environment import ExperimentConfig, ResearchEnvironment
    from bike_sim.sim.ride.control import RideControl
    from bike_sim.sim.ride_sim import RideSimulation
    from bike_sim.terrain import get_preset
    sim = RideSimulation(track=get_preset('flat'), rider='lumped',
        physics_config=SimulationPhysicsConfig('physical', drive_mode='crank_effort',
            tires=TireBackendConfig(backend='compliant_2d', surface_mode='track')))
    env = ResearchEnvironment(sim, ExperimentConfig(duration_s=.05), replace(SensorConfig.ideal(), imu_enabled=False))
    assert env.observation.specific_force_body_mps2 == (0., 0., 0.)
    while not env.done:
        obs = env.step(RideControl(motor_torque_nm=50., human_torque_nm=0.)).observation
        assert obs.specific_force_body_mps2 == (0., 0., 0.) and obs.pitch_rate_up_rad_s == 0.
    assert obs.rear_wheel_rad_s != 0.
