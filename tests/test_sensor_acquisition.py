from dataclasses import replace
import numpy as np
import pytest
from bike_sim.sim.research.sensors import SensorConfig, SensorObservation, SensorPipeline
from bike_sim.sim.research.environment import ExperimentConfig, ResearchEnvironment
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.physical_config import TireBackendConfig
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.sim.ride.control import RideControl
from bike_sim.terrain import get_preset


def observation(t=0.):
    return SensorObservation(t, t, True, (1., 2., 3.), .3, 4., 5., 6., 7., 8.)


def test_bias_is_fixed_and_invalid_source_stays_invalid():
    cfg = replace(SensorConfig.ideal(), acceleration_bias_mps2=(.1, -.2, .3), gyro_bias_rad_s=-.05)
    pipe = SensorPipeline(cfg)
    pipe.reset(replace(observation(), valid=False))
    result = pipe.read(0.)
    assert not result.valid
    assert result.specific_force_body_mps2 == pytest.approx((1.1, 1.8, 3.3))
    assert result.pitch_rate_up_rad_s == pytest.approx(.25)


def test_dropouts_hold_last_sample_and_expire_instead_of_inventing_measurements():
    cfg = replace(SensorConfig.ideal(), dropout_probability=1., maximum_age_s=.01)
    pipe = SensorPipeline(cfg)
    pipe.reset(observation())
    pipe.push(observation(.02))
    assert not pipe.read(.02).valid
    assert pipe.samples_dropped == 2
    assert pipe.read(.02) == pipe.read(.02)
    live = SensorPipeline(replace(cfg, dropout_probability=0.))
    live.reset(observation())
    assert live.read(0.).valid
    assert not live.read(.011).valid
    assert live.read(.012).source_time_s == 0.


def test_dropouts_noise_and_reset_are_deterministic():
    c = SensorConfig(sample_period_s=.001, dropout_probability=.25)
    a, b = SensorPipeline(c, seed=71), SensorPipeline(c, seed=71)
    for pipe in (a, b):
        pipe.reset(observation())
        for i in range(1, 100):
            pipe.push(observation(i*.001))
    assert a.read(.1) == b.read(.1)
    assert a.samples_dropped == b.samples_dropped
    assert 0 < a.samples_dropped < a.samples_attempted


@pytest.mark.parametrize('values', [
    {'sample_period_s': -1.}, {'sample_period_s': float('nan')},
    {'dropout_probability': 1.01}, {'dropout_probability': -1.},
    {'acceleration_bias_mps2': (1., 2.)}, {'maximum_age_s': -.01},
    {'gyro_bias_rad_s': float('inf')},
])
def test_bad_sensor_parameters(values):
    with pytest.raises(ValueError):
        SensorConfig(**values)


@pytest.fixture(scope='module')
def sim():
    return RideSimulation(track=get_preset('flat'), rider='lumped',
        physics_config=SimulationPhysicsConfig('physical', drive_mode='crank_effort',
            tires=TireBackendConfig(backend='compliant_2d', surface_mode='track')))


def test_sensor_clock_and_noise_do_not_depend_on_policy_period(sim):
    samples = []
    for period in (.01, .02):
        sim.reset()
        cfg = ExperimentConfig(duration_s=.04, control_period_s=period, actuator_delay_s=0.)
        env = ResearchEnvironment(sim, cfg, SensorConfig(sample_period_s=.001, latency_s=.003))
        while not env.done:
            env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.))
        samples.append({round(o.time_s, 8): o for o in env.observations})
        assert env.pipeline.samples_attempted == 40
    assert samples[0][.02] == samples[1][.02]
    assert samples[0][.04] == samples[1][.04]
    assert samples[0][.02].source_time_s == pytest.approx(.017)


def test_sensor_period_must_be_aligned_with_physics(sim):
    sim.reset()
    with pytest.raises(ValueError, match='sensor period'):
        ResearchEnvironment(sim, sensors=SensorConfig(sample_period_s=.0007))
