from dataclasses import replace
import pytest
from bike_sim.sim.research.sensors import SensorConfig, SensorObservation, SensorPipeline


def sample(t):
    return SensorObservation(time_s=t, source_time_s=t, valid=True,
        specific_force_body_mps2=(0., 0., 9.81), pitch_rate_up_rad_s=t,
        front_wheel_rad_s=2., rear_wheel_rad_s=2., crank_rad_s=1.,
        motor_torque_nm=0., human_torque_nm=0.)


def test_delay_and_repeated_reads_are_causal_and_pure():
    p = SensorPipeline(SensorConfig(latency_s=.02), seed=3)
    p.reset(sample(0.))
    assert not p.read(0.).valid
    for t in (.01, .02, .03):
        p.push(sample(t))
    a = p.read(.03)
    assert a.source_time_s == pytest.approx(.01)
    assert a == p.read(.03)
    assert a.valid
    with pytest.raises(ValueError):
        p.push(sample(.03))


def test_noise_reset_is_reproducible_and_observation_has_no_privileged_channels():
    p = SensorPipeline(SensorConfig(latency_s=0.), seed=17)
    p.reset(sample(0.)); initial = p.read(0.)
    p.push(sample(.01)); p.reset(sample(0.))
    assert p.read(0.) == initial
    assert not hasattr(initial, 'front_load_n')
    assert not hasattr(initial, 'road_pitch_rad')
    assert not hasattr(initial, 'pitch_up_rad')
    assert initial.specific_force_body_mps2 != (0., 0., 9.81)


def test_ideal_sensors_preserve_signed_readings():
    p = SensorPipeline(SensorConfig.ideal(), seed=0)
    p.reset(sample(0.))
    assert p.read(0.) == sample(0.)


@pytest.mark.parametrize('bad', [-.1, float('nan'), True])
def test_bad_latency_rejected(bad):
    with pytest.raises(ValueError):
        SensorConfig(latency_s=bad)


def test_delivery_clock_cannot_go_backwards():
    p = SensorPipeline(SensorConfig.ideal())
    p.reset(sample(0.))
    p.read(.02)
    with pytest.raises(ValueError, match='backwards'):
        p.read(.01)
    assert p.read(.02).time_s == .02
