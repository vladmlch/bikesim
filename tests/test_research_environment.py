import json
import numpy as np
import pytest
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.physical_config import TireBackendConfig
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.sim.research.environment import ExperimentConfig, ResearchEnvironment
from bike_sim.sim.research.sensors import SensorConfig
from bike_sim.terrain import get_preset


@pytest.fixture(scope='module')
def plant():
    return RideSimulation(track=get_preset('flat'), rider='lumped',
        physics_config=SimulationPhysicsConfig('physical', drive_mode='crank_effort',
            tires=TireBackendConfig(backend='compliant_2d', surface_mode='track')))


def test_control_delay_hold_duration_recording_and_reset(plant, tmp_path):
    env = ResearchEnvironment(plant, ExperimentConfig(duration_s=.02,
        actuator_delay_s=.005, record_decimation=1), SensorConfig.ideal())
    initial = plant.data.qpos.copy()
    command = RideControl(motor_torque_nm=60., human_torque_nm=0.)
    result = env.step(command)
    assert not result.terminated and not result.truncated
    assert plant.steps == 20
    assert result.truth.time_s == pytest.approx(.0095)
    assert result.observation.source_time_s == pytest.approx(.0095)
    for sample in env.recorder.samples:
        expected = 0. if sample.time_s < .005-1e-12 else 60.
        assert sample.channels['control']['motor_torque_nm'] == expected
        assert 'rear_drive' not in sample.forces
    assert plant.cruise.torque_nm == 0.
    assert plant.physics_config.pitch_assist is False
    env.step(command)
    assert env.done and env.truncated and env.reason == 'duration'
    assert env.tracker.metrics['duration_s'] == pytest.approx(.02)
    with pytest.raises(RuntimeError):
        env.step(command)
    env.save(tmp_path/'run')
    summary = json.loads((tmp_path/'run'/'summary.json').read_text())
    assert summary['research']['sensor_config']['latency_s'] == 0.
    assert summary['research']['actuator_delay_steps'] == 10
    assert (tmp_path/'run'/'commands_applied.jsonl').is_file()
    saved = plant.data.qpos.copy()
    env.reset()
    np.testing.assert_allclose(plant.data.qpos, initial, atol=1e-10)
    env.step(command); env.step(command)
    np.testing.assert_allclose(plant.data.qpos, saved, atol=1e-10)


def test_nonintegral_timing_is_rejected(plant):
    with pytest.raises(ValueError, match='integer'):
        ResearchEnvironment(plant, ExperimentConfig(control_period_s=.0007))


def test_numerical_quality_failure_is_latched_and_saved(plant, monkeypatch, tmp_path):
    from bike_sim.sim.research.quality import EnergyQuality
    import bike_sim.sim.research.environment as module
    plant.reset()
    env = ResearchEnvironment(plant, ExperimentConfig(duration_s=.02), SensorConfig.ideal())
    monkeypatch.setattr(module, 'energy_quality', lambda *args, **kwargs: EnergyQuality(.8, 0., False))
    result = env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.))
    assert result.physics_steps == 1
    assert result.truncated and not result.terminated
    assert result.reason == 'numerical_quality' and not result.numerically_valid
    with pytest.raises(RuntimeError, match='episode has ended'):
        env.step(RideControl())
    env.save(tmp_path/'invalid')
    report = json.loads((tmp_path/'invalid'/'summary.json').read_text())
    assert report['research']['numerically_valid'] is False
    assert report['research']['max_energy_residual_ratio'] == .8
    env.reset()
    assert not env.done and env.numerically_valid and env.max_energy_residual_ratio == 0.
