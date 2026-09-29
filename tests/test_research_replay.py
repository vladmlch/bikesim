import json
import shutil
import numpy as np
import pytest
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.physical_config import TireBackendConfig
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.research.environment import ExperimentConfig, ResearchEnvironment
from bike_sim.sim.research.sensors import SensorConfig
from bike_sim.sim.research.replay import rebuild_environment, replay_episode, validate_recording
from bike_sim.terrain import get_preset


@pytest.fixture(scope='module')
def recording(tmp_path_factory):
    sim = RideSimulation(track=get_preset('flat'), rider='lumped',
        physics_config=SimulationPhysicsConfig('physical', drive_mode='crank_effort',
            tires=TireBackendConfig(backend='compliant_2d', surface_mode='track')))
    env = ResearchEnvironment(sim, ExperimentConfig(duration_s=.01, control_period_s=.005,
        actuator_delay_s=.001), SensorConfig(sample_period_s=.001, latency_s=.001))
    env.step(RideControl(motor_torque_nm=10., human_torque_nm=0.))
    env.step(RideControl(motor_torque_nm=0., motor_limit_nm=0.), front_brake_demand=.1)
    path = tmp_path_factory.mktemp('replay')/'run'
    env.save(path)
    return path


def test_saved_controls_states_sensors_and_truth_replay(recording):
    summary, manifest = validate_recording(recording)
    assert 'transitions.jsonl' in manifest['file_sha256']
    assert manifest['start_x_m'] == 2.
    report = replay_episode(recording)
    assert report['passed']
    assert report['replayed_control_steps'] == 2
    assert report['final_state_max_abs_error'] < 1e-9


def test_integrity_failure_happens_before_constructing_a_plant(recording, tmp_path):
    copied = tmp_path/'bad'
    shutil.copytree(recording, copied)
    with (copied/'commands_requested.jsonl').open('a') as stream:
        stream.write('{}\n')
    with pytest.raises(ValueError, match='checksum'):
        rebuild_environment(copied)


def test_unsupported_manifest_version_is_rejected(recording, tmp_path):
    copied = tmp_path/'bad'
    shutil.copytree(recording, copied)
    manifest = json.loads((copied/'replay.json').read_text())
    manifest['schema_version'] = 999
    (copied/'replay.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='schema'):
        validate_recording(copied)
