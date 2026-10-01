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


def test_replay_restores_time_varying_demand_and_reset_seed(tmp_path):
    from bike_sim.sim.research.demand import DemandProgram
    sim = RideSimulation(track=get_preset('flat'), rider='lumped',
        physics_config=SimulationPhysicsConfig('physical', drive_mode='crank_effort',
            tires=TireBackendConfig(backend='compliant_2d', surface_mode='track')))
    demand = DemandProgram(((0., 10.), (.01, 30.)))
    env = ResearchEnvironment(sim, ExperimentConfig(duration_s=.02, actuator_delay_s=0.),
                              SensorConfig.ideal(), demand=demand)
    for seed in (17, 23):
        env.reset(seed=seed)
        while not env.done:
            env.step(RideControl(motor_torque_nm=env.demand_nm, human_torque_nm=0.))
        destination = env.save(tmp_path / str(seed))
        rebuilt = rebuild_environment(destination)
        assert rebuilt.demand.to_dict() == demand.to_dict()
        assert rebuilt.seed == seed
        assert replay_episode(destination)['passed']


def test_replay_bundles_joint_envelope_and_applies_rider_program_once(tmp_path):
    from pathlib import Path
    from bike_sim.physics.resolution import load_physics_config
    from bike_sim.physics.rider_posture import RiderPosture
    from bike_sim.sim.research.rider_program import RiderKeyframe, RiderProgram
    source = Path('examples/research/rider_joint_envelope_synthetic.json')
    envelope = tmp_path / 'original-envelope.json'
    envelope.write_bytes(source.read_bytes())
    physics = load_physics_config('examples/research/viewer_physics_fast.toml', {
        'timestep_s': .000625,
        'articulated': {'joint_envelope_path': str(envelope)},
    })
    sim = RideSimulation(track=get_preset('flat'), rider='articulated_planar', physics_config=physics)
    program = RiderProgram((
        RiderKeyframe(0., RiderPosture(), human_torque_nm=0.),
        RiderKeyframe(.01, RiderPosture(torso_lean_rad=.02), human_torque_nm=2.),
    ))
    env = ResearchEnvironment(sim, ExperimentConfig(duration_s=.02),
        SensorConfig(sample_period_s=.005), rider_program=program)
    while not env.done:
        env.step(RideControl(motor_torque_nm=0.))
    destination = env.save(tmp_path / 'portable')
    envelope.write_text('{}')
    assert replay_episode(destination)['passed']
    assert rebuild_environment(destination).rider_program.to_dict() == program.to_dict()
    bundled = destination / 'rider_joint_envelope.json'
    bundled.write_text('{}')
    with pytest.raises(ValueError, match='checksum'):
        rebuild_environment(destination)


def test_cached_initial_equilibrium_does_not_change_first_torque_measurement(tmp_path, monkeypatch):
    from bike_sim.cli.research import parser, make_environment
    monkeypatch.setenv('BIKE_SIM_EQUILIBRIUM_CACHE_DIR', str(tmp_path / 'equilibria'))
    env = make_environment(parser().parse_args([
        '--physics-config', 'examples/research/viewer_physics_fast.toml',
        '--track-file', 'examples/research/flat.toml', '--ideal-sensors',
        '--duration', '.01', '--assist']))
    first = env.observation
    assert not env.sim.equilibrium['cache_hit']
    env.reset()
    assert env.sim.equilibrium['cache_hit']
    assert env.observation == first
