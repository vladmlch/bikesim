"""New/legacy inputs must pass the same complete physical replay verifier."""
from hashlib import sha256

import numpy as np
import pytest

from bike_sim.recording.json_codec import dumps, loads
from bike_sim.recording.replay_input import load_replay_input
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.research.recording_lifecycle import ResearchRun
from bike_sim.sim.research.replay import replay_episode
from bike_sim.sim.research.replay_session import ReplaySession
from test_recording_storage import capture_rows


def save_run(env, tmp_path, *, two_episodes=False):
    launch = ResearchRun(env, tmp_path, enabled=True, mode='test')
    env.step(RideControl(0.))
    if two_episodes:
        launch.reset()
        env.step(RideControl(0.))
    env.stop()
    return launch.finish()


def test_new_recording_requires_all_verification_before_report(environment_factory, tmp_path):
    path = save_run(environment_factory(capture_mode='full'), tmp_path)
    data = load_replay_input(path)
    assert data.container_schema_version == 1 and data.replay_contract_version == 3
    assert data.episode_id == 0
    with pytest.raises(ValueError):
        data.final_state.setflags(write=True)
    session = ReplaySession(path)
    try:
        with pytest.raises(RuntimeError, match='incomplete'):
            session.report()
        assert session.advance(wall_budget_s=0.) is False
        assert session._env.capture_mode == 'verify'
        assert not session._env.commands_requested
        while not session.advance():
            pass
        result = session.report()
        assert result['passed'] and result['replay_contract_version'] == 3
        assert result['container_schema_version'] == 1
    finally:
        session.close()
    assert replay_episode(path)['passed']


def test_multi_episode_selection_is_explicit(environment_factory, tmp_path):
    path = save_run(environment_factory(capture_mode='full'), tmp_path, two_episodes=True)
    with pytest.raises(ValueError, match='select --episode'):
        ReplaySession(path)
    for episode_id in (0, 1):
        assert replay_episode(path, episode_id=episode_id)['passed']
    with pytest.raises(ValueError, match='no episode'):
        load_replay_input(path, episode_id=2)


def test_ordinary_recording_does_not_claim_checked_replay(tmp_path):
    path = capture_rows().finalize(tmp_path)
    with pytest.raises(ValueError, match='ordinary physical'):
        load_replay_input(path)


def test_new_replay_does_not_load_recorded_policy(environment_factory, tmp_path, monkeypatch):
    import bike_sim.sim.research.policy_session as policies
    env = environment_factory(capture_mode='full')
    env.run_metadata['policy'] = {'reference': 'absent_untrusted_policy:factory'}
    path = save_run(env, tmp_path)
    def forbidden(*args, **kwargs):
        raise AssertionError('recorded policy was loaded')
    monkeypatch.setattr(policies, 'load_policy', forbidden)
    assert replay_episode(path)['passed']


def test_resealed_final_state_tamper_fails_last_boundary_not_format_loading(environment_factory, tmp_path):
    import duckdb
    path = save_run(environment_factory(capture_mode='full'), tmp_path)
    connection = duckdb.connect(str(path / 'run.duckdb'))
    try:
        state = connection.execute("SELECT state FROM integration_states WHERE phase='final'").fetchone()[0]
        state[1] += .01
        connection.execute("UPDATE integration_states SET state=? WHERE phase='final'", [state])
        connection.execute('CHECKPOINT')
    finally:
        connection.close()
    manifest = loads((path / 'manifest.json').read_text())
    manifest['file_sha256']['run.duckdb'] = sha256((path / 'run.duckdb').read_bytes()).hexdigest()
    (path / 'manifest.json').write_text(dumps(manifest))
    session = ReplaySession(path)
    try:
        frame = session.snapshot()
        with pytest.raises(ValueError, match='final integration state'):
            while not session.advance():
                pass
        assert session.snapshot() is frame
        with pytest.raises(RuntimeError, match='incomplete'):
            session.report()
    finally:
        session.close()


def test_new_checksum_failure_precedes_plant_construction(environment_factory, tmp_path, monkeypatch):
    import bike_sim.sim.research.replay_session as implementation
    path = save_run(environment_factory(capture_mode='full'), tmp_path)
    with (path / 'run.duckdb').open('ab') as stream:
        stream.write(b'not the recorded checksum')
    def forbidden(*args, **kwargs):
        raise AssertionError('constructed a plant before checksum validation')
    monkeypatch.setattr(implementation, '_rebuild_input', forbidden)
    with pytest.raises(ValueError, match='checksum'):
        ReplaySession(path)
