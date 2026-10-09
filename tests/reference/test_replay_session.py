"""Checked replay regression sources; the test runner is never invoked implicitly."""
from hashlib import sha256
import json
import numpy as np
import pytest
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.research.replay import replay_episode
from bike_sim.sim.research.replay_session import ReplaySession
from bike_sim.sim.research.replay_viewer import replay_exit_code


def record_prefix(env, path, count=2):
    for _ in range(count):
        env.step(RideControl(0.))
    if count:
        env.stop()
    env.save(path)
    return path


def reseal(path, name):
    manifest = json.loads((path/'replay.json').read_text())
    manifest['file_sha256'][name] = sha256((path/name).read_bytes()).hexdigest()
    (path/'replay.json').write_text(json.dumps(manifest, sort_keys=True)+'\n')


@pytest.mark.parametrize('count', [0, 2])
def test_python_headless_and_incremental_replay_share_checks(environment_factory, tmp_path, count):
    path = record_prefix(environment_factory(), tmp_path/'episode', count)
    expected = replay_episode(path)
    session = ReplaySession(path)
    try:
        if count:
            assert session.advance(wall_budget_s=0.) is False
            assert session.advance(wall_budget_s=0.) is False
            assert session.step == 0
            assert len(session._env.commands_requested) == 1
        while not session.advance(target_step=session.step+1):
            assert not session.done
        assert session.report() == expected
        assert replay_exit_code(session) == 0
        old = session.snapshot()
        held = old.integration_state.copy()
        session.restart()
        assert session.step == 0
        assert session.snapshot().generation == old.generation+1
        assert not session.done
        with pytest.raises(RuntimeError, match='incomplete'):
            session.report()
        np.testing.assert_array_equal(old.integration_state, held)
    finally:
        session.close()


def test_early_close_never_finishes_a_pending_replay(environment_factory, tmp_path):
    path = record_prefix(environment_factory(), tmp_path/'episode')
    session = ReplaySession(path)
    session.advance(target_step=1)
    assert session.step == 1
    assert session.pending
    assert replay_exit_code(session) == 2
    frame = session.snapshot()
    session.close()
    assert frame.step == 1
    with pytest.raises(RuntimeError, match='incomplete'):
        session.report()


def test_last_frame_remains_unverified_until_final_state_comparison(environment_factory, tmp_path):
    path = record_prefix(environment_factory(), tmp_path/'episode')
    with np.load(path/'states.npz', allow_pickle=False) as stored:
        initial, final = stored['initial'].copy(), stored['final'].copy()
    final[1] += .01
    np.savez(path/'states.npz', initial=initial, final=final)
    reseal(path, 'states.npz')
    session = ReplaySession(path)
    try:
        assert session.advance() is False
        verified = session.snapshot()
        with pytest.raises(ValueError, match='final integration state'):
            session.advance()
        assert session.snapshot() is verified
        assert replay_exit_code(session) == 1
        with pytest.raises(RuntimeError, match='incomplete'):
            session.report()
    finally:
        session.close()


def test_recorded_policy_reference_is_never_loaded(environment_factory, tmp_path, monkeypatch):
    import bike_sim.sim.research.policy_session as policies
    env = environment_factory()
    env.run_metadata['policy'] = dict(reference='recorded_untrusted_missing_module:factory')
    path = record_prefix(env, tmp_path/'episode')
    def forbidden(*args):
        raise AssertionError('replay loaded a recorded policy')
    monkeypatch.setattr(policies, 'load_policy', forbidden)
    assert replay_episode(path)['passed']


def test_identity_mismatch_rejected_before_plant_construction(monkeypatch, tmp_path):
    import bike_sim.sim.research.replay as replay
    import bike_sim.sim.ride_sim as plant
    import bike_sim.native.artifact as artifact
    monkeypatch.setattr(replay, '_check_runtime', lambda summary: None)
    monkeypatch.setattr(artifact, 'execution_provenance', lambda backend: {'backend': backend, 'stamp': 'current'})
    def forbidden(*args, **kwargs):
        raise AssertionError('constructed a plant before rejecting identity')
    monkeypatch.setattr(plant, 'RideSimulation', forbidden)
    with pytest.raises(ValueError, match='execution identity'):
        replay._rebuild(tmp_path, {}, {'schema_version': 2,
            'execution': {'backend': 'native', 'stamp': 'different'}})
