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
            assert session._env.commands_requested == []
            assert session._env.last_requested['step'] == 0
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


@pytest.mark.parametrize('name,index,key,label', [
    ('commands_applied.jsonl', 0, 'step', 'applied commands'),
    ('transitions.jsonl', 0, 'physics_steps', 'transition'),
    ('observations.jsonl', 1, 'time_s', 'observation')])
def test_resealed_semantic_tamper_still_fails_verification(
        environment_factory, tmp_path, name, index, key, label):
    """A checksum-honest edit must still fail the semantic comparison."""
    path = record_prefix(environment_factory(), tmp_path/'episode')
    lines = (path/name).read_text().splitlines()
    row = json.loads(lines[index])
    row[key] += 1
    lines[index] = json.dumps(row)
    (path/name).write_text('\n'.join(lines)+'\n')
    reseal(path, name)
    session = ReplaySession(path)
    try:
        with pytest.raises(ValueError, match=label):
            while not session.advance():
                pass
        assert not session.done
        assert replay_exit_code(session) == 1
        with pytest.raises(RuntimeError, match='incomplete'):
            session.report()
    finally:
        session.close()


def _execution_identity(backend='native', **overrides):
    record = dict(backend=backend, runtime_schema=1, python_source_sha256='a'*64,
        native_source_sha256='b'*64 if backend == 'native' else None,
        extension_sha256='c'*64 if backend == 'native' else None,
        mujoco_version='3.12.0',
        mujoco_library_sha256='d'*64 if backend == 'native' else None,
        build_context=dict(build_type='Release', compiler_id='AppleClang',
            compiler_version='21.0.0', numerical_flags='-O2 -ffp-contract=off')
            if backend == 'native' else None)
    record.update(overrides)
    return record


def test_identity_mismatch_rejected_before_plant_construction(monkeypatch, tmp_path):
    import bike_sim.sim.research.replay as replay
    import bike_sim.sim.ride_sim as plant
    import bike_sim.native.artifact as artifact
    monkeypatch.setattr(replay, '_check_runtime', lambda summary: None)
    monkeypatch.setattr(artifact, 'execution_provenance',
                        lambda backend: _execution_identity(mujoco_version='3.12.1'))
    def forbidden(*args, **kwargs):
        raise AssertionError('constructed a plant before rejecting identity')
    monkeypatch.setattr(plant, 'RideSimulation', forbidden)
    with pytest.raises(ValueError, match='execution identity mismatch.*mujoco_version'):
        replay._rebuild(tmp_path, {}, {'schema_version': 2,
            'execution': _execution_identity()})


def test_identity_recheck_failure_advances_no_physics(environment_factory, tmp_path, monkeypatch):
    """A playback-time identity drift surfaces at the final boundary with the
    named field — and once raised, the session commits no further physics."""
    from copy import deepcopy
    import bike_sim.native.artifact as artifact
    env = environment_factory()
    path = record_prefix(env, tmp_path/'episode')
    drifted = deepcopy(env.execution)
    drifted['mujoco_version'] = '0.0.0'
    session = ReplaySession(path)
    try:
        monkeypatch.setattr(artifact, 'execution_provenance',
                            lambda backend: deepcopy(drifted))
        # The per-command transitions still verify; the recheck fires at the
        # terminal boundary — the failure must not advance the step count.
        with pytest.raises(ValueError, match='execution identity mismatch.*mujoco_version'):
            while not session.advance(target_step=session.step+1):
                pass
        steps_at_failure = session._env.sim.steps
        assert not session.done
        assert session.error is not None
        with pytest.raises(RuntimeError, match='replay stopped after mismatch'):
            session.advance()
        assert session._env.sim.steps == steps_at_failure
        with pytest.raises(RuntimeError, match='incomplete'):
            session.report()
    finally:
        session.close()
