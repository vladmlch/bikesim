import numpy as np
import pytest
from native_loader import load_native
from test_replay_recording_v3 import save_run
from test_replay_session import record_prefix
from bike_sim.sim.research.replay import replay_episode
from bike_sim.sim.research.replay_session import ReplaySession
from pathlib import Path
import sys

_TESTS_DIR = str(Path(__file__).resolve().parents[1])
sys.path.insert(0, _TESTS_DIR)
from test_recording_platform_compatibility import (  # noqa: E402
    _foreign_system, _patch_execution)
sys.path.remove(_TESTS_DIR)


# The selected-artifact import doubles as the sanitizer-runtime check.
bike_native = load_native()


def test_native_recording_replays_same_backend_and_restarts(environment_pair, tmp_path):
    _, native = environment_pair()
    path = record_prefix(native, tmp_path/'native')
    expected = replay_episode(path)
    assert expected['backend'] == 'native'
    assert expected['passed']
    session = ReplaySession(path)
    try:
        session.advance(wall_budget_s=0.)
        assert session.pending and session.step == 0
        assert session._env.commands_requested == []
        assert session._env.last_requested['step'] == 0
        while not session.advance(target_step=session.step+1):
            pass
        assert session.report() == expected
        final = session.snapshot()
        before = final.integration_state.copy()
        session.restart()
        assert session.snapshot().generation == final.generation+1
        assert session.backend == 'native'
        while not session.advance():
            pass
        assert session.report() == expected
        np.testing.assert_array_equal(final.integration_state, before)
    finally:
        session.close()


def test_native_replay_rejects_a_missing_selected_extension(environment_pair, tmp_path, monkeypatch):
    _, native = environment_pair()
    path = record_prefix(native, tmp_path/'native')
    empty = tmp_path/'empty-build'
    empty.mkdir()
    monkeypatch.setenv('BIKE_NATIVE_BUILD_PATH', str(empty))
    with pytest.raises(RuntimeError, match='empty-build') as failure:
        ReplaySession(path)
    message = str(failure.value)
    assert 'tools/dev.py doctor --build' in message
    assert 'tools/dev.py configure --build' in message
    assert 'tools/dev.py build --build' in message


def test_native_session_gate_invoked_with_the_recorded_identity(
        environment_pair, tmp_path, monkeypatch):
    """T22: ReplaySession construction runs the execution-compatibility gate
    against the recorded identity before any plant exists, then again at the
    final boundary — the checked --viewer session shares exactly this path."""
    import bike_sim.native.execution_identity as identity
    _, native = environment_pair(capture_mode='full')
    path = save_run(native, tmp_path)
    calls = []
    original = identity.require_execution_compatible

    def spy(recorded, current):
        calls.append(recorded)
        return original(recorded, current)

    monkeypatch.setattr(identity, 'require_execution_compatible', spy)
    session = ReplaySession(path)
    try:
        assert calls and all(call['backend'] == 'native' for call in calls)
        construction_calls = len(calls)
        while not session.advance():
            pass
        report = session.report()
        assert report['passed'] and report['backend'] == 'native'
        assert report['episode_id'] == 0
        assert len(calls) == construction_calls + 1
    finally:
        session.close()


def test_native_session_rejects_foreign_identity_before_any_plant(
        environment_pair, tmp_path, monkeypatch):
    """T22: a transported recording names the mismatched field; no plant, no
    backend wiring and no physics step exist at the point of rejection."""
    import bike_sim.native.research as native_research
    import bike_sim.sim.ride_sim as plant
    import mujoco
    _, native = environment_pair(capture_mode='full')
    path = save_run(native, tmp_path)
    _patch_execution(path, _foreign_system)

    def forbidden(*args, **kwargs):
        raise AssertionError('foreign recording reached replay machinery')

    # Loading the selected artifact is itself part of the identity gate; the
    # plant, the native runtime wiring and physics stepping are not.
    monkeypatch.setattr(plant, 'RideSimulation', forbidden)
    monkeypatch.setattr(native_research, 'create_native_research', forbidden)
    monkeypatch.setattr(mujoco, 'mj_step', forbidden)
    with pytest.raises(ValueError,
                       match=r'execution\.build_context\.platform\.system'):
        ReplaySession(path)
