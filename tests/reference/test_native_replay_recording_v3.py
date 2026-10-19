"""Native new-container replay shares the Python/legacy verifier."""
import pytest

from bike_sim.recording.reader import RecordingError, open_recording
from bike_sim.recording.replay_input import load_replay_input
from bike_sim.sim.research.playback import load_playback
from bike_sim.sim.research.recording_lifecycle import ResearchRun
from bike_sim.sim.research.replay import replay_episode
from bike_sim.sim.research.replay_session import ReplaySession
from bike_sim.sim.ride.control import RideControl
from native_loader import load_native
from test_replay_recording_v3 import save_run

# The selected-artifact import doubles as the sanitizer-runtime check.
bike_native = load_native()


def test_native_replay_v3_and_suffix_capture(environment_pair, tmp_path):
    _, native = environment_pair(capture_mode='full')
    path = save_run(native, tmp_path)
    expected = replay_episode(path)
    assert expected['passed'] and expected['backend'] == 'native'
    session = ReplaySession(path)
    try:
        while not session.advance(target_step=session.step + 1):
            assert not session.done
        assert session.report() == expected
        assert not session._env.commands_requested
        assert not session._env.commands_applied
        assert not session._env.trace
        session.restart()
        while not session.advance():
            pass
        assert session.report() == expected
    finally:
        session.close()


def test_native_failed_recording_stays_analyzable_and_playable(environment_pair, tmp_path):
    """T22: a policy-failure native run is failure evidence — analyzable and
    frame-playable, but never a completed replayable prefix."""
    _, native = environment_pair(capture_mode='full')
    launch = ResearchRun(native, tmp_path / 'failed', enabled=True, mode='test')
    native.step(RideControl(0.))
    native.fail_policy(RuntimeError('policy exploded'))
    path = launch.finish()
    with pytest.raises(RecordingError, match='not completed replay'):
        load_replay_input(path)
    with pytest.raises(RecordingError):
        replay_episode(path)
    playback = load_playback(path)
    assert playback.verified is False and playback.times_s.size >= 2
    with open_recording(path) as recording:
        assert recording.rows(
            'SELECT count(*) AS n FROM physics WHERE episode_id=0'
        )[0]['n'] == playback.times_s.size


def test_native_playback_never_loads_artifact_or_identity(
        environment_pair, tmp_path, monkeypatch):
    """T22: native recorded frames play without the extension, the runtime
    provenance probe or the compatibility gate — playback is not verification."""
    import bike_sim.native.artifact as artifact
    import bike_sim.native.execution_identity as identity
    import bike_sim.native.research as native_research
    _, native = environment_pair(capture_mode='full')
    path = save_run(native, tmp_path)

    def forbidden(*args, **kwargs):
        raise AssertionError('verify machinery reached in the playback path')

    monkeypatch.setattr(artifact, 'load_native_extension', forbidden)
    monkeypatch.setattr(artifact, 'execution_provenance', forbidden)
    monkeypatch.setattr(identity, 'require_execution_compatible', forbidden)
    monkeypatch.setattr(native_research, 'create_native_research', forbidden)
    playback = load_playback(path)
    assert playback.verified is False
    with open_recording(path) as recording:
        rows = recording.rows(
            'SELECT count(*) AS n FROM physics WHERE episode_id=?',
            [playback.episode_id])
    assert rows[0]['n'] == playback.times_s.size
    # The frames are the recorded physics rows — same count, same state width.
    assert playback.qpos.shape == (playback.times_s.size, playback.sim.model.nq)
    assert playback.qvel.shape == (playback.times_s.size, playback.sim.model.nv)
