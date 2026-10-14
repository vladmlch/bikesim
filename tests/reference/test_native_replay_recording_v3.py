"""Native new-container replay shares the Python/legacy verifier."""
from bike_sim.sim.research.replay import replay_episode
from bike_sim.sim.research.replay_session import ReplaySession
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
