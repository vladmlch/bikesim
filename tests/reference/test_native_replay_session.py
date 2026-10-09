import numpy as np
from bike_sim.sim.research.replay import replay_episode
from bike_sim.sim.research.replay_session import ReplaySession
from .test_replay_session import record_prefix


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
        assert len(session._env.commands_requested) == 1
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
