from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.research.policy_session import PolicySession
from bike_sim.sim.research.viewer import advance_playback, stop_at_boundary


class CountingPolicy:
    def __init__(self):
        self.calls = 0
    def reset(self, seed):
        self.calls = 0
    def act(self, observation, demand_nm):
        self.calls += 1
        return RideControl(0.)


def test_partial_ticks_call_policy_once_and_stop_does_not_start_another(environment_factory):
    env = environment_factory()
    policy = CountingPolicy()
    session = PolicySession(env, policy)
    assert advance_playback(session, 1) is None
    assert policy.calls == 1
    assert session.pending and env.sim.steps == 1
    assert advance_playback(session, 2) is None
    assert env.sim.steps == 2 and policy.calls == 1
    session.pause()
    stop_at_boundary(session)
    assert not session.pending
    assert env.sim.steps == env.control_steps
    assert policy.calls == len(env.commands_requested) == len(env.trace) == 1
    assert env.reason == 'operator_stop'
