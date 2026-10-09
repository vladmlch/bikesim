from bike_sim.sim.research.policy_session import PolicySession
from bike_sim.sim.research.viewer import advance_playback, stop_at_boundary
from .test_research_frontend import CountingPolicy


def test_native_partial_ticks_latch_policy_until_external_boundary(environment_pair):
    _, env = environment_pair()
    policy = CountingPolicy()
    session = PolicySession(env, policy)
    advance_playback(session, 1)
    assert env.sim.steps == 1 and session.pending
    for _ in range(3):
        session.advance_pending(wall_budget_s=0.)
    assert env.sim.steps == 1 and policy.calls == 1
    session.set_brakes(front_brake_demand=.5, rear_brake_demand=.5)
    stop_at_boundary(session)
    assert env.sim.steps == env.control_steps
    assert policy.calls == len(env.commands_requested) == 1
    assert env.commands_requested[0]['front_brake_demand'] == 0.
    assert env.reason == 'operator_stop'
