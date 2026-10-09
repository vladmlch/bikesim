"""The same external policy lifecycle is exercised against both backends."""
import pytest
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.research.policy_session import PolicySession


class CountingPolicy:
    def __init__(self):
        self.seeds = []
        self.calls = []

    def reset(self, seed):
        self.seeds.append(seed)

    def act(self, observation, demand_nm):
        self.calls.append((observation, demand_nm))
        return RideControl(0.)


@pytest.fixture(params=['python', 'native'])
def policy_environment(request, environment_factory):
    if request.param == 'python':
        return environment_factory()
    pair = request.getfixturevalue('environment_pair')
    return pair()[1]


def test_one_act_per_window_and_queued_pause_brakes(policy_environment):
    env = policy_environment
    policy = CountingPolicy()
    session = PolicySession(env, policy)
    assert policy.seeds == [env.seed]
    session.begin_advance()
    for _ in range(4):
        assert session.advance_pending(wall_budget_s=0.) is None
    assert len(policy.calls) == 1
    assert len(env.commands_requested) == 1
    session.pause()
    session.set_brakes(front_brake_demand=.2, rear_brake_demand=.1)
    assert not session.paused
    result = session.advance_pending()
    assert result is not None and session.paused
    assert len(policy.calls) == 1
    assert env.commands_requested[0]['front_brake_demand'] == 0.
    with pytest.raises(RuntimeError):
        session.begin_advance()
    assert len(policy.calls) == 1
    session.resume()
    session.advance()
    assert len(policy.calls) == 2
    assert env.commands_requested[1]['front_brake_demand'] == .2
    assert env.commands_requested[1]['rear_brake_demand'] == .1
    assert env.run_metadata['operator_intervention']


def test_stop_and_reset_are_deferred_to_external_boundary(policy_environment):
    env = policy_environment
    policy = CountingPolicy()
    session = PolicySession(env, policy)
    session.begin_advance()
    assert session.advance_pending(wall_budget_s=0.) is None
    session.stop()
    assert not env.done
    session.advance_pending()
    assert env.reason == 'operator_stop' and not session.pending
    session.reset(seed=47)
    assert env.sim.steps == 0 and not env.done
    assert policy.seeds == [19, 47]
    session.begin_advance()
    session.reset(seed=53)
    session.set_brakes(rear_brake_demand=.3)
    assert env.seed == 47
    previous = session.advance_pending()
    assert previous.physics_steps == env.control_steps
    assert env.seed == 53 and env.sim.steps == 0
    assert policy.seeds == [19, 47, 53]
    session.advance()
    assert env.commands_requested[0]['rear_brake_demand'] == .3
    assert len(policy.calls) == 3


def test_invalid_operator_values_do_not_call_policy(policy_environment):
    env = policy_environment
    policy = CountingPolicy()
    session = PolicySession(env, policy)
    with pytest.raises(ValueError):
        session.begin_advance(front_brake_demand=True)
    with pytest.raises(ValueError):
        session.set_brakes(rear_brake_demand=2.)
    with pytest.raises(ValueError):
        session.reset(seed=True)
    assert policy.calls == []
    assert not env.done and not session.pending


def test_policy_failure_has_no_manufactured_transition(policy_environment):
    class BrokenPolicy(CountingPolicy):
        def act(self, observation, demand_nm):
            raise RuntimeError('policy sentinel')
    env = policy_environment
    session = PolicySession(env, BrokenPolicy())
    with pytest.raises(RuntimeError, match='policy sentinel'):
        session.begin_advance()
    assert env.terminated and env.reason == 'policy_error'
    assert env.error == 'RuntimeError: policy sentinel'
    assert env.sim.steps == 0
    assert env.commands_requested == [] and env.trace == []


def test_policy_cannot_smuggle_rider_inputs(policy_environment):
    class RiderPolicy(CountingPolicy):
        def act(self, observation, demand_nm):
            return RideControl(0., human_torque_nm=0.)
    env = policy_environment
    session = PolicySession(env, RiderPolicy())
    with pytest.raises(ValueError, match='rider inputs'):
        session.advance()
    assert env.reason == 'policy_error'
    assert not session.pending
