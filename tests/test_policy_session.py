from types import SimpleNamespace

import pytest

from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.research.policy_session import PolicySession, load_policy


class FakeEnvironment:
    def __init__(self):
        self.seed = 17
        self.observation = object()
        self.demand_nm = None
        self.done = False
        self.commands = []
        self.sim = SimpleNamespace(time_s=0.)
        self.run_metadata = {}

    def step(self, control, **brakes):
        self.commands.append((control, brakes))
        self.sim.time_s += .01
        return 'advanced'

    def reset(self, *, seed=None):
        if seed is not None:
            self.seed = seed
        self.commands.clear()
        self.run_metadata = {}
        self.sim.time_s = 0.
        return self.observation


class RecordingPolicy:
    def __init__(self, control=None):
        self.control = RideControl() if control is None else control
        self.resets = []
        self.inputs = []

    def reset(self, seed):
        self.resets.append(seed)

    def act(self, observation, demand_nm):
        self.inputs.append((observation, demand_nm))
        return self.control


def test_session_preserves_assist_and_rider_and_resets_policy():
    env = FakeEnvironment()
    policy = RecordingPolicy(RideControl(motor_limit_nm=40.))
    session = PolicySession(env, policy)
    assert session.advance(rear_brake_demand=.4) == 'advanced'
    assert policy.inputs == [(env.observation, None)]
    command, brakes = env.commands[0]
    assert command.motor_torque_nm is None
    assert command.human_torque_nm is None
    assert brakes['rear_brake_demand'] == .4
    assert env.run_metadata['operator_intervention']
    session.reset(seed=23)
    assert policy.resets == [17, 23]
    assert session.operator_events == []
    assert not env.run_metadata['operator_intervention']


@pytest.mark.parametrize('control', [
    RideControl(human_torque_nm=0.), RideControl(rider_enabled=False)])
def test_motor_policy_cannot_change_rider(control):
    env = FakeEnvironment()
    session = PolicySession(env, RecordingPolicy(control))
    with pytest.raises(ValueError, match='rider'):
        session.advance()
    assert env.commands == []
    assert env.reason == 'policy_error'


def test_wrong_policy_return_is_rejected_before_advancing():
    env = FakeEnvironment()
    session = PolicySession(env, RecordingPolicy('invalid'))
    with pytest.raises(ValueError, match='RideControl'):
        session.advance()
    assert env.sim.time_s == 0.


def test_policy_exception_stops_the_episode():
    class FailingPolicy(RecordingPolicy):
        def act(self, observation, demand_nm):
            raise RuntimeError('broken policy')
    env = FakeEnvironment()
    session = PolicySession(env, FailingPolicy())
    with pytest.raises(RuntimeError, match='broken policy'):
        session.advance()
    assert env.reason == 'policy_error'
    assert env.terminated
    assert env.commands == []


def test_loader_constructs_new_policy_per_run_and_preserves_none_demand():
    reference = 'bike_sim.sim.research.policies:passthrough_factory'
    first, second = load_policy(reference), load_policy(reference)
    assert first is not second
    assert first.act(object(), None) == RideControl()
    assert first.act(object(), 30.).motor_torque_nm == 30.


@pytest.mark.parametrize('reference', ['bad', 'missing_package:factory',
                                      'bike_sim.sim.research.policies:missing_factory'])
def test_loader_explains_bad_references(reference):
    with pytest.raises(ValueError, match='policy'):
        load_policy(reference)
