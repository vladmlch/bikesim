from dataclasses import asdict

import numpy as np
import pytest

from bike_sim.cli.research import parser, make_environment
from bike_sim.cli.ride import parse_args, resolve_rider, resolve_track
from bike_sim.sim.research.policy_session import PolicySession
from bike_sim.sim.research.policies import passthrough_factory
from bike_sim.sim.research.viewer import advance_control_ticks, make_ride_session


def run_grouped(groups):
    arguments = parser().parse_args([
        '--scenario', 'flat', '--rider', 'lumped', '--duration', '.04',
        '--demand', '20', '--initial-speed', '0', '--seed', '7'])
    env = make_environment(arguments)
    session = PolicySession(env, passthrough_factory())
    for count in groups:
        advance_control_ticks(session, count)
    return env


def test_render_grouping_does_not_change_physics_or_policy_samples():
    regular = run_grouped([1, 1, 1, 1])
    delayed = run_grouped([0, 3, 0, 1])
    assert regular.commands_applied == delayed.commands_applied
    assert regular.observations == delayed.observations
    assert regular.tracker.metrics == delayed.tracker.metrics
    assert regular.reason == delayed.reason
    np.testing.assert_array_equal(regular.sim.data.qpos, delayed.sim.data.qpos)
    assert regular.sim.physical.research_accounting_valid
    assert delayed.sim.physical.sample is not None


@pytest.mark.parametrize('value', [-1, True, .5])
def test_invalid_tick_counts_are_rejected(value):
    with pytest.raises(ValueError, match='tick count'):
        advance_control_ticks(None, value)


def test_viewer_factory_uses_resolved_physics_and_duration(monkeypatch):
    import bike_sim.sim.research.viewer as module
    from types import SimpleNamespace
    captured = {}
    def capture(**values):
        captured.update(values)
        return SimpleNamespace(seed=17, run_metadata={})
    monkeypatch.setattr(module, 'build_environment', capture)
    args = parse_args([
        '--research', '--physics-config', 'examples/research/viewer_physics_fast.toml',
        '--rider', 'articulated_planar', '--track', 'examples/research/rough_uphill_extreme.toml',
        '--duration', '4', '--seed', '17'])
    session = make_ride_session(resolve_track(args.track), args, resolve_rider(args))
    assert asdict(captured['physics_config']) == asdict(args.resolved_physics)
    assert captured['experiment'].duration_s == 4.
    assert captured['experiment'].record_decimation == 80
    assert captured['sensors'].sample_period_s == .005
    assert captured['demand'] is None
    assert session.policy.act(object(), None).motor_torque_nm is None


def test_research_requires_explicit_physical_effort_mode():
    with pytest.raises(SystemExit):
        parse_args(['--research'])
    with pytest.raises(SystemExit):
        parse_args(['--policy', 'bike_sim.sim.research.policies:zero_factory'])


def test_authored_track_seed_is_independent_of_research_sensor_seed(monkeypatch):
    import bike_sim.cli.ride as cli
    import bike_sim.sim.research.viewer as module
    captured = {}
    def capture(track, arguments, rider):
        captured.update(track=track, seed=arguments.seed)
        return 0
    monkeypatch.setattr(module, 'run_ride_research', capture)
    result = cli.main([
        '--research', '--headless',
        '--physics-config', 'examples/research/viewer_physics_fast.toml',
        '--track', 'examples/research/rough_uphill_extreme.toml', '--seed', '17'])
    assert result == 0
    assert captured['seed'] == 17
    assert captured['track'].name == 'research_rough_uphill_extreme'
