"""Replay of a golden episode on the same code is bitwise identical."""
import pytest


def _env(tmp_path):
    """Fresh args per call: a parsed namespace must not be reused across
    make_environment runs."""
    from test_pinned_topology import _pinned_config
    from bike_sim.cli import research as research_cli
    args = research_cli.parser().parse_args([
        '--physics-config', str(_pinned_config(tmp_path)),
        '--track-file', 'examples/research/rough_uphill_savage.toml',
        '--duration', '1', '--dt', '.00125', '--out', str(tmp_path/'out'),
        # Non-strict monitor: record first_failure instead of dying on it.
        '--diagnostic-model-limits'])
    return research_cli.make_environment(args)


@pytest.mark.slow
def test_replay_matches_golden_bitwise(tmp_path):
    from tools.golden_episode import capture_episode
    from tools.episode_compare import replay_episode, compare_rows
    from bike_sim.sim.ride.control import RideControl
    env = _env(tmp_path)
    ep = capture_episode(env, 50, RideControl(human_torque_nm=35.))
    env2 = _env(tmp_path)
    cand = replay_episode(env2, ep)
    assert compare_rows(ep.rows, cand) == []
    assert env.sim.physical.reference_monitor.first_failure \
        == env2.sim.physical.reference_monitor.first_failure


@pytest.mark.slow
def test_compare_episode_on_saved_artifact(tmp_path):
    """Full surface: save + load + replay + first_failure — the entry point
    native tests will call."""
    from tools.golden_episode import capture_episode, save
    from tools.episode_compare import compare_episode
    from bike_sim.sim.ride.control import RideControl
    env = _env(tmp_path)
    ep = capture_episode(env, 50, RideControl(human_torque_nm=35.))
    save(ep, tmp_path/'golden', env.sim.model)
    env2 = _env(tmp_path)
    assert compare_episode(tmp_path/'golden', env2) == []


def test_compare_reports_first_divergence():
    from tools.episode_compare import compare_rows
    a = [{'x': 1.0, 'y': 2.0}]; b = [{'x': 1.0, 'y': 2.0 + 1e-9}]
    diffs = compare_rows(a, b)
    assert diffs and 'step=0' in diffs[0] and 'y' in diffs[0]
