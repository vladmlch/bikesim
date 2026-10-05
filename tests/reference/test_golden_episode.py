"""Golden-episode capture is deterministic and complete."""
from pathlib import Path
import pytest

def _ep(tmp_path, steps=50):
    from bike_sim.cli import research as research_cli
    from test_pinned_topology import _pinned_config
    args = research_cli.parser().parse_args([
        '--physics-config', str(_pinned_config(tmp_path)),
        '--track-file', 'examples/research/rough_uphill_savage.toml',
        '--duration', '1', '--dt', '.00125', '--out', str(tmp_path/'out'),
        # Non-strict monitor: the oracle must observe a full episode and
        # *record* first_failure instead of dying on it (same flag _model uses).
        '--diagnostic-model-limits'])
    env = research_cli.make_environment(args)
    from tools.golden_episode import capture_episode
    from bike_sim.sim.ride.control import RideControl
    return env, capture_episode(env, steps, RideControl(human_torque_nm=35.))

@pytest.mark.slow
def test_capture_is_deterministic(tmp_path):
    (_, a), (_, b) = _ep(tmp_path), _ep(tmp_path)
    assert a.channel_names == b.channel_names and len(a.rows) == len(b.rows)
    for ra, rb in zip(a.rows, b.rows):
        assert ra == rb            # bitwise-equal dicts: same run must replay identically
    assert a.first_failure == b.first_failure

@pytest.mark.slow
def test_saved_artifact_roundtrips(tmp_path):
    env, ep = _ep(tmp_path)
    from tools.golden_episode import load_episode, flatten_row, save
    save(ep, tmp_path/'golden', env.sim.model)
    loaded = load_episode(tmp_path/'golden')
    assert loaded.channel_names == ep.channel_names
    assert loaded.rows == [flatten_row(r) for r in ep.rows]
    assert loaded.first_failure == ep.first_failure
    assert (tmp_path/'golden'/'model.mjb').exists()
