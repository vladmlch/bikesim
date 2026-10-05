"""Capture v2 stores per-step states and the full force-component matrix."""
import numpy as np
import pytest

def _ep(tmp_path, steps=40):
    from bike_sim.cli import research as research_cli
    from test_pinned_topology import _pinned_config
    from tools.golden_episode import capture_episode
    from bike_sim.sim.ride.control import RideControl
    args = research_cli.parser().parse_args([
        '--physics-config', str(_pinned_config(tmp_path)),
        '--track-file', 'examples/research/rough_uphill_savage.toml',
        '--duration', '1', '--dt', '.00125', '--diagnostic-model-limits',
        '--out', str(tmp_path/'out')])
    env = research_cli.make_environment(args)
    return env, capture_episode(env, steps, RideControl(human_torque_nm=35.))

@pytest.mark.slow
def test_v2_state_and_forces_recorded(tmp_path):
    env, ep = _ep(tmp_path)
    nv = env.sim.model.nv
    assert ep.state_qpos.shape == (40, env.sim.model.nq)
    assert ep.state_qvel.shape == (40, nv)
    assert ep.state_warmstart.shape == (40, nv)
    assert ep.forces.shape == (40, len(ep.force_names), nv)
    # suspension components must be present under their acc names
    assert {'fork_spring', 'shock_coil', 'shock_damper'} <= set(ep.force_names)
    # insertion-order sum of components is finite (ordering is the contract)
    total = np.zeros(nv)
    for i in range(len(ep.force_names)):
        total += ep.forces[10][i]
    assert np.isfinite(total).all()

@pytest.mark.slow
def test_v2_roundtrip_and_determinism(tmp_path):
    env, ep = _ep(tmp_path)
    _, other = _ep(tmp_path)
    for f in ('state_qpos', 'state_qvel', 'forces', 'ctrl_written'):
        assert np.array_equal(getattr(ep, f), getattr(other, f))
    from tools.golden_episode import save, load_episode
    save(ep, tmp_path/'g', env.sim.model)
    loaded = load_episode(tmp_path/'g')
    for f in ('state_qpos', 'state_qvel', 'forces', 'ctrl_written'):
        assert np.array_equal(getattr(loaded, f), getattr(ep, f))
    assert loaded.force_names == ep.force_names
