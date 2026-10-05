"""Capture v2 stores per-step states and the full force-component matrix."""
import numpy as np
import pytest

from _bits import assert_bitwise_equal

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
    assert ep.qfrc_applied.shape == (40, nv)
    assert len(ep.manifest['tire_snapshots']) == 40
    # suspension components must be present under their acc names
    assert {'fork_spring', 'shock_coil', 'shock_damper'} <= set(ep.force_names)
    # The component fold must reproduce the independently captured live total.
    for k in range(len(ep.forces)):
        total = np.zeros(nv)
        for component in ep.forces[k]:
            total += component
        assert_bitwise_equal(total, ep.qfrc_applied[k], f'exit total step {k}')

@pytest.mark.slow
def test_v2_roundtrip_and_determinism(tmp_path):
    env, ep = _ep(tmp_path)
    _, other = _ep(tmp_path)
    fields = ('state_qpos', 'state_qvel', 'state_act', 'state_warmstart',
              'state_time', 'forces', 'ctrl_written', 'tire_state',
              'qfrc_applied')
    for f in fields:
        assert_bitwise_equal(getattr(ep, f), getattr(other, f), f)
    from tools.golden_episode import save, load_episode
    save(ep, tmp_path/'g', env.sim.model)
    loaded = load_episode(tmp_path/'g')
    for f in fields:
        assert_bitwise_equal(getattr(loaded, f), getattr(ep, f), f)
    assert loaded.force_names == ep.force_names


@pytest.mark.slow
def test_exit_total_is_captured_independently_of_components(tmp_path, monkeypatch):
    from tools.golden_episode import capture_episode, save, load_episode
    from bike_sim.sim.ride.control import RideControl
    env, empty = _ep(tmp_path, steps=0)
    assert empty.qfrc_applied.shape == (0, env.sim.model.nv)
    runtime = env.sim.physical
    orig = type(runtime).apply_forces
    observed = []

    def changed_exit(self, **kw):
        out = orig(self, **kw)
        # A write after accumulator assembly proves capture reads mjData's
        # exit buffer rather than reconstructing the same component mapping.
        self.sim.data.qfrc_applied[0] += 0.125
        if kw.get('advance', True):
            observed.append(self.sim.data.qfrc_applied.copy())
        return out

    monkeypatch.setattr(type(runtime), 'apply_forces', changed_exit)
    ep = capture_episode(env, 3, RideControl(human_torque_nm=35.))
    assert_bitwise_equal(ep.qfrc_applied, np.array(observed[-3:]))
    reconstructed = np.zeros(env.sim.model.nv)
    for component in ep.forces[-1]:
        reconstructed += component
    assert reconstructed.tobytes() != ep.qfrc_applied[-1].tobytes()
    save(ep, tmp_path/'changed', env.sim.model)
    loaded = load_episode(tmp_path/'changed')
    assert_bitwise_equal(loaded.qfrc_applied, ep.qfrc_applied)


@pytest.mark.parametrize('version', ('v1', 'v2'))
def test_load_old_artifact_without_exit_total(tmp_path, version):
    import mujoco
    from tools.golden_episode import EpisodeArtifact, save, load_episode
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body><joint type="slide"/>'
        '<geom size=".1"/></body></worldbody></mujoco>')
    data = mujoco.MjData(model)
    initial = {k: getattr(data, k).copy() for k in
               ('qpos', 'qvel', 'act', 'ctrl', 'qacc_warmstart')}
    initial['time'] = np.array([0.])
    ep = EpisodeArtifact([], initial, [], [], None, {},
                         force_names=['old'],
                         forces=np.ones((2, 1, model.nv)))
    save(ep, tmp_path, model)
    with np.load(tmp_path/'episode.npz', allow_pickle=False) as z:
        old = {key: z[key] for key in z.files if key != 'qfrc_applied' and
               (version == 'v2' or not key.startswith(('state_', 'tire_')) and
                key not in ('ctrl_written', 'forces', 'force_names'))}
    np.savez(tmp_path/'episode.npz', **old)
    loaded = load_episode(tmp_path)
    assert loaded.qfrc_applied.shape == (0, model.nv)
    assert loaded.forces.shape == ((2, 1, model.nv) if version == 'v2'
                                   else (0, 0, model.nv))
    # Missing independent observations stay empty; they must not be invented
    # from the old matrix and silently weaken the final accumulator gate.
