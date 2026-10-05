"""Native suspension components are bitwise-equal to recorded acc components."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'native' / 'build'))

from _bits import assert_bitwise_equal

MJB = 'tools/proto_native_bench/artifacts/model.mjb'


def _golden(tmp_path, steps=40):
    from bike_sim.cli import research as research_cli
    from test_pinned_topology import _pinned_config
    from tools.golden_episode import capture_episode, save
    from bike_sim.sim.ride.control import RideControl
    args = research_cli.parser().parse_args([
        '--physics-config', str(_pinned_config(tmp_path)),
        '--track-file', 'examples/research/rough_uphill_savage.toml',
        '--duration', '1', '--dt', '.00125', '--diagnostic-model-limits',
        '--out', str(tmp_path/'out')])
    env = research_cli.make_environment(args)
    ep = capture_episode(env, steps, RideControl(human_torque_nm=35.))
    save(ep, tmp_path/'g', env.sim.model)
    return env, tmp_path/'g'


@pytest.mark.slow
def test_suspension_components_bitwise(tmp_path):
    # Bitwise contract: every `**` in the Python source reaches libm pow()
    # via pyfloat::pow, and -ffp-contract=off keeps clang from fusing
    # mul+add into FMA — verified: all 8 components × 40 states compare
    # byte-identical, zero ulp budget consumed.
    bike_native = pytest.importorskip('bike_native')
    from tools.golden_episode import load_episode
    from tools.native_config import project
    env, g = _golden(tmp_path)
    ep = load_episode(g)
    st = bike_native.Stepper(str(g/'model.mjb'), project(env))
    sus = {n for n in ep.force_names if n.startswith(('fork_', 'shock_'))}
    for k in range(len(ep.state_qpos)):
        st.set_state(ep.state_qpos[k], ep.state_qvel[k], ep.state_act[k],
                     ep.state_warmstart[k], float(ep.state_time[k]))
        st.forward()
        comp = st.suspension_components()
        # The native dict must carry exactly the suspension names the
        # accumulator recorded, in the golden force-matrix's insertion
        # order (shock_hbo exists only in physical mode).
        assert list(comp) == [n for n in ep.force_names if n in sus]
        for i, name in enumerate(ep.force_names):
            if name in sus:
                assert_bitwise_equal(comp[name], ep.forces[k][i],
                                     f'{name} step {k}')


@pytest.mark.slow
def test_suspension_components_bitwise_legacy(tmp_path):
    """The legacy physics mode (default SimulationPhysicsConfig) emits the
    7-component dict — no shock_hbo, legacy coil/damper branches — and the
    native writer must match it bitwise as well."""
    bike_native = pytest.importorskip('bike_native')
    import mujoco
    from bike_sim.sim.ride_sim import RideSimulation
    from tools.native_config import project
    sim = RideSimulation()          # legacy mode, shipped defaults
    assert sim.applier.physics_config.physics_mode == 'legacy'
    mjb = tmp_path / 'model.mjb'
    mujoco.mj_saveModel(sim.model, str(mjb))
    st = bike_native.Stepper(str(mjb), project(sim))
    # Probe a spread of compression/velocity states, including top-out
    # (negative) and bottom-out-adjacent strokes — the legacy paths keep
    # the bumper and both damper branches reachable.
    ap = sim.applier
    act = np.zeros(sim.model.na)
    for travel_mm, stroke_mm, fork_v, shock_v in [
            (0.0, 0.0, 0.0, 0.0), (61.3, 12.5, 0.0087, -0.31),
            (179.5, 64.9, 2.4, 1.9), (0.5, 1.0, -1.2, -0.9)]:
        qpos = sim.data.qpos.copy()
        qvel = np.zeros(sim.model.nv)
        qpos[ap.fork_qposadr] = travel_mm / 1000.0
        qpos[ap.shock_qposadr] = stroke_mm / 1000.0
        qvel[ap.fork_dofadr] = fork_v
        qvel[ap.shock_dofadr] = shock_v
        sim.data.qpos[:] = qpos
        sim.data.qvel[:] = qvel
        ref = ap.compute_qfrc_components(sim.model, sim.data)
        st.set_state(qpos, qvel, act, np.zeros(sim.model.nv), 0.0)
        st.forward()
        comp = st.suspension_components()
        assert set(ref) == {'fork_spring', 'fork_damper', 'shock_coil',
                            'shock_bumper', 'shock_damper', 'shock_top_out',
                            'shock_upper_stop'}
        assert list(comp) == list(ref)
        for name, expected in ref.items():
            assert_bitwise_equal(comp[name], expected,
                                 f'{name} @ travel={travel_mm} '
                                 f'stroke={stroke_mm}')


def test_suspension_components_require_config():
    bike_native = pytest.importorskip('bike_native')
    # Both the one-argument form and an explicitly empty config leave the
    # suspension writer disabled: the method raises std::logic_error,
    # which nanobind surfaces as RuntimeError.
    for st in (bike_native.Stepper(MJB), bike_native.Stepper(MJB, {})):
        with pytest.raises(RuntimeError):
            st.suspension_components()


def test_config_missing_key_names_it():
    bike_native = pytest.importorskip('bike_native')
    with pytest.raises(ValueError, match='schema'):
        bike_native.Stepper(MJB, {'suspension': {}})
    with pytest.raises(ValueError, match='physics_mode'):
        bike_native.Stepper(MJB, {'schema': 1, 'suspension': {}})
