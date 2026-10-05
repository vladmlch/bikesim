"""Native brake + resistance writers are bitwise-equal to the Python writers.

Dual-run oracle per stored golden state (mirrors the T2 oracle): a fresh
``MjData`` on the saved ``model.mjb`` is reset to ``state_*[k]`` and forwarded,
then the PYTHON writer objects from the capturing env
(``env.sim.brakes`` / ``env.sim.physical.resistance``) run on it — this
exercises input-derivation too, not just the artifact. The native writers run
through ``Stepper(path, project(env))``.
"""
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

BUILD = Path(__file__).resolve().parents[2] / 'native' / 'build'
selected = os.environ.get('NATIVE_TEST_BUILD_DIR', '')
if selected not in ('', 'asan', 'coverage', 'rtsan'):
    raise ValueError('NATIVE_TEST_BUILD_DIR must be empty or a known build dir')
if selected:
    BUILD /= selected
sys.path.insert(0, str(BUILD))

from _bits import assert_bitwise_equal

MJB = 'tools/proto_native_bench/artifacts/model.mjb'

# Demand sweep: interior points, both ends, and out-of-range values for the
# clamp path (braking.py:102 clamps into [0, 1] rather than rejecting).
DEMANDS = (0.0, 0.3, 0.7, 1.0, -0.2, 1.5)


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


def _restore(model, data, ep, k):
    mujoco.mj_resetData(model, data)
    data.qpos[:] = ep.state_qpos[k]
    data.qvel[:] = ep.state_qvel[k]
    data.act[:] = ep.state_act[k]
    data.qacc_warmstart[:] = ep.state_warmstart[k]
    data.time = float(ep.state_time[k])
    mujoco.mj_forward(model, data)


def _py_snapshots(row):
    """Manifest tire-snapshot row -> duck-typed objects for
    ``compute_components``: SimpleNamespace with ``patches`` /
    ``effective_radius_m``; patches carry ``normal_load_n`` /
    ``working_surface``."""
    out = {}
    for side, snap in row.items():
        out[side] = SimpleNamespace(
            effective_radius_m=float(snap['effective_radius_m']),
            patches=[SimpleNamespace(
                normal_load_n=float(p['normal_load_n']),
                working_surface=bool(p['working_surface']))
                for p in snap['patches']])
    return out


def _native_snapshots(row):
    """The same row in the flat-array schema the binding consumes:
    ``{side: {'patch_loads': f64[n], 'patch_working': bool[n],
    'eff_radius': float}}``."""
    out = {}
    for side, snap in row.items():
        out[side] = {
            'patch_loads': np.array([float(p['normal_load_n'])
                                     for p in snap['patches']]),
            'patch_working': np.array([bool(p['working_surface'])
                                       for p in snap['patches']]),
            'eff_radius': float(snap['effective_radius_m']),
        }
    return out


@pytest.mark.slow
def test_brake_torques_and_apply_bitwise(tmp_path):
    bike_native = pytest.importorskip('bike_native')
    from tools.golden_episode import load_episode
    from tools.native_config import project
    env, g = _golden(tmp_path)
    ep = load_episode(g)
    ref = mujoco.MjModel.from_binary_path(str(g/'model.mjb'))
    dref = mujoco.MjData(ref)
    st = bike_native.Stepper(str(g/'model.mjb'), project(env))
    brake = env.sim.brakes     # BrakeController — the braking.py writer
    front_adr = mujoco.mj_name2id(ref, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                  'front_brake')
    rear_adr = mujoco.mj_name2id(ref, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                 'rear_brake')
    for k in range(len(ep.state_qpos)):
        _restore(ref, dref, ep, k)
        st.set_state(ep.state_qpos[k], ep.state_qvel[k], ep.state_act[k],
                     ep.state_warmstart[k], float(ep.state_time[k]))
        st.forward()
        # The episode was captured with front=rear=0: the golden ctrl rows
        # verify the write path only trivially — every brake adr is 0.
        assert ep.ctrl_written[k][front_adr] == 0.0
        assert ep.ctrl_written[k][rear_adr] == 0.0
        for f in DEMANDS:
            for r in DEMANDS:
                ef, er = brake.compute(dref, f, r)
                nf, nr = st.brake_torques(f, r)
                assert_bitwise_equal(np.asarray([nf, nr]),
                                     np.asarray([ef, er]),
                                     f'brake_torques({f},{r}) step {k}')
                # apply_brake must write the SAME torques into d.ctrl at the
                # two brake actuator addresses — and nothing else. Both sides
                # sit on freshly reset data (ctrl all-zero apart from these
                # slots: mj_resetData clears it, and only the two brake
                # addresses are ever written between restores).
                dref.ctrl.fill(0.)
                dref.ctrl[front_adr] = ef
                dref.ctrl[rear_adr] = er
                st.apply_brake(f, r)
                assert_bitwise_equal(np.asarray(st.ctrl),
                                     np.asarray(dref.ctrl),
                                     f'apply_brake({f},{r}) step {k}')


@pytest.mark.slow
def test_resistance_components_bitwise(tmp_path):
    bike_native = pytest.importorskip('bike_native')
    from tools.golden_episode import load_episode
    from tools.native_config import project
    env, g = _golden(tmp_path)
    ep = load_episode(g)
    ref = mujoco.MjModel.from_binary_path(str(g/'model.mjb'))
    dref = mujoco.MjData(ref)
    st = bike_native.Stepper(str(g/'model.mjb'), project(env))
    resistance = env.sim.physical.resistance   # ExternalResistanceApplier
    for k in range(len(ep.state_qpos)):
        _restore(ref, dref, ep, k)
        st.set_state(ep.state_qpos[k], ep.state_qvel[k], ep.state_act[k],
                     ep.state_warmstart[k], float(ep.state_time[k]))
        st.forward()
        row = ep.manifest['tire_snapshots'][k]
        pyc = resistance.compute_components(ref, dref, _py_snapshots(row))
        nat = st.resistance_components(_native_snapshots(row))
        assert list(nat) == list(pyc) == ['road_rolling', 'aerodynamic']
        for name, expected in pyc.items():
            assert_bitwise_equal(nat[name], expected,
                                 f'{name} oracle step {k}')
            # Same vectors must also equal the artifact's recorded
            # acc.add rows for this step.
            i = ep.force_names.index(name)
            assert_bitwise_equal(nat[name], ep.forces[k][i],
                                 f'{name} golden step {k}')


def test_brake_resistance_require_config():
    bike_native = pytest.importorskip('bike_native')
    # Both writers stay disabled without their config sections; each call
    # raises std::logic_error, which nanobind surfaces as RuntimeError —
    # reached once the input conversion itself succeeds.
    empty_side = {'patch_loads': np.empty(0),
                  'patch_working': np.empty(0, dtype=bool),
                  'eff_radius': 0.3}
    for st in (bike_native.Stepper(MJB), bike_native.Stepper(MJB, {})):
        with pytest.raises(RuntimeError):
            st.brake_torques(0.0, 0.0)
        with pytest.raises(RuntimeError):
            st.apply_brake(0.0, 0.0)
        with pytest.raises(RuntimeError):
            st.resistance_components({'front': empty_side,
                                      'rear': empty_side})
        # Input validation precedes the core call: a malformed snapshot
        # dict fails as ValueError even on a disabled writer.
        with pytest.raises(ValueError, match='front'):
            st.resistance_components({})


def test_brake_resistance_missing_key_names_it():
    bike_native = pytest.importorskip('bike_native')
    with pytest.raises(ValueError, match='torque_ceiling_nm'):
        bike_native.Stepper(MJB, {'schema': 1, 'brake': {}})
    with pytest.raises(ValueError, match='crr'):
        bike_native.Stepper(MJB, {'schema': 1, 'resistance': {}})


@pytest.mark.parametrize('layout', ('contiguous', 'strided', 'reversed'))
@pytest.mark.parametrize('converted_dtype', (False, True))
def test_resistance_snapshot_array_conversion_lifetime(layout, converted_dtype):
    """Converted front AND rear buffers must stay alive until the writer reads
    them. Exercise their reads under ASan and compare converted values in
    ordinary builds."""
    bike_native = pytest.importorskip('bike_native')
    cfg = {'schema': 1, 'resistance': {
        'crr': 0.01, 'rolling_taper_rad_s': 0.5, 'rho_kg_m3': 1.2,
        'cda_m2': 0.4, 'wind_world_mps': [0., 0., 0.],
        'point_body_m': [0., 0., 0.],
        'bodies': {'frame': 'frame', 'front_wheel': 'front_wheel',
                   'rear_wheel': 'rear_wheel'}}}
    model = mujoco.MjModel.from_binary_path(MJB)
    data = mujoco.MjData(model)
    data.qvel[:] = np.linspace(0.2, 1.2, model.nv)
    st = bike_native.Stepper(MJB, cfg)
    st.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart, 0.)
    st.forward()

    def array(values, dtype):
        a = np.array(values, dtype=dtype)
        if layout == 'strided':
            storage = np.repeat(a, 2)
            return storage[::2]
        if layout == 'reversed':
            return a[::-1].copy()[::-1]
        return a

    loads_dtype, working_dtype = ((np.float32, np.uint8) if converted_dtype
                                  else (np.float64, np.bool_))
    snaps = {side: {'patch_loads': array([100., 900., 300.], loads_dtype),
                    'patch_working': array([1, 0, 1], working_dtype),
                    'eff_radius': radius}
             for side, radius in (('front', 0.3), ('rear', 0.35))}
    canonical = {side: {'patch_loads': np.array([100., 900., 300.]),
                        'patch_working': np.array([True, False, True]),
                        'eff_radius': snap['eff_radius']}
                 for side, snap in snaps.items()}
    want = st.resistance_components(canonical)
    assert np.any(want['road_rolling'] != 0.0)
    for _ in range(8):
        got = st.resistance_components(snaps)
        for name in want:
            assert_bitwise_equal(got[name], want[name], f'{name} {layout}')
