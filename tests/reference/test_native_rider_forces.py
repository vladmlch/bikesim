"""Native rider_forces writer is bitwise-equal to RiderForceApplier —
plus the P2 accumulator-order gate.

FINDING this file's shape encodes: the canonical golden env
(--rider articulated_planar) leaves ``env.sim.rider_forces`` inactive
(ride_sim.py:246-247 gives it ``pose=None``), so ``'seated_interfaces'``
never appears in ``ep.force_names`` — the canonical episode does not
exercise this writer. The port is verified instead on a seated-variant
plant (``generate_mujoco_xml`` + ``seated_pose`` + ``RiderForceApplier``,
the same construction ride_sim.py:296 runs) via the dual-run oracle:
the live applier's ``apply`` on a restored ``MjData`` vs
``Stepper.rider_forces_qfrc()`` on the same (qpos, qvel), sweeping each
path across the spring's active range — loaded, lifted off, the exact
deflection==0 boundary, and the damper-dominated clamp — with the
projection-time pedal offsets (``set_pedal_offsets``,
ride_sim.py:587) carried through the config.
"""
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

from native_loader import load_native
bike_native = load_native()

from _bits import assert_bitwise_equal

MJB = 'tools/proto_native_bench/artifacts/model.mjb'


def _plant(tmp_path):
    """Model + live RiderForceApplier for the seated variant — the writer's
    real oracle without a full sim build (same construction ride_sim.py:296
    runs: pose resolved from specs, applier over the compiled model)."""
    from bike_sim.geometry.specs import BikeSpecs
    from bike_sim.mujoco.builder import generate_mujoco_xml
    from bike_sim.physics.rider import RiderSpecs
    from bike_sim.sim.ride.rider_forces import RiderForceApplier
    specs = BikeSpecs()
    rider = RiderSpecs(variant='seated')
    pose = rider.seated_pose(specs)
    model = mujoco.MjModel.from_xml_string(
        generate_mujoco_xml(specs, mode='ride', rider=rider))
    applier = RiderForceApplier(model, pose)
    assert applier.active
    mjb = tmp_path / 'model.mjb'
    mujoco.mj_saveModel(model, str(mjb))
    return model, applier, mjb


def _rf_section(model, rf):
    """What tools.native_config.project emits for 'rider_forces' — the same
    resolved fields, read straight off the live applier."""
    from tools.native_config import _joint_name
    return {'paths': [
        {'name': p.body.name,
         'joint': _joint_name(model, p.qposadr),
         'stiffness_n_m': float(p.body.stiffness_n_m),
         'damping_ns_m': float(p.body.damping_ns_m),
         'preload_deflection_m': float(p.body.preload_deflection_m),
         'unilateral': bool(p.body.unilateral),
         'offset_m': float(p.offset_m)}
        for p in rf._paths]}


def _py_qfrc(model, rf, qpos, qvel, d):
    """Python oracle: apply() on a zeroed qfrc_applied, then the whole
    buffer — the same surface acc.add('seated_interfaces', ...) records
    (physical_runtime.py:256-259, qfrc_applied is filled with zeros first)."""
    mujoco.mj_resetData(model, d)
    d.qpos[:] = qpos
    d.qvel[:] = qvel
    d.qfrc_applied.fill(0.0)
    rf.apply(model, d)
    return d.qfrc_applied.copy()


def _sweep_states(model, rf, qpos0):
    """(label, qpos, qvel) probes covering every path branch."""
    states = []
    for p in rf._paths:
        b = p.body
        # compute()'s boundary in its own expression order: the deflection
        # is (preload_deflection_m + offset_m) - q, so q == bound gives
        # deflection exactly +0.0 — inside the gap branch.
        bound = b.preload_deflection_m + p.offset_m
        qs = (0.0, 0.5 * bound, np.nextafter(bound, -np.inf), bound,
              np.nextafter(bound, np.inf), bound + 0.02, -0.03, 0.05)
        # The unilateral damper clamp fires when k*d - c*qd turns negative;
        # straddle the ratio evaluated at q=0 (deflection = bound).
        thr = b.stiffness_n_m * bound / b.damping_ns_m
        qds = (0.0, -0.5 * thr, 0.5 * thr, 0.9 * thr, thr, 1.5 * thr)
        for q in qs:
            for qd in qds:
                qpos = qpos0.copy()
                qvel = np.zeros(model.nv)
                qpos[p.qposadr] = q
                qvel[p.dofadr] = qd
                states.append((f'{b.name} q={q!r} qd={qd!r}', qpos, qvel))
    # Combined probes: every path at the same regime simultaneously, plus
    # seeded-random rider states.
    bounds = {p.qposadr: p.body.preload_deflection_m + p.offset_m
              for p in rf._paths}
    for tag, shift in (('boundary', 0.0), ('gap', 0.03),
                       ('compressed', -0.02)):
        qpos = qpos0.copy()
        qvel = np.zeros(model.nv)
        for adr, bound in bounds.items():
            qpos[adr] = bound + shift
        for p in rf._paths:
            qvel[p.dofadr] = 0.37
        states.append((f'all-{tag}', qpos, qvel))
    rng = np.random.default_rng(0xB1CE)
    for j in range(40):
        qpos = qpos0.copy()
        qvel = np.zeros(model.nv)
        for p in rf._paths:
            bound = p.body.preload_deflection_m + p.offset_m
            qpos[p.qposadr] = bound + rng.uniform(-0.05, 0.05)
            qvel[p.dofadr] = rng.uniform(-3.0, 3.0)
        states.append((f'rng{j}', qpos, qvel))
    return states


def test_rider_forces_qfrc_bitwise(tmp_path):
    model, rf, mjb = _plant(tmp_path)
    # Projection-time pedal offsets (ride_sim.py:573-587 sets them once
    # per _follow_cranks): applied before projection so the config carries
    # the resolved offset_m like the ruling specifies.
    rf.set_pedal_offsets(0.023, -0.031)
    st = bike_native.Stepper(str(mjb), {'schema': 1, 'rider_forces':
                                        _rf_section(model, rf)})
    d = mujoco.MjData(model)
    act = np.zeros(model.na)
    warm = np.zeros(model.nv)
    mujoco.mj_resetData(model, d)
    qpos0 = d.qpos.copy()
    for label, qpos, qvel in _sweep_states(model, rf, qpos0):
        want = _py_qfrc(model, rf, qpos, qvel, d)
        st.set_state(qpos, qvel, act, warm, 0.0)
        # No forward() on either side — compute() reads qpos/qvel only
        # (rider_forces.py:152-154).
        got = st.rider_forces_qfrc()
        assert_bitwise_equal(got, want, f'rider_forces {label}')


@pytest.mark.slow
def test_project_emits_resolved_paths(tmp_path):
    """The config bridge emits per-path resolved params + live offsets, in
    _paths order, only while the applier is active. RideSimulation()'s
    default variant is 'seated' (DEFAULT_RIDER_VARIANT), so its
    rider_forces is active — unlike the canonical articulated env."""
    from bike_sim.sim.ride_sim import RideSimulation
    from tools.native_config import project
    sim = RideSimulation()
    rf = sim.rider_forces
    assert rf.active
    mjb = tmp_path / 'model.mjb'
    mujoco.mj_saveModel(sim.model, str(mjb))
    rf.set_pedal_offsets(0.023, -0.031)
    sec = project(sim)['rider_forces']['paths']
    assert len(sec) == len(rf._paths)
    for want, got in zip(rf._paths, sec):
        b = want.body
        assert got['stiffness_n_m'] == b.stiffness_n_m
        assert got['damping_ns_m'] == b.damping_ns_m
        assert got['preload_deflection_m'] == b.preload_deflection_m
        assert got['unilateral'] == b.unilateral
        assert got['offset_m'] == want.offset_m
        jid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT,
                                got['joint'])
        assert int(sim.model.jnt_qposadr[jid]) == want.qposadr
    # The projected section must drive the native writer: feed it.
    assert bike_native.Stepper(str(mjb), {'schema': 1, 'rider_forces': {
        'paths': sec}}).rider_forces_qfrc().shape == (sim.model.nv,)


def _rf_cfg(**over):
    """Minimal hand-built rider_forces section on the pinned model —
    'root_x' is an unlimited slide there."""
    path = {'name': 'probe_path', 'joint': 'root_x', 'stiffness_n_m': 1e5,
            'damping_ns_m': 1e3,
            'preload_deflection_m': 0.01, 'unilateral': True,
            'offset_m': 0.0}
    path.update(over)
    return {'schema': 1, 'rider_forces': {'paths': [path]}}


def test_rider_forces_hand_built_smoke():
    st = bike_native.Stepper(MJB, _rf_cfg())
    got = st.rider_forces_qfrc()
    nv = mujoco.MjModel.from_binary_path(MJB).nv
    assert got.shape == (nv,)
    # q=qd=0: force = max(0, 1e5*(0.01+0-0) - 1e3*0) — same expression order.
    assert_bitwise_equal(got[0],
                         max(0.0, 1e5 * (0.01 + 0.0 - 0.0) - 1e3 * 0.0),
                         'hand-built path force')


def test_rider_forces_requires_config():
    for st in (bike_native.Stepper(MJB), bike_native.Stepper(MJB, {})):
        with pytest.raises(RuntimeError):
            st.rider_forces_qfrc()


def test_rider_forces_missing_key_names_it():
    with pytest.raises(ValueError, match='paths'):
        bike_native.Stepper(MJB, {'schema': 1, 'rider_forces': {}})
    for key in ('joint', 'stiffness_n_m', 'damping_ns_m',
                'preload_deflection_m', 'unilateral', 'offset_m'):
        cfg = _rf_cfg()
        del cfg['rider_forces']['paths'][0][key]
        with pytest.raises(ValueError, match=key):
            bike_native.Stepper(MJB, cfg)
    with pytest.raises(ValueError, match='paths'):
        bike_native.Stepper(
            MJB, {'schema': 1,
                  'rider_forces': {'paths': {'joint': 'root_x'}}})


def test_rider_forces_joint_validation():
    # Missing joint, non-slide joint (a hinge), and a LIMITED slide — the
    # three _resolve_slide rejections (rider_forces.py:109-115).
    for joint, match in [('no_such_joint', 'no joint'),
                         ('front_wheel_spin', 'unlimited slide'),
                         ('shock_stroke', 'unlimited slide')]:
        with pytest.raises(ValueError, match=match):
            bike_native.Stepper(MJB, _rf_cfg(joint=joint))


def _acc_total(components, nv):
    """ForceAccumulator.add + total() (force_accumulator.py:19-25,47-51) —
    the Python-side ordering oracle."""
    from bike_sim.sim.ride.force_accumulator import ForceAccumulator
    acc = ForceAccumulator(nv)
    for name, qfrc in components.items():
        acc.add(name, np.asarray(qfrc, float))
    return acc.total()


def test_accumulator_order_bitwise():
    """The P2 ordering gate: Stepper.total(dict) folds in dict insertion
    order, bitwise-equal to ForceAccumulator.total() — the contract P4's
    step loop replicates."""
    nv = int(mujoco.MjModel.from_binary_path(MJB).nv)
    st = bike_native.Stepper(MJB)
    # Order-sensitive triple: 1e20 absorbs 1.0 mid-fold, so (a,b,c) sums to
    # 0 while (a,c,b) sums to 1 — both folds must differ, and each must
    # match the Python accumulator in the same order.
    a = np.full(nv, 1e20)
    b = np.ones(nv)
    c = np.full(nv, -1e20)
    first = {'a': a, 'b': b, 'c': c}
    reordered = {'a': a, 'c': c, 'b': b}
    got1 = st.total(first)
    got2 = st.total(reordered)
    assert_bitwise_equal(got1, _acc_total(first, nv), 'order a,b,c')
    assert_bitwise_equal(got2, _acc_total(reordered, nv), 'order a,c,b')
    assert not np.array_equal(got1, got2)   # the order actually mattered
    assert_bitwise_equal(got1, np.zeros(nv), 'absorbed fold')
    # Seeded fuzz across sizes/magnitudes, forward and reversed orders.
    rng = np.random.default_rng(0xACC0)
    for trial in range(6):
        comps = {}
        for i in range(int(rng.integers(1, 8))):
            scale = 10.0 ** int(rng.integers(-6, 7))
            comps[f'c{i}'] = rng.normal(0.0, scale, nv)
        for cand in (comps, dict(reversed(list(comps.items())))):
            assert_bitwise_equal(st.total(cand), _acc_total(cand, nv),
                                 f'fuzz trial {trial}')
    # Empty fold -> zeros, like acc.total() with no components.
    assert_bitwise_equal(st.total({}), np.zeros(nv), 'empty total')


def test_accumulator_total_validates():
    nv = int(mujoco.MjModel.from_binary_path(MJB).nv)
    st = bike_native.Stepper(MJB)
    # acc.add's contract: shape != (nv,) or non-finite -> ValueError
    # Model-width errors retain 'invalid generalized force'; the wire reader
    # names non-finite components by their full public field path.
    with pytest.raises(ValueError, match='invalid generalized force'):
        st.total({'bad': np.zeros(nv + 1)})
    with pytest.raises(ValueError, match='invalid generalized force'):
        st.total({'ok': np.zeros(nv), 'bad': np.zeros(nv + 1)})
    with pytest.raises(ValueError, match='total.bad'):
        st.total({'bad': np.full(nv, np.inf)})
    with pytest.raises(ValueError, match='total.bad'):
        st.total({'bad': np.full(nv, np.nan)})
    # Wrong rank or a non-convertible value — rejected at the edge.
    with pytest.raises(ValueError):
        st.total({'bad': np.zeros((nv, 1))})
    with pytest.raises(ValueError):
        st.total({'bad': 'not an array'})


@pytest.mark.parametrize('layout', ('contiguous', 'strided', 'reversed'))
@pytest.mark.parametrize('dtype', (np.float64, np.float32, np.int32))
def test_accumulator_total_accepts_array_conversion(layout, dtype):
    nv = int(mujoco.MjModel.from_binary_path(MJB).nv)
    values = np.arange(nv, dtype=dtype)
    if layout == 'strided':
        values = np.repeat(values, 2)[::2]
    elif layout == 'reversed':
        values = values[::-1]
    components = {'converted': values, 'second': np.ones(nv)}
    st = bike_native.Stepper(MJB)
    assert_bitwise_equal(st.total(components), _acc_total(components, nv),
                         f'total converts {dtype} {layout}')


@pytest.mark.slow
def test_accumulator_order_golden(tmp_path):
    """Ordering gate on the recorded acc matrix: for every stored step the
    native total() folds force_names-ordered components — ported ones
    computed by the native writers, the rest recorded — bitwise-equal to
    the independent apply_forces-exit qfrc_applied captured from mjData."""
    from test_native_brake_resistance import _golden
    from tools.golden_episode import load_episode
    from tools.native_config import project
    env, g = _golden(tmp_path)
    ep = load_episode(g)
    st = bike_native.Stepper(str(g/'model.mjb'), project(env))
    assert ep.qfrc_applied.shape == (len(ep.forces), ep.forces.shape[2])
    # FINDING: the canonical episode (articulated rider) never records
    # 'seated_interfaces' — the rider writer is inert there, so the golden
    # matrix cannot oracle it; the sweep above is the oracle instead.
    assert 'seated_interfaces' not in ep.force_names
    assert 'rider_forces' not in project(env)
    tire_names = list(ep.tire_state_names)
    dt = float(ep.manifest['dt_s'])
    for k in range(len(ep.forces)):
        st.set_state(ep.state_qpos[k], ep.state_qvel[k], ep.state_act[k],
                     ep.state_warmstart[k], float(ep.state_time[k]))
        st.forward()
        st.set_tire_state(tire_names, ep.tire_state[k])
        ported = dict(st.suspension_components())
        ported['tires'] = st.tire_qfrc(dt)
        ported.update(st.resistance_components(st.tire_snapshots()))
        comp = {}
        for i, name in enumerate(ep.force_names):
            comp[name] = ported.get(name, ep.forces[k][i])
        assert_bitwise_equal(st.total(comp), ep.qfrc_applied[k],
                             f'acc.total step {k}')
