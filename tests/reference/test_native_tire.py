"""Native tire writer is bitwise-equal to the Python TireForceApplier.

Per stored golden state: ``set_state`` + ``set_tire_state(ep.tire_state[k])``
restores the brush states ``apply_forces[k]`` consumed, then
``tire_qfrc(dt)`` must equal ``forces[k]['tires']`` bitwise AND the live
Python ``TireForceApplier.compute_qfrc`` fed the same restored
``_BrushState``s. The evolved brush states and the contact snapshots are
checked bitwise as well, and the native snapshot dicts feed
``resistance_components`` end-to-end (the same dict shape the binding test
uses).
"""
import os
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

from native_loader import load_native
bike_native = load_native()

from _bits import assert_bitwise_equal
from test_native_brake_resistance import _golden, _restore

MJB = 'tools/proto_native_bench/artifacts/model.mjb'

# The flat names the writer emits/consumes — `flatten_row(asdict(_BrushState))`
# per side, `_BrushState` field order: xi, tangent, point, segment, center.
CANONICAL_NAMES = tuple(
    f'{side}.{field}' for side in ('front', 'rear') for field in (
        'xi', 'tangent.0', 'tangent.1', 'tangent.2',
        'point.0', 'point.1', 'point.2', 'segment',
        'center.0', 'center.1', 'center.2'))


def _canonical_row(names, row):
    """Project an artifact/flattened tire-state row onto the 22-column schema."""
    index = {n: i for i, n in enumerate(names)}
    return np.array([row[index[n]] if n in index else np.nan
                     for n in CANONICAL_NAMES])


def _flat_to_row(flat):
    """Same projection for a ``flatten_row`` dict (None/'nan' stay NaN)."""
    return np.array([float(v) if isinstance(v, (int, float)) else np.nan
                     for n in CANONICAL_NAMES
                     for v in (flat.get(n, np.nan),)])


def _restored_states(names, row):
    """Decode an artifact tire-state row into {side: _BrushState} — the same
    NaN convention the binding follows (all-NaN vector -> None, NaN segment
    -> None)."""
    from bike_sim.sim.ride.tire_forces import _BrushState
    index = {n: i for i, n in enumerate(names)}

    def val(key):
        return row[index[key]] if key in index else np.nan

    out = {}
    for side in ('front', 'rear'):
        st = _BrushState()
        st.xi = float(val(f'{side}.xi'))
        for field in ('tangent', 'point', 'center'):
            comps = np.array([val(f'{side}.{field}.{i}') for i in range(3)])
            setattr(st, field, None if np.isnan(comps).all() else comps)
        seg = val(f'{side}.segment')
        st.segment = None if np.isnan(seg) else int(seg)
        out[side] = st
    return out


def _flatten_states(states):
    """Python {side: _BrushState} -> canonical 22-row — mirrors the artifact's
    ``flatten_row`` + NaN-for-unset encoding."""
    from dataclasses import asdict
    from tools.golden_episode import flatten_row
    flat = flatten_row({s: asdict(v) for s, v in states.items()})
    return _flat_to_row(flat)


def _assert_snapshots_equal(native, row, k):
    """Native tire_snapshots() dict vs the manifest's per-step JSON row."""
    assert set(native) == set(row) == {'front', 'rear'}
    for side in ('front', 'rear'):
        snap, want = native[side], row[side]
        assert snap['interval_id'] == want['interval_id']
        assert snap['backend'] == want['backend'] == 'compliant_2d'
        assert snap['geometric_contact'] == bool(want['geometric_contact'])
        assert_bitwise_equal(snap['time_s'], float(want['time_s']),
                             f'{side}.time_s step {k}')
        assert_bitwise_equal(snap['effective_radius_m'],
                             float(want['effective_radius_m']),
                             f'{side}.effective_radius_m step {k}')
        assert len(snap['patches']) == len(want['patches'])
        for i, (p, w) in enumerate(zip(snap['patches'], want['patches'])):
            assert bool(p['working_surface']) == bool(w['working_surface'])
            for f in ('normal_load_n', 'tangent_force_n', 'slip_mps'):
                assert_bitwise_equal(p[f], float(w[f]),
                                     f'{side}.patches[{i}].{f} step {k}')


def _live_applier(env, ref):
    from bike_sim.sim.ride.tire_forces import TireForceApplier
    return TireForceApplier(ref, env.sim.physical.vertices,
                            env.sim.physics_config.tires,
                            env.sim.track.surface_map)


@pytest.mark.slow
def test_tire_qfrc_bitwise(tmp_path):
    from tools.golden_episode import load_episode
    from tools.native_config import project
    env, g = _golden(tmp_path)
    ep = load_episode(g)
    ref = mujoco.MjModel.from_binary_path(str(g/'model.mjb'))
    dref = mujoco.MjData(ref)
    st = bike_native.Stepper(str(g/'model.mjb'), project(env))
    applier = _live_applier(env, ref)
    tire_i = ep.force_names.index('tires')
    names = list(ep.tire_state_names)
    dt = float(ref.opt.timestep)
    for k in range(len(ep.state_qpos)):
        _restore(ref, dref, ep, k)
        st.set_state(ep.state_qpos[k], ep.state_qvel[k], ep.state_act[k],
                     ep.state_warmstart[k], float(ep.state_time[k]))
        st.forward()
        st.set_tire_state(names, ep.tire_state[k])
        got = st.tire_qfrc(dt)
        assert_bitwise_equal(got, ep.forces[k][tire_i],
                             f'tires golden step {k}')
        # Live Python oracle on the same restored states.
        applier.states = _restored_states(names, ep.tire_state[k])
        applier.last_time_s = None
        want = applier.compute_qfrc(ref, dref, dt)
        assert_bitwise_equal(got, want, f'tires live-oracle step {k}')
        # The evolved brush state is the step's second output: native state
        # must bitwise-match both the Python states it was checked against
        # and (for k+1 < K) the artifact's next consumed-state row.
        evolved = st.tire_state()
        assert_bitwise_equal(evolved, _flatten_states(applier.states),
                             f'tire_state evolve step {k}')
        if k + 1 < len(ep.tire_state):
            assert_bitwise_equal(evolved,
                                 _canonical_row(names, ep.tire_state[k + 1]),
                                 f'tire_state[k+1] step {k}')
        # Contact snapshots equal the manifest digest bitwise and feed the
        # resistance writer with the same schema the binding test uses.
        _assert_snapshots_equal(st.tire_snapshots(),
                                ep.manifest['tire_snapshots'][k], k)
        res = st.resistance_components(st.tire_snapshots())
        for name, want_r in res.items():
            assert_bitwise_equal(
                want_r, ep.forces[k][ep.force_names.index(name)],
                f'{name} via tire_snapshots step {k}')


@pytest.mark.slow
def test_tire_state_roundtrip_and_clock(tmp_path):
    from tools.golden_episode import load_episode
    from tools.native_config import project
    env, g = _golden(tmp_path)
    ep = load_episode(g)
    ref = mujoco.MjModel.from_binary_path(str(g/'model.mjb'))
    st = bike_native.Stepper(str(g/'model.mjb'), project(env))
    names = list(ep.tire_state_names)
    dt = float(ref.opt.timestep)
    assert tuple(st.tire_state_names) == CANONICAL_NAMES
    for k in range(len(ep.state_qpos)):
        st.set_state(ep.state_qpos[k], ep.state_qvel[k], ep.state_act[k],
                     ep.state_warmstart[k], float(ep.state_time[k]))
        st.forward()
        st.set_tire_state(names, ep.tire_state[k])
        assert_bitwise_equal(st.tire_state(),
                             _canonical_row(names, ep.tire_state[k]),
                             f'state roundtrip step {k}')
    # last_time_s discipline: advancing twice at one timestamp is an error
    # (tire_forces.py:127-128); a fresh set_tire_state restores the clock.
    st.set_state(ep.state_qpos[0], ep.state_qvel[0], ep.state_act[0],
                 ep.state_warmstart[0], float(ep.state_time[0]))
    st.forward()
    st.set_tire_state(names, ep.tire_state[0])
    st.tire_qfrc(dt)
    with pytest.raises(ValueError, match='once per increasing timestamp'):
        st.tire_qfrc(dt)
    st.set_tire_state(names, ep.tire_state[0])
    st.tire_qfrc(dt)          # restore clears the clock — valid again


def _tire_cfg(*missing):
    """Minimal hand-built tire section on the pinned model (compliant_2d,
    constant-mu mode — needs no surface_map)."""
    side = {'material': {'radial_k_n_m': 130000., 'radial_c_ns_m': 800.,
                         'pressure_pa_gauge': 200000.,
                         'provenance': 'synthetic',
                         'valid_load_range_n': [0., 2000.]},
            'tangent_k_n_m': 20000., 'mu': .8, 'relaxation_length_m': .2}
    cfg = {'schema': 1, 'tire': {
        'backend': 'compliant_2d', 'surface_mode': 'configured',
        'significant_delta_m': .0001, 'significance_fraction': .05,
        'distinct_normal_deg': 20., 'front': dict(side), 'rear': dict(side)}}
    for key in missing:
        del cfg['tire'][key]
    return cfg


def test_tire_requires_config():
    row = np.zeros(22)
    for st in (bike_native.Stepper(MJB), bike_native.Stepper(MJB, {})):
        with pytest.raises(RuntimeError):
            st.tire_qfrc(0.00125)
        with pytest.raises(RuntimeError):
            st.set_tire_state(CANONICAL_NAMES, row)
        with pytest.raises(RuntimeError):
            st.tire_state()
        with pytest.raises(RuntimeError):
            st.tire_snapshots()


def test_tire_config_missing_key_names_it():
    with pytest.raises(ValueError, match='backend'):
        bike_native.Stepper(MJB, {'schema': 1, 'tire': {}})
    with pytest.raises(ValueError, match='surface_mode'):
        bike_native.Stepper(MJB, _tire_cfg('surface_mode'))
    bad_mu = _tire_cfg()
    del bad_mu['tire']['front']['mu']
    with pytest.raises(ValueError, match='mu'):
        bike_native.Stepper(MJB, bad_mu)
    # track mode without a serialized surface_map is rejected like the
    # Python applier's 'track material mode requires an explicit SurfaceMap'.
    bad = _tire_cfg()
    bad['tire']['surface_mode'] = 'track'
    with pytest.raises(ValueError):
        bike_native.Stepper(MJB, bad)


def test_tire_set_state_validation():
    st = bike_native.Stepper(MJB, _tire_cfg())
    row = np.zeros(len(CANONICAL_NAMES))
    st.set_tire_state(CANONICAL_NAMES, row)
    with pytest.raises(ValueError):
        st.set_tire_state(CANONICAL_NAMES[:-1], row)   # row/name length mismatch
    with pytest.raises(ValueError):
        st.set_tire_state(CANONICAL_NAMES, row[:-1])
    with pytest.raises(ValueError, match='front.xi'):
        st.set_tire_state(('bogus.xi',) + CANONICAL_NAMES[1:], row)


@pytest.mark.parametrize('side', ('front', 'rear'))
@pytest.mark.parametrize('segment', (np.inf, -np.inf, 0.5, -0.5,
                                     2147483648., -2147483649.,
                                     np.finfo(np.float64).max))
def test_tire_segment_rejects_invalid_without_mutation(side, segment):
    st = bike_native.Stepper(MJB, _tire_cfg())
    model = mujoco.MjModel.from_binary_path(MJB)
    data = mujoco.MjData(model)
    data.qpos[0] += 0.6
    data.qpos[14] += 0.6
    st.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart, 0.)
    st.forward()
    row = np.zeros(len(CANONICAL_NAMES))
    st.set_tire_state(CANONICAL_NAMES, row)
    st.tire_qfrc(0.00125)
    before = st.tire_state().copy()
    bad = before.copy()
    bad[0] += 0.123  # rejected late fields must not commit earlier fields
    bad[CANONICAL_NAMES.index(f'{side}.segment')] = segment
    with pytest.raises(ValueError, match=f'{side}.segment'):
        st.set_tire_state(CANONICAL_NAMES, bad)
    assert_bitwise_equal(st.tire_state(), before, 'rejected segment state')
    # The once-per-timestamp clock is part of the strong guarantee too.
    with pytest.raises(ValueError, match='once per increasing timestamp'):
        st.tire_qfrc(0.00125)


@pytest.mark.parametrize('side', ('front', 'rear'))
@pytest.mark.parametrize('segment', (-2147483648., -1., 0.,
                                     2147483647., np.nan))
def test_tire_segment_int_boundaries_and_unset_roundtrip(side, segment):
    st = bike_native.Stepper(MJB, _tire_cfg())
    row = np.zeros(len(CANONICAL_NAMES))
    idx = CANONICAL_NAMES.index(f'{side}.segment')
    row[idx] = segment
    if np.isnan(segment):
        # An unset segment is only emitted inside a fully-unset contact group —
        # a stored tangent without its segment is now a rejected partial group.
        for i, name in enumerate(CANONICAL_NAMES):
            if name.startswith(f'{side}.') and name != f'{side}.xi':
                row[i] = np.nan
    st.set_tire_state(CANONICAL_NAMES, row)
    assert_bitwise_equal(st.tire_state(), row, 'segment roundtrip')

def test_tire_qfrc_happy_path_and_clock():
    """Non-slow smoke on the pinned model: slide root_x/rider_root_x +0.6 so
    both wheels sit over the road, then tire_qfrc advances once per time."""
    m = mujoco.MjModel.from_binary_path(MJB)
    d = mujoco.MjData(m)
    d.qpos[0] += 0.6
    d.qpos[14] += 0.6
    mujoco.mj_forward(m, d)
    st = bike_native.Stepper(MJB, _tire_cfg())
    st.set_state(d.qpos, d.qvel, d.act, d.qacc_warmstart, float(d.time))
    st.forward()
    st.set_tire_state(CANONICAL_NAMES, np.zeros(len(CANONICAL_NAMES)))
    dt = float(m.opt.timestep)
    q = st.tire_qfrc(dt)
    assert q.shape == (m.nv,)
    with pytest.raises(ValueError):
        st.tire_qfrc(dt)          # same d.time — double-advance is an error
    st.forward()
    with pytest.raises(ValueError, match='tire interval'):
        st.tire_qfrc(0.0)
    with pytest.raises(ValueError, match='tire_qfrc.dt'):
        st.tire_qfrc(float('nan'))
    # Time advanced: the interval check passes and compute runs again —
    # then the same-timestamp call errors once more.
    st.set_state(d.qpos, d.qvel, d.act, d.qacc_warmstart, float(d.time) + dt)
    st.forward()
    st.tire_qfrc(dt)
    with pytest.raises(ValueError, match='once per increasing timestamp'):
        st.tire_qfrc(dt)
