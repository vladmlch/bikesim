"""Native forward on a stored golden state is bitwise-equal to Python forward."""
import copy
import os
import sys
from pathlib import Path
import mujoco
import numpy as np
import pytest

from native_loader import load_native
bike_native = load_native()

from _bits import assert_bitwise_equal
from native_safety_helpers import drive_pair, freeze


def test_set_state_snapshots_own_views_before_reset():
    mjb = 'tools/proto_native_bench/artifacts/model.mjb'
    model = mujoco.MjModel.from_binary_path(mjb)
    data = mujoco.MjData(model)
    data.qpos[0] += 0.37
    data.qvel[:] = np.linspace(-0.2, 0.4, model.nv)
    st = bike_native.Stepper(mjb)
    st.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart, 0.125)
    st.forward()
    # qacc is another owned buffer resetData clears: aliases are not limited
    # to the destination with the same name.
    qpos, qvel, warm = st.qpos, st.qvel, st.qacc
    saved = [a.copy() for a in (qpos, qvel, warm)]
    assert np.any(saved[2] != 0.0)
    st.set_state(qpos, qvel, data.act, warm, 0.25)
    assert_bitwise_equal(st.qpos, saved[0], 'aliased qpos restore')
    assert_bitwise_equal(st.qvel, saved[1], 'aliased qvel restore')
    assert st.time == 0.25
    mujoco.mj_resetData(model, data)
    data.qpos[:], data.qvel[:], data.qacc_warmstart[:] = saved
    data.time = 0.25
    mujoco.mj_forward(model, data)
    st.forward()
    assert_bitwise_equal(st.qacc, data.qacc, 'aliased warmstart restore')
    assert_bitwise_equal(st.qfrc_constraint, data.qfrc_constraint)

def _golden(tmp_path, steps=8):
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
    return tmp_path/'g'

@pytest.mark.slow
def test_forward_on_stored_state_is_bitwise(tmp_path):
    from tools.golden_episode import load_episode
    ep = load_episode(_golden(tmp_path))
    ref = mujoco.MjModel.from_binary_path(str(tmp_path/'g'/'model.mjb'))
    dref = mujoco.MjData(ref)
    st = bike_native.Stepper(str(tmp_path/'g'/'model.mjb'))
    for k in range(len(ep.state_qpos)):
        mujoco.mj_resetData(ref, dref)
        dref.qpos[:] = ep.state_qpos[k]; dref.qvel[:] = ep.state_qvel[k]
        dref.act[:] = ep.state_act[k]
        dref.qacc_warmstart[:] = ep.state_warmstart[k]
        dref.time = float(ep.state_time[k])
        mujoco.mj_forward(ref, dref)
        st.set_state(ep.state_qpos[k], ep.state_qvel[k], ep.state_act[k],
                     ep.state_warmstart[k], float(ep.state_time[k]))
        st.forward()
        assert_bitwise_equal(st.qacc, dref.qacc)
        assert_bitwise_equal(st.qfrc_constraint, dref.qfrc_constraint)
        assert_bitwise_equal(st.efc_force, dref.efc_force)

@pytest.mark.slow
def test_set_state_size_mismatch_raises_and_keeps_state(tmp_path):
    from tools.golden_episode import load_episode
    g = _golden(tmp_path)
    ep = load_episode(g)
    st = bike_native.Stepper(str(g/'model.mjb'))
    st.set_state(ep.state_qpos[0], ep.state_qvel[0], ep.state_act[0],
                 ep.state_warmstart[0], float(ep.state_time[0]))
    st.forward()
    before = np.asarray(st.qacc).copy()
    bad_qpos = np.zeros(ep.state_qpos.shape[1] + 1)
    with pytest.raises(ValueError):
        st.set_state(bad_qpos, ep.state_qvel[0], ep.state_act[0],
                     ep.state_warmstart[0], float(ep.state_time[0]))
    # The rejected call must not clobber the previously restored state:
    # every span is width-checked before mj_resetData runs.
    assert_bitwise_equal(np.asarray(st.qacc), before)


def _filter_stepper(tmp_path):
    """nq == nv == na == nu == 1 rig: the filter-dyn actuator gives ``act``
    a slot, so every set_state/set_inputs span carries an element."""
    m = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><body><joint name="a"/>'
        '<geom size=".1" mass="1"/></body></worldbody>'
        '<actuator><general joint="a" dyntype="filter"/></actuator></mujoco>')
    path = tmp_path / 'actrig.mjb'
    mujoco.mj_saveModel(m, str(path), None)
    return bike_native.Stepper(str(path))


def _live_buffers(st):
    """Byte copies of every readable live buffer plus the data clock."""
    arrays = {name: np.asarray(getattr(st, name)).copy() for name in
              ('qpos', 'qvel', 'qacc', 'qfrc_constraint', 'efc_force',
               'ctrl', 'qfrc_applied')}
    return arrays, st.time


def _assert_live_buffers(st, arrays, at_time):
    for name, before in arrays.items():
        assert_bitwise_equal(getattr(st, name), before,
                             f'{name} after rejected call')
    assert st.time == at_time


def test_state_rejection_preserves_buffers():
    """NaN in qpos raises set_state.qpos and never reaches mj_resetData."""
    mjb = 'tools/proto_native_bench/artifacts/model.mjb'
    model = mujoco.MjModel.from_binary_path(mjb)
    data = mujoco.MjData(model)
    data.qpos[0] += 0.37
    data.qvel[:] = np.linspace(-0.2, 0.4, model.nv)
    st = bike_native.Stepper(mjb)
    st.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart, 0.125)
    st.forward()
    st.set_inputs(np.linspace(0.1, 0.3, model.nu),
                  np.linspace(-0.1, 0.1, model.nv))
    arrays, at_time = _live_buffers(st)
    bad_qpos = data.qpos.copy()
    bad_qpos[0] = np.nan
    with pytest.raises(ValueError, match='set_state.qpos'):
        st.set_state(bad_qpos, data.qvel, data.act, data.qacc_warmstart, 0.25)
    _assert_live_buffers(st, arrays, at_time)


@pytest.mark.parametrize('field', ['qvel', 'act', 'warmstart'])
@pytest.mark.parametrize('bad_value', [np.nan, np.inf])
def test_set_state_nonfinite_span_rejects_atomically(tmp_path, field,
                                                     bad_value):
    st = _filter_stepper(tmp_path)
    st.set_state(np.array([.31]), np.array([-.2]), np.array([.45]),
                 np.array([.7]), .02)
    st.forward()
    st.set_inputs(np.array([.6]), np.array([.4]))
    arrays, at_time = _live_buffers(st)
    args = {'qpos': np.array([.31]), 'qvel': np.array([-.2]),
            'act': np.array([.45]), 'warmstart': np.array([.7])}
    args[field][0] = bad_value
    with pytest.raises(ValueError, match=f'set_state.{field}'):
        st.set_state(args['qpos'], args['qvel'], args['act'],
                     args['warmstart'], .03)
    _assert_live_buffers(st, arrays, at_time)


@pytest.mark.parametrize('bad_time', [-.5, np.nan, np.inf])
def test_set_state_bad_time_rejects_atomically(tmp_path, bad_time):
    st = _filter_stepper(tmp_path)
    st.set_state(np.array([.31]), np.array([-.2]), np.array([.45]),
                 np.array([.7]), .02)
    st.forward()
    arrays, at_time = _live_buffers(st)
    with pytest.raises(ValueError, match='set_state.time'):
        st.set_state(np.array([.31]), np.array([-.2]), np.array([.45]),
                     np.array([.7]), bad_time)
    _assert_live_buffers(st, arrays, at_time)


@pytest.mark.parametrize('field', ['ctrl', 'qfrc_applied'])
@pytest.mark.parametrize('bad_value', [np.nan, -np.inf])
def test_set_inputs_nonfinite_rejects_atomically(tmp_path, field, bad_value):
    st = _filter_stepper(tmp_path)
    st.set_inputs(np.array([.6]), np.array([.4]))
    arrays, at_time = _live_buffers(st)
    args = {'ctrl': np.array([.6]), 'qfrc_applied': np.array([.4])}
    args[field][0] = bad_value
    with pytest.raises(ValueError, match=f'set_inputs.{field}'):
        st.set_inputs(args['ctrl'], args['qfrc_applied'])
    _assert_live_buffers(st, arrays, at_time)


def test_set_state_accepts_cross_buffer_alias():
    """A qvel argument sharing the live qpos storage lands correctly: the
    binding snapshots every argument before mj_resetData runs."""
    mjb = 'tools/proto_native_bench/artifacts/model.mjb'
    model = mujoco.MjModel.from_binary_path(mjb)
    data = mujoco.MjData(model)
    data.qpos[:] = np.linspace(.11, .22, model.nq)
    data.qvel[:] = np.linspace(-.2, .4, model.nv)
    st = bike_native.Stepper(mjb)
    st.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart, .125)
    st.forward()
    qpos_before = np.asarray(st.qpos).copy()
    alias = np.asarray(st.qpos)
    assert np.shares_memory(alias, st.qpos)
    data.qpos[:] = np.linspace(.5, .7, model.nq)
    st.set_state(data.qpos, alias, data.act, data.qacc_warmstart, .5)
    assert_bitwise_equal(st.qvel, qpos_before,
                         'qvel restored through live-qpos alias')
    assert_bitwise_equal(st.qpos, data.qpos)
    assert st.time == .5


def test_set_state_rejection_with_aliased_buffers_preserves_state():
    """Rejection is equally atomic when an input aliases a live buffer — and
    when the NaN rides a foreign shared-memory view."""
    mjb = 'tools/proto_native_bench/artifacts/model.mjb'
    model = mujoco.MjModel.from_binary_path(mjb)
    data = mujoco.MjData(model)
    data.qpos[:] = np.linspace(.11, .22, model.nq)
    data.qvel[:] = np.linspace(-.2, .4, model.nv)
    st = bike_native.Stepper(mjb)
    st.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart, .125)
    st.forward()
    arrays, at_time = _live_buffers(st)
    # Live-buffer view in the qpos slot; the NaN arrives via qvel.
    bad_qvel = data.qvel.copy()
    bad_qvel[3] = np.nan
    with pytest.raises(ValueError, match='set_state.qvel'):
        st.set_state(np.asarray(st.qpos), bad_qvel, data.act,
                     data.qacc_warmstart, .75)
    _assert_live_buffers(st, arrays, at_time)
    # A NaN written through the aliased mjData view itself.
    data.qpos[0] = np.nan
    with pytest.raises(ValueError, match='set_state.qpos'):
        st.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart, .75)
    _assert_live_buffers(st, arrays, at_time)


def _reject_drive_state(n, mutate, match):
    """A rejected restore must leave the whole drive snapshot untouched."""
    before = n.drive_state()
    bad = copy.deepcopy(before)
    mutate(bad)
    with pytest.raises(ValueError, match=match):
        n.set_drive_state(bad)
    assert freeze(n.drive_state()) == freeze(before)


@pytest.mark.parametrize('kind', ['ideal_mid_drive',
                                  'geometric_ideal_mid_drive'])
@pytest.mark.parametrize('path,spec,match', [
    # The ideal-hub ratio is always the shifter's current gear.
    ('ideal_hub.ratio', 'x1.5', 'ratio/gear'),
    # The transmission sprocket tracks shifting.rear_teeth (geometric) or
    # the configured gearing (ideal).
    ('ideal_hub.rear_teeth', 'i28', 'rear-teeth'),
    ('ideal_hub.coefficients', 'short', 'coefficient width'),
    # The gear-ratio relation reaches the shifter's sprocket even with the
    # shifter disabled (cassette membership applies only when enabled).
    ('policies.shifting.rear_teeth', 'i25', 'ratio/gear'),
    # count == 0 requires no shift clock, direction 'none', zero cooldowns.
    ('policies.shifting.direction', 'up', 'shift clock/count'),
    ('policies.shifting.cooldown_s', 'half', 'shift clock/count'),
    ('policies.shifting.cut_remaining_s', 'half', 'shift clock/count'),
    ('shift_time_s', 'half', 'shift clock/count'),
    # The store is sized by the config and only ever drains.
    ('policies.battery.energy_j', 'over', 'battery energy'),
    ('policies.battery.drawn_energy_j', 'over', 'battery energy'),
    ('policies.battery.initial_energy_j', 'over', 'battery energy'),
])
def test_drive_restore_relational_rejection_is_atomic(tmp_path, kind, path,
                                                      spec, match):
    _, _, _, n = drive_pair(tmp_path, kind=kind)

    def mutate(bad):
        parent = bad
        for part in path.split('.')[:-1]:
            parent = parent[part]
        key = path.rsplit('.', 1)[-1]
        if spec == 'x1.5':
            parent[key] = parent[key] * 1.5
        elif spec == 'short':
            parent[key] = parent[key][:-1]
        elif spec == 'i28':
            parent[key] = 28
        elif spec == 'i25':
            parent[key] = 25
        elif spec == 'up':
            parent[key] = 'up'
        elif spec == 'half':
            parent[key] = .5
        elif spec == 'over':
            parent[key] = parent['initial_energy_j'] + 1.

    _reject_drive_state(n, mutate, match)


@pytest.mark.parametrize('kind,match', [
    ('ideal_mid_drive', 'coefficient/ratio'),
    ('geometric_ideal_mid_drive', 'coefficients/prepared Jacobian'),
])
def test_drive_restore_rejects_coefficient_drift(tmp_path, kind, match):
    """The ideal hub's single wrap coefficient IS the ratio; the geometric
    hub's coefficients must reproduce the prepared Jacobian."""
    _, _, _, n = drive_pair(tmp_path, kind=kind)
    _reject_drive_state(
        n, lambda b: b['ideal_hub']['coefficients'].__setitem__(
            0, b['ideal_hub']['coefficients'][0] + 1.), match)


def test_drive_restore_rejects_geometric_prepared_jacobian_drift(tmp_path):
    _, _, _, n = drive_pair(tmp_path, kind='geometric_ideal_mid_drive')
    _reject_drive_state(
        n, lambda b: b['ideal_hub']['prepared']['jacobian'].__setitem__(
            0, b['ideal_hub']['prepared']['jacobian'][0] + 1.),
        'coefficients/prepared Jacobian')


def test_drive_restore_rejects_pending_actuation_without_interval(tmp_path):
    """pending_actuation exists only after a compute that also set the
    interval clock — both directions of the relation are checked."""
    _, _, _, n = drive_pair(tmp_path)
    state = n.drive_state()
    assert state['pending_actuation'] is None and state['last_time_s'] is None
    _reject_drive_state(
        n, lambda b: b.update(pending_actuation={
            'requested': 1., 'omega': 2., 'dt': .002, 'enabled': True}),
        'live interval')
    n.drive_components({}, .002, 2.)
    state = n.drive_state()
    assert state['pending_actuation'] is not None and \
        state['last_time_s'] is not None
    _reject_drive_state(n, lambda b: b.update(last_time_s=None),
                        'live interval')


@pytest.mark.parametrize('dt', [0., -.002])
def test_drive_restore_rejects_nonpositive_pending_dt(tmp_path, dt):
    _, _, _, n = drive_pair(tmp_path)
    n.drive_components({}, .002, 2.)
    assert n.drive_state()['pending_actuation'] is not None
    _reject_drive_state(
        n, lambda b: b['pending_actuation'].update(dt=dt), 'pending')


def test_drive_restore_rejects_orphaned_shift_pending_on_ideal_hub(tmp_path):
    """shift_pending only exists on a geometric transmission with a prepared
    slot — an ideal hub can never carry it."""
    _, _, _, n = drive_pair(tmp_path)
    _reject_drive_state(
        n, lambda b: b['ideal_hub'].update(shift_pending=True),
        'shift_pending')


def test_drive_restore_geometric_shift_pending_accepted(tmp_path):
    """The same flag is legitimate on a geometric hub mid-shift."""
    _, _, _, n = drive_pair(tmp_path, kind='geometric_ideal_mid_drive')
    state = n.drive_state()
    assert state['ideal_hub']['prepared'] is not None
    state['ideal_hub']['shift_pending'] = True
    n.set_drive_state(state)
    assert freeze(n.drive_state()) == freeze(state)


def test_drive_restore_cassette_membership_when_shifter_enabled(tmp_path):
    from bike_sim.physics.physical_config import ShiftingConfig
    _, _, _, n = drive_pair(tmp_path, kind='ideal_mid_drive',
                            shifting=ShiftingConfig(
                                enabled=True, cadence_smoothing_tau_s=0.))
    assert n.drive_state()['policies']['shifting']['shift_count'] == 0
    for key in ('rear_teeth', 'from_teeth'):
        def mutate(bad, key=key):
            bad['policies']['shifting'][key] = 25  # outside CASSETTE_12S_TEETH
        _reject_drive_state(n, mutate, 'cassette')


@pytest.mark.parametrize('topology,name', [('clutch', 'clutch'),
                                           ('rotor', 'freewheel')])
@pytest.mark.parametrize('key,value', [
    ('ratio', 1.5), ('rear_teeth', 4),
    ('coefficients', np.array([2.])),
    ('coefficients', np.array([1., 1.])),
])
def test_drive_restore_rejects_auxiliary_transmission_drift(tmp_path, topology,
                                                            name, key, value):
    """Clutch/freewheel transmissions are fixed 1:1 one-way clutches with a
    single unit coefficient — the snapshot cannot retopologize them."""
    _, _, _, n = drive_pair(tmp_path, kind='ideal_mid_drive',
                            topology=topology)

    def mutate(bad):
        bad[name][key] = value

    _reject_drive_state(n, mutate, 'auxiliary')


def test_drive_policies_set_state_relational_rejections(tmp_path):
    """The standalone policy binding repeats the same relational guards:
    shift clock/count, cassette membership, battery bounds."""
    from bike_sim.physics.physical_config import ShiftingConfig
    from tools.native_config import project_drive_policies
    _, _, p, _ = drive_pair(tmp_path, kind='ideal_mid_drive',
                            shifting=ShiftingConfig(
                                enabled=True, cadence_smoothing_tau_s=0.))
    native = bike_native.DrivePolicies(project_drive_policies(p))
    before = native.state()
    cases = [
        ('shifting', 'direction', 'up'),
        ('shifting', 'cooldown_s', .5),
        ('shifting', 'cut_remaining_s', .1),
        ('shifting', 'rear_teeth', 25),      # outside the cassette
        ('battery', 'energy_j', 'over'),
        ('battery', 'drawn_energy_j', 'over'),
        ('battery', 'initial_energy_j', 'over'),
    ]
    for section, key, value in cases:
        bad = copy.deepcopy(before)
        bad[section][key] = (before['battery']['initial_energy_j'] + 1.
                             if value == 'over' else value)
        with pytest.raises(ValueError):
            native.set_state(bad)
        assert freeze(native.state()) == freeze(before), (section, key)
