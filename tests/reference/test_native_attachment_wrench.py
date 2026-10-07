"""Native equality reactions + Accelerate DGELSD least squares (T2a).

Every native reading is taken on the Stepper's owned (mjModel, mjData)
pair — the same solved EFC arena the Python oracle reads — so the outputs
must be bitwise identical. Equality rows are re-derived from the CURRENT
efc_type/efc_id arena on every evaluation: membership is never cached.
The least-squares hook runs the same Accelerate ILP64 DGELSD
(_dgelsd$NEWLAPACK$ILP64) that numpy.linalg.lstsq dispatches to on this
platform, with numpy's exact buffer layout, workspace query, FP-flag
error semantics, and output contract.
"""
import dataclasses
import math

import mujoco
import numpy as np
import pytest

from _bits import assert_bitwise_equal
from native_loader import load_native

from bike_sim.physics.attachment_budget import AttachmentSample
from bike_sim.sim.ride.attachment_wrench import (
    attachment_raw, attachment_raw_from_geometry, attachment_sample,
    decompose_wrench, equality_qfrc, prepare_attachment_geometry,
    recover_wrench, relative_planar_jacobian)
from bike_sim.sim.ride.weld_pedals import equality_rows, PedalWelds

bike_native = load_native()

_EQUALITY = int(mujoco.mjtConstraint.mjCNSTR_EQUALITY)


def bits(actual, expected, msg=''):
    assert_bitwise_equal(np.asarray(actual), np.asarray(expected), msg)


def _pair_model(equality_xml, *, extra_worldbody='', option=''):
    """Small 'rider' body equalized to a heavier 'bike' body (the
    test_attachment_wrench._pair_model shape, with room for unrelated
    constraint rows and extra equalities)."""
    equality = f'<equality>{equality_xml}</equality>' if equality_xml else ''
    return mujoco.MjModel.from_xml_string(f'''
<mujoco><option gravity="0 0 0" timestep="0.0005" {option}/>
<worldbody>
  <geom name="floor" type="plane" size="5 5 1" pos="0 0 0"/>
  <body name="bike" pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1e3" diaginertia="1e3 1e3 1e3"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="rider" pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="hand" pos=".2 0 1.1"><freejoint/>
    <inertial pos="0 0 0" mass=".5" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="ja_b" pos=".5 0 1"><joint name="ja" type="hinge" axis="0 1 0"/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="jb_b" pos=".7 0 1"><joint name="jb" type="hinge" axis="0 1 0"/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="lim_b" pos="-.5 0 1">
    <joint name="lim" type="hinge" axis="0 1 0" range="-0.02 0.02"/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="ball" pos="0.4 0 0.09"><freejoint/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.1"/></body>
  {extra_worldbody}
</worldbody>
{equality}</mujoco>''')


_ALL_EQUALITIES = ('<weld name="w" body1="rider" body2="bike"/>'
                   '<joint name="j" joint1="ja" joint2="jb"/>'
                   '<connect name="c" body1="hand" body2="bike" '
                   'anchor="0 .1 0"/>')


def _lambda(data, eq_id):
    """Solved constraint multipliers of one equality (efc rows)."""
    n = data.nefc
    sel = ((data.efc_type[:n] == _EQUALITY) & (data.efc_id[:n] == eq_id))
    return data.efc_force[:n][sel]


def _native_mirror(model, data, tmp_path):
    """A Stepper over the same model replaying the same solver inputs."""
    path = tmp_path / 'attachment.mjb'
    mujoco.mj_saveModel(model, str(path))
    stepper = bike_native.Stepper(str(path))
    stepper.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart,
                      data.time)
    stepper.set_inputs(data.ctrl, data.qfrc_applied)
    return stepper


def _settle(model, data, tmp_path, steps=100):
    """Step Python + native side by side; the arenas must stay bitwise
    identical the whole way (no second simulation owner, no drift)."""
    stepper = _native_mirror(model, data, tmp_path)
    for _ in range(steps):
        mujoco.mj_step(model, data)
        stepper.step()
        bits(stepper.qpos, data.qpos, 'qpos trajectory diverged')
        bits(stepper.qvel, data.qvel, 'qvel trajectory diverged')
    return stepper


def _drive(model, data, force=(30., -80., 12.5)):
    """Push the 'rider' free body through qfrc_applied and break the 'lim'
    hinge's joint range so unrelated limit rows enter the arena. Every
    equality in _ALL_EQUALITIES gets a real load: the 'ja' hinge torque
    must cross the joint equality, and the 'hand' free body loads the
    'c' connect."""
    data.qpos[model.joint('lim').qposadr[0]] = .4
    mujoco.mj_forward(model, data)
    rider_dof = int(model.body_dofadr[model.body('rider').id])
    data.qfrc_applied[rider_dof] = force[0]
    data.qfrc_applied[rider_dof + 2] = force[1]
    data.qfrc_applied[rider_dof + 4] = force[2]
    data.qfrc_applied[int(model.joint('ja').dofadr[0])] = 6.5
    hand_dof = int(model.body_dofadr[model.body('hand').id])
    data.qfrc_applied[hand_dof] = 9.
    data.qfrc_applied[hand_dof + 1] = -4.5


def _expected_reaction(model, data, eq_id):
    """The weld_pedals.py reader contract for one equality id."""
    n = data.nefc
    rows = np.flatnonzero((data.efc_type[:n] == _EQUALITY)
                          & (data.efc_id[:n] == eq_id))
    if rows.size == 0:
        return (rows, np.zeros(3), 0., equality_qfrc(model, data, eq_id))
    lam = np.asarray(data.efc_force[rows][:3], dtype=float)
    residual = float(np.linalg.norm(data.efc_pos[rows][:3]))
    return rows, lam, residual, equality_qfrc(model, data, eq_id)


@pytest.mark.parametrize('jacobian', ['sparse', 'dense'])
def test_rider_equality_qfrc_oracle(jacobian, tmp_path):
    """stepper.rider_equality_qfrc == equality_qfrc on the shared arena,
    bitwise, while contacts and a violated joint limit interleave
    non-equality rows through the equality blocks."""
    model = _pair_model(_ALL_EQUALITIES, option=f'jacobian="{jacobian}"')
    data = mujoco.MjData(model)
    _drive(model, data)
    stepper = _settle(model, data, tmp_path)
    types = data.efc_type[:data.nefc]
    assert np.any(types != _EQUALITY)  # unrelated rows really present
    assert np.any(types == int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT))
    for name in ('w', 'j', 'c'):
        eq_id = int(model.equality(name).id)
        actual = stepper.rider_equality_qfrc(eq_id)
        expected = equality_qfrc(model, data, eq_id)
        bits(actual, expected, f'qfrc equality {name}')
        lam = _lambda(data, eq_id)
        assert lam.size >= 1 and np.abs(lam).max() > 0.
    # Row membership is re-derived per call: a bogus id selects nothing.
    bits(stepper.rider_equality_qfrc(999), np.zeros(model.nv))
    bits(equality_qfrc(model, data, 999), np.zeros(model.nv))
    # A contact/limit row's efc_id must not be mistaken for an equality id:
    # the type half of the mask excludes it even when ids collide.
    limit_rows = np.flatnonzero(
        types == int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT))
    assert limit_rows.size >= 1
    limit_id = int(data.efc_id[limit_rows[0]])
    bits(stepper.rider_equality_qfrc(limit_id),
         equality_qfrc(model, data, limit_id))


def test_rider_equality_rows_grouping(tmp_path):
    """_rider_equality_rows == weld_pedals.equality_rows: one fresh pass,
    stable ascending groups, unrelated row types excluded."""
    model = _pair_model(_ALL_EQUALITIES)
    data = mujoco.MjData(model)
    _drive(model, data)
    stepper = _settle(model, data, tmp_path, steps=40)
    expected = equality_rows(data)
    actual = bike_native._rider_equality_rows(stepper)
    assert actual.keys() == expected.keys() == {
        int(model.equality(n).id) for n in ('w', 'j', 'c')}
    for eq_id, rows in expected.items():
        assert actual[eq_id].dtype == np.int64
        assert_bitwise_equal(actual[eq_id], rows)


@pytest.mark.parametrize('eq_name', ['w', 'j', 'c'])
def test_rider_equality_reaction_oracle(eq_name, tmp_path):
    """force_on_rider / translation_residual / qfrc for weld (6 rows),
    joint (1 row) and connect (3 rows) equalities — including the <3
    slice length the Python lam[:3] produces for a joint equality."""
    model = _pair_model(_ALL_EQUALITIES)
    data = mujoco.MjData(model)
    _drive(model, data)
    stepper = _settle(model, data, tmp_path)
    eq_id = int(model.equality(eq_name).id)
    actual = bike_native._rider_equality_reaction(stepper, eq_id)
    rows, force, residual, qfrc = _expected_reaction(model, data, eq_id)
    assert_bitwise_equal(actual['rows'], rows)
    bits(actual['force_on_rider_n'], force)
    bits(actual['translation_residual_m'], np.float64(residual))
    bits(actual['qfrc'], qfrc)


def test_rider_equality_reaction_missing_rows_zero(tmp_path):
    """A missing equality is 'no measurement': zero force, zero residual,
    zero qfrc — never invented values."""
    model = _pair_model('<weld name="w" body1="rider" body2="bike"/>')
    data = mujoco.MjData(model)
    _drive(model, data)
    stepper = _settle(model, data, tmp_path)
    actual = bike_native._rider_equality_reaction(stepper, 77)
    assert_bitwise_equal(actual['rows'], np.empty(0, dtype=np.int64))
    bits(actual['force_on_rider_n'], np.zeros(3))
    assert actual['translation_residual_m'] == 0.
    bits(actual['qfrc'], np.zeros(model.nv))


def test_rider_equality_qfrc_empty_arena(tmp_path):
    """No constraints at all: nefc == 0, zeros(nv) — mj_mulJacTVec's
    early return must not leak an uninitialized output."""
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body pos="0 0 1"><freejoint/>
        <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
        <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
    </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert data.nefc == 0
    stepper = _native_mirror(model, data, tmp_path)
    bits(stepper.rider_equality_qfrc(0), np.zeros(model.nv))
    bits(equality_qfrc(model, data, 0), np.zeros(model.nv))
    # The multi-equality torque diagnostic reports 'no measurement' too.
    assert bike_native._rider_equalities_qfrc_at(stepper, [0], 0) == 0.


def _pedal_model():
    """Two weld equalities on distinct foot bodies plus a crank hinge —
    PedalWelds' 'weld_foot_{front,rear}' + 'crank_spin' contract."""
    return mujoco.MjModel.from_xml_string('''
<mujoco><option gravity="0 0 0" timestep="0.0005"/>
<worldbody>
  <body name="bike" pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1e3" diaginertia="1e3 1e3 1e3"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/>
    <body name="crank" pos="0 .2 0">
      <joint name="crank_spin" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 0" mass="5" diaginertia="1 1 1"/>
      <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  </body>
  <body name="foot_a" pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="foot_b" pos="0 .4 1"><freejoint/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
</worldbody>
<equality>
  <weld name="weld_foot_front" body1="foot_a" body2="crank"/>
  <weld name="weld_foot_rear" body1="foot_b" body2="bike"/>
</equality></mujoco>''')


def test_rider_pedal_torque_oracle(tmp_path):
    """Sum-of-pedal-equalities torque at the crank dof ==
    PedalWelds.delivered_crank_torque_nm."""
    model = _pedal_model()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    foot_a = model.body('foot_a').id
    foot_b = model.body('foot_b').id
    data.qfrc_applied[int(model.body_dofadr[foot_a]) + 0] = 22.
    data.qfrc_applied[int(model.body_dofadr[foot_a]) + 2] = -55.
    data.qfrc_applied[int(model.body_dofadr[foot_b]) + 0] = -11.
    data.qfrc_applied[int(model.body_dofadr[foot_b]) + 2] = -37.
    stepper = _settle(model, data, tmp_path)
    welds = PedalWelds(model)
    expected = welds.delivered_crank_torque_nm(model, data)
    eq_ids = sorted(welds.eq_ids.values())
    actual = bike_native._rider_equalities_qfrc_at(stepper, eq_ids,
                                                 welds.crank_dof)
    bits(actual, np.float64(expected))
    # A nonexistent equality contributes nothing — same as a zero mask.
    only = bike_native._rider_equalities_qfrc_at(stepper, [eq_ids[0]],
                                               welds.crank_dof)
    only_missing = bike_native._rider_equalities_qfrc_at(
        stepper, [eq_ids[0], 999], welds.crank_dof)
    bits(only_missing, np.float64(only))


def test_stepper_qfrc_returns_owning_array(tmp_path):
    model = _pair_model('<weld name="w" body1="rider" body2="bike"/>')
    data = mujoco.MjData(model)
    _drive(model, data)
    stepper = _settle(model, data, tmp_path)
    eq_id = int(model.equality('w').id)
    first = stepper.rider_equality_qfrc(eq_id)
    second = stepper.rider_equality_qfrc(eq_id)
    # The capsule owns the buffer (OWNDATA is False — base is the
    # capsule, not self): the contract that matters is a fresh,
    # writeable, unshared allocation on every call.
    assert first.dtype == np.float64 and first.flags.writeable
    assert not np.may_share_memory(first, second)
    first[:] = 999.
    bits(second, equality_qfrc(model, data, eq_id))


# ------------------------------- DGELSD lstsq -----------------------------

def _lstsq_oracle(a, b, rcond=1e-12):
    """Run both solvers; every output must agree bitwise — or both must
    raise the same 'SVD did not converge' failure."""
    try:
        expected = np.linalg.lstsq(a, b, rcond=rcond)
    except np.linalg.LinAlgError as error:
        with pytest.raises(ValueError, match='SVD did not converge'):
            bike_native._rider_least_squares(a, b, rcond)
        return None
    actual = bike_native._rider_least_squares(a, b, rcond)
    x, resids, rank, s = actual
    ex_x, ex_resids, ex_rank, ex_s = expected
    assert_bitwise_equal(x, ex_x, 'x')
    assert_bitwise_equal(resids, ex_resids, 'resids')
    assert int(ex_rank) == int(rank)
    assert_bitwise_equal(s, ex_s, 'singular values')
    return expected


@pytest.mark.parametrize('m,n,seed', [(8, 3, 1), (3, 8, 2), (5, 5, 3),
                                      (10, 4, 4), (4, 10, 5), (1, 1, 6),
                                      (7, 7, 7), (2, 9, 8), (9, 2, 9)])
def test_least_squares_oracle(m, n, seed):
    rng = np.random.default_rng(seed)
    a = rng.normal(size=(m, n))
    _lstsq_oracle(a, rng.normal(size=m))
    _lstsq_oracle(a, rng.normal(size=(m, 3)))
    _lstsq_oracle(a, rng.normal(size=(m, 1)))


@pytest.mark.parametrize('n_rhs', [0, 1, 2, 5])
def test_least_squares_rhs_count(n_rhs):
    rng = np.random.default_rng(47)
    a = rng.normal(size=(6, 3))
    _lstsq_oracle(a, rng.normal(size=(6, n_rhs)))


@pytest.mark.parametrize('degenerate', [(0, 3), (3, 0), (0, 0), (1, 4)])
def test_least_squares_degenerate_dims(degenerate):
    """m == 0 (zero problem) and n == 0 (empty basis) follow numpy's exact
    contract: x zeroed, resids = sum of squared b rows when n == 0."""
    m, n = degenerate
    rng = np.random.default_rng(911)
    a = rng.normal(size=(m, n))
    _lstsq_oracle(a, rng.normal(size=m))


def test_least_squares_rank_deficient():
    rng = np.random.default_rng(99)
    for m, n in [(6, 4), (4, 4), (9, 3)]:
        a = rng.normal(size=(m, n))
        a[:, -1] = a[:, 0] * 2. + a[:, 1] * -0.5  # exact dependence
        _lstsq_oracle(a, rng.normal(size=m))
        _lstsq_oracle(a, rng.normal(size=(m, 2)))


@pytest.mark.parametrize('ratio', [0.9, 0.99, 1.0, 1.01, 1.1])
def test_least_squares_rcond_boundary(ratio):
    """Singular values straddling the rcond*s_max cutoff — rank flips
    must match numpy exactly (DGELSD keeps sv <= cutoff out)."""
    a = np.diag([1., 1e-12 * ratio])
    _lstsq_oracle(a, np.array([1., 2.]), rcond=1e-12)


@pytest.mark.parametrize('rcond', [1e-12, -1., 0., .5, 1e-3])
def test_least_squares_rcond_values(rcond):
    rng = np.random.default_rng(2024)
    a = rng.normal(size=(6, 4))
    _lstsq_oracle(a, rng.normal(size=6), rcond=rcond)


def test_least_squares_inconsistent_target():
    """Full-rank overdetermined system with a genuine residual — the
    resids array must carry numpy's squared residual bitwise."""
    a = np.array([[1., 0.], [0., 1.], [0., 0.]])
    _lstsq_oracle(a, np.array([1., 2., 3.]))
    rng = np.random.default_rng(31337)
    a = rng.normal(size=(8, 3))
    _lstsq_oracle(a, rng.normal(size=8) * 10.)


def test_least_squares_wide_minimum_norm():
    """Underdetermined: minimum-norm x, empty resids."""
    rng = np.random.default_rng(777)
    a = rng.normal(size=(3, 9))
    _lstsq_oracle(a, rng.normal(size=3))


@pytest.mark.parametrize('a,b', [
    (np.array([[1., np.nan], [0., 1.]]), np.array([1., 2.])),
    (np.array([[1., 2.], [np.nan, 0.], [0., 1.]]), np.array([1., 2., 3.])),
    (np.array([[1., np.inf], [0., 1.]]), np.array([1., 2.])),
    (np.array([[1., 2.]]), np.array([np.inf])),
    (np.array([[1., 0.], [0., 1.]]), np.array([np.nan, 1.])),
    (np.array([[1e308, 1e308]] * 4), np.array([1., 2., 3., 4.])),
])
def test_least_squares_nonfinite(a, b):
    """Nonfinite input is not pre-screened — it flows through DGELSD
    exactly like numpy: SVD non-convergence raises the same error;
    'successful' nonfinite solves return identical nonfinite output."""
    _lstsq_oracle(a, b)


@pytest.mark.parametrize('a,b', [
    (np.eye(3), np.zeros(4)),          # B rows != m -> Incompatible dimensions
    (np.zeros(4), np.zeros(4)),        # A 1-D
    (np.zeros((3, 2)), np.array(0.)),  # scalar B
    (np.eye(3), np.zeros((3, 1, 1))),  # B 3-D
    (np.zeros((3, 2, 2)), np.zeros(3)),  # A 3-D — numpy _assert_2d rejects
])
def test_least_squares_shape_errors(a, b):
    with pytest.raises(ValueError):
        np.linalg.lstsq(a, b, rcond=1e-12)
    with pytest.raises(ValueError):
        bike_native._rider_least_squares(a, b, 1e-12)


def test_least_squares_incompatible_dimensions_message():
    with pytest.raises(ValueError, match='Incompatible dimensions'):
        np.linalg.lstsq(np.eye(3), np.zeros(4), rcond=1e-12)
    with pytest.raises(ValueError, match='Incompatible dimensions'):
        bike_native._rider_least_squares(np.eye(3), np.zeros(4), 1e-12)


def test_least_squares_output_layout_and_ownership():
    rng = np.random.default_rng(5150)
    a32 = rng.normal(size=(6, 3)).astype(np.float32)
    b32 = rng.normal(size=6).astype(np.float32)
    # The hook is the float64 port (recover_wrench inputs): non-f64
    # dtypes convert to f64 before solving, while numpy would dispatch
    # f32 to sgelsd — so the bitwise oracle for widened inputs is lstsq
    # on the f64 casts, and the port's own output dtype stays float64.
    ex_x, ex_r, ex_rank, ex_s = np.linalg.lstsq(
        a32.astype(np.float64), b32.astype(np.float64), rcond=1e-12)
    ac_x, ac_r, ac_rank, ac_s = bike_native._rider_least_squares(
        a32, b32, 1e-12)
    assert ac_x.dtype == np.float64
    assert_bitwise_equal(ac_x, ex_x)
    # Non-contiguous input strides are accepted and identical.
    a_nc = rng.normal(size=(10, 3))[::2]
    b_nc = rng.normal(size=(8,))[::2]  # wrong length — keep b consistent:
    b_nc = rng.normal(size=(10, 2))[::2][:, 0]
    assert not a_nc.flags.c_contiguous or not b_nc.flags.c_contiguous
    _lstsq_oracle(a_nc, b_nc)
    # Every call returns fresh owned storage: mutating one result must not
    # disturb another, and outputs never alias the input arrays.
    a = rng.normal(size=(5, 3))
    b = rng.normal(size=5)
    x1, r1, k1, s1 = bike_native._rider_least_squares(a, b, 1e-12)
    x2, r2, k2, s2 = bike_native._rider_least_squares(a, b, 1e-12)
    assert x1.dtype == np.float64 and x1.flags.writeable
    # Capsule-owned buffers: OWNDATA is False by construction, so the
    # check that carries the contract is unshared writeable storage.
    assert not np.may_share_memory(x1, x2)
    assert not np.may_share_memory(s1, s2)
    assert not np.may_share_memory(x1, np.asarray(a))
    x1[:] = 0.
    assert_bitwise_equal(x2, np.linalg.lstsq(a, b, rcond=1e-12)[0])


# --------------------- T2b: solved attachment measurements -----------------
#
# Every Stepper measurement below is read on the same solved EFC arena the
# Python oracle reads — identical mj_step calls on the paired (m_, d_), so
# jacobians, qfrc slices, gaps and samples must be bitwise identical, and a
# ValueError on the Python side must be the same ValueError (same message)
# on the native side. `rotational` below mirrors the oracle's keyword.


def _attachment_ids(model):
    return (model.body('rider').id, model.body('bike').id)


def _assert_geometry(actual, expected, msg=''):
    """attachment dict <-> AttachmentGeometry, field by field, bitwise."""
    assert actual['eq_id'] == expected.eq_id, msg
    bits(actual['rider_jac'], expected.rider_jac, f'{msg} rider_jac')
    bits(actual['rider_columns'], expected.rider_columns,
         f'{msg} rider_columns')
    bits(actual['bike_jac'], expected.bike_jac, f'{msg} bike_jac')
    bits(actual['bike_columns'], expected.bike_columns,
         f'{msg} bike_columns')
    assert actual['observable'] == expected.observable, msg
    bits(actual['normal'], expected.normal, f'{msg} normal')
    assert actual['kind'] == expected.kind, msg
    assert actual['rotational'] == expected.rotational, msg
    bits(actual['half_patch_m'], np.float64(expected.half_patch_m),
         f'{msg} half_patch_m')
    if expected.pull_direction is None:
        assert actual['pull_direction'] is None, msg
    else:
        bits(actual['pull_direction'], expected.pull_direction,
             f'{msg} pull_direction')


def _assert_raw(actual, expected, msg=''):
    """attachment raw dict <-> AttachmentRaw, field by field, bitwise."""
    bits(actual['rider_jac'], expected.rider_jac, f'{msg} rider_jac')
    bits(actual['rider_qfrc'], expected.rider_qfrc, f'{msg} rider_qfrc')
    bits(actual['bike_jac'], expected.bike_jac, f'{msg} bike_jac')
    bits(actual['bike_qfrc'], expected.bike_qfrc, f'{msg} bike_qfrc')
    assert actual['observable'] == expected.observable, msg
    bits(actual['normal'], expected.normal, f'{msg} normal')
    assert actual['kind'] == expected.kind, msg
    assert actual['rotational'] == expected.rotational, msg
    bits(actual['half_patch_m'], np.float64(expected.half_patch_m),
         f'{msg} half_patch_m')
    bits(actual['gap_m'], np.float64(expected.gap_m), f'{msg} gap_m')
    if expected.pull_direction is None:
        assert actual['pull_direction'] is None, msg
    else:
        bits(actual['pull_direction'], expected.pull_direction,
             f'{msg} pull_direction')


def _assert_sample(actual, expected, msg=''):
    """sample dict <-> AttachmentSample, field by field, bitwise."""
    expected_dict = dataclasses.asdict(expected)
    assert set(actual.keys()) == set(expected_dict.keys()), msg
    for key, value in expected_dict.items():
        if isinstance(value, str):
            assert actual[key] == value, f'{msg} {key}'
        else:
            bits(actual[key], np.float64(value), f'{msg} {key}')


def test_native_recover_wrench():
    """The plan's worked example: recover_wrench bitwise on a fixed
    relative jacobian and the generalized force it explains."""
    jac = np.array([[1., 0., 0., -1.], [0., 1., 0., 0.],
                    [0., 0., 1., 0.]])
    force = jac.T @ np.array([20., 100., -2.])
    bits(bike_native.rider_recover_wrench(jac, force),
         recover_wrench(jac, force))


def _recover_oracle(jac, qfrc):
    """recover_wrench bitwise, or the same ValueError with the same
    message ('nonfinite wrench input' / 'rank-deficient attachment
    Jacobian' / 'does not explain generalized force' / 'SVD did not
    converge in Linear Least Squares')."""
    try:
        expected = recover_wrench(jac, qfrc)
    except ValueError as error:
        with pytest.raises(ValueError) as raised:
            bike_native.rider_recover_wrench(jac, qfrc)
        assert str(raised.value) == str(error)
        return None
    actual = bike_native.rider_recover_wrench(jac, qfrc)
    assert actual.dtype == np.float64 and actual.flags.writeable
    bits(actual, expected)
    return expected


@pytest.mark.parametrize('shape,seed', [((3, 8), 1), ((8, 3), 2),
                                        ((2, 9), 3), ((9, 2), 4),
                                        ((5, 5), 5), ((4, 6), 6),
                                        ((1, 1), 7), ((3, 1), 8),
                                        ((1, 4), 9)])
def test_recover_wrench_random_oracle(shape, seed):
    """Tall/wide/square relative jacobians, consistent and arbitrary
    targets — full-rank rows recover, rank-deficient or unexplained
    systems raise the same ValueError."""
    rng = np.random.default_rng(seed)
    jac = rng.normal(size=shape)
    wrench = rng.normal(size=shape[0])
    _recover_oracle(jac, jac.T @ wrench)               # consistent
    _recover_oracle(jac, jac.T @ wrench + 1e-12 * rng.normal(size=shape[1]))
    _recover_oracle(jac, rng.normal(size=shape[1]))    # arbitrary
    _recover_oracle(jac, rng.normal(size=shape[1]) * 1e6)


def test_recover_wrench_rank_deficient_and_inconsistent():
    _recover_oracle(np.zeros((2, 4)), np.zeros(4))
    rng = np.random.default_rng(31)
    jac = rng.normal(size=(4, 6))
    jac[3] = jac[0] * 2. - jac[1]                    # exact dependence
    _recover_oracle(jac, rng.normal(size=6))
    jac = rng.normal(size=(3, 3))
    jac[1] = jac[0]                                  # duplicated row
    _recover_oracle(jac, np.array([1., 1., 1.]))
    # Full-rank rows but the target is not a wrench on this support.
    _recover_oracle(np.array([[1., 0., 0., -1.], [0., 1., 0., 0.]]),
                    np.array([1., 0., 0., 1.]))


@pytest.mark.parametrize('ratio', [0.9, 0.99, 1.0, 1.01, 1.1])
def test_recover_wrench_rcond_boundary(ratio):
    """rank vs matrix.shape[1] at the pinned rcond=1e-12 cutoff — the
    same DGELSD, so the rank flip is bitwise-identical to the oracle's."""
    jac = np.diag([1., 1e-12 * ratio])
    _recover_oracle(jac, np.array([1., 2.]))


@pytest.mark.parametrize('jac,qfrc', [
    (np.array([[np.nan, 1.], [0., 1.]]), np.array([1., 2.])),
    (np.eye(2), np.array([np.inf, 1.])),
    (np.array([[1., 0.], [np.inf, 1.]]), np.array([1., 2.])),
    (np.eye(3), np.array([1., np.nan, 0.])),
])
def test_recover_wrench_nonfinite(jac, qfrc):
    """recover_wrench pre-screens (unlike np.linalg.lstsq): nonfinite
    input is 'nonfinite wrench input' before DGELSD ever runs."""
    _recover_oracle(jac, qfrc)


def test_recover_wrench_degenerate_and_shape_errors():
    _recover_oracle(np.zeros((3, 0)), np.zeros(0))
    for bad_jac in (np.zeros(4), np.zeros((2, 2, 2))):
        with pytest.raises(ValueError):
            bike_native.rider_recover_wrench(bad_jac, np.zeros(4))
    with pytest.raises(ValueError):
        bike_native.rider_recover_wrench(np.eye(3), np.zeros(4))


def test_recover_wrench_owns_its_output():
    rng = np.random.default_rng(747)
    jac = rng.normal(size=(3, 7))
    qfrc = jac.T @ rng.normal(size=3)
    first = bike_native.rider_recover_wrench(jac, qfrc)
    second = bike_native.rider_recover_wrench(jac, qfrc)
    assert not np.may_share_memory(first, second)
    assert not np.may_share_memory(first, np.asarray(jac))
    first[:] = -1.
    bits(second, recover_wrench(jac, qfrc))


@pytest.mark.parametrize('jacobian', ['sparse', 'dense'])
def test_relative_planar_jacobian_oracle(jacobian, tmp_path):
    """rider_relative_planar_jacobian == relative_planar_jacobian bitwise
    on a live arena; the point/body validation errors are identical."""
    model = _pair_model(_ALL_EQUALITIES, option=f'jacobian="{jacobian}"')
    data = mujoco.MjData(model)
    _drive(model, data)
    stepper = _settle(model, data, tmp_path)
    rider, bike = _attachment_ids(model)
    point = np.array([0., 0., 1.])
    for rotational in (False, True):
        expected = relative_planar_jacobian(model, data, rider, bike,
                                            point, rotational=rotational)
        actual = stepper.rider_relative_planar_jacobian(
            rider, bike, point, rotational)
        bits(actual, expected, f'rotational={rotational}')
    # A moving point reads the same engine jacobian. mj_jac reads cdof,
    # which mj_kinematics does NOT refresh — only mj_crb (inside
    # mj_forward) does; run forward on both sides like the Stepper does.
    data.qpos[:4] = [.03, .0, -.02, .965925826]
    mujoco.mj_forward(model, data)
    stepper.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart,
                      data.time)
    stepper.forward()
    point = np.array([.4, 0., .9])
    bits(stepper.rider_relative_planar_jacobian(rider, bike, point, True),
         relative_planar_jacobian(model, data, rider, bike, point,
                                  rotational=True))
    hand = model.body('hand').id
    bits(stepper.rider_relative_planar_jacobian(hand, bike, point, False),
         relative_planar_jacobian(model, data, hand, bike, point,
                                  rotational=False))
    # Validation: invalid world point / non-physical / equal bodies.
    for bad_point in (np.array([0., 0.]), np.array([0., 0., np.nan]),
                      np.array([0., 0., np.inf])):
        with pytest.raises(ValueError, match='invalid world point'):
            relative_planar_jacobian(model, data, rider, bike, bad_point,
                                     rotational=True)
        with pytest.raises(ValueError, match='invalid world point'):
            stepper.rider_relative_planar_jacobian(rider, bike, bad_point,
                                                   True)
    for bad_pair in [(0, bike), (rider, 0), (rider, rider),
                     (rider, model.nbody)]:
        with pytest.raises(ValueError, match='distinct physical bodies'):
            relative_planar_jacobian(model, data, *bad_pair, point,
                                     rotational=True)
        with pytest.raises(ValueError, match='distinct physical bodies'):
            stepper.rider_relative_planar_jacobian(*bad_pair, point, True)


@pytest.mark.parametrize('jacobian', ['sparse', 'dense'])
@pytest.mark.parametrize('eq_name,bodies,point,rotational,kind,pull', [
    ('w', ('rider', 'bike'), [0., 0., 1.], True, 'foot',
     [.1, 0., -.8]),
    ('w', ('rider', 'bike'), [0., 0., 1.], False, 'foot', None),
    # connect + nonrotational -> per-body compiled anchors, not the
    # shared point; the jacobians themselves prove which points ran.
    ('c', ('hand', 'bike'), [.2, 0., 1.1], False, 'grip',
     [-.2, 0., -1.1]),
])
def test_prepare_attachment_geometry_oracle(jacobian, eq_name, bodies,
                                            point, rotational, kind, pull,
                                            tmp_path):
    """Every AttachmentGeometry field is bitwise-identical, including the
    per-body CONNECT anchors (a shared-point evaluation would produce
    different bike jacobian columns)."""
    model = _pair_model(
        '<weld name="w" body1="rider" body2="bike"/>'
        '<connect name="c" body1="hand" body2="bike" anchor="0 .1 0"/>',
        option=f'jacobian="{jacobian}"')
    data = mujoco.MjData(model)
    _drive(model, data)
    stepper = _settle(model, data, tmp_path)
    body_a, body_b = (model.body(name).id for name in bodies)
    eq_id = int(model.equality(eq_name).id)
    point = np.asarray(point)
    normal = np.array([0., 0., 1.])
    pull_direction = None if pull is None else np.asarray(pull)
    expected = prepare_attachment_geometry(
        model, data, eq_id, body_a, body_b, point, normal, kind,
        rotational=rotational, half_patch_m=.05,
        pull_direction=pull_direction)
    actual = stepper.rider_prepare_attachment(
        eq_id, body_a, body_b, point, normal, kind, rotational, .05,
        pull_direction)
    _assert_geometry(actual, expected)
    # A body carrying no dofs is a hard failure on both sides.
    with pytest.raises(ValueError,
                       match='no degrees of freedom'):
        prepare_attachment_geometry(model, data, eq_id, 0, body_b, point,
                                    normal, kind, rotational=rotational)
    with pytest.raises(ValueError,
                       match='no degrees of freedom'):
        stepper.rider_prepare_attachment(eq_id, 0, body_b, point, normal,
                                         kind, rotational)


@pytest.mark.parametrize('eq_name,bodies,point,rotational,kind', [
    ('w', ('rider', 'bike'), [0., 0., 1.], True, 'foot'),
    ('w', ('rider', 'bike'), [0., 0., 1.], False, 'foot'),
    ('c', ('hand', 'bike'), [.2, 0., 1.1], False, 'grip'),
])
def test_attachment_raw_oracle(eq_name, bodies, point, rotational, kind,
                               tmp_path):
    """attachment_raw bitwise on the shared solved arena — contacts and a
    violated joint limit interleave unrelated rows through the arena."""
    model = _pair_model(
        '<weld name="w" body1="rider" body2="bike"/>'
        '<connect name="c" body1="hand" body2="bike" anchor="0 .1 0"/>')
    data = mujoco.MjData(model)
    _drive(model, data)
    stepper = _settle(model, data, tmp_path)
    assert np.any(data.efc_type[:data.nefc] != _EQUALITY)
    body_a, body_b = (model.body(name).id for name in bodies)
    eq_id = int(model.equality(eq_name).id)
    point = np.asarray(point)
    normal = np.array([0., 0., 1.])
    expected = attachment_raw(model, data, eq_id, body_a, body_b, point,
                              normal, kind, rotational=rotational,
                              half_patch_m=.025)
    actual = stepper.rider_attachment_raw(eq_id, body_a, body_b, point,
                                          normal, kind, rotational, .025)
    _assert_raw(actual, expected)


@pytest.mark.parametrize('eq_name,bodies,point,rotational,kind', [
    ('w', ('rider', 'bike'), [0., 0., 1.], True, 'foot'),
    ('w', ('rider', 'bike'), [0., 0., 1.], False, 'foot'),
    ('c', ('hand', 'bike'), [.2, 0., 1.1], False, 'grip'),
])
def test_attachment_raw_from_geometry_oracle(eq_name, bodies, point,
                                             rotational, kind, tmp_path):
    """The prepared path and the eager path agree bitwise on the same
    solved state — and a dataclass/dict of numpy fields feeds the native
    fast path identically."""
    model = _pair_model(
        '<weld name="w" body1="rider" body2="bike"/>'
        # A planar anchor offset: the compiled eq_data still differs per
        # body (the soft-connect anchor path is exercised), while a
        # y-offset would torque the assembly out of plane and trip the
        # validate_wrench planarity check.
        '<connect name="c" body1="hand" body2="bike" anchor="0 0 .1"/>')
    data = mujoco.MjData(model)
    _drive(model, data)
    # Keep every load planar: validate_wrench=True runs the planarity
    # check, and the _drive hand y-load would put the solved wrench out
    # of plane (dedicated coverage lives in test_attachment_out_of_plane).
    hand_dof = int(model.body_dofadr[model.body('hand').id])
    data.qfrc_applied[hand_dof + 1] = 0.
    stepper = _settle(model, data, tmp_path)
    body_a, body_b = (model.body(name).id for name in bodies)
    eq_id = int(model.equality(eq_name).id)
    point = np.asarray(point)
    normal = np.array([0., 0., 1.])
    py_geometry = prepare_attachment_geometry(
        model, data, eq_id, body_a, body_b, point, normal, kind,
        rotational=rotational, half_patch_m=.05)
    nt_geometry = stepper.rider_prepare_attachment(
        eq_id, body_a, body_b, point, normal, kind, rotational, .05)
    # Native geometry dict in, oracle dataclass in — both raws identical.
    expected = attachment_raw_from_geometry(model, data, py_geometry)
    actual = stepper.rider_attachment_raw_from_geometry(nt_geometry)
    _assert_raw(actual, expected)
    # vars()-style dicts of the python dataclass feed identically.
    _assert_raw(
        stepper.rider_attachment_raw_from_geometry(
            dataclasses.asdict(py_geometry)), expected)
    # validate_wrench=False defers every spatial check (here it passes
    # anyway — the deferral itself is exercised against the error cases
    # in test_attachment_unobservable_and_validation_deferral).
    _assert_raw(stepper.rider_attachment_raw_from_geometry(
        nt_geometry, validate_wrench=False), expected)
    # prepared == eager on an unchanged state, bitwise.
    _assert_raw(stepper.rider_attachment_raw(
        eq_id, body_a, body_b, point, normal, kind, rotational, .05),
        attachment_raw(model, data, eq_id, body_a, body_b, point, normal,
                       kind, rotational=rotational, half_patch_m=.05))


def test_attachment_sample_oracle(tmp_path):
    """attachment_sample bitwise on weld + connect equalities — normal,
    tangent, moment, gap and pull all recovered from real solves."""
    model = _pair_model(
        '<weld name="w" body1="rider" body2="bike"/>'
        '<connect name="c" body1="hand" body2="bike" anchor="0 0 0"/>')
    data = mujoco.MjData(model)
    _drive(model, data)
    hand_dof = int(model.body_dofadr[model.body('hand').id])
    data.qfrc_applied[hand_dof + 1] = 0.  # keep the grip load planar
    data.qfrc_applied[hand_dof + 2] = -7.
    stepper = _settle(model, data, tmp_path)
    rider, bike = _attachment_ids(model)
    hand = model.body('hand').id
    point = np.array([0., 0., 1.])
    grip = np.array([.2, 0., 1.1])
    normal = np.array([0., 0., 1.])
    cases = [
        ('w', rider, bike, point, True, 'foot', None),
        ('w', rider, bike, point, True, 'foot',
         np.array([.1, 0., -.8])),
        ('w', rider, bike, point, False, 'saddle', None),
        ('c', hand, bike, grip, False, 'grip',
         np.array([-.2, 0., -1.1])),
    ]
    for eq_name, body_a, body_b, at, rotational, kind, pull in cases:
        eq_id = int(model.equality(eq_name).id)
        expected = attachment_sample(
            model, data, eq_id, body_a, body_b, at, normal, kind,
            rotational=rotational, half_patch_m=.05,
            pull_direction=pull)
        actual = stepper.rider_attachment_sample(
            eq_id, body_a, body_b, at, normal, kind, rotational, .05,
            pull)
        _assert_sample(actual, expected, f'{eq_name} rot={rotational}')


def test_decompose_wrench_oracle():
    """decompose_wrench on synthetic wrenches — normal/tangent/moment/
    pull projections and every validation rejection, bitwise."""
    rng = np.random.default_rng(555)
    for i in range(200):
        wrench = rng.normal(size=6) * 10. ** rng.integers(-6, 6)
        normal = rng.normal(size=3)
        kind = str(rng.choice(['foot', 'saddle', 'grip']))
        rotational = bool(i % 2)
        gap = abs(rng.normal())
        half = abs(rng.normal())
        pull = rng.normal(size=3) if i % 3 else None
        expected = decompose_wrench(wrench, normal, kind,
                                    rotational=rotational, gap_m=gap,
                                    half_patch_m=half,
                                    pull_direction=pull)
        actual = bike_native.rider_decompose_wrench(
            wrench, normal, kind, rotational, half, gap, pull)
        _assert_sample(actual, expected, f'case {i}')
    # Pull projection gates on the sign, exactly like the oracle. (A
    # y-only pull has no planar component — that rejection is covered
    # with the other bad_pull cases below.)
    for pull, w in ((np.array([1., 0., 0.]), np.array([3., 0., -2.])),
                    (np.array([-1., 0., 0.]), np.array([3., 0., -2.]))):
        wrench = np.concatenate([w, np.zeros(3)])
        expected = decompose_wrench(wrench, np.array([0., 0., 1.]), 'grip',
                                    rotational=False,
                                    pull_direction=pull)
        actual = bike_native.rider_decompose_wrench(
            wrench, np.array([0., 0., 1.]), 'grip', False, 0., 0., pull)
        _assert_sample(actual, expected)
    # Error paths — same ValueError, same message.
    good = np.array([1., 0., -2., 0., .5, 0.])
    for bad_normal in (np.array([1., 0.]), np.array([0., 1., 0.]),
                       np.array([np.nan, 0., 1.]),
                       np.array([0., 0., 0.])):
        with pytest.raises(ValueError) as expected_error:
            decompose_wrench(good, bad_normal, 'foot', rotational=True)
        with pytest.raises(ValueError) as raised:
            bike_native.rider_decompose_wrench(good, bad_normal, 'foot',
                                               True)
        assert str(raised.value) == str(expected_error.value)
    for bad_pull in (np.array([1., 0.]), np.array([0., 1., 0.]),
                     np.array([np.inf, 0., 1.])):
        with pytest.raises(ValueError) as expected_error:
            decompose_wrench(good, np.array([0., 0., 1.]), 'grip',
                             rotational=False, pull_direction=bad_pull)
        with pytest.raises(ValueError) as raised:
            bike_native.rider_decompose_wrench(
                good, np.array([0., 0., 1.]), 'grip', False, 0., 0.,
                bad_pull)
        assert str(raised.value) == str(expected_error.value)


def test_attachment_missing_equality_is_no_measurement(tmp_path):
    """An equality with no solved rows is zero force + zero gap — the
    'no current measurement' contract, never an invented reading.
    A rotational sample reads no eq_type at all; a nonrotational one
    indexes eq_type[eq_id] like the oracle (IndexError for a bogus id)."""
    model = _pair_model('<weld name="w" body1="rider" body2="bike"/>')
    data = mujoco.MjData(model)
    _drive(model, data)
    stepper = _settle(model, data, tmp_path)
    rider, bike = _attachment_ids(model)
    point = np.array([0., 0., 1.])
    normal = np.array([0., 0., 1.])
    # rotational -> the eq_id is only used for row membership.
    expected = attachment_sample(model, data, 999, rider, bike, point,
                                 normal, 'foot', rotational=True,
                                 half_patch_m=.05)
    actual = stepper.rider_attachment_sample(999, rider, bike, point,
                                             normal, 'foot', True, .05)
    _assert_sample(actual, expected)
    assert actual['gap_m'] == 0. and actual['normal_n'] == 0.
    # nonrotational -> model.eq_type[eq_id] indexes like numpy.
    for fn in (attachment_sample, attachment_raw):
        with pytest.raises(IndexError):
            fn(model, data, 999, rider, bike, point, normal, 'foot',
               rotational=False)
    with pytest.raises(IndexError):
        stepper.rider_attachment_sample(999, rider, bike, point, normal,
                                        'foot', False)
    with pytest.raises(IndexError):
        stepper.rider_attachment_raw(999, rider, bike, point, normal,
                                     'foot', False)
    # Negative eq_id wraps inside eq_type like numpy indexing: eq[-1]
    # selects the weld's type (shared point) but never matches rows.
    expected = attachment_raw(model, data, -1, rider, bike, point, normal,
                              'foot', rotational=False)
    actual = stepper.rider_attachment_raw(-1, rider, bike, point, normal,
                                          'foot', False)
    _assert_raw(actual, expected)
    assert actual['gap_m'] == 0.


def test_attachment_shared_support_error(tmp_path):
    """Overlapping dof columns — 'attachment bodies share kinematic
    support' on every path that joins two supports."""
    model = _pair_model('<weld name="w" body1="rider" body2="bike"/>')
    data = mujoco.MjData(model)
    _drive(model, data)
    stepper = _settle(model, data, tmp_path)
    rider, _ = _attachment_ids(model)
    point = np.array([0., 0., 1.])
    normal = np.array([0., 0., 1.])
    eq_id = int(model.equality('w').id)
    for py_fn, nt_fn in (
            (prepare_attachment_geometry,
             lambda *a: stepper.rider_prepare_attachment(*a)),
            (attachment_raw,
             lambda *a: stepper.rider_attachment_raw(*a)),
            (attachment_sample,
             lambda *a: stepper.rider_attachment_sample(*a))):
        with pytest.raises(ValueError,
                           match='share kinematic support'):
            py_fn(model, data, eq_id, rider, rider, point, normal, 'foot',
                  rotational=True)
        with pytest.raises(ValueError,
                           match='share kinematic support'):
            nt_fn(eq_id, rider, rider, point, normal, 'foot', True)


def test_attachment_out_of_plane_error(tmp_path):
    """A y-force on the rider is a real out-of-plane wrench: 'attachment
    wrench leaves the planar model' — and validate_wrench=False returns
    the raw measurement instead of inventing a planar one."""
    model = _pair_model('<weld name="w" body1="rider" body2="bike"/>')
    data = mujoco.MjData(model)
    _drive(model, data)
    rider_dof = int(model.body_dofadr[model.body('rider').id])
    data.qfrc_applied[rider_dof + 1] = 40.
    stepper = _settle(model, data, tmp_path)
    rider, bike = _attachment_ids(model)
    point = np.array([0., 0., 1.])
    normal = np.array([0., 0., 1.])
    eq_id = int(model.equality('w').id)
    for rotational in (False, True):
        with pytest.raises(ValueError, match='planar model'):
            attachment_sample(model, data, eq_id, rider, bike, point,
                              normal, 'foot', rotational=rotational)
        with pytest.raises(ValueError, match='planar model'):
            stepper.rider_attachment_sample(eq_id, rider, bike, point,
                                            normal, 'foot', rotational)
    geometry = stepper.rider_prepare_attachment(
        eq_id, rider, bike, point, normal, 'foot', True)
    py_geometry = prepare_attachment_geometry(
        model, data, eq_id, rider, bike, point, normal, 'foot',
        rotational=True)
    with pytest.raises(ValueError, match='planar model'):
        attachment_raw_from_geometry(model, data, py_geometry)
    with pytest.raises(ValueError, match='planar model'):
        stepper.rider_attachment_raw_from_geometry(geometry)
    deferred = stepper.rider_attachment_raw_from_geometry(
        geometry, validate_wrench=False)
    _assert_raw(deferred, attachment_raw_from_geometry(
        model, data, py_geometry, validate_wrench=False))


def _yslide_model():
    """A 'rider' that can only translate in y: its in-plane force is
    genuinely unobservable (jac x/z rows are identically zero)."""
    return mujoco.MjModel.from_xml_string('''
<mujoco><option gravity="0 0 0" timestep="0.0005"/>
<worldbody>
  <body name="bike" pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1e3" diaginertia="1e3 1e3 1e3"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="rider" pos="0 .3 1">
    <joint name="ys" type="slide" axis="0 1 0"/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
</worldbody>
<equality><weld name="w" body1="rider" body2="bike"/></equality>
</mujoco>''')


def test_attachment_unobservable_and_validation_deferral(tmp_path):
    """The y-slide body keeps a dof but no x/z observability: prepare
    stores observable=False, the raw path stores it too, and only the
    validating readers raise 'in-plane attachment force is not
    observable'."""
    model = _yslide_model()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    data.qfrc_applied[int(model.joint('ys').dofadr[0])] = -12.
    stepper = _settle(model, data, tmp_path)
    rider, bike = _attachment_ids(model)
    eq_id = int(model.equality('w').id)
    point = np.array([0., 0., 1.])
    normal = np.array([0., 0., 1.])
    py_geometry = prepare_attachment_geometry(
        model, data, eq_id, rider, bike, point, normal, 'saddle',
        rotational=True)
    assert py_geometry.observable is False
    nt_geometry = stepper.rider_prepare_attachment(
        eq_id, rider, bike, point, normal, 'saddle', True)
    assert nt_geometry['observable'] is False
    _assert_geometry(nt_geometry, py_geometry)
    # Eager raw: observable=False is stored, never silently validated.
    _assert_raw(stepper.rider_attachment_raw(
        eq_id, rider, bike, point, normal, 'saddle', True),
        attachment_raw(model, data, eq_id, rider, bike, point, normal,
                       'saddle', rotational=True))
    # Deferred validation keeps the raw; eager validation raises.
    _assert_raw(stepper.rider_attachment_raw_from_geometry(
        nt_geometry, validate_wrench=False),
        attachment_raw_from_geometry(model, data, py_geometry,
                                     validate_wrench=False))
    with pytest.raises(ValueError, match='not observable'):
        attachment_raw_from_geometry(model, data, py_geometry)
    with pytest.raises(ValueError, match='not observable'):
        stepper.rider_attachment_raw_from_geometry(nt_geometry)
    # The sample path's body_wrench raises the same error.
    with pytest.raises(ValueError, match='not observable'):
        attachment_sample(model, data, eq_id, rider, bike, point, normal,
                          'saddle', rotational=True)
    with pytest.raises(ValueError, match='not observable'):
        stepper.rider_attachment_sample(eq_id, rider, bike, point, normal,
                                        'saddle', True)


def test_attachment_newton_third_law_error(tmp_path):
    """A joint equality applies +-tau to two hinge dofs — recovering each
    side's wrench at the same world point does NOT cancel: 'attachment
    wrenches fail Newton third law', bitwise from the solved torque."""
    model = _pair_model(
        '<joint name="j" joint1="ja" joint2="jb"/>')
    data = mujoco.MjData(model)
    _drive(model, data)
    stepper = _settle(model, data, tmp_path)
    ja = model.body('ja_b').id
    jb = model.body('jb_b').id
    eq_id = int(model.equality('j').id)
    point = np.array([0., 0., .8])
    normal = np.array([0., 0., 1.])
    with pytest.raises(ValueError, match='Newton third law'):
        attachment_sample(model, data, eq_id, ja, jb, point, normal,
                          'x', rotational=False)
    with pytest.raises(ValueError, match='Newton third law'):
        stepper.rider_attachment_sample(eq_id, ja, jb, point, normal,
                                        'x', False)
    # The same failing solve keeps its raw measurement under deferral.
    geometry = stepper.rider_prepare_attachment(eq_id, ja, jb, point,
                                                normal, 'x', False)
    py_geometry = prepare_attachment_geometry(model, data, eq_id, ja, jb,
                                              point, normal, 'x',
                                              rotational=False)
    with pytest.raises(ValueError, match='Newton third law'):
        attachment_raw_from_geometry(model, data, py_geometry)
    with pytest.raises(ValueError, match='Newton third law'):
        stepper.rider_attachment_raw_from_geometry(geometry)


def test_attachment_measurement_dicts_own_storage(tmp_path):
    """Every returned array is capsule-owned fresh storage: mutating one
    snapshot disturbs no other call, and prepared geometry keeps its
    interval-start jacobians after more steps land."""
    model = _pair_model('<weld name="w" body1="rider" body2="bike"/>')
    data = mujoco.MjData(model)
    _drive(model, data)
    stepper = _settle(model, data, tmp_path)
    rider, bike = _attachment_ids(model)
    eq_id = int(model.equality('w').id)
    point = np.array([0., 0., 1.])
    normal = np.array([0., 0., 1.])
    first = stepper.rider_prepare_attachment(eq_id, rider, bike, point,
                                             normal, 'foot', True)
    second = stepper.rider_prepare_attachment(eq_id, rider, bike, point,
                                              normal, 'foot', True)
    assert not np.may_share_memory(first['rider_jac'],
                                   second['rider_jac'])
    first['rider_jac'][:] = -7.
    _assert_geometry(second, prepare_attachment_geometry(
        model, data, eq_id, rider, bike, point, normal, 'foot',
        rotational=True))
    # A prepared snapshot survives stepping: the jacobians still describe
    # the pose they were captured at, while raw_from_geometry reads the
    # NEW multipliers of the current solve.
    snapshot = stepper.rider_prepare_attachment(eq_id, rider, bike, point,
                                                normal, 'foot', True)
    py_snapshot = prepare_attachment_geometry(model, data, eq_id, rider,
                                              bike, point, normal,
                                              'foot', rotational=True)
    mujoco.mj_step(model, data)
    stepper.step()
    _assert_raw(stepper.rider_attachment_raw_from_geometry(snapshot),
                attachment_raw_from_geometry(model, data, py_snapshot))
    # The frozen jacobian is not the post-step one.
    eager = attachment_raw(model, data, eq_id, rider, bike, point,
                           normal, 'foot', rotational=True)
    assert not np.array_equal(snapshot['rider_jac'], eager.rider_jac)


def test_attachment_interval_start_geometry(tmp_path):
    """The interval contract: capture at q0, step once, combine frozen
    jacobians with the just-solved multipliers — bitwise against the
    oracle's prepared path."""
    model = _pair_model('<weld name="w" body1="rider" body2="bike"/>')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    stepper = _native_mirror(model, data, tmp_path)
    # set_state copies no derived buffers: the owned pair needs its own
    # forward to reach the same post-forward cdof the oracle just read.
    stepper.forward()
    rider, bike = _attachment_ids(model)
    eq_id = int(model.equality('w').id)
    point = np.array([0., 0., 1.])
    normal = np.array([0., 0., 1.])
    nt_geometry = stepper.rider_prepare_attachment(
        eq_id, rider, bike, point, normal, 'foot', True, .05)
    py_geometry = prepare_attachment_geometry(
        model, data, eq_id, rider, bike, point, normal, 'foot',
        rotational=True, half_patch_m=.05)
    _assert_geometry(nt_geometry, py_geometry)
    rider_dof = int(model.body_dofadr[rider])
    data.qfrc_applied[rider_dof] = 35.
    data.qfrc_applied[rider_dof + 2] = 80.
    data.qfrc_applied[rider_dof + 4] = 7.
    stepper.set_inputs(data.ctrl, data.qfrc_applied)
    for _ in range(8):
        mujoco.mj_step(model, data)
        stepper.step()
    _assert_raw(stepper.rider_attachment_raw_from_geometry(nt_geometry),
                attachment_raw_from_geometry(model, data, py_geometry))
    # Sample path on the same solve.
    _assert_sample(
        stepper.rider_attachment_sample(eq_id, rider, bike, point,
                                        normal, 'foot', True, .05),
        attachment_sample(model, data, eq_id, rider, bike, point, normal,
                          'foot', rotational=True, half_patch_m=.05))
