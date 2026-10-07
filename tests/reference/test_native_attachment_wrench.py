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
import math

import mujoco
import numpy as np
import pytest

from _bits import assert_bitwise_equal
from native_loader import load_native

from bike_sim.sim.ride.attachment_wrench import equality_qfrc
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
