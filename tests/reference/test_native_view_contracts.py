"""Stepper array-owner contracts (F4): fixed-buffer views retain their
Stepper via rv_policy::reference_internal, while efc_force is an owned
snapshot because its arena offset moves with the constraint set. Drive
diagnostics expose storage generations and capacities — never raw
addresses — and _drive_core refuses kinds outside the closed set or any
call on a Stepper whose drive writer is installed."""
import gc
import sys

import mujoco
import numpy as np
import pytest

from native_loader import load_native
bike_native = load_native()

from _bits import assert_bitwise_equal
from native_safety_helpers import drive_pair, freeze

_ONE_SLIDE_XML = ('<mujoco><worldbody><body><joint name="slide" type="slide"/>'
                  '<geom size=".1" mass="1"/></body></worldbody></mujoco>')

_CONTACT_XML = """<mujoco><option timestep="0.002" gravity="0 0 -9.81"/>
<worldbody><geom name="floor" type="plane" size="5 5 .1"/>
<body name="a" pos="0 0 .15"><freejoint/><geom type="sphere" size=".1" mass="1" condim="3"/></body>
<body name="b" pos="0.05 0 .35"><freejoint/><geom type="sphere" size=".1" mass="1" condim="3"/></body>
</worldbody></mujoco>"""

# Zero-copy read-only windows onto mjData's fixed buffers; efc_force is
# the owned exception — the constraint arena relocates it, so the
# binding returns a detached writeable copy instead of a view.
VIEWS = ('qpos', 'qvel', 'qacc', 'qfrc_constraint', 'ctrl',
         'qfrc_applied', 'actuator_force')


@pytest.fixture
def saved_model(tmp_path):
    model = mujoco.MjModel.from_xml_string(_ONE_SLIDE_XML)
    path = tmp_path / 'model.mjb'
    mujoco.mj_saveModel(model, str(path), None)
    return path


def _contact_pair(tmp_path):
    model = mujoco.MjModel.from_xml_string(_CONTACT_XML)
    path = tmp_path / 'contacts.mjb'
    mujoco.mj_saveModel(model, str(path), None)
    return model, bike_native.Stepper(str(path))


def test_view_props_retain_temporary_stepper_and_stay_read_only(saved_model):
    # One slide joint, no actuators: nq=nv=1, nu=0.
    expected = {'qpos': 1, 'qvel': 1, 'qacc': 1, 'qfrc_constraint': 1,
                'ctrl': 0, 'qfrc_applied': 1, 'actuator_force': 0}
    for name in VIEWS:
        # The temporary Stepper dies at the end of the expression; the
        # view must keep it alive.
        array = getattr(bike_native.Stepper(str(saved_model)), name)
        gc.collect()
        assert np.isfinite(array).all(), name
        assert not array.flags.writeable, name
        assert array.shape == (expected[name],), name
    # Owned snapshots carry no engine lifetime either — same exercise.
    efc = bike_native.Stepper(str(saved_model)).efc_force
    gc.collect()
    assert efc.shape == (0,)
    assert efc.flags.writeable


def test_view_retains_named_stepper_after_deletion(saved_model):
    stepper = bike_native.Stepper(str(saved_model))
    before = sys.getrefcount(stepper)
    view = stepper.qpos
    # reference_internal installs the Stepper in the array's base chain.
    assert sys.getrefcount(stepper) == before + 1
    del stepper
    gc.collect()
    assert np.isfinite(view).all()


def test_views_are_live_windows_matching_engine_state(saved_model):
    stepper = bike_native.Stepper(str(saved_model))
    qpos_view = stepper.qpos
    qvel_view = stepper.qvel
    qacc_view = stepper.qacc
    stepper.set_state(np.array([0.5]), np.array([-0.25]), np.empty(0),
                      np.zeros(1), 0.0)
    stepper.forward()
    model = mujoco.MjModel.from_binary_path(str(saved_model))
    reference = mujoco.MjData(model)
    reference.qpos[:] = 0.5
    reference.qvel[:] = -0.25
    mujoco.mj_forward(model, reference)
    # The arrays fetched before set_state track the same buffers.
    assert_bitwise_equal(qpos_view, reference.qpos)
    assert_bitwise_equal(qvel_view, reference.qvel)
    assert_bitwise_equal(qacc_view, reference.qacc)


def test_efc_force_snapshot_survives_arena_growth(tmp_path):
    model, stepper = _contact_pair(tmp_path)
    assert stepper.efc_force.shape == (0,)
    assert stepper.efc_force.flags.writeable
    for _ in range(300):
        stepper.step()
        if stepper.efc_force.size:
            break
    snapshot = stepper.efc_force
    assert snapshot.size > 0, 'contacts never populated efc_force'
    frozen = snapshot.tobytes()
    # More steps relocate/resize the constraint arena; each fresh read is
    # a new snapshot of the current arena while the retained one is a
    # detached copy that must not follow or corrupt.
    for _ in range(200):
        stepper.step()
        current = stepper.efc_force
        if current.shape != snapshot.shape or current.tobytes() != frozen:
            break
    else:
        pytest.fail('efc_force never changed — arena growth not exercised')
    assert snapshot.tobytes() == frozen
    assert np.isfinite(snapshot).all()
    assert np.isfinite(stepper.efc_force).all()


def test_prepared_storage_reports_generations_not_addresses(tmp_path):
    m, d, p, n = drive_pair(tmp_path, kind='geometric_ideal_mid_drive')
    baseline = n._drive_prepared_storage()
    # Exactly four diagnostic counters — no address keys cross the bridge.
    assert set(baseline) == {'jacobian_generation', 'qpos_generation',
                           'jacobian_capacity', 'qpos_capacity'}
    assert all(isinstance(value, int) for value in baseline.values())
    assert baseline['jacobian_generation'] >= 1
    assert baseline['qpos_generation'] >= 1
    assert baseline['jacobian_capacity'] >= m.nv
    assert baseline['qpos_capacity'] >= m.nv
    # Generations count data() identity changes: repeated prepares reuse
    # the same storage, so the reported identity is stable.
    for _ in range(6):
        assert n._drive_prepared_storage() == baseline
    n.drive_reset()
    assert n._drive_prepared_storage() == baseline


def test_prepared_storage_none_without_geometric_transmission(tmp_path):
    m, d, p, n = drive_pair(tmp_path, kind='ideal_mid_drive')
    assert n._drive_prepared_storage() is None


def _bare_drive_stepper(tmp_path, kind):
    from test_native_drivetrain import model_xml
    model = mujoco.MjModel.from_xml_string(model_xml(kind))
    path = tmp_path / f'{kind}.mjb'
    mujoco.mj_saveModel(model, str(path), None)
    return bike_native.Stepper(str(path))


def test_drive_core_rejects_unknown_kind_before_mutation(tmp_path):
    for kind in ('elastic_chain', 'not_a_model'):
        stepper = _bare_drive_stepper(tmp_path, 'ideal_mid_drive')
        with pytest.raises(ValueError):
            stepper._drive_core(kind, 'reset')
    stepper = _bare_drive_stepper(tmp_path, 'ideal_mid_drive')
    with pytest.raises(ValueError):
        stepper._drive_core('ideal_mid_drive', 'not_an_operation')


def test_drive_core_kind_agnostic_on_bare_stepper(tmp_path):
    for kind in ('ideal_mid_drive', 'geometric_ideal_mid_drive'):
        stepper = _bare_drive_stepper(tmp_path, kind)
        result = stepper._drive_core(kind, 'ratio', 34. / 28.)
        assert result['ratio'] == 34. / 28.
        assert set(result) >= {'boundary', 'ratio', 'range', 'coefficients'}
        result = stepper._drive_core(kind, 'reset')
        assert np.isfinite(np.asarray(result['coefficients'])).all()


def test_drive_core_rejected_with_installed_drive_writer(tmp_path):
    m, d, p, n = drive_pair(tmp_path)
    state = n.drive_state()
    with pytest.raises(RuntimeError):
        n._drive_core('ideal_mid_drive', 'reset')
    with pytest.raises(RuntimeError):
        n._drive_core('geometric_ideal_mid_drive', 'ratio', 3.0)
    # Rejection happens before owner.mutate(): the writer state is
    # bit-identical to before the refused calls.
    assert freeze(n.drive_state()) == freeze(state)
