"""Native drivetrain transactions: a rejected staged update publishes nothing.

Every mutating drivetrain path — shift, prepare, reset, restore — stages the
candidate on owned scratch model/data and only commits fully validated state.
These tests pin the observable contract over the FFI: frozen ``drive_state``
snapshots, engine qpos/qvel, and the prepared-storage generation counters must
be bitwise-identical after any rejection, and a repeated attempt must reject
identically. Successful commits must land atomically without reallocating the
prepared buffers (generation and capacity stable).
"""
import copy

import mujoco
import numpy as np
import pytest

from _bits import assert_bitwise_equal
from bike_sim.physics.physical_config import ShiftingConfig
from bike_sim.sim.ride.control import RideControl
from native_loader import load_native
from native_safety_helpers import drive_pair, freeze
from test_native_drivetrain import python_state

bike_native = load_native()

GEOMETRIC = 'geometric_ideal_mid_drive'


def _invalid_cassette():
    """A parsed cassette whose next sprocket makes tangent geometry invalid."""
    return ShiftingConfig(enabled=True, cassette=(24, 1000),
                          cadence_smoothing_tau_s=0.,
                          target_cadence_min_rpm=100., target_cadence_max_rpm=1000.)


def _valid_cassette():
    return ShiftingConfig(enabled=True, cassette=(24, 28),
                          cadence_smoothing_tau_s=0.,
                          target_cadence_min_rpm=100., target_cadence_max_rpm=1000.)


def _spin(model, data, native, crank=4., wheel=1.):
    data.qvel[model.joint('crank_spin').dofadr[0]] = crank
    data.qvel[model.joint('rear_wheel_spin').dofadr[0]] = wheel
    native.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart, data.time)
    native.forward()


def _frozen(native):
    """Everything a rejected transaction must preserve, bit-exact.

    ``drive_state`` carries the policy snapshots, diagnostics channels,
    timestamps, pending actuation, and the transmission coefficient/range
    mirrors of the live model rows; ``ctrl`` is the actuator surface.
    """
    return (freeze(native.drive_state()), freeze(native.qpos), freeze(native.qvel),
            freeze(native.ctrl), freeze(native._drive_prepared_storage()))


def _python_frozen(drive, model):
    return freeze(python_state(drive, model))


def test_failed_geometric_shift_is_atomic(tmp_path):
    model, data, _, native = drive_pair(tmp_path, kind=GEOMETRIC,
                                        shifting=_invalid_cassette())
    _spin(model, data, native)
    before = _frozen(native)
    with pytest.raises(ValueError) as first:
        native.drive_prepare_pedaling({}, .1)
    assert _frozen(native) == before
    # A repeated attempt meets the same candidate, the same rejection, and
    # still publishes nothing — the shifter roll-back keeps the retry legal.
    with pytest.raises(ValueError) as second:
        native.drive_prepare_pedaling({}, .1)
    assert _frozen(native) == before
    assert str(first.value) == str(second.value)


@pytest.mark.parametrize('kind', ['ideal_mid_drive', GEOMETRIC])
def test_valid_shift_commits_atomically(tmp_path, kind):
    model, data, _, native = drive_pair(tmp_path, kind=kind,
                                        shifting=_valid_cassette())
    _spin(model, data, native)
    storage = native._drive_prepared_storage()
    result = native.drive_prepare_pedaling({}, .1)
    state = native.drive_state()
    # Cadence ~38 rpm is below the 100 rpm floor: downshift 24 -> 28 teeth.
    assert state['ideal_hub']['ratio'] == 34. / 28.
    # Geometric hubs re-derive their sprocket; ideal hubs keep the configured
    # rear_teeth — only the wrap coefficient carries the shifted ratio.
    assert state['ideal_hub']['rear_teeth'] == (28 if kind == GEOMETRIC else 24)
    assert state['ideal_hub']['shift_pending'] == (kind == GEOMETRIC)
    assert state['policies']['shifting']['rear_teeth'] == 28
    assert state['policies']['shifting']['shift_count'] == 1
    assert state['policies']['shifting']['direction'] == 'down'
    assert state['shift_time_s'] == data.time
    if kind == GEOMETRIC:
        # Successful commits copy into persistent prepared storage: vector
        # identity (generation) and capacity are stable across the publish.
        after = native._drive_prepared_storage()
        assert after['jacobian_generation'] == storage['jacobian_generation']
        assert after['qpos_generation'] == storage['qpos_generation']
        assert after['jacobian_capacity'] == storage['jacobian_capacity']
        assert after['qpos_capacity'] == storage['qpos_capacity']
        assert state['ideal_hub']['prepared'] is not None
    # The returned pedaling state is a plain snapshot of the same transaction.
    assert np.isfinite(result['effort_nm'])


@pytest.mark.parametrize('kind', ['ideal_mid_drive', GEOMETRIC])
def test_rejected_then_reset_prepare_and_restore(tmp_path, kind):
    model, data, _, native = drive_pair(tmp_path, kind=kind,
                                        shifting=_invalid_cassette())
    _spin(model, data, native)
    if kind == GEOMETRIC:
        with pytest.raises(ValueError):
            native.drive_prepare_pedaling({}, .1)
    # Reset after the rejection re-stages from live state: the hub returns to
    # the configured gear, and the snapshot round-trips through restore.
    native.drive_reset()
    state = native.drive_state()
    assert state['ideal_hub']['ratio'] == 34. / 24.
    assert state['policies']['shifting']['shift_count'] == 0
    native.set_drive_state(state)
    assert freeze(native.drive_state()) == freeze(state)
    # A non-shifting prepare commits and only touches prepared/boundary state.
    data.qvel[model.joint('crank_spin').dofadr[0]] = 12.  # ~115 rpm: no shift
    native.set_state(data.qpos, data.qvel, data.act, data.qacc_warmstart,
                     data.time + .1)
    native.forward()
    native.drive_prepare_pedaling({}, .1)
    assert native.drive_state()['ideal_hub']['ratio'] == 34. / 24.


def test_invalid_restore_is_atomic(tmp_path):
    model, data, _, native = drive_pair(tmp_path, kind=GEOMETRIC)
    _spin(model, data, native)
    native.drive_prepare_pedaling({}, .1)
    before = _frozen(native)
    snapshot = native.drive_state()
    snapshot['ideal_hub']['ratio'] = 9.9  # inconsistent with the gearing row
    with pytest.raises(ValueError):
        native.set_drive_state(snapshot)
    assert _frozen(native) == before
    snapshot = native.drive_state()
    snapshot['ideal_hub']['boundary'] = float('nan')
    with pytest.raises(ValueError):
        native.set_drive_state(snapshot)
    assert _frozen(native) == before


def test_rejected_timestamp_advance_is_atomic(tmp_path):
    model, data, _, native = drive_pair(tmp_path, kind=GEOMETRIC)
    _spin(model, data, native)
    native.drive_components({}, .002, 2.)  # publishes last_time_s
    before = _frozen(native)
    with pytest.raises(ValueError, match='once per timestamp'):
        native.drive_components({}, .002, 2.)
    assert _frozen(native) == before


@pytest.mark.parametrize('topology', ['clutch', 'rotor'])
def test_rejected_shift_preserves_auxiliary_transmissions(tmp_path, topology):
    model, data, _, native = drive_pair(tmp_path, kind=GEOMETRIC, topology=topology,
                                        shifting=_invalid_cassette())
    _spin(model, data, native)
    before = _frozen(native)
    with pytest.raises(ValueError):
        native.drive_prepare_pedaling({}, .1)
    assert _frozen(native) == before
    state = native.drive_state()
    auxiliary = state['clutch'] if topology == 'clutch' else state['freewheel']
    assert auxiliary['ratio'] == 1.


def test_probe_controls_do_not_publish(tmp_path):
    model, data, _, native = drive_pair(tmp_path, kind=GEOMETRIC,
                                        shifting=_invalid_cassette())
    _spin(model, data, native)
    with pytest.raises(ValueError):
        native.drive_prepare_pedaling({}, .1)
    before = native.drive_state()
    # Probe evaluation (advance=False) must not touch ratchets or live policy
    # state even right after a rejected transaction.
    native.drive_components({}, .002, 2., active=True, advance=False)
    after = native.drive_state()
    after['probe_last'] = before['probe_last']  # diagnostics channel only
    assert freeze(after) == freeze(before)


@pytest.mark.parametrize('kind', ['ideal_mid_drive', GEOMETRIC])
def test_rejected_prepare_keeps_policy_state(tmp_path, kind):
    """A ceiling rejection discards the candidate: pedaling, shifter EMAs,
    shift clock and staged model updates all stay at their pre-call values —
    and a valid retry equals one fresh call, never an accumulated retry."""
    model, data, _, native = drive_pair(tmp_path, kind=kind)
    _spin(model, data, native)
    before = _frozen(native)
    for _ in range(3):
        with pytest.raises(ValueError):
            native.drive_prepare_pedaling({}, .01, effort_ceiling_nm=-1.)
        assert _frozen(native) == before
    # A valid call after the rejections must land exactly the state a single
    # call on an untouched fixture lands — no effort accumulated in retries.
    fresh_dir = tmp_path / 'fresh'
    fresh_dir.mkdir()
    _, _, _, fresh = drive_pair(fresh_dir, kind=kind)
    _spin(model, data, fresh)
    result = native.drive_prepare_pedaling({}, .01)
    expected = fresh.drive_prepare_pedaling({}, .01)
    assert freeze(result) == freeze(expected)
    assert freeze(native.drive_state()) == freeze(fresh.drive_state())


@pytest.mark.parametrize('field,value', [
    ('control', {'motor_limit_nm': -1.}),
    ('control', {'human_torque_nm': float('nan')}),
    ('control', {'rider_enabled': 1}),
    ('sensed_human_torque_nm', float('nan')),
    ('sensed_human_torque_nm', float('inf')),
])
def test_rejected_compute_keeps_everything(tmp_path, field, value):
    """Wire-level and staging-level rejections publish nothing: not the
    policy candidates, not ctrl, not the diagnostics or timestamp state."""
    model, data, _, native = drive_pair(tmp_path, kind=GEOMETRIC)
    _spin(model, data, native)
    native.drive_components({}, .002, 2.)
    _spin(model, data, native, crank=4.5)
    before = _frozen(native)
    args = {'control': {}, 'dt': .002, 'speed_mps': 2.}
    args[field] = value
    with pytest.raises((ValueError, TypeError)):
        native.drive_components(**args)
    assert _frozen(native) == before


def test_unsettled_pending_rejects_before_policy_advance(tmp_path):
    """An unsettled actuation rejects before pedaling/assist candidates are
    built — retries can never accumulate effort or EMA state."""
    model, data, _, native = drive_pair(tmp_path, kind=GEOMETRIC)
    _spin(model, data, native)
    native.drive_components({}, .002, 2.)  # reserves the pending actuation
    data.time += .002
    _spin(model, data, native)
    before = _frozen(native)
    with pytest.raises(RuntimeError, match='not settled'):
        native.drive_components({}, .002, 2.)
    assert _frozen(native) == before
    with pytest.raises(RuntimeError, match='not settled'):
        native.drive_components({}, .002, 2.)
    assert _frozen(native) == before
    # Clearing pending through a snapshot round-trip restores forward motion.
    state = native.drive_state()
    state['pending_actuation'] = None
    native.set_drive_state(state)
    native.drive_components({}, .002, 2.)


def test_prepare_duplicate_timestamp_is_atomic(tmp_path):
    """The interval clock is shared: a successful compute makes a later
    same-timestamp prepare a rejection, and the rejection writes nothing."""
    model, data, _, native = drive_pair(tmp_path, kind=GEOMETRIC)
    _spin(model, data, native)
    native.drive_components({}, .002, 2.)
    before = _frozen(native)
    with pytest.raises(ValueError, match='once per timestamp'):
        native.drive_prepare_pedaling({}, .002)
    assert _frozen(native) == before


@pytest.mark.parametrize('kind', ['ideal_mid_drive', GEOMETRIC])
def test_probe_atomicity(tmp_path, kind):
    """A probe publishes only its declared telemetry — the force rows and the
    probe diagnostics channel. Policy, clock, pending and model state are
    bitwise untouched even while pending actuation is reserved, and a
    rejecting probe does not even touch probe_last."""
    model, data, _, native = drive_pair(tmp_path, kind=kind)
    _spin(model, data, native)
    native.drive_components({}, .002, 2.)  # set pending + interval clock
    _spin(model, data, native)
    before = native.drive_state()
    components = native.drive_components({}, .002, 2., advance=False)
    assert set(components)  # candidate force rows still returned
    after = native.drive_state()
    assert after['probe_last'] is not None
    probe = dict(after)
    probe['probe_last'] = before['probe_last']
    assert freeze(probe) == freeze(before)
    # A rejected probe leaves the whole snapshot, probe channel included.
    with pytest.raises(ValueError):
        native.drive_components({}, .002, 2., advance=False,
                                sensed_human_torque_nm=float('nan'))
    assert freeze(native.drive_state()) == freeze(after)


def test_python_probe_atomicity(tmp_path):
    """The Python probe clone publishes probe_last only; physical, policy,
    clock and model state are bitwise unchanged — even while a pending
    actuation is reserved."""
    model, data, drive, _ = drive_pair(tmp_path, kind='elastic_chain')
    drive.compute_components(model, data, .002, speed_mps=2.)
    before = python_state(drive, model)
    assert before['probe_last'] is None and before['pending_actuation']
    components = drive.compute_components(model, data, .002, speed_mps=2.,
                                          advance=False)
    assert components
    after = python_state(drive, model)
    assert after['probe_last'] is not None
    after['probe_last'] = before['probe_last']  # declared channel only
    assert freeze(after) == freeze(before)


@pytest.mark.parametrize('kind', ['ideal_mid_drive', GEOMETRIC])
def test_prepare_then_components_matches_internal_prepare(tmp_path, kind):
    """prepare→compute(supplied state) equals compute's internal prepare:
    the staged pedaling advance is identical whether it commits early or is
    supplied back to the same tick."""
    a_dir, b_dir = tmp_path / 'a', tmp_path / 'b'
    a_dir.mkdir()
    b_dir.mkdir()
    model_a, data_a, _, supplied = drive_pair(a_dir, kind=kind)
    model_b, data_b, _, internal = drive_pair(b_dir, kind=kind)
    _spin(model_a, data_a, supplied)
    _spin(model_b, data_b, internal)
    pedaling = supplied.drive_prepare_pedaling({}, .002)
    supplied.drive_components({}, .002, 2., pedaling_state=pedaling)
    internal.drive_components({}, .002, 2.)
    assert freeze(supplied.drive_state()) == freeze(internal.drive_state())


def test_python_rejected_prepare_keeps_policy_state(tmp_path):
    """The Python mirror must be atomic too: a rejected geometric shift must
    not leak shifter EMAs/counters even though the hub stages internally."""
    model, data, drive, _ = drive_pair(tmp_path, kind=GEOMETRIC,
                                       shifting=_invalid_cassette())
    data.qvel[model.joint('crank_spin').dofadr[0]] = 4.
    data.qvel[model.joint('rear_wheel_spin').dofadr[0]] = 1.
    mujoco.mj_forward(model, data)
    before = _python_frozen(drive, model)
    for _ in range(2):
        with pytest.raises(ValueError):
            drive.prepare_pedaling(data, .1, RideControl(), model=model)
        assert _python_frozen(drive, model) == before


def test_python_rejected_compute_keeps_everything(tmp_path):
    """A mid-tick Python rejection (unsettled pending) leaves hub boundary,
    assist state, ctrl and pending untouched."""
    model, data, drive, _ = drive_pair(tmp_path, kind='elastic_chain')
    drive.compute_components(model, data, .002, speed_mps=2.)
    data.time += .002
    mujoco.mj_forward(model, data)
    before = _python_frozen(drive, model)
    ctrl = data.ctrl.copy()
    with pytest.raises(RuntimeError, match='not settled'):
        drive.compute_components(model, data, .002, speed_mps=2.)
    assert _python_frozen(drive, model) == before
    assert np.array_equal(data.ctrl, ctrl)


# ----------------------------- E4: settlement -----------------------------

def _solved_fixture(native, model, data):
    """Arm a pending actuation whose reserved effort matches the solve.

    ``drive_components`` reserves the delivered torque in the pending row;
    a restored pending row plus a matching ctrl row reproduces that arming
    deterministically — the ceiling check admits the solve and the settle
    debits a real energy amount.
    """
    native.drive_components({}, .002, 2., sensed_human_torque_nm=20.)
    armed = native.drive_state()
    assert armed['pending_actuation'] is not None
    armed['pending_actuation'] = {'requested': 1., 'omega': 4., 'dt': .002,
                                  'enabled': True}
    native.set_drive_state(armed)
    data.ctrl[model.actuator('mid_drive').id] = 1.
    native.set_inputs(data.ctrl, data.qfrc_applied)
    mujoco.mj_step(model, data)
    native.step()
    armed = native.drive_state()
    assert armed['pending_actuation']['requested'] == 1.
    return armed


@pytest.mark.parametrize('kind', ['elastic_chain', 'ideal_mid_drive', GEOMETRIC])
def test_rejected_settle_keeps_everything(tmp_path, kind):
    """A settle rejection publishes nothing — not even transmission solve
    telemetry: pending validation runs ahead of every solve, so a refused
    settle leaves battery, pending, diagnostics and hub state frozen."""
    model, data, _, native = drive_pair(tmp_path, kind=kind)
    armed = _solved_fixture(native, model, data)
    # A pending reservation below the solved effort violates the reserved
    # ceiling — the rejection must land before any publication.
    corrupted = copy.deepcopy(armed)
    corrupted['pending_actuation'] = {'requested': .001, 'omega': 4.,
                                    'dt': .002, 'enabled': True}
    native.set_drive_state(corrupted)
    before = _frozen(native)
    for _ in range(2):
        with pytest.raises(ArithmeticError, match='reserved effort ceiling'):
            native.drive_settle_actuation()
        assert _frozen(native) == before
    # Retry from the honest armed state: bitwise-equal to a clean drivetrain
    # settling from the same snapshot — no residual from the rejections.
    # The control steps from the same pre-step engine state so its recorded
    # constraint forces (what the settle reads) are identical.
    native.set_drive_state(armed)
    out = native.drive_settle_actuation()
    fresh_dir = tmp_path / 'fresh'
    fresh_dir.mkdir()
    _, _, _, fresh = drive_pair(fresh_dir, kind=kind)
    fresh.set_drive_state(armed)
    fresh.set_inputs(data.ctrl, data.qfrc_applied)
    fresh.step()
    assert_bitwise_equal(out, fresh.drive_settle_actuation())
    assert freeze(native.drive_state()) == freeze(fresh.drive_state())


def test_battery_budget_rejection_is_atomic(tmp_path):
    """The store check rejects before any publication too — and the retry
    with an honest reservation debits exactly once."""
    model, data, _, native = drive_pair(tmp_path)
    armed = _solved_fixture(native, model, data)
    corrupted = copy.deepcopy(armed)
    corrupted['pending_actuation'] = {'requested': 1000., 'omega': 4.,
                                    'dt': .002, 'enabled': True}
    corrupted['policies']['battery']['energy_j'] = .001
    native.set_drive_state(corrupted)
    before = _frozen(native)
    with pytest.raises(ArithmeticError, match='battery storage'):
        native.drive_settle_actuation()
    assert _frozen(native) == before
    native.set_drive_state(armed)
    native.drive_settle_actuation()
    settled = native.drive_state()
    debit = (settled['policies']['battery']['drawn_energy_j'] -
             armed['policies']['battery']['drawn_energy_j'])
    assert debit > 0. and settled['pending_actuation'] is None


@pytest.mark.parametrize('kind,topology', [('elastic_chain', 'plain')] + [
    (kind, topology)
    for kind in ('ideal_mid_drive', GEOMETRIC)
    for topology in ('plain', 'clutch', 'rotor')])
def test_sparse_last_settle_debits_once(tmp_path, kind, topology):
    """The A12 shape at the FFI: a restored sparse ``last={}`` cannot change
    exactly-once settlement — the detached map republishes the telemetry,
    the battery moves exactly once, and a second settle debits nothing."""
    model, data, _, native = drive_pair(tmp_path, kind=kind, topology=topology)
    armed = _solved_fixture(native, model, data)
    sparse = copy.deepcopy(armed)
    sparse['last'] = {}
    native.set_drive_state(sparse)
    assert native.drive_state()['last'] == {}
    native.drive_settle_actuation()
    settled = native.drive_state()
    debit = (settled['policies']['battery']['drawn_energy_j'] -
             armed['policies']['battery']['drawn_energy_j'])
    assert debit > 0.
    assert settled['pending_actuation'] is None
    assert settled['last']  # telemetry republished through the detached map
    # Control: the same armed snapshot with full diagnostics settles into
    # the same battery/pending/hub outcome; it steps from the same pre-step
    # engine state so the recorded constraint forces match.
    control_dir = tmp_path / 'control'
    control_dir.mkdir()
    _, _, _, control = drive_pair(control_dir, kind=kind, topology=topology)
    control.set_drive_state(armed)
    control.set_inputs(data.ctrl, data.qfrc_applied)
    control.step()
    control.drive_settle_actuation()
    control_state = control.drive_state()
    assert freeze(settled['policies']) == freeze(control_state['policies'])
    for name in ('ideal_hub', 'clutch', 'freewheel'):
        assert freeze(settled[name]) == freeze(control_state[name])
    # Exactly once: settling again moves the battery by nothing at all.
    native.drive_settle_actuation()
    again = native.drive_state()
    assert freeze(again['policies']['battery']) == \
        freeze(settled['policies']['battery'])
    assert again['pending_actuation'] is None


@pytest.mark.parametrize('kind', ['ideal_mid_drive', GEOMETRIC])
def test_python_rejected_settle_keeps_everything(tmp_path, kind):
    """The Python mirror rejects before solving: a refused settle must not
    leak the transmission's own solve state — diagnostics, work counters,
    tension or the consumed shift flag — exactly the hole the staged solve
    closes on both sides."""
    model, data, drive, _ = drive_pair(tmp_path, kind=kind)
    drive.compute_components(model, data, .002, speed_mps=2.,
                             sensed_human_nm=20.)
    assert drive.pending_actuation is not None
    drive.pending_actuation = (1., 4., .002, True)
    # Solved effort far beyond the reserved ceiling: the settle must reject
    # before the transmission publishes anything of its own.
    data.ctrl[drive.actuators['mid_drive']] = 1000.
    data.qfrc_applied[:] = 0.
    mujoco.mj_step(model, data)
    before = _python_frozen(drive, model)
    for _ in range(2):
        with pytest.raises(ArithmeticError, match='reserved effort ceiling'):
            drive.settle_actuation(model, data)
        assert _python_frozen(drive, model) == before
    # An honest solve at the reserved torque settles and debits exactly once.
    data.ctrl[drive.actuators['mid_drive']] = 1.
    mujoco.mj_forward(model, data)
    drive.settle_actuation(model, data)
    drawn = drive.battery.drawn_energy_j
    assert drawn > 0. and drive.pending_actuation is None
    drive.settle_actuation(model, data)
    assert drive.battery.drawn_energy_j == drawn


def test_python_settle_debits_once_after_sparse_diagnostics(tmp_path):
    """The sparse-``last`` shape on the Python mirror: settlement rebuilds
    the telemetry but debits the battery exactly once."""
    model, data, drive, _ = drive_pair(tmp_path)
    drive.compute_components(model, data, .002, speed_mps=2.,
                             sensed_human_nm=20.)
    assert drive.pending_actuation is not None
    drive.pending_actuation = (1., 4., .002, True)
    data.ctrl[drive.actuators['mid_drive']] = 1.
    data.qfrc_applied[:] = 0.
    mujoco.mj_forward(model, data)
    drive.last = {}
    drive.settle_actuation(model, data)
    drawn = drive.battery.drawn_energy_j
    assert drawn > 0. and drive.pending_actuation is None and drive.last
    drive.settle_actuation(model, data)
    assert drive.battery.drawn_energy_j == drawn
