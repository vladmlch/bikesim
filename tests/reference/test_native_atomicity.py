"""Native drivetrain transactions: a rejected staged update publishes nothing.

Every mutating drivetrain path — shift, prepare, reset, restore — stages the
candidate on owned scratch model/data and only commits fully validated state.
These tests pin the observable contract over the FFI: frozen ``drive_state``
snapshots, engine qpos/qvel, and the prepared-storage generation counters must
be bitwise-identical after any rejection, and a repeated attempt must reject
identically. Successful commits must land atomically without reallocating the
prepared buffers (generation and capacity stable).
"""
import numpy as np
import pytest

from bike_sim.physics.physical_config import ShiftingConfig
from native_loader import load_native
from native_safety_helpers import drive_pair, freeze

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
    """Everything a rejected transaction must preserve, bit-exact."""
    return (freeze(native.drive_state()), freeze(native.qpos), freeze(native.qvel),
            freeze(native._drive_prepared_storage()))


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
