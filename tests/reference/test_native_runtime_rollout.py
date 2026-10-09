"""A4: schema-2 parity, explicit flush schedules, and one-second rollout."""
import mujoco
import numpy as np
import pytest

from native_loader import load_native
from _native_runtime_support import make_python_ride, advance_python, assert_tree_close
from bike_sim.native.runtime import create_native_ride
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.physical_recorder import PhysicalRecorder

bike_native = load_native()


def _collect(sim, rows):
    for sample in sim.physical.completed_samples:
        if sample.interval_id >= len(rows):
            assert sample.interval_id == len(rows)
            rows.append(sample.as_dict())


def _integration(sim):
    state = np.empty(mujoco.mj_stateSize(sim.model, mujoco.mjtState.mjSTATE_INTEGRATION))
    mujoco.mj_getState(sim.model, sim.data, state, mujoco.mjtState.mjSTATE_INTEGRATION)
    return state


def _assert_rows(actual, expected):
    assert len(actual) == len(expected)
    assert [row['interval_id'] for row in actual] == list(range(len(actual)))
    for actual_row, expected_row in zip(actual, expected):
        assert_tree_close(actual_row, expected_row)


@pytest.mark.slow
def test_native_full_rows_match_python_for_non_aligned_tail():
    sim = make_python_ride()
    native = create_native_ride(sim, strict=False, record_decimation=1)
    expected = advance_python(sim, 41)
    native.advance(41)
    assert native.flush() is None
    batch = native.drain_samples()
    _assert_rows(batch.as_dict_rows(), expected)
    assert len(batch) == 41
    assert len(native.drain_samples()) == 0
    snap = native.snapshot()
    assert_tree_close(snap.latest_sample.as_dict(), expected[-1])
    assert_tree_close(snap.view.channels, snap.latest_sample.channels)
    native.close()


@pytest.mark.slow
def test_identical_non_aligned_explicit_flush_schedule_matches_python():
    sim = make_python_ride(decimation=7)
    native = create_native_ride(sim, record_decimation=7)
    expected, actual = [], []
    # A command change may lie inside a physical control period. Captured
    # held terms, not the controller state at flush time, belong to each row.
    schedule = [(3, RideControl(human_torque_nm=0.)),
                (14, RideControl()), (19, RideControl(human_torque_nm=0.)),
                (41, RideControl())]
    for target, command in schedule:
        while sim.steps < target:
            sim.step(0., 0., control=command)
            _collect(sim, expected)
        sim.physical.flush()
        _collect(sim, expected)
        native.advance(target, command)
        native.flush()
        batch = native.drain_samples().as_dict_rows()
        assert 'attachment_samples' in batch[-1]
        assert 'component_work_j' in batch[-1]
        actual.extend(batch)
    _assert_rows(actual, expected)
    native.close()


@pytest.mark.slow
def test_target_snapshot_drain_and_budget_do_not_close_a_partial_period():
    sim = make_python_ride(decimation=7)
    native = create_native_ride(sim, record_decimation=7)
    whole = create_native_ride(sim, record_decimation=7)
    period = sim.physical.control_clock.steps_per_period
    assert period > 1
    native.advance(period-1)
    before = native.snapshot()
    assert before.latest_sample is None
    assert len(native.drain_samples()) == 0
    assert native.accounting_state['history']['last_id'] is None
    assert native.accounting_state['buffered_intervals'] == period-1
    result = native.advance(period+10, wall_budget_s=0.)
    assert result.reason == 'budget' and result.step == period-1
    assert len(native.drain_samples()) == 0
    np.testing.assert_array_equal(native.snapshot().integration_state, before.integration_state)
    actual = []
    for target in sorted({period, period+1, 2*period-1, 3*period+3}):
        native.advance(target)
        native.snapshot()
        actual.extend(native.drain_samples().as_dict_rows())
    whole.advance(3*period+3)
    whole.flush()
    native.flush()
    actual.extend(native.drain_samples().as_dict_rows())
    _assert_rows(actual, whole.drain_samples().as_dict_rows())
    np.testing.assert_array_equal(native.snapshot().integration_state, whole.snapshot().integration_state)
    native.close()
    whole.close()


@pytest.mark.slow
def test_accounting_all_intervals_is_independent_of_record_decimation():
    sim = make_python_ride()
    every = create_native_ride(sim, record_decimation=1)
    decimated = create_native_ride(sim, record_decimation=7)
    for driver in (every, decimated):
        driver.advance(81)
        driver.flush()
    assert_tree_close(decimated.accounting_state['history'], every.accounting_state['history'])
    assert_tree_close(decimated.accounting_state['energy'], every.accounting_state['energy'])
    assert_tree_close(decimated.model_status, every.model_status)
    assert decimated.first_failure == every.first_failure
    for driver in (every, decimated):
        assert driver.drain_samples().interval_ids == tuple(range(81))
    assert len(every.recorded_columns['interval_id']) == 81
    np.testing.assert_array_equal(decimated.recorded_columns['interval_id'], np.arange(0, 81, 7))
    every.close()
    decimated.close()


@pytest.mark.slow
def test_native_recorded_columns_match_scalar_recorder_without_row_roundtrip():
    sim = make_python_ride(decimation=7)
    recorder = PhysicalRecorder(sim, decimate=7)
    native = create_native_ride(sim, record_decimation=7)
    for _ in range(41):
        sim.step(0., 0., control=RideControl())
        recorder.record(sim)
    sim.physical.flush()
    recorder.record(sim)
    native.advance(41)
    native.flush()
    # Read the native column path before any as_dict_rows() materialization.
    actual, expected = native.recorded_columns, recorder.columns()
    assert actual.keys() == expected.keys()
    for name, column in expected.items():
        np.testing.assert_allclose(actual[name], column, atol=1e-9, rtol=1e-9,
                                   equal_nan=True, err_msg=name)
        assert not actual[name].flags.writeable
    native.close()


@pytest.mark.slow
def test_one_second_800_interval_golden_episode_all_channels():
    sim = make_python_ride(strict=False, decimation=1)
    native = create_native_ride(sim, strict=False, record_decimation=1)
    expected = advance_python(sim, 800)
    assert float(sim.model.opt.timestep) == .00125
    assert len(expected) == 800
    assert native.advance(800).step == 800
    native.flush()
    _assert_rows(native.drain_samples().as_dict_rows(), expected)
    snap = native.snapshot()
    assert snap.time_s == pytest.approx(1., abs=1e-12)
    np.testing.assert_allclose(snap.integration_state, _integration(sim), rtol=1e-9, atol=1e-9)
    assert_tree_close(snap.first_failure, sim.physical.reference_monitor.first_failure)
    assert_tree_close(snap.model_status, sim.physical.model_status.as_dict())
    assert_tree_close(native.accounting_state['history']['work_j'], sim.physical.history.work_j)
    assert_tree_close(native.accounting_state['history']['airtime_s'], sim.physical.history.airtime_s)
    assert native.accounting_state['history']['duration_s'] == pytest.approx(1., abs=1e-12)
    native.close()
