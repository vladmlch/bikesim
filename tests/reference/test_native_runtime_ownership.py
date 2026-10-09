"""A4: acknowledgement, immutable outputs, reset/close, and fatal prefixes.

Fatal probes run in subprocesses under the existing selected-artifact loader;
no process-global MuJoCo failure is injected into the pytest worker.
"""
import gc
from types import SimpleNamespace

import numpy as np
import pytest

from native_loader import load_native
from _native_runtime_support import make_python_ride, assert_tree_close
from bike_sim.native.contracts import SampleBatch
from bike_sim.native.runtime import create_native_ride

bike_native = load_native()


@pytest.fixture
def pair():
    sim = make_python_ride()
    driver = create_native_ride(sim)
    yield sim, driver
    driver.close()


def test_batch_nested_values_are_owned_and_read_only():
    source = {'interval_id': 0, 'nested': {'values': [1., 2.]}}
    columns = {'interval_id': np.array([0.])}
    batch = SampleBatch(rows=(source,), interval_ids=(0,), columns=columns)
    source['nested']['values'][0] = 99.
    columns['interval_id'][0] = 99.
    assert batch.rows[0]['nested']['values'] == (1., 2.)
    assert batch.columns['interval_id'][0] == 0.
    with pytest.raises(TypeError):
        batch.rows[0]['nested']['other'] = 1
    with pytest.raises(ValueError):
        batch.columns['interval_id'][0] = 42.
    exported = batch.as_dict_rows()
    exported[0]['nested']['values'][0] = 42.
    assert batch.as_dict_rows()[0]['nested']['values'][0] == 1.


def test_row_materialization_is_lazy_and_retryable():
    class Payload:
        interval_ids = np.array([0], dtype=np.int64)
        columns = {'interval_id': np.array([0.])}
        generation = 4
        attempts = 0

        def as_dict_rows(self):
            self.attempts += 1
            if self.attempts == 1:
                raise MemoryError('injected lazy row allocation failure')
            return [{'interval_id': 0, 'nested': {'values': [3.]}}]

    payload = Payload()
    batch = SampleBatch.from_native(payload)
    assert payload.attempts == 0
    with pytest.raises(MemoryError, match='injected'):
        batch.as_dict_rows()
    assert batch.as_dict_rows() == [{'interval_id': 0, 'nested': {'values': [3.]}}]
    assert payload.attempts == 2
    assert batch.as_dict_rows()[0]['interval_id'] == 0
    assert payload.attempts == 2


@pytest.mark.slow
def test_failed_batch_construction_retains_pending_rows(pair, monkeypatch):
    _, driver = pair
    driver.advance(13)
    driver.flush()
    before = driver.accounting_state['pending_intervals']

    def fail_box(cls, payload):
        raise MemoryError('injected batch boxing failure')

    with monkeypatch.context() as patch:
        patch.setattr(SampleBatch, 'from_native', classmethod(fail_box))
        with pytest.raises(MemoryError, match='injected'):
            driver.drain_samples()
    assert driver.accounting_state['pending_intervals'] == before == 13
    batch = driver.drain_samples()
    assert batch.interval_ids == tuple(range(13))
    assert driver.accounting_state['pending_intervals'] == 0
    assert len(driver.drain_samples()) == 0


@pytest.mark.slow
def test_acknowledgement_rejects_stale_or_cross_generation_prefix(pair):
    _, driver = pair
    driver.advance(3)
    driver.flush()
    old = driver._native.prepare_samples()
    # A previously prepared prefix may be acknowledged after more intervals
    # were published, but it must not discard the appended suffix.
    driver.advance(7)
    driver.flush()
    driver._native.acknowledge_samples(old)
    assert driver.drain_samples().interval_ids == (3, 4, 5, 6)
    with pytest.raises(ValueError, match='prefix|stale'):
        driver._native.acknowledge_samples(old)
    driver.reset()
    driver.advance(3)
    driver.flush()
    with pytest.raises(ValueError, match='generation'):
        driver._native.acknowledge_samples(old)
    assert driver.drain_samples().interval_ids == (0, 1, 2)


@pytest.mark.slow
def test_acknowledgement_cannot_consume_a_different_runtime(pair):
    sim, driver = pair
    other = create_native_ride(sim)
    for owner in (driver, other):
        owner.advance(3)
        owner.flush()
    foreign = other._native.prepare_samples()
    with pytest.raises(ValueError, match='prefix|stale'):
        driver._native.acknowledge_samples(foreign)
    assert driver.drain_samples().interval_ids == (0, 1, 2)
    assert other.drain_samples().interval_ids == (0, 1, 2)
    other.close()


@pytest.mark.slow
def test_samples_columns_and_snapshots_survive_reset_and_close(pair):
    _, driver = pair
    initial = driver.snapshot()
    driver.advance(41)
    driver.flush()
    frame = driver.snapshot()
    batch = driver.drain_samples()
    columns = driver.recorded_columns
    expected = batch.as_dict_rows()
    saved_state = frame.integration_state.tobytes()
    saved_columns = {name: value.copy() for name, value in columns.items()}
    assert frame.latest_sample is not None
    with pytest.raises(ValueError):
        frame.integration_state[0] = 99.
    with pytest.raises(ValueError):
        frame.latest_sample.qvel[0] = 99.
    with pytest.raises(TypeError):
        frame.latest_sample.channels['energy']['loss_j'] = 99.
    with pytest.raises(TypeError):
        frame.view.channels['energy']['loss_j'] = 99.
    reset = driver.reset()
    assert reset.generation == initial.generation + 1
    assert reset.step == 0 and reset.latest_sample is None and reset.first_failure is None
    np.testing.assert_array_equal(reset.integration_state, initial.integration_state)
    assert len(driver.drain_samples()) == 0 and not driver.recorded_columns
    driver.advance(9)
    driver.flush()
    driver.close()
    gc.collect()
    assert frame.integration_state.tobytes() == saved_state
    assert_tree_close(frame.latest_sample.as_dict(), expected[-1])
    assert_tree_close(batch.as_dict_rows(), expected)
    for name, values in saved_columns.items():
        np.testing.assert_array_equal(columns[name], values)
    assert all(value.flags.owndata and not value.flags.writeable for value in batch.columns.values())
    for operation in (driver.snapshot, driver.flush, driver.drain_samples, driver.reset):
        with pytest.raises(RuntimeError, match='closed'):
            operation()


@pytest.mark.slow
def test_unmaterialized_batch_keeps_native_rows_alive_after_close(pair):
    _, driver = pair
    driver.advance(13)
    driver.flush()
    batch = driver.drain_samples()
    assert batch._rows is None
    driver.close()
    gc.collect()
    rows = batch.as_dict_rows()
    assert len(rows) == 13 and [row['interval_id'] for row in rows] == list(range(13))
    assert all(row['schema_version'] == 2 for row in rows)


@pytest.mark.slow
def test_source_and_bootstrap_mutations_do_not_affect_accounting_or_reset(tmp_path):
    sim = make_python_ride()
    driver = create_native_ride(sim, directory=tmp_path / 'captured')
    clean = create_native_ride(sim, directory=tmp_path / 'clean')
    initial = driver.snapshot()
    # The public capture is caller-owned; the runtime decoded its own copy.
    driver._bootstrap.state['runtime']['monitor']['strict'] = True
    driver._bootstrap.state['runtime']['energy']['initial_energy_j'] += 1e9
    driver._bootstrap.state['runtime']['history']['work_j']['forged'] = 1e9
    driver._bootstrap.state['integration_state'][:] = np.nan
    driver._bootstrap.config['record_decimation'] = 999
    sim.data.qpos[:] = 1e4
    del sim
    gc.collect()
    for owner in (driver, clean):
        owner.advance(13)
        owner.flush()
    expected = clean.drain_samples().as_dict_rows()
    assert_tree_close(driver.drain_samples().as_dict_rows(), expected)
    assert_tree_close(driver.accounting_state, clean.accounting_state)
    reset = driver.reset()
    np.testing.assert_array_equal(reset.integration_state, initial.integration_state)
    assert 'forged' not in driver.accounting_state['history']['work_j']
    driver.advance(13)
    driver.flush()
    assert_tree_close(driver.drain_samples().as_dict_rows(), expected)
    driver.close()
    clean.close()


@pytest.mark.parametrize('target', [True, 1.5, '3', -1])
def test_invalid_target_is_rejected_before_native_call(target):
    from bike_sim.native.runtime import NativeRideDriver

    def must_not_run(*args, **kwargs):
        raise AssertionError('native side must not be entered')

    driver = NativeRideDriver(SimpleNamespace(advance=must_not_run), None,
                              native_reference_error=RuntimeError)
    with pytest.raises(ValueError, match='target_step'):
        driver.advance(target)


@pytest.mark.slow
def test_fatal_engine_error_preserves_completed_prefix_and_can_reset(tmp_path):
    from test_native_engine_errors import run_child

    script = r'''
import sys, warnings
from pathlib import Path
import mujoco
import numpy as np
from native_loader import load_native
from _native_runtime_support import make_python_ride, assert_tree_close
from bike_sim.native.runtime import create_native_ride
load_native()
warnings.simplefilter('ignore', RuntimeWarning)
sim = make_python_ride()
root = Path(sys.argv[1])
driver = create_native_ride(sim, directory=root/'faulted')
clean = create_native_ride(sim, directory=root/'clean')
clean.advance(3)
clean.flush()
expected = clean.drain_samples().as_dict_rows()
expected_frame = clean.snapshot()
driver._native._test_fail_at_step(3)
try:
    driver.advance(9)
except mujoco.FatalError:
    pass
else:
    raise AssertionError('fatal engine boundary did not fire')
frame = driver.snapshot()
assert frame.step == 3
np.testing.assert_array_equal(frame.integration_state, expected_frame.integration_state)
assert_tree_close(driver.first_failure, clean.first_failure)
assert driver.accounting_state['history']['last_id'] == 2
assert driver.accounting_state['buffered_intervals'] == 0
assert_tree_close(driver.drain_samples().as_dict_rows(), expected)
try:
    driver.advance(4)
except RuntimeError as error:
    assert 'poison' in str(error).lower()
else:
    raise AssertionError('poisoned engine accepted another step')
reset = driver.reset()
assert reset.step == 0 and reset.generation == frame.generation + 1
assert reset.first_failure is None
driver.advance(3)
driver.flush()
assert_tree_close(driver.drain_samples().as_dict_rows(), expected)
driver.close()
clean.close()
print('A4 fatal prefix retained; reset recovered')
'''
    child = run_child(script, str(tmp_path))
    assert child.returncode == 0, child.stdout + '\n' + child.stderr
    assert 'A4 fatal prefix retained; reset recovered' in child.stdout
