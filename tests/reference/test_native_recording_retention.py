"""A2: retention modes, owned episode sealing and the typed native source.

Retention policy is independent of the measurement schedule: ``full`` keeps
every committed interval even under a decimated energy cadence, ``legacy``
keeps the historical decimated projection and ``off`` keeps no growing
history while pending consumer evidence stays intact. ``seal_episode()``
copies the retained index into a binding-owned value that outlives
``reset()`` and ``close()``, and ``NativePhysicalSource`` encodes it through
the production catalog.
"""
from types import SimpleNamespace

import numpy as np
import pytest

from native_loader import load_native
from _bits import assert_bitwise_equal
from _native_runtime_support import make_python_ride, assert_tree_close
from bike_sim.native.runtime import create_native_ride
from bike_sim.recording.native import NativePhysicalSource, model_joints
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.physical_driver import PhysicalRideDriver
from bike_sim.sim.ride.reference_monitor import InvalidReferenceRun
from bike_sim.recording.catalog import build_catalog
from bike_sim.recording.encoding import decode_rows

bike_native = load_native()

# create_native_ride resolves the application artifact selector
# (BIKE_NATIVE_BUILD_PATH) inside test bodies; the session fixture aligns
# it with the test-selected build before any test here runs.
pytestmark = pytest.mark.usefixtures('native_module')


def _catalog(driver):
    return build_catalog(model_joints(driver.make_render_model()))


def _batch_column(batch, name):
    """One scalar column of a yielded batch, whichever producer emitted it.

    A real native episode dispatches physics/forces through the projector
    (``PieceBatch`` pieces); channels and duck-typed episodes stay on the
    ``encode_rows`` path (``TableBatch`` columns).
    """
    from bike_sim.recording.contracts import PieceBatch
    if isinstance(batch, PieceBatch):
        return batch.pieces[name]['data']
    return batch.columns[name]


def _seal(driver, steps, control=None):
    driver.advance(steps, RideControl() if control is None else control)
    return driver.seal_episode()


def test_full_retention_keeps_every_interval_under_measurement_decimation():
    sim = make_python_ride()
    full = create_native_ride(sim, record_decimation=80, retention='full')
    legacy = create_native_ride(sim, record_decimation=80, retention='legacy')
    try:
        episode = _seal(full, 81)
        assert episode.generation == full.snapshot().generation
        assert episode.row_count == 81
        batch = episode.batch(0, 81)
        assert batch.size == 81
        ids = batch.interval_ids
        assert ids.dtype == np.int64
        assert tuple(int(i) for i in ids) == tuple(range(81))
        # The valid legacy control reports its historical decimated row
        # count under the identical measurement cadence.
        legacy.advance(81)
        legacy.flush()
        np.testing.assert_array_equal(
            legacy.recorded_columns['interval_id'], np.array([0., 80.]))
        sealed = legacy.seal_episode()
        assert sealed.row_count == 2
        assert tuple(int(i) for i in sealed.batch(0, 2).interval_ids) == (0, 80)
    finally:
        full.close()
        legacy.close()


def test_default_retention_is_the_legacy_projection():
    sim = make_python_ride()
    driver = create_native_ride(sim, record_decimation=7)
    try:
        driver.advance(29)
        driver.flush()
        assert driver.recorded_columns.keys() != set()
        np.testing.assert_array_equal(
            driver.recorded_columns['interval_id'], np.arange(0, 29, 7))
        episode = driver.seal_episode()
        assert episode.row_count == 5
        assert tuple(int(i) for i in episode.batch(0, 5).interval_ids) == \
            (0, 7, 14, 21, 28)
    finally:
        driver.close()


def test_off_retention_keeps_pending_evidence_without_history():
    sim = make_python_ride()
    driver = create_native_ride(sim, record_decimation=80, retention='off')
    try:
        driver.advance(13)
        driver.flush()
        assert driver.recorded_columns == {}
        assert driver.recorded_intervals() == []
        # Pending consumer delivery is not optional history: drain still
        # returns every committed interval.
        batch = driver.drain_samples()
        assert batch.interval_ids == tuple(range(13))
        assert driver.seal_episode().row_count == 0
    finally:
        driver.close()


def test_retention_rejects_unknown_mode():
    sim = make_python_ride()
    for mode in ('every', '', 1, True):
        with pytest.raises(ValueError, match='retention'):
            create_native_ride(sim, retention=mode)


def test_seal_flushes_partial_period_and_empty_run():
    sim = make_python_ride()
    driver = create_native_ride(sim, retention='full')
    try:
        period = sim.physical.control_clock.steps_per_period
        assert period > 1
        driver.advance(period - 1)
        episode = driver.seal_episode()
        assert episode.row_count == period - 1
        ids = episode.batch(0, period - 1).interval_ids
        assert tuple(int(i) for i in ids) == tuple(range(period - 1))
        # A fresh runtime seals to an empty episode that still reads.
        empty = create_native_ride(sim, retention='full')
        try:
            sealed = empty.seal_episode()
            assert sealed.row_count == 0
            assert sealed.batch(0, 1).size == 0
        finally:
            empty.close()
    finally:
        driver.close()


def test_episode_batch_validates_range_and_types_before_boxing():
    sim = make_python_ride()
    driver = create_native_ride(sim, retention='full')
    try:
        episode = _seal(driver, 5)
        for start, limit in ((-1, 1), (0, 0), (6, 1), (0, -2),
                             (0.5, 1), (0, 1.5), ('0', 1), (0, 'x'),
                             (True, 1), (0, True)):
            with pytest.raises((ValueError, TypeError)):
                episode.batch(start, limit)
        assert episode.batch(5, 1).size == 0
        assert episode.batch(4, 9).size == 1
    finally:
        driver.close()


def test_episode_samples_expose_typed_fields_and_provenance():
    sim = make_python_ride()
    driver = create_native_ride(sim, record_decimation=80, retention='full')
    try:
        episode = _seal(driver, 25)
        period = sim.physical.control_clock.steps_per_period
        batch = episode.batch(0, 25)
        for index, sample in enumerate(batch.samples):
            assert int(sample.interval_id) == index
            assert isinstance(sample.channels, dict)
            energy = sample.channels['energy']
            assert isinstance(energy['measured'], bool)
            assert energy['measured'] == (
                index % 80 == 0 or (index + 1) % period == 0)
            if energy['measured']:
                assert energy['source_interval_id'] == index
                assert energy['source_time_s'] == pytest.approx(
                    sample.end_time_s)
            else:
                # The forwarded values came from this period's closing
                # measurement — always a measured interval — or, for an
                # unmeasured partial tail, the previous period's closer.
                closing = (index // period + 1) * period - 1
                if closing >= episode.row_count:
                    closing = (index // period) * period - 1
                assert closing >= 0
                assert energy['source_interval_id'] == closing
            assert isinstance(sample.forces, dict) and len(sample.forces) > 0
            for name, force in sample.forces.items():
                assert isinstance(name, str)
                assert isinstance(force, np.ndarray)
                assert force.shape == sample.qvel.shape
            powers = sample.powers_w
            assert isinstance(powers, dict)
            for name, force in sample.forces.items():
                assert powers[name] == pytest.approx(float(force @ sample.qvel))
    finally:
        driver.close()


def test_measurement_provenance_marks_forwarded_rows():
    sim = make_python_ride()
    driver = create_native_ride(sim, record_decimation=80, retention='full')
    try:
        period = sim.physical.control_clock.steps_per_period
        episode = _seal(driver, period + 3)
        rows = [sample.channels['energy']
                for sample in episode.batch(0, period + 3).samples]
        assert rows[0]['measured'] is True
        assert rows[0]['source_interval_id'] == 0
        for index in range(1, period - 1):
            row = rows[index]
            assert row['measured'] is False
            # The whole first period forwards the closing interval's
            # measurement, never the mid-period bootstrap datum.
            assert row['source_interval_id'] == period - 1
            assert row['source_time_s'] == pytest.approx(
                episode.batch(period - 1, 1).samples[0].end_time_s)
        closing_row = rows[period - 1]
        assert closing_row['measured'] is True
        assert closing_row['source_interval_id'] == period - 1
        # The unmeasured partial tail forwards the last measured interval
        # of the PREVIOUS period — the closing row that supplied its datum.
        tail = rows[period]
        assert tail['measured'] is False
        assert tail['source_interval_id'] == period - 1
        assert tail['source_time_s'] == pytest.approx(
            rows[period - 1]['source_time_s'])
    finally:
        driver.close()


def test_sealed_episode_survives_reset_and_close():
    sim = make_python_ride()
    driver = create_native_ride(sim, retention='full')
    episode = _seal(driver, 41)
    ids = tuple(int(i) for i in episode.batch(0, 41).interval_ids)
    driver.reset()
    assert episode.row_count == 41
    assert tuple(int(i) for i in episode.batch(0, 41).interval_ids) == ids
    new_generation = driver.snapshot().generation
    assert episode.generation == new_generation - 1
    driver.close()
    rows = episode.batch(0, 41).as_dict_rows()
    assert [row['interval_id'] for row in rows] == list(range(41))
    assert episode.batch(10, 4).samples[0].channels['energy']['measured'] is True


def test_failed_sealing_leaves_active_evidence_available(monkeypatch):
    sim = make_python_ride()
    driver = create_native_ride(sim, retention='full')
    try:
        driver.advance(9)
        native = driver._native
        calls = {'count': 0}

        class SealFault:
            def __getattr__(self, name):
                return getattr(native, name)

            def seal_episode(self):
                calls['count'] += 1
                if calls['count'] == 1:
                    raise MemoryError('injected sealing allocation failure')
                return native.seal_episode()

        monkeypatch.setattr(driver, '_native', SealFault())
        with pytest.raises(MemoryError, match='injected sealing'):
            driver.seal_episode()
        episode = driver.seal_episode()
        assert episode.row_count == 9
        # The failure did not consume pending delivery or retained rows.
        assert driver.drain_samples().interval_ids == tuple(range(9))
        assert driver.accounting_state['history']['last_id'] == 8
    finally:
        driver.close()


@pytest.mark.slow
def test_strict_flush_rejection_preserves_retained_evidence():
    sim = make_python_ride(strict=True, decimation=7)
    driver = create_native_ride(sim, strict=True, record_decimation=7,
                                retention='full')
    try:
        with pytest.raises(InvalidReferenceRun):
            driver.advance(800)
        episode = driver.seal_episode()
        first_failure = driver.first_failure
        assert first_failure is not None
        # The strict rejection published the offending period before
        # raising; sealing still returns every committed interval.
        assert episode.row_count == driver.accounting_state['history']['last_id'] + 1
        assert tuple(int(i) for i in
                     episode.batch(0, episode.row_count).interval_ids) == \
            tuple(range(episode.row_count))
    finally:
        driver.close()


def test_native_source_emits_catalog_tables_with_provenance():
    sim = make_python_ride()
    driver = create_native_ride(sim, record_decimation=80, retention='full')
    try:
        episode = _seal(driver, 41)
        catalog = _catalog(driver)
        source = NativePhysicalSource(episode, catalog)
        assert source.row_count == 41
        batches = {}
        for table in source.tables(batch_rows=8):
            batches.setdefault(table.table, []).append(table)
        expected_tables = {spec.name for spec in catalog.table_order()}
        assert set(batches) == expected_tables
        physics_rows = sum(batch.row_count for batch in batches['physics'])
        assert physics_rows == 41
        interval_ids = [int(i) for batch in batches['physics']
                        for i in _batch_column(batch, 'interval_id')]
        assert interval_ids == list(range(41))
        energy_rows = [row for batch in batches['channel_energy']
                       for row in decode_rows(
                           catalog.table('channel_energy'), batch)]
        assert len(energy_rows) == 41
        for index, row in enumerate(energy_rows):
            energy = row['energy']
            assert isinstance(energy['measured'], bool)
            assert isinstance(energy['source_interval_id'], int)
            assert isinstance(energy['source_time_s'], float)
        assert energy_rows[0]['energy']['measured'] is True
        assert energy_rows[1]['energy']['measured'] is False
        assert energy_rows[1]['energy']['source_interval_id'] == \
            energy_rows[1]['interval_id'] + (
                sim.physical.control_clock.steps_per_period
                - (energy_rows[1]['interval_id'] + 1)
                % sim.physical.control_clock.steps_per_period) - 1 + 1
        force_rows = sum(batch.row_count for batch in batches['forces'])
        assert force_rows > 41  # several components per interval
        components = set()
        for batch in batches['forces']:
            components.update(_batch_column(batch, 'component'))
        assert components
        # Missing optional members encode explicit NULLs, not padding: an
        # unmeasured interval has no endpoint_mass root.
        mass_rows = [row for batch in batches['channel_mass']
                     for row in decode_rows(
                         catalog.table('channel_mass'), batch)]
        assert 'mass' in mass_rows[1]
        assert 'endpoint_mass' not in mass_rows[1]
        assert 'endpoint_mass' in mass_rows[0]
    finally:
        driver.close()


def test_native_source_tables_validate_batch_rows_and_channel_roots():
    sim = make_python_ride()
    driver = create_native_ride(sim, retention='full')
    try:
        episode = _seal(driver, 4)
        source = NativePhysicalSource(episode, _catalog(driver))
        for batch_rows in (0, -1, 1.5, '8', True):
            with pytest.raises((ValueError, TypeError)):
                list(source.tables(batch_rows=batch_rows))
        assert [b.row_count for b in source.tables(batch_rows=2)
                if b.table == 'physics'] == [2, 2]
        # An unclaimed channel root fails before any batch is emitted.
        corrupt = SimpleNamespace(
            generation=episode.generation, row_count=episode.row_count,
            batch=lambda start, limit: episode.batch(start, limit))
        def bad_batch(start, limit):
            batch = episode.batch(start, limit)
            samples = []
            for sample in batch.samples:
                channels = dict(sample.channels, forged_root={})
                samples.append(SimpleNamespace(
                    interval_id=sample.interval_id, time_s=sample.time_s,
                    end_time_s=sample.end_time_s, qpos=sample.qpos,
                    qvel=sample.qvel, forces=sample.forces,
                    powers_w=sample.powers_w, channels=channels))
            return SimpleNamespace(samples=samples)
        corrupt.batch = bad_batch
        bad_source = NativePhysicalSource(corrupt, _catalog(driver))
        from bike_sim.recording.encoding import UnknownFieldError
        with pytest.raises(UnknownFieldError, match='forged_root'):
            list(bad_source.tables(batch_rows=2))
    finally:
        driver.close()


def test_native_source_encodes_missing_and_nonfinite_values():
    # A synthetic episode through the same duck-typed surface the native
    # value provides: only row_count/batch().samples are consumed.
    import _recording_support as support
    channels = support.energy_channels()
    channels['energy']['mechanical_energy_j'] = float('inf')
    channels['energy']['external_work_j'] = float('-inf')
    n = 8
    sample = SimpleNamespace(
        interval_id=0, time_s=0., end_time_s=.001, qpos=np.zeros(n),
        qvel=np.zeros(n), forces={'external': np.zeros(n)},
        powers_w={'external': 0.}, channels=channels)
    episode = SimpleNamespace(
        generation=1, row_count=1,
        batch=lambda start, limit: SimpleNamespace(samples=[sample]))
    source = NativePhysicalSource(episode, support.support_catalog())
    batches = {b.table: b for b in source.tables(batch_rows=4)}
    mechanical = batches['channel_energy'].columns['mechanical_energy_j']
    assert list(mechanical) == [float('inf')]
    assert list(batches['channel_energy'].columns['measured']) == [True]
    assert list(batches['channel_energy'].columns['source_interval_id']) == [0]


def test_retained_index_is_independent_of_pending_acknowledgement():
    sim = make_python_ride()
    driver = create_native_ride(sim, record_decimation=1, retention='full')
    try:
        driver.advance(9)
        driver.flush()
        # Acknowledgement drains only pending delivery; retained rows stay.
        assert driver.drain_samples().interval_ids == tuple(range(9))
        driver.release_samples()
        episode = driver.seal_episode()
        assert episode.row_count == 9
        assert tuple(int(i) for i in episode.batch(0, 9).interval_ids) == \
            tuple(range(9))
        driver.advance(13)
        episode = driver.seal_episode()
        assert episode.row_count == 13
    finally:
        driver.close()


@pytest.mark.slow
def test_recording_does_not_change_committed_trajectory(environment_factory):
    drivers = [PhysicalRideDriver(
        environment_factory(decimation=80).sim,
        backend='native', duration_s=.01, decimation=80, record=enabled)
        for enabled in (False, True)]
    try:
        for target in (1, 4, 8):
            for driver in drivers:
                driver.advance(target, RideControl(0.))
            assert_bitwise_equal(
                drivers[0].snapshot().integration_state,
                drivers[1].snapshot().integration_state)
            assert_tree_close(
                drivers[0].snapshot().model_status,
                drivers[1].snapshot().model_status)
            assert_tree_close(
                drivers[0]._native.accounting_state['history'],
                drivers[1]._native.accounting_state['history'])
            assert_tree_close(
                drivers[0]._native.accounting_state['energy'],
                drivers[1]._native.accounting_state['energy'])
            assert drivers[0].reason == drivers[1].reason
        drivers[0].flush()
        drivers[1].flush()
        assert_tree_close(drivers[0]._native.first_failure,
                          drivers[1]._native.first_failure)
        assert drivers[0].snapshot().model_status == \
            drivers[1].snapshot().model_status
        # Fixed-step applied inputs: the latest committed interval's
        # control channel (applied motor/human terms, limits and held
        # posture) agrees; the bitwise integration states above already
        # prove every earlier applied input matched.
        assert_tree_close(
            drivers[0].snapshot().latest_sample.channels['control'],
            drivers[1].snapshot().latest_sample.channels['control'])
        episode = drivers[1]._native.seal_episode()
        assert episode.row_count == drivers[1].snapshot().step
    finally:
        for driver in drivers:
            driver.close()


@pytest.mark.slow
def test_legacy_export_survives_record_default(environment_factory, tmp_path):
    driver = PhysicalRideDriver(environment_factory().sim, backend='native',
                                duration_s=.02, decimation=2)
    try:
        driver.advance(16, RideControl(0.))
        driver.flush()
        assert driver.snapshot().step == 16
        assert driver.recorder.rows == 8
        summary = driver.export(tmp_path/'run')
        for name in ('summary.json', 'telemetry.csv', 'intervals.jsonl'):
            assert (tmp_path/'run'/name).is_file()
        import csv
        with (tmp_path/'run'/'telemetry.csv').open() as stream:
            assert len(list(csv.reader(stream))) == 9
    finally:
        driver.close()
