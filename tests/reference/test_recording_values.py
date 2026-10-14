"""Independent SQL expectations for missing/null/nonfinite and typed ownership."""
from hashlib import sha256
import math
from types import SimpleNamespace
import pytest

from _recording_support import (energy_channels, sample_row,
                                support_catalog, synthetic_capture,
                                value_case_rows)
from test_recording_storage import capture_rows
from bike_sim.recording.json_codec import dumps, loads
from bike_sim.recording.native import NativePhysicalSource
from bike_sim.recording.reader import Recording


def test_optional_leaf_and_nullable_parent_have_separate_masks(tmp_path):
    a = energy_channels(energy={'residual_j': float('nan')})
    b = energy_channels(energy={'residual_j': None})
    c = energy_channels()
    del c['energy']['residual_j']
    payloads = [a, b, c, {'energy': None}, {}]
    rows = [sample_row(i, start=i / 8, dt=1 / 8, channels=value)
            for i, value in enumerate(payloads)]
    path = capture_rows(rows).finalize(tmp_path)
    with Recording(path) as recording:
        rows = recording.connection.execute('SELECT residual_j,residual_j_present,'
            '__energy_present,__energy_valid FROM channel_energy ORDER BY interval_id').fetchall()
        assert math.isnan(rows[0][0]) and rows[0][1:] == (True, True, True)
        assert rows[1:] == [(None, True, True, True), (None, False, True, True),
                           (None, False, True, False), (None, False, False, False)]
        assert recording.connection.execute('SELECT typeof(interval_id),typeof(qpos) FROM physics LIMIT 1').fetchone() == ('BIGINT', 'DOUBLE[]')


def test_synthetic_value_cases_cover_the_reviewed_optional_matrix(tmp_path):
    """value_cases=True emits NaN/null/absent leaf, null/absent parent and
    the signed-zero/nonfinite values, each self-measured."""
    path = synthetic_capture(value_cases=True).finalize(tmp_path)
    with Recording(path) as recording:
        rows = recording.connection.execute(
            'SELECT residual_j,residual_j_present,__energy_present,__energy_valid,'
            'measured,source_interval_id,source_time_s FROM channel_energy '
            'ORDER BY interval_id').fetchall()
        value, present, parent, valid = 0, 1, 2, 3
        assert math.isnan(rows[0][value]) and rows[0][present:valid + 1] == (True, True, True)
        assert rows[1][value:valid + 1] == (None, True, True, True)
        assert rows[2][value:valid + 1] == (None, False, True, True)
        assert rows[3][value:valid + 1] == (None, False, True, False)
        assert rows[4][value:valid + 1] == (None, False, False, False)
        assert [row[value] for row in rows[5:]] == \
            [0., -0., float('inf'), -float('inf')]
        # DuckDB's on-disk float compression may normalize the -0.0 sign
        # bit (layout-dependent, observed across reopen); authored fidelity
        # is asserted on the fixture itself rather than the stored column.
        assert math.copysign(1., value_case_rows()[6]['channels']
                             ['energy']['residual_j']) == -1.
        assert all(row[present] is True for row in rows[5:])
        assert all(row[4:7] == (True, i, (i + 1) / 8)
                   for i, row in enumerate(rows) if row[valid])


def test_resealed_presence_contradiction_is_rejected(tmp_path):
    import duckdb
    path = capture_rows([sample_row(channels=energy_channels())]).finalize(tmp_path)
    connection = duckdb.connect(str(path / 'run.duckdb'))
    try:
        connection.execute('UPDATE channel_energy SET residual_j_present=false')
        connection.execute('CHECKPOINT')
    finally:
        connection.close()
    manifest = loads((path / 'manifest.json').read_text())
    manifest['file_sha256']['run.duckdb'] = sha256((path / 'run.duckdb').read_bytes()).hexdigest()
    (path / 'manifest.json').write_text(dumps(manifest))
    with pytest.raises(ValueError, match='presence'):
        Recording(path)


def test_export_reads_each_native_batch_once_without_legacy_columns():
    rows = [sample_row(i, start=i / 8, dt=1 / 8) for i in range(5)]
    samples = [SimpleNamespace(interval_id=row['interval_id'], time_s=row['time_s'],
                end_time_s=row['end_time_s'], qpos=row['qpos'], qvel=row['qvel'],
                forces=row['forces'], powers_w=row['powers_w'], channels=row['channels']) for row in rows]
    calls = []
    class Owner:
        generation, row_count = 0, len(samples)
        def sample_batch(self, start, limit):
            calls.append((start, limit))
            return SimpleNamespace(samples=samples[start:start + limit])
        def batch(self, start, limit):
            raise AssertionError('new typed export requested legacy columns')
    batches = list(NativePhysicalSource(Owner(), support_catalog()).tables(batch_rows=2))
    assert calls == [(0, 2), (2, 2), (4, 2)]
    assert sum(batch.row_count for batch in batches if batch.table == 'physics') == 5
    assert all(batch.row_count <= 2 for batch in batches)


def test_ordinary_commands_tolerate_explicit_null_optional_drive():
    """channels['drive'] = None is a valid explicit null, not a TypeError;
    the required 'control' root still rejects absence and null alike."""
    control = {'motor_torque_nm': 1., 'rider_enabled': True}
    def owner(channels):
        row = SimpleNamespace(interval_id=0, time_s=0., end_time_s=.001,
            qpos=[0.] * 8, qvel=[0.] * 8, forces={}, powers_w={},
            channels=channels)
        return SimpleNamespace(generation=1, row_count=1,
            batch=lambda start, limit: SimpleNamespace(samples=[row]))
    for drive in (None, {'front_brake_demand': .5}):
        source = NativePhysicalSource(owner({'control': control, 'drive': drive}),
                                      support_catalog(), ordinary_commands=True)
        # commands_applied is emitted before the channel tables of the same
        # native batch; this check exercises only the applied projection.
        applied = next(source.tables(batch_rows=4))
        assert applied.table == 'commands_applied'
        demand = applied.columns['front_brake_demand'][0]
        present = applied.columns['front_brake_demand_present'][0]
        assert (demand, present) == ((.5, True) if drive else (None, False))
    from bike_sim.recording.encoding import UnknownFieldError
    for missing in (None,):
        source = NativePhysicalSource(owner({'control': missing, 'drive': {}}),
                                      support_catalog(), ordinary_commands=True)
        with pytest.raises(UnknownFieldError, match='ordinary applied control'):
            list(source.tables(batch_rows=4))
