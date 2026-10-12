"""Independent SQL expectations for missing/null/nonfinite and typed ownership."""
from hashlib import sha256
import math
from types import SimpleNamespace
import pytest

from _recording_support import energy_channels, sample_row, support_catalog
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
