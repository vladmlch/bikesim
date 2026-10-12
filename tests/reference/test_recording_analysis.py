"""Bounded, typed, read-only analysis: no engine or recorded policy needed."""
from datetime import datetime, timezone
from decimal import Decimal
import math

import pytest

from _recording_support import energy_channels, sample_row
from test_recording_storage import capture_rows
from bike_sim.recording import analysis
from bike_sim.recording.json_codec import TAG, dumps, loads
from bike_sim.recording.reader import Recording, RecordingError
from bike_sim.cli.recording import export, main


def _rows(count=3, *, offset=0., nan=False):
    return [sample_row(i, start=i / 8 + offset, dt=1 / 8,
                       channels=energy_channels(energy={'residual_j': float('nan') if nan else i * .1}))
            for i in range(count)]


def test_json_boundary_preserves_nonfinite_signed_zero_and_reserved_objects():
    value = dict(nan=float('nan'), positive=float('inf'), negative=-float('inf'), zero=-0.,
                 null=None, exists=False, literal={TAG: 'float64', 'value': 'nan'},
                 decimal=Decimal('1.2300'), timestamp=datetime(2026, 10, 12, tzinfo=timezone.utc))
    result = loads(dumps(value))
    assert math.isnan(result['nan'])
    assert result['positive'] == float('inf') and result['negative'] == -float('inf')
    assert math.copysign(1., result['zero']) == -1.
    assert result['null'] is None and result['exists'] is False
    assert result['literal'] == value['literal']
    assert result['decimal'].as_tuple() == value['decimal'].as_tuple()
    assert result['timestamp'] == value['timestamp']
    for text in ('{"x":1,"x":2}', '{"x":NaN}', '{"x":1e9999}'):
        with pytest.raises(ValueError):
            loads(text)


def test_window_is_start_inclusive_end_exclusive_and_reports_partial_overlap(tmp_path):
    path = capture_rows(_rows()).finalize(tmp_path)
    with Recording(path) as reader:
        window = analysis.window(reader, episode_id=0, start_s=.0625, end_s=.2,
                                 fields=['physics.time_s', 'physics.qvel'])
        assert [row['interval_id'] for row in window['rows']] == [1]
        assert window['rows'][0]['overlaps_end'] is True
        assert [row['interval_id'] for row in window['overlap_before_start']['rows']] == [0]
        assert analysis.window(reader, episode_id=0, start_s=.125, end_s=.25,
                               fields=['physics.interval_id'])['rows'][0]['physics.interval_id'] == 1


def test_bounded_query_and_full_export_are_separate(tmp_path):
    path = capture_rows(_rows(211)).finalize(tmp_path / 'recording')
    with Recording(path) as reader:
        result = reader.query('SELECT interval_id FROM physics ORDER BY interval_id')
        assert len(result['rows']) == 200 and result['truncated'] and result['has_more']
        destination = tmp_path / 'all.jsonl'
        result = export(reader, episode_id=0, format='jsonl', output=destination)
        assert result['rows'] == 211
        assert len(destination.read_text().splitlines()) == 211
        with pytest.raises(FileExistsError):
            export(reader, episode_id=0, format='jsonl', output=destination)
    assert main(['sql', str(path), '--query', 'SELECT 1.25 AS amount', '--json']) == 0


@pytest.mark.parametrize('sql', [
    'DELETE FROM physics', 'CREATE TABLE forbidden(x INTEGER)',
    'SELECT 1; SELECT 2', "COPY physics TO 'forbidden.csv'",
    "SELECT * FROM read_csv('/etc/passwd')", "SELECT * FROM read_parquet('https://example.invalid/a.parquet')",
    "INSTALL httpfs", "LOAD httpfs",
])
def test_sql_cannot_mutate_export_or_access_external_inputs(tmp_path, sql):
    path = capture_rows().finalize(tmp_path)
    with Recording(path) as reader:
        with pytest.raises(Exception):
            reader.query(sql)


def test_exact_comparison_refuses_mismatched_clocks_but_summary_is_available(tmp_path):
    a = capture_rows(_rows()).finalize(tmp_path / 'a')
    b = capture_rows(_rows(offset=.125)).finalize(tmp_path / 'b')
    with Recording(a) as left, Recording(b) as right:
        exact = analysis.compare(left, right, left_episode=0, right_episode=0)
        assert not exact['compatible'] and not exact['agrees']
        assert exact['incompatibilities'][0]['reason'] == 'time_grid'
        summary = analysis.compare(left, right, left_episode=0, right_episode=0, mode='summary')
        assert summary['alignment'] == 'none'
        assert summary['left']['physical_rows'] == summary['right']['physical_rows'] == 3


def test_matching_nonfinite_evidence_is_not_silently_valid(tmp_path):
    a = capture_rows(_rows(nan=True)).finalize(tmp_path / 'a')
    b = capture_rows(_rows(nan=True)).finalize(tmp_path / 'b')
    with Recording(a) as left, Recording(b) as right:
        result = analysis.compare(left, right, left_episode=0, right_episode=0,
                                  fields=['channel_energy.residual_j'])
        assert result['agrees'] and result['matching_invalid_pairs'] == 3
        assert 'INVALID' in result['interpretation']


def test_report_excludes_forwarded_energy_extrema(tmp_path):
    rows = [sample_row(0, dt=.125, channels=energy_channels()),
            sample_row(1, start=.125, dt=.125,
                       channels=energy_channels(energy={'measured': False, 'residual_j': 9999999.}))]
    path = capture_rows(rows).finalize(tmp_path)
    report = (path / 'summary.md').read_text()
    line = next(line for line in report.splitlines() if '`channel_energy.residual_j`' in line)
    assert '9999999' not in line
    assert 'measured IS TRUE' in report
