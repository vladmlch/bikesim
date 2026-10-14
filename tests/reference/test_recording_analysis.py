"""Bounded, typed, read-only analysis: no engine or recorded policy needed."""
from datetime import datetime, timezone
from decimal import Decimal
import math

import pytest

from _recording_support import (energy_channels, sample_row,
                                synthetic_capture, synthetic_joints)
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


def test_exact_comparison_reports_first_mismatch_with_stable_keys(tmp_path):
    right_rows = _rows()
    right_rows[1] = sample_row(1, start=1 / 8, dt=1 / 8,
        channels=energy_channels(energy={'residual_j': .1 + 1e-6}))
    a = capture_rows(_rows()).finalize(tmp_path / 'a')
    b = capture_rows(right_rows).finalize(tmp_path / 'b')
    with Recording(a) as left, Recording(b) as right:
        result = analysis.compare(left, right, left_episode=0, right_episode=0,
                                  fields=['channel_energy.residual_j'])
        assert result['compatible'] and not result['agrees']
        first = result['first_mismatch']
        assert first['kind'] == 'float'
        assert first['path'] == 'channel_energy.residual_j'
        assert (first['interval_id'], first['time_s']) == (1, .125)
        assert first['left'] == .1 and first['right'] == .1 + 1e-6
        assert result['mismatched_field_rows'] == 1
        assert result['compared_intervals'] == 3
        assert 'abs(a-b) <= atol + rtol*max(abs(a),abs(b))' == \
            result['tolerance_rule']


def test_exact_comparison_reports_prefix_extent_mismatch(tmp_path):
    a = capture_rows(_rows(3)).finalize(tmp_path / 'a')
    b = capture_rows(_rows(2)).finalize(tmp_path / 'b')
    with Recording(a) as left, Recording(b) as right:
        result = analysis.compare(left, right, left_episode=0, right_episode=0)
        assert result['compatible'] and not result['agrees']
        assert result['compared_intervals'] == 2
        assert result['unmatched_extents'] == dict(
            left_intervals=1, right_intervals=0,
            first=dict(left_first_interval=2, right_first_interval=None))
        assert result['first_mismatch'] is None


def test_exact_comparison_rejects_incompatible_units_and_phases(
        tmp_path, monkeypatch):
    # Two sealed recordings always carry the same reviewed field catalog;
    # the semantics gate exists for catalog evolution, so the differing
    # definition arrives through the projection seam directly.
    a = capture_rows(_rows()).finalize(tmp_path / 'a')
    b = capture_rows(_rows()).finalize(tmp_path / 'b')
    original = analysis._selection
    for semantic, value in (('unit', 'furlong/fortnight'), ('phase', 'invented')):
        def patched(reader, episode_id, requested, *, companions=True):
            selections = original(reader, episode_id, requested,
                                  companions=companions)
            for selection in selections:
                if selection['label'] == 'channel_energy.residual_j':
                    selection['definition'] = {**selection['definition'],
                                               semantic: value}
            return selections
        with Recording(a) as left, Recording(b) as right:
            select = lambda r, e, q, *, companions=True: (
                patched if r is right else original)(
                    r, e, q, companions=companions)
            monkeypatch.setattr(analysis, '_selection', select)
            result = analysis.compare(left, right, left_episode=0,
                                      right_episode=0)
            assert not result['compatible'] and not result['agrees']
            assert any(entry.get('reason') == 'field_semantics'
                       and entry.get('field') == 'channel_energy.residual_j'
                       for entry in result['incompatibilities'])
            monkeypatch.undo()


def test_exact_comparison_rejects_incompatible_dof_coordinate_identity(
        tmp_path):
    from bike_sim.recording.catalog import JointCoords, build_catalog
    renamed = tuple(JointCoords(
        name=('renamed_' if index == 0 else '') + joint.name,
        joint_type=joint.joint_type, qposadr=joint.qposadr,
        dofadr=joint.dofadr) for index, joint in enumerate(synthetic_joints()))
    a = capture_rows(_rows()).finalize(tmp_path / 'a')
    b = synthetic_capture(_rows(),
                          catalog=build_catalog(renamed, scalar_aliases={})
                          ).finalize(tmp_path / 'b')
    with Recording(a) as left, Recording(b) as right:
        result = analysis.compare(left, right, left_episode=0, right_episode=0)
        assert not result['compatible'] and not result['agrees']
        assert any(entry.get('reason') == 'coordinate_identity'
                   for entry in result['incompatibilities'])
        assert result['compared_intervals'] == 0


def test_exact_comparison_rejects_differing_measurement_provenance(tmp_path):
    right_rows = _rows()
    right_rows[2] = sample_row(2, start=2 / 8, dt=1 / 8, channels=energy_channels(
        energy={'residual_j': .2, 'measured': False,
                'source_interval_id': 1, 'source_time_s': .25}))
    a = capture_rows(_rows()).finalize(tmp_path / 'a')
    b = capture_rows(right_rows).finalize(tmp_path / 'b')
    with Recording(a) as left, Recording(b) as right:
        result = analysis.compare(left, right, left_episode=0, right_episode=0,
                                  fields=['channel_energy.residual_j'])
        assert not result['compatible'] and not result['agrees']
        assert result['incompatibilities'] == [
            dict(reason='measurement_provenance', interval_id=2)]


@pytest.mark.parametrize('delta,agrees', [
    (5e-10, True),              # strictly inside the default bound
    (1e-9 / (1 - 1e-9), True),  # exactly on abs(a-b) == atol + rtol*max
    (1e-8, False),              # beyond it
])
def test_exact_comparison_numeric_tolerance_boundary(tmp_path, delta, agrees):
    right_rows = _rows()
    right_rows[0] = sample_row(0, start=0., dt=1 / 8,
        channels=energy_channels(energy={'residual_j': delta}))
    a = capture_rows(_rows()).finalize(tmp_path / 'a')
    b = capture_rows(right_rows).finalize(tmp_path / 'b')
    with Recording(a) as left, Recording(b) as right:
        result = analysis.compare(left, right, left_episode=0, right_episode=0,
                                  fields=['channel_energy.residual_j'])
        assert result['compatible'] and result['agrees'] is agrees
        deviation = next(row for row in result['deviations']['rows']
                         if row['field'] == 'channel_energy.residual_j')
        assert deviation['max_absolute_error'] == delta
        if agrees:
            assert result['first_mismatch'] is None
            assert result['mismatched_field_rows'] == 0
        else:
            assert result['first_mismatch']['kind'] == 'float'
            assert result['first_mismatch']['interval_id'] == 0
            assert result['first_mismatch']['absolute_error'] == delta


def test_exact_comparison_distinguishes_null_from_missing(tmp_path):
    # An explicit null leaf and an absent leaf share the value column but
    # not the declared presence column; the companion comparison is what
    # keeps them distinct.
    left_row = energy_channels(energy={'residual_j': None})
    right_row = energy_channels()
    del right_row['energy']['residual_j']
    a = capture_rows([sample_row(0, dt=.125, channels=left_row)]
                     + _rows(2)[1:]).finalize(tmp_path / 'a')
    b = capture_rows([sample_row(0, dt=.125, channels=right_row)]
                     + _rows(2)[1:]).finalize(tmp_path / 'b')
    with Recording(a) as left, Recording(b) as right:
        result = analysis.compare(left, right, left_episode=0, right_episode=0,
                                  fields=['channel_energy.residual_j'])
        assert result['compatible'] and not result['agrees']
        assert result['first_mismatch'] == dict(
            interval_id=0, time_s=0., path='channel_energy.residual_j_present',
            kind='category', left=True, right=False)


def test_exact_comparison_reports_categorical_change(tmp_path):
    def status_rows(verdict):
        status = {'model_valid': True, 'model_violation_counts': {},
                  'numerically_valid': verdict,
                  'calibration_status': 'parameterized_unvalidated'}
        return [sample_row(i, start=i / 8, dt=1 / 8,
                           channels=energy_channels(model_status=status))
                for i in range(3)]
    a = capture_rows(status_rows('true')).finalize(tmp_path / 'a')
    b = capture_rows(status_rows('not_evaluated')).finalize(tmp_path / 'b')
    with Recording(a) as left, Recording(b) as right:
        result = analysis.compare(left, right, left_episode=0, right_episode=0,
                                  fields=['channel_model_status.model_status'])
        assert result['compatible'] and not result['agrees']
        first = result['first_mismatch']
        assert first['kind'] == 'category'
        assert first['path'] == \
            'channel_model_status.model_status.numerically_valid'
        assert first['left'] == 'true' and first['right'] == 'not_evaluated'
        assert result['mismatched_field_rows'] == 3


def test_summary_mode_reports_parameter_changes_without_alignment(tmp_path):
    summary = lambda controller, limit: {
        'resolved_config': {'controller': controller, 'assist_limit_w': limit},
        'outcome': {'reason': 'synthetic'},
        'episode_metrics': {'distance_m': limit}}
    a = synthetic_capture(_rows(), summary=summary('pid', 250.)).finalize(
        tmp_path / 'a')
    b = synthetic_capture(_rows(offset=.125),
                          summary=summary('mpc', 300.)).finalize(tmp_path / 'b')
    with Recording(a) as left, Recording(b) as right:
        result = analysis.compare(left, right, left_episode=0, right_episode=0,
                                  mode='summary')
        assert result['mode'] == 'summary' and result['alignment'] == 'none'
        assert result['left']['resolved_config'] == {
            'controller': 'pid', 'assist_limit_w': 250.}
        assert result['right']['resolved_config'] == {
            'controller': 'mpc', 'assist_limit_w': 300.}
        assert result['left']['metrics']['distance_m'] == 250.
        assert result['right']['metrics']['distance_m'] == 300.
        assert result['left']['physical_rows'] == \
            result['right']['physical_rows'] == 3
        assert 'no automatic alignment' in result['interpretation']
