"""The analyst ``bike-recording`` CLI over synthetic sealed runs.

These tests never launch a simulator: recordings come from
``synthetic_capture`` and every command is exercised through the same
``main(argv)`` entry point a user gets.
"""
import csv
import io

import pytest

from _recording_support import sample_row, synthetic_capture
from bike_sim.cli.recording import main
from bike_sim.recording.contracts import InputEvent
from bike_sim.recording.json_codec import loads


def _events(*, episode_id):
    return (
        InputEvent(episode_id=episode_id, event_id=0, kind='episode_start',
                   received_wall_s=episode_id + .1, processed_step=0,
                   applied_step=None),
        InputEvent(episode_id=episode_id, event_id=1, kind='quit',
                   received_wall_s=episode_id + .2, processed_step=1,
                   applied_step=None, values={'reason': 'user'}),
    )


def _json(capsys):
    return loads(capsys.readouterr().out)


def test_empty_result_sets_are_successful_and_typed(tmp_path, capsys):
    path = synthetic_capture([sample_row()]).finalize(tmp_path)
    assert main(['events', str(path), '--episode', '0', '--json']) == 0
    events = _json(capsys)
    assert events['rows'] == [] and events['total'] == 0
    assert events['truncated'] is False and events['has_more'] is False
    assert main(['fields', str(path), '--pattern', 'no_such_field_*',
                 '--json']) == 0
    fields = _json(capsys)
    assert fields['rows'] == [] and fields['total'] == 0
    assert main(['window', str(path), '--episode', '0', '--start', '5',
                 '--end', '6', '--fields', 'physics.time_s', '--json']) == 0
    assert _json(capsys)['rows'] == []


def test_unknown_field_exits_nonzero_with_an_explicit_message(
        tmp_path, capsys):
    path = synthetic_capture([sample_row()]).finalize(tmp_path)
    assert main(['window', str(path), '--episode', '0', '--start', '0',
                 '--end', '1', '--fields', 'invented.field']) == 1
    assert 'invented.field' in capsys.readouterr().err
    assert main(['sql', str(path), '--query',
                 'SELECT no_such_column FROM physics']) == 1
    assert 'no_such_column' in capsys.readouterr().err


def test_multi_episode_recording_selects_each_episode(tmp_path, capsys):
    path = synthetic_capture(
        episodes=[[sample_row(i, start=i / 8, dt=1 / 8) for i in range(2)],
                  [sample_row(i, start=i / 4, dt=1 / 4) for i in range(3)]],
        events=_events(episode_id=0) + _events(episode_id=1)).finalize(tmp_path)
    assert main(['overview', str(path), '--json']) == 0
    overview = _json(capsys)
    assert [row['episode_id'] for row in overview['episodes']['rows']] == [0, 1]
    assert [row['physical_rows'] for row in overview['episodes']['rows']] == [2, 3]
    assert main(['events', str(path), '--episode', '1', '--json']) == 0
    result = _json(capsys)
    assert [(row['event_id'], row['kind']) for row in result['rows']] == \
        [(0, 'episode_start'), (1, 'quit')]
    assert all(row['episode_id'] == 1 for row in result['rows'])
    assert main(['sql', str(path), '--query',
                 'SELECT interval_id FROM physics WHERE episode_id=1 '
                 'ORDER BY interval_id', '--json']) == 0
    assert [row['interval_id'] for row in _json(capsys)['rows']] == [0, 1, 2]


def test_csv_export_is_explicit_bounded_and_never_overwrites(
        tmp_path, capsys):
    rows = [sample_row(i, start=i / 8, dt=1 / 8) for i in range(3)]
    path = synthetic_capture(rows).finalize(tmp_path)
    destination = tmp_path / 'telemetry.csv'
    assert main(['export', str(path), '--episode', '0', '--format', 'csv',
                 '--output', str(destination)]) == 0
    result = _json(capsys)
    assert result['rows'] == 3 and result['format'] == 'csv'
    with destination.open(newline='') as stream:
        parsed = list(csv.reader(stream))
    assert parsed[0][0] == 'episode_id' and len(parsed) == 4
    assert [row[1] for row in parsed[1:]] == ['0', '1', '2']
    # A second export to the same name is refused, not silently replaced.
    assert main(['export', str(path), '--episode', '0', '--format', 'csv',
                 '--output', str(destination)]) == 1
    assert 'exists' in capsys.readouterr().err
    assert main(['export', str(path), '--episode', '0', '--format', 'csv',
                 '--output', str(tmp_path / 'one.csv'), '--limit', '1']) == 0
    _json(capsys)
    with (tmp_path / 'one.csv').open(newline='') as stream:
        assert len(list(csv.reader(stream))) == 2


def test_nonzero_exit_codes_for_bad_sql_and_missing_run(tmp_path, capsys):
    path = synthetic_capture([sample_row()]).finalize(tmp_path)
    for sql in ('DELETE FROM physics', 'SELECT 1; SELECT 2',
                'INSERT INTO physics DEFAULT VALUES', 'not sql at all'):
        assert main(['sql', str(path), '--query', sql]) == 1
        assert 'RECORDING ERROR' in capsys.readouterr().err
    assert main(['overview', str(tmp_path / 'missing-run')]) == 1
    assert 'RECORDING ERROR' in capsys.readouterr().err
    assert main(['sql', str(path), '--query', 'SELECT 1']) == 0


def test_sql_output_is_bounded_with_a_truncation_indicator(
        tmp_path, capsys):
    rows = [sample_row(i, start=i / 8, dt=1 / 8) for i in range(211)]
    path = synthetic_capture(rows).finalize(tmp_path)
    assert main(['sql', str(path), '--query',
                 'SELECT interval_id FROM physics ORDER BY interval_id',
                 '--json']) == 0
    result = _json(capsys)
    assert len(result['rows']) == 200
    assert result['truncated'] is True and result['has_more'] is True
    assert result['limit'] == 200
    assert main(['sql', str(path), '--query',
                 'SELECT count(*) AS n FROM physics', '--json']) == 0
    assert _json(capsys)['rows'] == [{'n': 211}]
