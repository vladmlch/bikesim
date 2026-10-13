"""Container/publication regressions. These tests do not launch a simulator."""
from dataclasses import replace
from hashlib import sha256
import math

import pytest

from _recording_support import (energy_channels, run_metadata, sample_row,
                                synthetic_capture)
from bike_sim.recording.capture import RunCapture
from bike_sim.recording.contracts import Asset, InputEvent
from bike_sim.recording.json_codec import dumps, loads
from bike_sim.recording.reader import Recording, RecordingError
from bike_sim.recording.storage import write_recording


def capture_rows(rows=(), *, summary=None, assets=()):
    return synthetic_capture(rows, summary=summary, assets=assets)


def test_roundtrip_typed_nonfinite_presence_and_owned_asset(tmp_path):
    values = [0., -0., float('nan'), float('inf'), -float('inf')]
    rows = [sample_row(i, start=i / 8, dt=1 / 8,
                       channels=energy_channels(energy={'residual_j': value}))
            for i, value in enumerate(values)]
    data = b'captured input, not a reference to a mutable pathname'
    asset = Asset('test.txt', 'text/plain', sha256(data).hexdigest(), data)
    capture = capture_rows(rows, summary={'outcome': {'reason': 'synthetic'}}, assets=(asset,))
    path = capture.finalize(tmp_path)
    assert sorted(p.name for p in path.iterdir()) == ['manifest.json', 'run.duckdb', 'summary.md']
    with Recording(path) as reader:
        stored = reader.rows('SELECT residual_j FROM channel_energy ORDER BY interval_id')
        assert stored[0]['residual_j'] == 0.
        assert math.copysign(1., stored[1]['residual_j']) == -1.
        assert math.isnan(stored[2]['residual_j'])
        assert stored[3]['residual_j'] == float('inf')
        assert stored[4]['residual_j'] == -float('inf')
        assert reader.rows('SELECT content FROM assets')[0]['content'] == data
        assert reader.episode()['physical_rows'] == len(rows)
        assert reader.episode()['replay_contract_version'] is None
        assert reader.manifest['report_status'] == 'complete'


def test_default_publisher_handles_empty_run_and_zero_step_episode(tmp_path):
    empty = RunCapture(run_metadata(), enabled=True).finalize(tmp_path / 'no-episodes')
    with Recording(empty) as reader:
        assert reader.episode_ids == ()
    zero = capture_rows().finalize(tmp_path / 'zero-step')
    with Recording(zero) as reader:
        assert reader.episode_ids == (0,)
        assert reader.episode()['physical_rows'] == 0


def test_disabled_capture_does_not_create_output(tmp_path):
    path = tmp_path / 'must-not-exist'
    assert RunCapture(run_metadata(), enabled=False).finalize(path) is None
    assert not path.exists()


def test_publication_collision_does_not_adopt_even_an_empty_directory(tmp_path):
    first = capture_rows().finalize(tmp_path)
    before = {p.name: p.read_bytes() for p in first.iterdir()}
    occupied = first.with_name(first.name + '-1')
    occupied.mkdir()
    second = capture_rows().finalize(tmp_path)
    assert second.name == first.name + '-2'
    assert not list(occupied.iterdir())
    assert {p.name: p.read_bytes() for p in first.iterdir()} == before


def test_report_failure_publishes_valid_database_with_explicit_status(tmp_path):
    def fail_report(database, destination):
        destination.write_text('partial report')
        raise OSError('report failure injection')
    path = write_recording(capture_rows(), tmp_path, _reporter=fail_report)
    with Recording(path) as reader:
        assert reader.manifest['report_status'] == 'failed'
        assert 'report failure injection' in reader.manifest['report_error']
        assert set(reader.manifest['file_sha256']) == {'run.duckdb'}
    assert not (path / 'summary.md').exists()


def test_manifest_is_last_and_failed_link_keeps_owned_evidence_retryable(tmp_path, monkeypatch):
    import bike_sim.recording.storage as storage
    original, calls = storage.os.link, []
    def fail_manifest(source, target):
        calls.append(target.name)
        if target.name == 'manifest.json':
            raise OSError('manifest publication injection')
        return original(source, target)
    capture = capture_rows([sample_row()])
    monkeypatch.setattr(storage.os, 'link', fail_manifest)
    with pytest.raises(OSError, match='manifest publication injection'):
        capture.finalize(tmp_path)
    assert calls == ['run.duckdb', 'summary.md', 'manifest.json']
    assert capture.state == 'publication_failed'
    assert next(capture.iter_episodes()).physical.row_count == 1
    for candidate in tmp_path.iterdir():
        with pytest.raises(RecordingError, match='unsealed'):
            Recording(candidate)
    monkeypatch.setattr(storage.os, 'link', original)
    published = capture.finalize(tmp_path)
    with Recording(published) as reader:
        assert reader.episode()['physical_rows'] == 1


@pytest.mark.parametrize('mutation', ['missing_manifest', 'changed_database', 'symlink_database'])
def test_reader_rejects_unsealed_or_tampered_payload(tmp_path, mutation):
    path = capture_rows().finalize(tmp_path)
    if mutation == 'missing_manifest':
        (path / 'manifest.json').unlink()
    elif mutation == 'changed_database':
        with (path / 'run.duckdb').open('ab') as stream:
            stream.write(b'checksum mismatch')
    else:
        original = path / 'elsewhere.db'
        (path / 'run.duckdb').rename(original)
        (path / 'run.duckdb').symlink_to(original)
    with pytest.raises(RecordingError):
        Recording(path)


def test_resealed_catalog_tamper_is_rejected_before_analysis(tmp_path):
    import duckdb
    path = capture_rows().finalize(tmp_path)
    connection = duckdb.connect(str(path / 'run.duckdb'))
    try:
        connection.execute("UPDATE field_catalog SET unit='invented' WHERE table_name='physics' AND field_name='time_s'")
        connection.execute('CHECKPOINT')
    finally:
        connection.close()
    manifest = loads((path / 'manifest.json').read_text())
    manifest['file_sha256']['run.duckdb'] = sha256((path / 'run.duckdb').read_bytes()).hexdigest()
    (path / 'manifest.json').write_text(dumps(manifest))
    with pytest.raises(ValueError, match='field catalog'):
        Recording(path)


def test_assets_are_verified_before_publication(tmp_path):
    asset = Asset('bad.txt', 'text/plain', '0' * 64, b'not the declared digest')
    capture = capture_rows(assets=(asset,))
    with pytest.raises(ValueError, match='asset checksum'):
        capture.finalize(tmp_path)
    assert not list(tmp_path.glob('*/manifest.json'))


def test_multi_episode_round_trip_keeps_extents_and_events_isolated(tmp_path):
    episodes = ([sample_row(i, start=i / 8, dt=1 / 8) for i in range(3)],
                [sample_row(i, start=i / 4, dt=1 / 4) for i in range(2)])
    events = (
        InputEvent(episode_id=0, event_id=0, kind='episode_start',
                   received_wall_s=0.1, processed_step=0, applied_step=None),
        InputEvent(episode_id=0, event_id=1, kind='quit',
                   received_wall_s=0.2, processed_step=2, applied_step=None,
                   values={'reason': 'user'}),
        InputEvent(episode_id=1, event_id=0, kind='episode_start',
                   received_wall_s=0.3, processed_step=0, applied_step=None),
    )
    path = synthetic_capture(
        episodes=episodes, events=events,
        summary=[{'outcome': {'reason': 'first'}},
                 {'outcome': {'reason': 'second'}}]).finalize(tmp_path)
    with Recording(path) as reader:
        assert reader.episode_ids == (0, 1)
        extents = reader.rows(
            'SELECT episode_id,count(*) AS n,min(interval_id) AS lo,'
            'max(interval_id) AS hi,min(time_s) AS t0,'
            'max(interval_end_s) AS t1 FROM physics GROUP BY episode_id '
            'ORDER BY episode_id')
        assert [(row['episode_id'], row['n'], row['lo'], row['hi'],
                 row['t0'], row['t1']) for row in extents] == \
            [(0, 3, 0, 2, 0., 3 / 8), (1, 2, 0, 1, 0., .5)]
        journal = reader.rows('SELECT episode_id,event_id,kind,values_json '
                              'FROM events ORDER BY episode_id,event_id')
        assert [(row['episode_id'], row['event_id'], row['kind'])
                for row in journal] == \
            [(0, 0, 'episode_start'), (0, 1, 'quit'), (1, 0, 'episode_start')]
        assert loads(journal[1]['values_json']) == {'reason': 'user'}
        assert [row['n'] for row in reader.rows(
            'SELECT count(*) AS n FROM events WHERE episode_id=1')] == [1]
        summaries = reader.rows('SELECT episode_id,summary_json FROM episodes '
                                'ORDER BY episode_id')
        assert [loads(row['summary_json'])['outcome']['reason']
                for row in summaries] == ['first', 'second']


def test_forces_round_trip_records_components_and_checks_qfrc_width(tmp_path):
    qvel = [2.] + [0.] * 7
    rows = [sample_row(i, start=i / 8, dt=1 / 8, qvel=qvel) for i in range(2)]
    path = synthetic_capture(rows, forces=True).finalize(tmp_path)
    with Recording(path) as reader:
        stored = reader.rows('SELECT episode_id,interval_id,component,qfrc,power_w '
                             'FROM forces ORDER BY interval_id,component')
        assert [(row['episode_id'], row['interval_id'], row['component'])
                for row in stored] == \
            [(0, i, component) for i in (0, 1) for component in ('chain', 'tires')]
        expected = {'chain': (3., 6.), 'tires': (-1., -2.)}
        for row in stored:
            qfrc0, power = expected[row['component']]
            assert list(row['qfrc']) == [qfrc0] + [0.] * 7
            assert row['power_w'] == power
        work = reader.rows('SELECT component_work_j FROM channel_energy '
                           'ORDER BY interval_id')
        assert [dict(row['component_work_j']) for row in work] == \
            [{'chain': .75, 'tires': -.25}] * 2
    bad = synthetic_capture([sample_row(0, forces={'chain': [1., 2.]},
                                        powers={'chain': 1.5})])
    with pytest.raises(ValueError, match='force coordinate width'):
        bad.finalize(tmp_path / 'bad')
