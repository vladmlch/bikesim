"""Fault injection for final-only publication; never run implicitly."""
import pytest

from _recording_support import sample_row
from test_recording_storage import capture_rows
from bike_sim.recording.reader import Recording


@pytest.mark.parametrize('phase', ['insert', 'COMMIT', 'CHECKPOINT', 'close'])
def test_database_failure_retains_capture_and_retry_uses_new_directory(tmp_path, monkeypatch, phase):
    import bike_sim.recording.storage as storage
    original = storage._connect
    opened = []

    class FailOnce:
        def __init__(self, connection):
            self.connection, self.armed = connection, True
        def __getattr__(self, name):
            return getattr(self.connection, name)
        def execute(self, sql, *args, **kwargs):
            matching = (sql.startswith('INSERT INTO "physics"') if phase == 'insert' else sql == phase)
            if self.armed and matching:
                self.armed = False
                raise OSError('injected ' + phase)
            return self.connection.execute(sql, *args, **kwargs)
        def close(self):
            if self.armed and phase == 'close':
                self.armed = False
                self.connection.close()
                raise OSError('injected close')
            self.connection.close()

    def connect(path):
        connection = original(path)
        opened.append(connection)
        return FailOnce(connection)

    capture = capture_rows([sample_row()])
    monkeypatch.setattr(storage, '_connect', connect)
    try:
        with pytest.raises(OSError, match='injected'):
            capture.finalize(tmp_path)
        assert capture.state == 'publication_failed'
        assert next(capture.iter_episodes()).physical.row_count == 1
        assert not list(tmp_path.glob('*/manifest.json'))
    finally:
        for connection in opened:
            connection.close()
    monkeypatch.setattr(storage, '_connect', original)
    path = capture.finalize(tmp_path)
    assert capture.finalize(tmp_path / 'ignored-after-success') == path
    with Recording(path) as recording:
        assert recording.episode()['physical_rows'] == 1


def test_payload_failure_does_not_seal_or_replace_existing_files(tmp_path, monkeypatch):
    import bike_sim.recording.storage as storage
    unrelated = tmp_path / 'notes.txt'
    unrelated.write_text('belongs to the user')
    original = storage.os.link
    def fail_payload(source, target):
        if target.name == 'run.duckdb':
            raise OSError('payload publication injection')
        return original(source, target)
    capture = capture_rows([sample_row()])
    monkeypatch.setattr(storage.os, 'link', fail_payload)
    with pytest.raises(OSError, match='payload publication'):
        capture.finalize(tmp_path)
    assert unrelated.read_text() == 'belongs to the user'
    assert not list(tmp_path.glob('*/manifest.json'))
    assert capture.state == 'publication_failed'
