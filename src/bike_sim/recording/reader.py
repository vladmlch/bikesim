"""Read-only, manifest-first container reader; imports no simulator or policy."""
from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
import re

from bike_sim.recording.json_codec import loads
from bike_sim.recording.schema import DATABASE, MANIFEST, REPORT, SCHEMA_VERSION
from bike_sim.recording.storage import sha256_file, validate_database


class RecordingError(ValueError):
    pass


def _readonly(path):
    import duckdb
    return duckdb.connect(str(path), read_only=True, config={
        'enable_external_access': 'false', 'autoload_known_extensions': 'false',
        'autoinstall_known_extensions': 'false'})


def read_manifest(directory):
    directory = Path(directory)
    path = directory/MANIFEST
    if path.is_symlink() or not path.is_file():
        raise RecordingError('recording is unsealed: a regular manifest.json is required')
    manifest = loads(path.read_text(encoding='utf-8'))
    required = {'schema_version', 'sealed', 'run_id', 'database', 'file_sha256',
                'row_counts', 'report_status', 'report_error'}
    if not isinstance(manifest, dict) or set(manifest) != required \
            or type(manifest['schema_version']) is not int or manifest['schema_version'] != SCHEMA_VERSION \
            or manifest['sealed'] is not True or manifest['database'] != DATABASE:
        raise RecordingError('unsupported recording manifest/container schema')
    from uuid import UUID
    try:
        UUID(manifest['run_id'])
    except (ValueError, TypeError, AttributeError) as error:
        raise RecordingError('invalid recording run UUID') from error
    status = manifest['report_status']
    if status not in ('complete', 'failed'):
        raise RecordingError('invalid report status')
    expected = {DATABASE, REPORT} if status == 'complete' else {DATABASE}
    hashes = manifest['file_sha256']
    if not isinstance(hashes, dict) or set(hashes) != expected:
        raise RecordingError('manifest must cover exactly its fixed payload files')
    if status == 'complete' and manifest['report_error'] is not None:
        raise RecordingError('successful report cannot have an error')
    if status == 'failed' and (not isinstance(manifest['report_error'], str)
                               or not manifest['report_error']):
        raise RecordingError('failed report must describe its error')
    for name, digest in hashes.items():
        payload = directory/name
        if not isinstance(digest, str) or re.fullmatch('[0-9a-f]{64}', digest) is None:
            raise RecordingError(f'invalid checksum: {name}')
        if payload.is_symlink() or not payload.is_file():
            raise RecordingError(f'recording payload is not a regular file: {name}')
        if sha256_file(payload) != digest:
            raise RecordingError(f'recording checksum mismatch: {name}')
    counts = manifest['row_counts']
    if not isinstance(counts, dict) or any(type(value) is not int or value < 0 for value in counts.values()):
        raise RecordingError('invalid manifest row counts')
    return manifest


class Recording:
    def __init__(self, directory):
        self.path = Path(directory)
        self.manifest = read_manifest(self.path)
        self.connection = _readonly(self.path/DATABASE)
        try:
            counts = validate_database(self.connection)
            identity = self.connection.execute('SELECT run_id FROM runs').fetchone()[0]
            if str(identity) != self.manifest['run_id'] or counts != self.manifest['row_counts']:
                raise RecordingError('manifest and database identity/extents disagree')
            import hashlib
            result = self.connection.execute('SELECT name,sha256,content FROM assets ORDER BY episode_id,name')
            while rows := result.fetchmany(64):
                for name, digest, content in rows:
                    if hashlib.sha256(content).hexdigest() != digest:
                        raise RecordingError(f'bundled asset checksum mismatch: {name}')
        except BaseException:
            self.connection.close()
            raise

    @property
    def metadata(self):
        row = self.rows('SELECT * FROM runs')[0]
        row['values'] = loads(row.pop('metadata_json'))
        return row

    @property
    def episode_ids(self):
        return tuple(row[0] for row in self.connection.execute(
            'SELECT episode_id FROM episodes ORDER BY episode_id').fetchall())

    @property
    def schema_version(self):
        return self.manifest['schema_version']

    def __enter__(self):
        return self

    def __exit__(self, *exception):
        self.close()

    def close(self):
        self.connection.close()

    def rows(self, sql, parameters=()):
        result = self.connection.execute(sql, parameters)
        names = [column[0] for column in result.description]
        if len(names) != len(set(names)):
            raise RecordingError('query output column names must be unique; use aliases')
        return [dict(zip(names, row)) for row in result.fetchall()]

    def episode(self, episode_id=None, *, research=False):
        episodes = self.rows('SELECT * FROM episodes ORDER BY episode_id')
        if episode_id is None:
            if len(episodes) != 1:
                raise RecordingError('recording has multiple or no episodes; select --episode explicitly')
            selected = episodes[0]
        else:
            if type(episode_id) is not int or episode_id < 0:
                raise RecordingError('episode must be a nonnegative integer')
            selected = next((row for row in episodes if row['episode_id'] == episode_id), None)
            if selected is None:
                raise RecordingError(f'no episode {episode_id}')
        if research and selected['replay_contract_version'] != 3:
            raise RecordingError('ordinary physical recordings are not checked research replay recipes')
        return selected

    def query(self, sql, *, parameters=(), limit=200):
        """One SELECT only, with filesystem/network/extension access disabled.

        Read-only alone does not forbid COPY TO or external table functions;
        both the statement class and engine external-access guard are needed.
        """
        import duckdb
        if type(limit) is not int or limit < 1:
            raise RecordingError('limit must be a positive integer')
        statements = self.connection.extract_statements(sql)
        if len(statements) != 1 or statements[0].type != duckdb.StatementType.SELECT:
            raise RecordingError('sql accepts exactly one read-only SELECT statement')
        result = self.connection.execute(sql, parameters)
        columns = [column[0] for column in result.description]
        if len(columns) != len(set(columns)):
            raise RecordingError('query output column names must be unique; use aliases')
        values = result.fetchmany(limit + 1)
        return dict(columns=columns, rows=[dict(zip(columns, row)) for row in values[:limit]],
                    limit=limit, truncated=len(values) > limit, has_more=len(values) > limit)


@contextmanager
def _report_connection(database):
    """Private pre-publication reader. Never used to bypass CLI manifests."""
    connection = _readonly(database)
    try:
        yield connection
    finally:
        connection.close()


RecordingReader = Recording


def open_recording(path):
    """Public manifest-validated reader factory."""
    return Recording(path)
