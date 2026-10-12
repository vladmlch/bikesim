"""End-of-launch transactional publication. No engine or policy imports.

An exclusively owned staging directory survives every failed publication.
Only the manifest makes a run readable. RunCapture retains its evidence on
failure and can retry; existing output, including empty directories, is
never adopted or replaced.
"""
from __future__ import annotations

from dataclasses import asdict
from datetime import timezone
import hashlib
import os
from pathlib import Path
import tempfile

from bike_sim.recording.json_codec import dumps, thaw
from bike_sim.recording.schema import (
    BATCH_ROWS, DATABASE, MANIFEST, REPORT, SCHEMA_VERSION, all_tables,
    arrow_batch, create_metadata, create_streams, create_views, parent_masks, quote,
)


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _connect(path):
    import duckdb
    return duckdb.connect(str(path), config={'autoload_known_extensions': 'false',
                                           'autoinstall_known_extensions': 'false'})


def _insert_rows(connection, table, rows):
    if rows:
        marks = ','.join('?' for _ in rows[0])
        connection.executemany(f'INSERT INTO {quote(table)} VALUES ({marks})', rows)


def _catalog_rows():
    rows = []
    for spec in all_tables():
        for field in spec.keys + spec.fields:
            rows.append((spec.name, field.key, field.duckdb_type(), field.unit,
                         field.phase, field.optional, field.measurement, field.source,
                         field.frame, dumps(asdict(field))))
            if field.optional:
                rows.append((spec.name, field.key + '_present', 'BOOLEAN', '1', field.phase,
                             False, 'declared', field.source, 'n/a', dumps({
                                 'presence_of': field.key, 'description': 'field existed in source payload'})))
        for name in parent_masks(spec):
            rows.append((spec.name, name, 'BOOLEAN', '1', 'interval', False,
                         'declared', spec.source, 'n/a', dumps({'parent_mask': name})))
    return rows


def _catalog(connection):
    _insert_rows(connection, 'field_catalog', _catalog_rows())


def _metrics(connection, episode_id, values):
    """Typed scalar metrics; structured outcome metadata is explicitly tagged."""
    for name, value in values.items():
        row = [episode_id, name, 'null', None, None, None, None, None, None]
        if type(value) is bool:
            row[2], row[5] = 'boolean', value
        elif type(value) is int:
            row[2], row[4] = 'bigint', value
        elif type(value) is float:
            row[2], row[3] = 'double', value
        elif isinstance(value, str):
            row[2], row[6] = 'text', value
        elif value is not None:
            row[2], row[8] = 'metadata', dumps(value)
        _insert_rows(connection, 'metrics', [tuple(row)])


def _write_batch(connection, spec, batch, episode_id):
    import pyarrow as pa
    if not batch.row_count:
        return
    record = arrow_batch(spec, batch, episode_id, pa)
    name = '_owned_recording_batch'
    connection.register(name, pa.Table.from_batches([record]))
    try:
        columns = ', '.join(quote(column) for column in record.schema.names)
        connection.execute(f'INSERT INTO {quote(batch.table)} ({columns}) SELECT {columns} FROM {quote(name)}')
    except BaseException as error:
        try:
            connection.unregister(name)
        except Exception as cleanup:
            error.add_note(f'Arrow relation cleanup also failed: {cleanup}')
        raise
    else:
        connection.unregister(name)


def _streams(connection, source, episode_id, specs, *, batch_rows):
    for batch in source.tables(batch_rows=batch_rows):
        if batch.table not in specs:
            raise ValueError(f'unreviewed table: {batch.table}')
        if batch.row_count > batch_rows:
            raise ValueError(f'{batch.table}: source exceeded the batch_rows bound')
        _write_batch(connection, specs[batch.table], batch, episode_id)


def _episode(connection, evidence, specs, *, batch_rows):
    research = evidence.research
    _insert_rows(connection, 'episodes', [(evidence.episode_id, evidence.generation,
        evidence.physical.row_count, 0 if research is None else research.complete_control_steps,
        None if research is None else research.replay_contract_version,
        False if research is None else research.replay_eligible, dumps(evidence.summary))])
    catalog = getattr(evidence.physical, 'catalog', None)
    if catalog is None and evidence.physical.row_count:
        raise ValueError('nonempty physical evidence must own its coordinate catalog')
    if catalog is not None:
        coordinates = catalog.coordinates.qpos + catalog.coordinates.dof
        _insert_rows(connection, 'coordinate_catalog', [
            (evidence.episode_id, c.kind, c.index, c.joint, c.component, c.dof_kind, c.unit)
            for c in coordinates])
        _insert_rows(connection, 'alias_catalog', [
            (evidence.episode_id, a.alias, a.canonical, a.index, a.parameter, a.note)
            for a in catalog.aliases])
    for asset in evidence.assets:
        if hashlib.sha256(asset.content).hexdigest() != asset.sha256:
            raise ValueError(f'asset checksum mismatch: {asset.name}')
        _insert_rows(connection, 'assets', [(evidence.episode_id, asset.name,
            asset.media_type, asset.sha256, asset.content)])
    physical_specs = {name: spec for name, spec in specs.items()
                      if name not in ('commands_requested', 'commands_applied', 'observations', 'transitions')}
    if research is None and getattr(evidence.physical, 'ordinary_commands', False):
        physical_specs['commands_applied'] = specs['commands_applied']
    _streams(connection, evidence.physical, evidence.episode_id, physical_specs, batch_rows=batch_rows)
    if research is not None:
        research_specs = {name: spec for name, spec in specs.items() if name not in physical_specs}
        _streams(connection, research.history, evidence.episode_id, research_specs, batch_rows=batch_rows)
        _insert_rows(connection, 'integration_states', [
            (evidence.episode_id, phase, 'mjSTATE_INTEGRATION', state.tolist())
            for phase, state in (('initial', research.initial_integration_state),
                                 ('final', research.final_integration_state))])
    _metrics(connection, evidence.episode_id, thaw(evidence.summary).get('episode_metrics', {}))


def _validate_presence(connection, spec):
    invalid = []
    parents = {field.path[:depth] for field in spec.fields
               for depth in range(1, len(field.path))}
    parents.update((root,) for root in spec.claims)
    for parent in sorted(parents):
        marker = '__' + '.'.join(parent)
        present, valid = quote(marker + '_present'), quote(marker + '_valid')
        invalid.extend((f'{present} IS NULL', f'{valid} IS NULL',
                        f'({valid} AND NOT {present})'))
        if len(parent) > 1:
            ancestor = quote('__' + '.'.join(parent[:-1]) + '_valid')
            invalid.append(f'({present} AND NOT {ancestor})')
    for field in spec.keys + spec.fields:
        value = quote(field.key)
        ancestors = [quote('__' + '.'.join(field.path[:depth]) + '_valid')
                     for depth in range(1, len(field.path))]
        active = ' AND '.join(ancestors) or 'TRUE'
        root_leaf = len(field.path) == 1 and field.path[0] in spec.claims
        if field.optional:
            present = quote(field.key + '_present')
            invalid.extend((f'{present} IS NULL', f'(NOT {present} AND {value} IS NOT NULL)',
                            f'(NOT ({active}) AND ({present} OR {value} IS NOT NULL))'))
            if root_leaf:
                root_present = quote('__' + field.path[0] + '_present')
                invalid.append(f'({present} != {root_present})')
        else:
            if root_leaf:
                active = quote('__' + field.path[0] + '_valid')
            invalid.extend((f'(({active}) AND {value} IS NULL)',
                            f'(NOT ({active}) AND {value} IS NOT NULL)'))
        if root_leaf:
            root_valid = quote('__' + field.path[0] + '_valid')
            invalid.append(f'({root_valid} != ({value} IS NOT NULL))')
    if invalid and connection.execute(f'SELECT count(*) FROM {quote(spec.name)} WHERE ' +
                                      ' OR '.join(invalid)).fetchone()[0]:
        raise ValueError(f'{spec.name}: inconsistent presence or missing required value')


def validate_database(connection):
    """Validate relational identity and extents before commit and on read.

    Failure evidence may have one requested-but-incomplete control window.
    That suffix is not a fabricated transition and is never replay eligible.
    """
    from bike_sim.recording.schema import column_types
    expected_tables = {spec.name for spec in all_tables()} | {
        'recording_meta', 'runs', 'episodes', 'events', 'field_catalog',
        'coordinate_catalog', 'alias_catalog', 'assets', 'integration_states', 'metrics'}
    tables = {row[0] for row in connection.execute(
        "SELECT table_name FROM information_schema.tables WHERE table_schema='main' "
        "AND table_type='BASE TABLE'").fetchall()}
    if tables != expected_tables:
        raise ValueError('recording table set differs from container schema 1')
    meta = connection.execute('SELECT schema_version, run_id, codec FROM recording_meta').fetchall()
    runs = connection.execute('SELECT run_id FROM runs').fetchall()
    if len(meta) != 1 or len(runs) != 1 or meta[0][0] != SCHEMA_VERSION or meta[0][1] != runs[0][0] \
            or meta[0][2] != 'bike_recording.json.v1':
        raise ValueError('invalid recording identity/schema')
    for spec in all_tables():
        actual = connection.execute(f'DESCRIBE {quote(spec.name)}').fetchall()
        expected_types = column_types(spec)
        if [row[0] for row in actual] != list(expected_types):
            raise ValueError(f'{spec.name}: columns do not match recording schema 1')
        normalize = lambda value: ''.join(value.replace('"', '').upper().split())
        if any(normalize(row[1]) != normalize(expected_types[row[0]]) for row in actual):
            raise ValueError(f'{spec.name}: types do not match recording schema 1')
        _validate_presence(connection, spec)
    expected_catalog = sorted(_catalog_rows(), key=lambda row: (row[0], row[1]))
    actual_catalog = connection.execute('SELECT * FROM field_catalog ORDER BY table_name,field_name').fetchall()
    if actual_catalog != expected_catalog:
        raise ValueError('field catalog differs from the reviewed container schema')
    for table in ('assets', 'integration_states', 'metrics', 'coordinate_catalog', 'alias_catalog'):
        if connection.execute(f'SELECT count(*) FROM {quote(table)} t LEFT JOIN episodes e '
                              'USING(episode_id) WHERE e.episode_id IS NULL').fetchone()[0]:
            raise ValueError(f'{table}: orphan episode')
    if connection.execute('SELECT count(*) FROM episodes WHERE episode_id<0 OR generation<0 '
                          'OR physical_rows<0 OR complete_control_steps<0').fetchone()[0]:
        raise ValueError('negative episode identity or extent')
    if connection.execute('SELECT count(*) FROM events WHERE event_id<0 OR processed_step<0 '
                          'OR applied_step<processed_step OR NOT isfinite(received_wall_s)').fetchone()[0]:
        raise ValueError('invalid event identity or processing clock')
    episodes = connection.execute('SELECT episode_id, physical_rows, complete_control_steps, '
                                  'replay_contract_version, replay_eligible FROM episodes ORDER BY episode_id').fetchall()
    identifiers = [row[0] for row in episodes]
    if identifiers and identifiers != list(range(identifiers[0], identifiers[0] + len(identifiers))):
        raise ValueError('episode identities are not consecutive')
    for spec in all_tables():
        if connection.execute(f'SELECT count(*) FROM {quote(spec.name)} t '
                              'LEFT JOIN episodes e USING(episode_id) WHERE e.episode_id IS NULL').fetchone()[0]:
            raise ValueError(f'{spec.name}: orphan episode')
        if spec.name.startswith('channel_') or spec.name == 'forces':
            if connection.execute(f'SELECT count(*) FROM {quote(spec.name)} t '
                                  'LEFT JOIN physics p USING(episode_id,interval_id) WHERE p.interval_id IS NULL').fetchone()[0]:
                raise ValueError(f'{spec.name}: orphan physical interval')
    if connection.execute('SELECT count(*) FROM events e LEFT JOIN episodes p USING(episode_id) '
                          'WHERE p.episode_id IS NULL').fetchone()[0]:
        raise ValueError('input event references an unsealed episode')
    for episode_id, physical_rows, controls, version, eligible in episodes:
        event_count, first_event, last_event = connection.execute(
            'SELECT count(*),min(event_id),max(event_id) FROM events WHERE episode_id=?',
            [episode_id]).fetchone()
        if event_count and (first_event != 0 or last_event != event_count - 1):
            raise ValueError('event identities must be consecutive from zero')
        extent = connection.execute('SELECT count(*), min(interval_id), max(interval_id) FROM physics WHERE episode_id=?', [episode_id]).fetchone()
        if extent[0] != physical_rows or (physical_rows and extent[2] - extent[1] + 1 != physical_rows):
            raise ValueError(f'episode {episode_id}: physical prefix is incomplete')
        if connection.execute('SELECT count(*) FROM physics WHERE episode_id=? AND '
            '(time_s IS NULL OR interval_end_s IS NULL OR dt_s IS NULL OR '
            'NOT isfinite(time_s) OR NOT isfinite(interval_end_s) OR NOT isfinite(dt_s) OR '
            'dt_s <= 0 OR interval_end_s <= time_s OR dt_s != interval_end_s-time_s)', [episode_id]).fetchone()[0]:
            raise ValueError(f'episode {episode_id}: invalid physical interval clock')
        if connection.execute('SELECT count(*) FROM (SELECT time_s, lag(interval_end_s) OVER '
            '(ORDER BY interval_id) AS previous_end FROM physics WHERE episode_id=?) '
            'WHERE previous_end IS NOT NULL AND time_s != previous_end', [episode_id]).fetchone()[0]:
            raise ValueError(f'episode {episode_id}: discontinuous physical clock')
        for spec in all_tables():
            if spec.name.startswith('channel_'):
                count = connection.execute(f'SELECT count(*) FROM {quote(spec.name)} WHERE episode_id=?',
                                           [episode_id]).fetchone()[0]
                if count != physical_rows:
                    raise ValueError(f'{spec.name}: channel coverage must match the physical prefix')
        coordinates = connection.execute('SELECT kind,coordinate_index FROM coordinate_catalog '
            'WHERE episode_id=? ORDER BY kind,coordinate_index', [episode_id]).fetchall()
        for kind in {row[0] for row in coordinates}:
            indices = [row[1] for row in coordinates if row[0] == kind]
            if kind not in ('qpos', 'dof') or indices != list(range(len(indices))):
                raise ValueError('invalid coordinate catalog identity')
        dimensions = dict(connection.execute('SELECT kind,count(*) FROM coordinate_catalog WHERE episode_id=? GROUP BY kind', [episode_id]).fetchall())
        for vector, kind in (('qpos', 'qpos'), ('qvel', 'dof')):
            dimension = dimensions.get(kind, 0)
            if connection.execute(f'SELECT count(*) FROM physics WHERE episode_id=? AND '
                                  f'({quote(vector)} IS NULL OR len({quote(vector)}) != ?)', [episode_id, dimension]).fetchone()[0]:
                raise ValueError(f'episode {episode_id}: {vector} does not match coordinate catalog')
        if connection.execute('SELECT count(*) FROM forces WHERE episode_id=? AND '
                              '(qfrc IS NULL OR len(qfrc) != ?)', [episode_id, dimensions.get('dof', 0)]).fetchone()[0]:
            raise ValueError(f'episode {episode_id}: force coordinate width differs')
        counts = {table: connection.execute(f'SELECT count(*) FROM {quote(table)} WHERE episode_id=?', [episode_id]).fetchone()[0]
                  for table in ('commands_requested', 'commands_applied', 'observations', 'transitions', 'integration_states')}
        if version is None:
            if eligible or controls or any(counts[name] for name in ('observations', 'transitions', 'integration_states')):
                raise ValueError('ordinary evidence must not claim checked research replay')
            continue
        if version != 3 or counts['transitions'] != controls or counts['observations'] != controls + 1 \
                or counts['commands_requested'] not in (controls, controls + 1) or counts['integration_states'] != 2:
            raise ValueError(f'episode {episode_id}: incomplete research identities/extents')
        for table, key in (('commands_requested', 'request_id'), ('commands_applied', 'application_id'),
                           ('observations', 'observation_id'), ('transitions', 'transition_id')):
            n, first, last = connection.execute(f'SELECT count(*), min({key}), max({key}) FROM {table} WHERE episode_id=?', [episode_id]).fetchone()
            if n and (first != 0 or last != n - 1):
                raise ValueError(f'{table}: identities must be consecutive from zero')
        if connection.execute('SELECT count(*) FROM transitions WHERE episode_id=? AND '
            '(request_id != transition_id OR previous_observation_id != transition_id OR observation_id != transition_id+1)', [episode_id]).fetchone()[0]:
            raise ValueError('transition observation/request references disagree')
        if connection.execute('SELECT count(*) FROM transitions WHERE episode_id=? '
                              'AND (physics_steps IS NULL OR physics_steps<=0)', [episode_id]).fetchone()[0]:
            raise ValueError('a completed transition must span positive physical steps')
        for table in ('commands_requested', 'commands_applied'):
            if connection.execute(f'SELECT count(*) FROM {table} WHERE episode_id=? AND '
                '(step IS NULL OR step<0 OR time_s IS NULL OR time_s<0 OR NOT isfinite(time_s))',
                [episode_id]).fetchone()[0]:
                raise ValueError(f'{table}: invalid research control clock')
        if connection.execute('SELECT count(*) FROM observations WHERE episode_id=? AND '
            '(time_s IS NULL OR source_time_s IS NULL OR NOT isfinite(time_s) OR '
            'NOT isfinite(source_time_s) OR time_s<0 OR source_time_s<0 OR '
            'source_time_s>time_s+1e-12 OR specific_force_body_mps2 IS NULL OR '
            'len(specific_force_body_mps2)!=3)', [episode_id]).fetchone()[0]:
            raise ValueError('invalid observation acquisition/delivery clock or IMU vector')
        if not counts['commands_applied']:
            raise ValueError('research capture must include the startup applied command')
        states = connection.execute('SELECT phase,signature,len(state) FROM integration_states WHERE episode_id=? ORDER BY phase', [episode_id]).fetchall()
        if [s[0] for s in states] != ['final', 'initial'] or any(s[1] != 'mjSTATE_INTEGRATION' for s in states) or states[0][2] != states[1][2]:
            raise ValueError('invalid integration state pair')
        if eligible:
            completed_steps = connection.execute('SELECT coalesce(sum(physics_steps),0) FROM transitions '
                                                 'WHERE episode_id=?', [episode_id]).fetchone()[0]
            if counts['commands_requested'] != controls or completed_steps != physical_rows:
                raise ValueError('an incomplete control/physical suffix cannot be checked-replay eligible')
    return {name: connection.execute(f'SELECT count(*) FROM {quote(name)}').fetchone()[0]
            for (name,) in connection.execute("SELECT table_name FROM information_schema.tables WHERE table_schema='main' AND table_type='BASE TABLE' ORDER BY table_name").fetchall()}


def _database(capture, path, *, batch_rows):
    connection = _connect(path)
    try:
        connection.execute('BEGIN TRANSACTION')
        create_metadata(connection)
        create_streams(connection)
        metadata = capture.metadata
        _insert_rows(connection, 'recording_meta', [(SCHEMA_VERSION, str(metadata.run_id), 'bike_recording.json.v1')])
        _insert_rows(connection, 'runs', [(str(metadata.run_id), metadata.started_at,
                                          metadata.backend, metadata.mode, dumps(metadata.values))])
        _catalog(connection)
        specs = {spec.name: spec for spec in all_tables()}
        ordinary = set()
        for evidence in capture.iter_episodes():
            if evidence.research is None:
                ordinary.add(evidence.episode_id)
            _episode(connection, evidence, specs, batch_rows=batch_rows)
        pending, requests = [], []
        request_episode, request_id = None, 0
        from bike_sim.recording.encoding import encode_rows

        def flush_requests():
            if requests:
                spec = specs['commands_requested']
                _write_batch(connection, spec, encode_rows(spec, requests, episode_id=request_episode), request_episode)
                requests.clear()

        for event in capture.iter_events():
            if event.episode_id in ordinary and event.kind == 'command_requested':
                if request_episode != event.episode_id:
                    flush_requests()
                    request_episode, request_id = event.episode_id, 0
                values = thaw(event.values)
                requests.append(dict(request_id=request_id,
                    **{key: values[key] for key in ('time_s', 'step', 'control',
                        'front_brake_demand', 'rear_brake_demand')}))
                request_id += 1
                if len(requests) == batch_rows:
                    flush_requests()
            pending.append((event.episode_id, event.event_id, event.kind, event.received_wall_s,
                            event.processed_step, event.applied_step, dumps(event.values)))
            if len(pending) == batch_rows:
                _insert_rows(connection, 'events', pending)
                pending = []
        _insert_rows(connection, 'events', pending)
        flush_requests()
        create_views(connection)
        from bike_sim.recording.schema import create_legacy_view
        create_legacy_view(connection)
        counts = validate_database(connection)
        connection.execute('COMMIT')
        connection.execute('CHECKPOINT')
    except BaseException as error:
        try:
            connection.execute('ROLLBACK')
        except Exception as rollback:
            error.add_note(f'rollback unavailable or also failed: {rollback}')
        try:
            connection.close()
        except Exception as close:
            error.add_note(f'database close also failed: {close}')
        raise
    else:
        connection.close()
        return counts


def _sync_file(path):
    with Path(path).open('rb') as stream:
        os.fsync(stream.fileno())


def _report(database, destination):
    from bike_sim.recording.report import write_summary
    write_summary(database, destination)


def write_recording(capture, out_root, *, batch_rows=BATCH_ROWS, _reporter=None):
    if not capture.enabled:
        return None
    if type(batch_rows) is not int or batch_rows < 1:
        raise ValueError('batch_rows must be a positive integer')
    root = Path(out_root)
    root.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix='.run-recording-unsealed-', dir=root))
    (staging/'UNSEALED').write_text('A manifest is required. This directory is not a recording.\n', encoding='utf-8')
    destination = None
    try:
        counts = _database(capture, staging/DATABASE, batch_rows=batch_rows)
        report_status, report_error = 'complete', None
        try:
            (_report if _reporter is None else _reporter)(staging/DATABASE, staging/REPORT)
            if not (staging/REPORT).is_file():
                raise RuntimeError('report generator produced no summary.md')
        except Exception as error:
            report_status, report_error = 'failed', f'{type(error).__name__}: {error}'
        payloads = [DATABASE] + ([REPORT] if report_status == 'complete' else [])
        for name in payloads:
            _sync_file(staging/name)
        manifest = dict(schema_version=SCHEMA_VERSION, sealed=True,
                        run_id=str(capture.metadata.run_id), database=DATABASE,
                        file_sha256={name: sha256_file(staging/name) for name in payloads},
                        row_counts=counts, report_status=report_status, report_error=report_error)
        candidate = staging/'sealed-manifest.json'
        candidate.write_text(dumps(manifest, indent=2)+'\n', encoding='utf-8')
        _sync_file(candidate)
        timestamp = capture.metadata.started_at.astimezone(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        # mkdir owns the name, not exists()+replace(). Even an empty collision
        # belongs to somebody else and is skipped, never overwritten.
        base = timestamp + '-' + str(capture.metadata.run_id)
        attempt = 0
        while True:
            name = base if not attempt else base + '-' + str(attempt)
            destination = root/name
            try:
                destination.mkdir()
                break
            except FileExistsError:
                attempt += 1
        for name in payloads:
            # Hard links keep a retryable staging copy. Both directories are
            # created under the same parent/filesystem; link is exclusive.
            os.link(staging/name, destination/name)
        os.link(candidate, destination/MANIFEST)  # sole publication point, LAST
    except BaseException as error:
        error.add_note(f'owned evidence retained; unsealed staging: {staging}')
        if destination is not None:
            error.add_note(f'publication destination (readable only with manifest): {destination}')
        raise
    # Cleanup is best effort AFTER publication and cannot turn a sealed success
    # into a reported export failure. Delete only files this function owns.
    try:
        for name in (DATABASE, REPORT, 'sealed-manifest.json', 'UNSEALED'):
            (staging/name).unlink(missing_ok=True)
        staging.rmdir()
    except OSError:
        pass
    return destination
