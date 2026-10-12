"""Container schema 1 and its explicit typed Arrow bridge.

Research replay contract 3 is independent of this container version. Only
catalogued fields enter numeric streams; there is no inferred JSON fallback.
"""
from __future__ import annotations

from bike_sim.recording.catalog import FieldSpec, TableSpec, build_catalog
from bike_sim.recording.json_codec import thaw

SCHEMA_VERSION = 1
REPLAY_CONTRACT_VERSION = 3
BATCH_ROWS = 4096
DATABASE = 'run.duckdb'
MANIFEST = 'manifest.json'
REPORT = 'summary.md'


def quote(name):
    if not isinstance(name, str) or not name or '\x00' in name:
        raise ValueError('invalid SQL identifier')
    return '"' + name.replace('"', '""') + '"'


def _field(name, dtype='double', unit='1', *, optional=False,
           phase='control_period', measurement='measured', children=(), frame='n/a'):
    return FieldSpec(name, dtype, unit, phase, optional, measurement,
                     'sim/research/environment.py; native/src/runtime/research.cpp',
                     name.replace('_', ' '), children=children, frame=frame)


def research_tables():
    key = lambda name: _field(name, 'bigint', phase='n/a', measurement='declared')
    posture = _field('posture', 'struct', optional=True, children=(
        _field('torso_lean_rad', unit='rad'), _field('pelvis_pitch_rad', unit='rad'),
        _field('pelvis_offset_m', 'double[]', 'm', optional=True),
        _field('use_saddle', 'boolean')))
    control = _field('control', 'struct', measurement='declared', children=(
        _field('motor_torque_nm', unit='N*m', optional=True),
        _field('motor_limit_nm', unit='N*m', optional=True),
        _field('human_torque_nm', unit='N*m', optional=True),
        _field('crank_target_rate_rad_s', unit='rad/s', optional=True),
        posture, _field('rider_enabled', 'boolean')))
    command = (_field('time_s', unit='s'), key('step'), control)
    obs = (
        _field('time_s', unit='s'), _field('source_time_s', unit='s'),
        _field('valid', 'boolean'),
        _field('specific_force_body_mps2', 'double[]', 'm/s^2', frame='IMU body'),
        _field('pitch_rate_up_rad_s', unit='rad/s'),
        _field('front_wheel_rad_s', unit='rad/s'),
        _field('rear_wheel_rad_s', unit='rad/s'), _field('crank_rad_s', unit='rad/s'),
        _field('motor_torque_nm', unit='N*m'), _field('human_torque_nm', unit='N*m'))
    truth_units = dict(time_s='s', front_load_n='N', rear_load_n='N',
        front_clearance_m='m', rear_clearance_m='m', relative_pitch_rad='rad',
        position_m='m', speed_mps='m/s', pitch_up_rad='rad', pitch_rate_up_rad_s='rad/s',
        axle_pitch_rad='rad', road_pitch_rad='rad', front_slip_mps='m/s',
        rear_slip_mps='m/s', com_x_m='m', com_z_m='m')
    truth = _field('truth', 'struct', optional=True, children=tuple(
        _field(name, unit=unit, frame='simulator truth') for name, unit in truth_units.items()))
    def table(name, identity, fields):
        return TableSpec(name, 'physics', (key('episode_id'), key(identity)), tuple(fields),
                         name.replace('_', ' '), 'research owned history')
    return (
        table('commands_requested', 'request_id', command + (
            _field('front_brake_demand'), _field('rear_brake_demand'))),
        table('commands_applied', 'application_id', command + (
            _field('front_brake_demand', optional=True),
            _field('rear_brake_demand', optional=True),
            _field('rollback_brake_demand', optional=True))),
        table('observations', 'observation_id', obs),
        table('transitions', 'transition_id', (
            key('request_id'), key('previous_observation_id'), key('observation_id'), truth,
            _field('contact_state', 'varchar'), _field('terminated', 'boolean'),
            _field('truncated', 'boolean'), _field('reason', 'varchar', optional=True),
            key('physics_steps'), _field('numerically_valid', 'boolean'),
            _field('demand_nm', unit='N*m', optional=True), _field('model_valid', 'boolean'))),
    )


def all_tables():
    return build_catalog((), scalar_aliases={}).table_order() + research_tables()


def parent_masks(spec):
    parents = {'.'.join(f.path[:depth]) for f in spec.fields
               for depth in range(1, len(f.path))} | set(spec.claims)
    return tuple('__' + parent + suffix for parent in sorted(parents)
                 for suffix in ('_present', '_valid'))


def column_types(spec):
    result = {}
    for field in spec.keys + spec.fields:
        result[field.key] = field.duckdb_type()
        if field.optional:
            result[field.key + '_present'] = 'BOOLEAN'
    result.update((name, 'BOOLEAN') for name in parent_masks(spec))
    return result


def create_streams(connection):
    for spec in all_tables():
        fields = ', '.join(f'{quote(name)} {dtype}' for name, dtype in column_types(spec).items())
        primary = ', '.join(quote(f.key) for f in spec.keys)
        connection.execute(f'CREATE TABLE {quote(spec.name)} ({fields}, PRIMARY KEY ({primary}))')


def arrow_type(field, pa):
    simple = {'bigint': pa.int64(), 'double': pa.float64(), 'boolean': pa.bool_(),
              'varchar': pa.string()}
    if field.dtype in simple:
        return simple[field.dtype]
    if field.dtype in ('bigint[]', 'double[]', 'varchar[]'):
        return pa.list_(simple[field.dtype[:-2]])
    members = []
    if field.children:
        for child in field.children:
            members.append(pa.field(child.key, arrow_type(child, pa)))
            if child.optional:
                members.append(pa.field(child.key + '_present', pa.bool_()))
    struct = pa.struct(members)
    if field.dtype == 'struct':
        return struct
    if field.dtype == 'struct[]':
        return pa.list_(struct)
    if field.dtype.startswith('map<'):
        value = field.dtype[4:-1]
        child_type = struct if value == 'struct' else (
            pa.list_(pa.float64()) if value == 'double[]' else simple[value])
        return pa.map_(pa.string(), child_type)
    raise ValueError(f'unsupported Arrow catalog type: {field.dtype}')


def arrow_batch(spec, batch, episode_id, pa):
    expected = column_types(spec)
    unknown = set(batch.columns) - set(expected)
    if unknown:
        raise ValueError(f'{spec.name}: unreviewed columns {sorted(unknown)}')
    missing = set(expected) - set(batch.columns) - {'episode_id'}
    if batch.row_count and missing:
        raise ValueError(f'{spec.name}: missing declared columns {sorted(missing)}')
    if batch.row_count and any(field.key not in batch.columns
                               for field in spec.keys if field.key != 'episode_id'):
        raise ValueError(f'{spec.name}: missing identity column')
    by_name = {field.key: field for field in spec.keys + spec.fields}
    arrays, names = [], []
    for name in expected:
        values = ([episode_id] * batch.row_count if name == 'episode_id' else
                  [thaw(v) for v in batch.columns.get(name, [None] * batch.row_count)])
        dtype = arrow_type(by_name[name], pa) if name in by_name else pa.bool_()
        arrays.append(pa.array(values, type=dtype, from_pandas=False, safe=True))
        names.append(name)
    return pa.RecordBatch.from_arrays(arrays, names=names)


def create_metadata(connection):
    definitions = {
        'recording_meta': 'schema_version INTEGER NOT NULL, run_id UUID NOT NULL, codec VARCHAR NOT NULL',
        'runs': 'run_id UUID PRIMARY KEY, started_at TIMESTAMPTZ NOT NULL, backend VARCHAR NOT NULL, mode VARCHAR NOT NULL, metadata_json VARCHAR NOT NULL',
        'episodes': 'episode_id BIGINT PRIMARY KEY, generation BIGINT NOT NULL, physical_rows BIGINT NOT NULL, complete_control_steps BIGINT NOT NULL, replay_contract_version INTEGER, replay_eligible BOOLEAN NOT NULL, summary_json VARCHAR NOT NULL',
        'events': 'episode_id BIGINT NOT NULL, event_id BIGINT NOT NULL, kind VARCHAR NOT NULL, received_wall_s DOUBLE NOT NULL, processed_step BIGINT NOT NULL, applied_step BIGINT, values_json VARCHAR NOT NULL, PRIMARY KEY(episode_id,event_id)',
        'field_catalog': 'table_name VARCHAR NOT NULL, field_name VARCHAR NOT NULL, dtype VARCHAR NOT NULL, unit VARCHAR NOT NULL, phase VARCHAR NOT NULL, optional BOOLEAN NOT NULL, measurement VARCHAR NOT NULL, source VARCHAR NOT NULL, frame VARCHAR NOT NULL, definition_json VARCHAR NOT NULL, PRIMARY KEY(table_name,field_name)',
        'coordinate_catalog': 'episode_id BIGINT NOT NULL, kind VARCHAR NOT NULL, coordinate_index BIGINT NOT NULL, joint VARCHAR NOT NULL, component VARCHAR NOT NULL, dof_kind VARCHAR NOT NULL, unit VARCHAR NOT NULL, PRIMARY KEY(episode_id,kind,coordinate_index)',
        'alias_catalog': 'episode_id BIGINT NOT NULL, alias VARCHAR NOT NULL, canonical VARCHAR NOT NULL, vector_index BIGINT, parameter VARCHAR, note VARCHAR NOT NULL, PRIMARY KEY(episode_id,alias)',
        'assets': 'episode_id BIGINT NOT NULL, name VARCHAR NOT NULL, media_type VARCHAR NOT NULL, sha256 VARCHAR NOT NULL, content BLOB NOT NULL, PRIMARY KEY(episode_id,name)',
        'integration_states': 'episode_id BIGINT NOT NULL, phase VARCHAR NOT NULL, signature VARCHAR NOT NULL, state DOUBLE[] NOT NULL, PRIMARY KEY(episode_id,phase)',
        'metrics': 'episode_id BIGINT NOT NULL, name VARCHAR NOT NULL, kind VARCHAR NOT NULL, double_value DOUBLE, bigint_value BIGINT, boolean_value BOOLEAN, text_value VARCHAR, list_value DOUBLE[], json_value VARCHAR, PRIMARY KEY(episode_id,name)',
    }
    for name, definition in definitions.items():
        connection.execute(f'CREATE TABLE {quote(name)} ({definition})')


def create_views(connection):
    # Only one row per physical interval; forces are intentionally not joined
    # here because doing so would multiply interval rows by component count.
    physical = build_catalog((), scalar_aliases={}).table_order()
    projections = ['p.episode_id', 'p.interval_id']
    joins = []
    for index, spec in enumerate(physical):
        if spec.name == 'forces':
            continue
        alias = 'p' if spec.name == 'physics' else f'c{index}'
        if alias != 'p':
            joins.append(f'LEFT JOIN {quote(spec.name)} {alias} USING (episode_id, interval_id)')
        for field in spec.keys + spec.fields:
            projections.append(f'{alias}.{quote(field.key)} AS {quote(spec.name + "." + field.key)}')
            if field.optional:
                projections.append(f'{alias}.{quote(field.key + "_present")} AS {quote(spec.name + "." + field.key + "_present")}')
        for mask in parent_masks(spec):
            projections.append(f'{alias}.{quote(mask)} AS {quote(spec.name + "." + mask)}')
    connection.execute('CREATE VIEW telemetry AS SELECT ' + ', '.join(projections) +
                       ' FROM physics p ' + ' '.join(joins))
    connection.execute('CREATE VIEW legacy_intervals AS SELECT episode_id, interval_id, time_s, '
                       'interval_end_s AS end_time_s, dt_s, qpos, qvel FROM physics')
    connection.execute('CREATE VIEW legacy_observations AS SELECT * EXCLUDE(episode_id, observation_id) FROM observations')


def create_legacy_view(connection):
    aliases = connection.execute('SELECT episode_id,alias,canonical,vector_index,parameter '
                                 'FROM alias_catalog ORDER BY alias,episode_id').fetchall()
    expressions = {}
    for episode, name, canonical, index, parameter in aliases:
        if '{' in name or parameter is not None or canonical.startswith('forces.'):
            continue
        if canonical.count('.') != 1:
            continue
        expression = quote(canonical)
        if index is not None:
            expression = f'({expression})[{index + 1}]'
        expressions.setdefault(name, []).append(f'WHEN episode_id={episode} THEN {expression}')
    columns = ['episode_id', 'interval_id']
    for name, cases in expressions.items():
        columns.append('CASE ' + ' '.join(cases) + ' END AS ' + quote(name))
    connection.execute('CREATE VIEW legacy_telemetry AS SELECT ' + ', '.join(columns) + ' FROM telemetry')
