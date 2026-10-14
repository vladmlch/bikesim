"""Typed, bounded analysis over saved evidence. No simulator construction."""
from __future__ import annotations

from collections.abc import Mapping
from itertools import zip_longest
import math
import re

from bike_sim.recording.json_codec import loads
from bike_sim.recording.schema import quote

DEFAULT_FIELDS = ('physics.time_s', 'physics.interval_end_s', 'physics.qpos',
                  'physics.qvel', 'channel_energy.residual_j',
                  'channel_model_status.model_status')
_PROVENANCE = ('channel_energy.measured', 'channel_energy.source_interval_id',
               'channel_energy.source_time_s', 'channel_mass.measurement_scope')


def _limit(value):
    if type(value) is not int or value < 1:
        raise ValueError('limit must be a positive integer')
    return value


def _bounded(rows, limit=200, *, total=None):
    _limit(limit)
    return dict(rows=rows[:limit], limit=limit, truncated=len(rows) > limit,
                has_more=len(rows) > limit, **({} if total is None else {'total': total}))


def overview(reader, *, limit=200):
    _limit(limit)
    run = reader.rows('SELECT * FROM runs')[0]
    run['metadata'] = loads(run.pop('metadata_json'))
    rows = reader.rows('SELECT e.*, min(p.time_s) AS first_time_s, max(p.interval_end_s) AS last_time_s '
                      'FROM episodes e LEFT JOIN physics p USING(episode_id) GROUP BY ALL '
                      'ORDER BY e.episode_id LIMIT ?', [limit + 1])
    for row in rows:
        row['summary'] = loads(row.pop('summary_json'))
    total = reader.connection.execute('SELECT count(*) FROM episodes').fetchone()[0]
    return dict(run=run, manifest=reader.manifest,
                episodes=_bounded(rows, limit, total=total),
                limitations=['Full launch retained in RAM; no crash-recovery promise.',
                             'Ordinary recipes are not checked research replay.',
                             'Model/numerical validity is not calibration evidence.'])


def fields(reader, *, pattern=None, limit=200):
    _limit(limit)
    condition, parameters = '', []
    if pattern is not None:
        if not isinstance(pattern, str):
            raise ValueError('field pattern must be a string')
        # Shell-style '*' is convenient; bound LIKE data never becomes SQL.
        value = pattern.replace('\\', '\\\\').replace('%', '\\%').replace('_', '\\_').replace('*', '%')
        if '*' not in pattern:
            value = '%' + value + '%'
        condition = " WHERE (table_name || '.' || field_name) LIKE ? ESCAPE '\\'"
        parameters.append(value)
    total = reader.connection.execute('SELECT count(*) FROM field_catalog' + condition, parameters).fetchone()[0]
    rows = reader.rows('SELECT * FROM field_catalog' + condition +
                      ' ORDER BY table_name,field_name LIMIT ?', parameters + [limit + 1])
    for row in rows:
        row['canonical'] = row['table_name'] + '.' + row['field_name']
        row['definition'] = loads(row.pop('definition_json'))
    return _bounded(rows, limit, total=total)


def events(reader, *, episode_id, kind=None, limit=200):
    _limit(limit)
    reader.episode(episode_id)
    condition = 'e.episode_id=?'
    parameters = [episode_id]
    if kind is not None:
        condition += ' AND e.kind=?'
        parameters.append(kind)
    total = reader.connection.execute('SELECT count(*) FROM events e WHERE ' + condition, parameters).fetchone()[0]
    rows = reader.rows('SELECT e.*, p.time_s AS interval_time_s FROM events e '
        'LEFT JOIN physics p ON p.episode_id=e.episode_id AND p.interval_id=e.processed_step '
        'WHERE ' + condition + ' ORDER BY e.event_id LIMIT ?', parameters + [limit + 1])
    for row in rows:
        row['values'] = loads(row.pop('values_json'))
    return dict(episode_id=episode_id, **_bounded(rows, limit, total=total))


def _literal(value):
    return "'" + value.replace("'", "''") + "'"


def _selection(reader, episode_id, requested, *, companions=True):
    catalog = {row['table_name'] + '.' + row['field_name']: row
               for row in reader.rows('SELECT * FROM field_catalog')}
    aliases = reader.rows('SELECT * FROM alias_catalog WHERE episode_id=? ORDER BY alias', [episode_id])
    if requested is None:
        requested = DEFAULT_FIELDS
    elif isinstance(requested, str):
        requested = tuple(name.strip() for name in requested.split(','))
    if not requested or any(not isinstance(name, str) or not name for name in requested):
        raise ValueError('fields must contain nonempty catalog names')
    selections, labels = [], set()
    for label in requested:
        if label in labels:
            continue
        canonical, index, parameter = label, None, None
        if canonical not in catalog:
            alias = next((a for a in aliases if a['alias'] == label), None)
            if alias is None:
                for candidate in aliases:
                    if '{component}' not in candidate['alias']:
                        continue
                    pattern = re.escape(candidate['alias']).replace(re.escape('{component}'), '(.+)')
                    match = re.fullmatch(pattern, label)
                    if match:
                        alias = dict(candidate, parameter=match.group(1))
                        break
            if alias is not None:
                canonical, index, parameter = alias['canonical'], alias['vector_index'], alias['parameter']
        root = next((candidate for candidate in sorted(catalog, key=len, reverse=True)
                     if canonical == candidate or canonical.startswith(candidate + '.')), None)
        if root is None:
            raise ValueError(f'unknown recording field: {label}')
        field = catalog[root]
        table, name = field['table_name'], field['field_name']
        if table == 'forces':
            if name not in ('qfrc', 'power_w'):
                raise ValueError('select forces.qfrc or forces.power_w; use sql for component identities')
            if parameter is None:
                expression = ('(SELECT map(list(f.component ORDER BY f.component), '
                    f'list(f.{quote(name)} ORDER BY f.component)) FROM forces f '
                    'WHERE f.episode_id=telemetry.episode_id AND f.interval_id=telemetry.interval_id)')
            else:
                expression = (f'(SELECT f.{quote(name)} FROM forces f WHERE '
                    'f.episode_id=telemetry.episode_id AND f.interval_id=telemetry.interval_id '
                    f'AND f.component={_literal(parameter)})')
        elif table == 'physics' or table.startswith('channel_'):
            expression = quote(root)
        else:
            raise ValueError(f'{label} is not an interval field; query {table} using sql')
        definition = loads(field['definition_json'])
        remainder = canonical[len(root):].lstrip('.')
        if remainder:
            current = definition
            for member in remainder.split('.'):
                if current.get('dtype', '').startswith('map<'):
                    expression = f'({expression})[{_literal(member)}]'
                    current = dict(current, dtype=current['dtype'][4:-1])
                else:
                    child = next((c for c in current.get('children', []) if c['key'] == member), None)
                    if child is None:
                        raise ValueError(f'unknown structured recording field: {canonical}')
                    expression = f'({expression})[{_literal(member)}]'
                    current = child
        elif parameter is not None and table != 'forces':
            if not definition.get('dtype', '').startswith('map<'):
                raise ValueError(f'{label} is not a map member')
            expression = f'({expression})[{_literal(parameter)}]'
        if index is not None:
            expression = f'({expression})[{int(index) + 1}]'
        selections.append(dict(label=label, canonical=canonical, root=root, expression=expression,
                               index=index, parameter=parameter, definition=field))
        labels.add(label)
        if companions:
            masks = [root + '_present']
            path = definition.get('path', [])
            masks += [table + '.__' + '.'.join(path[:depth]) + suffix
                      for depth in range(1, len(path)) for suffix in ('_present', '_valid')]
            if path:
                masks += [table + '.__' + path[0] + suffix for suffix in ('_present', '_valid')]
            for mask in masks:
                if mask in catalog and mask not in labels:
                    selections.append(dict(label=mask, canonical=mask, root=mask,
                        expression=quote(mask), index=None, parameter=None, definition=catalog[mask]))
                    labels.add(mask)
    return selections


def _projection(selections):
    return 'episode_id, interval_id, ' + ', '.join(
        selection['expression'] + ' AS ' + quote(selection['label']) for selection in selections)


def window(reader, *, episode_id, start_s, end_s, fields=None, limit=200):
    _limit(limit)
    reader.episode(episode_id)
    if type(start_s) not in (int, float) or type(end_s) not in (int, float) \
            or not math.isfinite(start_s) or not math.isfinite(end_s) or end_s <= start_s:
        raise ValueError('window requires finite start < end')
    selections = _selection(reader, episode_id, fields)
    condition = 'episode_id=? AND "physics.time_s">=? AND "physics.time_s"<?'
    parameters = [episode_id, start_s, end_s]
    total = reader.connection.execute('SELECT count(*) FROM telemetry WHERE ' + condition, parameters).fetchone()[0]
    rows = reader.rows('SELECT ' + _projection(selections) +
        ', "physics.interval_end_s">? AS overlaps_end FROM telemetry WHERE ' + condition +
        ' ORDER BY interval_id LIMIT ?', [end_s] + parameters + [limit + 1])
    overlap = reader.rows('SELECT episode_id,interval_id,time_s,interval_end_s FROM physics '
        'WHERE episode_id=? AND time_s<? AND interval_end_s>? ORDER BY interval_id LIMIT ?',
        [episode_id, start_s, start_s, limit + 1])
    return dict(episode_id=episode_id, start_s=start_s, end_s=end_s,
                selection='interval start in [start_s,end_s); no interpolation',
                fields=[dict(name=s['label'], canonical=s['canonical']) for s in selections],
                overlap_before_start=_bounded(overlap, limit),
                **_bounded(rows, limit, total=total))


def _iter_rows(reader, sql, parameters):
    cursor = reader.connection.cursor()
    try:
        cursor.execute(sql, parameters)
        names = [column[0] for column in cursor.description]
        while values := cursor.fetchmany(4096):
            for value in values:
                yield dict(zip(names, value))
    finally:
        cursor.close()


def _difference(left, right, path, *, atol, rtol, stats):
    """First differing leaf, preserving null, category and numeric type."""
    if left is None or right is None:
        return None if left is right else dict(path=path, kind='null_or_presence', left=left, right=right)
    if isinstance(left, Mapping) and isinstance(right, Mapping):
        first = None
        for key in sorted(set(left) | set(right)):
            if key not in left or key not in right:
                if first is None:
                    first = dict(path=path + '.' + str(key), kind='presence',
                                 left_present=key in left, right_present=key in right)
                continue
            difference = _difference(left[key], right[key], path + '.' + str(key),
                                     atol=atol, rtol=rtol, stats=stats)
            if first is None and difference is not None:
                first = difference
        return first
    if isinstance(left, (tuple, list)) and isinstance(right, (tuple, list)):
        if len(left) != len(right):
            return dict(path=path, kind='vector_width', left=len(left), right=len(right))
        first = None
        for index, (a, b) in enumerate(zip(left, right)):
            difference = _difference(a, b, f'{path}[{index}]', atol=atol, rtol=rtol, stats=stats)
            if first is None and difference is not None:
                first = difference
        return first
    if type(left) is not type(right):
        return dict(path=path, kind='type', left=left, right=right)
    if isinstance(left, float):
        if not math.isfinite(left) or not math.isfinite(right):
            stats['nonfinite_pairs'] += 1
            agrees = (math.isnan(left) and math.isnan(right)) or left == right
            if agrees:
                stats['matching_invalid_pairs'] += 1
                return None
            return dict(path=path, kind='nonfinite', left=left, right=right)
        absolute = abs(left - right)
        scale = max(abs(left), abs(right))
        relative = absolute / scale if scale else 0.
        stats['max_absolute_error'] = max(stats['max_absolute_error'], absolute)
        stats['max_relative_error'] = max(stats['max_relative_error'], relative)
        if left == right == 0. and math.copysign(1., left) != math.copysign(1., right):
            stats['signed_zero_differences'] += 1
        if absolute <= atol + rtol * scale:
            return None
        return dict(path=path, kind='float', left=left, right=right,
                    absolute_error=absolute, relative_error=relative)
    if left != right:
        return dict(path=path, kind='category', left=left, right=right)
    return None


def _statistics():
    return dict(max_absolute_error=0., max_relative_error=0., nonfinite_pairs=0,
                matching_invalid_pairs=0, signed_zero_differences=0)


def _summary(reader, episode_id, *, limit):
    episode = reader.episode(episode_id)
    summary = loads(episode['summary_json'])
    event_groups = reader.rows('SELECT e.kind,count(*) AS count,min(p.time_s) AS first_time_s, '
        'max(p.time_s) AS last_time_s FROM events e LEFT JOIN physics p '
        'ON e.episode_id=p.episode_id AND e.processed_step=p.interval_id '
        'WHERE e.episode_id=? GROUP BY e.kind ORDER BY e.kind LIMIT ?', [episode_id, limit + 1])
    # Event windows are explicitly bounded; no coordinate/distance alignment.
    anchors = reader.rows('SELECT e.event_id,e.kind,p.time_s,p.dt_s FROM events e '
        'JOIN physics p ON e.episode_id=p.episode_id AND e.processed_step=p.interval_id '
        "WHERE e.episode_id=? AND e.kind IN ('model_first_failure','derived_transition') "
        'ORDER BY e.event_id LIMIT ?', [episode_id, min(limit, 20) + 1])
    windows = [dict(event_id=e['event_id'], kind=e['kind'], data=window(reader,
        episode_id=episode_id, start_s=e['time_s'] - e['dt_s'],
        end_s=e['time_s'] + 2 * e['dt_s'], limit=min(limit, 10)))
        for e in anchors[:min(limit, 20)]]
    return dict(outcome=summary.get('outcome'), model_status=summary.get('model_status'),
                first_failure=summary.get('first_failure'), metrics=summary.get('episode_metrics', {}),
                resolved_config=summary.get('resolved_config'),
                physical_rows=episode['physical_rows'], complete_control_steps=episode['complete_control_steps'],
                replay_eligible=episode['replay_eligible'], events=_bounded(event_groups, limit),
                event_windows=_bounded(windows, min(limit, 20)),
                event_windows_truncated=len(anchors) > min(limit, 20))


def compare(left, right, *, left_episode, right_episode, mode='exact', fields=None,
            atol=1e-9, rtol=1e-9, limit=200):
    _limit(limit)
    for name, value in (('atol', atol), ('rtol', rtol)):
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            raise ValueError(f'{name} must be finite and nonnegative')
    lmeta, rmeta = left.episode(left_episode), right.episode(right_episode)
    base = dict(mode=mode, left_episode=left_episode, right_episode=right_episode,
                atol=atol, rtol=rtol, tolerance_rule='abs(a-b) <= atol + rtol*max(abs(a),abs(b))',
                alignment='none')
    if mode == 'summary':
        return dict(base, left=_summary(left, left_episode, limit=limit),
                    right=_summary(right, right_episode, limit=limit),
                    interpretation='Compare outcome/metrics/event times and saved surrounding windows; no automatic alignment.')
    if mode != 'exact':
        raise ValueError('comparison mode must be exact or summary')
    requested = list(DEFAULT_FIELDS if fields is None else
                     (fields.split(',') if isinstance(fields, str) else fields))
    requested += list(_PROVENANCE)
    lfields, rfields = _selection(left, left_episode, requested), _selection(right, right_episode, requested)
    incompatible = []
    for a, b in zip(lfields, rfields):
        identity = ('canonical', 'index', 'parameter')
        semantics = ('dtype', 'unit', 'phase', 'measurement', 'frame', 'source', 'definition_json')
        if any(a[key] != b[key] for key in identity) or any(a['definition'][key] != b['definition'][key] for key in semantics):
            incompatible.append(dict(field=a['label'], reason='field_semantics'))
    if len(lfields) != len(rfields):
        incompatible.append(dict(reason='field_or_presence_schema'))
    coordinates = 'SELECT kind,coordinate_index,joint,component,dof_kind,unit FROM coordinate_catalog WHERE episode_id=? ORDER BY kind,coordinate_index'
    if left.rows(coordinates, [left_episode]) != right.rows(coordinates, [right_episode]):
        incompatible.append(dict(reason='coordinate_identity'))
    validity = dict(left=loads(lmeta['summary_json']).get('model_status'),
                    right=loads(rmeta['summary_json']).get('model_status'))
    if incompatible:
        return dict(base, compatible=False, agrees=False, incompatibilities=incompatible,
                    validity=validity, compared_intervals=0)
    # Clocks must match exactly even when values are compared with tolerances.
    clock = ', "physics.time_s" AS __start, "physics.interval_end_s" AS __end, "physics.dt_s" AS __dt'
    query = lambda selection: ('SELECT ' + _projection(selection) + clock +
                                ' FROM telemetry WHERE episode_id=? ORDER BY interval_id')
    lrows = _iter_rows(left, query(lfields), [left_episode])
    rrows = _iter_rows(right, query(rfields), [right_episode])
    stats = {field['label']: _statistics() for field in lfields}
    compared, mismatched = 0, 0
    first, unmatched = None, None
    try:
        for a, b in zip_longest(lrows, rrows):
            if a is None or b is None:
                unmatched = dict(left_first_interval=None if a is None else a['interval_id'],
                                 right_first_interval=None if b is None else b['interval_id'])
                break
            if any(a[key] != b[key] for key in ('interval_id', '__start', '__end', '__dt')):
                incompatible.append(dict(reason='time_grid', left_interval=a['interval_id'],
                    right_interval=b['interval_id'], left_time_s=a['__start'], right_time_s=b['__start']))
                break
            if any(a.get(name) != b.get(name) for name in _PROVENANCE):
                incompatible.append(dict(reason='measurement_provenance', interval_id=a['interval_id']))
                break
            compared += 1
            for field in lfields:
                name = field['label']
                difference = _difference(a[name], b[name], name, atol=atol, rtol=rtol, stats=stats[name])
                if difference is not None:
                    mismatched += 1
                    if first is None:
                        first = dict(interval_id=a['interval_id'], time_s=a['__start'], **difference)
    finally:
        lrows.close()
        rrows.close()
    deviations = [dict(field=name, **value) for name, value in stats.items()]
    invalid_pairs = sum(row['matching_invalid_pairs'] for row in deviations)
    return dict(base, compatible=not incompatible, agrees=not incompatible and first is None and unmatched is None,
        first_mismatch=first, mismatched_field_rows=mismatched, compared_intervals=compared,
        incompatibilities=incompatible, unmatched_extents=dict(
            left_intervals=lmeta['physical_rows'] - compared,
            right_intervals=rmeta['physical_rows'] - compared, first=unmatched),
        deviations=_bounded(deviations, limit, total=len(deviations)), validity=validity,
        matching_invalid_pairs=invalid_pairs,
        interpretation=('Matching nonfinite values are agreeing INVALID evidence, not a valid physical result.'
                        if invalid_pairs else 'Agreement concerns saved fields, not physical calibration.'))
