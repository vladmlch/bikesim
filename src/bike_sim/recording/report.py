"""Deterministic evidence report, generated only from a closed database."""
from __future__ import annotations

from pathlib import Path

from bike_sim.recording.json_codec import dumps, loads
from bike_sim.recording.reader import _report_connection
from bike_sim.recording.schema import quote


class _Queries:
    def __init__(self, connection):
        self.connection = connection

    def rows(self, sql, parameters=()):
        result = self.connection.execute(sql, parameters)
        names = [column[0] for column in result.description]
        return [dict(zip(names, row)) for row in result.fetchall()]


def _value(value):
    if value is None:
        return 'unavailable / null'
    return dumps(value).replace('|', '\\|').replace('\n', ' ')


def render_report(reader):
    run = reader.rows('SELECT * FROM runs')[0]
    episodes = reader.rows('SELECT * FROM episodes ORDER BY episode_id')
    fields = reader.rows("SELECT * FROM field_catalog WHERE dtype='DOUBLE' AND "
                         "(table_name='physics' OR table_name LIKE 'channel_%') ORDER BY table_name,field_name")
    lines = ['# Run recording', '', f'Run UUID: `{run["run_id"]}`', '',
             f'Started (UTC): `{run["started_at"]}`. Backend: `{run["backend"]}`. Mode: `{run["mode"]}`.',
             '', '## Invocation and provenance', '',
             'Evidence: `runs.metadata_json`; per-episode setup identity is in `episodes.summary_json`.',
             '', '```json', dumps(loads(run['metadata_json']), indent=2), '```', '',
             '## Interpretation and limits', '',
             'This is saved numerical evidence, not a claim that the model is calibrated. '
             'Recording retains the launch in RAM and publishes at finalization; crashes, '
             'SIGKILL and out-of-memory termination have no durability guarantee. '
             'NULL, absence and nonfinite values are separate states. '
             'No cause is inferred from temporal coincidence.', '',
             'A forwarded endpoint measurement is not an instantaneous peak at every interval. '
             'The extrema below exclude forwarded energy rows; cumulative fields are reported '
             'as final values and are not summed. Missing/nonfinite coverage is reported explicitly.', '']
    if not episodes:
        lines += ['No episode was sealed. No physical or checked-replay claim is available.', '']
    for episode in episodes:
        identity = episode['episode_id']
        summary = loads(episode['summary_json'])
        extent = reader.rows('SELECT count(*) AS rows,min(interval_id) AS first_interval,max(interval_id) AS last_interval, '
                             'min(time_s) AS first_time_s,max(interval_end_s) AS last_time_s FROM physics WHERE episode_id=?', [identity])[0]
        lines += [f'## Episode {identity}', '',
                  f'Native generation: `{episode["generation"]}`. Physical prefix: `{_value(extent)}`.', '',
                  f'Completed control transitions: `{episode["complete_control_steps"]}`. '
                  f'Checked research replay eligible: `{episode["replay_eligible"]}` '
                  f'(contract `{episode["replay_contract_version"]}`).', '',
                  '### Outcome, validity and first failure', '',
                  'Evidence: `episodes.summary_json`, selected by `episode_id`.', '', '```json',
                  dumps({key: summary.get(key) for key in ('outcome', 'model_status', 'first_failure',
                        'failure', 'tail_failure', 'source_changed_during_run', 'execution_changed_during_run')}, indent=2),
                  '```', '', '### Important events', '',
                  'Source events keep their recorded classification. `derived_transition` events '
                  'describe retained channel changes under a named rule; they are not controller explanations.', '']
        events = reader.rows('SELECT event_id,kind,processed_step,applied_step,values_json '
                             'FROM events WHERE episode_id=? ORDER BY event_id LIMIT 201', [identity])
        lines += ['| Event | Kind | Processed step | Applied step | Recorded detail |',
                  '| --- | --- | --- | --- | --- |']
        for event in events[:200]:
            lines.append(f'| {event["event_id"]} | {event["kind"]} | {event["processed_step"]} | '
                         f'{_value(event["applied_step"])} | {_value(loads(event["values_json"]))} |')
        if not events:
            lines.append('| - | No source events recorded | - | - | - |')
        if len(events) > 200:
            lines += ['', 'Event display truncated at 200; remaining evidence is available with '
                      f'`SELECT * FROM events WHERE episode_id={identity} ORDER BY event_id`.']
        lines += ['', '### Scalar coverage and extrema', '',
                  '| Field | Unit / phase | Finite | Missing | Nonfinite | Minimum / final | Maximum |',
                  '| --- | --- | --- | --- | --- | --- | --- |']
        for field in fields:
            table, name = field['table_name'], field['field_name']
            domain = f'episode_id={identity}'
            if table == 'channel_energy' and field['measurement'] == 'forwarded':
                domain += ' AND measured IS TRUE'
            column = quote(name)
            sql = (f'SELECT count(*) AS n,count({column}) AS present, '
                   f'count(*) FILTER(WHERE isfinite({column})) AS finite, '
                   f'min({column}) FILTER(WHERE isfinite({column})) AS minimum, '
                   f'max({column}) FILTER(WHERE isfinite({column})) AS maximum '
                   f'FROM {quote(table)} WHERE {domain}')
            values = reader.rows(sql)[0]
            if field['phase'] == 'episode':
                endpoint = reader.rows(f'SELECT {column} AS value FROM {quote(table)} '
                                       f'WHERE {domain} ORDER BY interval_id DESC LIMIT 1')
                minimum = None if not endpoint else endpoint[0]['value']
                maximum = 'cumulative: not a peak'
            else:
                minimum, maximum = values['minimum'], values['maximum']
            lines.append(f'| `{table}.{name}` | {field["unit"]} / {field["phase"]} | '
                         f'{values["finite"]} | {values["n"]-values["present"]} | '
                         f'{values["present"]-values["finite"]} | {_value(minimum)} | {_value(maximum)} |')
        lines += ['', 'Reproduce each extrema row using the named table/field with '
                  f'`WHERE episode_id={identity}` and `isfinite(FIELD)`. '
                  'For `channel_energy` fields marked `forwarded` in `field_catalog`, '
                  'also require `measured IS TRUE`. For phase `episode`, select the '
                  'last `interval_id` instead of aggregating cumulative values.', '',
                  '### Recorded component power integrals', '',
                  'Rule `left_rectangle_recorded_power.v1`: sum of recorded `power_w * dt_s`. '
                  'This is a derived finite-subset integral when any power is missing/nonfinite; '
                  'it does not replace the engine\'s recorded cumulative accounting.', '',
                  '| Component | Intervals | Finite powers | Derived work (J) |',
                  '| --- | --- | --- | --- |']
        powers = reader.rows('SELECT f.component,count(*) AS n, '
            'count(*) FILTER(WHERE isfinite(f.power_w)) AS finite, '
            'sum(f.power_w*p.dt_s) FILTER(WHERE isfinite(f.power_w)) AS work_j '
            'FROM forces f JOIN physics p USING(episode_id,interval_id) WHERE episode_id=? '
            'GROUP BY f.component ORDER BY f.component', [identity])
        for row in powers:
            lines.append(f'| `{row["component"]}` | {row["n"]} | {row["finite"]} | {_value(row["work_j"])} |')
        if not powers:
            lines.append('| - | 0 | 0 | unavailable |')
        lines += ['', 'Evidence query: `forces JOIN physics USING(episode_id,interval_id)`, '
                  f'`episode_id={identity}`, grouped by `component`.', '',
                  '### Episode metrics and source identity', '', '```json',
                  dumps({'episode_metrics': summary.get('episode_metrics'),
                         'research': summary.get('research'),
                         'resolved_config': summary.get('resolved_config'),
                         'model_source_sha256': summary.get('model_source_sha256'),
                         'execution': summary.get('execution')}, indent=2), '```', '']
    lines += ['## Field and coordinate coverage', '',
              'The complete finite field inventory is `field_catalog`; model-address identities '
              'are `coordinate_catalog`. `_present` columns describe existence independently '
              'of nullable values. Parent `_valid` flags distinguish an explicitly null parent '
              'from an empty or absent parent. Structured children carry their own optionality flags.', '',
              'Diagnostics not represented by this catalog were not recorded and cannot be '
              'reconstructed as source facts. Observation delivery and acquisition timestamps '
              'are separate; transitions reference observations rather than duplicating them.', '']
    return '\n'.join(lines)


def write_summary(database, destination):
    with _report_connection(database) as connection:
        text = render_report(_Queries(connection))
    Path(destination).write_text(text, encoding='utf-8')
    return Path(destination)
