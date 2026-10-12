"""Inspect a sealed run without constructing a simulator or importing a policy."""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import sys

from bike_sim.recording import analysis
from bike_sim.recording.json_codec import dumps
from bike_sim.recording.reader import open_recording
from bike_sim.recording.schema import quote


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    commands = result.add_subparsers(dest='command', required=True)
    for name in ('overview', 'fields', 'events', 'window', 'sql', 'export'):
        command = commands.add_parser(name)
        command.add_argument('run', type=Path)
        if name != 'export':
            command.add_argument('--limit', type=int, default=200)
            command.add_argument('--json', action='store_true')
        if name in ('events', 'window', 'export'):
            command.add_argument('--episode', type=int, required=True)
        if name == 'fields':
            command.add_argument('--pattern')
        if name == 'events':
            command.add_argument('--kind')
        if name == 'window':
            command.add_argument('--start', type=float, required=True)
            command.add_argument('--end', type=float, required=True)
            command.add_argument('--fields', required=True)
        if name == 'sql':
            command.add_argument('--query', required=True)
        if name == 'export':
            command.add_argument('--format', choices=('csv', 'jsonl'), required=True)
            command.add_argument('--output', type=Path, required=True)
            command.add_argument('--table', default='telemetry', choices=(
                'telemetry', 'legacy_telemetry', 'forces', 'commands_requested', 'commands_applied',
                'observations', 'transitions', 'events', 'metrics'))
            command.add_argument('--limit', type=int, default=None,
                                 help='optional explicit export row limit; default exports all rows')
    command = commands.add_parser('compare')
    command.add_argument('left', type=Path)
    command.add_argument('right', type=Path)
    command.add_argument('--left-episode', type=int, required=True)
    command.add_argument('--right-episode', type=int, required=True)
    command.add_argument('--mode', choices=('exact', 'summary'), default='exact')
    command.add_argument('--fields')
    command.add_argument('--atol', type=float, default=1e-9)
    command.add_argument('--rtol', type=float, default=1e-9)
    command.add_argument('--limit', type=int, default=200)
    command.add_argument('--json', action='store_true')
    return result


def export(reader, *, episode_id, format, output, table='telemetry', limit=None):
    reader.episode(episode_id)
    orders = {'telemetry': 'interval_id', 'legacy_telemetry': 'interval_id',
              'forces': 'interval_id,component', 'commands_requested': 'request_id',
              'commands_applied': 'application_id', 'observations': 'observation_id',
              'transitions': 'transition_id', 'events': 'event_id', 'metrics': 'name'}
    if table not in orders or format not in ('csv', 'jsonl'):
        raise ValueError('unsupported explicit recording export')
    parameters = [episode_id]
    sql = f'SELECT * FROM {quote(table)} WHERE episode_id=? ORDER BY {orders[table]}'
    if limit is not None:
        analysis._limit(limit)
        sql += ' LIMIT ?'
        parameters.append(limit)
    cursor = reader.connection.cursor()
    try:
        cursor.execute(sql, parameters)
        columns = [column[0] for column in cursor.description]
        # Exclusive creation: an export never overwrites another artifact.
        with Path(output).open('x', encoding='utf-8', newline='') as destination:
            writer = csv.writer(destination) if format == 'csv' else None
            if writer is not None:
                writer.writerow(columns)
            count = 0
            while rows := cursor.fetchmany(4096):
                for row in rows:
                    if writer is None:
                        destination.write(dumps(dict(zip(columns, row))) + '\n')
                    else:
                        writer.writerow([dumps(value) if value is None or isinstance(value, (dict, list, tuple))
                                         or isinstance(value, float) and not math.isfinite(value)
                                         else value for value in row])
                    count += 1
    finally:
        cursor.close()
    return dict(output=str(output), format=format, table=table, rows=count,
                semantics='explicit analytics export, not a checked-replay bundle')


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.command == 'compare':
            with open_recording(args.left) as left, open_recording(args.right) as right:
                result = analysis.compare(left, right, left_episode=args.left_episode,
                    right_episode=args.right_episode, mode=args.mode, fields=args.fields,
                    atol=args.atol, rtol=args.rtol, limit=args.limit)
        else:
            with open_recording(args.run) as reader:
                if args.command == 'overview':
                    result = analysis.overview(reader, limit=args.limit)
                elif args.command == 'fields':
                    result = analysis.fields(reader, pattern=args.pattern, limit=args.limit)
                elif args.command == 'events':
                    result = analysis.events(reader, episode_id=args.episode, kind=args.kind, limit=args.limit)
                elif args.command == 'window':
                    result = analysis.window(reader, episode_id=args.episode, start_s=args.start,
                                             end_s=args.end, fields=args.fields, limit=args.limit)
                elif args.command == 'sql':
                    result = reader.query(args.query, limit=args.limit)
                else:
                    result = export(reader, episode_id=args.episode, format=args.format,
                                    output=args.output, table=args.table, limit=args.limit)
        # Both presentations serialize the same typed tree. Nonfinite values
        # are tagged in either presentation, never changed into NULL.
        print(dumps(result, indent=None if getattr(args, 'json', False) else 2))
        return 0
    except Exception as error:
        print(f'RECORDING ERROR: {type(error).__name__}: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
