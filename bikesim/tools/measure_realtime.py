#!/usr/bin/env python3
"""Measure full physical Python/native boundaries using the profile's timestep.

This script is opt-in. It never configures or builds the native extension.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backend', choices=('python', 'native'), default='python')
    p.add_argument('--physics-config', type=Path, default=ROOT/'examples/research/viewer_physics_welded.toml')
    p.add_argument('--track', default=str(ROOT/'examples/research/rough_uphill_savage.toml'))
    p.add_argument('--duration', type=float, default=30.)
    p.add_argument('--dt', type=float, default=None, help='explicit override; absent means profile timestep')
    p.add_argument('--warmup', type=int, default=1)
    p.add_argument('--runs', type=int, default=3)
    p.add_argument('--record-decimation', '--decimate', dest='decimation', type=int, default=80)
    p.add_argument('--seed', type=int, default=None, help='generated-road seed override only')
    p.add_argument('--out', type=Path, required=True, help='new or empty directory')
    p.add_argument('--compare', type=Path, help='earlier report.json from the other backend')
    return p


def ride_arguments(args):
    from bike_sim.cli.ride import parse_args
    values = ['--backend', args.backend, '--physics-config', str(args.physics_config),
              '--track', args.track, '--headless', '--no-plots', '--duration', str(args.duration),
              '--decimate', str(args.decimation)]
    if args.dt is not None:
        values += ['--timestep', str(args.dt)]
    if args.seed is not None:
        values += ['--seed', str(args.seed)]
    return parse_args(values)


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    if args.runs < 1 or args.warmup < 0 or args.decimation < 1:
        p.error('--runs/--record-decimation must be positive and --warmup nonnegative')
    if not math.isfinite(args.duration) or args.duration <= 0.:
        p.error('--duration must be finite and positive')
    if args.dt is not None and (not math.isfinite(args.dt) or args.dt <= 0.):
        p.error('--dt must be finite and positive')
    try:
        if args.out.exists() and (not args.out.is_dir() or any(args.out.iterdir())):
            raise ValueError('--out must be a new or empty directory')
        previous = None if args.compare is None else json.loads(args.compare.read_text(encoding='utf-8'))
        ride = ride_arguments(args)
        from bike_sim.cli.ride import resolve_rider, resolve_track, track_seed
        from bike_sim.sim.backend import require_backend
        from bike_sim.sim.ride.physical_driver import build_physical_driver
        from bike_sim.sim.realtime_measurement import measure_run, build_report, compare_runs
        rider = resolve_rider(ride)
        require_backend(args.backend, ride.resolved_physics, rider)
        track = resolve_track(ride.track, seed=ride.seed, length_m=ride.length)
        seed = track_seed(ride.track, ride.seed)
        def factory():
            return build_physical_driver(track, ride, rider, seed=seed, strict=True)
        warmups, runs = [], []
        for phase, count, result in (('warmup', args.warmup, warmups), ('run', args.runs, runs)):
            for index in range(count):
                run = measure_run(factory, args.out/f'{phase}-{index:02d}')
                result.append(run)
                args.out.mkdir(parents=True, exist_ok=True)
                (args.out/f'{phase}-{index:02d}.json').write_text(
                    json.dumps(run, indent=2, sort_keys=True, allow_nan=False)+'\n', encoding='utf-8')
                print(f'{phase} {index+1}/{count}: backend={args.backend}; '
                      f'steps={run["physics_steps"]}; RTF={run["real_time_factor"]}; '
                      f'outcome={run["prefix"]["reason"]}; valid={run["valid_prefix"]}', flush=True)
        report = build_report(warmups, runs)
        report['invocation'] = dict(physics_config=str(args.physics_config.resolve()),
            track=args.track, duration_s=args.duration, dt_override=args.dt,
            effective_dt_s=ride.resolved_physics.timestep_s, record_decimation=args.decimation)
        if previous is not None:
            if previous.get('schema_version') != 1 or not previous.get('runs'):
                raise ValueError('--compare does not contain a measurement schema-1 run set')
            checks = [compare_runs(left, right) for left in previous['runs'] for right in runs]
            comparable = all(check['comparable'] for check in checks)
            prior = previous.get('median_real_time_factor')
            report['baseline_comparison'] = dict(checks=checks, comparable=comparable,
                median_speedup=(report['median_real_time_factor']/prior if comparable and prior and
                                report['median_real_time_factor'] is not None else None))
        (args.out/'report.json').write_text(
            json.dumps(report, indent=2, sort_keys=True, allow_nan=False)+'\n', encoding='utf-8')
        print(f'median RTF={report["median_real_time_factor"]}; report={args.out/"report.json"}')
        return 0 if all(run['measured_ok'] for run in warmups+runs) else 1
    except (ValueError, RuntimeError, ArithmeticError, OSError, ImportError, KeyError, TypeError) as error:
        print(f'MEASUREMENT ERROR: {type(error).__name__}: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
