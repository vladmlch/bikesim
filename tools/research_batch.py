#!/usr/bin/env python3
"""Run a tracks x seeds x demand x policy grid and aggregate episode_metrics.

Batch TOML:

    [grid]
    tracks = ["examples/research/eval/eval_00.toml"]   # explicit track files, and/or
    scenarios = ["generated"]                          # research scenarios (seeded by `seeds`)
    seeds = [1, 2, 3]
    demand_nm = [60.0, 80.0]      # or demand_files = ["d.toml"]; omit for no demand
    policy = "passthrough"        # a name, or a list to make policy a grid axis
    transmission = "ideal_mid_drive"
    dt = 0.0005
    duration = 15.0
    extra_args = ["--rider-random"]   # appended verbatim to the bike-research argv
    ideal_sensors = true              # default; false adds sensor noise and latency (seed-dependent)

A failed run is a record with outcome 'error', never a crash of the batch; the
exit code is 1 if any run errored. outcome_counts keeps the model_violation and
numerical_quality truncations apart from crash:* / finish / duration, so runs
that left the model's scope are not read as policy failures.
"""
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import argparse
import csv
import hashlib
import itertools
import json
import re
import sys
import tomllib
import traceback

GRID_KEYS = {'tracks', 'scenarios', 'seeds', 'demand_nm', 'demand_files', 'policy', 'transmission',
             'dt', 'duration', 'extra_args', 'ideal_sensors'}
CSV_COLUMNS = ('run_id', 'track', 'scenario', 'seed', 'demand_nm', 'demand_file', 'policy', 'outcome',
               'duration_s', 'progress_m', 'mean_speed_mps', 'motor_pass_fraction', 'loop_out', 'endo',
               'wheelie_time_s', 'wheelie_episodes', 'front_load_fraction_min', 'max_energy_residual_ratio',
               'numerically_valid', 'model_valid', 'error')


def _listed(value, name):
    if isinstance(value, (str, int, float)) and not isinstance(value, bool):
        return [value]
    if not isinstance(value, list):
        raise ValueError(f'grid.{name} must be a value or a list')
    return value


def expand_grid(grid, policies=None):
    """Cartesian product of the grid axes as a list of run descriptors (stable run_ids)."""
    if set(grid)-GRID_KEYS:
        raise ValueError(f'unknown grid keys: {sorted(set(grid)-GRID_KEYS)}')
    if not grid.get('tracks') and not grid.get('scenarios'):
        raise ValueError('grid needs tracks and/or scenarios')
    sources = [(t, None) for t in grid.get('tracks', [])]+[(None, s) for s in grid.get('scenarios', [])]
    seeds = _listed(grid.get('seeds', [0]), 'seeds')
    demands = [(nm, None) for nm in _listed(grid['demand_nm'], 'demand_nm')] if 'demand_nm' in grid else []
    demands += [(None, f) for f in _listed(grid['demand_files'], 'demand_files')] if 'demand_files' in grid else []
    demands = demands or [(None, None)]
    policy_names = _listed(grid.get('policy', 'passthrough'), 'policy')
    known = policies if policies is not None else _policies()
    for name in policy_names:
        if name not in known and (not isinstance(name, str) or ':' not in name):
            raise ValueError(f'unknown policy {name!r}; choose from {sorted(known)}')
    ideal = grid.get('ideal_sensors', True)
    if type(ideal) is not bool:
        raise ValueError('grid.ideal_sensors must be true or false')
    runs = []
    for (track, scenario), seed, (demand, demand_file), policy in itertools.product(
            sources, seeds, demands, policy_names):
        key = json.dumps([track, scenario, seed, demand, demand_file, policy], sort_keys=True)
        stem = Path(track).stem if track else scenario
        label = f'{stem}_s{seed}_d{demand if demand is not None else demand_file or "none"}_{policy}'
        run_id = re.sub(r'[^A-Za-z0-9_.-]', '_', label).strip('._')
        runs.append(dict(run_id=f'{run_id}_{hashlib.sha256(key.encode()).hexdigest()[:6]}',
                         track=track, scenario=scenario, seed=seed, demand_nm=demand, demand_file=demand_file,
                         policy=policy, transmission=grid.get('transmission'), dt=grid.get('dt'),
                         duration=grid.get('duration'), ideal_sensors=ideal, extra_args=list(grid.get('extra_args', []))))
    return runs


def _policies():
    from bike_sim.sim.research.policies import POLICIES
    return POLICIES


def _argv(run, out):
    argv = ['--seed', str(run['seed']), '--out', str(out)]
    if run['ideal_sensors']:
        argv.append('--ideal-sensors')
    argv += ['--track-file', run['track']] if run['track'] else ['--scenario', run['scenario']]
    if run['demand_nm'] is not None:
        argv += ['--demand', str(run['demand_nm'])]
    if run['demand_file'] is not None:
        argv += ['--demand-file', run['demand_file']]
    for flag, key in (('--transmission', 'transmission'), ('--dt', 'dt'), ('--duration', 'duration')):
        if run[key] is not None:
            argv += [flag, str(run[key])]
    return argv+run['extra_args']


def run_one(payload):
    run, root = payload
    from bike_sim.cli.research import parser, make_environment
    from bike_sim.sim.research.metrics import episode_metrics
    from bike_sim.sim.research.policy_session import PolicySession, load_policy
    record = dict(run, outcome='error')
    session = None
    try:
        out = Path(root)/run['run_id']
        reference = run['policy'] if ':' in run['policy'] else (
            'bike_sim.sim.research.policies:' + run['policy'] + '_factory')
        policy = load_policy(reference)
        env = make_environment(parser().parse_args(_argv(run, out)))
        session = PolicySession(env, policy, reference=reference)
        while not env.done:
            session.advance()
        session.save(out)
        record.update(episode_metrics(env))
    except (Exception, SystemExit) as exc:  # argparse exits; a bad run must not kill the batch
        if session is not None and session.env.error is not None and not out.exists():
            session.save(out)
        record.update(error=f'{type(exc).__name__}: {exc}', traceback=traceback.format_exc())
    return record


def _row(record):
    wheelie = record.get('wheelie', {})
    row = {k: record.get(k) for k in CSV_COLUMNS}
    row.update(wheelie_time_s=wheelie.get('wheelie_time_s'), wheelie_episodes=wheelie.get('wheelie_episodes'),
               front_load_fraction_min=wheelie.get('front_load_fraction_min'),
               model_valid=record.get('model_status', {}).get('model_valid'))
    return row


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--spec', type=Path, required=True)
    p.add_argument('--jobs', type=int, default=1)
    p.add_argument('--out', type=Path, default=Path('output/batch'))
    args = p.parse_args(argv)
    if args.jobs < 1:
        p.error('--jobs must be positive')
    try:
        runs = expand_grid(tomllib.loads(args.spec.read_text()).get('grid', {}))
    except (ValueError, OSError, tomllib.TOMLDecodeError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 2
    args.out.mkdir(parents=True, exist_ok=True)
    work = [(run, str(args.out)) for run in runs]
    if args.jobs == 1:
        records = [run_one(item) for item in work]
    else:
        with ProcessPoolExecutor(max_workers=args.jobs) as pool:
            records = list(pool.map(run_one, work))
    counts = {}
    for record in records:
        counts[record['outcome']] = counts.get(record['outcome'], 0)+1
    report = dict(spec=str(args.spec), outcome_counts=dict(sorted(counts.items())), runs=records)
    (args.out/'batch_report.json').write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False, default=str)+'\n')
    with (args.out/'batch_report.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        writer.writerows(_row(r) for r in records)
    return 1 if counts.get('error') else 0


if __name__ == '__main__':
    raise SystemExit(main())
