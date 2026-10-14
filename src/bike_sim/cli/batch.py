"""Parallel research episodes: expand a TOML spec into bike-research jobs and run them in a process pool.

Spec shape:

    [base]                      # any bike-research long flag; merged into every job
    physics-config = "examples/research/viewer_physics_welded.toml"
    duration = 10

    [grid]                      # cartesian product; a list per flag, or {start, stop, step}/{start, count}
    seed = {start = 0, count = 12}
    rider-mass = [60.0, 80.0]

    [[job]]                     # optional explicit jobs: base merged with these overrides
    seed = 7
    rider-random = true

Relative paths in flag values resolve against the directory bike-batch runs in
(workers inherit the parent's cwd), not against the spec file.
"""
import argparse
import contextlib
import io
import itertools
import json
import multiprocessing as mp
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import tomllib
from concurrent.futures import ProcessPoolExecutor, as_completed
from concurrent.futures.process import BrokenProcessPool
from dataclasses import dataclass

RESERVED_KEYS = {'out'}  # batch assigns --out itself


@dataclass(frozen=True)
class Job:
    index: int
    name: str
    params: dict  # merged flag values actually passed
    argv: list


def _flag_tokens(key, value):
    if key in RESERVED_KEYS:
        raise ValueError(f"'{key}' is assigned by bike-batch; remove it from the spec")
    if isinstance(value, bool):
        return [f'--{key}'] if value else []
    if isinstance(value, (int, float, str)):
        return [f'--{key}', str(value)]
    raise ValueError(f"flag '{key}': unsupported value {value!r} "
                     '(want bool, int, float or str; lists belong in [grid])')


def _grid_values(key, raw):
    if isinstance(raw, dict):
        keys = set(raw)
        if keys == {'start', 'count'}:
            return [raw['start'] + i for i in range(raw['count'])]
        if keys <= {'start', 'stop', 'step'} and 'stop' in keys:
            return list(range(raw['start'], raw['stop'], raw.get('step', 1)))
        raise ValueError(f"grid.{key}: range table wants {{start,count}} or {{start,stop[,step]}}")
    if isinstance(raw, list):
        if not raw:
            raise ValueError(f'grid.{key}: empty list')
        for v in raw:
            if isinstance(v, (dict, list)):
                raise ValueError(f'grid.{key}: entries must be scalar, got {v!r}')
        return raw
    raise ValueError(f'grid.{key} must be a list or a range table')


def _slug(overrides):
    parts = []
    for k, v in overrides.items():
        text = k if v is True else f'no-{k}' if v is False else f'{k}{v}'
        parts.append(re.sub(r'[^A-Za-z0-9]+', '-', text).strip('-'))
    return '-'.join(p for p in parts if p)[:48]


def _job_argv(params):
    argv = []
    for k, v in params.items():
        argv += _flag_tokens(k, v)
    return argv


def expand_spec(spec):
    """TOML dict -> [Job]. Jobs = base x grid, then one job per [[job]] (base merged)."""
    unknown = set(spec) - {'base', 'grid', 'job'}
    if unknown:
        raise ValueError(f'unknown spec sections: {sorted(unknown)} (want base/grid/job)')
    base = spec.get('base', {})
    grid = spec.get('grid', {})
    explicit = spec.get('job', [])
    if not isinstance(base, dict) or not isinstance(grid, dict) or not isinstance(explicit, list):
        raise ValueError('spec wants [base] table, [grid] table, [[job]] array of tables')
    grid_keys = list(grid)
    grid_values = [_grid_values(k, grid[k]) for k in grid_keys]
    # jobs = base x grid when a grid exists; [[job]] entries each merge over base.
    # With neither, the spec is a single base episode.
    merged = ([dict(base, **dict(zip(grid_keys, combo)))
               for combo in itertools.product(*grid_values)] if grid
              else [] if explicit else [dict(base)])
    for entry in explicit:
        if not isinstance(entry, dict):
            raise ValueError('[[job]] entries must be tables')
        merged.append({**base, **entry})
    jobs = []
    for i, params in enumerate(merged):
        overrides = {k: v for k, v in params.items() if k not in base or base[k] != v}
        slug = _slug(overrides)
        jobs.append(Job(index=i, name=f'{i:04d}-{slug}' if slug else f'{i:04d}',
                        params=params, argv=_job_argv(params)))
    return jobs


def _validate_argv(research_parser, jobs):
    """Every job's flags must survive bike-research's own argparse before any worker spawns."""
    for job in jobs:
        try:
            with contextlib.redirect_stderr(io.StringIO()):
                research_parser.parse_args(job.argv)
        except SystemExit:
            raise ValueError(f'job {job.name}: bike-research rejects argv {job.argv}') from None


def _tail_json(text):
    """research.main prints one indented JSON block at the end; recover it."""
    start = text.rfind('\n{')
    if start == -1:
        start = 0 if text.startswith('{') else -1
    if start == -1:
        return None
    try:
        return json.loads(text[start:].strip())
    except json.JSONDecodeError:
        return None


def _run_job(task):
    """Worker: run one episode in-process. Exceptions become results, never propagate."""
    from bike_sim.cli import research
    index, name, argv, out_dir = task
    buf_out, buf_err = io.StringIO(), io.StringIO()
    t0 = time.perf_counter()
    try:
        with contextlib.redirect_stdout(buf_out), contextlib.redirect_stderr(buf_err):
            code = research.main([*argv, '--out', out_dir])
    except (Exception, SystemExit) as exc:
        code, error = -1, f'{type(exc).__name__}: {exc}'
    else:
        error = None
    result = {'index': index, 'name': name, 'exit': code,
              'wall_s': round(time.perf_counter() - t0, 3), 'out': out_dir}
    if error is not None:
        result['error'] = error
    report = _tail_json(buf_out.getvalue().strip())
    if report:
        result['outcome'] = report.get('outcome')
        result['metrics'] = report.get('metrics')
    stderr_tail = buf_err.getvalue().strip()
    if stderr_tail and code != 0:
        result['stderr_tail'] = stderr_tail[-400:]
    return result


def _performance_cores():
    """P-core count on Apple Silicon; fall back to all logical CPUs elsewhere."""
    try:
        out = subprocess.run(['sysctl', '-n', 'hw.perflevel0.physicalcpu'],
                             capture_output=True, text=True, timeout=5)
        if out.returncode == 0:
            return max(1, int(out.stdout.strip()))
    except (OSError, ValueError, subprocess.TimeoutExpired):
        pass
    return os.process_cpu_count() or os.cpu_count() or 1


def parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--spec', type=Path, required=True, help='TOML spec: [base] + [grid] and/or [[job]]')
    p.add_argument('--out', type=Path, required=True, help='batch directory; one job-NNNN subdir per job')
    p.add_argument('--jobs', '-j', type=int, default=None,
                   help='worker processes (default: performance-core count via '
                        'sysctl hw.perflevel0.physicalcpu)')
    p.add_argument('--serial', action='store_true', help='run jobs in-process, one after another (debugging)')
    p.add_argument('--dry-run', action='store_true', help='print the expanded job list and exit')
    p.add_argument('--overwrite', action='store_true')
    return p


def _emit(record, done, total, fh):
    fh.write(json.dumps(record) + '\n')
    fh.flush()  # results stay complete up to the last finished job even on Ctrl-C
    line = f'[{done}/{total}] {record["name"]} exit={record["exit"]} wall={record["wall_s"]}s'
    if record.get('outcome'):
        line += f' outcome={record["outcome"]}'
    if record.get('error'):
        line += f' error={record["error"]}'
    print(line)


def main(argv=None):
    args = parser().parse_args(argv)
    spec = tomllib.loads(args.spec.read_text())
    jobs = expand_spec(spec)
    from bike_sim.cli import research
    _validate_argv(research.parser(), jobs)

    if args.dry_run:
        for job in jobs:
            print(f'{job.name}: bike-research {" ".join(job.argv)}')
        print(f'{len(jobs)} job(s)')
        return 0

    if args.out.exists() and any(args.out.iterdir()) and not args.overwrite:
        print(f'ERROR: output exists; choose another --out or pass --overwrite: {args.out}',
              file=sys.stderr)
        return 2
    args.out.mkdir(parents=True, exist_ok=True)
    results_path = args.out / 'batch_results.jsonl'
    tasks = [(j.index, j.name, j.argv, str(args.out / j.name)) for j in jobs]
    by_index = {j.index: j for j in jobs}
    started = time.perf_counter()
    done = 0
    interrupted = False

    with results_path.open('w') as results_file:
        def consume(result):
            nonlocal done
            done += 1
            job = by_index[result['index']]
            _emit({**result, 'params': job.params, 'argv': job.argv},
                  done, len(tasks), results_file)

        def fail_remaining(live, error):
            for task in live:
                consume({'index': task[0], 'name': task[1], 'exit': -2,
                         'wall_s': 0.0, 'out': task[3], 'error': error})

        if args.serial or args.jobs == 1:
            for task in tasks:
                consume(_run_job(task))
        else:
            pool = ProcessPoolExecutor(max_workers=args.jobs or _performance_cores(),
                                       mp_context=mp.get_context('spawn'))
            live = {}
            try:
                futures = {}
                for task in tasks:
                    futures[pool.submit(_run_job, task)] = task
                    live[task[0]] = task
                for future in as_completed(futures):
                    task = futures[future]
                    try:
                        consume(future.result())
                    except BrokenProcessPool:
                        consume({'index': task[0], 'name': task[1], 'exit': -2,
                                 'wall_s': 0.0, 'out': task[3],
                                 'error': 'worker process died'})
                        del live[task[0]]
                        fail_remaining(list(live.values()), 'worker process died')
                        break
                    except Exception as exc:
                        consume({'index': task[0], 'name': task[1], 'exit': -1,
                                 'wall_s': 0.0, 'out': task[3],
                                 'error': f'result lost: {exc}'})
                    del live[task[0]]
                pool.shutdown()
            except KeyboardInterrupt:
                interrupted = True
                pool.shutdown(wait=False, cancel_futures=True)
            except BrokenProcessPool:
                fail_remaining(list(live.values()), 'worker process died')
                pool.shutdown(wait=False, cancel_futures=True)

    elapsed = time.perf_counter() - started
    records = [json.loads(line) for line in results_path.read_text().splitlines() if line.strip()]
    exits = {}
    for r in records:
        exits[r['exit']] = exits.get(r['exit'], 0) + 1
    wall_sum = sum(r['wall_s'] for r in records)
    sim_sum = sum(float(by_index[r['index']].params.get('duration', 3.)) for r in records)
    summary = {'jobs': len(records), 'exit_codes': exits,
               'elapsed_s': round(elapsed, 2), 'worker_wall_s': round(wall_sum, 2),
               'requested_sim_seconds': round(float(sim_sum), 3),
               'throughput_x': round(wall_sum / elapsed, 2) if elapsed else None,
               'results': str(results_path), 'interrupted': interrupted}
    print(json.dumps(summary, indent=2))
    return 130 if interrupted else (0 if set(exits) <= {0} else 1)


if __name__ == '__main__':
    raise SystemExit(main())
