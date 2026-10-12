"""Run independent bike-research episodes in parallel processes, one per seed.

    uv run python tools/research_sweep.py --out sweeps/climb --seeds 1-8 --jobs 8 -- \
        --track-file examples/research/rough_uphill.toml --duration 20 --record

Episodes share nothing: each process has its own MuJoCo model, data and
recording directory when --record is passed through explicitly. Without it
only sweep logs/return-code metadata are written. Throughput scales with cores;
per-episode speed does not.
"""
import argparse
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path


def parse_seeds(text: str) -> list[int]:
    seeds = []
    for part in text.split(','):
        part = part.strip()
        if '-' in part:
            low, high = (int(v) for v in part.split('-', 1))
            seeds.extend(range(low, high+1))
        elif part:
            seeds.append(int(part))
    return seeds


def command_for(seed: int, out: Path, passthrough: list[str]) -> list[str]:
    return ['uv', 'run', 'bike-research', *passthrough, '--seed', str(seed), '--out', str(out/f'seed-{seed}')]


def run_one(cmd: list[str], log: Path) -> tuple[int, float]:
    # A parent VIRTUAL_ENV would leak into the child's `uv run` environment.
    env = {k: v for k, v in os.environ.items() if k != 'VIRTUAL_ENV'}
    started = time.perf_counter()
    with log.open('w') as handle:
        completed = subprocess.run(cmd, stdout=handle, stderr=subprocess.STDOUT, env=env)
    return completed.returncode, time.perf_counter()-started


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seeds', required=True, help='e.g. 1-8 or 1,2,5')
    parser.add_argument('--jobs', type=int, default=os.cpu_count() or 1)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('passthrough', nargs=argparse.REMAINDER, help='arguments after -- go to bike-research')
    args = parser.parse_args(argv)
    # Only the leading '--' separator is ours; later ones belong to the child.
    passthrough = (args.passthrough[1:] if args.passthrough[:1] == ['--']
                   else args.passthrough)
    # Duplicate seeds would alias one future yet repeat in sweep.json.
    seeds = list(dict.fromkeys(parse_seeds(args.seeds)))
    commands = {seed: command_for(seed, args.out, passthrough) for seed in seeds}
    if args.dry_run:
        for seed in seeds:
            print(' '.join(commands[seed]))
        return 0
    args.out.mkdir(parents=True, exist_ok=True)
    runs = []
    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        futures = {seed: pool.submit(run_one, commands[seed], args.out/f'seed-{seed}.log') for seed in seeds}
        for seed in seeds:
            returncode, wall_s = futures[seed].result()
            runs.append(dict(seed=seed, returncode=returncode, wall_s=wall_s))
            print(f'seed {seed}: rc={returncode} wall={wall_s:.1f}s', flush=True)
    (args.out/'sweep.json').write_text(json.dumps(dict(jobs=args.jobs, passthrough=passthrough, runs=runs), indent=2)+'\n')
    return 0 if all(run['returncode'] == 0 for run in runs) else 1


if __name__ == '__main__':
    sys.exit(main())
