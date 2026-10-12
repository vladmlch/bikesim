"""Time and profile the ride loop alone: construction excluded, non-strict.

    uv run python tools/step_profile.py                 # per-step ms and RTF
    uv run python tools/step_profile.py --profile       # + hottest 40 functions
"""
import argparse
import cProfile
import io
import pstats
import sys
import time
import warnings
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'src'))


def build_driver(track_path):
    import measure_realtime as mr
    args = mr.parser().parse_args(['--out', '/tmp/step_profile_unused', '--track', str(track_path)])
    ride = mr.ride_arguments(args)
    from bike_sim.cli.ride import resolve_rider, resolve_track, track_seed
    from bike_sim.sim.ride.physical_driver import build_physical_driver
    rider = resolve_rider(ride)
    track = resolve_track(ride.track, seed=ride.seed, length_m=ride.length)
    return build_physical_driver(track, ride, rider, seed=track_seed(ride.track, ride.seed), strict=False)


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('--steps', type=int, default=1600)
    p.add_argument('--track', type=Path, default=ROOT/'examples/research/rough_uphill_savage.toml')
    p.add_argument('--profile', action='store_true')
    p.add_argument('--top', type=int, default=40)
    args = p.parse_args(argv)
    warnings.simplefilter('ignore')
    sys.path.insert(0, str(ROOT/'tools'))
    driver = build_driver(args.track)
    dt = driver.dt_s
    t0 = time.perf_counter()
    driver.advance(driver.step+args.steps)
    wall = time.perf_counter()-t0
    print(f'steps={args.steps} dt={dt} wall={wall:.2f}s per-step={wall/args.steps*1e3:.3f}ms RTF={args.steps*dt/wall:.3f}')
    if not args.profile:
        return 0
    profiler = cProfile.Profile()
    profiler.enable()
    driver.advance(driver.step+args.steps)
    profiler.disable()
    stats = pstats.Stats(profiler)
    stats.sort_stats('tottime')
    buffer = io.StringIO()
    stats.stream = buffer
    stats.print_stats(args.top)
    text = buffer.getvalue().replace(str(ROOT)+'/', '')
    print('\n'.join(line for line in text.splitlines() if line.strip()))
    return 0


if __name__ == '__main__':
    sys.exit(main())
