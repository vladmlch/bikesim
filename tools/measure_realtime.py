"""Measure the common accounted physical path; startup and final flush are explicit.

Two opt-in entry points share this file: the historical ``measure_realtime``
Python-only accounted path (kept for the realtime gate), and ``main`` -- the
backend-selectable whole-boundary measurement built on
``bike_sim.sim.realtime_measurement``. Neither configures or builds the native
extension; a requested native backend fails at its own selection check.
"""
from __future__ import annotations

import argparse
import json
from math import isfinite
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))


# Keys the baseline comparison treats as measured numbers. p50/p95/p99 are
# per-step latencies in milliseconds; factor/wall_seconds are whole-run.
_METRIC_KEYS = ('factor', 'step_ms_mean', 'step_ms_p50', 'step_ms_p95',
                'step_ms_p99', 'wall_seconds', 'steps', 'sim_seconds')

# Keys whose mismatch makes a baseline delta descriptive rather than
# like-for-like: different inputs, machines, revisions or step sizes.
_COMPARABILITY_KEYS = ('git_sha', 'track', 'physics', 'machine',
                       'effective_timestep_s', 'requested_duration_s',
                       'record_decimation')


def _native_build_provenance(root):
    """Build/hardening provenance for the selected native extension.

    Resolves the same build directory the test loader selects (the
    NATIVE_TEST_BUILD_PATH/NATIVE_TEST_BUILD_DIR contract), then reads the
    configure-time context CMake wrote there. Every missing piece degrades
    to a recorded error — the measurement never fails over bookkeeping.
    """
    provenance = {}
    loader_dir = str(root / 'tests' / 'reference')
    try:
        sys.path.insert(0, loader_dir)
        from native_loader import selected_build
    except ImportError as error:
        return {'error': f'native_loader unavailable: {error}'}
    finally:
        try:
            sys.path.remove(loader_dir)
        except ValueError:
            pass
    try:
        build = selected_build(os.environ)
    except ValueError as error:
        return {'error': str(error)}
    provenance['build_dir'] = str(build)

    context_path = build / 'native_check_context.json'
    try:
        context = json.loads(context_path.read_text())
    except (OSError, json.JSONDecodeError) as error:
        provenance['context_error'] = (
            f'cannot read {context_path.name}: {error}')
        context = None
    if isinstance(context, dict):
        target = context.get('target_contexts', {}).get('bike_native', {})
        provenance['compiler'] = context.get('compiler')
        provenance['sdk'] = context.get('sdk')
        provenance['hardening'] = context.get('libcpp_hardening')
        provenance['target_configuration'] = target.get('configuration')
        provenance['cxx_standard'] = target.get('cxx_standard')
        provenance['cxx_extensions'] = target.get('cxx_extensions')
        provenance['compile_definitions'] = target.get('compile_definitions')
        provenance['compile_options'] = target.get('compile_options')

    cache = {}
    try:
        for line in (build / 'CMakeCache.txt').read_text().splitlines():
            if line.startswith('NATIVE_') and ':BOOL=' in line:
                key, _, value = line.partition(':BOOL=')
                cache[key] = value.strip() == 'ON'
    except OSError:
        pass
    if cache:
        provenance['sanitizer_options'] = {
            'address': cache.get('NATIVE_SANITIZE', False),
            'realtime': cache.get('NATIVE_RTSAN', False),
            'coverage': cache.get('NATIVE_COVERAGE', False),
        }
    provenance['sanitizer_runtime'] = os.environ.get(
        'NATIVE_TEST_SANITIZER_RUNTIME')
    # The extension actually mapped into this process — the measurement
    # runs whatever the interpreter loaded, which the selected build may
    # or may not own. Report the imported path, not the selector.
    module = sys.modules.get('bike_native')
    provenance['imported_extension'] = (
        str(Path(module.__file__).resolve())
        if module is not None and isinstance(
            getattr(module, '__file__', None), str) else None)
    return provenance


def _baseline_comparison(report, baseline_path):
    """Deltas against a previous realtime.json.

    Missing keys in an older report stay missing rather than fabricating a
    comparison — the comparability block records whether the runs are even
    like-for-like before the numbers are trusted.
    """
    baseline_path = Path(baseline_path)
    try:
        baseline = json.loads(baseline_path.read_text())
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(
            f'cannot read baseline report {baseline_path}: {error}') from error
    if not isinstance(baseline, dict):
        raise ValueError(
            f'baseline report is not a JSON object: {baseline_path}')
    metrics = {}
    for key in _METRIC_KEYS:
        current, reference = report.get(key), baseline.get(key)
        entry = {'current': current, 'baseline': reference}
        numeric = (
            isinstance(value, (int, float)) and not isinstance(value, bool)
            for value in (current, reference))
        if all(numeric):
            entry['delta'] = current - reference
            if reference != 0:
                entry['delta_pct'] = (
                    (current - reference) / abs(reference) * 100.)
        metrics[key] = entry
    comparability = {
        key: {'current': report.get(key), 'baseline': baseline.get(key),
              'match': report.get(key) == baseline.get(key)}
        for key in _COMPARABILITY_KEYS
    }
    return {'path': str(baseline_path), 'metrics': metrics,
            'comparability': comparability}


def measure_realtime(track_path, physics_path, duration_s, *, out, dt_s=.0005,
                     baseline=None):
    if not isfinite(duration_s) or duration_s <= 0.:
        raise ValueError('duration must be finite and positive')
    started=time.perf_counter()
    from bike_sim.cli import ride as ride_cli
    from bike_sim.physics.resolution import load_physics_config
    args=ride_cli.parse_args([
        '--track',str(track_path),'--physics-config',str(physics_path),'--headless',
        '--no-plots','--duration',str(duration_s),'--decimate','80',
        '--timestep',str(dt_s),'--out',str(out)])
    source_cfg=load_physics_config(physics_path)
    sim=ride_cli.build_physical_simulation_from_args(args)
    # Builder-only timing must still choose the real diagnostic demand; a
    # recorder would otherwise be the only owner setting this decimation.
    sim.physical.set_measurement_decimation(args.decimate)
    sim.physical.reference_monitor.strict=False
    startup=time.perf_counter()-started
    dt=float(sim.model.opt.timestep)
    steps=max(1,int(round(duration_s/dt)))
    costs=np.empty(steps)
    reason='duration'
    measured=0
    for i in range(steps):
        tick=time.perf_counter()
        sim.step()
        costs[i]=time.perf_counter()-tick
        measured=i+1
        if sim.crash is not None:
            reason='crash';break
        if sim.position_m >= sim.track.length_m:
            reason='finish';break
    costs=costs[:measured]
    tick=time.perf_counter()
    sim.physical.flush()
    flush_wall=time.perf_counter()-tick
    wall=float(costs.sum())+flush_wall
    root=Path(__file__).resolve().parents[1]
    git=subprocess.run(['git','rev-parse','HEAD'],cwd=root,capture_output=True,text=True,check=True)
    status=subprocess.run(['git','status','--porcelain','--','src/bike_sim','tools/measure_realtime.py'],
                          cwd=root,capture_output=True,text=True,check=True)
    from bike_sim.validation.environment import source_fingerprint
    import bike_sim
    import mujoco
    model_status=sim.physical.model_status.as_dict()
    report={'steps':int(costs.size),'sim_seconds':float(costs.size*dt),
        'wall_seconds':wall,'startup_seconds':startup,'flush_wall_seconds':flush_wall,
        'factor':float(costs.size*dt/wall),'step_ms_mean':float(costs.mean()*1e3),
        'step_ms_p50':float(np.percentile(costs,50)*1e3),
        'step_ms_p95':float(np.percentile(costs,95)*1e3),
        'step_ms_p99':float(np.percentile(costs,99)*1e3),'git_sha':git.stdout.strip(),
        'machine':platform.platform(),'track':str(track_path),'physics':str(physics_path),
        'working_tree_dirty':bool(status.stdout.strip()),
        'source_sha256':source_fingerprint(root/'src/bike_sim'),
        'requested_duration_s':float(duration_s),'source_timestep_s':source_cfg.timestep_s,
        'effective_timestep_s':dt,'controller_interval_s':sim.physical.control_clock.period_s,
        'record_decimation':args.decimate,'monitor_strict':False,'measurement_path':'accounted_headless',
        'reason':reason,'model_valid':model_status['model_valid'],
        'numerically_valid':model_status.get('numerically_valid','not_evaluated'),
        'model_status':model_status,
        'first_failure':sim.physical.reference_monitor.first_failure,
        'provenance':{'python_version':platform.python_version(),
                      'bike_sim_version':bike_sim.__version__,
                      'mujoco_version':mujoco.__version__,
                      'numpy_version':np.__version__,
                      'native_build':_native_build_provenance(root)}}
    if baseline is not None:
        report['baseline']=_baseline_comparison(report,baseline)
    # Use JSON's canonical lists for first_failure in both return and artifact.
    report=json.loads(json.dumps(report,allow_nan=False))
    destination=Path(out);destination.mkdir(parents=True,exist_ok=True)
    (destination/'realtime.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return report


def parser():
    """Whole-boundary backend measurement; the profile timestep is the default."""
    p = argparse.ArgumentParser(description=(
        'Measure the complete physical run boundary -- advance calls, loop '
        'overhead and final flush -- for the selected backend. Setup and export '
        'are reported separately. This tool never configures or builds an '
        'extension.'))
    p.add_argument('--backend', choices=('python', 'native'), default='python',
                   help='execution backend; native uses the selected artifact, never a fallback')
    p.add_argument('--physics-config', type=Path,
                   default=ROOT/'examples/research/viewer_physics_welded.toml')
    p.add_argument('--track', default=str(ROOT/'examples/research/rough_uphill_savage.toml'))
    p.add_argument('--duration', type=float, default=30.,
                   help='requested simulated seconds; a natural terminal prefix stays a prefix')
    p.add_argument('--dt', type=float, default=None,
                   help='explicit physical timestep override; absent keeps the profile value')
    p.add_argument('--warmup', type=int, default=1)
    p.add_argument('--runs', type=int, default=3)
    p.add_argument('--record-decimation', '--decimate', dest='decimation', type=int, default=80)
    p.add_argument('--seed', type=int, default=None, help='generated-road seed override only')
    p.add_argument('--out', type=Path, required=True, help='new or empty directory')
    p.add_argument('--compare', type=Path,
                   help='earlier report.json from the other backend for a same-prefix check')
    p.add_argument('--benchmark', action='store_true',
                   help='steady-state throughput mode: the reference monitor is non-strict, so a '
                        'documented baseline violation is retained as first_failure instead of '
                        'stopping physics; the result is never a valid ride or a threshold claim')
    return p


# Benchmark mode changes only the stop decision: the monitor records the same
# first failure, the integration and every force writer run unchanged. The
# comparison key carries `strict`, so a benchmark report is never comparable
# with a strict one, and an invalid prefix never meets the headless threshold.
def monitor_strict(args):
    return not args.benchmark


def annotate_report(report, args, effective_dt_s):
    report['invocation'] = dict(physics_config=str(args.physics_config.resolve()),
        track=args.track, duration_s=args.duration, dt_override=args.dt,
        effective_dt_s=effective_dt_s, record_decimation=args.decimation,
        monitor_strict=monitor_strict(args), benchmark_mode=bool(args.benchmark),
        steady_state_minimum_simulated_time_s=args.duration)
    report['benchmark_mode'] = bool(args.benchmark)
    if args.benchmark:
        report['limitations'] = ['Benchmark mode: the reference monitor was non-strict; the '
                                 'measured prefix is a throughput sample of a ride whose '
                                 'validity is not established, never a valid-ride claim.']
    return report


def ride_arguments(args):
    """Project measurement arguments onto the physical headless ride CLI."""
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
        from bike_sim.sim.realtime_measurement import (
            measure_run, build_report, compare_runs, steady_state_evidence)
        rider = resolve_rider(ride)
        require_backend(args.backend, ride.resolved_physics, rider)
        track = resolve_track(ride.track, seed=ride.seed, length_m=ride.length)
        seed = track_seed(ride.track, ride.seed)
        def factory():
            return build_physical_driver(track, ride, rider, seed=seed, strict=monitor_strict(args))
        warmups, runs = [], []
        for phase, count, result in (('warmup', args.warmup, warmups), ('run', args.runs, runs)):
            for index in range(count):
                run = measure_run(factory, args.out/f'{phase}-{index:02d}')
                # Steady-state claim classification is per measured run against
                # the explicit requested duration; warm-ups stay unclassified —
                # they are excluded from every claim by the sampling protocol.
                if phase == 'run':
                    run['steady_state'] = steady_state_evidence(run, args.duration)
                result.append(run)
                args.out.mkdir(parents=True, exist_ok=True)
                (args.out/f'{phase}-{index:02d}.json').write_text(
                    json.dumps(run, indent=2, sort_keys=True, allow_nan=False)+'\n', encoding='utf-8')
                line = (f'{phase} {index+1}/{count}: backend={args.backend}; '
                        f'steps={run["physics_steps"]}; RTF={run["real_time_factor"]}; '
                        f'outcome={run["prefix"]["reason"]}; valid={run["valid_prefix"]}')
                if phase == 'run':
                    line += f'; steady_state={run["steady_state"]["status"]}'
                    if run['steady_state']['reasons']:
                        line += f' ({",".join(run["steady_state"]["reasons"])})'
                print(line, flush=True)
        report = annotate_report(build_report(warmups, runs), args, ride.resolved_physics.timestep_s)
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
