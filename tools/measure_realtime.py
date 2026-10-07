"""Measure the common accounted physical path; startup and final flush are explicit."""
import argparse
import json
from math import isfinite
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

import numpy as np


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
    sim.physical.set_record_decimation(args.decimate)
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


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--track',required=True)
    parser.add_argument('--physics-config',required=True)
    parser.add_argument('--duration',type=float,default=20.)
    parser.add_argument('--dt',type=float,default=.0005,
                        help='explicit physical timestep; v2 default .0005 (profile remains unchanged)')
    parser.add_argument('--out',default='output/realtime')
    parser.add_argument('--baseline',default=None,
                        help='optional prior realtime.json — emits metric deltas '
                             'and a like-for-like comparability block')
    args=parser.parse_args()
    print(json.dumps(measure_realtime(args.track,args.physics_config,args.duration,
                                     out=Path(args.out),dt_s=args.dt,
                                     baseline=args.baseline),indent=2))
