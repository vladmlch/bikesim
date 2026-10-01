"""Unpaced exact-profile timing, with setup and invalid trajectories kept separate."""
import argparse
from contextlib import nullcontext
from hashlib import sha256
import json
import os
from pathlib import Path
import platform
from time import perf_counter, perf_counter_ns

import numpy as np

from bike_sim.physics.checks import scalar
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.physical_session import configuration_metadata
from bike_sim.validation.environment import source_fingerprint
from bike_sim.validation.rider_replay import build_sim, write_report


def timing_metrics(step_ns: list[int], dt_s: float) -> dict:
    interval = scalar(dt_s, 'timed physics interval', positive=True)
    if not step_ns:
        raise ValueError('timing requires at least one completed step')
    durations = np.array([scalar(value, 'step nanoseconds', positive=True) for value in step_ns])
    percentiles = np.percentile(durations, [50, 95, 99])/1e6
    return {'step_count': len(durations), 'p50_step_ms': float(percentiles[0]),
        'p95_step_ms': float(percentiles[1]), 'p99_step_ms': float(percentiles[2]),
        'maximum_step_ms': float(durations.max()/1e6),
        'real_time_factor': float(len(durations)*interval/(durations.sum()/1e9))}


def run_benchmark(physics_profile, track_path, *, mode, duration_s, repeats, output_dir):
    if mode not in ('preview', 'accounted', 'research'):
        raise ValueError('benchmark mode must be preview, accounted or research')
    duration = scalar(duration_s, 'benchmark duration', positive=True)
    if type(repeats) is not int or repeats < 1:
        raise ValueError('benchmark repetitions must be positive integers')
    output = Path(output_dir)
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise FileExistsError('benchmark output already exists; choose a new directory')
    output.mkdir(parents=True, exist_ok=True)
    source_root = Path(__file__).resolve().parents[1]
    source = source_fingerprint(source_root)
    report = {'schema_version': 1, 'mode': mode, 'duration_requested_s': duration,
        'physics_profile': str(physics_profile), 'track_path': str(track_path),
        'source_sha256': source, 'hardware': {'machine': platform.machine(),
            'platform': platform.platform(), 'cpu_count': os.cpu_count()},
        'runs': [], 'passed': False}
    for repetition in range(repeats):
        result = {'repetition': repetition, 'complete': False, 'error': None}
        sim = session = None
        timings = []
        start_setup = perf_counter()
        try:
            sim = build_sim(physics_profile, track_path)
            result.update(configuration_metadata(sim), equilibrium=sim.equilibrium)
            dt = float(sim.model.opt.timestep)
            required_steps = round(duration/dt)
            if abs(required_steps*dt-duration) > 1e-9:
                raise ValueError('benchmark duration must be an integer number of physics steps')
            timed_interval = dt
            if mode == 'research':
                from bike_sim.sim.research.environment import ExperimentConfig, ResearchEnvironment
                from bike_sim.sim.research.sensors import SensorConfig
                from bike_sim.sim.research.policy_session import PolicySession
                from bike_sim.sim.research.policies import passthrough_factory
                experiment = ExperimentConfig(duration_s=duration, control_period_s=.01,
                    actuator_delay_s=.005, record_decimation=80)
                if abs(round(duration/.01)*.01-duration) > 1e-9:
                    raise ValueError('research timing horizon must contain complete control ticks')
                env = ResearchEnvironment(sim, experiment, SensorConfig(sample_period_s=.005))
                session = PolicySession(env, passthrough_factory(),
                    reference='bike_sim.sim.research.policies:passthrough_factory')
                timed_interval = .01
            result['setup_wall_s'] = perf_counter()-start_setup
            result['timing_unit'] = 'control_tick' if session is not None else 'physics_step'
            warmup_steps = min(round(.5/dt), required_steps//2)
            loop_start = perf_counter()
            warmed_at = None
            minimum_front_load = float('inf')
            maximum_gap = 0.
            context = sim.physical.preview_mode() if mode == 'preview' else nullcontext()
            with context:
                while sim.steps < required_steps:
                    incoming_step = sim.steps
                    if incoming_step >= warmup_steps and warmed_at is None:
                        warmed_at = perf_counter(), sim.time_s
                    started = perf_counter_ns()
                    if session is None:
                        sim.step(control=RideControl())
                    else:
                        session.advance()
                    elapsed = perf_counter_ns()-started
                    if incoming_step >= warmup_steps:
                        timings.append(elapsed)
                    status = sim.physical.model_status.as_dict()
                    minimum_front_load = min(minimum_front_load,
                        float(sim.physical.snapshots['front'].normal_load_n))
                    if sim.physical.rider_contacts is not None:
                        diagnostics = sim.physical.rider_contacts.diagnostics
                        maximum_gap = max(maximum_gap, *(float(diagnostics[side+'_pedal']['gap_m'])
                                                        for side in ('front', 'rear')))
                    if status['model_valid'] is not True or status['numerically_valid'] is False:
                        result['reason'] = 'invalid_model_or_numerics'
                        break
                    if sim.crash is not None:
                        result['reason'] = str(sim.crash)
                        break
                    if session is not None and session.env.done:
                        result['reason'] = session.env.reason
                        break
            result['loop_wall_s'] = perf_counter()-loop_start
            result['warmup_sim_s'] = None if warmed_at is None else warmed_at[1]
            result['warmed_loop_real_time_factor'] = (None if warmed_at is None else
                (sim.time_s-warmed_at[1])/max(perf_counter()-warmed_at[0], 1e-12))
            result['timing'] = timing_metrics(timings, timed_interval) if timings else None
            result['complete'] = sim.steps == required_steps
            result['max_pedal_gap_m'] = maximum_gap
            result['min_front_load_n'] = minimum_front_load if np.isfinite(minimum_front_load) else None
            if session is not None:
                save_start = perf_counter()
                session.save(output/f'episode-{repetition:04d}')
                result['recording_write_wall_s'] = perf_counter()-save_start
        except (ValueError, RuntimeError, ArithmeticError, OSError) as error:
            result['error'] = f'{type(error).__name__}: {error}'
            result.setdefault('setup_wall_s', perf_counter()-start_setup)
        finally:
            if sim is not None:
                from bike_sim.sim.research.replay import integration_state
                result.update(end_time_s=sim.time_s, end_position_m=sim.position_m,
                    model_status=sim.physical.model_status.as_dict(),
                    final_state_sha256=sha256(integration_state(sim).tobytes()).hexdigest())
            result['source_unchanged_during_run'] = source_fingerprint(source_root) == source
            result['performance_passed'] = bool(result['complete'] and not result['error']
                and result.get('reason') != 'invalid_model_or_numerics'
                and result.get('model_status', {}).get('model_valid') is True
                and result.get('model_status', {}).get('numerically_valid') is not False
                and result.get('warmed_loop_real_time_factor') is not None
                and result['warmed_loop_real_time_factor'] >= 1.
                and result['source_unchanged_during_run'])
            report['runs'].append(result)
            write_report(output/f'run-{repetition:04d}.json', result)
            write_report(output/'report.json', report)
        print(json.dumps({key: result.get(key) for key in
            ('repetition', 'complete', 'error', 'warmed_loop_real_time_factor')}, sort_keys=True), flush=True)
    report['passed'] = bool(report['runs']) and all(run['performance_passed'] for run in report['runs'])
    write_report(output/'report.json', report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--physics-config', required=True)
    parser.add_argument('--track', required=True)
    parser.add_argument('--mode', choices=('preview', 'accounted', 'research'), required=True)
    parser.add_argument('--duration', type=float, default=8.)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--output', required=True)
    args = parser.parse_args(argv)
    report = run_benchmark(args.physics_config, args.track, mode=args.mode,
        duration_s=args.duration, repeats=args.repeats, output_dir=args.output)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
