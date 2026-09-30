#!/usr/bin/env python3
"""Run synthetic plant acceptance cases, with raw evidence and energy gates.

No success in this suite establishes measured accuracy or a safe real controller.
Use --dt 0.000125 0.0000625 for a timestep comparison, --jobs for independent
processes. Coarse-step rejection deliberately uses 0.0005 s in both matrices.
"""
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
from pathlib import Path
import argparse
import json
import time
import traceback
import numpy as np

TRANSMISSIONS = ('elastic_chain', 'ideal_mid_drive', 'geometric_ideal_mid_drive')
CASES = ('wheelie', 'limited', 'rough', 'standing', 'low_grip', 'crest', 'incline', 'reject_coarse')


def case_transmission(name, requested):
    """reject_coarse checks that the chain's 0.5 ms instability is caught; the ideal modes have
    no chain spring, so the case is only meaningful on elastic_chain."""
    return 'elastic_chain' if name == 'reject_coarse' else requested


def run_case(payload):
    name, dt, root, seed, overwrite, transmission = payload
    transmission = case_transmission(name, transmission)
    from bike_sim.cli.research import parser, make_environment, posture_at
    from bike_sim.sim.ride.control import RideControl
    from bike_sim.sim.ride.physical_samples import plain
    from bike_sim.terrain import TrackSpec, GradeProfile
    from bike_sim.terrain.trackfile import save_track
    path = Path(root)/f'{name}_{transmission}_dt_{dt:.8f}'
    started = time.monotonic()
    env = None
    result = dict(case=name, timestep_s=dt, seed=seed, transmission=transmission, output=str(path), passed=False)
    try:
        if path.exists() and any(path.iterdir()) and not overwrite:
            raise FileExistsError(f'output already exists: {path}')
        argv = ['--scenario', 'flat', '--rider', 'lumped', '--duration', '1',
                '--initial-speed', '.5', '--dt', str(dt), '--seed', str(seed),
                '--transmission', transmission, '--ideal-sensors', '--record-decimation', str(max(1, round(.01/dt)))]
        if name in ('wheelie', 'limited', 'reject_coarse'):
            argv += ['--motor-torque', '300', '--motor-max-torque', '300', '--motor-max-power', '10000']
            if name != 'wheelie':
                argv += ['--motor-limit', '60']
        elif name in ('rough', 'standing'):
            argv += ['--rider', 'articulated_planar', '--duration', '2', '--initial-speed', '3']
            if name == 'rough':
                argv += ['--scenario', 'rough_uphill']
            else:
                argv += ['--posture', 'standing', '--motor-torque', '0']
        elif name == 'low_grip':
            argv += ['--scenario', 'low_grip', '--initial-speed', '3', '--duration', '2']
        elif name == 'crest':
            argv += ['--scenario', 'crest', '--initial-speed', '10', '--duration', '2', '--motor-torque', '0']
        elif name == 'incline':
            path.mkdir(parents=True, exist_ok=True)
            track = TrackSpec('constant_25_percent', 12., surface='asphalt',
                grade_profile=GradeProfile(((0., .25), (12., .25))))
            save_track(track, path/'authored_incline.toml')
            argv += ['--track-file', str(path/'authored_incline.toml'), '--motor-torque', '0',
                     '--duration', '.2', '--initial-speed', '2']
        args = parser().parse_args(argv)
        env = make_environment(args)
        print(f'initialized {name}, dt={dt:g}, residual={env.sim.equilibrium["residual_qacc"]:.3g}', flush=True)
        rider = env.sim.physical.rider_control
        def hip_height():
            if rider is None:
                return 0.
            d = env.sim.data
            return float((d.xmat[rider.frame].reshape(3, 3).T @ (d.xpos[rider.pelvis]-d.xpos[rider.frame]))[2])
        initial_hip = hip_height()
        max_hip_lift = 0.
        surfaces = set()
        while not env.done:
            env.step(RideControl(motor_torque_nm=args.motor_torque, motor_limit_nm=args.motor_limit,
                human_torque_nm=0., posture=posture_at(args.posture, env.sim.time_s)))
            max_hip_lift = max(max_hip_lift, hip_height()-initial_hip)
            for side in ('front', 'rear'):
                surfaces.add(env.sim.physical.tire.diagnostics[side]['surface'])
        env.save(path, overwrite=True)
        metrics = env.tracker.metrics
        checks = dict(numerical_energy_gate=env.numerically_valid, duration_or_expected_crash=
                      env.reason in ('duration', 'crash:loop_out', 'crash:endo'))
        if name == 'wheelie':
            checks.update(front_lift_observed=metrics['wheelie_episodes'] >= 1 and metrics['max_front_clearance_m'] > .05)
        elif name == 'limited':
            checks.update(front_supported=metrics['wheelie_time_s'] == 0. and metrics['min_front_load_n'] > 100.,
                          duration_reached=env.reason == 'duration')
        elif name == 'rough':
            checks.update(reached_rough_grade=env.last_truth.position_m > 4.2, duration_reached=env.reason == 'duration')
        elif name == 'standing':
            checks.update(physical_hip_lift=max_hip_lift > .03, duration_reached=env.reason == 'duration')
        elif name == 'low_grip':
            checks.update(actual_wet_contact='wet' in surfaces, duration_reached=env.reason == 'duration')
        elif name == 'crest':
            checks.update(two_wheel_flight=metrics['flight_time_s'] > .005)
        elif name == 'incline':
            checks.update(grade_not_wheelie=metrics['wheelie_time_s'] == 0. and
                          abs(env.last_truth.relative_pitch_rad) < .03 and env.last_truth.road_pitch_rad > .15)
        else:
            checks = dict(unstable_step_rejected=env.reason == 'numerical_quality' and not env.numerically_valid)
        result.update(passed=all(checks.values()), checks=checks, outcome=env.reason, metrics=metrics,
            max_energy_residual_ratio=env.max_energy_residual_ratio,
            energy=plain(env.sim.physical.energy), final_truth=asdict(env.last_truth),
            max_hip_lift_m=max_hip_lift, contacted_surfaces=sorted(surfaces),
            source_sha256=env.metadata['model_source_sha256'])
    except Exception as exc:
        result.update(error=f'{type(exc).__name__}: {exc}', traceback=traceback.format_exc())
    result['wall_time_s'] = time.monotonic()-started
    path.mkdir(parents=True, exist_ok=True)
    (path/'acceptance.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(f'{name} dt={dt:g}: {"PASS" if result["passed"] else "FAIL"} {result.get("outcome", result.get("error"))}', flush=True)
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cases', nargs='+', choices=CASES, default=list(CASES))
    p.add_argument('--dt', nargs='+', type=float, default=[.000125])
    p.add_argument('--transmission', choices=TRANSMISSIONS, default='ideal_mid_drive')
    p.add_argument('--seed', type=int, default=17)
    p.add_argument('--jobs', type=int, default=1)
    p.add_argument('--out', type=Path, default=Path('verification/antiwheelie'))
    p.add_argument('--overwrite', action='store_true')
    args = p.parse_args(argv)
    if args.jobs < 1 or any(not np.isfinite(dt) or dt <= 0. for dt in args.dt):
        p.error('jobs and finite timesteps must be positive')
    args.out.mkdir(parents=True, exist_ok=True)
    work = list(dict.fromkeys((name, .0005 if name == 'reject_coarse' else dt,
                              str(args.out), args.seed, args.overwrite, args.transmission) for dt in args.dt for name in args.cases))
    if args.jobs == 1:
        results = [run_case(item) for item in work]
    else:
        with ProcessPoolExecutor(max_workers=args.jobs) as pool:
            results = list(pool.map(run_case, work))
    convergence = []
    for name in ('wheelie', 'limited'):
        available = sorted((r for r in results if r['case'] == name and 'metrics' in r), key=lambda r: r['timestep_s'])
        for fine, coarse in zip(available, available[1:]):
            time_difference = abs(fine['metrics']['wheelie_time_s']-coarse['metrics']['wheelie_time_s'])
            gap_difference = abs(fine['metrics']['max_front_clearance_m']-coarse['metrics']['max_front_clearance_m'])
            convergence.append(dict(case=name, fine_dt_s=fine['timestep_s'], coarse_dt_s=coarse['timestep_s'],
                wheelie_time_difference_s=time_difference, max_gap_difference_m=gap_difference,
                passed=time_difference <= .035 and gap_difference <= .05))
    report = dict(transmission=args.transmission, passed=all(r['passed'] for r in results) and all(r['passed'] for r in convergence),
        scope='Synthetic planar plant acceptance; not experimental calibration or real-controller certification.',
        convergence=convergence, cases=results)
    (args.out/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
