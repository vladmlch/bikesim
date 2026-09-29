#!/usr/bin/env python3
"""Executable acceptance of added research features; no measured-accuracy claim."""
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import argparse
import json
import time
import traceback

CASES = ('nonlinear_tire', 'sensor_replay', 'rider_program', 'road_refinement')
ROOT = Path(__file__).resolve().parents[1]


def run_case(payload):
    name, output = payload
    import numpy as np
    from bike_sim.cli.research import parser, make_environment
    from bike_sim.sim.ride.control import RideControl
    from bike_sim.sim.research.replay import replay_episode
    from bike_sim.physics.tire_curve import TabulatedTireSpec

    started = time.monotonic()
    path = Path(output)/name
    path.mkdir(parents=True, exist_ok=True)
    result = dict(case=name, passed=False)
    env = None

    def make(extra):
        return make_environment(parser().parse_args([
            '--scenario', 'flat', '--rider', 'lumped', '--duration', '.2',
            '--dt', '.000125', '--initial-speed', '2', '--motor-torque', '0',
            '--seed', '31', '--record-decimation', '80', *extra]))

    def hip_height(plant):
        rider = plant.physical.rider_control
        data = plant.data
        return float((data.xmat[rider.frame].reshape(3, 3).T @
                      (data.xpos[rider.pelvis]-data.xpos[rider.frame]))[2])

    def run(plant_env, *, program=False):
        initial = hip_height(plant_env.sim) if program else 0.
        maximum_lift = 0.
        while not plant_env.done:
            t = plant_env.sim.time_s
            motor = 40. if t < .1 else 0.
            plant_env.step(RideControl(motor_torque_nm=0. if program else motor,
                human_torque_nm=None if program else 0.))
            if program:
                maximum_lift = max(maximum_lift, hip_height(plant_env.sim)-initial)
        return maximum_lift

    def quality(plant_env):
        return dict(duration_reached=plant_env.reason == 'duration',
                    numerical_gate=plant_env.numerically_valid,
                    applicability_gate=plant_env.model_valid)

    try:
        if name == 'nonlinear_tire':
            env = make(['--physics-config', str(ROOT/'examples/research/nonlinear_tires.toml'),
                        '--duration', '.5'])
            run(env)
            env.save(path/'run')
            checks = quality(env)
            checks['table_instantiated'] = isinstance(env.sim.physics_config.tires.front.material, TabulatedTireSpec)
            result['max_energy_residual_ratio'] = env.max_energy_residual_ratio
        elif name == 'sensor_replay':
            env = make(['--sensor-period', '.002', '--sensor-delay', '.007',
                        '--sensor-dropout', '.25', '--sensor-max-age', '.025',
                        '--gyro-bias', '.015', '--accel-bias', '.1', '0', '-.05'])
            run(env)
            env.save(path/'run')
            replay = replay_episode(path/'run')
            (path/'replay.json').write_text(json.dumps(replay, indent=2, allow_nan=False)+'\n')
            checks = quality(env)
            checks.update(replay_matches=replay['passed'],
                          independent_acquisition=env.pipeline.samples_attempted > len(env.observations)*3,
                          missing_samples_exercised=env.pipeline.samples_dropped > 0)
            result.update(samples_attempted=env.pipeline.samples_attempted,
                          samples_dropped=env.pipeline.samples_dropped, policy_observations=len(env.observations))
        elif name == 'rider_program':
            env = make(['--rider', 'articulated_planar', '--duration', '3',
                        '--rider-program', str(ROOT/'examples/research/rider_shift.toml')])
            lift = run(env, program=True)
            env.save(path/'run')
            checks = quality(env)
            m = env.sim.model
            roots = {m.joint(n).id for n in ('root_x', 'root_z', 'root_pitch',
                                            'rider_root_x', 'rider_root_z', 'rider_root_pitch')}
            checks.update(physical_hip_lift=lift > .03,
                          no_root_actuators=not any(int(j) in roots for j in m.actuator_trnid[:, 0]))
            result.update(max_hip_lift_m=lift, max_energy_residual_ratio=env.max_energy_residual_ratio)
        else:
            results = []
            checks = {}
            for spacing in (.005, .0025):
                env = make(['--scenario', 'crest', '--initial-speed', '10', '--duration', '2',
                            '--road-resolution', str(spacing), '--ideal-sensors'])
                while not env.done:
                    env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.))
                env.save(path/f'road_{spacing}')
                checks.update({f'{spacing}:{key}': value for key, value in quality(env).items()})
                results.append(dict(spacing_m=spacing, metrics=env.tracker.metrics.copy()))
            gap_difference = abs(results[0]['metrics']['max_front_clearance_m']-
                                 results[1]['metrics']['max_front_clearance_m'])
            flight_difference = abs(results[0]['metrics']['flight_time_s']-
                                    results[1]['metrics']['flight_time_s'])
            checks.update(clearance_converges=gap_difference <= .02,
                          flight_time_converges=flight_difference <= .02)
            result.update(roads=results, max_gap_difference_m=gap_difference,
                          flight_time_difference_s=flight_difference)
        result.update(passed=all(checks.values()), checks=checks, outcome=env.reason,
                      source_sha256=env.metadata['model_source_sha256'],
                      equilibrium=env.sim.equilibrium)
    except Exception as exc:
        result.update(error=f'{type(exc).__name__}: {exc}', traceback=traceback.format_exc())
    result['wall_time_s'] = time.monotonic()-started
    (path/'acceptance.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(f'{name}: {"PASS" if result["passed"] else "FAIL"} {result.get("outcome", result.get("error"))}', flush=True)
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cases', nargs='+', choices=CASES, default=list(CASES))
    p.add_argument('--out', type=Path, default=Path('verification/completion/features'))
    p.add_argument('--jobs', type=int, default=1)
    args = p.parse_args(argv)
    if args.jobs < 1:
        p.error('jobs must be positive')
    args.out.mkdir(parents=True, exist_ok=True)
    work = [(name, str(args.out)) for name in args.cases]
    if args.jobs == 1:
        results = [run_case(item) for item in work]
    else:
        with ProcessPoolExecutor(max_workers=args.jobs) as pool:
            results = list(pool.map(run_case, work))
    report = dict(passed=all(r['passed'] for r in results), cases=results,
                  scope='Synthetic planar numerical acceptance, not experimental calibration.')
    (args.out/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
