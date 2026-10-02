"""Source-matched physical evidence; no control law or automatic promotion."""
import argparse
from dataclasses import asdict
from hashlib import sha256
import importlib.metadata
import json
from math import atan, isfinite
from pathlib import Path
import platform
import traceback

from bike_sim.physics.resolution import load_physics_config
from bike_sim.validation.environment import environment_contract, source_fingerprint
from bike_sim.validation.rider_replay import write_report
from bike_sim.validation.antiwheelie_reports import assess_run, pair_key, paired_summary


def runtime_versions():
    return {'python': platform.python_version(), **{
        name: importlib.metadata.version(name) for name in ('mujoco', 'numpy', 'scipy')}}


def evidence_matches(record, expected):
    keys = ('source_sha256', 'configuration_sha256', 'versions')
    return all(expected.get(key) is not None and record.get(key) == expected[key]
               for key in keys)


def profile_identity(physics_profile):
    path = Path(physics_profile)
    config = load_physics_config(path)
    values = asdict(config)
    envelope = values['articulated']['joint_envelope_path']
    if envelope is not None:
        values['articulated']['joint_envelope_path'] = sha256(Path(envelope).read_bytes()).hexdigest()
    encoded = json.dumps(values, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    return {
        'source_sha256': source_fingerprint(Path(__file__).resolve().parents[1]),
        'configuration_sha256': sha256(encoded).hexdigest(),
        'configuration_scope': 'resolved profile inputs; individual fixtures declare their own parameters',
        'physics_profile': str(path), 'physics_profile_sha256': sha256(path.read_bytes()).hexdigest(),
        'versions': runtime_versions(), 'environment': environment_contract(),
    }


def _bounded_result(name, metrics, bounds):
    criteria = {key: {
        'actual': float(metrics[key]),
        'minimum': float(lower) if isfinite(lower) else None,
        'maximum': float(upper) if isfinite(upper) else None,
        'passed': bool(isfinite(metrics[key]) and lower <= metrics[key] <= upper),
    } for key, (lower, upper) in bounds.items()}
    return {'name': name, 'metrics': metrics, 'criteria': criteria,
            'passed': bool(criteria) and all(item['passed'] for item in criteria.values())}


def _load_and_shaft_report(output):
    from bike_sim.validation.load_transfer import load_transfer_rig
    from bike_sim.validation.plant_torque_rig import shaft_ratio_rig
    cases = []
    for grade in (0., .12, .28, .35):
        for com_x, com_height in ((.45, .9), (.55, .8)):
            metrics, bounds = load_transfer_rig(.00025, atan(grade), com_x, com_height)
            cases.append(_bounded_result(f'load_grade{grade}_com{com_x}_{com_height}', metrics, bounds))
    for rear_teeth in (10, 24, 51):
        for overrun in (False, True):
            metrics, bounds = shaft_ratio_rig(.000125, ratio=34/rear_teeth, overrun=overrun)
            cases.append(_bounded_result(f'shaft_34_{rear_teeth}_overrun{overrun}', metrics, bounds))
    report = {'cases': cases, 'passed': all(case['passed'] for case in cases)}
    write_report(output, report)
    return report


def run_mechanics(output_dir, physics_profile):
    from bike_sim.validation.benchmarks import run_suite
    from bike_sim.validation.contact_resolution import run_contact_resolution
    from bike_sim.validation.drive_suspension_rig import run_comparison
    from bike_sim.validation.rider_factor_matrix import run_matrix
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    report = {'schema_version': 1, **profile_identity(physics_profile),
              'checks': [], 'passed': False}
    analytic_cases = ('wheel_inertia', 'chain_suspension_motion', 'suspension_cycle',
                      'airborne_internal_actuation', 'incline_30_deg', 'brake_hold')
    operations = (
        ('analytic', output/'analytic/report.json',
         lambda: run_suite(output/'analytic', cases=analytic_cases, jobs=1)),
        ('load_and_shaft', output/'load-and-shaft.json',
         lambda: _load_and_shaft_report(output/'load-and-shaft.json')),
        ('drive_suspension', output/'drive-suspension.json',
         lambda: run_comparison(output/'drive-suspension.json')),
        ('contact', output/'contact/report.json',
         lambda: run_contact_resolution(output/'contact')),
        ('rider', output/'rider/report.json',
         lambda: run_matrix(output/'rider', physics_profile=physics_profile)),
    )
    for name, artifact, operation in operations:
        try:
            result = operation()
            passed = (result.get('passed') is True if name != 'rider' else
                      result.get('baseline_resumed') is True and
                      result.get('causal_replay_resolved') is True)
            check = {'name': name, 'passed': passed, 'artifact': str(artifact),
                     'sha256': sha256(artifact.read_bytes()).hexdigest(), 'error': None}
        except Exception as error:
            check = {'name': name, 'passed': False, 'artifact': str(artifact),
                     'sha256': None, 'error': f'{type(error).__name__}: {error}',
                     'traceback': traceback.format_exc()}
        report['checks'].append(check)
        write_report(output/'mechanics.json', report)
        print(f"{name}: {'passed' if check['passed'] else 'failed'}", flush=True)
    report['source_unchanged_during_run'] = (
        report['source_sha256'] == source_fingerprint(Path(__file__).resolve().parents[1]))
    report['passed'] = (len(report['checks']) == len(operations)
                        and all(check['passed'] for check in report['checks'])
                        and report['source_unchanged_during_run'])
    write_report(output/'mechanics.json', report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--physics-config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--phase', choices=('mechanics',), default='mechanics')
    arguments = parser.parse_args(argv)
    report = run_mechanics(arguments.output, arguments.physics_config)
    print(json.dumps({'passed': report['passed']}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
