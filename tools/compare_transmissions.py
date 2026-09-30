#!/usr/bin/env python3
"""A/B parity of research transmissions against the elastic_chain reference.

Identical episodes (scenario, seed, dt, initial speed, constant motor torque)
run on the reference and on each candidate; wheelie onset, front-load margin,
peak shock stroke and mean speed are diffed. `passed` only says the candidate is
within the stated tolerances on these synthetic scenarios; it is not a claim
about a real bicycle. The reference is frozen: nothing here tunes the chain.
"""
from pathlib import Path
import argparse
import json
import sys

REFERENCE = 'elastic_chain'
CANDIDATES = ('ideal_mid_drive', 'geometric_ideal_mid_drive')
ONSET_TOLERANCE_S = .05
MIN_LOAD_RELATIVE = .15
SHOCK_STROKE_RELATIVE = .10
SPEED_RELATIVE = .05
# Below this magnitude a reference value is treated as zero, so a relative
# tolerance cannot divide by noise.
ABSOLUTE_FLOOR = 1e-6


def _first_onset(metrics):
    starts = [e['start_s'] for e in metrics['wheelie']['wheelie_episode_records'] if e['confirmed']]
    return min(starts) if starts else None


def _relative(reference, candidate, tolerance):
    if reference is None or candidate is None:
        return dict(reference=reference, candidate=candidate, tolerance=tolerance,
                    passed=reference is None and candidate is None)
    delta = abs(candidate-reference)
    if abs(reference) > ABSOLUTE_FLOOR:
        relative = delta/abs(reference)
        passed = relative <= tolerance
    else:  # relative error undefined (strict JSON has no inf): pass only if both are ~zero
        relative, passed = None, delta <= ABSOLUTE_FLOOR
    return dict(reference=reference, candidate=candidate, delta=delta, relative=relative,
                tolerance=tolerance, passed=passed)


def compare_metrics(reference, candidate):
    """Diff two episode_metrics records; returns per-check deltas and a verdict."""
    ref_onset, cand_onset = _first_onset(reference), _first_onset(candidate)
    onset = dict(reference=ref_onset, candidate=cand_onset, tolerance=ONSET_TOLERANCE_S)
    if ref_onset is None or cand_onset is None:
        onset['passed'] = ref_onset is None and cand_onset is None
    else:
        onset.update(delta=abs(cand_onset-ref_onset), passed=abs(cand_onset-ref_onset) <= ONSET_TOLERANCE_S+1e-12)
    rw, cw = reference['wheelie'], candidate['wheelie']
    checks = {
        'onset_s': onset,
        'front_load_fraction_min': _relative(rw['front_load_fraction_min'], cw['front_load_fraction_min'], MIN_LOAD_RELATIVE),
        'min_front_load_n': _relative(rw['min_front_load_n'], cw['min_front_load_n'], MIN_LOAD_RELATIVE),
        'max_shock_stroke_m': _relative(reference['max_shock_stroke_m'], candidate['max_shock_stroke_m'], SHOCK_STROKE_RELATIVE),
        'mean_speed_mps': _relative(reference['mean_speed_mps'], candidate['mean_speed_mps'], SPEED_RELATIVE),
    }
    return dict(checks=checks, passed=all(c['passed'] for c in checks.values()))


def run_one(scenario, seed, transmission, dt, out, *, duration=5., initial_speed=3., motor_torque=80.):
    from bike_sim.cli.research import parser, make_environment
    from bike_sim.sim.ride.control import RideControl
    argv = ['--scenario', scenario, '--transmission', transmission, '--dt', str(dt),
            '--seed', str(seed), '--duration', str(duration), '--initial-speed', str(initial_speed),
            '--ideal-sensors', '--motor-torque', str(motor_torque), '--out', str(out)]
    env = make_environment(parser().parse_args(argv))
    while not env.done:
        env.step(RideControl(motor_torque_nm=motor_torque, human_torque_nm=0.))
    env.save(out, overwrite=True)
    return json.loads((Path(out)/'episode_metrics.json').read_text())


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenarios', nargs='+', default=['flat', 'uphill', 'rough_uphill', 'crest', 'step_up'])
    p.add_argument('--candidates', nargs='+', choices=CANDIDATES, default=list(CANDIDATES))
    p.add_argument('--dt', type=float, required=True, help='validated physics timestep for the candidates')
    p.add_argument('--reference-dt', type=float, default=None,
                   help='reference timestep (default: --dt); the chain may need a finer step')
    p.add_argument('--seed', type=int, default=17)
    p.add_argument('--duration', type=float, default=5.)
    p.add_argument('--out', type=Path, default=Path('verification/ab_transmission'))
    args = p.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)
    reference_dt = args.dt if args.reference_dt is None else args.reference_dt
    records = {c: [] for c in args.candidates}
    for scenario in args.scenarios:
        ref = run_one(scenario, args.seed, REFERENCE, reference_dt, args.out/f'{scenario}_{REFERENCE}',
                      duration=args.duration)
        for candidate in args.candidates:
            cand = run_one(scenario, args.seed, candidate, args.dt, args.out/f'{scenario}_{candidate}',
                           duration=args.duration)
            verdict = compare_metrics(ref, cand)
            records[candidate].append(dict(scenario=scenario, seed=args.seed, reference_outcome=ref['outcome'],
                                           candidate_outcome=cand['outcome'], **verdict))
    report = dict(reference=REFERENCE, dt_s=args.dt, reference_dt_s=reference_dt, seed=args.seed,
        tolerances=dict(onset_s=ONSET_TOLERANCE_S, min_load_relative=MIN_LOAD_RELATIVE,
            shock_stroke_relative=SHOCK_STROKE_RELATIVE, speed_relative=SPEED_RELATIVE),
        candidates={c: dict(passed=all(r['passed'] for r in rows), scenarios=rows) for c, rows in records.items()})
    (args.out/'comparison.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    failed = False
    for candidate, body in report['candidates'].items():
        print(f'{candidate}: {"PASS" if body["passed"] else "FAIL"}')
        for row in body['scenarios']:
            for name, check in row['checks'].items():
                if not check['passed']:
                    failed = True
                    print(f'  {row["scenario"]}.{name}: {check}', file=sys.stderr)
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
