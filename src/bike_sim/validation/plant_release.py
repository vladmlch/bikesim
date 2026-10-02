"""Plant release decision gate and physical signal integral calculator.

Aggregates independent evidence into a fail-closed SIL release decision.
Does not claim physical calibration or real-world safety certification.
"""
import numpy as np

_REQUIRED = ('model_valid', 'numerically_valid', 'contact_passed',
             'convergence_passed', 'rider_budget_passed', 'mass_momentum_passed',
             'transmission_parity_passed', 'sensor_split_passed')


def release_decision(evidence: dict, *, require_completion: bool) -> dict:
    checks = {name: evidence.get(name) is True for name in _REQUIRED}
    if require_completion:
        checks['finished'] = evidence.get('finished') is True
    return {'sil_ready': all(checks.values()), 'checks': checks,
            'calibration_status': 'unvalidated', 'real_world_safety_claim': False}


def trace_integral(time_s, value):
    t = np.asarray(time_s, dtype=float)
    y = np.asarray(value, dtype=float)
    if t.ndim != 1 or y.shape != t.shape or len(t) < 2:
        raise ValueError('trace needs paired scalar samples')
    if not np.isfinite(t).all() or not np.isfinite(y).all() or np.any(np.diff(t) <= 0):
        raise ValueError('trace must have finite strictly increasing times')
    return float(np.sum(.5 * (y[1:] + y[:-1]) * np.diff(t)))
