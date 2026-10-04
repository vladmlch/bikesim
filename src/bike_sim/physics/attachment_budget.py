"""Physical budgets of the seated rider's attachments, checked per step.

Forces are the support force **on the rider** in newtons, expressed in the
attachment's own normal/tangent basis, never world axes. A violation marks the
run invalid; it is never repaired by clipping the solved reaction.
"""
from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class AttachmentSample:
    kind: str
    normal_n: float
    tangent_n: float
    moment_nm: float
    gap_m: float
    pull_n: float = 0.0
    half_patch_m: float = 0.0


@dataclass(frozen=True)
class AttachmentBudget:
    foot_min_normal_n: float
    foot_mu: float
    saddle_mu: float
    grip_pull_n: float
    max_gap_m: float


def attachment_violations(s: AttachmentSample, budget: AttachmentBudget) -> tuple[str, ...]:
    if s.kind not in {'foot', 'saddle', 'grip'}:
        raise ValueError('unknown attachment kind')
    values = (s.normal_n, s.tangent_n, s.moment_nm, s.gap_m,
              s.pull_n, s.half_patch_m)
    if not all(isfinite(v) for v in values):
        return ('nonfinite',)
    if min(s.gap_m, s.pull_n, s.half_patch_m) < 0:
        return ('invalid_measurement',)
    errors = []
    if s.gap_m > budget.max_gap_m:
        errors.append('gap')
    if s.kind == 'grip':
        if s.pull_n > budget.grip_pull_n:
            errors.append('pull')
        return tuple(errors)
    if (s.kind == 'foot' and s.normal_n < budget.foot_min_normal_n) or (
            s.kind == 'saddle' and s.normal_n <= 0.0):
        errors.append('normal')
    mu = budget.foot_mu if s.kind == 'foot' else budget.saddle_mu
    if abs(s.tangent_n) > mu * max(0.0, s.normal_n):
        errors.append('friction')
    if abs(s.moment_nm) > s.half_patch_m * max(0.0, s.normal_n):
        errors.append('cop')
    return tuple(errors)
