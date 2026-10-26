"""Physical budgets of the seated rider's attachments, checked per step.

Forces are the support force **on the rider** in newtons, expressed in the
attachment's own normal/tangent basis, never world axes. A violation marks the
run invalid; it is never repaired by clipping the solved reaction.

F1 re-scope: connect-equality welds (spindle pedals, pinned saddle, connect
grips) are bidirectional soft spring joints — they carry tension and shear
loads a unilateral friction cone cannot express. For those plants the budget
declares a weld load envelope: normal range plus a flat shear capacity. The
defaults keep the unilateral-contact checks bitwise-identical.
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
    # Weld load envelope (F1): normal < min means the weld's tension retention
    # is exceeded; a set max bounds compression; the shear capacities are flat
    # hardware bounds added to the friction cone. None/0 disables each bound.
    saddle_min_normal_n: float = 0.0
    foot_max_normal_n: float | None = None
    saddle_max_normal_n: float | None = None
    foot_shear_n: float = 0.0
    saddle_shear_n: float = 0.0


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
    if (s.kind == 'foot' and (s.normal_n < budget.foot_min_normal_n or
            (budget.foot_max_normal_n is not None
             and s.normal_n > budget.foot_max_normal_n))) or (
            s.kind == 'saddle' and (s.normal_n <= budget.saddle_min_normal_n or
            (budget.saddle_max_normal_n is not None
             and s.normal_n > budget.saddle_max_normal_n))):
        errors.append('normal')
    mu = budget.foot_mu if s.kind == 'foot' else budget.saddle_mu
    shear = budget.foot_shear_n if s.kind == 'foot' else budget.saddle_shear_n
    # Weld shear capacity plus whatever the friction cone admits while the
    # interface is compressive; a flat pad (shear 0) reduces to the cone alone.
    if abs(s.tangent_n) > shear + mu * max(0.0, s.normal_n):
        errors.append('friction')
    if abs(s.moment_nm) > s.half_patch_m * max(0.0, s.normal_n):
        errors.append('cop')
    return tuple(errors)
