"""Per-interval work bookkeeping: sources and constraints never blur.

Muscle and motor work are recorded as both the signed total and the
positive-only total, so a joint that brakes cannot cancel a joint that drives
inside the same sum. Numerical constraint work (joint limits, closures, ideal
transmissions) is tracked by signed and by absolute value, and is never
counted as physical dissipation or as source work.
"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class StepWork:
    muscle_signed_j: float
    muscle_positive_j: float
    motor_signed_j: float
    motor_positive_j: float
    constraint_signed_j: float
    constraint_absolute_j: float


def step_work(muscle_power_w, motor_power_w, numerical_constraint_power_w,
              dt_s: float) -> StepWork:
    muscle = np.asarray(muscle_power_w, dtype=float)
    constraint = np.asarray(numerical_constraint_power_w, dtype=float)
    if (not np.isfinite(muscle).all() or not np.isfinite(constraint).all()
            or not np.isfinite(motor_power_w) or not np.isfinite(dt_s) or dt_s <= 0):
        raise ValueError('nonfinite work input or invalid timestep')
    return StepWork(float(muscle.sum()*dt_s),
                    float(np.maximum(muscle, 0).sum()*dt_s),
                    float(motor_power_w*dt_s), max(0., float(motor_power_w))*dt_s,
                    float(constraint.sum()*dt_s),
                    float(np.abs(constraint).sum()*dt_s))


def constraint_work_ok(absolute_j: float, source_positive_j: float,
                       roundoff_j: float = 1e-8) -> bool:
    values = (absolute_j, source_positive_j, roundoff_j)
    if not all(np.isfinite(v) and v >= 0 for v in values):
        raise ValueError('invalid work budget')
    return absolute_j <= max(roundoff_j, .01*source_positive_j)
