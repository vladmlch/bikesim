"""Metrics collection restricted strictly to the valid prefix of simulation plant samples."""
from dataclasses import dataclass
from math import isfinite


@dataclass
class ValidPrefix:
    seen: int = 0
    accepted: int = 0
    valid_until_s: float = 0.
    first_bad: dict | None = None
    front_normal_impulse_ns: float = 0.
    rear_normal_impulse_ns: float = 0.

    def observe(self, sample) -> bool:
        dt = float(sample.end_time_s - sample.time_s)
        if not isfinite(dt) or dt <= 0:
            raise ValueError('audit needs a positive finite interval')
        self.seen += 1
        status = sample.channels['model_status']
        flags = (status.get('model_valid'), status.get('numerically_valid'))
        reasons = []
        if flags[0] is not True:
            reasons.append('model_violation' if flags[0] is False else 'model_status_unavailable')
        if flags[1] is not True:
            reasons.append('numerical_quality' if flags[1] is False else 'numerical_not_evaluated')
        if sample.channels.get('rider_effort_budget_exceeded') is True:
            reasons.append('rider_effort_budget')
        if reasons and self.first_bad is None:
            self.first_bad = {'time_s': float(sample.time_s),
                              'reasons': reasons,
                              'model_event': status.get('first_model_violation')}
        if self.first_bad is not None:
            return False
        loads = [float(sample.channels['tires'][side]['normal_load_n'])
                 for side in ('front', 'rear')]
        if any(not isfinite(x) or x < 0 for x in loads):
            raise ValueError('invalid observed normal load')
        self.accepted += 1
        self.valid_until_s = float(sample.end_time_s)
        self.front_normal_impulse_ns += loads[0] * dt
        self.rear_normal_impulse_ns += loads[1] * dt
        return True
