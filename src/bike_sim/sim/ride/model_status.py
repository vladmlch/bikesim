"""Episode-local applicability, numerical quality and calibration are separate."""
from collections import Counter
from dataclasses import dataclass, field
from math import isfinite
import copy
from bike_sim.sim.research.validity import channel_violations


@dataclass
class ModelStatus:
    counts: Counter = field(default_factory=Counter)
    first: dict | None = None
    last_interval: int = -1
    maximum_compression_fraction: float = .15
    maximum_linkage_error_m: float = .002
    numerically_valid: bool | str = 'not_evaluated'
    calibration_status: str = 'parameterized_unvalidated'

    def observe(self, interval_id: int, time_s: float, channels) -> None:
        if type(interval_id) is not int or interval_id <= self.last_interval or not isfinite(time_s):
            raise ValueError('model status requires one advancing finite interval')
        self.last_interval = interval_id
        reasons = channel_violations(channels, self.maximum_compression_fraction,
                                     self.maximum_linkage_error_m)
        self.counts.update(reasons)
        if reasons and self.first is None:
            self.first = {'interval_id': interval_id, 'time_s': time_s, 'reasons': list(reasons)}
        energy = channels.get('energy')
        if energy:
            from bike_sim.sim.research.quality import energy_quality
            quality = energy_quality(energy)
            self.numerically_valid = bool(quality.acceptable and self.numerically_valid is not False)

    def as_dict(self) -> dict:
        return {'model_valid': not bool(self.counts),
                'first_model_violation': copy.deepcopy(self.first),
                'model_violation_counts': dict(self.counts),
                'numerically_valid': self.numerically_valid,
                'calibration_status': self.calibration_status}

    def reset(self) -> None:
        self.counts.clear()
        self.first = None
        self.last_interval = -1
        self.numerically_valid = 'not_evaluated'
