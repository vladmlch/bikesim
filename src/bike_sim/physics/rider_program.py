"""Per-cycle rider intent for the seated pedaling program.

A `PedalIntent` is a bounded wish, never a trajectory command: cadence is a
request the R4 allocator may relax, the ankle offset is a foot-pitch
modulation inside the existing joint limits, and the scrape fraction only
chooses how much of the friction cone the return foot asks for. Nothing here
assigns qpos/qvel or brakes the crank from outside the legs.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PedalIntent:
    cadence_rad_s: float
    ankle_offset_rad: float
    scrape_fraction: float


def pedal_intent(phase_rad, cadence_rad_s, *, ankle_amplitude_rad,
                 scrape_fraction) -> PedalIntent:
    values = (phase_rad, cadence_rad_s, ankle_amplitude_rad, scrape_fraction)
    if not all(math.isfinite(x) for x in values):
        raise ValueError('nonfinite pedaling intent')
    if cadence_rad_s < 0 or ankle_amplitude_rad < 0 or not 0 <= scrape_fraction <= 1:
        raise ValueError('invalid pedaling intent')
    return PedalIntent(cadence_rad_s,
        ankle_amplitude_rad*math.sin(phase_rad), scrape_fraction)
