"""Numerical acceptance diagnostics, not a claim of physical calibration."""
from dataclasses import dataclass
from bike_sim.physics.checks import scalar


@dataclass(frozen=True)
class EnergyQuality:
    residual_ratio: float
    electrical_residual_j: float
    acceptable: bool


def energy_quality(energy, *, maximum_ratio=.05):
    """Normalize the energy defect without hiding it in a large loss estimate.

    Gravitational potential's arbitrary datum is deliberately not a scale.
    Absolute source/road/constraint work and the initial kinetic/elastic energy
    define the available energy budget. Material loss is NOT in the denominator:
    a numerically excited dissipative limit cycle can make that quantity huge.
    """
    limit = scalar(maximum_ratio, 'maximum energy residual ratio', positive=True)
    scale = max(1., scalar(energy.get('energy_scale_j', 1.), 'initial energy scale', minimum=0.))
    for key in ('active_work_j', 'external_work_j', 'solver_constraint_work_j'):
        scale += abs(scalar(energy.get(key, 0.), key))
    ratio = abs(scalar(energy.get('residual_j', 0.), 'mechanical residual'))/scale
    electrical = scalar(energy.get('electrical_residual_j', 0.), 'electrical residual')
    electrical_budget = max(1., abs(scalar(energy.get('electrical_work_j', 0.), 'electrical work')))
    return EnergyQuality(ratio, electrical, ratio <= limit and abs(electrical) <= 1e-7*electrical_budget)
