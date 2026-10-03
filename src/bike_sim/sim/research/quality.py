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
    # energy_scale_j is the caller-declared absolute reference (initial
    # kinetic + elastic energy plus an explicit floor for zero-energy rigs);
    # no silent 1 J floor is added here. Positive source work grows the
    # budget; constraint work never does -- a large ideal-constraint defect
    # cannot normalize its own residual away.
    scale = scalar(energy.get('energy_scale_j', 1.), 'initial energy scale', minimum=0.)
    scale += abs(scalar(energy.get('external_work_j', 0.), 'external_work_j'))
    source = energy.get('source_positive_work_j', energy.get('active_work_j', 0.))
    scale += abs(scalar(source, 'source_positive_work_j'))
    ratio = abs(scalar(energy.get('residual_j', 0.), 'mechanical residual'))/scale
    electrical = scalar(energy.get('electrical_residual_j', 0.), 'electrical residual')
    electrical_budget = max(1., abs(scalar(energy.get('electrical_work_j', 0.), 'electrical work')))
    return EnergyQuality(ratio, electrical, ratio <= limit and abs(electrical) <= 1e-7*electrical_budget)
