"""Seeded rider population: mass, stature, posture and pedalling effort.

The rider is an uncontrolled plant part here. The sampled posture is held for
the whole episode (a single keyframe), so it is a per-episode disturbance
condition, never a balance strategy.
"""
from dataclasses import dataclass
import numpy as np
from bike_sim.physics.checks import scalar
from bike_sim.physics.rider import RiderSpecs
from bike_sim.physics.rider_posture import RiderPosture
from bike_sim.sim.research.rider_program import RiderKeyframe, RiderProgram


def _range(value, name, low_limit, high_limit):
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise ValueError(f'{name} must be a (low, high) pair')
    lo, hi = scalar(value[0], name), scalar(value[1], name)
    if not low_limit <= lo <= hi <= high_limit:
        raise ValueError(f'{name} must satisfy {low_limit} <= low <= high <= {high_limit}')
    return (lo, hi)


@dataclass(frozen=True)
class RiderRandomSpec:
    mass_kg: tuple = (60., 100.)
    height_m: tuple = (1.55, 1.95)
    torso_lean_rad: tuple = (0., .25)
    pelvis_pitch_rad: tuple = (0., .08)
    human_torque_nm: tuple = (0., 25.)

    def __post_init__(self):
        # Limits: posture bounds are RiderPosture's envelope; mass/height are sanity caps.
        for name, lo, hi in (('mass_kg', 20., 200.), ('height_m', 1., 2.3), ('torso_lean_rad', -.8, .8),
                             ('pelvis_pitch_rad', -.5, .5), ('human_torque_nm', 0., 1000.)):
            object.__setattr__(self, name, _range(getattr(self, name), name, lo, hi))


def sample_rider(spec, seed):
    if not isinstance(spec, RiderRandomSpec):
        raise ValueError('expected a RiderRandomSpec')
    rng = np.random.default_rng(seed)
    rider = RiderSpecs(variant='articulated_planar',
        mass_kg=float(rng.uniform(*spec.mass_kg)), height_m=float(rng.uniform(*spec.height_m)))
    posture = RiderPosture(torso_lean_rad=float(rng.uniform(*spec.torso_lean_rad)),
        pelvis_pitch_rad=float(rng.uniform(*spec.pelvis_pitch_rad)))
    program = RiderProgram((RiderKeyframe(0., posture,
        human_torque_nm=float(rng.uniform(*spec.human_torque_nm))),))
    return rider, program
