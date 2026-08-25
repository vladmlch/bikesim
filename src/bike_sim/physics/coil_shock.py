"""
Coil Spring Rear Shock Model.

Models the steel coil and the bottom-out bumper of a coil-sprung rear shock as a function
of shaft stroke. Nothing here imports MuJoCo: it is a pure calculator, in the style of
`physics/air_spring.py`, and the axial force it returns is applied to the `shock_stroke`
coordinate by the ride-mode force path.

**The coil is linear, and that is not a simplification.** A metal coil spring has a
constant rate; all of the progression a Horst-link bike shows at the rear wheel comes from
the linkage, which MuJoCo applies exactly through the constraint Jacobian every step. See
docs/RIDE.md section 5 -- multiplying this force by an analytic leverage ratio would apply
the ratio twice.

The bumper is quadratic in engagement depth, so both force and slope are continuous where
it engages, and it reaches its peak exactly at full stroke.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class CoilShockSpecs:
    """
    Dataclass holding the coil, preload and bumper specifications of a rear shock.

    Attributes:
        rate_n_m: Coil spring rate in N/m. The default is the repository's shipped
            `shock_stiffness`.
        preload_mm: Static coil compression already wound into the spring, in mm. The
            default is zero deliberately: the shipped defaults must keep reproducing the
            documented sag behaviour (docs/RIDE.md section 9).
        stroke_mm: Usable shaft stroke in mm.
        bumper_length_mm: Free length of the bottom-out bumper in mm; it engages this far
            before full stroke.
        bumper_peak_n: Bumper force at full stroke, in N.
    """

    rate_n_m: float = 114600.0
    preload_mm: float = 0.0
    stroke_mm: float = 65.0
    bumper_length_mm: float = 10.0
    bumper_peak_n: float = 7000.0

    @property
    def bumper_engage_mm(self) -> float:
        """Shaft stroke at which the bumper first touches, in mm."""
        return self.stroke_mm - self.bumper_length_mm


class CoilShock:
    """
    Linear coil spring with a progressive bottom-out bumper.
    """

    def __init__(self, specs: Optional[CoilShockSpecs] = None) -> None:
        self.specs = specs if specs is not None else CoilShockSpecs()

    def compute_spring_force(self, stroke_mm: float) -> float:
        """
        Computes the coil force at a given shaft stroke.

        Args:
            stroke_mm: Shaft stroke in mm, increasing with compression.

        Returns:
            Coil force in N, including the preload offset.
        """
        return float(self.specs.rate_n_m * (float(stroke_mm) + self.specs.preload_mm) / 1000.0)

    def compute_bumper_force(self, stroke_mm: float) -> float:
        """
        Computes the bottom-out bumper force at a given shaft stroke.

        Args:
            stroke_mm: Shaft stroke in mm, increasing with compression.

        Returns:
            Bumper force in N: exactly zero until the bumper engages, then quadratic in
            engagement depth, reaching `bumper_peak_n` at full stroke.
        """
        excess_mm = max(0.0, float(stroke_mm) - self.specs.bumper_engage_mm)
        return float(self.specs.bumper_peak_n * (excess_mm / self.specs.bumper_length_mm) ** 2)

    def compute_axial_force(self, stroke_mm: float) -> float:
        """
        Computes the total axial spring force along the shock shaft.

        Args:
            stroke_mm: Shaft stroke in mm, increasing with compression.

        Returns:
            Coil plus bumper force in N. This is the generalized force the ride-mode force
            path writes onto the `shock_stroke` coordinate; no leverage ratio is applied.
        """
        return self.compute_spring_force(stroke_mm) + self.compute_bumper_force(stroke_mm)


__all__ = ["CoilShockSpecs", "CoilShock"]
