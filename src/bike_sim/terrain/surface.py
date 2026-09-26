"""
Road Surface Properties.

The friction characteristics the pneumatic tyre reads (docs/RIDE.md section 4.1): a peak and
a sliding friction coefficient, the tyre's normalised longitudinal slip stiffness on that
surface, and the Stribeck speed over which friction falls from peak to sliding.

**One surface per run today.** The tyre never holds a surface itself: it asks a `SurfaceMap`
for the surface at a track position. That map holds one surface now, and is the seam through
which per-zone surfaces in a track file can be added later without touching the tyre
(docs/superpowers/plans/2026-09-26-pneumatic-tyre.md, D9).

**Every value is authored** (docs/RIDE.md section 11.2): car Burckhardt curves scaled to the
little published MTB data. They are a considered starting point, not a measurement.

The rigid road is unchanged by any of this: a surface alters the friction curve and nothing
else (docs/RIDE.md section 12, item 10). Pure data -- no MuJoCo, no other ``bike_sim``
package.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np


@dataclass(frozen=True)
class SurfaceSpec:
    """
    Friction characteristics of one road surface for a knobby MTB tyre.

    Attributes:
        name: Registry name.
        mu_peak: Friction coefficient at vanishing sliding speed -- the static end of the
            curve, which sets the brush model's peak.
        mu_slide: Friction coefficient once the tread slides fast; never above `mu_peak`.
        slip_stiffness_per_load: Initial slope of longitudinal force against slip,
            normalised by the normal load (``C_kappa / F_z``, per unit slip).
        stribeck_speed_mps: Sliding speed over which friction decays from peak to sliding,
            in m/s.
        description: One line for listings.
    """

    name: str
    mu_peak: float
    mu_slide: float
    slip_stiffness_per_load: float
    stribeck_speed_mps: float = 1.0
    description: str = ""

    def __post_init__(self) -> None:
        """
        Raises:
            ValueError: If a coefficient is not positive, or sliding friction exceeds peak.
        """
        for label, value in (
            ("mu_peak", self.mu_peak),
            ("mu_slide", self.mu_slide),
            ("slip_stiffness_per_load", self.slip_stiffness_per_load),
            ("stribeck_speed_mps", self.stribeck_speed_mps),
        ):
            if not value > 0.0:
                raise ValueError(f"surface '{self.name}': {label} must be positive, got {value}")
        if self.mu_slide > self.mu_peak:
            raise ValueError(
                f"surface '{self.name}': mu_slide {self.mu_slide} exceeds mu_peak {self.mu_peak}"
            )

    def mu(self, sliding_speed_mps):
        """
        Friction coefficient at a sliding speed.

        ``mu(V_s) = mu_slide + (mu_peak - mu_slide) . exp(-|V_s| / V_str)`` (docs/RIDE.md
        section 4.1).

        Args:
            sliding_speed_mps: Sliding speed of the tread over the road, in m/s; a scalar or
                an array. The sign is ignored.

        Returns:
            The coefficient, with the argument's shape (a float for a scalar).
        """
        decay = np.exp(-np.abs(sliding_speed_mps) / self.stribeck_speed_mps)
        value = self.mu_slide + (self.mu_peak - self.mu_slide) * decay
        return float(value) if np.ndim(value) == 0 else value


SURFACES: Dict[str, SurfaceSpec] = {
    spec.name: spec
    for spec in (
        SurfaceSpec("asphalt", mu_peak=1.05, mu_slide=0.75, slip_stiffness_per_load=15.0,
                    description="dry asphalt"),
        SurfaceSpec("hardpack", mu_peak=0.80, mu_slide=0.60, slip_stiffness_per_load=12.0,
                    description="dry hard-packed dirt"),
        SurfaceSpec("loose", mu_peak=0.55, mu_slide=0.45, slip_stiffness_per_load=7.0,
                    description="loose gravel over hardpack"),
        SurfaceSpec("wet", mu_peak=0.50, mu_slide=0.40, slip_stiffness_per_load=10.0,
                    description="wet dirt"),
    )
}

# Default surfaces by kind of track (docs/RIDE.md section 4.1): roads are asphalt, trails and
# the synthetic test tracks are hardpack.
ROAD_SURFACE = "asphalt"
TRAIL_SURFACE = "hardpack"
DEFAULT_SURFACE = TRAIL_SURFACE


def available_surfaces() -> List[str]:
    """Returns the names of every registered surface, in registry order."""
    return list(SURFACES)


def get_surface(name: str) -> SurfaceSpec:
    """
    Looks a surface up by name.

    Args:
        name: Registered surface name.

    Returns:
        The surface.

    Raises:
        KeyError: If no surface is registered under that name.
    """
    if name not in SURFACES:
        raise KeyError(f"unknown surface '{name}'; available: {', '.join(available_surfaces())}")
    return SURFACES[name]


class SurfaceMap:
    """
    The road surface as a function of track position.

    Holds a single surface today, so `at` ignores its argument. The tyre calls `at` with the
    contact patch's track position on every step all the same, so that zones can be added
    here without changing the caller.
    """

    def __init__(self, surface: SurfaceSpec) -> None:
        """
        Args:
            surface: The surface the whole track is made of.
        """
        self._surface = surface

    @classmethod
    def uniform(cls, name: str) -> "SurfaceMap":
        """
        Builds a single-surface map by surface name.

        Raises:
            KeyError: If the name is not registered.
        """
        return cls(get_surface(name))

    def at(self, x_m: float) -> SurfaceSpec:
        """
        Returns the surface at a track position.

        Args:
            x_m: Position along the track, in metres. Unused while the map is uniform.
        """
        return self._surface

    @property
    def surfaces(self) -> Tuple[SurfaceSpec, ...]:
        """Every distinct surface on the map."""
        return (self._surface,)

    @property
    def name(self) -> str:
        """Human-readable label: the surface name while the map is uniform."""
        return self._surface.name


def slip_stiffness_n(surface: SurfaceSpec, normal_load_n: float) -> float:
    """
    Longitudinal slip stiffness at a normal load, in N per unit slip.

    Args:
        surface: Surface the tyre is on.
        normal_load_n: Patch normal load, in newtons.

    Returns:
        ``C_kappa = (C_kappa / F_z) . F_z``; zero for a non-positive load.
    """
    if not normal_load_n > 0.0 or not math.isfinite(normal_load_n):
        return 0.0
    return surface.slip_stiffness_per_load * normal_load_n


__all__ = [
    "SurfaceSpec",
    "SURFACES",
    "ROAD_SURFACE",
    "TRAIL_SURFACE",
    "DEFAULT_SURFACE",
    "available_surfaces",
    "get_surface",
    "SurfaceMap",
    "slip_stiffness_n",
]
