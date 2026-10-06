"""
Road Surface Properties.

The friction characteristics the pneumatic tyre reads (docs/RIDE.md section 4.1): a peak and
a sliding friction coefficient, the tyre's normalised longitudinal slip stiffness on that
surface, and the Stribeck speed over which friction falls from peak to sliding.

SurfaceMap resolves a base surface and validated half-open material zones at
an actual contact position. Both pneumatic tyres and physical compliant_2d
track-material tyres can use it. The configured physical mode preserves the
original constant-mu contract for existing simulations.

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
from bike_sim.physics.checks import array, derived, derived_array, scalar

DEFAULT_STRIBECK_SPEED_MPS = 4.5


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
    stribeck_speed_mps: float = DEFAULT_STRIBECK_SPEED_MPS
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
            scalar(value, f"SurfaceSpec.{label}", positive=True)
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
        self.__post_init__()
        array(sliding_speed_mps, "SurfaceSpec.sliding_speed_mps")
        decay = np.exp(-np.abs(sliding_speed_mps) / self.stribeck_speed_mps)
        value = self.mu_slide + (self.mu_peak - self.mu_slide) * decay
        derived_array(value, 'SurfaceSpec.mu')
        return float(value) if np.ndim(value) == 0 else value


SURFACES: Dict[str, SurfaceSpec] = {
    spec.name: spec
    for spec in (
        SurfaceSpec("asphalt", mu_peak=1.05, mu_slide=0.75, slip_stiffness_per_load=15.0,
                    stribeck_speed_mps=DEFAULT_STRIBECK_SPEED_MPS,
                    description="dry asphalt"),
        SurfaceSpec("hardpack", mu_peak=0.80, mu_slide=0.60, slip_stiffness_per_load=12.0,
                    stribeck_speed_mps=DEFAULT_STRIBECK_SPEED_MPS,
                    description="dry hard-packed dirt"),
        SurfaceSpec("loose", mu_peak=0.55, mu_slide=0.45, slip_stiffness_per_load=7.0,
                    stribeck_speed_mps=DEFAULT_STRIBECK_SPEED_MPS,
                    description="loose gravel over hardpack"),
        SurfaceSpec("wet", mu_peak=0.50, mu_slide=0.40, slip_stiffness_per_load=10.0,
                    stribeck_speed_mps=DEFAULT_STRIBECK_SPEED_MPS,
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


@dataclass(frozen=True)
class SurfaceSection:
    """A half-open material interval [start_m, end_m) in track coordinates."""
    start_m: float
    end_m: float
    surface: str

    def __post_init__(self):
        from math import isfinite
        from numbers import Real
        for name in ('start_m', 'end_m'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, Real) or not isfinite(value):
                raise ValueError('surface interval stations must be finite real numbers')
            object.__setattr__(self, name, float(value))
        if not 0. <= self.start_m < self.end_m:
            raise ValueError('surface interval must satisfy 0 <= start < end')
        if not isinstance(self.surface, str) or self.surface not in SURFACES:
            raise ValueError('unknown surface material')


class SurfaceMap:
    """
    The road surface as a function of track position.

    A default material plus nonoverlapping, half-open material sections.
    Queries use each wheel contact position, not the frame or wheel center.
    """

    def __init__(self, surface: SurfaceSpec, sections=()) -> None:
        """
        Args:
            surface: The surface the whole track is made of.
        """
        if not isinstance(surface, SurfaceSpec):
            raise ValueError('default material must be a SurfaceSpec')
        sections = tuple(sections)
        if any(not isinstance(s, SurfaceSection) for s in sections):
            raise ValueError('material intervals must be SurfaceSection objects')
        sections = tuple(sorted(sections, key=lambda s: s.start_m))
        if any(b.start_m < a.end_m for a, b in zip(sections, sections[1:])):
            raise ValueError('material intervals must not overlap')
        self._surface = surface
        self.sections = sections

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
            x_m: Contact position along the track in metres.
        """
        self.validate()
        scalar(x_m, 'SurfaceMap.x_m')
        for section in self.sections:
            if section.start_m <= x_m < section.end_m:
                return get_surface(section.surface)
        return self._surface

    def validate(self) -> None:
        if not isinstance(self._surface, SurfaceSpec):
            raise ValueError('SurfaceMap.surface: expected SurfaceSpec')
        self._surface.__post_init__()
        previous = 0.
        for section in self.sections:
            if not isinstance(section, SurfaceSection):
                raise ValueError('SurfaceMap.sections: expected SurfaceSection')
            section.__post_init__()
            get_surface(section.surface).__post_init__()
            if section.start_m < previous:
                raise ValueError('SurfaceMap.sections: unordered or overlapping intervals')
            previous = section.end_m

    @property
    def surfaces(self) -> Tuple[SurfaceSpec, ...]:
        """Every distinct surface on the map."""
        values = {self._surface.name: self._surface}
        values.update((s.surface, get_surface(s.surface)) for s in self.sections)
        return tuple(values.values())

    @property
    def name(self) -> str:
        """Human-readable label: the surface name while the map is uniform."""
        return self._surface.name + ('+zones' if self.sections else '')


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
    "DEFAULT_STRIBECK_SPEED_MPS",
    "SURFACES",
    "ROAD_SURFACE",
    "TRAIL_SURFACE",
    "DEFAULT_SURFACE",
    "available_surfaces",
    "get_surface",
    "SurfaceMap",
    "slip_stiffness_n",
]
