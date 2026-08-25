"""
Road Profile Obstacle Catalogue.

Defines the obstacle primitives from which a longitudinal road profile z = h(x) is
composed. Every obstacle is pure geometry: it maps a local longitudinal coordinate to a
height contribution in metres and knows nothing about MuJoCo, physics, or the sampling
grid it will eventually be rasterized onto.

Two quantities describe an obstacle:

- ``elevation(s)`` -- height in metres relative to the datum the obstacle is entered at,
  defined for local ``s`` in ``[0, length_m]`` and returning zero at both ends unless the
  obstacle deliberately steps the road down;
- ``datum_shift_m`` -- permanent change of road level applied to everything downstream,
  non-zero only for the obstacles that genuinely descend (Drop, Kicker).

Obstacles that depend on pseudo-randomness (RockGarden, Roots) are seeded and are
**grid-independent**: their shape is an analytic function of the seed alone, so sampling
the same obstacle at 5 mm and at 2.5 mm yields the same surface.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Tuple
import numpy as np
from scipy.ndimage import gaussian_filter1d


@dataclass
class Obstacle(ABC):
    """Abstract base for a single feature of the longitudinal road profile."""

    start_m: float

    @property
    @abstractmethod
    def length_m(self) -> float:
        """Longitudinal extent of the obstacle in metres."""

    @property
    def datum_shift_m(self) -> float:
        """Permanent road-level change applied downstream of this obstacle, in metres."""
        return 0.0

    @property
    def end_m(self) -> float:
        """Absolute longitudinal coordinate at which the obstacle ends, in metres."""
        return self.start_m + self.length_m

    @property
    def label(self) -> str:
        """Short human-readable identifier used in plots and telemetry annotations."""
        return f"{type(self).__name__}@{self.start_m:.0f}m"

    @abstractmethod
    def elevation(self, s: np.ndarray) -> np.ndarray:
        """
        Computes the height contribution of the obstacle.

        Args:
            s: Local longitudinal coordinate in metres, within [0, length_m].

        Returns:
            Height in metres relative to the datum at which the obstacle is entered.
        """


@dataclass
class SquareEdge(Obstacle):
    """
    Rectangular ledge: a step up, a flat top, and a step down.

    Rasterized onto a heightfield the vertical faces become one-cell ramps; at 5 mm
    resolution a 90 mm edge is an 87 degree face.
    """

    height_m: float = 0.090
    ledge_length_m: float = 0.250

    @property
    def length_m(self) -> float:
        return self.ledge_length_m

    def elevation(self, s: np.ndarray) -> np.ndarray:
        return np.full_like(s, self.height_m, dtype=float)


@dataclass
class Pothole(Obstacle):
    """
    Rectangular hole in the road surface.

    Width is chosen against wheel radius: a wheel of radius R crossing a hole of width W
    drops ``R - sqrt(R^2 - (W/2)^2)``, which for a 600 mm hole and a 372 mm wheel is
    152 mm -- so a 180 mm hole is not floored by the tyre.
    """

    depth_m: float = 0.180
    hole_length_m: float = 0.600

    @property
    def length_m(self) -> float:
        return self.hole_length_m

    def elevation(self, s: np.ndarray) -> np.ndarray:
        return np.full_like(s, -self.depth_m, dtype=float)


@dataclass
class Washboard(Obstacle):
    """
    Periodic braking bumps: a whole number of sine waves returning exactly to the datum.

    ``amplitude_m`` is the half peak-to-peak height.
    """

    amplitude_m: float = 0.035
    wavelength_m: float = 0.900
    n_waves: int = 9

    @property
    def length_m(self) -> float:
        return self.wavelength_m * self.n_waves

    def elevation(self, s: np.ndarray) -> np.ndarray:
        return self.amplitude_m * np.sin(2.0 * np.pi * s / self.wavelength_m)


@dataclass
class GOut(Obstacle):
    """
    Smooth compression: a raised-cosine dip that loads the suspension without an impact.

    Tangent to the datum at both ends, deepest at the midpoint.
    """

    depth_m: float = 0.350
    dip_length_m: float = 5.0

    @property
    def length_m(self) -> float:
        return self.dip_length_m

    def elevation(self, s: np.ndarray) -> np.ndarray:
        return -self.depth_m * 0.5 * (1.0 - np.cos(2.0 * np.pi * s / self.dip_length_m))


@dataclass
class Drop(Obstacle):
    """
    Step down to a permanently lower road level.

    With ``ramp_m = 0`` the obstacle occupies no longitudinal extent and the entire
    descent is carried by ``datum_shift_m``; the heightfield renders it as a single-cell
    face. A non-zero ramp descends linearly instead.
    """

    height_m: float = 0.600
    ramp_m: float = 0.0

    @property
    def length_m(self) -> float:
        return self.ramp_m

    @property
    def datum_shift_m(self) -> float:
        return -self.height_m

    def elevation(self, s: np.ndarray) -> np.ndarray:
        if self.ramp_m <= 0.0:
            return np.zeros_like(s, dtype=float)
        return -self.height_m * (s / self.ramp_m)


@dataclass
class Kicker(Obstacle):
    """
    Take-off ramp, gap, and descending landing.

    Geometry, in order: a linear ramp rising to ``height_m`` over ``ramp_m`` (launch angle
    ``atan(height_m / ramp_m)``); a step down to the datum at the lip followed by a flat
    gap of ``gap_m``; then a landing slope descending at ``landing_angle_deg`` for
    ``landing_m``, which becomes the downstream datum shift.

    The ramp is linear rather than eased so that the launch angle is well defined: an
    ease-out transition would leave the lip horizontal and there would be no jump.
    """

    height_m: float = 0.350
    ramp_m: float = 1.400
    gap_m: float = 2.500
    landing_angle_deg: float = 20.0
    landing_m: float = 3.000

    @property
    def length_m(self) -> float:
        return self.ramp_m + self.gap_m + self.landing_m

    @property
    def landing_drop_m(self) -> float:
        """Vertical descent of the landing slope in metres."""
        return self.landing_m * np.tan(np.radians(self.landing_angle_deg))

    @property
    def launch_angle_deg(self) -> float:
        """Angle of the take-off ramp above horizontal, in degrees."""
        return float(np.degrees(np.arctan2(self.height_m, self.ramp_m)))

    @property
    def datum_shift_m(self) -> float:
        return -self.landing_drop_m

    def elevation(self, s: np.ndarray) -> np.ndarray:
        z = np.zeros_like(s, dtype=float)
        lip = self.ramp_m
        gap_end = self.ramp_m + self.gap_m

        on_ramp = s < lip
        z[on_ramp] = self.height_m * (s[on_ramp] / self.ramp_m)

        on_landing = s >= gap_end
        z[on_landing] = -(s[on_landing] - gap_end) * np.tan(np.radians(self.landing_angle_deg))

        return z


@dataclass
class Roots(Obstacle):
    """
    A series of smoothed bumps at irregular spacing, standing in for a root section.

    Bump centres are evenly spaced and then jittered by a seeded generator, so the section
    is reproducible and independent of the sampling grid.
    """

    n_bumps: int = 7
    height_m: float = 0.080
    width_m: float = 0.250
    section_length_m: float = 7.0
    jitter: float = 0.35
    seed: int = 0

    @property
    def length_m(self) -> float:
        return self.section_length_m

    def bump_centres_m(self) -> np.ndarray:
        """Local longitudinal positions of the bump centres, in metres."""
        margin = self.width_m
        span = self.section_length_m - 2.0 * margin
        even = margin + span * (np.arange(self.n_bumps) + 0.5) / self.n_bumps
        spacing = span / self.n_bumps
        rng = np.random.default_rng(self.seed)
        offsets = rng.uniform(-self.jitter, self.jitter, size=self.n_bumps) * spacing
        return np.clip(even + offsets, margin, self.section_length_m - margin)

    def elevation(self, s: np.ndarray) -> np.ndarray:
        z = np.zeros_like(s, dtype=float)
        half = self.width_m / 2.0
        for centre in self.bump_centres_m():
            d = np.abs(s - centre)
            inside = d < half
            z[inside] += self.height_m * 0.5 * (1.0 + np.cos(np.pi * d[inside] / half))
        return z


@dataclass
class RockGarden(Obstacle):
    """
    Correlated pseudo-random roughness, standing in for a rock garden.

    White noise is generated on an internal node grid whose spacing is derived from
    ``correlation_length_m``, smoothed, tapered to zero at both ends so the section joins
    the datum, normalized so the extreme excursion equals ``amplitude_m``, and finally
    interpolated to the requested samples. Because the node grid depends only on the
    obstacle's own parameters, the resulting surface is identical at any sampling
    resolution.

    Interpolation between nodes is linear: the resulting facets are a fair rendering of
    rock, and a smooth spline would overshoot the stated amplitude.
    """

    section_length_m: float = 8.0
    amplitude_m: float = 0.060
    correlation_length_m: float = 0.250
    seed: int = 0

    @property
    def length_m(self) -> float:
        return self.section_length_m

    def _nodes(self) -> Tuple[np.ndarray, np.ndarray]:
        """Returns the internal node positions (m) and their tapered, normalized heights (m)."""
        spacing = self.correlation_length_m / 2.0
        n_nodes = max(int(np.ceil(self.section_length_m / spacing)) + 1, 4)
        s_nodes = np.linspace(0.0, self.section_length_m, n_nodes)

        rng = np.random.default_rng(self.seed)
        raw = rng.standard_normal(n_nodes)
        smoothed = gaussian_filter1d(raw, sigma=1.0, mode="nearest")

        taper_len = min(self.correlation_length_m * 2.0, self.section_length_m / 2.0)
        taper = np.ones(n_nodes)
        rising = s_nodes < taper_len
        taper[rising] = 0.5 * (1.0 - np.cos(np.pi * s_nodes[rising] / taper_len))
        falling = s_nodes > self.section_length_m - taper_len
        taper[falling] = 0.5 * (
            1.0 - np.cos(np.pi * (self.section_length_m - s_nodes[falling]) / taper_len)
        )
        shaped = smoothed * taper

        peak = float(np.max(np.abs(shaped)))
        if peak > 0.0:
            shaped = shaped * (self.amplitude_m / peak)
        return s_nodes, shaped

    def elevation(self, s: np.ndarray) -> np.ndarray:
        s_nodes, z_nodes = self._nodes()
        return np.interp(s, s_nodes, z_nodes)


__all__ = [
    "Obstacle",
    "SquareEdge",
    "Pothole",
    "Washboard",
    "GOut",
    "Drop",
    "Kicker",
    "Roots",
    "RockGarden",
]
