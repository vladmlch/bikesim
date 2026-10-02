"""
Longitudinal Road Profile Assembly.

Composes an ordered list of obstacles into a single height profile z = h(x) in metres.

The profile is not a plain sum of local features: obstacles that descend permanently
(Drop, Kicker) carry a ``datum_shift_m``, and everything downstream of them sits at the
new road level. Assembly therefore walks the track left to right, tracking the running
datum, rather than adding independent contributions.

Obstacles may not overlap. This is enforced rather than blended, so that every feature in
a run can be attributed unambiguously to one entry of the catalogue.
"""

from dataclasses import dataclass, field
from typing import List, Tuple
import numpy as np

from bike_sim.terrain.obstacles import Obstacle
from bike_sim.terrain.surface import DEFAULT_SURFACE, SURFACES, SurfaceMap, SurfaceSection, get_surface
from bike_sim.terrain.grade import GradeProfile


@dataclass
class TrackSpec:
    """
    A named track: an ordered set of obstacles laid out along a road of known length.

    ``surface`` names the road surface the pneumatic tyre rides on (docs/RIDE.md section
    4.1). It does not touch the geometry: the profile, the heightfield and every `sphere`
    run are the same whatever it says.
    """

    name: str
    length_m: float
    obstacles: List[Obstacle] = field(default_factory=list)
    description: str = ""
    surface: str = DEFAULT_SURFACE
    grade_profile: GradeProfile | None = None
    surface_sections: tuple[SurfaceSection, ...] = ()

    @property
    def surface_map(self) -> SurfaceMap:
        return SurfaceMap(get_surface(self.surface), self.surface_sections)

    @property
    def sorted_obstacles(self) -> List[Obstacle]:
        """Obstacles ordered by their longitudinal start position."""
        return sorted(self.obstacles, key=lambda o: o.start_m)

    @property
    def datum_shift_m(self) -> float:
        """Net change of road level between the start and the end of the track, in metres."""
        grade = 0. if self.grade_profile is None else self.grade_profile.elevation(self.length_m)
        return float(sum(o.datum_shift_m for o in self.obstacles)) + grade

    @property
    def markers(self) -> List[Tuple[float, str]]:
        """(position, label) pairs for annotating telemetry plots.

        Background texture (``annotate == False``) is left out.
        """
        return [(o.start_m, o.label) for o in self.sorted_obstacles if o.annotate]

    def validate(self) -> None:
        """
        Checks that the layout is well formed.

        Raises:
            ValueError: If the track length is non-positive, the surface is not registered,
                an obstacle falls outside the track, or two obstacles overlap.
        """
        if not np.isfinite(self.length_m) or self.length_m <= 0.0:
            raise ValueError(f"track '{self.name}' has non-positive length {self.length_m}")
        if self.surface not in SURFACES:
            raise ValueError(
                f"track '{self.name}': unknown surface '{self.surface}'; "
                f"available: {', '.join(SURFACES)}"
            )

        if self.grade_profile is not None:
            if not isinstance(self.grade_profile, GradeProfile):
                raise ValueError('grade_profile must be a GradeProfile')
            if self.grade_profile.knots[-1][0] > self.length_m:
                raise ValueError('grade stations must fit inside the track')
        mapping = self.surface_map
        if any(s.end_m > self.length_m for s in mapping.sections):
            raise ValueError('material intervals must fit inside the track')
        ordered = self.sorted_obstacles
        for obs in ordered:
            if obs.length_m < 0.0:
                raise ValueError(f"track '{self.name}': {obs.label} has negative length")
            if obs.start_m < 0.0 or obs.end_m > self.length_m:
                raise ValueError(
                    f"track '{self.name}': {obs.label} spans "
                    f"[{obs.start_m:.3f}, {obs.end_m:.3f}] m, outside [0, {self.length_m:.3f}] m"
                )

        for prev, nxt in zip(ordered, ordered[1:]):
            if nxt.start_m < prev.end_m:
                raise ValueError(
                    f"track '{self.name}': {prev.label} ends at {prev.end_m:.3f} m but "
                    f"{nxt.label} starts at {nxt.start_m:.3f} m -- obstacles may not overlap"
                )


def build_profile(track: TrackSpec, x: np.ndarray) -> np.ndarray:
    """
    Rasterizes a track into a height profile.

    Args:
        track: Track layout to assemble.
        x: Longitudinal sample positions in metres, measured from the track start.

    Returns:
        Height in metres at each sample, relative to the road level at x = 0.
    """
    track.validate()

    x = np.asarray(x, dtype=float)
    z = np.zeros_like(x, dtype=float)
    datum = 0.0

    for obs in track.sorted_obstacles:
        z[x >= obs.start_m] = datum

        if obs.length_m > 0.0:
            span = (x >= obs.start_m) & (x < obs.end_m)
            if np.any(span):
                z[span] = datum + obs.elevation(x[span] - obs.start_m)

        datum += obs.datum_shift_m
        z[x >= obs.end_m] = datum

    if track.grade_profile is not None:
        # The compiled field includes runout beyond the finish. Preserve the
        # established flat-runout contract instead of extrapolating a grade
        # across the (possibly much longer) heightfield.
        z += track.grade_profile.elevation(np.clip(x, 0., track.length_m))
    return z


def profile_extent(track: TrackSpec, resolution_m: float = 0.005) -> Tuple[float, float]:
    """
    Computes the vertical extent a track occupies.

    Args:
        track: Track layout to measure.
        resolution_m: Sampling interval in metres.

    Returns:
        Tuple of (minimum, maximum) height in metres relative to the start datum.
    """
    x = np.arange(0.0, track.length_m + resolution_m, resolution_m)
    z = build_profile(track, x)
    return float(np.min(z)), float(np.max(z))


__all__ = ["TrackSpec", "build_profile", "profile_extent"]
