"""
Heightfield Rasterization.

Converts a longitudinal road profile into the normalized elevation grid a MuJoCo
heightfield expects, and carries the fixed field geometry every track preset shares.

Nothing here imports MuJoCo. The module produces a plain ``(nrow, ncol)`` array in [0, 1]
plus the scalar placement values a caller needs; writing the array into
``model.hfield_data`` and emitting the MJCF asset belong to the MuJoCo layer.

**The field geometry is deliberately fixed.** Every preset shares one grid, so ride mode
compiles to exactly one XML with one golden baseline, and a track is purely a Python
object. The price is an envelope every preset must fit: 120 m long, 5 mm resolution, and a
profile confined to [-2.8, +0.6] m about the starting road level, which leaves the default
preset roughly 290 mm of headroom below and 510 mm above.
"""

from dataclasses import dataclass
from typing import Tuple
import numpy as np

from bike_sim.terrain.profile import TrackSpec, build_profile, profile_extent


@dataclass
class HeightFieldSpec:
    """
    Fixed geometry of the ride-mode heightfield.

    Attributes:
        nrow: Samples across Y. Two is sufficient: the profile does not vary laterally.
        ncol: Samples along X, chosen so the spacing is exactly 5 mm.
        radius_x_m: Half-length of the field along X.
        radius_y_m: Half-width along Y. Kept small because the bike cannot move laterally;
            a wide strip would produce 5 mm by 4 m sliver triangles, the worst case for
            MuJoCo's convex collision routine.
        elevation_m: Vertical span the normalized [0, 1] data is scaled onto.
        base_m: Depth of solid material below the field's local zero.
        datum_z_m: Local height at which the road sits at the start of the track.
    """

    nrow: int = 2
    ncol: int = 24001
    radius_x_m: float = 60.0
    radius_y_m: float = 0.5
    elevation_m: float = 3.4
    base_m: float = 0.5
    datum_z_m: float = 2.8

    @property
    def track_length_m(self) -> float:
        """Longitudinal extent of the field in metres."""
        return 2.0 * self.radius_x_m

    @property
    def resolution_m(self) -> float:
        """Longitudinal distance between adjacent samples in metres."""
        return self.track_length_m / (self.ncol - 1)

    @property
    def min_profile_m(self) -> float:
        """Lowest profile height the field can represent, relative to the start datum."""
        return -self.datum_z_m

    @property
    def max_profile_m(self) -> float:
        """Highest profile height the field can represent, relative to the start datum."""
        return self.elevation_m - self.datum_z_m

    @property
    def size_attr(self) -> str:
        """The MJCF ``size`` attribute of the hfield asset."""
        return (
            f"{self.radius_x_m:.6f} {self.radius_y_m:.6f} "
            f"{self.elevation_m:.6f} {self.base_m:.6f}"
        )

    def track_x(self) -> np.ndarray:
        """Longitudinal sample positions in track coordinates, from 0 to the field length."""
        return np.linspace(0.0, self.track_length_m, self.ncol)

    def geom_x_m(self) -> float:
        """World X at which the field geom must sit for track x = 0 to land at world x = 0."""
        return self.radius_x_m

    def geom_z_m(self, ground_z_m: float) -> float:
        """
        Computes the world Z of the field geom.

        Args:
            ground_z_m: World height at which the road surface should start.

        Returns:
            World Z of the heightfield geom origin, in metres.
        """
        return ground_z_m - self.datum_z_m

    def catch_plane_z_m(self, ground_z_m: float, clearance_m: float = 1.0) -> float:
        """
        Computes the world Z for the runaway catch plane.

        The plane must sit below the *field's* floor rather than below any particular
        preset, because a MuJoCo plane geom is an infinite half-space for collision and
        would otherwise bridge the deepest features of the track.

        Args:
            ground_z_m: World height of the road surface at the track start.
            clearance_m: Gap between the field floor and the catch plane.

        Returns:
            World Z of the catch plane, in metres.
        """
        return self.geom_z_m(ground_z_m) - clearance_m

    def fits(self, extent_m: Tuple[float, float]) -> bool:
        """Reports whether a (minimum, maximum) profile extent fits inside the field."""
        lo, hi = extent_m
        return lo >= self.min_profile_m and hi <= self.max_profile_m

    def normalize(self, profile_m: np.ndarray) -> np.ndarray:
        """
        Scales a height profile onto the field's normalized [0, 1] grid.

        Args:
            profile_m: Height in metres at each of ``ncol`` samples, relative to the road
                level at the track start.

        Returns:
            Array of shape ``(nrow, ncol)`` with values in [0, 1], the profile repeated
            across every row.

        Raises:
            ValueError: If the sample count is wrong or the profile leaves the field's
                vertical envelope.
        """
        profile_m = np.asarray(profile_m, dtype=float)
        if profile_m.shape != (self.ncol,):
            raise ValueError(f"expected {self.ncol} samples, got {profile_m.shape}")

        lo, hi = float(np.min(profile_m)), float(np.max(profile_m))
        if not self.fits((lo, hi)):
            raise ValueError(
                f"profile extent [{lo:+.3f}, {hi:+.3f}] m leaves the field envelope "
                f"[{self.min_profile_m:+.3f}, {self.max_profile_m:+.3f}] m"
            )

        data = (profile_m + self.datum_z_m) / self.elevation_m
        return np.tile(data, (self.nrow, 1))


FIELD = HeightFieldSpec()


def build_field_data(track: TrackSpec, spec: HeightFieldSpec = FIELD) -> np.ndarray:
    """
    Rasterizes a track onto the heightfield grid.

    Samples beyond the end of the track continue flat at the track's final road level, so
    a preset shorter than the field simply gains runout.

    Args:
        track: Track layout to rasterize.
        spec: Field geometry to rasterize onto.

    Returns:
        Array of shape ``(nrow, ncol)`` with values in [0, 1], ready to be flattened into
        ``model.hfield_data``.
    """
    profile = build_profile(track, spec.track_x())
    return spec.normalize(profile)


def assert_track_fits(track: TrackSpec, spec: HeightFieldSpec = FIELD) -> None:
    """
    Checks a track against the fixed field envelope before any model is built.

    Args:
        track: Track layout to check.
        spec: Field geometry to check against.

    Raises:
        ValueError: If the track is longer than the field or leaves its vertical envelope.
    """
    track.validate()
    if track.length_m > spec.track_length_m:
        raise ValueError(
            f"track '{track.name}' is {track.length_m:.1f} m long, "
            f"field is {spec.track_length_m:.1f} m"
        )
    extent = profile_extent(track, resolution_m=spec.resolution_m)
    if not spec.fits(extent):
        raise ValueError(
            f"track '{track.name}' extent [{extent[0]:+.3f}, {extent[1]:+.3f}] m leaves the "
            f"field envelope [{spec.min_profile_m:+.3f}, {spec.max_profile_m:+.3f}] m"
        )


__all__ = ["HeightFieldSpec", "FIELD", "build_field_data", "assert_track_fits"]
