"""
Radial-Element Geometry: the Tyre's Rays Against the Road.

Each wheel carries `n` rays from the hub, fixed in the world frame and spread uniformly over
+/-`half_angle` about world -Z (docs/RIDE.md section 3.1). For every ray this module finds
the distance `r_i` from the hub to the first **visible** road point along it; the element
deflection is `R - r_i` where that is positive. Contiguous runs of loaded rays are the
contact patches.

The road is the 1-D profile the heightfield is rasterised from, a polyline on a uniform
grid (`RoadProfile`). A ray's first crossing of that polyline is by construction the
visible one: road hidden behind a nearer part of the profile along the same ray -- the back
face of a descent -- is never reached.

**How it is fast.** Only profile samples within `R` of the hub can load the tyre. The
window from the first to the last such sample, one neighbour either side, is sliced out
(a view, no copy). Seen from the hub, a visible profile has monotonically increasing polar
angle along that window, so each ray's segment is found by `searchsorted` and intersected
exactly. When the window is not monotone -- a back face is inside the circle -- the exact
per-ray, per-segment minimum is taken instead. That is slower, but it only happens in the
aftermath of a sharp descent. A wheel with no sample inside its circle is airborne and costs
one distance test.

Angles: `theta` is measured from world -Z towards +X (the front), so the ray at `theta` has
direction `(sin theta, -cos theta)` in world (x, z).
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

# Profile samples whose polar angle falls outside the coverage by more than this are
# reported as a coverage event rather than silently ignored.
COVERAGE_SLACK_RAD = 1e-9


@dataclass(frozen=True)
class RoadProfile:
    """
    The road as a polyline on a uniform longitudinal grid, in world coordinates.

    Attributes:
        x0_m: World X of the first sample.
        dx_m: Grid spacing.
        z_m: World Z of the road surface at each sample.
    """

    x0_m: float
    dx_m: float
    z_m: np.ndarray

    @classmethod
    def from_samples(cls, x_m: np.ndarray, z_m: np.ndarray) -> "RoadProfile":
        """
        Builds a profile from uniformly spaced samples.

        Raises:
            ValueError: If the samples are not uniformly spaced, too few, or mismatched.
        """
        x_m = np.asarray(x_m, dtype=float)
        z_m = np.ascontiguousarray(z_m, dtype=float)
        if x_m.shape != z_m.shape or x_m.ndim != 1 or x_m.size < 2:
            raise ValueError("profile needs matching 1-D x and z arrays of at least two samples")
        steps = np.diff(x_m)
        dx = float(steps[0])
        if not dx > 0.0 or not np.allclose(steps, dx, rtol=1e-9, atol=1e-12):
            raise ValueError("profile samples must be uniformly spaced and increasing")
        return cls(x0_m=float(x_m[0]), dx_m=dx, z_m=z_m)

    @property
    def x_m(self) -> np.ndarray:
        """World X of every sample."""
        return self.x0_m + self.dx_m * np.arange(self.z_m.size)

    def height_at(self, x_m):
        """Linearly interpolated road height at world X (clamped at the ends)."""
        return np.interp(x_m, self.x_m, self.z_m)

    def index_range(self, x_lo_m: float, x_hi_m: float) -> Tuple[int, int]:
        """Sample indices [lo, hi) covering an X interval, clamped to the profile."""
        lo = int(np.floor((x_lo_m - self.x0_m) / self.dx_m))
        hi = int(np.ceil((x_hi_m - self.x0_m) / self.dx_m)) + 1
        n = self.z_m.size
        return max(0, lo), min(n, hi)


class RayRing:
    """
    The world-fixed ray directions of one tyre.

    Attributes:
        theta_rad: Ray angles from -Z towards +X, increasing, shape (n,).
        ux, uz: World components of each ray's unit direction.
        dtheta_rad: Uniform angular spacing.
        half_angle_rad: Coverage either side of the vertical.
    """

    def __init__(self, n_rays: int, half_angle_rad: float) -> None:
        """
        Args:
            n_rays: Number of rays; at least 3.
            half_angle_rad: Coverage either side of -Z, below pi/2.

        Raises:
            ValueError: On too few rays or a coverage reaching the horizontal.
        """
        if n_rays < 3:
            raise ValueError(f"a ray ring needs at least 3 rays, got {n_rays}")
        if not 0.0 < half_angle_rad < 0.5 * np.pi:
            raise ValueError(f"half angle must be inside (0, pi/2), got {half_angle_rad}")
        self.n = int(n_rays)
        self.half_angle_rad = float(half_angle_rad)
        self.theta_rad = np.linspace(-half_angle_rad, half_angle_rad, n_rays)
        self.dtheta_rad = float(self.theta_rad[1] - self.theta_rad[0])
        self.ux = np.sin(self.theta_rad)
        self.uz = -np.cos(self.theta_rad)


@dataclass
class RayHits:
    """
    Where each ray meets the road this step.

    Attributes:
        r_m: Distance from the hub to the first visible road point, `inf` where the ray
            meets no road within the tyre radius.
        delta_m: Element deflection `max(0, R - r)`.
        road_x_m, road_z_m: World position of each ray's road point (hub + R.u where the
            ray is unloaded).
        coverage_event: Road inside the tyre circle but outside the ray coverage.
        airborne: No profile sample inside the tyre circle.
    """

    r_m: np.ndarray
    delta_m: np.ndarray
    road_x_m: np.ndarray
    road_z_m: np.ndarray
    coverage_event: bool
    airborne: bool


def _cross(ax, az, bx, bz):
    return ax * bz - az * bx


def intersect(ring: RayRing, centre_x_m: float, centre_z_m: float, radius_m: float,
              road: RoadProfile) -> RayHits:
    """
    Finds where every ray of a tyre meets the road.

    Args:
        ring: The tyre's rays.
        centre_x_m, centre_z_m: World position of the hub.
        radius_m: Unloaded tyre radius `R`.
        road: The road profile.

    Returns:
        Per-ray distances, deflections and road points, and the step's flags.
    """
    n = ring.n
    r = np.full(n, np.inf)
    lo, hi = road.index_range(centre_x_m - radius_m, centre_x_m + radius_m)
    if hi - lo < 2:
        return _hits(ring, centre_x_m, centre_z_m, radius_m, r, False, True)

    zs = road.z_m[lo:hi]
    xs = road.x0_m + road.dx_m * np.arange(lo, hi)
    dxs = xs - centre_x_m
    dzs = centre_z_m - zs                      # positive below the hub
    inside = dxs * dxs + dzs * dzs < radius_m * radius_m
    if not inside.any():
        return _hits(ring, centre_x_m, centre_z_m, radius_m, r, False, True)

    idx = np.flatnonzero(inside)
    a = max(int(idx[0]) - 1, 0)
    b = min(int(idx[-1]) + 2, xs.size)
    px, pz = xs[a:b], zs[a:b]
    phi = np.arctan2(px - centre_x_m, centre_z_m - pz)

    phi_in = phi[inside[a:b]]
    coverage = bool(
        np.any(np.abs(phi_in) > ring.half_angle_rad + COVERAGE_SLACK_RAD) or np.any(pz[inside[a:b]] >= centre_z_m)
    )

    if px.size >= 2 and np.all(np.diff(phi) > 0.0):
        j = np.searchsorted(phi, ring.theta_rad, side="right") - 1
        valid = (j >= 0) & (j < px.size - 1)
        jj = np.clip(j, 0, px.size - 2)
        s = _segment_distance(ring.ux, ring.uz, centre_x_m, centre_z_m,
                              px[jj], pz[jj], px[jj + 1], pz[jj + 1])
        r = np.where(valid & (s > 0.0), s, np.inf)
    else:
        r = _brute_force(ring, centre_x_m, centre_z_m, px, pz)

    return _hits(ring, centre_x_m, centre_z_m, radius_m, r, coverage, False)


def _segment_distance(ux, uz, cx, cz, x1, z1, x2, z2):
    """Distance along each ray to the line through its segment (inf where parallel)."""
    dx, dz = x2 - x1, z2 - z1
    denom = _cross(ux, uz, dx, dz)
    with np.errstate(divide="ignore", invalid="ignore"):
        s = _cross(x1 - cx, z1 - cz, dx, dz) / denom
    return np.where(np.abs(denom) > 1e-15, s, np.inf)


def _brute_force(ring: RayRing, cx: float, cz: float, px: np.ndarray, pz: np.ndarray) -> np.ndarray:
    """Exact first crossing of every ray with every window segment, for non-monotone windows."""
    x1, z1, x2, z2 = px[:-1], pz[:-1], px[1:], pz[1:]
    dx, dz = x2 - x1, z2 - z1
    ux, uz = ring.ux[:, None], ring.uz[:, None]
    denom = _cross(ux, uz, dx[None, :], dz[None, :])
    wx, wz = (x1 - cx)[None, :], (z1 - cz)[None, :]
    with np.errstate(divide="ignore", invalid="ignore"):
        s = _cross(wx, wz, dx[None, :], dz[None, :]) / denom
        t = _cross(wx, wz, ux, uz) / denom
    ok = (np.abs(denom) > 1e-15) & (s > 0.0) & (t >= 0.0) & (t <= 1.0)
    s = np.where(ok, s, np.inf)
    return s.min(axis=1)


def _hits(ring: RayRing, cx: float, cz: float, radius: float, r: np.ndarray,
          coverage: bool, airborne: bool) -> RayHits:
    delta = np.maximum(radius - r, 0.0)
    reach = np.minimum(r, radius)
    return RayHits(
        r_m=r,
        delta_m=delta,
        road_x_m=cx + reach * ring.ux,
        road_z_m=cz + reach * ring.uz,
        coverage_event=coverage,
        airborne=airborne,
    )


def patches(delta_m: np.ndarray) -> List[Tuple[int, int]]:
    """
    Splits the loaded rays into contiguous patches.

    Args:
        delta_m: Element deflections.

    Returns:
        ``(start, stop)`` index pairs, ``stop`` exclusive, in ray order (rear to front).
    """
    loaded = np.flatnonzero(delta_m > 0.0)
    if loaded.size == 0:
        return []
    breaks = np.flatnonzero(np.diff(loaded) > 1)
    starts = np.concatenate(([loaded[0]], loaded[breaks + 1]))
    stops = np.concatenate((loaded[breaks] + 1, [loaded[-1] + 1]))
    return [(int(a), int(b)) for a, b in zip(starts, stops)]


def first_crossing_reference(theta_rad: float, cx: float, cz: float, radius: float,
                             road: RoadProfile, samples: int = 20000) -> Optional[float]:
    """
    Slow reference: the first road crossing along one ray, by scan and bisection.

    Used by the tests as the oracle for `intersect`.

    Returns:
        The distance along the ray, or None if the ray meets no road within `radius`.
    """
    ux, uz = np.sin(theta_rad), -np.cos(theta_rad)

    def below(s):
        return cz + s * uz <= road.height_at(cx + s * ux)

    s_grid = np.linspace(0.0, radius, samples)
    hit = below(s_grid)
    if not hit.any():
        return None
    k = int(np.argmax(hit))
    if k == 0:
        return 0.0
    a, b = s_grid[k - 1], s_grid[k]
    for _ in range(60):
        m = 0.5 * (a + b)
        if below(m):
            b = m
        else:
            a = m
    return 0.5 * (a + b)


__all__ = ["RoadProfile", "RayRing", "RayHits", "intersect", "patches", "first_crossing_reference"]
