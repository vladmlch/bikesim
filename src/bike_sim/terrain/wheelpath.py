"""
Rolling-Wheel Envelope Over an Obstacle.

Answers the question the road profile alone cannot: how far does a wheel actually drop
into a hole, or rise over a bump? A rigid wheel of radius ``r`` does not follow the
surface; its centre follows the upper envelope of circles of radius ``r`` resting on the
profile. For a hole shorter than ``2r`` the wheel bridges the gap and never reaches the
floor, so the declared depth is not what the suspension sees.

The computation is purely geometric and shape-agnostic: it samples the profile, then for
every wheel-centre position takes the highest point the rim can rest on. The envelope of
two circles meeting over a hole has a kink at its minimum, so the error is first order in
the sampling interval; 0.5 mm keeps it under half a millimetre for any hole the wheel can
bridge. The obstacle's exact boundary points are always included among the samples so a
vertical edge is rested on exactly, not at the nearest grid node.
"""

from typing import Tuple
import numpy as np

from bike_sim.terrain.obstacles import Obstacle

DEFAULT_RESOLUTION_M = 0.0005
FRONT_WHEEL_RADIUS_M = 0.372
REAR_WHEEL_RADIUS_M = 0.352
_CHUNK_ROWS = 256


def wheel_centre_path(
    obstacle: Obstacle,
    wheel_radius_m: float,
    resolution_m: float = DEFAULT_RESOLUTION_M,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Computes the path of a rigid wheel's centre rolling across one obstacle.

    The obstacle is embedded in flat ground at the entry datum; downstream datum shifts
    are ignored because the question is about the local excursion, not the road level.

    Args:
        obstacle: Obstacle to roll over.
        wheel_radius_m: Wheel radius in metres.
        resolution_m: Sampling interval in metres.

    Returns:
        Tuple of (x, z_centre): wheel-centre longitudinal positions spanning the obstacle
        plus one radius either side, and the centre height above the entry datum. On flat
        ground ``z_centre == wheel_radius_m``.
    """
    r = float(wheel_radius_m)
    length = float(obstacle.length_m)

    s_grid = np.arange(-2.0 * r, length + 2.0 * r + resolution_m, resolution_m)
    s_edges = np.array([-1e-9, 0.0, max(length - 1e-9, 0.0), length])
    s_surface = np.unique(np.concatenate([s_grid, s_edges]))
    z_surface = np.zeros_like(s_surface)
    inside = (s_surface >= 0.0) & (s_surface < length)
    if np.any(inside):
        z_surface[inside] = obstacle.elevation(s_surface[inside])

    x_centre = np.arange(-r, length + r + resolution_m, resolution_m)
    z_centre = np.empty_like(x_centre)

    for lo in range(0, len(x_centre), _CHUNK_ROWS):
        hi = min(lo + _CHUNK_ROWS, len(x_centre))
        dx = x_centre[lo:hi, None] - s_surface[None, :]
        reach = r * r - dx * dx
        lift = np.where(reach >= 0.0, np.sqrt(np.clip(reach, 0.0, None)), -np.inf)
        z_centre[lo:hi] = np.max(z_surface[None, :] + lift, axis=1)

    return x_centre, z_centre


def effective_drop_m(
    obstacle: Obstacle,
    wheel_radius_m: float,
    resolution_m: float = DEFAULT_RESOLUTION_M,
) -> float:
    """
    Computes how far a rigid wheel's centre drops below flat-ground height over an obstacle.

    Args:
        obstacle: Obstacle to roll over.
        wheel_radius_m: Wheel radius in metres.
        resolution_m: Sampling interval in metres.

    Returns:
        Maximum drop of the wheel centre in metres; zero for obstacles the wheel bridges
        or rides over without descending.
    """
    _, z_centre = wheel_centre_path(obstacle, wheel_radius_m, resolution_m)
    return max(0.0, float(wheel_radius_m - np.min(z_centre)))


def effective_rise_m(
    obstacle: Obstacle,
    wheel_radius_m: float,
    resolution_m: float = DEFAULT_RESOLUTION_M,
) -> float:
    """
    Computes how far a rigid wheel's centre rises above flat-ground height over an obstacle.

    Args:
        obstacle: Obstacle to roll over.
        wheel_radius_m: Wheel radius in metres.
        resolution_m: Sampling interval in metres.

    Returns:
        Maximum rise of the wheel centre in metres; for any bump this equals its height.
    """
    _, z_centre = wheel_centre_path(obstacle, wheel_radius_m, resolution_m)
    return max(0.0, float(np.max(z_centre) - wheel_radius_m))


def bridged_drop_m(hole_length_m: float, wheel_radius_m: float) -> float:
    """
    Closed-form drop of a wheel into a rectangular hole it cannot floor.

    Args:
        hole_length_m: Hole length in metres.
        wheel_radius_m: Wheel radius in metres.

    Returns:
        ``r - sqrt(r^2 - (w/2)^2)`` in metres, or ``inf`` when the hole is at least one
        wheel diameter long and the wheel reaches the floor.
    """
    half = hole_length_m / 2.0
    if half >= wheel_radius_m:
        return float("inf")
    return float(wheel_radius_m - np.sqrt(wheel_radius_m**2 - half**2))


__all__ = [
    "DEFAULT_RESOLUTION_M",
    "FRONT_WHEEL_RADIUS_M",
    "REAR_WHEEL_RADIUS_M",
    "wheel_centre_path",
    "effective_drop_m",
    "effective_rise_m",
    "bridged_drop_m",
]
