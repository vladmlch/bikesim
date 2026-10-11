"""
Single-Contact Geometry Guards for the ``compliant_2d`` Tyre.

The 2D tyre rests on one support. Two significant supports whose normals differ by more
than ``distinct_normal_deg`` (20 degrees) end a strict run as ``multi_support``. Two
closed-form guards reproduce where that happens
(docs/superpowers/specs/2026-10-11-alpine-scree-track-design.md, section 4):

- smooth raised-cosine shapes (bump, bowl) must keep their radius of curvature at or above
  ``min_radius_m`` -- a crest or trough tighter than the wheel is bridged by two rim points;
- faceted shapes (RockGarden, RoadRoughness) must keep the concave slope increase across any
  ``window_m`` stretch below ``max_kink_deg`` -- the chord the contact patch covers,
  ``2 sqrt(2 R delta)``, is about 0.24 m at 20 mm penetration.

Obstacles with vertical or linear faces are hazards by construction. Pure geometry: no MuJoCo.
"""
from math import pi, sqrt
from typing import List

import numpy as np

from bike_sim.terrain.obstacles import BowlPothole, Bump, GOut, RockGarden
from bike_sim.terrain.profile import TrackSpec

DEFAULT_MIN_CONTACT_RADIUS_M = .45
DEFAULT_MAX_KINK_DEG = 17.
DEFAULT_CONTACT_WINDOW_M = .25


def min_bump_length_m(height_m: float, min_radius_m: float = DEFAULT_MIN_CONTACT_RADIUS_M) -> float:
    """Shortest raised-cosine bump of ``height_m`` whose crest radius stays at or above ``min_radius_m``.

    The crest curvature of ``h/2 (1 - cos(2 pi s / L))`` is ``2 pi^2 h / L^2``.
    """
    return pi * sqrt(2. * height_m * min_radius_m)


def max_bowl_depth_m(length_m: float, min_radius_m: float = DEFAULT_MIN_CONTACT_RADIUS_M) -> float:
    """Deepest raised-cosine bowl of ``length_m`` whose trough radius stays at or above ``min_radius_m``."""
    return length_m ** 2 / (2. * pi ** 2 * min_radius_m)


def concave_span_deg(obstacle: RockGarden, window_m: float = DEFAULT_CONTACT_WINDOW_M) -> float:
    """Largest slope increase between two facets at most ``window_m`` apart, in degrees.

    The joins to flat ground at both ends are included. Convex crests do not count: a wheel
    on a crest has a single support.
    """
    s, z = obstacle._nodes()
    s = np.r_[s[0] - 1., s, s[-1] + 1.]
    z = np.r_[0., z, 0.]
    mid = (s[:-1] + s[1:]) / 2.
    slopes = np.degrees(np.arctan(np.diff(z) / np.diff(s)))
    worst = 0.
    for i in range(len(slopes) - 1):
        ahead = slopes[(mid > mid[i]) & (mid - mid[i] <= window_m)]
        if len(ahead):
            worst = max(worst, float(np.max(ahead) - slopes[i]))
    return worst


def single_contact_hazards(track: TrackSpec, *, min_radius_m: float = DEFAULT_MIN_CONTACT_RADIUS_M,
                           max_kink_deg: float = DEFAULT_MAX_KINK_DEG,
                           window_m: float = DEFAULT_CONTACT_WINDOW_M) -> List[str]:
    """Obstacles of ``track`` that can put the 2D tyre on two supports; an empty list means clean."""
    hazards = []
    for o in track.sorted_obstacles:
        if isinstance(o, RockGarden):
            span = concave_span_deg(o, window_m)
            if span >= max_kink_deg:
                hazards.append(f'{o.label}: concave span {span:.1f} deg >= {max_kink_deg:g} deg')
        elif isinstance(o, Bump):
            need = min_bump_length_m(o.height_m, min_radius_m)
            if o.bump_length_m < need - 1e-9:
                hazards.append(f'{o.label}: length {o.bump_length_m:.3f} m < {need:.3f} m')
        elif isinstance(o, (BowlPothole, GOut)):
            length = o.hole_length_m if isinstance(o, BowlPothole) else o.dip_length_m
            limit = max_bowl_depth_m(length, min_radius_m)
            if o.depth_m > limit + 1e-9:
                hazards.append(f'{o.label}: depth {o.depth_m:.3f} m > {limit:.3f} m')
        else:
            hazards.append(f'{o.label}: {type(o).__name__} has faces the 2D tyre cannot ride')
    return hazards


__all__ = [
    'DEFAULT_MIN_CONTACT_RADIUS_M', 'DEFAULT_MAX_KINK_DEG', 'DEFAULT_CONTACT_WINDOW_M',
    'min_bump_length_m', 'max_bowl_depth_m', 'concave_span_deg', 'single_contact_hazards',
]
