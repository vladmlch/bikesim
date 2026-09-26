"""
Tests for the tyre's ray geometry (docs/RIDE.md section 3.1, *Radial elements*).

`intersect` is checked against a slow scan-and-bisect oracle along every ray, on the real
obstacle shapes the tracks are built from, at hub heights from light contact to near rim
strike. The fast path (monotone window) and the exact fallback (a back face inside the
circle) must both agree with it.

Measured on the development Mac (M4 Max), 64 rays, loaded wheel: about 15 us per call on
flat road and 20 us at a square edge; 256 rays about 25 / 35 us. The airborne cull costs
about 5 us. Those numbers sit well inside the 120 us per wheel budget (plan, Measured
baseline); the benchmark test below only guards against an order-of-magnitude regression.
"""

import time

import numpy as np
import pytest

from bike_sim.sim.ride.tyre.geometry import (
    RayRing,
    RoadProfile,
    first_crossing_reference,
    intersect,
    patches,
)
from bike_sim.terrain import BowlPothole, Drop, Pothole, SquareEdge, TrackSpec, Washboard, build_profile

R_FRONT = 0.372
DX = 0.005
GROUND_Z = -0.3495


def _road(*obstacles, length=30.0):
    track = TrackSpec(name="t", length_m=length, obstacles=list(obstacles))
    x = np.arange(0.0, length + DX / 2, DX)
    return RoadProfile.from_samples(x, GROUND_Z + build_profile(track, x))


ROADS = {
    "flat": _road(),
    "square_edge": _road(SquareEdge(start_m=10.0, height_m=0.090, ledge_length_m=0.250)),
    "sharp_pothole": _road(Pothole(start_m=10.0, depth_m=0.150, hole_length_m=0.600)),
    "bowl_pothole": _road(BowlPothole(start_m=10.0, depth_m=0.080, hole_length_m=0.5)),
    "washboard": _road(Washboard(start_m=9.0, amplitude_m=0.035, wavelength_m=0.9, n_waves=4)),
    "drop": _road(Drop(start_m=10.0, height_m=0.300, ramp_m=0.0)),
}


def _hub_height(road, x, deflection):
    """Hub height putting the tyre `deflection` into the highest road point below it."""
    lo, hi = road.index_range(x - 0.1, x + 0.1)
    return float(np.max(road.z_m[lo:hi])) + R_FRONT - deflection


def _check_against_oracle(ring, road, cx, cz, radius=R_FRONT):
    hits = intersect(ring, cx, cz, radius, road)
    for i, theta in enumerate(ring.theta_rad):
        ref = first_crossing_reference(theta, cx, cz, radius, road)
        if ref is None or ref >= radius:
            assert hits.delta_m[i] == 0.0 or hits.delta_m[i] < 1e-4, (i, theta, hits.r_m[i], ref)
        else:
            assert hits.r_m[i] == pytest.approx(ref, abs=1e-4), (i, np.degrees(theta), hits.r_m[i], ref)
    return hits


@pytest.mark.parametrize("n_rays", [64, 256])
@pytest.mark.parametrize("name", list(ROADS))
def test_intersect_matches_the_oracle(name, n_rays):
    ring = RayRing(n_rays, np.radians(75.0))
    road = ROADS[name]
    for x in (9.7, 9.95, 10.02, 10.2, 10.45, 10.62):
        for deflection in (0.002, 0.012, 0.035):
            _check_against_oracle(ring, road, x, _hub_height(road, x, deflection))


def test_flat_road_one_patch_with_the_circle_chord():
    ring = RayRing(256, np.radians(75.0))
    road = ROADS["flat"]
    delta = 0.008
    hits = intersect(ring, 5.0, GROUND_Z + R_FRONT - delta, R_FRONT, road)
    ps = patches(hits.delta_m)
    assert len(ps) == 1
    a, b = ps[0]
    span = hits.road_x_m[b - 1] - hits.road_x_m[a]
    chord = 2.0 * np.sqrt(2.0 * R_FRONT * delta - delta * delta)
    assert span == pytest.approx(chord, abs=2 * R_FRONT * ring.dtheta_rad)
    # An even ray count has no ray at exactly 0 degrees; the nearest is half a spacing off.
    assert hits.delta_m.max() == pytest.approx(delta, abs=R_FRONT * ring.dtheta_rad ** 2)
    assert not hits.coverage_event and not hits.airborne


def test_square_edge_gives_two_patches_while_climbing():
    ring = RayRing(256, np.radians(75.0))
    road = ROADS["square_edge"]
    # Wheel on the flat, pressed into the edge corner ahead of it.
    cx = 10.0 - 0.20
    cz = GROUND_Z + R_FRONT - 0.010
    hits = _check_against_oracle(ring, road, cx, cz)
    ps = patches(hits.delta_m)
    assert len(ps) == 2
    # The front patch sits on the corner, well forward of the vertical.
    front = ps[1]
    assert ring.theta_rad[front[0]] > np.radians(20.0)


def test_airborne_wheel_has_no_contact():
    ring = RayRing(64, np.radians(75.0))
    hits = intersect(ring, 5.0, GROUND_Z + R_FRONT + 0.05, R_FRONT, ROADS["flat"])
    assert hits.airborne and patches(hits.delta_m) == []
    assert np.all(hits.delta_m == 0.0)


def test_far_edge_of_a_150_mm_pothole_is_inside_the_coverage():
    ring = RayRing(256, np.radians(75.0))
    road = ROADS["sharp_pothole"]           # 600 mm long, 150 mm deep, 10.0-10.6 m
    # Wheel dropped into the hole against its far wall: the corner is ~55 degrees forward.
    cx = 10.6 - 0.26
    lo, hi = road.index_range(cx - 0.3, cx + 0.3)
    cz = GROUND_Z - 0.150 + np.sqrt(R_FRONT ** 2 - 0.26 ** 2) + 0.03
    hits = _check_against_oracle(ring, road, cx, cz)
    assert patches(hits.delta_m)
    assert not hits.coverage_event


def test_road_beyond_the_coverage_is_reported():
    ring = RayRing(64, np.radians(75.0))
    wall = _road(SquareEdge(start_m=10.0, height_m=0.35, ledge_length_m=1.0))
    # A 350 mm wall touching the tyre almost level with the hub.
    hits = intersect(ring, 10.0 - 0.36, GROUND_Z + R_FRONT - 0.001, R_FRONT, wall)
    assert hits.coverage_event


def test_patches_splits_runs():
    delta = np.array([0, 0.1, 0.2, 0, 0, 0.3, 0.1, 0, 0.2])
    assert patches(delta) == [(1, 3), (5, 7), (8, 9)]
    assert patches(np.zeros(5)) == []


def test_profile_validation():
    with pytest.raises(ValueError, match="uniformly spaced"):
        RoadProfile.from_samples(np.array([0.0, 0.1, 0.3]), np.zeros(3))
    with pytest.raises(ValueError, match="matching"):
        RoadProfile.from_samples(np.zeros(3), np.zeros(4))
    with pytest.raises(ValueError, match="at least 3 rays"):
        RayRing(2, 1.0)
    with pytest.raises(ValueError, match="half angle"):
        RayRing(10, 2.0)


def test_cost_per_call_is_microseconds():
    ring = RayRing(64, np.radians(75.0))
    road = ROADS["square_edge"]
    cz = GROUND_Z + R_FRONT - 0.010
    n = 2000
    t0 = time.perf_counter()
    for k in range(n):
        intersect(ring, 9.5 + 0.0005 * k, cz, R_FRONT, road)
    per_call_us = 1e6 * (time.perf_counter() - t0) / n
    print(f"intersect, 64 rays: {per_call_us:.1f} us/call")
    assert per_call_us < 500.0
