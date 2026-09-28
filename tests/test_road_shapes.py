"""
Unit tests for the road-scale obstacle shapes and the rolling-wheel envelope.

Tests include:
- Geometry of Bump, TrapezoidBump, SlopedPothole, BowlPothole: extent, endpoint
  continuity, extrema, degenerate parameters.
- RoadRoughness: amplitude, taper, absence from plot markers.
- Wheel envelope: bridged holes match the closed form, long holes reach the floor,
  bumps lift the wheel by exactly their height, flat ground gives zero excursion.

Pure geometry; nothing here compiles a MuJoCo model.
"""

import numpy as np
import pytest

from bike_sim.terrain.obstacles import (
    BUMP_TYPES,
    POTHOLE_TYPES,
    BowlPothole,
    Bump,
    Pothole,
    RoadRoughness,
    SlopedPothole,
    SquareEdge,
    SteppedClimb,
    TrapezoidBump,
)
from bike_sim.terrain.profile import TrackSpec, build_profile
from bike_sim.terrain.wheelpath import (
    FRONT_WHEEL_RADIUS_M,
    REAR_WHEEL_RADIUS_M,
    bridged_drop_m,
    effective_drop_m,
    effective_rise_m,
    wheel_centre_path,
)


def _local(obstacle, n=4001):
    return np.linspace(0.0, obstacle.length_m, n)


# --------------------------------------------------------------------------------------
# Shapes
# --------------------------------------------------------------------------------------


def test_bump_is_a_raised_cosine_tangent_at_both_ends():
    bump = Bump(start_m=10.0, height_m=0.050, bump_length_m=0.400)
    s = _local(bump)
    z = bump.elevation(s)
    slope = np.gradient(z, s)

    assert bump.length_m == pytest.approx(0.400)
    assert bump.datum_shift_m == 0.0
    assert z[0] == pytest.approx(0.0, abs=1e-12)
    assert z[-1] == pytest.approx(0.0, abs=1e-12)
    assert np.max(z) == pytest.approx(0.050, rel=1e-6)
    assert np.argmax(z) == len(z) // 2
    assert abs(slope[0]) < 1e-3 and abs(slope[-1]) < 1e-3
    assert np.all(z >= -1e-12)


def test_trapezoid_bump_has_ramps_and_a_plateau():
    bump = TrapezoidBump(start_m=0.0, height_m=0.060, ramp_m=0.150, plateau_m=0.300)
    s = _local(bump, n=6001)
    z = bump.elevation(s)

    assert bump.length_m == pytest.approx(0.600)
    assert z[0] == pytest.approx(0.0, abs=1e-12)
    assert z[-1] == pytest.approx(0.0, abs=1e-9)
    on_plateau = (s > 0.150 + 1e-9) & (s < 0.450 - 1e-9)
    assert np.allclose(z[on_plateau], 0.060)
    rising = s < 0.150
    assert np.allclose(np.diff(z[rising]), np.diff(z[rising])[0])
    assert np.max(z) == pytest.approx(0.060)


def test_trapezoid_bump_with_zero_plateau_is_a_triangle():
    bump = TrapezoidBump(start_m=0.0, height_m=0.040, ramp_m=0.200, plateau_m=0.0)
    z = bump.elevation(_local(bump))

    assert bump.length_m == pytest.approx(0.400)
    assert np.max(z) == pytest.approx(0.040)
    assert np.sum(np.isclose(z, 0.040)) == 1


def test_trapezoid_bump_with_zero_ramp_is_a_square_edge():
    bump = TrapezoidBump(start_m=0.0, height_m=0.040, ramp_m=0.0, plateau_m=0.300)
    edge = SquareEdge(start_m=0.0, height_m=0.040, ledge_length_m=0.300)
    s = _local(bump)

    assert np.allclose(bump.elevation(s), edge.elevation(s))


def test_sloped_pothole_has_chamfers_and_a_flat_floor():
    hole = SlopedPothole(start_m=0.0, depth_m=0.080, hole_length_m=0.500, edge_m=0.100)
    s = _local(hole, n=5001)
    z = hole.elevation(s)

    assert hole.length_m == pytest.approx(0.500)
    assert hole.effective_edge_m == pytest.approx(0.100)
    assert z[0] == pytest.approx(0.0, abs=1e-12)
    assert z[-1] == pytest.approx(0.0, abs=1e-9)
    on_floor = (s > 0.100 + 1e-9) & (s < 0.400 - 1e-9)
    assert np.allclose(z[on_floor], -0.080)
    assert np.min(z) == pytest.approx(-0.080)
    assert np.all(z <= 1e-12)


def test_sloped_pothole_clamps_chamfers_into_a_v():
    hole = SlopedPothole(start_m=0.0, depth_m=0.080, hole_length_m=0.300, edge_m=0.500)
    z = hole.elevation(_local(hole))

    assert hole.effective_edge_m == pytest.approx(0.150)
    assert np.min(z) == pytest.approx(-0.080)
    assert np.sum(np.isclose(z, -0.080)) == 1


def test_sloped_pothole_with_zero_edge_is_a_pothole():
    sloped = SlopedPothole(start_m=0.0, depth_m=0.080, hole_length_m=0.500, edge_m=0.0)
    sharp = Pothole(start_m=0.0, depth_m=0.080, hole_length_m=0.500)
    s = _local(sloped)

    assert np.allclose(sloped.elevation(s), sharp.elevation(s))


def test_bowl_pothole_is_a_raised_cosine_dip():
    hole = BowlPothole(start_m=0.0, depth_m=0.080, hole_length_m=0.500)
    s = _local(hole)
    z = hole.elevation(s)
    slope = np.gradient(z, s)

    assert z[0] == pytest.approx(0.0, abs=1e-12)
    assert z[-1] == pytest.approx(0.0, abs=1e-12)
    assert np.min(z) == pytest.approx(-0.080, rel=1e-6)
    assert abs(slope[0]) < 1e-3 and abs(slope[-1]) < 1e-3


def test_type_groups_cover_the_road_shapes():
    assert set(POTHOLE_TYPES) == {Pothole, SlopedPothole, BowlPothole}
    assert set(BUMP_TYPES) == {Bump, TrapezoidBump}


def test_stepped_climb_endpoints_and_monotonicity():
    climb = SteppedClimb(
        start_m=0.0,
        steps=((5.0, 15.0), (10.0, 15.0), (15.0, 15.0), (20.0, 15.0), (25.0, 15.0)),
        transition_m=3.0,
    )
    assert climb.length_m == pytest.approx(90.0)
    s = _local(climb, n=9001)
    z = climb.elevation(s)

    assert z[0] == pytest.approx(0.0, abs=1e-12)
    assert z[-1] == pytest.approx(climb.datum_shift_m, abs=1e-9)
    assert climb.datum_shift_m > 10.0
    # Elevation is monotonically non-decreasing
    assert np.all(np.diff(z) >= -1e-12)

    # Derivative / slope is continuous and non-negative
    slope = np.gradient(z, s)
    assert np.all(slope >= -1e-6)
    assert slope[0] == pytest.approx(0.0, abs=1e-3)


def test_stepped_climb_in_build_profile():
    climb = SteppedClimb(
        start_m=10.0,
        steps=((5.0, 10.0), (10.0, 10.0)),
        transition_m=2.0,
    )
    track = TrackSpec(name="climb_profile_test", length_m=50.0, obstacles=[climb])
    track.validate()
    x = np.arange(0.0, 50.0, 0.01)
    z = build_profile(track, x)

    # Flat before climb
    assert np.allclose(z[x < 10.0], 0.0)
    # Flat runout after climb at datum_shift_m
    assert np.allclose(z[x >= 10.0 + climb.length_m], climb.datum_shift_m)


# --------------------------------------------------------------------------------------
# Road roughness
# --------------------------------------------------------------------------------------


def test_road_roughness_holds_amplitude_and_joins_the_datum():
    rough = RoadRoughness(start_m=0.0, section_length_m=10.0, amplitude_m=0.003, seed=7)
    s = _local(rough, n=20001)
    z = rough.elevation(s)

    assert np.max(np.abs(z)) == pytest.approx(0.003, rel=1e-6)
    assert z[0] == pytest.approx(0.0, abs=1e-12)
    assert z[-1] == pytest.approx(0.0, abs=1e-12)


def test_road_roughness_is_seed_deterministic_and_grid_independent():
    a = RoadRoughness(start_m=0.0, section_length_m=10.0, amplitude_m=0.003, seed=3)
    b = RoadRoughness(start_m=0.0, section_length_m=10.0, amplitude_m=0.003, seed=3)
    c = RoadRoughness(start_m=0.0, section_length_m=10.0, amplitude_m=0.003, seed=4)
    coarse = np.arange(0.0, 10.0, 0.005)

    assert np.array_equal(a.elevation(coarse), b.elevation(coarse))
    assert not np.allclose(a.elevation(coarse), c.elevation(coarse))
    assert np.allclose(a.elevation(coarse), a.elevation(np.arange(0.0, 10.0, 0.0025))[::2])


def test_road_roughness_is_left_out_of_markers_but_kept_in_the_profile():
    track = TrackSpec(
        name="t",
        length_m=40.0,
        obstacles=[
            RoadRoughness(start_m=5.0, section_length_m=10.0, amplitude_m=0.003, seed=1),
            Pothole(start_m=20.0, depth_m=0.080, hole_length_m=0.500),
        ],
    )
    track.validate()
    x = np.arange(0.0, 40.0, 0.005)
    z = build_profile(track, x)

    assert [label for _, label in track.markers] == ["Pothole@20m"]
    assert np.max(np.abs(z[(x > 5.0) & (x < 15.0)])) == pytest.approx(0.003, rel=1e-3)


# --------------------------------------------------------------------------------------
# Wheel envelope
# --------------------------------------------------------------------------------------


@pytest.mark.parametrize("radius_m", [FRONT_WHEEL_RADIUS_M, REAR_WHEEL_RADIUS_M])
@pytest.mark.parametrize("hole_length_m", [0.300, 0.500, 0.600])
def test_bridged_pothole_drop_matches_the_closed_form(radius_m, hole_length_m):
    """A rectangular hole shorter than the wheel diameter is bridged by the rim."""
    hole = Pothole(start_m=0.0, depth_m=0.500, hole_length_m=hole_length_m)
    expected = bridged_drop_m(hole_length_m, radius_m)

    assert expected < hole.depth_m
    assert effective_drop_m(hole, radius_m) == pytest.approx(expected, abs=5e-4)


def test_short_pothole_drop_is_insensitive_to_declared_depth():
    """This is the fact the generator reports: past the geometric bound depth is moot."""
    shallow = Pothole(start_m=0.0, depth_m=0.040, hole_length_m=0.300)
    deep = Pothole(start_m=0.0, depth_m=0.120, hole_length_m=0.300)

    drop_shallow = effective_drop_m(shallow, FRONT_WHEEL_RADIUS_M)
    drop_deep = effective_drop_m(deep, FRONT_WHEEL_RADIUS_M)

    assert drop_shallow == pytest.approx(0.0316, abs=5e-4)
    assert drop_deep == pytest.approx(drop_shallow, abs=1e-6)


def test_long_pothole_floors_the_wheel():
    hole = Pothole(start_m=0.0, depth_m=0.060, hole_length_m=1.000)

    assert bridged_drop_m(1.000, FRONT_WHEEL_RADIUS_M) == float("inf")
    assert effective_drop_m(hole, FRONT_WHEEL_RADIUS_M) == pytest.approx(0.060, abs=1e-5)


def test_bump_lifts_the_wheel_by_its_height_and_never_drops_it():
    bump = Bump(start_m=0.0, height_m=0.050, bump_length_m=0.400)

    assert effective_rise_m(bump, FRONT_WHEEL_RADIUS_M) == pytest.approx(0.050, abs=1e-5)
    assert effective_drop_m(bump, FRONT_WHEEL_RADIUS_M) == pytest.approx(0.0, abs=1e-6)


def test_wheel_centre_path_is_flat_on_flat_ground_and_spans_the_obstacle():
    hole = Pothole(start_m=0.0, depth_m=0.080, hole_length_m=0.500)
    x, z = wheel_centre_path(hole, FRONT_WHEEL_RADIUS_M)

    assert x[0] == pytest.approx(-FRONT_WHEEL_RADIUS_M)
    assert x[-1] >= hole.length_m + FRONT_WHEEL_RADIUS_M - 0.0005
    assert z[0] == pytest.approx(FRONT_WHEEL_RADIUS_M, abs=1e-6)
    assert z[-1] == pytest.approx(FRONT_WHEEL_RADIUS_M, abs=1e-6)
    assert np.min(z) < FRONT_WHEEL_RADIUS_M


def test_bowl_pothole_is_gentler_on_the_wheel_than_a_sharp_one_of_equal_size():
    sharp = Pothole(start_m=0.0, depth_m=0.080, hole_length_m=0.500)
    bowl = BowlPothole(start_m=0.0, depth_m=0.080, hole_length_m=0.500)

    assert effective_drop_m(bowl, FRONT_WHEEL_RADIUS_M) < effective_drop_m(
        sharp, FRONT_WHEEL_RADIUS_M
    )
