"""
Unit tests for the road profile and heightfield package.

Tests include:
- Geometry of every obstacle primitive: extent, endpoint continuity, extrema.
- Grid independence and seed determinism of the pseudo-random sections.
- Profile assembly: datum carry across descending obstacles, overlap rejection.
- Track presets: validation, independence between calls, expected layout.
- Heightfield rasterization: resolution, normalization round-trip, envelope enforcement.

The whole package is pure geometry, so nothing here compiles a MuJoCo model.
"""

import numpy as np
import pytest

from bike_sim.terrain.heightfield import (
    FIELD,
    HeightFieldSpec,
    assert_track_fits,
    build_field_data,
)
from bike_sim.terrain.obstacles import (
    Drop,
    GOut,
    Kicker,
    Pothole,
    RockGarden,
    Roots,
    SquareEdge,
    Washboard,
)
from bike_sim.terrain.presets import (
    PRESETS,
    available_presets,
    enduro_aggressive,
    get_preset,
)
from bike_sim.terrain.profile import TrackSpec, build_profile, profile_extent


GROUND_Z_M = -0.3495  # compute_ground_z(BikeSpecs()) / 1000


def _local(obstacle, n=2001):
    """Dense local sample grid spanning one obstacle."""
    return np.linspace(0.0, obstacle.length_m, n)


# --------------------------------------------------------------------------------------
# Obstacle primitives
# --------------------------------------------------------------------------------------


def test_square_edge_is_a_constant_ledge():
    edge = SquareEdge(start_m=10.0, height_m=0.090, ledge_length_m=0.250)
    z = edge.elevation(_local(edge))

    assert edge.length_m == pytest.approx(0.250)
    assert edge.end_m == pytest.approx(10.250)
    assert edge.datum_shift_m == 0.0
    assert np.allclose(z, 0.090)


def test_pothole_is_a_constant_depression():
    hole = Pothole(start_m=5.0, depth_m=0.180, hole_length_m=0.600)
    z = hole.elevation(_local(hole))

    assert hole.length_m == pytest.approx(0.600)
    assert np.allclose(z, -0.180)
    assert hole.datum_shift_m == 0.0


def test_pothole_width_leaves_the_floor_untouched_by_the_wheel():
    """A 600 mm hole must be wider than the tyre can bridge but deeper than it can reach."""
    hole = Pothole(start_m=0.0, depth_m=0.180, hole_length_m=0.600)
    for radius_m in (0.372, 0.352):
        drop_m = radius_m - np.sqrt(radius_m**2 - (hole.hole_length_m / 2.0) ** 2)
        assert 0.0 < drop_m < hole.depth_m, f"wheel of radius {radius_m} m floors the hole"


def test_washboard_returns_to_datum_and_holds_amplitude():
    wb = Washboard(start_m=0.0, amplitude_m=0.035, wavelength_m=0.900, n_waves=9)
    s = _local(wb, n=90001)
    z = wb.elevation(s)

    assert wb.length_m == pytest.approx(8.1)
    assert wb.elevation(np.array([0.0]))[0] == pytest.approx(0.0, abs=1e-12)
    assert wb.elevation(np.array([wb.length_m]))[0] == pytest.approx(0.0, abs=1e-12)
    assert np.max(z) == pytest.approx(0.035, rel=1e-6)
    assert np.min(z) == pytest.approx(-0.035, rel=1e-6)


def test_gout_is_tangent_to_the_datum_at_both_ends():
    dip = GOut(start_m=0.0, depth_m=0.350, dip_length_m=5.0)
    s = _local(dip, n=50001)
    z = dip.elevation(s)
    slope = np.gradient(z, s)

    assert z[0] == pytest.approx(0.0, abs=1e-12)
    assert z[-1] == pytest.approx(0.0, abs=1e-12)
    assert np.min(z) == pytest.approx(-0.350, rel=1e-6)
    assert abs(slope[0]) < 1e-4 and abs(slope[-1]) < 1e-4


def test_drop_carries_its_descent_in_the_datum_shift():
    step = Drop(start_m=52.0, height_m=0.600, ramp_m=0.0)

    assert step.length_m == 0.0
    assert step.end_m == pytest.approx(52.0)
    assert step.datum_shift_m == pytest.approx(-0.600)
    assert np.allclose(step.elevation(np.array([0.0])), 0.0)


def test_drop_with_a_ramp_descends_linearly():
    step = Drop(start_m=0.0, height_m=0.600, ramp_m=0.300)
    s = _local(step)
    z = step.elevation(s)

    assert step.length_m == pytest.approx(0.300)
    assert z[0] == pytest.approx(0.0)
    assert z[-1] == pytest.approx(-0.600)
    assert np.allclose(np.diff(z), np.diff(z)[0])


def test_kicker_geometry():
    k = Kicker(
        start_m=57.0,
        height_m=0.350,
        ramp_m=1.400,
        gap_m=2.500,
        landing_angle_deg=20.0,
        landing_m=3.000,
    )

    assert k.length_m == pytest.approx(6.9)
    assert k.launch_angle_deg == pytest.approx(14.036, abs=1e-3)
    assert k.landing_drop_m == pytest.approx(3.0 * np.tan(np.radians(20.0)))
    assert k.datum_shift_m == pytest.approx(-k.landing_drop_m)

    # Ramp rises linearly to the lip, then steps down to the gap floor.
    assert k.elevation(np.array([1.399]))[0] == pytest.approx(0.350 * 1.399 / 1.400, rel=1e-9)
    assert k.elevation(np.array([1.400]))[0] == pytest.approx(0.0)
    assert k.elevation(np.array([2.5]))[0] == pytest.approx(0.0)

    # Landing descends at the requested angle and ends exactly at the datum shift.
    assert k.elevation(np.array([k.length_m]))[0] == pytest.approx(-k.landing_drop_m)
    on_landing = np.linspace(3.9, 6.9, 501)
    slope = np.gradient(k.elevation(on_landing), on_landing)
    assert np.allclose(slope, -np.tan(np.radians(20.0)), atol=1e-6)


def test_roots_produce_isolated_bumps_of_the_requested_height():
    roots = Roots(start_m=0.0, n_bumps=7, height_m=0.080, width_m=0.250, section_length_m=7.0, seed=2029)
    s = _local(roots, n=70001)
    z = roots.elevation(s)

    assert len(roots.bump_centres_m()) == 7
    assert z[0] == pytest.approx(0.0)
    assert z[-1] == pytest.approx(0.0)
    assert np.min(z) == pytest.approx(0.0)
    assert np.max(z) == pytest.approx(0.080, rel=1e-3)
    assert roots.datum_shift_m == 0.0


def test_roots_bumps_never_overlap():
    roots = Roots(start_m=0.0, n_bumps=7, height_m=0.080, width_m=0.250, section_length_m=7.0, seed=2029)
    gaps = np.diff(np.sort(roots.bump_centres_m()))
    assert np.all(gaps > roots.width_m), "jittered centres must stay further apart than one bump width"


def test_rock_garden_is_tapered_and_amplitude_exact():
    rg = RockGarden(start_m=0.0, section_length_m=8.0, amplitude_m=0.060, correlation_length_m=0.250, seed=1701)
    s = _local(rg, n=80001)
    z = rg.elevation(s)

    assert z[0] == pytest.approx(0.0, abs=1e-12)
    assert z[-1] == pytest.approx(0.0, abs=1e-12)
    assert np.max(np.abs(z)) == pytest.approx(0.060, rel=1e-6)
    assert rg.datum_shift_m == 0.0


@pytest.mark.parametrize(
    "obstacle",
    [
        RockGarden(start_m=0.0, section_length_m=8.0, amplitude_m=0.060, correlation_length_m=0.250, seed=1701),
        Roots(start_m=0.0, n_bumps=7, height_m=0.080, width_m=0.250, section_length_m=7.0, seed=2029),
    ],
)
def test_random_sections_are_independent_of_the_sampling_grid(obstacle):
    """A section sampled at 5 mm and at 1 mm must agree exactly on their shared points."""
    coarse = np.arange(0.0, obstacle.length_m + 1e-9, 0.005)
    fine = np.arange(0.0, obstacle.length_m + 1e-9, 0.001)

    z_coarse = obstacle.elevation(coarse)
    z_fine = obstacle.elevation(fine)[::5]

    assert np.allclose(z_coarse, z_fine, atol=1e-12)


@pytest.mark.parametrize("cls,kwargs", [
    (RockGarden, dict(section_length_m=8.0, amplitude_m=0.060, correlation_length_m=0.250)),
    (Roots, dict(n_bumps=7, height_m=0.080, width_m=0.250, section_length_m=7.0)),
])
def test_random_sections_are_seed_deterministic(cls, kwargs):
    s = np.linspace(0.0, kwargs["section_length_m"], 4001)

    same_a = cls(start_m=0.0, seed=7, **kwargs).elevation(s)
    same_b = cls(start_m=0.0, seed=7, **kwargs).elevation(s)
    other = cls(start_m=0.0, seed=8, **kwargs).elevation(s)

    assert np.array_equal(same_a, same_b)
    assert not np.allclose(same_a, other)


# --------------------------------------------------------------------------------------
# Profile assembly
# --------------------------------------------------------------------------------------


def test_flat_track_is_identically_zero():
    track = TrackSpec(name="t", length_m=50.0)
    z = build_profile(track, np.linspace(0.0, 50.0, 1001))

    assert np.allclose(z, 0.0)
    assert track.datum_shift_m == 0.0


def test_datum_is_carried_across_a_drop():
    track = TrackSpec(
        name="t",
        length_m=40.0,
        obstacles=[
            SquareEdge(start_m=5.0, height_m=0.100, ledge_length_m=0.250),
            Drop(start_m=20.0, height_m=0.600, ramp_m=0.0),
            SquareEdge(start_m=30.0, height_m=0.100, ledge_length_m=0.250),
        ],
    )
    x = np.linspace(0.0, 40.0, 40001)
    z = build_profile(track, x)

    assert z[np.searchsorted(x, 10.0)] == pytest.approx(0.0)
    assert z[np.searchsorted(x, 5.1)] == pytest.approx(0.100)
    assert z[np.searchsorted(x, 25.0)] == pytest.approx(-0.600)
    # The second ledge is 100 mm above the *lowered* road, not above the start datum.
    assert z[np.searchsorted(x, 30.1)] == pytest.approx(-0.500)
    assert track.datum_shift_m == pytest.approx(-0.600)


def test_profile_extends_flat_beyond_the_last_obstacle():
    track = TrackSpec(name="t", length_m=30.0, obstacles=[Drop(start_m=10.0, height_m=0.4)])
    x = np.linspace(0.0, 100.0, 10001)
    z = build_profile(track, x)

    assert np.allclose(z[x > 10.0], -0.4)


def test_final_profile_value_equals_the_declared_datum_shift():
    track = enduro_aggressive()
    x = np.linspace(0.0, track.length_m, 22401)
    z = build_profile(track, x)

    assert z[-1] == pytest.approx(track.datum_shift_m, abs=1e-9)


def test_overlapping_obstacles_are_rejected():
    track = TrackSpec(
        name="bad",
        length_m=40.0,
        obstacles=[
            SquareEdge(start_m=10.0, height_m=0.1, ledge_length_m=1.000),
            Pothole(start_m=10.5, depth_m=0.1, hole_length_m=0.500),
        ],
    )
    with pytest.raises(ValueError, match="may not overlap"):
        track.validate()


def test_obstacle_outside_the_track_is_rejected():
    track = TrackSpec(name="bad", length_m=10.0, obstacles=[SquareEdge(start_m=9.9, height_m=0.1, ledge_length_m=0.5)])
    with pytest.raises(ValueError, match="outside"):
        track.validate()


def test_non_positive_track_length_is_rejected():
    with pytest.raises(ValueError, match="non-positive length"):
        TrackSpec(name="bad", length_m=0.0).validate()


def test_touching_obstacles_are_allowed():
    track = TrackSpec(
        name="ok",
        length_m=40.0,
        obstacles=[
            SquareEdge(start_m=10.0, height_m=0.1, ledge_length_m=0.250),
            Pothole(start_m=10.250, depth_m=0.1, hole_length_m=0.500),
        ],
    )
    track.validate()


# --------------------------------------------------------------------------------------
# Presets
# --------------------------------------------------------------------------------------


@pytest.mark.parametrize("name", sorted(PRESETS))
def test_every_preset_validates_and_fits_the_field(name):
    assert_track_fits(get_preset(name))


def test_preset_registry_is_complete():
    assert available_presets() == sorted(PRESETS)
    assert "enduro_aggressive" in available_presets()


def test_unknown_preset_raises():
    with pytest.raises(KeyError, match="unknown track preset"):
        get_preset("no_such_track")


def test_presets_are_independent_between_calls():
    first = get_preset("enduro_aggressive")
    first.obstacles.clear()
    second = get_preset("enduro_aggressive")

    assert len(second.obstacles) > 0


def test_enduro_aggressive_layout():
    track = enduro_aggressive()
    lo, hi = profile_extent(track, resolution_m=FIELD.resolution_m)

    assert track.length_m == pytest.approx(115.0)
    assert len(track.obstacles) == 11
    assert track.datum_shift_m == pytest.approx(-2.420, abs=1e-3)
    assert (lo, hi) == pytest.approx((-2.510, 0.090), abs=1e-3)
    assert len(track.markers) == 11


# --------------------------------------------------------------------------------------
# Heightfield
# --------------------------------------------------------------------------------------


def test_field_resolution_is_exactly_five_millimetres():
    assert FIELD.resolution_m == pytest.approx(0.005, abs=1e-12)
    assert FIELD.track_length_m == pytest.approx(120.0)
    assert FIELD.nrow == 2


def test_field_envelope_matches_the_documented_numbers():
    assert FIELD.min_profile_m == pytest.approx(-2.8)
    assert FIELD.max_profile_m == pytest.approx(0.6)
    assert FIELD.size_attr == "60.000000 0.500000 3.400000 0.500000"


def test_normalize_round_trips_the_profile():
    track = enduro_aggressive()
    profile = build_profile(track, FIELD.track_x())
    data = FIELD.normalize(profile)
    recovered = data[0] * FIELD.elevation_m - FIELD.datum_z_m

    assert np.allclose(recovered, profile, atol=1e-12)


def test_normalize_repeats_the_profile_across_every_row():
    data = build_field_data(enduro_aggressive())

    assert data.shape == (FIELD.nrow, FIELD.ncol)
    assert data.min() >= 0.0 and data.max() <= 1.0
    for row in range(1, FIELD.nrow):
        assert np.array_equal(data[0], data[row])


def test_normalize_rejects_a_wrong_sample_count():
    with pytest.raises(ValueError, match="expected"):
        FIELD.normalize(np.zeros(10))


def test_normalize_rejects_a_profile_that_leaves_the_envelope():
    profile = np.zeros(FIELD.ncol)
    profile[0] = -3.5
    with pytest.raises(ValueError, match="leaves the field envelope"):
        FIELD.normalize(profile)


def test_assert_track_fits_rejects_an_over_long_track():
    track = TrackSpec(name="long", length_m=200.0)
    with pytest.raises(ValueError, match="field is"):
        assert_track_fits(track)


def test_assert_track_fits_rejects_an_over_deep_track():
    track = TrackSpec(name="deep", length_m=40.0, obstacles=[Drop(start_m=10.0, height_m=3.0)])
    with pytest.raises(ValueError, match="leaves the field envelope"):
        assert_track_fits(track)


def test_catch_plane_sits_below_the_field_floor_and_every_preset():
    catch_z = FIELD.catch_plane_z_m(GROUND_Z_M)
    field_floor_z = FIELD.geom_z_m(GROUND_Z_M)

    assert catch_z < field_floor_z
    for name in PRESETS:
        lo, _ = profile_extent(get_preset(name), resolution_m=FIELD.resolution_m)
        assert catch_z < GROUND_Z_M + lo, f"catch plane would bridge the deepest point of '{name}'"


def test_field_places_track_origin_at_world_origin():
    assert FIELD.geom_x_m() == pytest.approx(FIELD.radius_x_m)
    assert FIELD.track_x()[0] == pytest.approx(0.0)
    assert FIELD.track_x()[-1] == pytest.approx(FIELD.track_length_m)


def test_a_custom_field_spec_recomputes_its_derived_geometry():
    spec = HeightFieldSpec(ncol=1001, radius_x_m=25.0, elevation_m=1.0, datum_z_m=0.5)

    assert spec.track_length_m == pytest.approx(50.0)
    assert spec.resolution_m == pytest.approx(0.05)
    assert (spec.min_profile_m, spec.max_profile_m) == pytest.approx((-0.5, 0.5))
