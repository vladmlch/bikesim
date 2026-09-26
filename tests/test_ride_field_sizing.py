"""
Tests for heightfield sizing derived from track length.

Tests include:
- Pure geometry: tracks that fit the default field get the default field unchanged;
  longer tracks get a stretched field at the same 5 mm resolution and vertical envelope,
  rounded up to 10 m with a 5 m runout margin; invalid lengths are rejected.
- MJCF: the ride XML for the default field is byte-identical whether or not `field` is
  passed (the golden baseline covers the default; this pins the plumbing), and a
  stretched field compiles with the expected hfield dimensions and catch-plane depth.
- Simulation: a 300 m generated road compiles into a RideSimulation whose hfield data
  is filled at the new size, and the bike rolls forward on it.
"""

import mujoco
import numpy as np
import pytest

from bike_sim.geometry.hardpoints import compute_ground_z
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import (
    FIELD,
    HeightFieldSpec,
    RoadGeneratorSpec,
    build_field_data,
    build_road,
    get_preset,
)
from bike_sim.terrain.heightfield import FIELD_LENGTH_STEP_M, FIELD_RUNOUT_MARGIN_M


# --------------------------------------------------------------------------------------
# Geometry
# --------------------------------------------------------------------------------------


@pytest.mark.parametrize("name", ["enduro_aggressive", "flat", "single_edge", "road_broken"])
def test_shipped_presets_use_the_default_field(name):
    field = HeightFieldSpec.for_track(get_preset(name))

    assert field == FIELD
    assert field.is_default


def test_default_field_covers_exactly_the_margin():
    assert HeightFieldSpec.for_track_length(FIELD.track_length_m - FIELD_RUNOUT_MARGIN_M) == FIELD
    assert HeightFieldSpec.for_track_length(FIELD.track_length_m - FIELD_RUNOUT_MARGIN_M + 0.01) != FIELD


@pytest.mark.parametrize("track_length_m, expected_field_m", [(116.0, 130.0), (200.0, 210.0), (295.0, 300.0), (1000.0, 1010.0)])
def test_long_tracks_get_a_stretched_field(track_length_m, expected_field_m):
    field = HeightFieldSpec.for_track_length(track_length_m)

    assert field.track_length_m == pytest.approx(expected_field_m)
    assert field.resolution_m == pytest.approx(FIELD.resolution_m)
    assert field.ncol == int(round(expected_field_m / FIELD.resolution_m)) + 1
    assert (field.nrow, field.radius_y_m, field.elevation_m, field.base_m, field.datum_z_m) == (
        FIELD.nrow, FIELD.radius_y_m, FIELD.elevation_m, FIELD.base_m, FIELD.datum_z_m,
    )
    assert field.geom_x_m() == pytest.approx(expected_field_m / 2.0)
    assert not field.is_default
    assert expected_field_m % FIELD_LENGTH_STEP_M == 0


def test_with_length_rejects_lengths_off_the_grid():
    with pytest.raises(ValueError, match="multiple"):
        FIELD.with_length(100.0033)
    with pytest.raises(ValueError, match="positive"):
        FIELD.with_length(0.0)


def test_field_data_for_a_long_track_has_the_new_width():
    track = build_road("long", RoadGeneratorSpec(seed=1), length_m=300.0)
    field = HeightFieldSpec.for_track(track)
    data = build_field_data(track, field)

    assert data.shape == (field.nrow, field.ncol)
    assert np.all((data >= 0.0) & (data <= 1.0))


# --------------------------------------------------------------------------------------
# MJCF
# --------------------------------------------------------------------------------------


def test_passing_the_default_field_changes_nothing_in_the_xml():
    implicit = generate_mujoco_xml(mode="ride", include_rider=True)
    explicit = generate_mujoco_xml(mode="ride", include_rider=True, field=FIELD)

    assert implicit == explicit


def test_stretched_field_compiles_with_matching_hfield_and_catch_plane():
    field = HeightFieldSpec.for_track_length(300.0)
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode="ride", include_rider=True, field=field))
    ground_z_m = compute_ground_z(BikeSpecs()) / 1000.0

    hid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, "road")
    assert model.hfield_ncol[hid] == field.ncol
    assert model.hfield_nrow[hid] == field.nrow
    assert model.hfield_size[hid][0] == pytest.approx(field.radius_x_m)

    terrain = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "terrain")
    assert model.geom_pos[terrain][0] == pytest.approx(field.geom_x_m())
    catch = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "catch_plane")
    assert model.geom_pos[catch][2] == pytest.approx(field.catch_plane_z_m(ground_z_m))


def test_field_is_ignored_outside_ride_mode():
    field = HeightFieldSpec.for_track_length(300.0)
    assert generate_mujoco_xml(mode="standard") == generate_mujoco_xml(mode="standard", field=field)


# --------------------------------------------------------------------------------------
# Simulation
# --------------------------------------------------------------------------------------


def test_ride_simulation_on_a_long_road_rolls_forward():
    track = build_road("long", RoadGeneratorSpec(seed=1, roughness_m=0.0), length_m=300.0)
    sim = RideSimulation(track=track, target_speed_kmh=25.0)

    assert sim.field == HeightFieldSpec.for_track_length(300.0)
    assert sim.model.hfield_data.shape[0] == sim.field.nrow * sim.field.ncol
    assert not np.allclose(sim.model.hfield_data, sim.model.hfield_data[0]) or track.obstacles == []

    x0 = sim.position_m
    for _ in range(2000):  # 1 s of sim time
        sim.step()

    assert sim.position_m > x0 + 1.0
    assert not sim.crash
