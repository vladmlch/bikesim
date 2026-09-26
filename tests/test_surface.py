"""
Tests for road surfaces (docs/RIDE.md section 4.1).

Surfaces are pure data the pneumatic tyre reads. What matters here: the registered values
are the contract's, the friction curve falls from peak to sliding, every track carries a
surface with the right default, track files read and write it, and none of it touches the
road geometry.
"""

from dataclasses import replace

import numpy as np
import pytest

from bike_sim.terrain import (
    DEFAULT_SURFACE,
    ROAD_SURFACE,
    SURFACES,
    TRAIL_SURFACE,
    SurfaceMap,
    SurfaceSpec,
    TrackFileError,
    TrackSpec,
    available_presets,
    available_surfaces,
    build_field_data,
    get_preset,
    get_surface,
    load_track,
    save_track,
    track_from_dict,
)
from bike_sim.terrain.surface import slip_stiffness_n

# The docs/RIDE.md section 4.1 table.
CONTRACT_TABLE = {
    "asphalt": (1.05, 0.75, 15.0),
    "hardpack": (0.80, 0.60, 12.0),
    "loose": (0.55, 0.45, 7.0),
    "wet": (0.50, 0.40, 10.0),
}


def test_registered_surfaces_are_the_contract_table():
    assert available_surfaces() == list(CONTRACT_TABLE)
    for name, (mu_peak, mu_slide, c_kappa) in CONTRACT_TABLE.items():
        s = get_surface(name)
        assert (s.mu_peak, s.mu_slide, s.slip_stiffness_per_load) == (mu_peak, mu_slide, c_kappa)
        assert s.stribeck_speed_mps == 4.5


def test_defaults_by_kind_of_track():
    assert ROAD_SURFACE == "asphalt"
    assert TRAIL_SURFACE == DEFAULT_SURFACE == "hardpack"


def test_friction_falls_from_peak_to_sliding_with_sliding_speed():
    s = get_surface("asphalt")
    assert s.mu(0.0) == pytest.approx(s.mu_peak)
    assert s.mu(-0.0) == pytest.approx(s.mu_peak)
    assert s.mu(200.0) == pytest.approx(s.mu_slide, abs=1e-12)
    speeds = np.linspace(0.0, 10.0, 101)
    mu = s.mu(speeds)
    assert mu.shape == speeds.shape
    assert np.all(np.diff(mu) < 0.0)
    # Symmetric in the sign of the sliding speed; one Stribeck speed is 1/e of the drop.
    assert s.mu(-2.0) == pytest.approx(s.mu(2.0))
    assert s.mu(4.5) == pytest.approx(s.mu_slide + (s.mu_peak - s.mu_slide) / np.e)


@pytest.mark.parametrize(
    "kwargs, message",
    [
        (dict(mu_peak=0.5, mu_slide=0.6, slip_stiffness_per_load=10.0), "exceeds mu_peak"),
        (dict(mu_peak=0.0, mu_slide=0.0, slip_stiffness_per_load=10.0), "mu_peak must be positive"),
        (dict(mu_peak=0.5, mu_slide=0.4, slip_stiffness_per_load=0.0), "slip_stiffness_per_load"),
        (dict(mu_peak=0.5, mu_slide=0.4, slip_stiffness_per_load=1.0, stribeck_speed_mps=-1.0),
         "stribeck_speed_mps"),
    ],
)
def test_surface_validation(kwargs, message):
    with pytest.raises(ValueError, match=message):
        SurfaceSpec("bad", **kwargs)


def test_unknown_surface_lists_the_registered_ones():
    with pytest.raises(KeyError, match="asphalt, hardpack, loose, wet"):
        get_surface("ice")


def test_uniform_map_answers_the_same_surface_everywhere():
    m = SurfaceMap.uniform("loose")
    assert m.name == "loose"
    assert m.surfaces == (SURFACES["loose"],)
    for x in (-5.0, 0.0, 17.3, 1e4):
        assert m.at(x) is SURFACES["loose"]
    with pytest.raises(KeyError):
        SurfaceMap.uniform("snow")


def test_slip_stiffness_scales_with_load():
    s = get_surface("hardpack")
    assert slip_stiffness_n(s, 500.0) == pytest.approx(12.0 * 500.0)
    assert slip_stiffness_n(s, 0.0) == 0.0
    assert slip_stiffness_n(s, -3.0) == 0.0
    assert slip_stiffness_n(s, float("nan")) == 0.0


def test_every_preset_has_its_kind_of_surface():
    for name in available_presets():
        expected = ROAD_SURFACE if name.startswith("road_") else TRAIL_SURFACE
        assert get_preset(name).surface == expected, name


def test_track_validation_rejects_an_unknown_surface():
    TrackSpec(name="t", length_m=10.0, surface="wet").validate()
    with pytest.raises(ValueError, match="unknown surface 'mud'"):
        TrackSpec(name="t", length_m=10.0, surface="mud").validate()


def test_surface_does_not_touch_the_geometry():
    for name in available_presets():
        track = get_preset(name)
        base = build_field_data(track)
        for surface in available_surfaces():
            assert np.array_equal(build_field_data(replace(track, surface=surface)), base), (name, surface)


def test_track_file_default_follows_the_preset_rule():
    hand_placed = {"name": "x", "length_m": 20.0}
    generated = {"name": "y", "length_m": 60.0, "generator": {"seed": 0}}
    assert track_from_dict(hand_placed).surface == TRAIL_SURFACE
    assert track_from_dict(generated).surface == ROAD_SURFACE


def test_track_file_explicit_surface_wins_and_is_validated():
    assert track_from_dict({"name": "x", "length_m": 20.0, "surface": "wet"}).surface == "wet"
    assert track_from_dict(
        {"name": "y", "length_m": 60.0, "surface": "loose", "generator": {"seed": 0}}
    ).surface == "loose"
    for bad in ("mud", 3, ""):
        with pytest.raises(TrackFileError, match="unknown surface"):
            track_from_dict({"name": "x", "length_m": 20.0, "surface": bad})


def test_track_file_round_trips_the_surface(tmp_path):
    for track in (get_preset("road_worn"), get_preset("single_edge"),
                  TrackSpec(name="gravel", length_m=30.0, surface="loose")):
        path = save_track(track, tmp_path / f"{track.name}.toml")
        assert f'surface = "{track.surface}"' in path.read_text(encoding="utf-8")
        loaded = load_track(path)
        assert loaded.surface == track.surface
        assert loaded.obstacles == track.sorted_obstacles
