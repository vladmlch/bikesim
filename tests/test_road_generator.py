"""
Unit tests for the rough-road generator and the TOML track file.

Tests include:
- Generator: seed determinism, counts from rates, sizes within the declared ranges,
  shapes drawn from the declared weights, clearance from hand-placed obstacles and the
  run-up / run-out reserves, roughness filling only the free stretches, validation.
- Built-in road levels: validate, fit the default field, differ in severity.
- Track file: unit conversion, shape resolution, generator block, seed and length
  overrides, dump -> load round-trip equal to the source, error messages.

Pure geometry; nothing here compiles a MuJoCo model.
"""

from pathlib import Path

import numpy as np
import pytest

from bike_sim.terrain.heightfield import assert_track_fits
from bike_sim.terrain.obstacles import (
    BUMP_TYPES,
    POTHOLE_TYPES,
    BowlPothole,
    Bump,
    Pothole,
    RoadRoughness,
    SlopedPothole,
    SquareEdge,
    TrapezoidBump,
    Washboard,
)
from bike_sim.terrain.presets import get_preset
from bike_sim.terrain.profile import TrackSpec, profile_extent
from bike_sim.terrain.road import (
    RoadGeneratorSpec,
    build_road,
    generate_road,
    road_broken,
    road_smooth,
    road_worn,
)
from bike_sim.terrain.trackfile import (
    TrackFileError,
    dump_track,
    load_track,
    save_track,
    track_from_dict,
)


def _defects(obstacles):
    return [o for o in obstacles if not isinstance(o, RoadRoughness)]


def _roughness(obstacles):
    return [o for o in obstacles if isinstance(o, RoadRoughness)]


# --------------------------------------------------------------------------------------
# Generator
# --------------------------------------------------------------------------------------


def test_generator_is_seed_deterministic():
    spec = RoadGeneratorSpec(seed=11)
    a = generate_road(spec, 100.0)
    b = generate_road(RoadGeneratorSpec(seed=11), 100.0)
    c = generate_road(RoadGeneratorSpec(seed=12), 100.0)

    assert a == b
    assert a != c


def test_generator_counts_follow_the_rates_over_the_usable_length():
    spec = RoadGeneratorSpec(seed=0, runup_m=10.0, runout_m=5.0, potholes_per_100m=4.0, bumps_per_100m=6.0)
    obstacles = generate_road(spec, 100.0)
    defects = _defects(obstacles)

    # usable = 85 m -> 3.4 -> 3 potholes, 5.1 -> 5 bumps
    assert sum(isinstance(o, POTHOLE_TYPES) for o in defects) == 3
    assert sum(isinstance(o, BUMP_TYPES) for o in defects) == 5


def test_generator_sizes_stay_within_the_declared_ranges():
    spec = RoadGeneratorSpec(
        seed=5,
        potholes_per_100m=10.0,
        pothole_depth_m=(0.040, 0.120),
        pothole_length_m=(0.300, 0.800),
        bumps_per_100m=10.0,
        bump_height_m=(0.030, 0.080),
        bump_length_m=(0.300, 0.600),
    )
    for o in _defects(generate_road(spec, 200.0)):
        if isinstance(o, POTHOLE_TYPES):
            assert 0.040 <= o.depth_m <= 0.120
            assert 0.300 <= o.hole_length_m <= 0.800
        else:
            assert 0.030 <= o.height_m <= 0.080
            assert 0.300 <= o.length_m <= 0.600 + 1e-12


def test_generator_fixed_size_is_a_degenerate_range():
    spec = RoadGeneratorSpec(seed=1, potholes_per_100m=5.0, pothole_depth_m=(0.070, 0.070), bumps_per_100m=0.0)
    holes = [o for o in _defects(generate_road(spec, 100.0)) if isinstance(o, POTHOLE_TYPES)]

    assert holes and all(o.depth_m == 0.070 for o in holes)


def test_generator_honours_shape_weights():
    only_bowl = RoadGeneratorSpec(seed=2, potholes_per_100m=6.0, pothole_edge={"bowl": 1.0},
                                  bumps_per_100m=6.0, bump_shape={"trapezoid": 1.0})
    defects = _defects(generate_road(only_bowl, 100.0))

    assert all(isinstance(o, (BowlPothole, TrapezoidBump)) for o in defects)

    mixed = RoadGeneratorSpec(seed=3, potholes_per_100m=15.0, pothole_edge={"sharp": 1.0, "sloped": 1.0},
                              bumps_per_100m=0.0, roughness_m=0.0)
    kinds = {type(o) for o in _defects(generate_road(mixed, 300.0))}

    assert kinds == {Pothole, SlopedPothole}


def test_generator_keeps_clear_of_reserves_and_hand_placed_obstacles():
    spec = RoadGeneratorSpec(seed=4, runup_m=12.0, runout_m=6.0, min_gap_m=1.0,
                             potholes_per_100m=6.0, bumps_per_100m=6.0)
    edge = SquareEdge(start_m=50.0, height_m=0.090, ledge_length_m=0.250)
    track = build_road("t", spec, 100.0, hand_placed=[edge])
    generated = [o for o in track.obstacles if o is not edge]

    for o in _defects(generated):
        assert o.start_m >= 12.0
        assert o.end_m <= 94.0
        assert o.end_m <= edge.start_m - 1.0 or o.start_m >= edge.end_m + 1.0
    assert edge in track.obstacles


def test_generator_roughness_fills_free_stretches_only():
    spec = RoadGeneratorSpec(seed=6, potholes_per_100m=3.0, bumps_per_100m=3.0, roughness_m=0.003)
    obstacles = generate_road(spec, 100.0)
    defects = sorted(_defects(obstacles), key=lambda o: o.start_m)
    rough = sorted(_roughness(obstacles), key=lambda o: o.start_m)

    assert rough, "roughness segments expected"
    assert all(o.amplitude_m == 0.003 for o in rough)
    TrackSpec(name="t", length_m=100.0, obstacles=obstacles).validate()
    # Every roughness segment is bounded by a defect or by the run-up / run-out reserve.
    for seg in rough:
        left_ok = seg.start_m == pytest.approx(spec.runup_m) or any(
            d.end_m == pytest.approx(seg.start_m) for d in defects
        )
        right_ok = seg.end_m == pytest.approx(100.0 - spec.runout_m) or any(
            d.start_m == pytest.approx(seg.end_m) for d in defects
        )
        assert left_ok and right_ok
    assert len({o.seed for o in rough}) == len(rough)


def test_generator_without_roughness_emits_no_texture():
    obstacles = generate_road(RoadGeneratorSpec(seed=0, roughness_m=0.0), 100.0)
    assert not _roughness(obstacles)


def test_generator_rejects_impossible_density():
    spec = RoadGeneratorSpec(seed=0, potholes_per_100m=200.0, pothole_length_m=(0.8, 0.8))
    with pytest.raises(ValueError, match="no room left"):
        generate_road(spec, 100.0)


@pytest.mark.parametrize(
    "kwargs, message",
    [
        ({"potholes_per_100m": -1.0}, "non-negative"),
        ({"pothole_depth_m": (0.2, 0.1)}, "range"),
        ({"pothole_edge": {"round": 1.0}}, "unknown shape"),
        ({"bump_shape": {}}, "at least one positive"),
        ({"bump_ramp_fraction": 0.7}, "bump_ramp_fraction"),
    ],
)
def test_generator_spec_validation(kwargs, message):
    with pytest.raises(ValueError, match=message):
        RoadGeneratorSpec(**kwargs).validate()


def test_generator_needs_room_between_runup_and_runout():
    with pytest.raises(ValueError, match="no room between"):
        generate_road(RoadGeneratorSpec(runup_m=10.0, runout_m=5.0), 15.0)


# --------------------------------------------------------------------------------------
# Built-in levels
# --------------------------------------------------------------------------------------


@pytest.mark.parametrize("name", ["road_smooth", "road_worn", "road_broken"])
def test_road_levels_are_registered_valid_and_fit_the_field(name):
    track = get_preset(name)
    assert_track_fits(track)
    assert track.length_m == pytest.approx(100.0)
    assert track.datum_shift_m == 0.0


def test_road_levels_increase_in_severity():
    def severity(track):
        lo, hi = profile_extent(track)
        return len(_defects(track.obstacles)), hi - lo

    n_smooth, span_smooth = severity(road_smooth())
    n_worn, span_worn = severity(road_worn())
    n_broken, span_broken = severity(road_broken())

    assert n_smooth < n_worn < n_broken
    assert span_smooth < span_worn < span_broken


def test_road_worn_layout_is_pinned():
    """Regenerated layouts must not drift silently; update this deliberately."""
    track = road_worn()
    defects = _defects(track.obstacles)

    assert len(defects) == 8
    assert sum(isinstance(o, Pothole) for o in defects) == 3
    assert sum(isinstance(o, Bump) for o in defects) == 5
    assert [label for _, label in track.markers] == [o.label for o in sorted(defects, key=lambda o: o.start_m)]


# --------------------------------------------------------------------------------------
# Track file
# --------------------------------------------------------------------------------------


HAND_FILE = """
name = "hand"
length_m = 60
description = "two defects"

[[obstacles]]
type = "pothole"
start_m = 20.0
depth_mm = 80
length_m = 0.5
edge = "sloped"
edge_m = 0.12

[[obstacles]]
type = "bump"
start_m = 35.0
height_mm = 50
length_m = 0.4

[[obstacles]]
type = "washboard"
start_m = 45.0
amplitude_mm = 20
wavelength_m = 0.6
n_waves = 5
"""


def _write(tmp_path: Path, text: str, name: str = "track.toml") -> Path:
    path = tmp_path / name
    path.write_text(text, encoding="utf-8")
    return path


def test_track_file_converts_units_and_resolves_shapes(tmp_path):
    track = load_track(_write(tmp_path, HAND_FILE))

    assert track.name == "hand"
    assert track.length_m == 60.0
    assert track.description == "two defects"
    assert track.obstacles == [
        SlopedPothole(start_m=20.0, depth_m=0.08, hole_length_m=0.5, edge_m=0.12),
        Bump(start_m=35.0, height_m=0.05, bump_length_m=0.4),
        Washboard(start_m=45.0, amplitude_m=0.02, wavelength_m=0.6, n_waves=5),
    ]


def test_track_file_generator_block_fills_around_hand_placed(tmp_path):
    text = """
name = "mixed"
length_m = 100

[[obstacles]]
type = "square_edge"
start_m = 50.0
height_mm = 90
length_m = 0.25

[generator]
seed = 3
potholes_per_100m = 4
pothole_depth_mm = [40, 120]
pothole_length_m = [0.3, 0.8]
pothole_edge = { sharp = 0.5, bowl = 0.5 }
bumps_per_100m = 6
bump_height_mm = 50
bump_length_m = [0.3, 0.6]
roughness_mm = 3
"""
    track = load_track(_write(tmp_path, text))
    edge = track.obstacles[0]
    defects = _defects(track.obstacles[1:])

    assert isinstance(edge, SquareEdge) and edge.height_m == pytest.approx(0.09)
    assert len(defects) == 3 + 5
    assert all(o.height_m == pytest.approx(0.05) for o in defects if isinstance(o, BUMP_TYPES))
    assert {type(o) for o in defects if isinstance(o, POTHOLE_TYPES)} <= {Pothole, BowlPothole}
    assert _roughness(track.obstacles)

    same = load_track(_write(tmp_path, text, "again.toml"))
    assert same == track


def test_track_file_seed_and_length_overrides(tmp_path):
    text = """
name = "gen"
length_m = 100
[generator]
seed = 0
"""
    path = _write(tmp_path, text)
    base = load_track(path)
    reseeded = load_track(path, seed=9)
    longer = load_track(path, length_m=160.0)

    assert reseeded != base
    assert longer.length_m == 160.0
    assert len(_defects(longer.obstacles)) > len(_defects(base.obstacles))


def test_dump_then_load_round_trips_every_preset(tmp_path):
    for name in ("enduro_aggressive", "road_broken", "single_edge", "flat"):
        source = get_preset(name)
        path = save_track(source, tmp_path / f"{name}.toml")
        loaded = load_track(path)

        assert loaded.name == source.name
        assert loaded.length_m == source.length_m
        assert loaded.description == source.description
        assert loaded.obstacles == source.sorted_obstacles


def test_dump_materialises_the_layout_without_a_generator_block():
    text = dump_track(road_worn())

    assert "[generator]" not in text
    assert text.count("[[obstacles]]") == len(road_worn().obstacles)
    assert 'type = "roughness"' in text


@pytest.mark.parametrize(
    "text, message",
    [
        ('length_m = 10\n[[obstacles]]\ntype = "pothole"\nstart_m = 1', "non-empty 'name'"),
        ('name = "x"\n[[obstacles]]\ntype = "pothole"\nstart_m = 1', "positive 'length_m'"),
        ('name = "x"\nlength_m = 10\n[[obstacles]]\nstart_m = 1', "missing 'type'"),
        ('name = "x"\nlength_m = 10\n[[obstacles]]\ntype = "crater"\nstart_m = 1', "unknown type"),
        ('name = "x"\nlength_m = 10\n[[obstacles]]\ntype = "pothole"\nstart_m = 1\nedge = "soft"', "unknown pothole edge"),
        ('name = "x"\nlength_m = 10\n[[obstacles]]\ntype = "bump"\nstart_m = 1\nshape = "cube"', "unknown bump shape"),
        ('name = "x"\nlength_m = 10\n[[obstacles]]\ntype = "pothole"\nstart_m = 1\nwidth_m = 1', "unknown key"),
        ('name = "x"\nlength_m = 10\n[[obstacles]]\ntype = "pothole"\ndepth_mm = 50', "missing 'start_m'"),
        ('name = "x"\nlength_m = 10\n[[obstacles]]\ntype = "pothole"\nstart_m = 1\ndepth_m = 0.05\ndepth_mm = 50', "given twice"),
        ('name = "x"\nlength_m = 10\n[[obstacles]]\ntype = "pothole"\nstart_m = 1\n[[obstacles]]\ntype = "pothole"\nstart_m = 1.2', "may not overlap"),
        ('name = "x"\nlength_m = 100\n[generator]\npothole_depth_mm = [1, 2, 3]', "low, high"),
        ('name = "x"\nlength_m = 100\n[generator]\npothole_edge = "soft"', "unknown shape"),
        ('name = "x"\nlength_m = 100\n[generator]\nfoo = 1', "unknown key"),
    ],
)
def test_track_file_errors_are_specific(tmp_path, text, message):
    with pytest.raises(TrackFileError, match=message):
        load_track(_write(tmp_path, text))


def test_track_file_syntax_error_names_the_file(tmp_path):
    path = _write(tmp_path, 'name = "x\n')
    with pytest.raises(TrackFileError, match="track.toml"):
        load_track(path)


def test_track_from_dict_accepts_a_single_shape_name_for_weights():
    track = track_from_dict({"name": "d", "length_m": 100, "generator": {"pothole_edge": "bowl", "bumps_per_100m": 0}})
    holes = [o for o in track.obstacles if isinstance(o, POTHOLE_TYPES)]

    assert holes and all(isinstance(o, BowlPothole) for o in holes)


# --------------------------------------------------------------------------------------
# Editing a dumped road: roughness yields to whatever is placed on top of it
# --------------------------------------------------------------------------------------


def test_adding_a_defect_to_a_dumped_road_splits_the_roughness(tmp_path):
    """The natural edit -- dump road_worn, add a pothole -- must load, not collide."""
    path = save_track(get_preset("road_worn"), tmp_path / "edit.toml")
    text = path.read_text(encoding="utf-8") + (
        '\n[[obstacles]]\ntype = "pothole"\nstart_m = 20.0\ndepth_mm = 90\nlength_m = 0.45\nedge = "sloped"\n'
    )
    path.write_text(text, encoding="utf-8")

    track = load_track(path)
    added = [o for o in track.obstacles if isinstance(o, SlopedPothole)]
    rough = sorted(_roughness(track.obstacles), key=lambda o: o.start_m)
    original_rough = sorted(_roughness(get_preset("road_worn").obstacles), key=lambda o: o.start_m)

    assert len(added) == 1 and added[0].start_m == 20.0
    assert len(rough) == len(original_rough) + 1
    left = [o for o in rough if o.end_m == pytest.approx(20.0)]
    right = [o for o in rough if o.start_m == pytest.approx(20.45)]
    assert len(left) == 1 and len(right) == 1
    assert left[0].seed == original_rough[0].seed
    assert right[0].seed == original_rough[0].seed + 1_000_003
    assert left[0].start_m == original_rough[0].start_m
    assert right[0].end_m == pytest.approx(original_rough[0].end_m)
    # Everything else is untouched.
    assert len(_defects(track.obstacles)) == len(_defects(get_preset("road_worn").obstacles)) + 1


def test_carve_roughness_drops_slivers_and_leaves_real_overlaps_to_validate():
    from bike_sim.terrain.trackfile import carve_roughness

    seg = RoadRoughness(start_m=10.0, section_length_m=10.0, amplitude_m=0.003, seed=5)
    near_start = Pothole(start_m=10.2, depth_m=0.05, hole_length_m=0.5)   # 0.2 m sliver on the left
    carved = carve_roughness([seg, near_start])
    rough = _roughness(carved)

    assert len(rough) == 1
    assert rough[0].start_m == pytest.approx(10.7) and rough[0].end_m == pytest.approx(20.0)

    two_defects = carve_roughness([Pothole(start_m=5.0), Pothole(start_m=5.2)])
    assert len(two_defects) == 2
    with pytest.raises(ValueError, match="may not overlap"):
        TrackSpec(name="t", length_m=30.0, obstacles=two_defects).validate()
