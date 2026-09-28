"""
Tests for the ``bike-ride`` command line.

Tests include:
- Track resolution: preset, road preset with seed/length overrides, TOML file with
  overrides, error messages for unknown tracks and inapplicable overrides.
- Run directory naming.
- Exit codes and stderr for bad arguments.
- ``--list-tracks``, ``--dump-track`` (and that the dump loads back), ``--preview``.
- One real ``--headless`` run on a short 40 m road: every artifact written, summary
  consistent with the run, ``--decimate`` honoured, ``--no-plots`` honoured.

The headless run is ~11 000 steps and takes a couple of seconds; it runs once.
"""

import json
from pathlib import Path

import pytest

from bike_sim.cli.ride import (
    _fit_sag,
    _drivetrain_config,
    _tyre_config,
    main,
    parse_args,
    resolve_leg_config,
    resolve_rider,
    resolve_track,
    run_dir_name,
    track_seed,
)
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.physics.rider import RiderSpecs
from bike_sim.physics.tyre import TyreConfig
from bike_sim.sim.ride.recorder import CHANNELS, read_csv
from bike_sim.terrain import (
    RoadRoughness,
    SquareEdge,
    TrackSpec,
    available_presets,
    get_preset,
    load_track,
)

SHORT_ROAD = """
name = "short_road"
length_m = 40

[[obstacles]]
type = "bump"
start_m = 20.0
height_mm = 40
length_m = 0.4

[[obstacles]]
type = "pothole"
start_m = 28.0
depth_mm = 60
length_m = 0.5

[generator]
seed = 1
runup_m = 12.0
runout_m = 4.0
potholes_per_100m = 0
bumps_per_100m = 5
bump_height_mm = [30, 40]
bump_length_m = [0.3, 0.5]
roughness_mm = 0
"""


@pytest.fixture()
def short_road(tmp_path) -> Path:
    path = tmp_path / "short_road.toml"
    path.write_text(SHORT_ROAD, encoding="utf-8")
    return path


# --------------------------------------------------------------------------------------
# Resolution and naming
# --------------------------------------------------------------------------------------


def test_resolve_preset_and_road_overrides():
    assert resolve_track("single_edge") == get_preset("single_edge")
    assert resolve_track("road_worn") == get_preset("road_worn")

    reseeded = resolve_track("road_worn", seed=7)
    longer = resolve_track("road_worn", length_m=150.0)

    assert reseeded.name == "road_worn" and reseeded != get_preset("road_worn")
    assert longer.length_m == 150.0
    assert longer.description == get_preset("road_worn").description


def test_resolve_track_file_with_overrides(short_road):
    track = resolve_track(str(short_road))
    assert track.name == "short_road" and track.length_m == 40.0

    longer = resolve_track(str(short_road), length_m=60.0, seed=3)
    assert longer.length_m == 60.0
    assert longer != track


def test_climb_steps_preset_defaults():
    assert "climb_steps" in available_presets()
    track = resolve_track("climb_steps")
    assert track.name == "climb_steps"
    assert track.surface == "hardpack"

    args = parse_args(["--track", "climb_steps"])
    assert args.speed == 16.0
    assert args.tyre_model == "pneumatic"
    assert args.drive_mode == "pedelec"
    assert args.assist == "turbo"


def test_gearing_flag_disables_autoshift():
    args = parse_args(["--gearing", "32x16"])
    specs = _drivetrain_config(args)
    assert specs.chainring_teeth == 32
    assert specs.cog_teeth == 16
    assert specs.auto_shift is False

    # Default without --gearing preserves auto_shift=True
    args_default = parse_args([])
    specs_default = _drivetrain_config(args_default)
    assert specs_default.auto_shift is True


@pytest.mark.parametrize(
    "args, message",
    [
        (("no_such_track",), "unknown track"),
        (("missing.toml",), "not found"),
        (("enduro_aggressive",), "only apply to generated roads"),
    ],
)
def test_resolve_track_errors(args, message, tmp_path):
    kwargs = {"seed": 1} if args[0] == "enduro_aggressive" else {}
    with pytest.raises(ValueError, match=message):
        resolve_track(*args, **kwargs)


def test_track_seed_and_run_dir_name():
    assert track_seed("road_worn", None) == 0
    assert track_seed("road_worn", 5) == 5
    assert track_seed("enduro_aggressive", None) is None
    assert run_dir_name(get_preset("road_worn"), 25.0, 0) == "road_worn_25_s0"
    assert run_dir_name(get_preset("single_edge"), 32.5, None) == "single_edge_32.5"


def test_tyre_cli_options_and_pressure_pair():
    args = parse_args([
        "--tyre-model", "pneumatic",
        "--tyre-tier", "detailed",
        "--tyre-pressure", "1.3/1.8",
        "--surface", "wet",
    ])
    config = _tyre_config(args)
    assert (config.model, config.tier, config.surface) == ("pneumatic", "detailed", "wet")
    assert (config.front.pressure_bar, config.rear.pressure_bar) == pytest.approx((1.3, 1.8))
    assert run_dir_name(get_preset("road_worn"), 25.0, 0, config) == (
        "road_worn_25_s0_pneumatic-detailed_1.30-1.80bar_wet"
    )
    assert _tyre_config(parse_args([])).model == TyreConfig().model == "sphere"


def test_tyre_pressure_cli_rejects_invalid_pair(capsys):
    with pytest.raises(SystemExit):
        parse_args(["--tyre-pressure", "1.2"])
    assert "FRONT/REAR" in capsys.readouterr().err
    with pytest.raises(SystemExit):
        parse_args(["--tyre-pressure", "0.5/1.7"])
    assert "0.8, 3.0" in capsys.readouterr().err


# --------------------------------------------------------------------------------------
# Argument handling
# --------------------------------------------------------------------------------------


def test_bad_arguments_exit_nonzero(capsys, tmp_path):
    assert main(["--track", "nope", "--headless"]) == 1
    assert "unknown track" in capsys.readouterr().err
    assert main(["--speed", "5", "--headless"]) == 2
    assert "--speed" in capsys.readouterr().err
    assert main(["--decimate", "0", "--headless"]) == 2
    assert main(["--track", "enduro_aggressive", "--seed", "3", "--preview", "--out", str(tmp_path)]) == 1


def test_list_tracks(capsys):
    assert main(["--list-tracks"]) == 0
    out = capsys.readouterr().out
    for name in ("enduro_aggressive", "road_worn", "single_edge"):
        assert name in out
    assert "generated" in out and "authored" in out


def test_dump_track_loads_back(capsys, tmp_path):
    assert main(["--dump-track", "road_smooth"]) == 0
    text = capsys.readouterr().out
    path = tmp_path / "dumped.toml"
    path.write_text(text, encoding="utf-8")

    loaded = load_track(path)
    assert loaded.obstacles == get_preset("road_smooth").sorted_obstacles

    assert main(["--dump-track", "nope"]) == 1


def test_preview_writes_a_png_and_lists_markers(capsys, tmp_path, short_road):
    assert main(["--track", str(short_road), "--preview", "--out", str(tmp_path)]) == 0
    out = capsys.readouterr().out

    png = tmp_path / "preview_short_road_s1.png"
    assert png.exists() and png.stat().st_size > 5000
    assert "Bump@20m" in out and "Pothole@28m" in out


# --------------------------------------------------------------------------------------
# Headless run
# --------------------------------------------------------------------------------------


def test_headless_run_writes_every_artifact(capsys, tmp_path, short_road):
    code = main(["--headless", "--track", str(short_road), "--speed", "25", "--out", str(tmp_path), "--decimate", "4"])
    out = capsys.readouterr().out

    assert code == 0
    run_dir = tmp_path / "short_road_25_s1"
    for name in ("telemetry.csv", "summary.json", "travel.png", "shaft_velocity.png", "acceleration.png", "profile.png"):
        assert (run_dir / name).exists(), name
    assert not (run_dir / "tyres.png").exists()

    summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
    assert summary["track"] == "short_road"
    assert summary["completed"] is True
    assert summary["outcome"] == "end of track"
    assert summary["n_potholes"] == 1
    assert summary["n_bumps"] >= 2
    assert summary["extras"]["seed"] == 1.0
    assert abs(summary["mean_speed_kmh"] - 25.0) < 1.5
    assert summary["sample_interval_s"] == pytest.approx(4 * 0.0005)

    channels = read_csv(run_dir / "telemetry.csv")
    assert list(channels) == list(CHANNELS)
    rows = len(channels["time_s"])
    assert rows == summary["steps"] // 4 + 1
    # The last stored row is up to three steps before the terminating one.
    assert channels["x_m"][-1] >= 40.0 - 4 * 0.0005 * 8.0

    assert "Ride summary -- short_road" in out
    assert "Pothole@28m" in out


def test_headless_pneumatic_run_records_model_pressure_and_surface(capsys, tmp_path):
    path = tmp_path / "flat_short.toml"
    path.write_text('name = "flat_short"\nlength_m = 20\nsurface = "asphalt"\n', encoding="utf-8")
    code = main([
        "--headless", "--track", str(path), "--speed", "15", "--rider", "none",
        "--tyre-model", "pneumatic", "--tyre-tier", "fast",
        "--tyre-pressure", "1.4/1.8", "--surface", "wet",
        "--out", str(tmp_path),
    ])
    out = capsys.readouterr().out
    run_dir = tmp_path / "flat_short_15_pneumatic-fast_1.40-1.80bar_wet"
    assert code == 0
    assert "tyres: pneumatic/fast, pressure 1.40/1.80 bar, surface wet" in out
    summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
    assert summary["extras"]["tyre_model"] == "pneumatic"
    assert summary["extras"]["tyre_tier"] == "fast"
    assert summary["extras"]["tyre_pressure_front_bar"] == pytest.approx(1.4)
    assert summary["extras"]["tyre_pressure_rear_bar"] == pytest.approx(1.8)
    assert summary["extras"]["surface"] == "wet"
    channels = read_csv(run_dir / "telemetry.csv")
    assert channels["front_tyre_pressure_bar"][0] == pytest.approx(1.4)
    assert channels["rear_tyre_pressure_bar"][0] == pytest.approx(1.8)
    assert set(summary["tyres"]) == {"front", "rear"}
    assert (run_dir / "tyres.png").exists()


def test_headless_no_plots_and_sag(capsys, tmp_path, short_road):
    """`--sag` fits against the lumped rider's centre of mass when that rider is chosen."""
    code = main(["--headless", "--track", str(short_road), "--out", str(tmp_path), "--no-plots", "--sag", "30",
                 "--rider", "lumped"])
    out = capsys.readouterr().out

    assert code == 0
    run_dir = tmp_path / "short_road_25_s1"
    assert (run_dir / "telemetry.csv").exists() and (run_dir / "summary.json").exists()
    assert not (run_dir / "travel.png").exists()
    assert "--sag 30%" in out and "118.5 psi" in out and "93,984 N/m" in out
    assert "rider: lumped" in out

    summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
    assert summary["extras"]["sag_target_pct"] == 30.0
    assert summary["extras"]["fork_psi"] == pytest.approx(118.5, abs=0.1)
    assert summary["rider_variant"] == "lumped"
    assert summary["rider"] is None


def test_sag_fitting_keeps_legacy_damper_law() -> None:
    controller, _, _ = _fit_sag(30.0, BikeSpecs(), RiderSpecs(variant="lumped"))
    shock = controller.suspension_system.shock_damper
    shock.set_clicks(lockout=True)

    assert shock.compute_damping_force(0.03, 20.0) == pytest.approx(480.0)


def test_sag_fits_the_seated_rider_against_its_own_centre_of_mass(capsys, tmp_path, short_road):
    """
    The seated rider's centre of mass is further back than the lumped rider's, so `--sag 30`
    wants a softer fork (100.5 vs 118.5 psi) and a stiffer coil (105,148 vs 93,984 N/m).
    """
    code = main(["--headless", "--track", str(short_road), "--out", str(tmp_path), "--no-plots", "--sag", "30"])
    out = capsys.readouterr().out

    assert code == 0
    assert "--sag 30%" in out and "100.5 psi" in out and "105,148 N/m" in out
    assert "rider: seated, 80 kg, 1.80 m" in out
    assert "solved rider load split: saddle 5" in out and "/ pedals 33.0 % / bar 12.0 %" in out
    assert "seated rider: mean load split" in out

    summary = json.loads((tmp_path / "short_road_25_s1" / "summary.json").read_text(encoding="utf-8"))
    assert summary["rider_variant"] == "seated"
    assert summary["extras"]["rider_mass_kg"] == 80.0
    rider = summary["rider"]
    assert rider["mean_saddle_share"] == pytest.approx(0.55, abs=0.03)
    assert rider["torso"]["point"] == "torso" and rider["pelvis"]["point"] == "pelvis"
    assert rider["saddle_lift_events"] >= 0


# --------------------------------------------------------------------------------------
# Regressions from the 2026-09-26 review
# --------------------------------------------------------------------------------------


def test_sag_requires_the_rider(capsys):
    assert main(["--headless", "--sag", "30", "--no-rider"]) == 2
    assert "--no-rider" in capsys.readouterr().err
    assert main(["--headless", "--sag", "30", "--rider", "none"]) == 2
    assert "--rider none" in capsys.readouterr().err


# --------------------------------------------------------------------------------------
# Rider arguments
# --------------------------------------------------------------------------------------


def test_no_rider_and_a_rider_variant_contradict(capsys, tmp_path):
    assert main(["--preview", "--out", str(tmp_path), "--no-rider", "--rider", "seated"]) == 2
    assert "contradicts" in capsys.readouterr().err
    # `--no-rider` with `--rider none` is redundant, not contradictory.
    assert main(["--preview", "--out", str(tmp_path), "--no-rider", "--rider", "none"]) == 0


def test_a_rider_the_frame_cannot_seat_is_refused(capsys, tmp_path):
    """A 2.00 m rider needs more seatpost than the frame has; the run refuses before compiling."""
    assert main(["--headless", "--out", str(tmp_path), "--rider-height", "2.0"]) == 2
    err = capsys.readouterr().err
    assert "seatpost" in err and "--rider-height" in err


def test_seated_rider_on_a_flight_track_is_warned(capsys, tmp_path):
    """The aggressive preset has a drop and a kicker; a seated run over it is flagged, not refused."""
    assert main(["--preview", "--track", "enduro_aggressive", "--out", str(tmp_path)]) == 0
    err = capsys.readouterr().err
    assert "warning" in err and "drop/kicker" in err and "--rider lumped" in err
    assert main(["--preview", "--track", "enduro_aggressive", "--out", str(tmp_path), "--rider", "lumped"]) == 0
    assert "drop/kicker" not in capsys.readouterr().err
    assert main(["--preview", "--track", "road_worn", "--out", str(tmp_path)]) == 0
    assert "drop/kicker" not in capsys.readouterr().err


def test_seed_on_a_file_without_generator_is_rejected(capsys, tmp_path):
    path = tmp_path / "authored.toml"
    path.write_text('name = "authored"\nlength_m = 40\n[[obstacles]]\ntype = "pothole"\nstart_m = 20\n', encoding="utf-8")

    assert main(["--track", str(path), "--seed", "4", "--preview", "--out", str(tmp_path)]) == 1
    assert "[generator]" in capsys.readouterr().err
    assert track_seed(str(path), None) is None


# --------------------------------------------------------------------------------------
# Leg arguments
# --------------------------------------------------------------------------------------


def test_legs_flag_defaults():
    args = parse_args(["--drive-mode", "pedal"])
    assert resolve_leg_config(args).legs == "articulated"
    args = parse_args(["--drive-mode", "motor"])
    assert resolve_leg_config(args).legs == "rigid"
    args = parse_args(["--drive-mode", "motor", "--visual-pedalling"])
    assert resolve_leg_config(args).legs == "articulated"


def test_visual_pedalling_requires_motor():
    args = parse_args(["--drive-mode", "pedal", "--visual-pedalling"])
    with pytest.raises(ValueError):
        resolve_leg_config(args)


def test_legs_explicit_overrides_and_contradictions(capsys):
    """--legs wins over the drive-mode default; impossible pairings are refused."""
    args = parse_args(["--drive-mode", "pedal", "--legs", "rigid"])
    assert resolve_leg_config(args).legs == "rigid"
    args = parse_args(["--drive-mode", "motor", "--legs", "articulated", "--visual-pedalling"])
    assert resolve_leg_config(args).legs == "articulated"
    assert resolve_leg_config(args).visual_pedalling is True

    # The legs cannot pump without a turning crankset to weld the feet to.
    with pytest.raises(ValueError, match="--visual-pedalling"):
        resolve_leg_config(parse_args(["--drive-mode", "motor", "--legs", "articulated"]))
    # Asking for the visual stroke with rigid legs is a contradiction.
    with pytest.raises(ValueError, match="--legs rigid"):
        resolve_leg_config(parse_args(["--drive-mode", "motor", "--legs", "rigid",
                                       "--visual-pedalling"]))

    # main() reports the same contradictions as an argument error, exit code 2.
    assert main(["--drive-mode", "pedal", "--visual-pedalling", "--headless"]) == 2
    assert "--visual-pedalling" in capsys.readouterr().err


def test_resolve_rider_carries_the_resolved_legs():
    assert resolve_rider(parse_args(["--drive-mode", "pedal"])).legs == "articulated"
    assert resolve_rider(parse_args([])).legs == "rigid"
    assert resolve_rider(parse_args(["--legs", "rigid", "--drive-mode", "pedal"])).legs == "rigid"


def test_a_stray_file_named_like_a_preset_does_not_shadow_it(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    (tmp_path / "road_worn").write_text("not toml at all", encoding="utf-8")

    assert resolve_track("road_worn") == get_preset("road_worn")
    assert track_seed("road_worn", None) == 0
