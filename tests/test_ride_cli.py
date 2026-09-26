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

from bike_sim.cli.ride import main, resolve_track, run_dir_name, track_seed
from bike_sim.sim.ride.recorder import CHANNELS, read_csv
from bike_sim.terrain import RoadRoughness, SquareEdge, TrackSpec, get_preset, load_track

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


def test_headless_no_plots_and_sag(capsys, tmp_path, short_road):
    code = main(["--headless", "--track", str(short_road), "--out", str(tmp_path), "--no-plots", "--sag", "30"])
    out = capsys.readouterr().out

    assert code == 0
    run_dir = tmp_path / "short_road_25_s1"
    assert (run_dir / "telemetry.csv").exists() and (run_dir / "summary.json").exists()
    assert not (run_dir / "travel.png").exists()
    assert "--sag 30%" in out and "118.5 psi" in out and "93,984 N/m" in out

    summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
    assert summary["extras"]["sag_target_pct"] == 30.0
    assert summary["extras"]["fork_psi"] == pytest.approx(118.5, abs=0.1)


# --------------------------------------------------------------------------------------
# Regressions from the 2026-09-26 review
# --------------------------------------------------------------------------------------


def test_sag_requires_the_rider(capsys):
    assert main(["--headless", "--sag", "30", "--no-rider"]) == 2
    assert "--no-rider" in capsys.readouterr().err


def test_seed_on_a_file_without_generator_is_rejected(capsys, tmp_path):
    path = tmp_path / "authored.toml"
    path.write_text('name = "authored"\nlength_m = 40\n[[obstacles]]\ntype = "pothole"\nstart_m = 20\n', encoding="utf-8")

    assert main(["--track", str(path), "--seed", "4", "--preview", "--out", str(tmp_path)]) == 1
    assert "[generator]" in capsys.readouterr().err
    assert track_seed(str(path), None) is None


def test_a_stray_file_named_like_a_preset_does_not_shadow_it(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    (tmp_path / "road_worn").write_text("not toml at all", encoding="utf-8")

    assert resolve_track("road_worn") == get_preset("road_worn")
    assert track_seed("road_worn", None) == 0
