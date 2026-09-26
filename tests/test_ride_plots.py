"""
Tests for the ride telemetry figures and the track preview.

The figures are checked for rendering headlessly to non-empty PNG files from synthetic
channels, so this module compiles no MuJoCo model. Visual content is judged by eye, not
asserted.
"""

import numpy as np
import pytest

from bike_sim.sim.ride.metrics import GRAVITY_MPS2
from bike_sim.sim.ride.recorder import CHANNELS
from bike_sim.terrain import Pothole, TrackSpec, get_preset
from bike_sim.viz.ride_plots import (
    plot_acceleration,
    plot_ride,
    plot_shaft_velocity,
    plot_track_profile,
    plot_tyres,
    plot_travel,
)

PNG_MAGIC = b"\x89PNG\r\n\x1a\n"


def _channels(n: int = 3000, length_m: float = 40.0):
    x = np.linspace(0.0, length_m, n)
    rng = np.random.default_rng(0)
    c = {name: np.zeros(n) for name in CHANNELS}
    c["x_m"] = x
    c["time_s"] = np.arange(n) * 0.0005
    c["fork_travel_mm"] = 70.0 + 30.0 * np.sin(x)
    c["rear_wheel_mm"] = 40.0 + 20.0 * np.sin(x)
    c["shock_stroke_mm"] = 15.0 + 8.0 * np.sin(x)
    c["fork_shaft_mps"] = rng.normal(0.0, 0.3, n)
    c["shock_shaft_mps"] = rng.normal(0.0, 0.2, n)
    c["bar_acc_vert_mps2"] = GRAVITY_MPS2 + rng.normal(0.0, 2.0, n)
    c["saddle_acc_vert_mps2"] = GRAVITY_MPS2 + rng.normal(0.0, 1.5, n)
    c["bar_acc_vert_mps2"][1500:1502] += 200.0
    c["front_contact"] = np.ones(n)
    c["rear_contact"] = np.ones(n)
    return c


def _track():
    return TrackSpec(name="synthetic", length_m=40.0, description="test",
                     obstacles=[Pothole(start_m=20.0, depth_m=0.08, hole_length_m=0.5)])


def _is_png(path):
    return path.exists() and path.read_bytes()[:8] == PNG_MAGIC and path.stat().st_size > 5000


def test_each_figure_renders_headless(tmp_path):
    c = _channels()
    track = _track()

    assert _is_png(plot_travel(c, track, 180.0, 65.0, tmp_path / "travel.png"))
    assert _is_png(plot_shaft_velocity(c, c["x_m"] >= 10.0, tmp_path / "shaft.png"))
    assert _is_png(plot_acceleration(c, 0.0005, track, tmp_path / "acc.png"))


def test_plot_ride_writes_the_three_named_files(tmp_path):
    paths = plot_ride(_channels(), 0.0005, _track(), 180.0, 65.0, tmp_path / "ride")

    assert [p.name for p in paths] == ["travel.png", "shaft_velocity.png", "acceleration.png"]
    assert all(_is_png(p) for p in paths)


def test_plot_tyres_renders_load_slip_patch_and_rim_strikes(tmp_path):
    channels = _channels()
    for wheel, offset in (("front", 0.0), ("rear", 10.0)):
        channels[f"{wheel}_tyre_fz_n"] = 400.0 + 20.0 * np.sin(channels["x_m"] + offset)
        channels[f"{wheel}_tyre_deflection_mm"] = 8.0 + 2.0 * np.sin(channels["x_m"] + offset)
        channels[f"{wheel}_patch_length_mm"] = 120.0 + 10.0 * np.sin(channels["x_m"] + offset)
        channels[f"{wheel}_slip_ratio"] = 0.2 * np.sin(channels["x_m"] + offset)
    channels["front_rim_strike"][1500:1503] = 1.0

    path = plot_tyres(channels, _track(), tmp_path / "tyres.png")
    assert _is_png(path)


def test_shaft_velocity_tolerates_an_empty_window(tmp_path):
    c = _channels()
    assert _is_png(plot_shaft_velocity(c, np.zeros_like(c["x_m"], dtype=bool), tmp_path / "empty.png"))


@pytest.mark.parametrize("name", ["road_broken", "flat", "enduro_aggressive"])
def test_track_profile_preview_renders_for_presets(tmp_path, name):
    assert _is_png(plot_track_profile(get_preset(name), tmp_path / f"{name}.png"))
