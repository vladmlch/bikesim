"""
Tests for wheelspin analysis tool (tools/analyze_wheelspin.py).
"""

from pathlib import Path
import json
import pytest

from bike_sim.terrain import SteppedClimb, TrackSpec
from tools.analyze_wheelspin import run_wheelspin_analysis, WheelspinReport


def test_wheelspin_analysis_smoke(tmp_path):
    # Short test climb to ensure simulation, metrics, plot, and JSON output run cleanly
    track = TrackSpec(
        name="test_short_climb",
        length_m=30.0,
        surface="hardpack",
        obstacles=[
            SteppedClimb(
                start_m=5.0,
                steps=((10.0, 8.0), (20.0, 8.0)),
                transition_m=2.0,
            )
        ],
    )

    report = run_wheelspin_analysis(track=track, out_dir=tmp_path, max_seconds=1.5)

    assert isinstance(report, WheelspinReport)
    assert report.track_name == "test_short_climb"
    assert report.total_time_s > 0.0
    assert report.total_distance_m > 0.0
    assert isinstance(report.step_wheelspin_duration_s, dict)

    png_path = tmp_path / "climb_kinematics.png"
    assert png_path.exists()
    assert png_path.stat().st_size > 5000

    json_path = tmp_path / "summary.json"
    assert json_path.exists()
    data = json.loads(json_path.read_text(encoding="utf-8"))
    assert data["track_name"] == "test_short_climb"
    assert "total_dissipated_energy_j" in data
    assert "critical_gradient_pct" in data
