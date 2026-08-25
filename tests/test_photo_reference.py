"""Tests for calibrated pivot extraction from the reference photograph."""

import json
from pathlib import Path

import pytest

from tools.photo_reference import (
    BB_DROP_CHECK_MM,
    BB_DROP_TOLERANCE_MM,
    build_reference,
    calibrate,
    detect_red_clusters,
)

REPO = Path(__file__).resolve().parent.parent
PHOTO = REPO / "docs" / "reference" / "bulls_sonic_evo_side.jpg"


def test_detects_the_ten_pivot_clusters():
    clusters = detect_red_clusters(PHOTO)
    assert len(clusters) == 10, f"expected 10 red clusters, got {len(clusters)}"
    assert clusters[0]["area"] >= clusters[-1]["area"], "clusters must be sorted by area"


def test_calibration_reproduces_bb_drop():
    """The scale is fitted from wheelbase only; BB drop is an independent check."""
    clusters = detect_red_clusters(PHOTO)
    cal = calibrate(clusters)
    assert cal.mm_per_px == pytest.approx(1.6193, abs=0.005)

    ref = build_reference(PHOTO)
    front_axle_z = ref["points_mm"]["front_axle"][1]
    assert front_axle_z == pytest.approx(
        BB_DROP_CHECK_MM, abs=BB_DROP_TOLERANCE_MM
    ), f"calibration self-check failed: front axle Z={front_axle_z}"


def test_reference_payload_contains_every_named_pivot():
    ref = build_reference(PHOTO)
    for name in (
        "P0_candidate", "P2_candidate", "P3", "P4", "P5", "P6", "P7",
        "rear_axle", "front_axle", "bb",
    ):
        assert name in ref["points_mm"], f"missing {name}"

    assert ref["points_mm"]["P3"][0] == pytest.approx(-71.3, abs=2.0)
    assert ref["points_mm"]["P3"][1] == pytest.approx(206.4, abs=2.0)
    assert ref["points_mm"]["P7"][0] == pytest.approx(171.4, abs=2.0)
    assert ref["points_mm"]["P7"][1] == pytest.approx(401.7, abs=2.0)


def test_reference_json_is_committed_and_matches_the_tool():
    path = REPO / "docs" / "reference" / "bulls_reference_points.json"
    assert path.exists(), "run: uv run python -m tools.photo_reference"
    on_disk = json.loads(path.read_text())
    assert on_disk["points_mm"] == build_reference(PHOTO)["points_mm"]
