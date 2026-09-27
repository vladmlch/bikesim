"""Smoke test for the sphere-versus-pneumatic comparison report."""

import json

from bike_sim.terrain import TrackSpec
from tools.compare_tyre_models import compare_tracks, generated_reading, write_report


def test_short_flat_comparison_writes_both_models(tmp_path):
    track = TrackSpec(name="flat_compare_short", length_m=20.0)
    records = compare_tracks(
        [track],
        target_speed_kmh=15.0,
        rider="none",
    )

    assert len(records) == 1
    variants = records[0]["variants"]
    assert set(variants) == {"sphere", "pneumatic/fast"}
    assert variants["sphere"].completed
    assert variants["pneumatic/fast"].completed

    reading = generated_reading(records)
    assert "completed 1/1" in reading
    summary_path, table_path = write_report(records, tmp_path, 15.0, "none")
    payload = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["tracks"][0]["variants"]["sphere"]["completed"]
    assert "pneumatic/fast" in table_path.read_text(encoding="utf-8")
