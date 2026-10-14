"""Reports are deterministic saved-data queries, never a second simulation."""
from _recording_support import energy_channels, sample_row
from test_recording_storage import capture_rows
from bike_sim.recording.reader import Recording
from bike_sim.recording.report import render_report


def test_report_regeneration_is_read_only_and_identical(tmp_path):
    path = capture_rows([sample_row(channels=energy_channels())],
                        summary={'outcome': {'reason': 'synthetic'}, 'first_failure': None}).finalize(tmp_path)
    before = {p.name: p.read_bytes() for p in path.iterdir()}
    with Recording(path) as recording:
        first = render_report(recording)
        second = render_report(recording)
    assert first == second == (path / 'summary.md').read_text()
    assert {p.name: p.read_bytes() for p in path.iterdir()} == before
    assert 'not a claim that the model is calibrated' in first
    assert 'Episode 0' in first and 'first failure' in first
    assert 'SELECT' in first and 'channel_energy.residual_j' in first
