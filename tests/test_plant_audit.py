from pathlib import Path
import json
import numpy as np
import pytest
from bike_sim.validation.rider_replay import build_sim
from tools.plant_audit import checked_steps, run_audit, main


@pytest.mark.parametrize('duration,dt,count', [(1., .00125, 800), (.1, .000625, 160)])
def test_steps_preserve_requested_physical_duration(duration, dt, count):
    assert checked_steps(duration, dt) == count


@pytest.mark.parametrize('duration,dt', [(0., .001), (.0015, .001), (float('nan'), .001)])
def test_fractional_or_invalid_horizon_is_rejected(duration, dt):
    with pytest.raises(ValueError):
        checked_steps(duration, dt)


def test_preview_and_audit_match_for_identical_short_initial_state():
    physics = 'examples/research/viewer_physics_welded.toml'
    track = 'examples/research/rough_uphill_savage.toml'
    build_sim(physics, track)
    a, b = build_sim(physics, track), build_sim(physics, track)
    np.testing.assert_array_equal(a.data.qpos, b.data.qpos)
    np.testing.assert_array_equal(a.data.qvel, b.data.qvel)
    with b.physical.preview_mode():
        for _ in range(200):
            a.step()
            b.step()
            np.testing.assert_allclose(a.data.qpos, b.data.qpos, rtol=0, atol=1e-12)
            np.testing.assert_allclose(a.data.qvel, b.data.qvel, rtol=0, atol=1e-12)


@pytest.mark.parametrize('bad_decimate', [0, -1, '8', 1.5, True])
def test_invalid_decimate_is_rejected(bad_decimate, tmp_path):
    with pytest.raises(ValueError, match='decimate must be a positive integer'):
        run_audit('examples/research/viewer_physics_welded.toml',
                  'examples/research/rough_uphill_savage.toml',
                  0.01, tmp_path / 'audit', decimate=bad_decimate)


def test_run_audit_generates_all_expected_artifacts(tmp_path):
    physics = 'examples/research/viewer_physics_welded.toml'
    track = 'examples/research/rough_uphill_savage.toml'
    out = tmp_path / 'audit_test'
    # Run short duration (e.g. 0.05 s = 40 steps at dt=0.00125)
    report = run_audit(physics, track, 0.05, out, decimate=4)

    assert (out / 'summary.json').exists()
    assert (out / 'samples.jsonl').exists()
    assert (out / 'track.toml').exists()
    assert (out / 'terrain_vertices.npy').exists()

    # Check terrain_vertices.npy
    saved_v = np.load(out / 'terrain_vertices.npy')
    assert saved_v.ndim == 2 and saved_v.shape[1] == 2
    assert np.isfinite(saved_v).all()

    # Check summary.json structure
    assert report['mode'] == 'audited'
    assert 'metadata' in report
    assert 'valid_prefix' in report
    assert 'model_status' in report
    assert report['time_s'] == pytest.approx(0.05)

    # Check samples.jsonl lines
    lines = [json.loads(line) for line in (out / 'samples.jsonl').read_text().splitlines() if line]
    assert len(lines) > 0
    for row in lines:
        assert 'time_s' in row
        assert 'x_m' in row
        assert 'eligible' in row
        assert 'tires' in row
        assert 'drive' in row
        assert 'energy' in row
        assert 'model_status' in row


def test_main_cli_returns_nonzero_when_invalid(monkeypatch, tmp_path):
    # Mock run_audit to return invalid report
    def mock_run_audit(*args, **kwargs):
        return {'valid_prefix': {'first_bad': {'time_s': 1.0, 'reasons': ['model_violation']}}}

    monkeypatch.setattr('tools.plant_audit.run_audit', mock_run_audit)
    monkeypatch.setattr('sys.argv', ['plant_audit.py', '--physics-config', 'p.toml',
                                     '--track', 't.toml', '--duration', '1.0',
                                     '--out', str(tmp_path / 'out')])
    assert main() == 2


def test_main_cli_returns_zero_when_valid(monkeypatch, tmp_path):
    # Mock run_audit to return valid report
    def mock_run_audit(*args, **kwargs):
        return {'valid_prefix': {'first_bad': None}}

    monkeypatch.setattr('tools.plant_audit.run_audit', mock_run_audit)
    monkeypatch.setattr('sys.argv', ['plant_audit.py', '--physics-config', 'p.toml',
                                     '--track', 't.toml', '--duration', '1.0',
                                     '--out', str(tmp_path / 'out')])
    assert main() == 0


def test_first_invalid_always_written_even_if_not_decimated(monkeypatch, tmp_path):
    physics = 'examples/research/viewer_physics_welded.toml'
    track = 'examples/research/rough_uphill_savage.toml'
    out = tmp_path / 'decimate_audit'
    # Use decimate=10. The savage baseline fails at step ~6198 (which is not a multiple of 10 if step is e.g. 6198 or odd).
    # More directly, run audit for duration 8.0 s with decimate=16
    report = run_audit(physics, track, 8.0, out, decimate=16, continue_invalid=False)
    lines = [json.loads(line) for line in (out / 'samples.jsonl').read_text().splitlines() if line]
    first_bad = report['valid_prefix']['first_bad']
    if first_bad is not None:
        # Check that a line with the exact first_bad time_s exists in samples.jsonl
        match = [r for r in lines if abs(r['time_s'] - first_bad['time_s']) < 1e-9]
        assert len(match) == 1
        assert not match[0]['eligible']
