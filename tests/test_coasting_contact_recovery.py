import pytest

from bike_sim.validation.rider_replay import run_automatic_coast_case
from bike_sim.validation.rider_replay import run_replay


@pytest.mark.slow
def test_automatic_coasting_keeps_reachable_pedals_and_resumes_work():
    report = run_automatic_coast_case(dt_s=.0003125, duration_s=5.)
    assert report['duration_s'] == pytest.approx(5.)
    assert report['coast_entered']
    assert report['max_both_unloaded_coast_s'] <= .20
    assert report['stopped_coast_duration_s'] > .2
    assert report['max_stopped_coasting_gap_m'] <= .02
    assert report['pedaling_resumed']
    assert report['resume_positive_work_j'] > 0.
    assert report['model_valid']
    assert report['numerically_valid']


@pytest.mark.slow
@pytest.mark.parametrize('track', ['rider_resume_flat', 'rider_resume_incline'])
def test_resume_transmits_human_work_and_completes_a_crank_turn(tmp_path, track):
    report = run_replay(
        'examples/research/viewer_physics_fast.toml',
        f'examples/research/{track}.toml', mode='human-only',
        timestep_s=.000625, duration_s=10., output=tmp_path/'resume.json.gz')
    assert 'error' not in report
    assert report['completed_requested_duration']
    assert report['evidence']['diagnosis'] == 'resumed'
    assert report['evidence']['crank_rotation_rad'] >= 2.*3.141592653589793
    assert report['evidence']['positive_crank_work_j'] > 0.
    assert report['model_status']['model_valid']
    assert report['evidence']['window_s'] == [4., 10.]
    assert report['support_evidence']['numerically_valid']
    assert report['support_evidence']['complete']
