"""Motor-only replay must be explicitly unsupported by pedelec permission."""
import pytest

from bike_sim.validation.rider_replay import open_loop_schedule, run_replay, main
from bike_sim.validation.fidelity_sweep import case_physics, run_fidelity_sweep


@pytest.mark.parametrize('time', [0., 1.5, 3., 5.])
def test_motor_only_schedule_is_explicitly_unsupported(time):
    with pytest.raises(ValueError, match='motor-only open-loop'):
        open_loop_schedule(time)


def test_unsupported_motor_replay_reports_failure_before_building(tmp_path):
    report = run_replay(tmp_path/'missing-physics.toml', tmp_path/'missing-track.toml',
                        mode='open-loop')
    assert report['mode'] == 'open-loop'
    assert report['diagnosis'] == 'unsupported_experiment'
    assert report['completed_requested_duration'] is False
    assert report['rows'] == []
    assert 'motor-only open-loop' in report['error']
    assert report['support_evidence']['valid_for_learning'] is False


def test_motor_only_cli_has_nonzero_exit_and_retains_error(tmp_path, capsys):
    assert main(['--mode', 'open-loop', '--physics-config', str(tmp_path/'missing.toml'),
                 '--track', str(tmp_path/'missing-track.toml'),
                 '--output', str(tmp_path/'failure.json')]) == 1
    assert 'motor-only open-loop' in capsys.readouterr().out


def test_motor_ramp_fidelity_rejects_before_loading_the_fixture(tmp_path):
    with pytest.raises(ValueError, match='motor-only open-loop'):
        case_physics('motor_ramp_grade', .00125, 'compliant_2d', 128,
                     'ideal_mid_drive', tmp_path/'missing.toml')


def test_unsupported_fidelity_case_cannot_report_pass(tmp_path):
    report = run_fidelity_sweep(str(tmp_path/'out'), cases=('motor_ramp_grade',),
        physics_path=tmp_path/'missing.toml', time_steps=(.00125, .000625, .0003125),
        road_steps=(.01, .005, .0025), station_counts=(128, 256, 512))
    assert report['passed'] is False
    assert 'motor-only open-loop' in report['initialization_errors']['motor_ramp_grade']
    assert all(row['termination'] == 'initialization_failure' for row in report['series'])
