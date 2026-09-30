import csv
import json
import pytest
from bike_sim.sim.research.policies import POLICIES
from bike_sim.sim.research.sensors import SensorObservation
from tools.research_batch import expand_grid, main


def _spec(tmp_path, body):
    path = tmp_path/'batch.toml'
    path.write_text(body)
    return path


def _obs():
    return SensorObservation(time_s=0., source_time_s=0., valid=True, specific_force_body_mps2=(0., 0., 9.8),
        pitch_rate_up_rad_s=0., front_wheel_rad_s=0., rear_wheel_rad_s=0., crank_rad_s=0.,
        motor_torque_nm=0., human_torque_nm=0.)


def test_demo_policies_are_plumbing():
    assert set(POLICIES) == {'passthrough', 'zero', 'fixed_limit_40'}
    assert POLICIES['passthrough'](_obs(), 55.).motor_torque_nm == 55.
    assert POLICIES['passthrough'](_obs(), None).motor_torque_nm == 0.   # None would mean pedelec assist
    assert POLICIES['zero'](_obs(), 55.).motor_torque_nm == 0.
    limited = POLICIES['fixed_limit_40'](_obs(), 80.)
    assert (limited.motor_torque_nm, limited.motor_limit_nm) == (80., 40.)
    import bike_sim.sim.research.policies as module
    assert 'not anti-wheelie solutions' in module.__doc__


def test_expand_grid_is_the_cartesian_product_with_stable_ids():
    runs = expand_grid(dict(scenarios=['flat', 'uphill'], seeds=[1, 2], demand_nm=[60., 80.], policy='zero'))
    assert len(runs) == 8 and len({r['run_id'] for r in runs}) == 8
    assert runs[0]['run_id'] == expand_grid(dict(scenarios=['flat', 'uphill'], seeds=[1, 2],
                                                 demand_nm=[60., 80.], policy='zero'))[0]['run_id']
    tracks = expand_grid(dict(tracks=['a.toml'], scenarios=['flat'], seeds=[0]))
    assert [r['track'] for r in tracks] == ['a.toml', None]


@pytest.mark.parametrize('body', ['[grid]\nseeds=[1]\n',                              # no track source
                                  '[grid]\nscenarios=["flat"]\npolicy="nope"\n',         # unknown policy
                                  '[grid]\nscenarios=["flat"]\nbogus=1\n'])             # unknown key
def test_bad_specs_are_rejected_before_running(tmp_path, body):
    assert main(['--spec', str(_spec(tmp_path, body)), '--out', str(tmp_path/'out')]) == 2
    assert not (tmp_path/'out').exists() or not any((tmp_path/'out').iterdir())


def test_grid_runs_write_per_run_dirs_and_reports(tmp_path):
    spec = _spec(tmp_path, '[grid]\nscenarios=["flat"]\nseeds=[1, 2]\ndemand_nm=[40.0, 60.0]\n'
                           'policy="passthrough"\nduration=0.05\nextra_args=["--rider", "lumped", "--ideal-sensors"]\n')
    out = tmp_path/'out'
    assert main(['--spec', str(spec), '--jobs', '2', '--out', str(out)]) == 0
    report = json.loads((out/'batch_report.json').read_text())
    assert len(report['runs']) == 4
    for record in report['runs']:
        for key in ('run_id', 'outcome', 'progress_m', 'motor_pass_fraction', 'loop_out'):
            assert key in record
        assert 'wheelie_time_s' in record['wheelie']
        assert (out/record['run_id']/'episode_metrics.json').is_file()
    assert report['outcome_counts'] == {'duration': 4}
    rows = list(csv.DictReader((out/'batch_report.csv').open()))
    assert len(rows) == 4 and {r['demand_nm'] for r in rows} == {'40.0', '60.0'}
    assert float(rows[0]['progress_m']) > 0.


def test_failed_run_becomes_a_record_and_nonzero_exit(tmp_path):
    spec = _spec(tmp_path, '[grid]\ntracks=["does/not/exist.toml"]\nseeds=[0]\ndemand_nm=[60.0]\nduration=0.05\n')
    out = tmp_path/'out'
    assert main(['--spec', str(spec), '--out', str(out)]) == 1
    report = json.loads((out/'batch_report.json').read_text())
    assert report['runs'][0]['outcome'] == 'error' and 'exist' in report['runs'][0]['error']
    assert report['outcome_counts'] == {'error': 1}
