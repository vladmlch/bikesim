import numpy as np
import pytest
from bike_sim.validation.rider_replay import resume_schedule,diagnose_resume,resume_evidence


def test_explicit_drive_coast_resume_intervals():
    assert resume_schedule(.5).human_torque_nm==20.
    assert resume_schedule(2.5).human_torque_nm==0.
    assert resume_schedule(4.5).human_torque_nm==20.
    assert resume_schedule(.5).motor_torque_nm==0.


def rows(speed=2.,request=20.,delivered=10.):
    return [{'time_s':4.+i*.1,'end_time_s':4.+(i+1)*.1,
        'drive':{'human_command_nm':request,'human_sensor_nm':delivered,'crank_rad_s':speed,'crank_phase_rad':i*.1*speed},
        'control':{'human_torque_nm':request},'rider':{'front_pedal':{'in_platform':True,'normal_load_n':100.}},'joint_terms':{}}
        for i in range(40)]


def test_diagnosis_requires_actual_power_and_full_crank_turn():
    assert diagnose_resume(rows())=='resumed'
    assert diagnose_resume(rows(delivered=0.))!='resumed'
    assert diagnose_resume(rows(speed=.1))=='mechanically_stalled'
    assert diagnose_resume(rows(request=0.,delivered=0.))=='no_effort_request'
    assert diagnose_resume(rows()[:3])=='mixed_or_unresolved'


def test_multiple_causes_remain_unresolved():
    r=rows(speed=0.)
    for row in r:
        row['rider']={};row['joint_terms']={'hip':{'saturated':True}}
    assert diagnose_resume(r)=='mixed_or_unresolved'
    assert set(resume_evidence(r)['candidate_causes'])=={'support_unavailable','actuator_saturated'}


@pytest.mark.parametrize('time',[float('nan'),-1.])
def test_bad_schedule_time(time):
    with pytest.raises(ValueError):resume_schedule(time)


def test_failed_replay_preserves_accepted_intervals(tmp_path, monkeypatch):
    import gzip
    import json
    from types import SimpleNamespace
    from bike_sim.validation import rider_replay
    from bike_sim.sim.ride import physical_session

    class FailingSimulation:
        def __init__(self):
            self.model = SimpleNamespace(opt=SimpleNamespace(timestep=.01))
            self.time_s = 0.
            self.position_m = 2.
            self.crash = None
            self.equilibrium = {}
            self.physical = SimpleNamespace(interactive_preview=False, sample=None,
                model_status=SimpleNamespace(as_dict=lambda: {'model_valid': True}))

        def step(self, *, control):
            if self.time_s > 0.:
                raise RuntimeError('force evaluation failed')
            self.time_s = .01
            channels = {
                'drive': {'rider_mode': 'pedaling', 'crank_rad_s': 2.},
                'rider': {side + '_pedal': {'in_platform': True,
                    'normal_load_n': 100., 'gap_m': -.001} for side in ('front', 'rear')},
                'rider_support_targets': {}, 'rider_control': {}, 'rider_ik_saturation': {},
                'tires': {}, 'energy': {'energy_scale_j': 10., 'residual_j': 0.},
                'model_status': {'model_valid': True, 'numerically_valid': True},
            }
            self.physical.sample = SimpleNamespace(interval_id=0, time_s=0.,
                end_time_s=.01, channels=channels)

    monkeypatch.setattr(rider_replay, 'build_sim', lambda *args, **kwargs: FailingSimulation())
    monkeypatch.setattr(physical_session, 'configuration_metadata', lambda sim: {})
    path = tmp_path / 'failure.json.gz'
    report = rider_replay.run_replay('unused', 'unused', duration_s=.02,
                                   mode='automatic', output=path)
    assert report['error'] == 'RuntimeError: force evaluation failed'
    assert len(report['rows']) == 1
    assert report['rows'][0]['end_time_s'] == .01
    assert not report['completed_requested_duration']
    assert not report['support_evidence']['complete']
    with gzip.open(path, 'rt') as stream:
        saved = json.load(stream)
    assert saved['rows'] == report['rows']


def test_malformed_diagnostic_evidence_still_saves_the_partial_record(tmp_path, monkeypatch):
    import json
    from types import SimpleNamespace
    from bike_sim.validation import rider_replay
    from bike_sim.sim.ride import physical_session

    sim = SimpleNamespace(equilibrium={}, time_s=.1, position_m=2., crash=None,
        physical=SimpleNamespace(model_status=SimpleNamespace(as_dict=lambda: {})))
    def broken_replay(sim, schedule, duration_s, *, row_sink):
        row_sink({'time_s': .2, 'end_time_s': .1, 'rider': {}})
        raise RuntimeError('recorded force failure')

    monkeypatch.setattr(rider_replay, 'build_sim', lambda *args, **kwargs: sim)
    monkeypatch.setattr(rider_replay, 'replay', broken_replay)
    monkeypatch.setattr(physical_session, 'configuration_metadata', lambda sim: {})
    path = tmp_path / 'failure.json'
    report = rider_replay.run_replay('unused', 'unused', duration_s=1., output=path)
    saved = json.loads(path.read_text())
    assert saved['error'] == 'RuntimeError: recorded force failure'
    assert saved['rows'] == report['rows']
    assert not saved['support_evidence']['valid_for_learning']
    assert saved['support_evidence']['evaluation_error']
