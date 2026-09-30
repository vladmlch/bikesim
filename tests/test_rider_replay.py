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
