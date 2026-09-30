import copy
import math
import pytest
from bike_sim.validation.reference_release import (release_gates,release_level,select_verified_resolution,
    classify_outcome,scenario_definitions,_scenario_track,_scenario_control,write_release_manifest)


def test_finished_course_does_not_override_physics_gates():
    report=dict(known_tests_passed=True,coast_resume_resolved=False,mechanics_passed=True,
        numerics_converged=False,model_scope_passed=True,holdout_passed=False,synthetic=True,
        scenarios={'extreme_finished':True})
    ok,reasons=release_gates(report)
    assert not ok and set(reasons)=={'coast_resume_resolved','numerics_converged'}
    assert release_level(report)=='parameterized_unvalidated'
    report.update(coast_resume_resolved=True,numerics_converged=True)
    assert release_level(report)=='numerically_verified_synthetic'
    report.update(synthetic=False,holdout_passed=True)
    assert release_level(report)!='validated_within_declared_scope'
    report['known_tests_passed']=1
    assert not release_gates(report)[0]


def test_only_actual_passing_resolution_may_be_chosen():
    common={'passed':True,'axes_passed':{'time':True,'road':True,'stations':True},'dx_m':.005,'station_count':256}
    rows=[dict(common,dt_s=.00125,passed=False),dict(common,dt_s=.000625),dict(common,dt_s=.0003125)]
    assert select_verified_resolution(rows)['dt_s']==.000625
    assert select_verified_resolution([dict(common,dt_s=.00125,axes_passed={'time':True})]) is None


def row(t,**kwargs):
    return dict(time_s=t,end_time_s=t+.01,position_m=2.,speed_mps=0.,crank_rad_s=0.,rear_slip_mps=0.,
        front_load_n=400.,rear_load_n=500.,motor_torque_nm=20.,human_torque_nm=0.,
        model_status={'model_valid':True},control={'motor_torque_nm':20.})|kwargs


def test_outcome_requires_persistent_evidence_and_reports_first_cause():
    assert classify_outcome([row(3.)])['outcome']=='duration_limit'
    rows=[row(3.+i*.01) for i in range(76)]
    result=classify_outcome(rows)
    assert result['outcome']=='physical_stall' and result['time_s']==3.
    assert result['evidence']['motor_torque_nm']==20.
    rows[-1]['model_status']={'model_valid':False,'first_model_violation':{'reasons':['front:multi_support']}}
    assert classify_outcome(rows)['outcome']=='model_invalid'
    assert classify_outcome(rows,numerical_error='nonfinite')['outcome']=='numerical_failure'
    assert classify_outcome([row(3.,position_m=6.)],goal_x_m=5.)['outcome']=='completed'


def test_entire_matrix_geometry_and_input_contract():
    items=scenario_definitions()
    assert len(items)==18
    grade=next(r for r in items if r['name']=='grade_35pct')
    assert grade['angle_rad']==pytest.approx(math.atan(.35))
    for item in items:
        track=_scenario_track(item);track.validate()
        assert _scenario_control(item,2.).motor_torque_nm is not None
    assert next(i for i in items if i['name']=='extreme_100m')['goal_x_m']==100.


def test_manual_booleans_cannot_release_without_actual_evidence(tmp_path):
    report={k:True for k in ('known_tests_passed','coast_resume_resolved','mechanics_passed','numerics_converged','model_scope_passed')}
    profile=tmp_path/'profile.toml';profile.write_text('physics_mode="physical"')
    with pytest.raises(ValueError,match='hashed evidence'):
        write_release_manifest(tmp_path/'validated.json',report,physics_profile=profile)
