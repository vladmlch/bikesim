import copy
import pytest
from bike_sim.validation.fidelity_sweep import (interval_integral,scalar_relative_error,
    compare_metrics,metrics_from_rows,persistent_events,case_control,case_physics,DEFAULT_CASES)


def test_interval_contract_and_floors():
    rows=[dict(time_s=0.,end_time_s=.01,load=100.),dict(time_s=.01,end_time_s=.03,load=200.)]
    assert interval_integral(rows,'load')==pytest.approx(5.)
    assert scalar_relative_error(.1,0.,1.)==pytest.approx(.1)
    for value in (float('nan'),True,float('inf')):
        with pytest.raises(ValueError):scalar_relative_error(value,1.,1.)
    for start in (.009,.011):
        rows[1]['time_s']=start
        with pytest.raises(ValueError,match='gap'):interval_integral(rows,'load')


def fixture_rows():
    return [dict(time_s=i*.01,end_time_s=(i+1)*.01,contact_state='two_wheels',front_load_n=400.,rear_load_n=500.,
        front_vertical_force_n=400.,rear_vertical_force_n=500.,fork_travel_m=.03,shock_stroke_m=.01,
        pitch_rad=0.,pitch_rate_rad_s=0.,com_x_m=2.,com_z_m=1.,joint_positive_power_w=0.,motor_torque_nm=0.,
        human_torque_nm=0.,crank_rad_s=0.,energy_residual_ratio=.001,
        model_status={'model_valid':True,'numerically_valid':True}) for i in range(4)]


def test_compares_integrals_events_energy_scope_and_initial_conditions():
    metrics=metrics_from_rows(fixture_rows(),.04,'same-state')
    assert compare_metrics(metrics,metrics)['passed']
    for key,value in [('complete',False),('model_valid',False),('numerically_valid',False),('initial_state_sha256','other')]:
        bad=copy.deepcopy(metrics);bad[key]=value
        assert not compare_metrics(bad,metrics)['passed']
    bad=copy.deepcopy(metrics);bad['contact_events'].append({'state':'flight','time_s':.03})
    result=compare_metrics(bad,metrics)
    assert result['criteria']['contact_event_time_s']['actual'] is None
    assert not result['passed']
    bad=copy.deepcopy(metrics);bad['front_normal_impulse_ns']*=1.03
    assert 'front_normal_impulse_ns' in compare_metrics(bad,metrics)['failed_criteria']


def test_energy_residual_cannot_pass_through_a_stale_validity_flag():
    rows = fixture_rows()
    rows[2]['energy_residual_ratio'] = .2
    metrics = metrics_from_rows(rows, .04, 'same-state')
    assert not metrics['numerically_valid']
    assert not compare_metrics(metrics, metrics)['passed']


def test_resolution_axes_share_an_actually_tested_candidate():
    from bike_sim.validation.fidelity_sweep import resolution_variants
    anchor, variants = resolution_variants(
        (.00125, .000625, .0003125), (.01, .005, .0025), (128, 256, 512))
    assert anchor == (.000625, .005, 256)
    for axis, timestep, road_step, station_count in variants:
        values = (timestep, road_step, station_count)
        axis_index = {'time': 0, 'road': 1, 'stations': 2}[axis]
        assert all(value == anchor[index] for index, value in enumerate(values)
                   if index != axis_index)
    for axis in ('time', 'road', 'stations'):
        assert (axis, *anchor) in variants


def test_persistent_transitions_not_single_sample_noise():
    rows=fixture_rows();rows[2]['contact_state']='flight'
    assert [e['state'] for e in persistent_events(rows)]==['two_wheels']
    rows[3]['contact_state']='flight'
    assert [e['state'] for e in persistent_events(rows)]==['two_wheels','flight']


@pytest.mark.parametrize('case',DEFAULT_CASES)
def test_case_controls_and_fixed_closure_contract(case):
    cfg=case_physics(case,.00125,'compliant_2d',128,'ideal_mid_drive')
    assert cfg.closure_time_constant_s==.0025
    assert not cfg.drive.shifting.enabled and not cfg.drive.pedaling.rollback_brake
    command=case_control(case,1.5)
    command.validate_for(cfg,'articulated_planar')
