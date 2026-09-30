import copy
import pytest
from bike_sim.validation.experiment_acceptance import accepted_calibration_status,validate_split
from bike_sim.validation.datasets import RIG_UNITS,validate_dataset


def test_whole_experiment_split():
    validate_split(['a'],['b'])
    for a,b in [(['a'],['a']),(['a','a'],['b']),([],['b']),([' a'],['b']),('a',['b'])]:
        with pytest.raises(ValueError):validate_split(a,b)


def test_no_synthetic_validation_or_truthy_flags():
    assert accepted_calibration_status(synthetic=True,holdout_passed=True,converged=True,within_scope=True)=='parameterized_unvalidated'
    assert accepted_calibration_status(synthetic=False,holdout_passed=True,converged=True,within_scope=True)=='validated_within_declared_scope'
    with pytest.raises(ValueError):accepted_calibration_status(synthetic=0,holdout_passed=True,converged=True,within_scope=True)


def bicycle_payload(kind):
    units=RIG_UNITS[kind]
    conditions=dict(bike_mass_kg=25.,rider_mass_kg=80.,front_pressure_pa_gauge=180000.,
                    rear_pressure_pa_gauge=200000.,temperature_c=20.,front_teeth=34,rear_teeth=51,
                    bike_config_hash='bike-fixture',rider_config_hash='rider-fixture')
    return dict(kind=kind,dataset_id='fixture',source='synthetic:test',synthetic=True,measured_at='2026-09-30',
        bike_config_hash='bike-fixture',rider_config_hash='rider-fixture',units=units,
        sensor_uncertainty={k:.01 for k in units},conditions=conditions,
        synchronisation_method='shared fixture clock',sensor_calibration={k:'fixture-not-laboratory' for k in units},
        quantity_provenance={k:{'kind':'sensor','method':'synthetic schema fixture'} for k in units},
        experiment_metadata={eid:copy.deepcopy(conditions) for eid in ('fit-1','holdout-2')},
        split={'fit':['fit-1'],'holdout':['holdout-2']},
        samples=[dict(experiment_id=eid,**{k:0. for k in units}) for eid in ('fit-1','holdout-2')])


@pytest.mark.parametrize('kind',['axle_loads','rider_pose','suspension_kinematics','full_bike_run'])
def test_bicycle_contracts_and_real_data_gate(kind):
    data=bicycle_payload(kind)
    assert validate_dataset(data,allow_synthetic=True)==data
    with pytest.raises(ValueError,match='synthetic'):validate_dataset(data)
    for key in ('rider_config_hash','synchronisation_method','sensor_calibration','experiment_metadata','quantity_provenance'):
        bad=copy.deepcopy(data);del bad[key]
        with pytest.raises(ValueError):validate_dataset(bad,allow_synthetic=True)
    bad=copy.deepcopy(data);bad['experiment_metadata']['fit-1']['front_teeth']=34.
    with pytest.raises(ValueError):validate_dataset(bad,allow_synthetic=True)


def test_estimates_have_separate_uncertainty_and_time_order():
    data=bicycle_payload('full_bike_run');key='motor_torque_nm'
    data['quantity_provenance'][key]={'kind':'estimate','method':'current observer'}
    with pytest.raises(ValueError,match='uncertainty'):validate_dataset(data,allow_synthetic=True)
    data['quantity_provenance'][key]['estimate_uncertainty']=2.
    validate_dataset(data,allow_synthetic=True)
    data['samples'].append(copy.deepcopy(data['samples'][0]))
    with pytest.raises(ValueError,match='increase'):validate_dataset(data,allow_synthetic=True)
