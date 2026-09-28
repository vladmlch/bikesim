import copy
import numpy as np
import pytest
from bike_sim.validation.datasets import validate_dataset, holdout_metrics, dataset_hash, calibration_report


def payload(source='synthetic'):
    return {'dataset_id':'tire-1','source':source,'measured_at':'2026-09-28',
            'units':{'force':'N','deflection':'m'},'bike_config_hash':'fixture',
            'sensor_uncertainty':{'force':1.,'deflection':.00001},
            'conditions':{'pressure_pa_gauge':200000.,'temperature_c':20.},
            'samples':[{'experiment_id':'a','force':100.,'deflection':.001},
                       {'experiment_id':'b','force':200.,'deflection':.002}],
            'split':{'fit':['a'],'holdout':['b']}}


def test_disjoint_whole_experiments_and_no_input_mutation():
    p=payload(); out=validate_dataset(p,allow_synthetic=True)
    out['samples'][0]['force']=0.
    assert p['samples'][0]['force']==100.
    p['split']['holdout']=['a']
    with pytest.raises(ValueError,match='overlap'): validate_dataset(p,allow_synthetic=True)


@pytest.mark.parametrize('change',[
    lambda p:p.pop('source'), lambda p:p['units'].update(force='lbf'),
    lambda p:p['split'].update(holdout=[]), lambda p:p['split'].update(holdout=['unknown']),
    lambda p:p['samples'][0].update(force=float('nan')),
    lambda p:p['samples'][0].update(force=True), lambda p:p.update(measured_at='yesterday'),
    lambda p:p.update(source=''), lambda p:p['sensor_uncertainty'].update(force=-1),
    lambda p:p['samples'][0].update(unlabelled=3.),
    lambda p:p['split'].update(fit=['a','a']), lambda p:p['conditions'].update(pressure_pa_gauge=float('inf')),
])
def test_invalid_dataset_rejected(change):
    p=payload(); change(p)
    with pytest.raises(ValueError): validate_dataset(p,allow_synthetic=True)


def test_synthetic_is_not_measured_even_with_good_predictions():
    p=payload()
    with pytest.raises(ValueError,match='synthetic'): validate_dataset(p)
    report=calibration_report(p,{'a':[100.],'b':[200.]},output='force',parameters={'k':100000.},
        bounds={'k':[10000.,200000.]},error_budget={'budget_id':'before-fit-1','rmse':2.,'max_abs':3.,'bias_abs':2.},allow_synthetic=True)
    assert report['holdout_passed']
    assert report['calibration_status']=='parameterized_unvalidated'
    assert report['experiments']['a']['split']=='fit'
    assert report['experiments']['b']['split']=='holdout'


def test_measured_requires_complete_conditions_and_uncertainty():
    p=payload('lab:stand-5:run-8')
    assert validate_dataset(p)==p
    del p['conditions']['temperature_c']
    with pytest.raises(ValueError,match='conditions'): validate_dataset(p)
    p=payload('lab:run'); del p['sensor_uncertainty']['deflection']
    with pytest.raises(ValueError,match='uncertainty'): validate_dataset(p)


def test_holdout_metrics_are_not_training_errors():
    assert holdout_metrics([1.,2.,3.],[2.,3.,4.])==pytest.approx({'rmse':1.,'max_abs':1.,'bias':1.})
    with pytest.raises(ValueError): holdout_metrics([],[])
    with pytest.raises(ValueError): holdout_metrics([1.],[float('nan')])


def test_hash_canonical_and_parameter_bound_check():
    p=payload(); q=dict(reversed(list(p.items())))
    assert dataset_hash(p)==dataset_hash(q)
    q=copy.deepcopy(p); q['conditions']['pressure_pa_gauge']+=1
    assert dataset_hash(p)!=dataset_hash(q)
    with pytest.raises(ValueError,match='bound'):
        calibration_report(p,{'a':[100.],'b':[200.]},output='force',parameters={'k':100.},
            bounds={'k':[1.,10.]},error_budget={'budget_id':'x','rmse':1.,'max_abs':1.,'bias_abs':1.},allow_synthetic=True)


@pytest.mark.parametrize('units,row,conditions',[
 ({'force':'N','velocity':'m/s','stroke':'m'},{'force':100.,'velocity':.1,'stroke':.01},{'temperature_c':20.}),
 ({'torque':'N*m','angular_speed':'rad/s','electrical_power':'W'},
  {'torque':20.,'angular_speed':8.,'electrical_power':200.},{'temperature_c':20.,'voltage_v':36.}),
])
def test_rig_specific_units(units,row,conditions):
    p=payload('lab:run');p['units']=units;p['conditions']=conditions
    p['sensor_uncertainty']={k:.01 for k in units}
    p['samples']=[dict(row,experiment_id=key) for key in ('a','b')]
    assert validate_dataset(p)==p
