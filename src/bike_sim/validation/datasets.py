"""Calibration schemas and whole-experiment holdouts; no synthetic V2 claims."""
import copy
from datetime import date, datetime
import hashlib
import json
import numpy as np
from bike_sim.physics.checks import scalar
from bike_sim.validation.experiment_acceptance import is_synthetic, validate_split

RIG_UNITS={
    'tire':{'force':'N','deflection':'m'},
    'damper':{'force':'N','velocity':'m/s','stroke':'m'},
    'motor':{'torque':'N*m','angular_speed':'rad/s','electrical_power':'W'},
    'axle_loads':{'front_load_n':'N','rear_load_n':'N','slope_rad':'rad'},
    'rider_pose':{'pelvis_x_m':'m','pelvis_z_m':'m','torso_pitch_rad':'rad','com_x_m':'m','com_z_m':'m'},
    'suspension_kinematics':{'rear_travel_m':'m','shock_stroke_m':'m'},
    'full_bike_run':{'time_s':'s','pitch_rad':'rad','pitch_rate_rad_s':'rad/s','speed_mps':'m/s',
                     'fork_travel_m':'m','shock_stroke_m':'m','motor_torque_nm':'N*m','crank_rad_s':'rad/s'},
}
REQUIRED={'dataset_id','source','measured_at','units','bike_config_hash',
          'sensor_uncertainty','conditions','samples','split'}


def _text(value,name):
    if not isinstance(value,str) or not value.strip():
        raise ValueError(f'{name} needs a nonempty identifier')
    return value


def _canonical(value):
    try:
        return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode('utf-8')
    except (TypeError,ValueError) as exc:
        raise ValueError('metadata must be finite JSON data') from exc


def dataset_hash(payload):
    return hashlib.sha256(_canonical(payload)).hexdigest()


def validate_dataset(payload,allow_synthetic=False):
    if not isinstance(payload,dict) or not REQUIRED<=payload.keys():
        raise ValueError('missing dataset metadata')
    if not isinstance(allow_synthetic,bool):
        raise ValueError('allow_synthetic must be a bool')
    split=payload['split']
    if not isinstance(split,dict) or set(split)!={'fit','holdout'}:
        raise ValueError('invalid experiment split')
    groups={}
    for name in ('fit','holdout'):
        group=split[name]
        if not isinstance(group,list) or not all(isinstance(x,str) and x.strip() for x in group):
            raise ValueError('split must list experiment identifiers')
        if len(set(group))!=len(group):
            raise ValueError('duplicate experiment in split')
        groups[name]=set(group)
    fit,holdout=groups['fit'],groups['holdout']
    if fit & holdout:
        raise ValueError('fit/holdout experiment overlap')
    if not fit or not holdout:
        raise ValueError('fit and holdout experiments are both required')
    for name in ('dataset_id','source','measured_at','bike_config_hash'):
        _text(payload[name],name)
    try:
        stamp=payload['measured_at']
        datetime.fromisoformat(stamp.replace('Z','+00:00')) if 'T' in stamp else date.fromisoformat(stamp)
    except ValueError as exc:
        raise ValueError('measured_at must be an ISO date or datetime') from exc
    synthetic=is_synthetic(payload)
    if synthetic and not allow_synthetic:
        raise ValueError('synthetic data cannot validate a measured model')
    units=payload['units']
    if not isinstance(units,dict):
        raise ValueError('missing physical units')
    kinds=[kind for kind,expected in RIG_UNITS.items() if units==expected]
    if len(kinds)!=1:
        raise ValueError('unsupported or incomplete rig units; no implicit unit conversion')
    kind=kinds[0]
    if 'kind' in payload and payload['kind']!=kind:
        raise ValueError('rig kind conflicts with its units')
    uncertainty=payload['sensor_uncertainty']
    if not isinstance(uncertainty,dict) or not set(units)<=uncertainty.keys():
        raise ValueError('sensor uncertainty required for every measured quantity')
    if set(uncertainty)!=set(units):
        raise ValueError('uncertainty contains an unlabelled quantity')
    for key,value in uncertainty.items():
        scalar(value,'sensor uncertainty '+key,minimum=0)
    conditions=payload['conditions']
    if not isinstance(conditions,dict) or not conditions:
        raise ValueError('experiment conditions are required')
    expected={'pressure_pa_gauge'} if kind=='tire' else set()
    if not synthetic:
        expected.add('temperature_c')
        if kind=='motor':
            expected.add('voltage_v')
    if not expected<=conditions.keys():
        raise ValueError('incomplete experiment conditions')
    if 'temperature_c' in conditions and scalar(conditions['temperature_c'],'temperature')<=-273.15:
        raise ValueError('temperature is below absolute zero')
    for name in ('pressure_pa_gauge','voltage_v'):
        if name in conditions:
            scalar(conditions[name],name,minimum=0)
    samples=payload['samples']
    if not isinstance(samples,list) or not samples:
        raise ValueError('nonempty samples required')
    ids=set()
    for row in samples:
        if not isinstance(row,dict) or set(row)!={'experiment_id'}|set(units):
            raise ValueError('sample columns must match the declared units exactly')
        ids.add(_text(row['experiment_id'],'experiment_id'))
        for key in units:
            scalar(row[key],'measurement '+key)
    if kind in {'axle_loads','rider_pose','suspension_kinematics','full_bike_run'}:
        _validate_bicycle_metadata(payload,kind)
    if fit|holdout!=ids:
        raise ValueError('every experiment must have exactly one split')
    _canonical(payload)
    return copy.deepcopy(payload)


def holdout_metrics(measured,predicted):
    try:
        y,p=np.asarray(measured,dtype=float),np.asarray(predicted,dtype=float)
    except (TypeError,ValueError) as exc:
        raise ValueError('invalid holdout vectors') from exc
    if y.shape!=p.shape or y.size==0 or not np.isfinite(y).all() or not np.isfinite(p).all():
        raise ValueError('invalid holdout vectors')
    error=p-y
    with np.errstate(over='ignore',invalid='ignore'):
        rmse=float(np.sqrt(np.mean(error**2)))
    if not np.isfinite(rmse):
        raise ValueError('holdout error overflow')
    return {'rmse':rmse,'max_abs':float(np.max(np.abs(error))),'bias':float(np.mean(error))}


def calibration_report(payload,predictions,*,output,parameters,bounds,error_budget,allow_synthetic=False):
    """Report fit and holdout separately against an explicitly identified budget.

    This does not fit parameters, pick thresholds from holdout errors, or prove
    that a caller's claimed source is a physical experiment. The recorded budget
    ID/hash permits comparison with a pre-registered experimental protocol.
    """
    data=validate_dataset(payload,allow_synthetic=allow_synthetic)
    if output not in data['units']:
        raise ValueError('output quantity lacks units')
    if not isinstance(parameters,dict) or not parameters or set(parameters)!=set(bounds):
        raise ValueError('every parameter needs declared bounds')
    for name,value in parameters.items():
        value=scalar(value,'fitted parameter')
        pair=bounds[name]
        if not isinstance(pair,(list,tuple)) or len(pair)!=2:
            raise ValueError('parameter bound must be a pair')
        lo,hi=(scalar(x,'parameter bound') for x in pair)
        if not lo<=value<=hi or lo>=hi:
            raise ValueError('parameter lies outside its declared bound')
    if not isinstance(error_budget,dict) or set(error_budget)!={'budget_id','rmse','max_abs','bias_abs'}:
        raise ValueError('predeclared budget ID and all three error limits are required')
    _text(error_budget['budget_id'],'error budget')
    for key in ('rmse','max_abs','bias_abs'):
        scalar(error_budget[key],'error budget '+key,minimum=0)
    ids=set(data['split']['fit'])|set(data['split']['holdout'])
    if not isinstance(predictions,dict) or set(predictions)!=ids:
        raise ValueError('predictions must cover every fit/holdout experiment exactly')
    experiments={}
    for eid in sorted(ids):
        y=[row[output] for row in data['samples'] if row['experiment_id']==eid]
        metrics=holdout_metrics(y,predictions[eid])
        passed=(metrics['rmse']<=error_budget['rmse'] and metrics['max_abs']<=error_budget['max_abs']
                and abs(metrics['bias'])<=error_budget['bias_abs'])
        experiments[eid]={'split':'fit' if eid in data['split']['fit'] else 'holdout',
                          'metrics':metrics,'passed':bool(passed),'sample_count':len(y)}
    passed=all(experiments[eid]['passed'] for eid in data['split']['holdout'])
    synthetic=is_synthetic(data)
    result={'dataset_id':data['dataset_id'],'dataset_hash':dataset_hash(data),'output':output,
            'unit':data['units'][output],'source':data['source'],
            'parameters':copy.deepcopy(parameters),'bounds':copy.deepcopy(bounds),
            'sensor_uncertainty':copy.deepcopy(data['sensor_uncertainty']),
            'conditions':copy.deepcopy(data['conditions']),'error_budget':copy.deepcopy(error_budget),
            'error_budget_hash':dataset_hash(error_budget),'experiments':experiments,
            'holdout_passed':passed,
            'calibration_status':'measured_holdout_passed' if passed and not synthetic else 'parameterized_unvalidated'}
    _canonical(result)
    return result


def _validate_bicycle_metadata(payload,kind):
    """No implicit sensors, unit conversions, zero uncertainty or split leakage."""
    validate_split(payload['split']['fit'],payload['split']['holdout'])
    required={'rider_config_hash','sensor_calibration','synchronisation_method',
              'quantity_provenance','experiment_metadata'}
    if not required<=payload.keys():
        raise ValueError('missing bicycle/rider measurement metadata')
    _text(payload['rider_config_hash'],'rider configuration hash')
    _text(payload['synchronisation_method'],'synchronisation method')
    units=payload['units']; provenance=payload['quantity_provenance']
    if not isinstance(provenance,dict) or set(provenance)!=set(units):
        raise ValueError('quantity provenance required for every channel')
    calibration=payload['sensor_calibration']
    if not isinstance(calibration,dict):
        raise ValueError('sensor calibration must identify measured channels')
    for channel,entry in provenance.items():
        if not isinstance(entry,dict) or entry.get('kind') not in ('sensor','estimate'):
            raise ValueError('channel must be explicitly sensor or estimate')
        _text(entry.get('method'),'channel method')
        if entry['kind']=='sensor':
            _text(calibration.get(channel),'sensor calibration identifier')
        else:
            if 'estimate_uncertainty' not in entry:
                raise ValueError('estimated channels need separate uncertainty')
            scalar(entry['estimate_uncertainty'],'estimate uncertainty',minimum=0.)
    experiments=payload['experiment_metadata']
    ids=set(payload['split']['fit'])|set(payload['split']['holdout'])
    if not isinstance(experiments,dict) or set(experiments)!=ids:
        raise ValueError('metadata must cover whole experiments exactly')
    mandatory={'bike_mass_kg','rider_mass_kg','front_pressure_pa_gauge',
               'rear_pressure_pa_gauge','temperature_c','front_teeth','rear_teeth',
               'bike_config_hash','rider_config_hash'}
    for eid,conditions in experiments.items():
        if not isinstance(conditions,dict) or not mandatory<=conditions.keys():
            raise ValueError('incomplete experiment mass/pressure/temperature/gear metadata')
        for name in ('bike_mass_kg','rider_mass_kg','front_pressure_pa_gauge','rear_pressure_pa_gauge'):
            value=scalar(conditions[name],name,minimum=0.)
            if name!='rider_mass_kg' and value==0.:
                raise ValueError('positive mass and pressure required')
        if scalar(conditions['temperature_c'],'temperature')<=-273.15:
            raise ValueError('temperature is below absolute zero')
        for name in ('front_teeth','rear_teeth'):
            if type(conditions[name]) is not int or conditions[name]<=0:
                raise ValueError('gear tooth counts must be positive integers')
        for name in ('bike_config_hash','rider_config_hash'):
            _text(conditions[name],name)
            if conditions[name]!=payload[name]:
                raise ValueError('mixed configuration hashes require separate datasets')
        if kind=='full_bike_run':
            times=[r['time_s'] for r in payload['samples'] if r['experiment_id']==eid]
            if any(b<=a for a,b in zip(times,times[1:])):
                raise ValueError('full-bike times must increase within each experiment')
