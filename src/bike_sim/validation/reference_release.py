"""Evidence-gated reference profiles and observable scenario outcomes.

A completed course is neither numerical verification nor calibration. Synthetic
measurements can exercise the import path but can never release a validated file.
"""
from dataclasses import asdict,replace
import argparse
import hashlib
import json
from math import atan,isfinite
from pathlib import Path
import time
import traceback
from bike_sim.validation.rider_replay import write_report,resume_schedule

REQUIRED_NUMERICAL_GATES=('known_tests_passed','coast_resume_resolved','mechanics_passed',
                          'numerics_converged','model_scope_passed')
OUTCOMES=('completed','physical_stall','loss_of_traction','contact_loss','crash',
          'model_invalid','numerical_failure','duration_limit')


def release_gates(report: dict) -> tuple[bool,tuple[str,...]]:
    if not isinstance(report,dict):raise ValueError('release report must be a dictionary')
    missing=tuple(key for key in REQUIRED_NUMERICAL_GATES if report.get(key) is not True)
    return not missing,missing


def release_level(report):
    ok,_=release_gates(report)
    if not ok:return 'parameterized_unvalidated'
    # Merely claiming real data and passing holdout is insufficient. The writer
    # below verifies experiment payloads and recomputes their holdout reports.
    return 'numerically_verified_synthetic'


def select_verified_resolution(comparisons):
    """Select only exact tested configurations, never infer a coarser pass.

    Inputs identify fully measured candidates and all three independent axes;
    performance is not a physics criterion. Fastest tested dt, then fewest
    stations, then coarsest tested road mesh among accepted configurations.
    """
    candidates=[]
    for item in comparisons:
        if item.get('passed') is not True or item.get('axes_passed')!={'time':True,'road':True,'stations':True}:
            continue
        dt,dx,n=item.get('dt_s'),item.get('dx_m'),item.get('station_count')
        if type(n) is not int or n<16 or any(type(v) not in (int,float) or not isfinite(v) or v<=0 for v in (dt,dx)):
            raise ValueError('invalid measured resolution')
        candidates.append(item)
    if not candidates:return None
    return dict(min(candidates,key=lambda r:(-r['dt_s'],r['station_count'],-r['dx_m'])))


def write_release_manifest(path,report,*,physics_profile,measurement_evidence=()):
    """Refuse release absent numerical gates or reproducible evidence files.

    measurement_evidence contains real dataset + predictions + predeclared
    calibration_report arguments, not manually supplied acceptance flags.
    This validates internal consistency, not the authenticity of a laboratory.
    """
    from bike_sim.validation.datasets import validate_dataset,calibration_report,dataset_hash
    from bike_sim.validation.experiment_acceptance import accepted_calibration_status,is_synthetic
    ok,reasons=release_gates(report)
    if not ok:raise ValueError('reference release blocked: '+', '.join(reasons))
    evidence=report.get('evidence_files',{})
    for gate in REQUIRED_NUMERICAL_GATES:
        records=evidence.get(gate,[])
        if not records:raise ValueError('release needs hashed evidence for '+gate)
        for record in records:
            file=Path(record['path'])
            if not file.is_file() or hashlib.sha256(file.read_bytes()).hexdigest()!=record.get('sha256'):
                raise ValueError('release evidence file/hash mismatch')
    profile=Path(physics_profile)
    if not profile.is_file():raise ValueError('physics profile is missing')
    status='numerically_verified_synthetic';calibrations=[]
    if measurement_evidence:
        if report.get('synthetic') is not False:raise ValueError('real release must declare actual non-synthetic experiments')
        for evidence in measurement_evidence:
            payload=validate_dataset(evidence['dataset'])
            result=calibration_report(payload,evidence['predictions'],**evidence['arguments'])
            if is_synthetic(payload) or result['holdout_passed'] is not True:
                raise ValueError('measured holdout has not passed')
            if payload['bike_config_hash']!=report.get('configuration_sha256'):
                raise ValueError('holdout measured configuration does not match the release')
            calibrations.append({'dataset_id':payload['dataset_id'],'dataset_hash':dataset_hash(payload),
                                 'holdout':result})
        status=accepted_calibration_status(synthetic=False,holdout_passed=True,converged=True,within_scope=True)
    target=Path(path)
    if 'validated' in target.stem.lower() and status!='validated_within_declared_scope':
        raise ValueError('synthetic evidence cannot produce a validated file')
    manifest={'schema_version':1,'status':status,'gates':{k:report[k] for k in REQUIRED_NUMERICAL_GATES},
              'physics_profile':str(profile),'physics_profile_sha256':hashlib.sha256(profile.read_bytes()).hexdigest(),
              'configuration_sha256':report.get('configuration_sha256'),
              'source_sha256':report.get('source_sha256'),'versions':report.get('versions'),
              'declared_scope':report.get('declared_scope'),'evidence_files':report['evidence_files'],
              'calibration_datasets':calibrations,'scenarios':report.get('scenarios',{})}
    if not manifest['configuration_sha256'] or not manifest['source_sha256'] or not manifest['versions'] or not manifest['declared_scope']:
        raise ValueError('release requires configuration/source versions and explicit scope')
    write_report(target,manifest);return manifest


def classify_outcome(rows,*,goal_x_m=None,crashed=False,numerical_error=None,duration_reached=False):
    """First observable cause; persistence prevents a one-sample stall verdict."""
    if numerical_error is not None:
        last=rows[-1] if rows else {}
        return {'outcome':'numerical_failure','reason':str(numerical_error),
                'time_s':last.get('end_time_s',0.),'position_m':last.get('position_m'),
                'control':last.get('control',{})}
    for row in rows:
        if row.get('model_status',{}).get('numerically_valid') is False:
            return {'outcome':'numerical_failure','reason':'energy quality gate failed',
                    'time_s':row['time_s'],'position_m':row['position_m'],'control':row.get('control',{})}
        status=row.get('model_status',{})
        if status.get('model_valid') is False:
            return {'outcome':'model_invalid','reason':status.get('first_model_violation'),
                    'time_s':row['time_s'],'position_m':row['position_m'],'control':row.get('control',{})}
    if crashed:
        row=rows[-1] if rows else {}
        return {'outcome':'crash','reason':'runtime crash detector','time_s':row.get('end_time_s',0.),
                'position_m':row.get('position_m'),'control':row.get('control',{})}
    if goal_x_m is not None:
        reached=next((r for r in rows if r['position_m']>=goal_x_m),None)
        if reached:
            return {'outcome':'completed','reason':'declared distance reached','time_s':reached['time_s'],
                    'position_m':reached['position_m'],'control':reached.get('control',{})}
    starts={};candidates=[]
    for row in rows:
        loaded=row['front_load_n']+row['rear_load_n']>10.
        effort=row.get('motor_torque_nm',0.)+max(row.get('human_torque_nm',0.),0.)>5.
        slow=abs(row['speed_mps'])<.1
        flags={'physical_stall':row['time_s']>=2. and loaded and effort and slow
               and abs(row.get('crank_rad_s',0.))<.3 and abs(row.get('rear_slip_mps',0.))<.2,
               'loss_of_traction':loaded and effort and slow and abs(row.get('rear_slip_mps',0.))>.5,
               'contact_loss':row['front_load_n']<=5. and row['rear_load_n']<=5.}
        for name,flag in flags.items():
            if not flag:starts.pop(name,None);continue
            starts.setdefault(name,row)
            persistence=1. if name=='contact_loss' else .75
            if row['end_time_s']-starts[name]['time_s']+1e-12>=persistence:
                first=starts[name]
                candidates.append({'outcome':name,'reason':'persistent measured condition',
                    'time_s':first['time_s'],'confirmed_at_s':row['end_time_s'],
                    'position_m':first['position_m'],'control':first.get('control',{}),
                    'evidence':{k:first.get(k) for k in ('speed_mps','crank_rad_s','rear_slip_mps',
                        'front_load_n','rear_load_n','motor_torque_nm','human_torque_nm')}})
        if candidates:return min(candidates,key=lambda c:c['time_s'])
    last=rows[-1] if rows else {}
    return {'outcome':'duration_limit','reason':'declared observation duration ended' if duration_reached else 'no confirmed terminal condition',
            'time_s':last.get('end_time_s',0.),'position_m':last.get('position_m'),'control':last.get('control',{})}


def scenario_definitions():
    scenarios=[{'name':'flat','duration_s':5.,'goal_x_m':5.}]
    scenarios += [{'name':f'grade_{percent}pct','grade':percent/100.,'angle_rad':atan(percent/100.),
                   'duration_s':8.,'goal_x_m':5.} for percent in (5,12,20,28,35)]
    scenarios += [{'name':n,'duration_s':t,'goal_x_m':goal} for n,t,goal in
        [('bump',3.,6.),('crest',3.,7.),('step_4cm',3.,6.),('step_6cm',3.,6.),
         ('two_support',3.,6.),('recontact',1.,None),('coast_resume',8.,None)]]
    scenarios += [{'name':'posture_'+name,'duration_s':3.,'goal_x_m':None,'posture':name}
                  for name in ('seated','standing','forward','rearward')]
    scenarios += [{'name':'extreme_100m','duration_s':90.,'goal_x_m':100.}]
    return scenarios


def _scenario_track(item):
    from bike_sim.terrain import TrackSpec
    from bike_sim.terrain.grade import GradeProfile
    from bike_sim.terrain.obstacles import Bump,SquareEdge
    name=item['name'];obstacles=[];grade=None
    if name=='extreme_100m':
        from bike_sim.terrain.trackfile import load_track
        return load_track(Path(__file__).resolve().parents[3]/'examples/research/rough_uphill_extreme.toml')
    if 'grade' in item:grade=GradeProfile(((0.,item['grade']),(20.,item['grade'])))
    elif name=='bump':obstacles=[Bump(4.,.02,.4)]
    elif name=='crest':obstacles=[Bump(4.,.12,1.5)]
    elif name.startswith('step_'):obstacles=[SquareEdge(4.,.04 if name=='step_4cm' else .06,1.)]
    elif name=='two_support':obstacles=[Bump(4.,.03,.05),Bump(4.26,.03,.05)]
    return TrackSpec(name,20.,obstacles,grade_profile=grade)


def _scenario_control(item,t):
    from bike_sim.sim.ride.control import RideControl
    from bike_sim.physics.rider_posture import RiderPosture
    name=item['name']
    if name=='coast_resume':return resume_schedule(t)
    torque=0. if t<1. else 40.*min(1.,t-1.)
    if name in ('bump','crest','step_4cm','step_6cm','two_support','recontact'):torque=0.
    postures={'seated':RiderPosture(),'standing':RiderPosture.standing(),
              'forward':RiderPosture(torso_lean_rad=.2,pelvis_offset_m=(.08,.02)),
              'rearward':RiderPosture(torso_lean_rad=-.1,pelvis_offset_m=(-.08,.02))}
    return RideControl(human_torque_nm=0.,motor_torque_nm=torque,posture=postures.get(item.get('posture')))


def run_reference_matrix(output_dir,*,physics_profile,dt_s=None,dx_m=.005,names=None):
    """Run every declared case; preserve errors and stop on first unsafe scope loss."""
    import numpy as np
    import mujoco
    import tomllib
    from bike_sim.physics.resolution import resolve_physics_config,resolve_config_paths
    from bike_sim.sim.ride_sim import RideSimulation
    from bike_sim.sim.ride.initial_state import PhysicalInitialState
    from bike_sim.sim.research.configuration import research_field
    from bike_sim.sim.ride.physical_session import configuration_metadata
    from bike_sim.sim.ride.wheelie import WheelieTracker
    from bike_sim.validation.fidelity_sweep import interval_row,metrics_from_rows
    from bike_sim.validation.ride_cases import rebase_initial_state
    from bike_sim.validation.environment import environment_contract,source_fingerprint
    profile=Path(physics_profile)
    with profile.open('rb') as f:base=resolve_config_paths(tomllib.load(f),profile.parent)
    out=Path(output_dir);out.mkdir(parents=True,exist_ok=True)
    items=scenario_definitions()
    if names is not None:
        if not names or set(names)-{i['name'] for i in items}:raise ValueError('unknown/empty reference scenario selection')
        items=[i for i in items if i['name'] in names]
    summary={'schema_version':1,'source_sha256':source_fingerprint(Path(__file__).resolve().parents[1]),
             'environment':environment_contract(),'physics_profile':str(profile),'scenarios':[],
             'release_status':'parameterized_unvalidated'}
    seeds={}
    for item in items:
        name=item['name'];record={'scenario':item,'rows':[]};start=time.monotonic()
        try:
            track=_scenario_track(item)
            speed=2. if name in ('bump','crest','step_4cm','step_6cm','two_support','recontact') else 0.
            override={'initial_speed_mps':speed,'drive':{'human_torque_nm':0.,'shifting':{'enabled':False},'pedaling':{'rollback_brake':False}}}
            if dt_s is not None:override['timestep_s']=dt_s
            cfg=resolve_physics_config(base,override)
            # Same topology/settings and initial flat support: explicit state
            # reuse, not a cache entry with a forged track hash.
            key=(speed,item.get('grade',0.))
            seed=seeds.get(key)
            sim=RideSimulation(track=track,field=research_field(track,dx_m),rider='articulated_planar',
                physics_config=cfg,physical_initial_state=seed)
            if seed is None:
                seeds[key]=PhysicalInitialState.capture(sim)
            if name=='recontact':
                for root in ('root_z','rider_root_z'):
                    sim.data.qpos[sim.physical.address(root)[0]]+=.12
                    sim.data.qvel[sim.physical.address(root)[1]]=-.5
                sim.physical.tire.reset();mujoco.mj_forward(sim.model,sim.data);rebase_initial_state(sim)
                sim.equilibrium['airborne_initial_offset_m']=.12
            record.update(metadata=configuration_metadata(sim),equilibrium=dict(sim.equilibrium),
                initial_state_sha256=PhysicalInitialState.capture(sim).sha256,
                initial_qpos=sim.data.qpos.tolist(),initial_qvel=sim.data.qvel.tolist())
            tracker=WheelieTracker();error=None;dt=float(sim.model.opt.timestep);count=round(item['duration_s']/dt)
            if abs(count*dt-item['duration_s'])>1e-9:raise ValueError('scenario duration must divide timestep')
            # Keep row storage at physics rate; outcome checks only every 0.1 s
            # plus immediate crash/model-invalid to avoid an O(n^2) monitor.
            check_every=max(1,round(.1/dt));result=None
            for index in range(count):
                try:
                    sim.step(control=_scenario_control(item,sim.time_s))
                    row=interval_row(sim,tracker);record['rows'].append(row)
                except Exception as exc:
                    error=f'{type(exc).__name__}: {exc}';break
                if (sim.crash is not None or not row['model_status']['model_valid']
                        or not row['model_status']['numerically_valid'] or index%check_every==0):
                    result=classify_outcome(record['rows'],goal_x_m=item['goal_x_m'],crashed=sim.crash is not None)
                    if result['outcome']!='duration_limit':break
            if error is not None or result is None or result['outcome']=='duration_limit':
                result=classify_outcome(record['rows'],goal_x_m=item['goal_x_m'],crashed=sim.crash is not None,
                                        numerical_error=error,duration_reached=True)
            record['outcome']=result
            if record['rows']:
                record['metrics']=metrics_from_rows(record['rows'],item['duration_s'],record['initial_state_sha256'])
            record['observer']=tracker.metrics
        except Exception as exc:
            record['outcome']={'outcome':'numerical_failure','reason':'initialization: '+f'{type(exc).__name__}: {exc}',
                               'time_s':0.,'position_m':None,'control':asdict(_scenario_control(item,0.))}
            record['traceback']=traceback.format_exc()
        record['wall_time_s']=time.monotonic()-start
        file=name+'.json.gz';write_report(out/file,record)
        summary['scenarios'].append({k:v for k,v in record.items() if k!='rows'}|{'file':file,'interval_count':len(record['rows'])})
        write_report(out/'matrix.json',summary)
        print(name,record['outcome']['outcome'],record['outcome']['time_s'],flush=True)
    return summary


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',default='results/reference-matrix')
    parser.add_argument('--physics-config',default='examples/research/plant_reference_open_loop.toml')
    parser.add_argument('--dt',type=float);parser.add_argument('--dx',type=float,default=.005)
    parser.add_argument('--scenarios',nargs='+')
    args=parser.parse_args(argv)
    result=run_reference_matrix(args.output,physics_profile=args.physics_config,dt_s=args.dt,dx_m=args.dx,names=args.scenarios)
    return 2 if any(r['outcome']['outcome'] in ('model_invalid','numerical_failure') for r in result['scenarios']) else 0

if __name__=='__main__':raise SystemExit(main())
