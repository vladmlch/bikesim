"""Orthogonal full-bicycle time/terrain/contact-resolution experiments.

Every row is an incoming physics interval. Failed, aborted and out-of-scope
series remain in the report; none are accepted as numerical convergence.
"""
from dataclasses import asdict,replace
from itertools import combinations
from math import isfinite
from pathlib import Path
import argparse
import json
import time
import traceback
import numpy as np
from bike_sim.validation.rider_replay import write_report,resume_schedule,open_loop_schedule

TIME_STEPS=(.00125,.000625,.0003125)
ROAD_STEPS=(.01,.005,.0025)
STATIONS=(128,256,512)
DEFAULT_CASES=('static_sag','motor_ramp_grade','bump_coast','crest_coast','step_4cm','recontact','coast_resume')


def resolution_variants(time_steps, road_steps, station_counts, *, include_stations=True):
    axes = (tuple(time_steps), tuple(road_steps), tuple(station_counts))
    for index, values in enumerate(axes):
        if (len(values) < 3 or len(set(values)) != len(values)
                or any(isinstance(value, bool) or not isfinite(value) or value <= 0. for value in values)):
            raise ValueError('resolution axes require at least three distinct positive values')
        if index == 2 and any(type(value) is not int or value < 16 for value in values):
            raise ValueError('station counts must be integers of at least sixteen')
        if tuple(sorted(values, reverse=index != 2)) != values:
            raise ValueError('time and road steps decrease; station counts increase')
    anchor = (axes[0][-2], axes[1][-2], axes[2][-2])
    variants = [('time', value, anchor[1], anchor[2]) for value in axes[0]]
    variants += [('road', anchor[0], value, anchor[2]) for value in axes[1]]
    if include_stations:
        variants += [('stations', anchor[0], anchor[1], value) for value in axes[2]]
    return anchor, variants


def scalar_relative_error(value: float, reference: float, floor: float) -> float:
    if any(isinstance(v,bool) or not isfinite(v) for v in (value,reference,floor)) or floor<=0:
        raise ValueError('finite comparison and positive floor required')
    return abs(value-reference)/max(abs(reference),floor)


def interval_integral(rows: list[dict], key: str) -> float:
    result=0.;previous_end=None
    for row in rows:
        t,end,value=row['time_s'],row['end_time_s'],row[key]
        if any(isinstance(v,bool) or not isfinite(v) for v in (t,end,value)) or end<=t:
            raise ValueError('invalid force interval')
        if previous_end is not None and abs(t-previous_end)>1e-9:
            raise ValueError('gap or overlap in force history')
        result+=value*(end-t);previous_end=end
    if not isfinite(result):raise ValueError('interval integral overflow')
    return result


def _criterion(actual,limit):
    return {'actual':float(actual),'maximum':float(limit),'passed':bool(isfinite(actual) and actual<=limit)}


def compare_metrics(coarse: dict, reference: dict) -> dict:
    """Compare integrals, extrema and matched persistent event sequences.

    A missing event is a categorical mismatch, never time zero. Convergence
    cannot be inferred from two identical early failures or empty histories.
    """
    criteria={}
    for key in ('front_normal_impulse_ns','rear_normal_impulse_ns',
                'front_vertical_impulse_ns','rear_vertical_impulse_ns'):
        criteria[key]=_criterion(scalar_relative_error(coarse[key],reference[key],.1),.02)
    for side in ('front','rear'):
        for metric in ('mean_load_n','min_load_n'):
            key=side+'_'+metric
            criteria[key]=_criterion(scalar_relative_error(coarse[key],reference[key],10.),.02)
    for key in ('mean_fork_travel_m','max_fork_travel_m','mean_shock_stroke_m','max_shock_stroke_m'):
        criteria[key]=_criterion(abs(coarse[key]-reference[key]),.001)
    criteria['peak_pitch_rate_rad_s']=_criterion(scalar_relative_error(
        coarse['peak_pitch_rate_rad_s'],reference['peak_pitch_rate_rad_s'],.05),.05)
    a,b=coarse['contact_events'],reference['contact_events']
    matched=len(a)==len(b) and all(x['state']==y['state'] for x,y in zip(a,b))
    criteria['event_sequence']={'passed':matched,'actual':[x['state'] for x in a],
                                'reference':[x['state'] for x in b]}
    if matched:
        criteria['contact_event_time_s']=_criterion(max((abs(x['time_s']-y['time_s']) for x,y in zip(a,b)),default=0.),.005)
    else:
        criteria['contact_event_time_s']={'passed':False,'actual':None,'maximum':.005,'reason':'categorical mismatch'}
    criteria['full_duration']={'passed':coarse.get('complete') is True and reference.get('complete') is True
        and abs(coarse['duration_s']-reference['duration_s'])<1e-9}
    criteria['model_scope']={'passed':coarse.get('model_valid') is True and reference.get('model_valid') is True,
                            'agreement':coarse.get('model_valid')==reference.get('model_valid')}
    criteria['energy_quality']={'passed':coarse.get('numerically_valid') is True and reference.get('numerically_valid') is True,
        'coarse_max_residual_ratio':coarse.get('max_energy_residual_ratio'),
        'reference_max_residual_ratio':reference.get('max_energy_residual_ratio')}
    # Independent equilibria must not be advertised as an integrator-only test.
    criteria['identical_initial_state']={'passed':coarse.get('initial_state_sha256') is not None and
        coarse.get('initial_state_sha256')==reference.get('initial_state_sha256')}
    return {'passed':all(v['passed'] for v in criteria.values()),'criteria':criteria,
            'failed_criteria':[k for k,v in criteria.items() if not v['passed']]}


def case_track(case):
    from bike_sim.terrain import TrackSpec
    from bike_sim.terrain.grade import GradeProfile
    from bike_sim.terrain.obstacles import Bump,SquareEdge
    if case not in DEFAULT_CASES:raise ValueError('unknown fidelity case')
    obstacles=[];grade=None
    if case=='motor_ramp_grade':grade=GradeProfile(((0.,.05),(20.,.05)))
    elif case=='bump_coast':obstacles=[Bump(4.,.02,.4)]
    elif case=='crest_coast':obstacles=[Bump(4.,.12,1.5)]
    elif case=='step_4cm':obstacles=[SquareEdge(4.,.04,1.)]
    return TrackSpec(case,20.,obstacles,grade_profile=grade)


def case_duration(case):
    return {'static_sag':.25,'motor_ramp_grade':5.,'bump_coast':2.,'crest_coast':3.,
            'step_4cm':2.,'recontact':1.,'coast_resume':8.}[case]


def case_control(case,time_s):
    from bike_sim.sim.ride.control import RideControl
    if case=='coast_resume':return resume_schedule(time_s)
    if case=='motor_ramp_grade':return open_loop_schedule(time_s)
    if case=='static_sag':return RideControl(motor_torque_nm=0.,human_torque_nm=0.)
    return RideControl(motor_torque_nm=0.,human_torque_nm=0.)


def case_physics(case,dt_s,backend,station_count,transmission,physics_path=None):
    import tomllib
    from bike_sim.physics.resolution import resolve_physics_config,resolve_config_paths
    path=Path(physics_path) if physics_path is not None else Path(__file__).resolve().parents[3]/'examples/research/plant_reference_open_loop.toml'
    with path.open('rb') as f:base=resolve_config_paths(tomllib.load(f),path.parent)
    return resolve_physics_config(base,{'timestep_s':dt_s,'closure_time_constant_s':.0025,
        'initial_speed_mps':2. if case in ('bump_coast','crest_coast','step_4cm','recontact') else 0.,
        'tires':{'backend':backend,'distributed':{'station_count':station_count}},
        'drive':{'transmission_model':transmission},'equilibrium_cache_enabled':True})


def make_case_sim(case,dt_s,dx_m,station_count,*,backend='compliant_2d',
                  transmission='ideal_mid_drive',initial_state=None,physics_path=None):
    import mujoco
    from bike_sim.sim.ride_sim import RideSimulation
    from bike_sim.sim.research.configuration import research_field
    from bike_sim.validation.ride_cases import rebase_initial_state
    track=case_track(case)
    sim=RideSimulation(track=track,field=research_field(track,dx_m),rider='articulated_planar',
        physics_config=case_physics(case,dt_s,backend,station_count,transmission,physics_path),physical_initial_state=initial_state)
    if case=='recontact' and initial_state is None:
        for name in ('root_z','rider_root_z'):sim.data.qpos[sim.physical.address(name)[0]]+=.12
        sim.data.qvel[sim.physical.address('root_z')[1]]=-.5
        sim.data.qvel[sim.physical.address('rider_root_z')[1]]=-.5
        sim.physical.tire.reset();mujoco.mj_forward(sim.model,sim.data)
        sim.equilibrium['initial_condition_fixture']={'airborne_height_increment_m':.12,'vertical_velocity_mps':-.5}
        rebase_initial_state(sim)
    return sim


def interval_row(sim,tracker):
    from bike_sim.sim.ride.wheelie import truth_from_sample
    from bike_sim.sim.research.quality import energy_quality
    s=sim.physical.sample;c=s.channels;dt=s.end_time_s-s.time_s
    truth=truth_from_sample(sim,s);state=tracker.update(truth,dt)
    quality = energy_quality(c['energy'])
    row={'time_s':s.time_s,'end_time_s':s.end_time_s,'contact_state':state,
         'position_m':truth.position_m,'speed_mps':truth.speed_mps,
         'pitch_rad':truth.pitch_up_rad,'pitch_rate_rad_s':truth.pitch_rate_up_rad_s,
         'com_x_m':truth.com_x_m,'com_z_m':truth.com_z_m,
         'fork_travel_m':c['suspension']['fork_travel_m'],'shock_stroke_m':c['suspension']['shock_stroke_m'],
         'human_torque_nm':c['drive'].get('human_sensor_nm',0.),'motor_torque_nm':c['drive'].get('motor_torque_nm',0.),
         'crank_rad_s':c['drive'].get('crank_rad_s',0.),'rear_slip_mps':truth.rear_slip_mps,
         'joint_positive_power_w':c.get('rider_positive_power_w',0.),'control':dict(c['control']),
         'model_status':dict(c['model_status']),'energy':dict(c['energy']),
         'energy_residual_ratio':quality.residual_ratio}
    row['model_status']['numerically_valid'] = (
        row['model_status']['numerically_valid'] is True and quality.acceptable)
    for side in ('front','rear'):
        row[side+'_load_n']=c['tires'][side]['normal_load_n']
        row[side+'_vertical_force_n']=c['tires'][side]['vertical_force_n']
        row[side+'_clearance_m']=getattr(truth,side+'_clearance_m')
    return row


def persistent_events(rows,persistence_s=.02):
    """WheelieTracker supplies labels; persistence only timestamps transitions."""
    events=[];accepted=None;pending=None;start=0.
    for row in rows:
        state=row['contact_state']
        if state!=pending:pending=state;start=row['time_s']
        if state!=accepted and row['end_time_s']-start+1e-12>=persistence_s:
            accepted=state;events.append({'state':state,'time_s':start,'confirmed_at_s':row['end_time_s']})
    return events


def metrics_from_rows(rows,expected_duration_s,initial_state_sha256):
    if not rows:raise ValueError('no completed physics intervals')
    duration=interval_integral([{**r,'one':1.} for r in rows],'one')
    # Validate all force histories independently; no silent missing-tail average.
    metrics={'duration_s':duration,'complete':abs(duration-expected_duration_s)<1e-8,
        'initial_state_sha256':initial_state_sha256,'contact_events':persistent_events(rows),
        'peak_pitch_rate_rad_s':max(abs(r['pitch_rate_rad_s']) for r in rows),
        'model_valid':all(r['model_status']['model_valid'] is True for r in rows),
        'numerically_valid':all(r['model_status']['numerically_valid'] is True
                               and isfinite(r['energy_residual_ratio'])
                               and r['energy_residual_ratio'] <= .05 for r in rows),
        'max_energy_residual_ratio':max(r['energy_residual_ratio'] for r in rows),
        'joint_positive_work_j':interval_integral(rows,'joint_positive_power_w'),
        'motor_shaft_work_j':interval_integral([{**r,'power':r['motor_torque_nm']*r['crank_rad_s']} for r in rows],'power'),
        'human_crank_work_j':interval_integral([{**r,'power':r['human_torque_nm']*r['crank_rad_s']} for r in rows],'power'),
        'max_pitch_rad':max(r['pitch_rad'] for r in rows),
        'com_dx_m':rows[-1]['com_x_m']-rows[0]['com_x_m'],
        'com_dz_m':rows[-1]['com_z_m']-rows[0]['com_z_m']}
    for side in ('front','rear'):
        metrics[side+'_normal_impulse_ns']=interval_integral(rows,side+'_load_n')
        metrics[side+'_vertical_impulse_ns']=interval_integral(rows,side+'_vertical_force_n')
        metrics[side+'_mean_load_n']=metrics[side+'_normal_impulse_ns']/duration
        metrics[side+'_min_load_n']=min(r[side+'_load_n'] for r in rows)
    for key in ('fork_travel_m','shock_stroke_m'):
        metrics['mean_'+key]=interval_integral(rows,key)/duration
        metrics['max_'+key]=max(r[key] for r in rows)
    return metrics


def run_case(sim,case,*,duration_s=None,output=None,initial_state_sha256=None):
    from bike_sim.sim.ride.wheelie import WheelieTracker
    from bike_sim.sim.ride.physical_session import configuration_metadata
    from bike_sim.sim.ride.initial_state import PhysicalInitialState
    duration=case_duration(case) if duration_s is None else duration_s
    dt=float(sim.model.opt.timestep);count=round(duration/dt)
    if count<1 or abs(count*dt-duration)>1e-9:raise ValueError('duration must contain whole intervals')
    initial=PhysicalInitialState.capture(sim)
    report={'case':case,'metadata':configuration_metadata(sim),'equilibrium':dict(sim.equilibrium),
            'initial_state_sha256':initial_state_sha256 or initial.sha256,
            'serialized_initial_state_sha256':initial.sha256,
            'station_projection':None if sim.physical_initial_state is None else sim.physical_initial_state.payload.get('station_projection'),
            'initial_qpos':sim.data.qpos.tolist(),'initial_qvel':sim.data.qvel.tolist(),
            'expected_duration_s':duration,'rows':[]}
    tracker=WheelieTracker();start=time.monotonic()
    try:
        for _ in range(count):
            sim.step(1. if case=="static_sag" else 0.,1. if case=="static_sag" else 0.,
                     control=case_control(case,sim.time_s))
            report['rows'].append(interval_row(sim,tracker))
            if not report['rows'][-1]['model_status']['numerically_valid']:
                report['termination']='numerical_quality';break
            if sim.crash is not None:
                report['termination']='crash';break
            if not sim.physical.model_status.as_dict()['model_valid']:
                report['termination']='model_invalid';break
        else:report['termination']='duration_completed'
    except Exception as exc:
        report.update(termination='numerical_failure',error=f'{type(exc).__name__}: {exc}',traceback=traceback.format_exc())
    report['wall_time_s']=time.monotonic()-start
    if report['rows']:
        report['metrics']=metrics_from_rows(report['rows'],duration,report['initial_state_sha256'])
        if report['termination']=='numerical_failure':report['metrics']['numerically_valid']=False
    else:report['metrics']=None
    report['wheelie_observer']=tracker.metrics
    if output is not None:write_report(output,report)
    return report


def run_fidelity_sweep(output_dir: str,cases: tuple[str,...]=DEFAULT_CASES,*,
                       backend='compliant_2d',transmission='ideal_mid_drive',include_stations=False,physics_path=None,
                       time_steps=TIME_STEPS,road_steps=ROAD_STEPS,station_counts=STATIONS):
    from bike_sim.sim.ride.initial_state import PhysicalInitialState
    from bike_sim.validation.environment import environment_contract,source_fingerprint
    if not cases or len(set(cases))!=len(cases) or any(c not in DEFAULT_CASES for c in cases):
        raise ValueError('distinct known fidelity cases required')
    out=Path(output_dir);out.mkdir(parents=True,exist_ok=True)
    anchor, variants = resolution_variants(time_steps, road_steps, station_counts,
                                          include_stations=include_stations)
    if include_stations and backend!='distributed_2d_reference':
        raise ValueError('station sweep requires distributed backend')
    report={'schema_version':1,'backend':backend,'transmission':transmission,'physics_profile':str(physics_path),
        'closure_time_constant_s':.0025,
        'environment':environment_contract(),'source_sha256':source_fingerprint(Path(__file__).resolve().parents[1]),
        'series':[],'comparisons':[],'passed':False,
        'candidate_resolution':{'dt_s':anchor[0],'dx_m':anchor[1],'station_count':anchor[2]},
        'time_steps':list(time_steps),'road_steps':list(road_steps),'station_counts':list(station_counts),
        'station_initialization':'same qpos/qvel; explicit periodic angular material-field projection, target residual recorded'}
    for case in cases:
        seed=None;rows={};station_seeds={}
        try:
            initial_sim = make_case_sim(case,*anchor,backend=backend,
                                       transmission=transmission,physics_path=physics_path)
            seed = PhysicalInitialState.capture(initial_sim)
            (out/(case+'_initial_state.json')).write_text(seed.payload_json+'\n')
            if include_stations:
                station_seeds={count:PhysicalInitialState.project_stations(initial_sim,count)
                               for count in station_counts if count!=anchor[2]}
        except Exception as error:
            report.setdefault('initialization_errors',{})[case]=f'{type(error).__name__}: {error}'
        for axis,dt,dx,n in variants:
            key=(dt,dx,n)
            if key in rows:continue
            name=f'{case}_dt{dt:.7f}_dx{dx:g}_n{n}'
            item={'case':case,'dt_s':dt,'dx_m':dx,'station_count':n,'axis':axis,'file':name+'.json.gz'}
            started=time.monotonic()
            try:
                if seed is None:
                    raise ValueError('shared candidate initialization failed')
                target_seed=seed if n==anchor[2] else station_seeds[n]
                sim=make_case_sim(case,dt,dx,n,backend=backend,transmission=transmission,initial_state=target_seed,physics_path=physics_path)
                result=run_case(sim,case,output=out/item['file'],initial_state_sha256=seed.sha256)
                item.update(metrics=result['metrics'],termination=result['termination'],metadata=result['metadata'],
                            equilibrium=result['equilibrium'],runtime_s=result['wall_time_s'])
            except Exception as exc:
                item.update(metrics=None,termination='initialization_failure',error=f'{type(exc).__name__}: {exc}')
            item['total_wall_time_s']=time.monotonic()-started
            rows[key]=item;report['series'].append(item)
            write_report(out/'report.json',report)
            print(name,item['termination'],flush=True)
        groups={'time':[(dt,anchor[1],anchor[2]) for dt in time_steps],
                'road':[(anchor[0],dx,anchor[2]) for dx in road_steps]}
        if include_stations:groups['stations']=[(anchor[0],anchor[1],count) for count in station_counts]
        for axis,keys in groups.items():
            for ai,bi in combinations(range(len(keys)),2):
                a,b=rows[keys[ai]],rows[keys[bi]]
                comparison={'case':case,'axis':axis,'coarse':a['file'],'reference':b['file'],
                            'finest_pair':(ai,bi)==(len(keys)-2,len(keys)-1)}
                try:
                    if a['metrics'] is None or b['metrics'] is None:raise ValueError('missing/aborted series')
                    comparison.update(compare_metrics(a['metrics'],b['metrics']))
                except (ValueError,KeyError,TypeError) as exc:comparison.update(passed=False,failed_criteria=['incomplete_evidence'],error=str(exc))
                report['comparisons'].append(comparison)
    finest=[r for r in report['comparisons'] if r['finest_pair']]
    report['passed']=bool(finest) and all(r['passed'] for r in finest)
    report['failed_criteria']=sorted({k for r in finest for k in r.get('failed_criteria',[])})
    report['axes_passed']={axis:bool([item for item in finest if item['axis']==axis])
        and all(item['passed'] for item in finest if item['axis']==axis)
        for axis in ('time','road','stations')}
    report['source_unchanged_during_run']=report['source_sha256']==source_fingerprint(Path(__file__).resolve().parents[1])
    report['passed']=report['passed'] and report['source_unchanged_during_run']
    report['maximum_errors']={key:max(v['actual'] for r in report['comparisons'] for k,v in r.get('criteria',{}).items()
        if k==key and isinstance(v.get('actual'),(int,float)) and not isinstance(v['actual'],bool))
        for key in sorted({k for r in report['comparisons'] for k,v in r.get('criteria',{}).items()
            if isinstance(v.get('actual'),(int,float)) and not isinstance(v['actual'],bool)})}
    write_report(out/'report.json',report)
    return report


def run_fidelity_parallel(output_dir,cases=DEFAULT_CASES,*,jobs=3,**settings):
    """Independent processes own models and files; never share mutable physics."""
    from concurrent.futures import ProcessPoolExecutor,as_completed
    from multiprocessing import get_context
    from bike_sim.validation.environment import environment_contract,source_fingerprint
    if type(jobs) is not int or not 1<=jobs<=16:raise ValueError('jobs must be 1..16')
    if not cases or len(set(cases))!=len(cases) or any(c not in DEFAULT_CASES for c in cases):
        raise ValueError('distinct known fidelity cases required')
    if jobs==1:return run_fidelity_sweep(output_dir,tuple(cases),**settings)
    out=Path(output_dir);out.mkdir(parents=True,exist_ok=True)
    report={'schema_version':1,'environment':environment_contract(),
        'source_sha256':source_fingerprint(Path(__file__).resolve().parents[1]),
        'jobs':jobs,'settings':settings,'series':[],'comparisons':[],'case_reports':{},'passed':False}
    with ProcessPoolExecutor(max_workers=jobs,mp_context=get_context('spawn')) as pool:
        futures={pool.submit(run_fidelity_sweep,str(out/case),(case,),**settings):case for case in cases}
        for future in as_completed(futures):
            case=futures[future]
            try:
                result=future.result();report['case_reports'][case]=str(Path(case)/'report.json')
                for item in result['series']:
                    item=dict(item);item['file']=str(Path(case)/item['file']);report['series'].append(item)
                for item in result['comparisons']:
                    item=dict(item)
                    for key in ('coarse','reference'):item[key]=str(Path(case)/item[key])
                    report['comparisons'].append(item)
            except Exception as exc:
                report.setdefault('worker_errors',{})[case]=f'{type(exc).__name__}: {exc}'
            write_report(out/'report.json',report)
    finest=[r for r in report['comparisons'] if r['finest_pair']]
    report['passed']=bool(finest) and not report.get('worker_errors') and all(r['passed'] for r in finest)
    report['failed_criteria']=sorted({k for r in finest for k in r.get('failed_criteria',[])})
    report['axes_passed']={axis:bool([r for r in finest if r['axis']==axis]) and
        all(r['passed'] for r in finest if r['axis']==axis) for axis in ('time','road','stations')}
    report['source_unchanged_during_run']=report['source_sha256']==source_fingerprint(Path(__file__).resolve().parents[1])
    if report['case_reports']:
        report['candidate_resolution']=json.loads((out/next(iter(report['case_reports'].values()))).read_text())['candidate_resolution']
    report['passed']=report['passed'] and report['source_unchanged_during_run']
    write_report(out/'report.json',report);return report


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',default='results/fidelity')
    p.add_argument('--cases',nargs='+',choices=DEFAULT_CASES,default=DEFAULT_CASES)
    p.add_argument('--backend',choices=('compliant_2d','distributed_2d_reference'),default='compliant_2d')
    p.add_argument('--transmission',choices=('ideal_mid_drive','geometric_ideal_mid_drive','elastic_chain'),default='ideal_mid_drive')
    p.add_argument('--stations',action='store_true');p.add_argument('--jobs',type=int,default=1)
    p.add_argument('--time-steps',nargs='+',type=float,default=TIME_STEPS)
    p.add_argument('--road-steps',nargs='+',type=float,default=ROAD_STEPS)
    p.add_argument('--station-counts',nargs='+',type=int,default=STATIONS)
    p.add_argument('--physics-config');a=p.parse_args(argv)
    report=run_fidelity_parallel(a.output,tuple(a.cases),jobs=a.jobs,backend=a.backend,transmission=a.transmission,
        include_stations=a.stations,physics_path=a.physics_config,
        time_steps=tuple(a.time_steps),road_steps=tuple(a.road_steps),station_counts=tuple(a.station_counts))
    print(json.dumps({'passed':report['passed'],'failed_criteria':report['failed_criteria']}))
    return 0 if report['passed'] else 2

if __name__=='__main__':raise SystemExit(main())
