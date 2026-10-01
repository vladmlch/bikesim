"""Traceable drive/coast/resume and open-loop torque experiments.

Uses the ordinary physical runtime. Commands depend on time only; no wheelie
label, pitch signal or front load is fed back into motor or rider demand.
"""
from dataclasses import replace
from math import isfinite,pi
from pathlib import Path
import argparse
import gzip
import json
import numpy as np
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.physical_samples import plain


def _time(value):
    if not isfinite(value) or value<0:
        raise ValueError('schedule time must be finite and nonnegative')
    return float(value)


def resume_schedule(time_s: float) -> RideControl:
    t=_time(time_s)
    return RideControl(human_torque_nm=20. if t<2. or t>=4. else 0.,motor_torque_nm=0.)


def open_loop_schedule(time_s: float) -> RideControl:
    t=_time(time_s)
    torque=0. if t<1. or t>=4. else 40.*(t-1.) if t<2. else 40.
    return RideControl(motor_torque_nm=torque,human_torque_nm=0.)


def automatic_schedule(time_s: float) -> RideControl:
    _time(time_s)
    return RideControl()


def coasting_evidence(rows: list[dict]) -> dict:
    coast_entered = False
    resumed = False
    loss_duration = 0.
    maximum_loss = 0.
    maximum_gap = 0.
    maximum_stopped_gap = 0.
    stopped_duration = 0.
    rotation = 0.
    positive_work = 0.
    first_loss = None
    for row in rows:
        interval = row['end_time_s'] - row['time_s']
        if interval <= 0. or not isfinite(interval):
            raise ValueError('invalid coasting diagnostic interval')
        drive = row['drive']
        coasting = drive['rider_mode'] == 'coasting'
        coast_entered = coast_entered or coasting
        pedals = [row['rider'][side + '_pedal'] for side in ('front', 'rear')]
        available = any(pedal['in_platform'] and pedal['normal_load_n'] > 1.
                        for pedal in pedals)
        if coasting:
            maximum_gap = max(maximum_gap, *(pedal['gap_m'] for pedal in pedals))
            if row['support']['crank_tracking']['target_rate_rad_s'] == 0.:
                maximum_stopped_gap = max(maximum_stopped_gap, *(pedal['gap_m'] for pedal in pedals))
                stopped_duration += interval
        loss_duration = loss_duration + interval if coasting and not available else 0.
        maximum_loss = max(maximum_loss, loss_duration)
        if loss_duration >= .20 and first_loss is None:
            first_loss = row['end_time_s'] - loss_duration
        if coast_entered and drive['rider_mode'] == 'pedaling':
            resumed = True
            rate = drive['crank_rad_s']
            rotation += rate * interval
            positive_work += max(0., drive['human_sensor_nm'] * rate) * interval
    return {
        'coast_entered': coast_entered, 'pedaling_resumed': resumed,
        'max_both_unloaded_coast_s': maximum_loss,
        'max_coasting_gap_m': maximum_gap,
        'max_stopped_coasting_gap_m': maximum_stopped_gap,
        'stopped_coast_duration_s': stopped_duration,
        'resume_crank_turns': rotation / (2. * pi),
        'resume_positive_work_j': positive_work,
        'first_support_loss': first_loss,
    }


def run_automatic_coast_case(*, dt_s: float, duration_s: float = 5.) -> dict:
    from bike_sim.sim.research.quality import energy_quality
    from bike_sim.sim.ride.physical_session import configuration_metadata
    root = Path(__file__).resolve().parents[3]
    sim = build_sim(root / 'examples/research/viewer_physics_fast.toml',
                    root / 'examples/research/rough_uphill_extreme.toml', timestep_s=dt_s)
    metadata = configuration_metadata(sim)
    rows = replay(sim, automatic_schedule, duration_s)
    return {
        'metadata': metadata, 'rows': rows, 'duration_s': sim.time_s,
        **coasting_evidence(rows),
        'model_valid': sim.physical.model_status.as_dict()['model_valid'],
        'numerically_valid': bool(rows) and all(
            energy_quality(row['energy']).acceptable for row in rows),
    }


def build_sim(physics_path: str, track_path: str, timestep_s: float | None=None,
              *, physics_overrides=None):
    from bike_sim.cli.ride import parse_args,resolve_track,resolve_rider
    from bike_sim.sim.ride.physical_session import build_physical_simulation
    from bike_sim.physics.resolution import resolve_physics_config
    from dataclasses import asdict
    argv=['--physics-config',str(physics_path),'--rider','articulated_planar','--track',str(track_path)]
    if timestep_s is not None:
        argv+=['--timestep',str(timestep_s)]
    args=parse_args(argv)
    if physics_overrides:
        args.resolved_physics=resolve_physics_config(asdict(args.resolved_physics),physics_overrides)
    return build_physical_simulation(resolve_track(args.track),args,resolve_rider(args))


def diagnostic_row(sample) -> dict:
    c=sample.channels
    row={'interval_id':sample.interval_id,'time_s':sample.time_s,'end_time_s':sample.end_time_s,
         'drive':plain(c['drive']),'rider':plain(c['rider']),
         'support':plain(c['rider_support_targets']),'joint_terms':plain(c['rider_control']),
         'ik_saturation':plain(c['rider_ik_saturation']),'tires':plain(c['tires']),
         'energy':plain(c['energy']),'control':plain(c.get('control',{})),
         'model_status':plain(c.get('model_status',{})),
         'rider_intent':plain(c.get('rider_intent',{})),
         'suspension':plain(c.get('suspension',{})), 'mass':plain(c.get('mass',{}))}
    row.update({k:plain(v) for k,v in c.items() if k.startswith('rider_active') or k in
                ('rider_positive_power_w','rider_activation_saturated','rider_passive_power_w',
                 'rider_effort_budget_exceeded')})
    return row


def replay(sim, schedule, duration_s: float, *, row_sink=None) -> list[dict]:
    dt=float(sim.model.opt.timestep)
    if not isfinite(duration_s) or duration_s<=0:
        raise ValueError('duration must be finite and positive')
    count=round(duration_s/dt)
    if count<1 or abs(count*dt-duration_s)>1e-9:
        raise ValueError('duration must be a positive integer number of steps')
    if sim.physical.interactive_preview:
        raise ValueError('diagnostic replay requires immutable research samples')
    rows=[]
    for _ in range(count):
        sim.step(control=schedule(sim.time_s))
        row = diagnostic_row(sim.physical.sample)
        rows.append(row)
        if row_sink is not None:
            row_sink(row)
        if sim.crash is not None:
            break
    return rows


def resume_evidence(rows: list[dict], *, end_s=8.) -> dict:
    if not isfinite(end_s) or end_s <= 4.:
        raise ValueError('resume observation must end after restart')
    window=[r for r in rows if r['end_time_s']>4. and r['time_s']<end_s]
    result={'window_s':[4.,end_s],'interval_count':len(window),'positive_crank_work_j':0.,
            'crank_rotation_rad':0.,'diagnosis':'mixed_or_unresolved'}
    if not window:
        return result
    durations=np.array([min(end_s,r['end_time_s'])-max(4.,r['time_s']) for r in window])
    if np.any(durations<=0) or not np.isfinite(durations).all():
        raise ValueError('invalid diagnostic intervals')
    duration=float(durations.sum())
    drive=[r['drive'] for r in window]
    speed=np.array([float(d.get('crank_rad_s',d.get('cadence_rpm',0.)*2*pi/60.)) for d in drive])
    torque=np.array([float(d.get('human_sensor_nm',0.)) for d in drive])
    request=np.array([float(r.get('control',{}).get('human_torque_nm') or d.get('human_command_nm',0.))
                      for r,d in zip(window,drive)])
    phases=np.unwrap([float(d['crank_phase_rad']) for d in drive])
    rotation=float(phases[-1]-phases[0]+speed[-1]*durations[-1])
    positive=float(np.sum(np.maximum(torque*speed,0.)*durations))
    unavailable=[];saturated=[];gated=[]
    for row,demand,tau in zip(window,request,torque):
        rider=row.get('rider',{})
        available=any(rider.get(side+'_pedal',{}).get('in_platform',False)
                      and rider.get(side+'_pedal',{}).get('normal_load_n',0.)>1.
                      for side in ('front','rear'))
        unavailable.append(demand>0. and not available)
        saturated.append(any(t.get('saturated',False) for t in row.get('joint_terms',{}).values()))
        d=row['drive']
        gated.append(tau>0. and d.get('motor_control_source')=='assist' and
                     bool(d.get('assist_stalled',False) or d.get('assist_demand_gated',False)))
    fractions={name:float(np.dot(durations,values)/duration) for name,values in
               [('support_unavailable_fraction',unavailable),('actuator_saturated_fraction',saturated),
                ('assist_gated_fraction',gated)]}
    result.update(fractions,window_duration_s=duration,positive_crank_work_j=positive,
                  crank_rotation_rad=rotation,requested_effort_peak_nm=float(np.max(request)),
                  delivered_human_mean_nm=float(np.dot(torque,durations)/duration),
                  mean_abs_crank_speed_rad_s=float(np.dot(np.abs(speed),durations)/duration))
    if duration<end_s-4.-.01:
        result['incomplete_window']=True
    elif positive>0. and rotation>=2*pi:
        result['diagnosis']='resumed'
    elif np.max(request)<=0.:
        result['diagnosis']='no_effort_request'
    else:
        candidates=[]
        if fractions['support_unavailable_fraction']>.5:candidates.append('support_unavailable')
        if fractions['actuator_saturated_fraction']>.5:candidates.append('actuator_saturated')
        if fractions['assist_gated_fraction']>.5:candidates.append('assist_gated')
        if not candidates and np.max(request)>0. and result['mean_abs_crank_speed_rad_s']<.2 and np.max(torque)>0.:
            candidates.append('mechanically_stalled')
        result['candidate_causes']=candidates
        if len(candidates)==1:result['diagnosis']=candidates[0]
    return result


def diagnose_resume(rows: list[dict]) -> str:
    return resume_evidence(rows)['diagnosis']


def write_report(path, report):
    file=Path(path);file.parent.mkdir(parents=True,exist_ok=True)
    opener=gzip.open if file.suffix=='.gz' else open
    with opener(file,'wt',encoding='utf-8') as stream:
        json.dump(plain(report),stream,sort_keys=True,allow_nan=False)
        stream.write('\n')


def run_replay(physics_path,track_path,*,duration_s=8.,mode='human-only',timestep_s=None,
               activation_tau_s=None,output=None):
    from bike_sim.sim.ride.physical_session import configuration_metadata
    from bike_sim.validation.environment import environment_contract,source_fingerprint
    if mode not in ('human-only','motor40','assist','shifting','rollback','open-loop','automatic'):
        raise ValueError('unknown isolated replay factor')
    overrides={}
    if mode=='shifting':overrides={'drive':{'shifting':{'enabled':True}}}
    elif mode=='rollback':overrides={'drive':{'pedaling':{'rollback_brake':True}}}
    if activation_tau_s is not None:
        overrides['articulated']={'activation_tau_s':activation_tau_s,'active_positive_power_limit_w':600.}
    def schedule(t):
        if mode=='automatic':return automatic_schedule(t)
        if mode=='open-loop':return open_loop_schedule(t)
        base=resume_schedule(t)
        return replace(base,motor_torque_nm=40. if mode=='motor40' else None if mode=='assist' else 0.)
    report={'schema_version':1,'mode':mode,'physics_path':str(physics_path),'track_path':str(track_path),
            'duration_requested_s':duration_s,'source_sha256':source_fingerprint(Path(__file__).resolve().parents[1]),
            'environment':environment_contract(),'calibration_status':'parameterized_unvalidated'}
    from bike_sim.validation.climbing_evidence import support_evidence
    rows=[]
    sim=None
    try:
        sim=build_sim(physics_path,track_path,timestep_s,physics_overrides=overrides)
        report.update(configuration_metadata(sim),equilibrium=sim.equilibrium)
        replay(sim,schedule,duration_s,row_sink=rows.append)
        report.update(rows=rows,evidence=coasting_evidence(rows) if mode=='automatic' else resume_evidence(rows,end_s=max(8.,duration_s)),model_status=sim.physical.model_status.as_dict(),
            end_time_s=sim.time_s,end_position_m=sim.position_m,crash=None if sim.crash is None else str(sim.crash),
            completed_requested_duration=abs(sim.time_s-duration_s)<1e-8)
    except (ValueError,RuntimeError,ArithmeticError) as exc:
        report.update(error=f'{type(exc).__name__}: {exc}',diagnosis='mixed_or_unresolved',
                      completed_requested_duration=False)
    finally:
        report['rows']=rows
        try:
            report['support_evidence']=support_evidence(rows,required_duration_s=duration_s)
        except (ValueError,KeyError,TypeError,ArithmeticError) as error:
            report['support_evidence']={
                'interval_count':len(rows),'complete':False,'valid_for_learning':False,
                'evaluation_error':f'{type(error).__name__}: {error}'}
            report['completed_requested_duration']=False
        if sim is not None:
            report.update(end_time_s=sim.time_s,end_position_m=sim.position_m,
                          model_status=sim.physical.model_status.as_dict())
        if output is not None:write_report(output,report)
    return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--physics-config',default='examples/research/rider_resume_physics.toml')
    parser.add_argument('--track',default='examples/research/rider_resume_flat.toml')
    parser.add_argument('--duration',type=float,default=8.)
    parser.add_argument('--timestep',type=float)
    parser.add_argument('--mode',choices=('human-only','motor40','assist','shifting','rollback','open-loop','automatic'),default='human-only')
    parser.add_argument('--activation-tau',type=float)
    parser.add_argument('--output',required=True)
    args=parser.parse_args(argv)
    report=run_replay(args.physics_config,args.track,duration_s=args.duration,mode=args.mode,
        timestep_s=args.timestep,activation_tau_s=args.activation_tau,output=args.output)
    print(json.dumps({k:v for k,v in report.items() if k in ('error','evidence','end_time_s','end_position_m','model_status')},sort_keys=True))
    return 1 if 'error' in report else 0


if __name__=='__main__':
    raise SystemExit(main())
