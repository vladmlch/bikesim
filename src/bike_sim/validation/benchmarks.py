"""Versioned executable physics registry and explicit convergence reports."""
from functools import partial
import importlib.metadata
import json
from math import isfinite
from pathlib import Path
import platform
import time
import traceback
import numpy as np
from bike_sim.validation.environment import environment_contract, source_fingerprint
from bike_sim.validation.rigs import (radial_rig,native_radial_rig,wheel_inertia_rig,
    incline_rig,brake_rig,chain_locked_rig,chain_geometry_rig,contact_event_rig)
from bike_sim.validation.rider_cases import airborne_internal_actuation
from bike_sim.validation.ride_cases import (flat_static,brake_hold,drive_low_mu,
    airborne_passive,released_rider,road_run,make_sim,static_metrics,terrain_resolution,requested_sag)


def relative_change(a,b,scale_floor):
    if not all(isfinite(float(x)) for x in (a,b,scale_floor)) or scale_floor<=0:
        raise ValueError('finite metrics and positive comparison scale required')
    return abs(float(a)-float(b))/max(abs(float(b)),float(scale_floor))


def _radial(dt,load):
    r=radial_rig(load,dt)
    r['deflection_error_m']=abs(r['deflection_m']-r['expected_deflection_m'])
    return r,{'deflection_error_m':(0.,max(.0001,.02*r['expected_deflection_m'])),
              'speed_mps':(-1e-5,1e-5)}


def _native_radial(dt):
    return {f'deflection_at_{int(load)}n_m':native_radial_rig(load,dt)['effective_deflection_m']
            for load in (100.,300.,600.,1000.)},{}


def _wheel(dt):
    r=wheel_inertia_rig(dt)
    return r,{'acceleration_relative_error':(0.,.001)}


def _incline(dt):
    r=incline_rig(dt)
    r['normal_relative_error']=relative_change(r['normal_load_n'],r['expected_normal_load_n'],1.)
    r['vertical_relative_error']=relative_change(r['vertical_with_stand_reaction_n'],r['expected_weight_n'],1.)
    return r,{'normal_relative_error':(0.,.005),'vertical_relative_error':(0.,.005)}


def _chain(dt):
    r=chain_locked_rig(dt)
    r['speed_ratio_error']=relative_change(r['speed_ratio'],r['expected_ratio'],1.)
    r['torque_ratio_error']=relative_change(r['torque_ratio'],r['expected_ratio'],1.)
    return r,{'speed_ratio_error':(0.,.001),'torque_ratio_error':(0.,.001),'power_relative_error':(0.,.005)}


def _geometry(dt):
    from bike_sim.validation.rigs import compiled_chain_motion_rig
    r=chain_geometry_rig()
    r.update(compiled_chain_motion_rig())
    return r,{'rigid_motion_extension_m':(-1e-12,1e-12),'directional_relative_error':(0.,1e-7),
              'compiled_jacobian_relative_error':(0.,1e-7),'linkage_force_per_tension_m':(1e-5,np.inf)}


def suspension_cycle(dt):
    from bike_sim.physics.damper import SuperDeluxeDamper
    damper=SuperDeluxeDamper(legacy_behavior=False)
    minimum=0.;work=0.
    for t in np.arange(0.,1.,dt):
        x=.0325+.03*np.sin(2*np.pi*t);v=.03*2*np.pi*np.cos(2*np.pi*t)
        power=damper.compute_damping_force(float(v),float(x*1000.))*v
        minimum=min(minimum,power);work+=power*dt
    damper.lockout_firm=True
    knee_jump=abs(damper.compute_damping_force(.03+1e-8,20.)-damper.compute_damping_force(.03-1e-8,20.))
    hbo=damper.compute_damping_components(.5,60.)['hbo_n']
    return {'minimum_dissipative_power_w':minimum,'cycle_loss_j':work,'firm_knee_jump_n':knee_jump,'firm_hbo_n':hbo}, {
        'minimum_dissipative_power_w':(-1e-9,np.inf),'cycle_loss_j':(0.,np.inf),
        'firm_knee_jump_n':(0.,.001),'firm_hbo_n':(1.,np.inf)}


def _native_static(dt):
    r=static_metrics(make_sim(dt,backend='native_reference'))
    return r,{'force_relative_error':(0.,.005),'moment_relative_error':(0.,.005),'residual_qacc':(0.,.05)}


def _event(dt):
    return contact_event_rig(dt), {'event_error_s':(0.,2.*dt)}


REGISTRY={
    'contact_event':_event,'terrain_resolution':terrain_resolution,'requested_sag':requested_sag,
    'wheel_inertia':_wheel,
    **{f'radial_{int(load)}':partial(_radial,load=load) for load in (100.,300.,600.,1000.)},
    'radial_native_reference':_native_radial,
    'flat_static':flat_static,'flat_static_native':_native_static,'incline_30_deg':_incline,
    'brake_hold':brake_hold,'drive_low_mu':drive_low_mu,
    'airborne_internal_actuation':airborne_internal_actuation,
    'airborne_passive':airborne_passive,'released_rider':released_rider,
    'suspension_cycle':suspension_cycle,'chain_locked_geometry':_chain,
    'chain_suspension_motion':_geometry,
    **{name:partial(road_run,case=name) for name in ('smooth_coast','single_edge','road_worn')},
}

# Review reference stands are independently executable through the same registry.
from bike_sim.validation.contact_manifold_rigs import distributed_flat_rig,distributed_step_rig,distributed_incline_rig
from bike_sim.validation.drive_suspension_rig import drive_suspension_rig
from bike_sim.validation.load_transfer import load_transfer_rig
from bike_sim.validation.system_momentum import momentum_rig
from bike_sim.validation.plant_torque_rig import shaft_ratio_rig
REGISTRY.update({
    'distributed_flat_600':partial(distributed_flat_rig,station_count=256,load_n=600.),
    'distributed_step':partial(distributed_step_rig,station_count=256,terrain_dx_m=.005),
    'distributed_incline':partial(distributed_incline_rig,station_count=256),
    'rigid_load_transfer':partial(load_transfer_rig,slope_rad=.1,com_x_m=.5,com_h_m=.9),
    'whole_system_passive':partial(momentum_rig,active=False),
    'whole_system_active':partial(momentum_rig,active=True),
    'motor_only_airborne':partial(momentum_rig,active=True,rider_active=False),
    'actual_shaft_ratio':shaft_ratio_rig,
    'actual_freehub_overrun':partial(shaft_ratio_rig,overrun=True),
    **{f'drive_suspension_{mode}_{int(torque)}':partial(drive_suspension_rig,mode=mode,imposed_torque_nm=torque)
       for mode in ('ideal_mid_drive','geometric_ideal_mid_drive','elastic_chain') for torque in (0.,20.,40.)},
})

# Peaks at non-smooth contacts are not used as sole convergence evidence.
CONVERGENCE={
    'wheel_inertia':{'omega_rad_s':1.},
    **{f'radial_{int(load)}':{'deflection_m':.0001} for load in (100.,300.,600.,1000.)},
    'flat_static':{'fork_travel_m':.001,'rear_travel_m':.001},
    'drive_low_mu':{'mean_rear_load_n':1.,'traction_impulse_ns':.1},
    'suspension_cycle':{'cycle_loss_j':.1},
    'chain_locked_geometry':{'speed_ratio':1.},
    **{name:{'mean_load_n':1.,'rms_load_n':1.,'normal_impulse_ns':.1,'mean_fork_travel_m':.001,
             **{side+'_'+metric:1. for side in ('front','rear')
                for metric in ('mean_load_n','rms_load_n','normal_impulse_ns')}}
       for name in ('smooth_coast','single_edge','road_worn')},
}


def _run_case(name, dt):
    start = time.monotonic()
    diagnostic = name == "radial_native_reference"
    row = {"case_id": name, "physics_mode": "physical",
           "backend": "native_reference" if "native" in name else "compliant_2d",
           "dt_s": dt, "mandatory": not diagnostic}
    try:
        metrics, bounds = REGISTRY[name](dt)
        metrics = {key: float(value) for key, value in metrics.items()}
        if not all(isfinite(value) for value in metrics.values()):
            raise ArithmeticError("non-finite validation metric")
        criteria = {key: {"minimum": lo if isfinite(lo) else None,
                          "maximum": hi if isfinite(hi) else None,
                          "actual": metrics[key], "passed": bool(lo <= metrics[key] <= hi)}
                    for key, (lo, hi) in bounds.items()}
        passed = all(criterion["passed"] for criterion in criteria.values())
        row.update(metrics=metrics, criteria=criteria, passed=passed,
                   status="diagnostic" if diagnostic else ("passed" if passed else "failed"))
    except Exception as exc:
        row.update(metrics={}, criteria={}, passed=False, status="error",
                   error=f"{type(exc).__name__}: {exc}", traceback=traceback.format_exc())
    row["wall_time_s"] = time.monotonic() - start
    return row


def run_suite(output_dir,dt_values=(.0005,.00025,.000125),*,cases=None,jobs=1):
    if isinstance(jobs,bool) or not isinstance(jobs,int) or not 1 <= jobs <= 32:
        raise ValueError('jobs must be an integer between 1 and 32')
    dts=sorted([float(dt) for dt in dt_values],reverse=True)
    if not dts or len(set(dts))!=len(dts) or any(not isfinite(dt) or dt<=0 for dt in dts):
        raise ValueError('positive, finite, distinct timestep values required')
    chosen=list(REGISTRY) if cases is None else list(cases)
    if not chosen or len(set(chosen))!=len(chosen) or any(name not in REGISTRY for name in chosen):
        raise ValueError('unknown, duplicate or empty validation case selection')
    out=Path(output_dir);out.mkdir(parents=True,exist_ok=True)
    source_root=Path(__file__).resolve().parents[1]
    fingerprint=source_fingerprint(source_root)
    environment=environment_contract()
    report={'schema_version':1,'calibration_status':'parameterized_unvalidated',
            'complete_suite':set(chosen)==set(REGISTRY),'dt_values_s':dts,
            'versions':{'python':platform.python_version(),**{p:importlib.metadata.version(p) for p in ('mujoco','numpy','scipy')}},
            'cases':[],'convergence':[],'passed':False,'jobs':jobs,
            'environment':environment,'source_sha256':fingerprint,
            'required_case_count':len(chosen)*len(dts)}
    def save():
        (out/'report.json').write_text(json.dumps(report,indent=2,sort_keys=True,allow_nan=False)+'\n')
    def accept(row):
        report['cases'].append(row)
        report['cases'].sort(key=lambda item: (chosen.index(item['case_id']), dts.index(item['dt_s'])))
        save()
        print(f"{row['case_id']} dt={row['dt_s']:g}: {row['status']}", flush=True)

    tasks = [(name, dt) for name in chosen for dt in dts]
    if jobs == 1:
        for name, dt in tasks:
            accept(_run_case(name, dt))
    else:
        # Each independent stand owns its own model, MjData, RNG and output.
        # Spawn avoids inheriting graphical or threaded library contexts.
        from concurrent.futures import ProcessPoolExecutor, as_completed
        from multiprocessing import get_context
        with ProcessPoolExecutor(max_workers=jobs, mp_context=get_context('spawn')) as pool:
            futures = {pool.submit(_run_case, name, dt): (name, dt) for name, dt in tasks}
            for future in as_completed(futures):
                name, dt = futures[future]
                try:
                    row = future.result()
                except Exception as exc:
                    row = {'case_id': name, 'physics_mode': 'physical', 'dt_s': dt,
                           'backend': 'native_reference' if 'native' in name else 'compliant_2d',
                           'mandatory': True, 'passed': False, 'status': 'error',
                           'metrics': {}, 'criteria': {}, 'error': str(exc)}
                accept(row)
    if len(dts)>=2:
        coarse,fine=dts[-2:]
        for name,keys in CONVERGENCE.items():
            if name not in chosen:continue
            a=next(row for row in report['cases'] if row['case_id']==name and row['dt_s']==coarse)
            b=next(row for row in report['cases'] if row['case_id']==name and row['dt_s']==fine)
            for key,floor in keys.items():
                change=(relative_change(a['metrics'][key],b['metrics'][key],floor)
                        if key in a['metrics'] and key in b['metrics'] else None)
                report['convergence'].append({'case_id':name,'metric':key,'coarse_dt_s':coarse,'fine_dt_s':fine,
                    'relative_change':change,'scale_floor':floor,'limit':.02,'passed':change is not None and change<=.02})
        if 'contact_event' in chosen:
            events=[row for row in report['cases'] if row['case_id']=='contact_event'][-2:]
            a,b=(row['metrics'].get('contact_time_s') for row in events)
            change=None if a is None or b is None else abs(a-b)
            report['convergence'].append({'case_id':'contact_event','metric':'contact_time_s',
                'absolute_change_s':change,'limit_s':2.*fine,'passed':change is not None and change<=2.*fine+1e-12})
        if 'smooth_coast' in chosen:
            values=[row for row in report['cases'] if row['case_id']=='smooth_coast']
            a,b=values[-2:]
            ea=a['metrics'].get('energy_residual_relative');eb=b['metrics'].get('energy_residual_relative')
            report['convergence'].append({'case_id':'smooth_coast','metric':'energy_residual_refinement',
                'coarse_residual':ea,'fine_residual':eb,'passed':ea is not None and eb is not None and eb<=max(ea,1e-8)})
    report['passed']=all(row['passed'] for row in report['cases'] if row['mandatory']) and all(row['passed'] for row in report['convergence'])
    report['source_unchanged_during_run'] = source_fingerprint(source_root) == fingerprint
    report['mechanics_gate_passed'] = (report['passed'] and report['complete_suite']
        and dts == [.0005,.00025,.000125] and report['source_unchanged_during_run'])
    report['release_gate_passed'] = (report['mechanics_gate_passed']
        and environment['locked_environment_verified'])
    save();return report
