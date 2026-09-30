"""Localized drivetrain/linkage experiment with declared external axle supports.

The fork and closed Horst linkage are free after initialization. Identical
constant vertical frame load, compliant roller supports and delivered mechanical
crank torque are used for all reductions. This is not a riding controller and
not a substitute for the motor/battery actuator contract test.
"""
from dataclasses import asdict
from itertools import product
from pathlib import Path
import argparse
import numpy as np
import mujoco
from scipy.optimize import least_squares

MODES=('ideal_mid_drive','geometric_ideal_mid_drive','elastic_chain')
STROKES=(.005,.015,.030)


def _fixture(dt_s,mode,stroke_m):
    from bike_sim.geometry.specs import BikeSpecs
    from bike_sim.physics.model_config import SimulationPhysicsConfig
    from bike_sim.physics.physical_config import PhysicalDriveConfig
    from bike_sim.physics.chain import DrivetrainSpecs
    from bike_sim.mujoco.builder import generate_mujoco_xml
    from bike_sim.sim.ride.forces import SuspensionForceApplier
    from bike_sim.sim.ride.drivetrain_forces import DrivetrainForceApplier
    from bike_sim.physics.suspension_config import build_suspension_components
    if mode not in MODES or not np.isfinite(stroke_m) or not 0<=stroke_m<=.04:
        raise ValueError('unknown transmission or suspension fixture position')
    specs=BikeSpecs()
    cfg=SimulationPhysicsConfig('physical',drive_mode='crank_effort',timestep_s=dt_s,
        closure_time_constant_s=.0025,drive=PhysicalDriveConfig(
            transmission_model=mode,human_torque_nm=0.,gearing=DrivetrainSpecs(34,51)))
    m=mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode='ride',rider='none',specs=specs,physics_config=cfg))
    # The external mechanical fixture replaces terrain, never adds a second tire.
    m.geom_contype[:]=0;m.geom_conaffinity[:]=0
    d=mujoco.MjData(m)
    indices=[int(m.joint(n).qposadr[0]) for n in
        ('main_pivot','horst_pivot','rocker_frame_pivot','yoke_pivot')]
    pairs=[(m.site(a).id,m.site(b).id) for a,b in
        (('site_P3_ss','site_P3_rocker'),('site_P7_shock','site_P7'))]
    d.qpos[m.joint('fork_travel').qposadr[0]]=.03
    def residual(q):
        d.qpos[indices]=q;mujoco.mj_kinematics(m,d)
        return np.concatenate([(d.site_xpos[a]-d.site_xpos[b])[[0,2]] for a,b in pairs])
    # Continuation selects the same linkage branch at all three positions.
    for stroke in np.linspace(0.,stroke_m,7):
        d.qpos[m.joint('shock_stroke').qposadr[0]]=stroke
        solution=least_squares(residual,d.qpos[indices],bounds=(-1.5,1.5),
                               ftol=1e-13,xtol=1e-13,gtol=1e-13,max_nfev=120)
        residual(solution.x)
    closure=float(np.max(np.abs(residual(d.qpos[indices]))))
    if closure>1e-7:raise ArithmeticError('initial closed linkage did not converge')
    mujoco.mj_forward(m,d)
    suspension=SuspensionForceApplier(m,*build_suspension_components(specs),physics_config=cfg)
    drive=DrivetrainForceApplier(m,cfg.drive,cfg.drive_mode);drive.reset(m,d)
    return m,d,drive,suspension,closure


def drive_suspension_rig(dt_s: float,mode: str,imposed_torque_nm: float,*,
                         initial_shock_stroke_m: float=.015,duration_s: float=.05):
    if not np.isfinite(imposed_torque_nm) or not 0<=imposed_torque_nm<=40:
        raise ValueError('fixture torque range is 0..40 Nm')
    m,d,drive,suspension,closure=_fixture(dt_s,mode,initial_shock_stroke_m)
    count=round(duration_s/dt_s)
    if count<1 or abs(count*dt_s-duration_s)>1e-10:raise ValueError('whole fixture intervals required')
    frame=m.body('frame').id;rear=m.body('rear_wheel').id
    axle_ids={s:m.body(s+'_wheel').id for s in ('front','rear')}
    radii={s:float(m.geom('geom_'+s+'_contact').size[0]) for s in axle_ids}
    initial_bottoms={s:float(d.xpos[b,2])-radii[s] for s,b in axle_ids.items()}
    # A vertical frame dead-load, not fictitious rider inertia or an actuator.
    external_frame_load_n=80.*9.81
    total_weight=float(m.body_mass.sum()*9.81+external_frame_load_n)
    preload={'front':total_weight*.41,'rear':total_weight*.59}
    k,c,viscous=130000.,800.,2000.
    initial_rear_z=float((d.xpos[rear]-d.xpos[frame])@d.xmat[frame].reshape(3,3)[:,2])
    q0=d.qpos.copy();rows=[];j=np.zeros((3,m.nv));r=np.zeros_like(j)
    max_defect=power_error=root_actuation=native=0.
    for i in range(count):
        mujoco.mj_forward(m,d)
        components=drive.compute_components(m,d,dt_s,speed_mps=0.,active=False)
        force=sum(components.values(),np.zeros(m.nv))+suspension.compute_qfrc(m,d)
        # Constant *delivered* shaft load isolates mechanics from electric lag.
        aid=drive.actuators['mid_drive'];d.ctrl[aid]=imposed_torque_nm
        wheel_forces={}
        for side,body in axle_ids.items():
            point=d.xpos[body].copy();point[2]-=radii[side]
            mujoco.mj_jac(m,d,j,r,point,body);velocity=j@d.qvel
            normal=max(0.,preload[side]-k*(point[2]-initial_bottoms[side])-c*velocity[2])
            tangent=float(np.clip(-viscous*velocity[0],-.8*normal,.8*normal))
            applied=np.array([tangent,0.,normal]);force+=j.T@applied;wheel_forces[side]=normal
        point=d.xipos[frame];mujoco.mj_jac(m,d,j,r,point,frame)
        force+=j.T@np.array([0.,0.,-external_frame_load_n])
        d.qfrc_applied[:]=force;omega=float(d.qvel[m.joint('crank_spin').dofadr[0]])
        rear_z=float((d.xpos[rear]-d.xpos[frame])@d.xmat[frame].reshape(3,3)[:,2])
        shock=suspension.shock_total_n
        mujoco.mj_step(m,d);drive.settle_actuation(m,d)
        actual=float(d.actuator_force[aid]);power=actual*omega
        power_error=max(power_error,abs(actual-imposed_torque_nm))
        if drive.ideal_hub is not None:
            max_defect=max(max_defect,abs(drive.last.get('transmission_constraint_defect_m',0.)))
        for name in ('root_x','root_z','root_pitch'):
            root_actuation=max(root_actuation,abs(float(d.qfrc_actuator[m.joint(name).dofadr[0]])))
        native=max(native,d.ncon)
        rows.append({'time_s':i*dt_s,'end_time_s':(i+1)*dt_s,'shock_force_n':shock,
            'front_normal_load_n':wheel_forces['front'],'rear_travel_delta_m':rear_z-initial_rear_z,
            'shaft_power_w':power,'delivered_torque_nm':actual})
        if not np.isfinite(d.qpos).all():raise ArithmeticError('nonfinite fixture state')
    # Compare the same declared window, including the physical slack transient.
    metrics={'mean_shock_force_n':float(np.mean([v['shock_force_n'] for v in rows])),
        'mean_front_normal_load_n':float(np.mean([v['front_normal_load_n'] for v in rows])),
        'mean_rear_travel_delta_m':float(np.mean([v['rear_travel_delta_m'] for v in rows])),
        'final_rear_travel_delta_m':rows[-1]['rear_travel_delta_m'],
        'mean_shaft_power_w':float(np.mean([v['shaft_power_w'] for v in rows])),
        'shaft_work_j':sum(v['shaft_power_w']*dt_s for v in rows),
        'mean_delivered_torque_nm':float(np.mean([v['delivered_torque_nm'] for v in rows])),
        'delivered_torque_error_nm':power_error,'constraint_defect_m':max_defect,
        'initial_closure_error_m':closure,'initial_shock_stroke_m':initial_shock_stroke_m,
        'duration_s':duration_s,'external_frame_load_n':external_frame_load_n,
        'compiled_mass_kg':float(m.body_mass.sum()),'root_actuator_force':root_actuation,
        'native_contact_count':native,'frame_pitch_initial_rad':float(q0[m.joint('root_pitch').qposadr[0]])}
    return metrics,{'delivered_torque_error_nm':(0.,1e-10),'initial_closure_error_m':(0.,1e-7),
                    'root_actuator_force':(0.,0.),'native_contact_count':(0.,0.)}


def run_comparison(output,dt_values=(.0003125,.00015625)):
    from bike_sim.validation.rider_replay import write_report
    from bike_sim.validation.environment import environment_contract,source_fingerprint
    if not dt_values or any(dt<=0 or not np.isfinite(dt) for dt in dt_values):raise ValueError('positive steps required')
    report={'schema_version':1,'environment':environment_contract(),
        'source_sha256':source_fingerprint(Path(__file__).resolve().parents[1]),
        'fixture':'free fork/linkage, vertical 784.8 N frame dead-load, compliant viscous rollers; initial rest, zero stored chain/freehub energy',
        'force_comparison_gate':.05,'force_floor_n':10.,'series':[],'comparisons':[],'passed':False}
    lookup={}
    for dt,stroke,torque,mode in product(dt_values,STROKES,(0.,20.,40.),MODES):
        item={'dt_s':dt,'mode':mode,'imposed_torque_nm':torque,'initial_shock_stroke_m':stroke}
        try:
            metrics,bounds=drive_suspension_rig(dt,mode,torque,initial_shock_stroke_m=stroke)
            item.update(metrics=metrics,passed=all(lo<=metrics[k]<=hi for k,(lo,hi) in bounds.items()))
        except Exception as exc:item.update(passed=False,error=f'{type(exc).__name__}: {exc}',metrics=None)
        lookup[(dt,stroke,torque,mode)]=item;report['series'].append(item)
        write_report(output,report)
    dt=min(dt_values)
    for stroke,torque,mode in product(STROKES,(0.,20.,40.),MODES[:-1]):
        a,b=lookup[(dt,stroke,torque,mode)],lookup[(dt,stroke,torque,'elastic_chain')]
        comparison={'dt_s':dt,'mode':mode,'initial_shock_stroke_m':stroke,'torque_nm':torque}
        if a['metrics'] is not None and b['metrics'] is not None:
            errors={k:abs(a['metrics'][k]-b['metrics'][k])/max(abs(b['metrics'][k]),10.)
                for k in ('mean_shock_force_n','mean_front_normal_load_n')}
            comparison.update(relative_force_errors=errors,passed=a['passed'] and b['passed'] and max(errors.values())<=.05,
                rear_travel_difference_m=a['metrics']['mean_rear_travel_delta_m']-b['metrics']['mean_rear_travel_delta_m'],
                shaft_power_difference_w=a['metrics']['mean_shaft_power_w']-b['metrics']['mean_shaft_power_w'])
        else:comparison.update(passed=False,reason='missing fixture result')
        report['comparisons'].append(comparison)
    proposed=[c for c in report['comparisons'] if c['mode']=='geometric_ideal_mid_drive' and c['torque_nm']>0]
    report['passed']=bool(proposed) and all(c['passed'] for c in proposed) and all(i['passed'] for i in report['series'])
    report['status']='synthetic_local_fixture_pass' if report['passed'] else 'experimental_gate_failed'
    write_report(output,report);return report

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',default='results/drive_suspension.json')
    a=p.parse_args();report=run_comparison(a.output);print(report['status'])
