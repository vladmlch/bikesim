"""Full compiled-bike validation scenarios with independently defined criteria."""
from dataclasses import dataclass, replace
from math import pi, tan
import mujoco
import numpy as np
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.physical_config import TireBackendConfig, PhysicalDriveConfig, ResistanceConfig, AssistConfig
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.sim.ride.physical_observations import energy_state
from bike_sim.sim.ride.physical_mapping import point_jacobian
from bike_sim.terrain import TrackSpec, get_preset, HeightFieldSpec
from bike_sim.terrain.obstacles import Obstacle, SquareEdge, Washboard


@dataclass
class Incline(Obstacle):
    extent_m: float = 8.
    angle_deg: float = 10.

    @property
    def length_m(self):return self.extent_m

    def elevation(self,s):return s*tan(self.angle_deg*pi/180.)


def make_sim(dt,*,rider='lumped',speed=0.,drive='coast',human=0.,mu=.8,track=None,field=None,brakes=False,backend='compliant_2d'):
    tires=TireBackendConfig(backend=backend)
    tires=replace(tires,front=replace(tires.front,mu=mu),rear=replace(tires.rear,mu=mu))
    cfg=SimulationPhysicsConfig('physical',drive_mode=drive,timestep_s=dt,tires=tires,
        initial_speed_mps=speed,initial_front_brake=float(brakes),initial_rear_brake=float(brakes),
        drive=PhysicalDriveConfig(human_torque_nm=human,assist=AssistConfig(gain=0.)),
        resistance=ResistanceConfig(crr=0.,cda_m2=0.))
    return RideSimulation(track=track or get_preset('flat'),field=field,rider=rider,physics_config=cfg)


def rebase_initial_state(sim):
    """A rig may impose its declared initial condition before the first step."""
    r=sim.physical;m,d=sim.model,sim.data
    if sim.steps:
        raise ValueError('a benchmark cannot rebase an already running energy ledger')
    mujoco.mj_forward(m,d)
    # Releases performed to construct the initial condition precede this new
    # energy datum. Do not charge their already-excluded energy a second time.
    if r.rider_contacts is not None:
        r.rider_contacts.pending_release_loss_j=0.
    mass,elastic,total=energy_state(r)
    r.initial_energy_j=total
    r.energy_scale_j=max(1.,mass['kinetic_energy_j']+sum(elastic.values()))
    return mass


def static_metrics(sim):
    m,d=sim.model,sim.data
    mass,_,_=energy_state(sim.physical)
    force=mass['mass_kg']*m.opt.gravity.copy();moment=np.zeros(3)
    for snapshot in sim.physical.snapshots.values():
        for p in snapshot.patches:
            force+=p.world_force_n
            moment+=np.cross(p.point_m-mass['com_m'],p.world_force_n)+p.couple_world_nm
    axes=[s.wheel_axis_m for s in sim.physical.snapshots.values()]
    wheelbase=float(np.linalg.norm(axes[0]-axes[1]))
    weight=mass['mass_kg']*float(np.linalg.norm(m.opt.gravity))
    return {'mass_kg':mass['mass_kg'],'mass_error_kg':abs(mass['mass_kg']-sim.mass_specs.total_bike_mass-sim.rider.total_rider_mass),
            'force_relative_error':float(np.linalg.norm(force))/weight,
            'moment_relative_error':float(np.linalg.norm(moment))/(weight*wheelbase),
            'residual_qacc':sim.equilibrium['residual_qacc'],
            'fork_travel_m':sim.fork_travel_mm/1000.,'rear_travel_m':sim.rear_travel_mm/1000.}


def flat_static(dt):
    sim=make_sim(dt)
    return static_metrics(sim), {'mass_error_kg':(0.,1e-8),'force_relative_error':(0.,.005),
        'moment_relative_error':(0.,.005),'residual_qacc':(0.,.05)}


def brake_hold(dt):
    track=TrackSpec('incline_10_deg',8.,[Incline(0.)])
    field=HeightFieldSpec(ncol=1601,radius_x_m=4.,elevation_m=2.,datum_z_m=.1)
    sim=make_sim(dt,track=track,field=field,brakes=True)
    initial=sim.position_m
    for _ in range(round(5./dt)):
        sim.step(1.,1.)
    metrics={'displacement_m':abs(sim.position_m-initial),
             'speed_mps':abs(sim.speed_mps),
             'motor_torque_nm':sim.physical.drive.last['motor_torque_nm'],
             'front_brake_work_j':sim.physical.history.work_j.get('front_static_brake',0.),
             'rear_brake_work_j':sim.physical.history.work_j.get('rear_static_brake',0.)}
    return metrics,{'displacement_m':(0.,.001),'motor_torque_nm':(0.,0.),
                    'front_brake_work_j':(-np.inf,1e-8),'rear_brake_work_j':(-np.inf,1e-8)}


def drive_low_mu(dt):
    sim=make_sim(dt,drive='crank_effort',human=80.,mu=.1)
    max_slip=friction_error=human_peak=0.;mean_load=impulse=0.
    for _ in range(round(.5/dt)):
        sim.step();s=sim.physical.sample.channels['tires']['rear']
        max_slip=max(max_slip,abs(s['slip_mps']))
        friction_error=max(friction_error,abs(s['tangent_force_n'])-.1*s['normal_load_n'])
        human_peak=max(human_peak,sim.physical.drive.last['human_torque_nm'])
        mean_load+=s['normal_load_n']*dt/.5;impulse+=s['tangent_force_n']*dt
    return {'max_slip_mps':max_slip,'friction_excess_n':friction_error,'human_peak_nm':human_peak,
            'mean_rear_load_n':mean_load,'traction_impulse_ns':impulse,'speed_mps':sim.speed_mps}, {
            'max_slip_mps':(.1,np.inf),'friction_excess_n':(0.,1e-6),'human_peak_nm':(50.,np.inf)}


def _airborne_sim(dt):
    sim=make_sim(dt,rider='articulated_planar')
    r=sim.physical;m,d=sim.model,sim.data
    r.rider_contacts.release_all();r.rider_control.enabled=False
    d.qpos[r.address('root_z')[0]]+=30.;d.qpos[r.address('rider_root_z')[0]]+=30.
    d.qvel.fill(0.)
    d.qvel[r.address('front_wheel_spin')[1]]=10.
    d.qvel[r.address('rear_wheel_spin')[1]]=20.
    return sim


def airborne_passive(dt):
    sim=_airborne_sim(dt);initial=rebase_initial_state(sim)
    maximum=0.;scale=max(abs(float(initial['angular_momentum_kg_m2_s'][1])),1.)
    for _ in range(round(2./dt)):
        sim.step()
        mass=sim.physical.sample.channels['endpoint_mass']
        maximum=max(maximum,abs(float(mass['angular_momentum_kg_m2_s'][1])-initial['angular_momentum_kg_m2_s'][1]))
    expected=initial['com_m']+2.*initial['com_velocity_mps']+.5*sim.model.opt.gravity*4.
    ballistic=float(np.linalg.norm(np.asarray(mass['com_m'])-expected))
    return {'relative_Ly_drift':maximum/scale,'Ly_scale_kg_m2_s':scale,'ballistic_position_error_m':ballistic,
            'root_pitch_rad':sim.pitch_rad}, {'relative_Ly_drift':(0.,.001),
            'ballistic_position_error_m':(0.,max(.012,10.*dt))}


def released_rider(dt):
    sim=_airborne_sim(dt);r=sim.physical;m,d=sim.model,sim.data;d.qvel.fill(0.)
    initial=rebase_initial_state(sim)
    # The integration contract uses incoming-state Jacobians. Compare the
    # applied impulse with that same discrete momentum map, then independently
    # measure the rider's actual endpoint momentum for the release criterion.
    incoming_bike_momentum_map=np.zeros(m.nv)
    jp,jr=np.zeros((3,m.nv)),np.zeros((3,m.nv))
    for body in range(m.nbody):
        if not m.body(body).name.startswith('rider_'):
            mujoco.mj_jacBodyCom(m,d,jp,jr,body)
            incoming_bike_momentum_map+=m.body_mass[body]*jp[0]
    force=np.zeros(m.nv);force[sim.root_x_dofadr]=100.
    sim.step(external_qfrc=force)
    rider_p=0.;bike_p=0.;jp,jr=np.zeros((3,m.nv)),np.zeros((3,m.nv))
    for body in range(m.nbody):
        mujoco.mj_jacBodyCom(m,d,jp,jr,body)
        momentum=m.body_mass[body]*float((jp@d.qvel)[0])
        if m.body(body).name.startswith('rider_'):rider_p+=momentum
        else:bike_p+=momentum
    contact_force=sum(float(np.linalg.norm(v.get('force_on_rider_n',[0,0,0]))) for v in r.rider_contacts.diagnostics.values())
    return {'rider_horizontal_momentum_kg_mps':abs(rider_p),'contact_force_n':contact_force,
            'mass_change_kg':abs(float(m.body_mass.sum())-initial['mass_kg']),
            'bike_impulse_error_ns':abs(float(incoming_bike_momentum_map@d.qvel)-100.*dt),
            'endpoint_bike_momentum_error_ns':abs(bike_p-100.*dt),
            'bike_root_speed_mps':sim.speed_mps}, {'rider_horizontal_momentum_kg_mps':(0.,1e-9),
            'contact_force_n':(0.,1e-9),'mass_change_kg':(0.,1e-10),'bike_impulse_error_ns':(0.,1e-5)}


def road_run(dt,case):
    duration=5. if case in ('smooth_coast','road_worn') else 2.
    if case=='single_edge':
        track=TrackSpec('single_edge_rig',20.,[SquareEdge(3.5,height_m=.04,ledge_length_m=.5)])
    elif case=='road_worn':
        track=get_preset('road_worn')
    else:
        track=TrackSpec('smooth_coast_rig',20.,[Washboard(3.,amplitude_m=.002,wavelength_m=2.,n_waves=6)])
    sim=make_sim(dt,speed=3.,track=track)
    start=sim.position_m;max_residual=0.;loads=[];travels=[];impulse=0.;multi=0
    wheel_loads={side:[] for side in ("front","rear")}
    for _ in range(round(duration/dt)):
        sim.step();s=sim.physical.sample
        max_residual=max(max_residual,abs(s.channels['energy']['residual_j'])/sim.physical.energy_scale_j)
        load=sum(s.channels['tires'][side]['normal_load_n'] for side in ('front','rear'))
        for side in wheel_loads:wheel_loads[side].append(s.channels['tires'][side]['normal_load_n'])
        loads.append(load);travels.append(sim.fork_travel_mm/1000.);impulse+=load*dt
        multi+=int(any(s.channels['tires'][side]['multi_support'] for side in ('front','rear')))
    values=np.asarray(loads)
    metrics={'mean_load_n':float(np.mean(values)),'rms_load_n':float(np.sqrt(np.mean(values**2))),
             'normal_impulse_ns':impulse,'mean_fork_travel_m':float(np.mean(travels)),
             'distance_m':sim.position_m-start,'energy_residual_relative':abs(sim.physical.energy['residual_j'])/sim.physical.energy_scale_j,
             'max_energy_residual_relative':max_residual,'multi_support_duration_s':multi*dt,
             'front_airtime_s':sim.physical.history.airtime_s['front']['0.0'],
             'rear_airtime_s':sim.physical.history.airtime_s['rear']['0.0']}
    for side, data in wheel_loads.items():
        data=np.asarray(data)
        metrics[side+'_mean_load_n']=float(np.mean(data))
        metrics[side+'_rms_load_n']=float(np.sqrt(np.mean(data**2)))
        metrics[side+'_normal_impulse_ns']=float(np.sum(data)*dt)
    vertices=sim.physical.vertices
    visited=vertices[(vertices[:,0]>=start)&(vertices[:,0]<=sim.position_m)]
    metrics['visited_profile_span_m']=float(np.ptp(visited[:,1])) if len(visited) else 0.
    criteria={'distance_m':(0.,np.inf),'mean_load_n':(0.,np.inf)}
    if case=='road_worn':criteria['visited_profile_span_m']=(1e-5,np.inf)
    if case=='smooth_coast':criteria['energy_residual_relative']=(0.,.01)
    return metrics,criteria


def terrain_resolution(dt):
    """Double raster resolution without changing geometry or initial conditions."""
    track=TrackSpec('spatial_refinement_rig',20.,[Washboard(3.,amplitude_m=.002,wavelength_m=2.,n_waves=6)])
    results=[]
    for ncol in (1001,2001):
        field=HeightFieldSpec(ncol=ncol,radius_x_m=10.)
        sim=make_sim(dt,speed=3.,track=track,field=field)
        loads=[];travels=[]
        for _ in range(round(2./dt)):
            sim.step()
            loads.append(sum(s.normal_load_n for s in sim.physical.snapshots.values()))
            travels.append(sim.fork_travel_mm/1000.)
        values=np.asarray(loads)
        results.append({'mean_load_n':float(np.mean(values)),
                        'rms_load_n':float(np.sqrt(np.mean(values**2))),
                        'normal_impulse_ns':float(np.sum(values)*dt),
                        'mean_fork_travel_m':float(np.mean(travels))})
    metrics={'coarse_dx_m':.02,'fine_dx_m':.01}
    for key,value in results[1].items():
        metrics[key+'_relative_change']=abs(results[0][key]-value)/max(abs(value),1e-4)
        metrics[key+'_fine']=value
    return metrics,{key:(0.,.02) for key in metrics if key.endswith('_relative_change')}


def requested_sag(dt):
    """Fit spring parameters against achieved, compiled static wheel travel."""
    from bike_sim.geometry.specs import BikeSpecs
    from bike_sim.physics.rider import RiderSpecs
    from bike_sim.sim.ride.sag_fit import build_equilibrium_evaluator,fit_sag
    specs=BikeSpecs();rider=RiderSpecs(variant='lumped')
    cfg=SimulationPhysicsConfig('physical',timestep_s=dt,tires=TireBackendConfig(backend='compliant_2d'))
    target=(specs.fork_travel*.3,specs.rear_wheel_travel*.3)
    evaluator=build_equilibrium_evaluator(specs=specs,track=get_preset('flat'),rider=rider,physics_config=cfg)
    psi,rate=fit_sag(evaluator,target,(specs.fork_initial_psi,specs.shock_stiffness),((10.,1000.),(400.,250000.)))
    sim=RideSimulation(specs=replace(specs,fork_initial_psi=float(psi),shock_stiffness=float(rate)),
        track=get_preset('flat'),rider=rider,physics_config=cfg)
    metrics=static_metrics(sim)
    metrics.update(fork_error_mm=abs(sim.fork_travel_mm-target[0]),rear_error_mm=abs(sim.rear_travel_mm-target[1]),
                   fork_pressure_psi=float(psi),coil_rate_n_m=float(rate))
    return metrics,{'fork_error_mm':(0.,.5),'rear_error_mm':(0.,.5),
                    'force_relative_error':(0.,.005),'moment_relative_error':(0.,.005)}
