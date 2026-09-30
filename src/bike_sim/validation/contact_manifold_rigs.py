"""Independent distributed-contact stands, separate from bicycle baselines.

All road cases are rasterized once. The engine heightfield and distance query
share the very same array. A stand's constant vertical axle load is explicit;
there is no stabilizing pitch actuator or hidden running-state correction.
"""
from dataclasses import replace
import numpy as np
import mujoco
from scipy.optimize import brentq
from bike_sim.physics.physical_config import TireBackendConfig
from bike_sim.physics.distributed_tire import DistributedTireConfig, HingeDensity, flat_load
from bike_sim.sim.ride.distributed_tire_forces import DistributedTireForceApplier
from bike_sim.sim.ride.tire_forces import compiled_profile_vertices


def wheel_stand(dt_s=.000625, station_count=128, *, vertices=None, density=None, mu=.8):
    if not np.isfinite(dt_s) or dt_s<=0:
        raise ValueError('stand step must be positive')
    if vertices is None:
        vertices=np.c_[np.linspace(-2,2,801),np.zeros(801)]
    v=np.asarray(vertices,float)
    if not np.allclose(np.diff(v[:,0]),np.diff(v[:,0])[0],rtol=1e-9,atol=1e-12):
        raise ValueError('stand heightfield must be uniformly rasterized')
    minimum=float(v[:,1].min()); maximum=float(v[:,1].max()); scale=max(.01,maximum-minimum)
    center=float((v[0,0]+v[-1,0])/2);radius=float((v[-1,0]-v[0,0])/2)
    xml=f'''<mujoco><compiler angle="radian"/><option timestep="{dt_s}" gravity="0 0 0" integrator="Euler"/>
      <asset><hfield name="road" nrow="2" ncol="{len(v)}" size="{radius} 1 {scale} .2"/></asset>
      <worldbody><geom name="terrain" type="hfield" hfield="road" pos="{center} 0 {minimum}"/>
        <body name="front_wheel" pos="0 0 .37"><joint name="root_x" type="slide" axis="1 0 0"/>
          <joint name="root_z" type="slide" axis="0 0 1"/><joint name="front_wheel_spin" type="hinge" axis="0 1 0"/>
          <inertial mass="2.4" pos="0 0 0" diaginertia=".15 .30 .15"/>
          <geom name="geom_front_contact" type="cylinder" size=".37 .025" euler="1.5707963267948966 0 0" contype="0" conaffinity="0" mass="0"/>
        </body><body name="rear_wheel" pos="0 0 2"><geom name="geom_rear_contact" type="sphere" size=".37" contype="0" conaffinity="0"/></body>
      </worldbody></mujoco>'''
    m=mujoco.MjModel.from_xml_string(xml)
    m.hfield_data[:]=np.tile((v[:,1]-minimum)/scale,2)
    d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    config=TireBackendConfig(backend='distributed_2d_reference',distributed=DistributedTireConfig(
        station_count=station_count,density=density or HingeDensity()))
    config=replace(config,front=replace(config.front,mu=mu),rear=replace(config.rear,mu=mu))
    compiled=compiled_profile_vertices(m,d)
    tire=DistributedTireForceApplier(m,compiled,config)
    return m,d,tire


def _kinetic(m,d):
    momentum=np.empty(m.nv);mujoco.mj_mulM(m,d,momentum,d.qvel)
    return .5*float(d.qvel@momentum)


def _run_stand(m,d,tire,load_n,duration_s):
    dt=float(m.opt.timestep);count=round(duration_s/dt)
    if abs(count*dt-duration_s)>1e-10 or count<1:
        raise ValueError('stand duration must have an integer number of intervals')
    initial=_kinetic(m,d)+tire.stored_energy(m,d)+load_n*float(d.qpos[1])
    scale=max(1.,_kinetic(m,d)+tire.stored_energy(m,d))
    loss=0.;rows=[];max_residual=0.;native=0
    for i in range(count):
        force=tire.compute_qfrc(m,d,dt)
        snap=tire.snapshots['front']
        rows.append({'time_s':i*dt,'end_time_s':(i+1)*dt,'normal_load_n':snap.normal_load_n,
                     'vertical_force_n':snap.vertical_force_n,'position_m':float(d.qpos[0]),
                     'pitch_rate_rad_s':float(d.qvel[2]),'patch_count':len(snap.patches),
                     'multi_support':tire.diagnostics['front']['multi_support']})
        loss+=tire.brush_loss_step_j+tire.radial_dissipation_power_w*dt
        d.qfrc_applied[:]=force;d.qfrc_applied[1]-=load_n
        mujoco.mj_step(m,d);mujoco.mj_forward(m,d)
        native=max(native,d.ncon)
        current=_kinetic(m,d)+tire.stored_energy(m,d)+load_n*float(d.qpos[1])
        residual=current-initial+loss
        max_residual=max(max_residual,abs(residual))
        if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():
            raise ArithmeticError('non-finite stand trajectory')
    return rows,{'energy_residual_j':residual,'energy_residual_ratio':abs(residual)/(scale+loss),
                 'max_energy_residual_j':max_residual,'dissipation_j':loss,
                 'native_wheel_contact_count':native,'final_speed_mps':float(d.qvel[0])}


def distributed_flat_rig(dt_s: float, station_count: int, load_n: float):
    if not np.isfinite(load_n) or load_n<=0:
        raise ValueError('axle load must be finite and positive')
    m,d,tire=wheel_stand(dt_s,station_count)
    depth=brentq(lambda x:float(flat_load(x,.37,tire.material,station_count)[0])-load_n,1e-9,.1,xtol=1e-14)
    d.qpos[1]=-depth;mujoco.mj_forward(m,d)
    rows,metrics=_run_stand(m,d,tire,load_n,.05)
    metrics.update(normal_force_n=float(np.mean([r['normal_load_n'] for r in rows])),deflection_m=depth,
                   force_relative_error=abs(rows[-1]['normal_load_n']-load_n)/load_n,
                   station_count=station_count)
    return metrics,{'force_relative_error':(0.,.02),'native_wheel_contact_count':(0.,0.),
                    'energy_residual_ratio':(0.,.02)}


def contact_case_vertices(case, terrain_dx_m):
    if not np.isfinite(terrain_dx_m) or terrain_dx_m<=0:
        raise ValueError('road mesh spacing must be positive')
    x=np.linspace(-2.,2.,round(4./terrain_dx_m)+1)
    if case=='step':y=np.where(x>=0.,.04,0.)
    elif case=='two_support':y=.03*np.exp(-((x+.13)/.025)**2)+.03*np.exp(-((x-.13)/.025)**2)
    elif case=='edge_unload':y=np.where(x<0.,.04,0.)
    elif case=='recontact':y=np.where((x>=0.)&(x<.2),-.05,0.)
    elif case=='bump':y=.02*np.exp(-(x/.08)**2)
    elif case=='incline':y=.1*x
    else:raise ValueError('unknown distributed contact stand')
    return np.c_[x,y]


def distributed_road_trace(dt_s,station_count,terrain_dx_m,*,case='step',duration_s=.4):
    vertices=contact_case_vertices(case,terrain_dx_m)
    m,d,tire=wheel_stand(dt_s,station_count,vertices=vertices)
    load=600.;depth=brentq(lambda x:float(flat_load(x,.37,tire.material,station_count)[0])-load,1e-9,.1)
    d.qpos[0]=-.45;d.qpos[1]=float(np.interp(-.45,vertices[:,0],vertices[:,1]))-depth
    d.qvel[0]=2.;d.qvel[2]=2./.37
    mujoco.mj_forward(m,d)
    rows,metrics=_run_stand(m,d,tire,load,duration_s)
    dt=float(m.opt.timestep)
    metrics.update(normal_impulse_ns=sum(r['normal_load_n']*dt for r in rows),
                   vertical_impulse_ns=sum(r['vertical_force_n']*dt for r in rows),
                   max_patch_count=max(r['patch_count'] for r in rows),
                   multi_support_fraction=sum(r['multi_support'] for r in rows)/len(rows),
                   station_count=station_count,terrain_dx_m=terrain_dx_m)
    return rows,metrics


def distributed_step_rig(dt_s: float, station_count: int, terrain_dx_m: float):
    _,metrics=distributed_road_trace(dt_s,station_count,terrain_dx_m)
    return metrics,{'native_wheel_contact_count':(0.,0.),'energy_residual_ratio':(0.,.05),
                    'max_patch_count':(2.,float('inf'))}


def distributed_incline_rig(dt_s: float,station_count: int=128,angle_deg: float=15.):
    """Normal slide; tangent and spin locked by explicit ideal fixture reactions.

    The locked directions do no work. Fn = mg*cos(angle), while the total
    vertical support includes the fixture's tangential reaction, not Fn alone.
    """
    if not np.isfinite(angle_deg) or abs(angle_deg)>35:raise ValueError('incline fixture angle range is +/-35 degrees')
    angle=np.radians(angle_deg);n=np.array([-np.sin(angle),0.,np.cos(angle)])
    t=np.array([np.cos(angle),0.,np.sin(angle)]);radius=.37;mass=20.;weight=mass*9.81
    vertices=np.c_[np.linspace(-2,2,801),np.linspace(-2,2,801)*np.tan(angle)]
    y=vertices[:,1];minimum=float(y.min());scale=max(.01,float(np.ptp(y)))
    xml=f'''<mujoco><option timestep="{dt_s}" gravity="0 0 -9.81"/>
      <asset><hfield name="road" nrow="2" ncol="801" size="2 1 {scale} .2"/></asset>
      <worldbody><geom name="terrain" type="hfield" hfield="road" pos="0 0 {minimum}" contype="0" conaffinity="0"/>
      <body name="front_wheel" pos="{n[0]*radius} 0 {n[2]*radius}">
      <joint name="normal_slide" type="slide" axis="{n[0]} 0 {n[2]}"/>
      <inertial mass="{mass}" pos="0 0 0" diaginertia="1 1 1"/>
      <geom name="geom_front_contact" type="sphere" size="{radius}" mass="0" contype="0" conaffinity="0"/>
      </body><body name="rear_wheel" pos="0 0 5"><geom name="geom_rear_contact" type="sphere" size="{radius}" contype="0" conaffinity="0"/></body>
      </worldbody></mujoco>'''
    m=mujoco.MjModel.from_xml_string(xml);m.hfield_data[:]=np.tile((y-minimum)/scale,2)
    d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    config=TireBackendConfig(backend='distributed_2d_reference',distributed=DistributedTireConfig(station_count=station_count))
    tire=DistributedTireForceApplier(m,compiled_profile_vertices(m,d),config)
    def balance(depth):
        d.qpos[0]=-depth;mujoco.mj_forward(m,d)
        return float(tire.compute_qfrc(m,d,dt_s,advance=False)[0])-weight*np.cos(angle)
    depth=brentq(balance,1e-7,.05,xtol=1e-13);balance(depth)
    count=round(.05/dt_s)
    if count<1 or abs(count*dt_s-.05)>1e-10:raise ValueError('whole incline fixture intervals required')
    for _ in range(count):
        d.qfrc_applied[:]=tire.compute_qfrc(m,d,dt_s)
        mujoco.mj_step(m,d);mujoco.mj_forward(m,d)
    fn=tire.snapshots['front'].normal_load_n
    tangential=weight*np.sin(angle)
    total_vertical=float((fn*n+tangential*t)[2])
    return {'normal_load_n':fn,'expected_normal_n':weight*np.cos(angle),
        'normal_relative_error':abs(fn-weight*np.cos(angle))/weight,
        'total_vertical_support_n':total_vertical,'vertical_relative_error':abs(total_vertical-weight)/weight,
        'tangent_fixture_reaction_n':tangential,'speed_mps':float(d.qvel[0]),
        'deflection_m':depth,'native_contact_count':float(d.ncon),'angle_deg':angle_deg}, {
        'normal_relative_error':(0.,.01),'vertical_relative_error':(0.,.01),
        'speed_mps':(-1e-5,1e-5),'native_contact_count':(0.,0.)}
