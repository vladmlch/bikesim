"""Independent engine rigs with analytic expectations, in SI units.

These rigs deliberately omit the bicycle suspension when testing tires and the
terrain when testing inertia/transmission, so one subsystem cannot mask another.
"""
from math import pi, sin, cos
import mujoco
import numpy as np
from bike_sim.physics.checks import scalar
from bike_sim.physics.tire import normal_contact
from bike_sim.physics.chain import DrivetrainSpecs, chain_jacobian, chain_extension
from bike_sim.physics.freehub import Freehub


def _steps(duration, dt):
    scalar(duration, 'duration', positive=True); scalar(dt, 'dt', positive=True)
    n=round(duration/dt)
    if n<1 or not np.isclose(n*dt,duration,rtol=0,atol=1e-10):
        raise ValueError('rig duration must be an integer number of timesteps')
    return n


def radial_rig(load_n,dt,duration_s=2.):
    load_n=scalar(load_n,'radial load',positive=True)
    steps=_steps(duration_s,dt)
    model=mujoco.MjModel.from_xml_string(f'''<mujoco><option timestep="{dt:.17g}" gravity="0 0 0"/>
    <worldbody><body><joint type="slide" axis="0 0 1"/>
    <inertial mass="2.4" pos="0 0 0" diaginertia="0.1 0.2 0.1"/></body></worldbody></mujoco>''')
    data=mujoco.MjData(model)
    for _ in range(steps):
        force,_=normal_contact(-float(data.qpos[0]),-float(data.qvel[0]),130000.,800.)
        data.qfrc_applied[0]=force-load_n
        mujoco.mj_step(model,data)
    return {'load_n':load_n,'dt_s':dt,'deflection_m':-float(data.qpos[0]),
            'speed_mps':float(data.qvel[0]),'expected_deflection_m':load_n/130000.}


def native_radial_rig(load_n,dt,duration_s=2.):
    load_n=scalar(load_n,'radial load',positive=True)
    steps=_steps(duration_s,dt)
    model=mujoco.MjModel.from_xml_string(f'''<mujoco><option timestep="{dt:.17g}" gravity="0 0 0"/>
    <worldbody><geom type="plane" size="1 1 .1"/><body pos="0 0 .353">
    <joint type="slide" axis="0 0 1"/><inertial mass="2.4" pos="0 0 0" diaginertia=".1 .2 .1"/>
    <geom type="sphere" size=".352" mass="0" solref="-130000 -800" priority="1"/>
    </body></worldbody></mujoco>''')
    data=mujoco.MjData(model)
    data.qfrc_applied[0]=-load_n
    for _ in range(steps):mujoco.mj_step(model,data)
    return {'load_n':load_n,'dt_s':dt,'effective_deflection_m':-float(data.qpos[0])-.001,
            'speed_mps':float(data.qvel[0])}


def wheel_inertia_rig(dt,duration_s=.5):
    steps=_steps(duration_s,dt);inertia=.24;torque=3.
    model=mujoco.MjModel.from_xml_string(f'''<mujoco><option timestep="{dt:.17g}" gravity="0 0 0"/>
    <worldbody><body><joint type="hinge" axis="0 1 0"/>
    <inertial mass="2.4" pos="0 0 0" diaginertia=".12 .24 .12"/>
    </body></worldbody></mujoco>''')
    data=mujoco.MjData(model);data.qfrc_applied[0]=torque
    for _ in range(steps):mujoco.mj_step(model,data)
    expected=torque/inertia*duration_s
    return {'omega_rad_s':float(data.qvel[0]),'expected_omega_rad_s':expected,
            'acceleration_relative_error':abs(float(data.qvel[0])/duration_s-torque/inertia)/(torque/inertia),
            'rotational_energy_j':.5*inertia*float(data.qvel[0])**2,
            'expected_energy_j':.5*inertia*expected**2}


def incline_rig(dt,duration_s=2.,angle_deg=30.):
    """A compliant wheel held by static axle friction, free to roll along a slope.

    The radial slide is free; only this diagnostic stand's tangential translation
    is constrained. Reaction Fn is W cos(angle), not total vertical load W.
    """
    steps=_steps(duration_s,dt); angle=angle_deg*pi/180.; mass=20.
    n=np.array([-sin(angle),0.,cos(angle)])
    model=mujoco.MjModel.from_xml_string(f'''<mujoco><option timestep="{dt:.17g}"/>
    <worldbody><body><joint type="slide" axis="{-sin(angle)} 0 {cos(angle)}"/>
    <inertial mass="{mass}" pos="0 0 0" diaginertia="1 1 1"/></body></worldbody></mujoco>''')
    data=mujoco.MjData(model); force=0.
    for _ in range(steps):
        force,_=normal_contact(-float(data.qpos[0]),-float(data.qvel[0]),130000.,800.)
        data.qfrc_applied[0]=force;mujoco.mj_step(model,data)
    tangential_force=mass*9.81*sin(angle)
    tangent=np.array([cos(angle),0.,sin(angle)])
    world=force*n+tangential_force*tangent
    return {'normal_load_n':force,'expected_normal_load_n':mass*9.81*cos(angle),
            'normal_vertical_n':force*cos(angle),'vertical_with_stand_reaction_n':float(world[2]),
            'expected_weight_n':mass*9.81,'deflection_m':-float(data.qpos[0])}


def brake_rig(dt,duration_s=5.):
    """One fixed axle: solved bounded friction must hold, then yield above ceiling."""
    steps=_steps(duration_s,dt)
    model=mujoco.MjModel.from_xml_string(f'''<mujoco><option timestep="{dt:.17g}" gravity="0 0 0"/>
    <worldbody><body><joint type="hinge" axis="0 1 0" frictionloss="20"
      solreffriction=".005 1" solimpfriction=".9999 .9999 .001 .5 2"/>
    <inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/></body></worldbody></mujoco>''')
    data=mujoco.MjData(model);data.qfrc_applied[0]=10.
    for _ in range(steps):mujoco.mj_step(model,data)
    held=float(data.qpos[0]); work=0.
    data.qfrc_applied[0]=30.
    for _ in range(_steps(.1,dt)):
        speed=float(data.qvel[0]);mujoco.mj_step(model,data)
        work+=float(data.qfrc_constraint[0])*speed*dt
    return {'held_angle_rad':held,'overload_speed_rad_s':float(data.qvel[0]),'brake_work_j':work}


def chain_locked_rig(dt,duration_s=1.):
    """Preloaded, lossless three-shaft stand in steady rotation under a known load."""
    steps=_steps(duration_s,dt); specs=DrivetrainSpecs(); rf=specs.front_radius_m;rr=specs.rear_radius_m
    ratio=rf/rr;load=10.; tension=load/rr; k=200000.; hub=Freehub(1000.,0.)
    model=mujoco.MjModel.from_xml_string(f'''<mujoco><option timestep="{dt:.17g}" gravity="0 0 0"/>
    <worldbody>{''.join(f'<body pos="{i} 0 0"><joint type="hinge" axis="0 1 0"/><inertial mass="1" pos="0 0 0" diaginertia="{I/2} {I} {I/2}"/></body>' for i,I in enumerate((.02,.005,.2)))}</worldbody></mujoco>''')
    data=mujoco.MjData(model); data.qpos[1]=load/hub.k
    data.qpos[0]=(tension/k+rr*data.qpos[1])/rf
    data.qvel[:]=[10.,10.*ratio,10.*ratio];hub.boundary=0.
    input_work=output_work=0.; last_t=last_h=0.
    for _ in range(steps):
        e=rf*data.qpos[0]-rr*data.qpos[1]; last_t=k*max(e,0.)
        last_h=hub.update(data.qpos[1],data.qpos[2],data.qvel[1],data.qvel[2])
        data.qfrc_applied[:]=[load*ratio-last_t*rf,last_t*rr-last_h,last_h-load]
        input_work+=load*ratio*float(data.qvel[0])*dt;output_work+=load*float(data.qvel[2])*dt
        mujoco.mj_step(model,data)
    return {'speed_ratio':float(data.qvel[2]/data.qvel[0]),'expected_ratio':ratio,
            'torque_ratio':last_t*rf/last_h,'power_relative_error':abs(input_work-output_work)/abs(input_work),
            'cassette_wheel_speed_difference':float(data.qvel[1]-data.qvel[2])}


def chain_geometry_rig():
    cf=np.array([0.,0.]);cr=np.array([-.45,.02]);rf=.07;rr=.04
    base=chain_extension(cf,cr,rf,rr,0.,0.,0.)
    angle=.41;R=np.array([[cos(angle),-sin(angle)],[sin(angle),cos(angle)]])
    rotated=chain_extension(R@cf+3,R@cr+3,rf,rr,-angle,-angle,base,up_xz=R@np.array([0.,1.]))
    def evaluate(q):return chain_extension(cf,q[:2],rf,rr,q[2],q[3],base)
    q=np.r_[cr,.1,.2];direction=np.array([.7,-.3,.2,-.1]);J=chain_jacobian(q,evaluate)
    h=2e-6; independent=(evaluate(q+h*direction)-evaluate(q-h*direction))/(2*h)
    return {'rigid_motion_extension_m':rotated,'directional_relative_error':abs(float(J@direction)-independent)/max(abs(independent),1e-8),
            'suspension_chain_gradient_x':float(J[0]),'suspension_chain_gradient_z':float(J[1])}


def contact_event_rig(dt):
    """First contact has the independent ballistic time clearance / speed."""
    model=mujoco.MjModel.from_xml_string(f'''<mujoco><option timestep="{dt:.17g}" gravity="0 0 0"/>
    <worldbody><body><joint type="slide" axis="0 0 1"/>
    <inertial mass="2.4" pos="0 0 0" diaginertia=".1 .2 .1"/></body></worldbody></mujoco>''')
    data=mujoco.MjData(model);clearance=.01;speed=1.
    data.qpos[0]=clearance;data.qvel[0]=-speed
    event=None
    for _ in range(_steps(.03,dt)):
        force,_=normal_contact(-float(data.qpos[0]),-float(data.qvel[0]),130000.,800.)
        if force>0 and event is None:event=float(data.time)
        data.qfrc_applied[0]=force;mujoco.mj_step(model,data)
    if event is None:raise RuntimeError('wheel never contacted the stand')
    return {'contact_time_s':event,'analytic_contact_time_s':clearance/speed,
            'event_error_s':abs(event-clearance/speed)}


def compiled_chain_motion_rig():
    """Move the real linkage while holding both sprockets' absolute phases.

    This is a kinematic virtual-work stand, not a freely evolving road run.
    Constraint closure is intentionally not imposed on the independent virtual
    coordinate perturbations used to check the generalized force gradient.
    """
    from bike_sim.mujoco.builder import generate_mujoco_xml
    from bike_sim.physics.model_config import SimulationPhysicsConfig
    from bike_sim.physics.physical_config import PhysicalDriveConfig
    from bike_sim.sim.ride.drivetrain_forces import DrivetrainForceApplier
    model = mujoco.MjModel.from_xml_string(generate_mujoco_xml(
        mode='ride', rider='none', physics_config=SimulationPhysicsConfig('physical')))
    data = mujoco.MjData(model)
    drive = DrivetrainForceApplier(model, PhysicalDriveConfig(), 'coast')
    mujoco.mj_forward(model, data)
    drive.reset(model, data)
    base = data.qpos.copy()
    pivot = model.joint('main_pivot').qposadr[0]
    cassette = model.joint('cassette_spin').qposadr[0]
    errors, coupling = [], []
    phase = drive._angle(data, 'cassette')
    for displacement in np.linspace(-.05, .05, 9):
        data.qpos[:] = base
        data.qpos[pivot] += displacement
        mujoco.mj_forward(model, data)
        data.qpos[cassette] += phase - drive._angle(data, 'cassette', phase)
        mujoco.mj_forward(model, data)
        saved = data.qpos.copy()
        analytic = drive.jacobian(model, data)
        numeric = drive.finite_difference_jacobian(model, data)
        errors.append(float(np.linalg.norm(analytic - numeric) / max(np.linalg.norm(numeric), 1e-12)))
        coupling.append(abs(float(analytic[model.joint('main_pivot').dofadr[0]])))
        if not np.array_equal(data.qpos, saved):
            raise AssertionError('chain oracle changed live generalized coordinates')
    return {'compiled_jacobian_relative_error': max(errors),
            'linkage_force_per_tension_m': max(coupling)}
