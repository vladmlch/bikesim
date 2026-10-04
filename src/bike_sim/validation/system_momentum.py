"""Independent whole-system momentum from compiled body mass and inertia."""
import numpy as np
import mujoco


def system_momentum(model,data):
    if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
        raise ValueError('momentum requires finite physical coordinates')
    bodies=np.flatnonzero(model.body_mass>0)
    masses=model.body_mass[bodies];positions=data.xipos[bodies]
    total=float(masses.sum())
    if not np.isfinite(total) or total<=0:
        raise ValueError('system must contain positive mass')
    # Subtract an origin before weighted summation for translation invariance.
    origin=positions[0];com=origin+np.sum(masses[:,None]*(positions-origin),axis=0)/total
    linear,angular=np.zeros(3),np.zeros(3)
    jp,jr=np.zeros((3,model.nv)),np.zeros((3,model.nv))
    for body in bodies:
        mujoco.mj_jacBodyCom(model,data,jp,jr,int(body))
        velocity,omega=jp@data.qvel,jr@data.qvel
        p=float(model.body_mass[body])*velocity
        rotation=data.ximat[body].reshape(3,3)
        inertia=rotation@np.diag(model.body_inertia[body])@rotation.T
        linear+=p;angular+=inertia@omega+np.cross(data.xipos[body]-com,p)
    return com,linear,angular


def momentum_rig(dt_s,active: bool,*,transmission='geometric_ideal_mid_drive',rider_active=None):
    """Actual motor + articulated joints on the complete airborne compiled tree.

    The initial airborne pose is declared, not called a ground equilibrium.
    Uniform gravity is the only external translational force. The omitted roll
    and yaw freedoms imply plane-constraint moments, reported separately; only
    the free pitch momentum is tested as conserved, never the full 3D vector. Suspension, drivetrain and
    rider-contact forces are applied as their ordinary internal Newton pairs.
    Declared synthetic rider intent and sensor input of 10 N.m permit the
    active motor pulse; neither applies rider force. The stand's separate
    joint controller still receives zero pedal effort. This tests momentum,
    not real pedal sensing or pedelec engagement.
    """
    if type(active) is not bool:raise ValueError('active must be a bool')
    if rider_active is None:rider_active=active
    if type(rider_active) is not bool:raise ValueError('rider_active must be a bool')
    from dataclasses import replace
    from bike_sim.physics.model_config import SimulationPhysicsConfig
    from bike_sim.physics.physical_config import PhysicalDriveConfig,AssistConfig
    from bike_sim.physics.chain import DrivetrainSpecs
    from bike_sim.physics.rider import RiderSpecs
    from bike_sim.geometry.specs import BikeSpecs
    from bike_sim.physics.rider_segments import geometry_pose
    from bike_sim.mujoco.builder import generate_mujoco_xml
    from bike_sim.sim.ride.rider_control import ArticulatedRiderController,RiderCommand
    from bike_sim.sim.ride.rider_contacts import RiderContactApplier
    from bike_sim.sim.ride.drivetrain_forces import DrivetrainForceApplier
    from bike_sim.sim.ride.control import RideControl
    from bike_sim.sim.ride.forces import SuspensionForceApplier
    from bike_sim.physics.suspension_config import build_suspension_components
    cfg=SimulationPhysicsConfig('physical',drive_mode='articulated_effort',timestep_s=dt_s,
        closure_time_constant_s=.0025,drive=PhysicalDriveConfig(transmission_model=transmission,
        human_torque_nm=0.,gearing=DrivetrainSpecs(34,51),assist=AssistConfig(gain=2.,tau=.03)))
    specs=BikeSpecs();rider=RiderSpecs(variant='articulated_planar');pose=geometry_pose(rider,specs)
    m=mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode='ride',specs=specs,rider=rider,physics_config=cfg))
    d=mujoco.MjData(m);controller=ArticulatedRiderController(m,pose,cfg.articulated,specs.crank_length/1000.)
    controller.initialize(m,d)
    for root in ('root_z','rider_root_z'):d.qpos[m.joint(root).qposadr[0]]+=30.
    d.qpos[controller.joints['rider_torso_hinge'][0]]+=.01
    for name in ('front_wheel_spin','rear_wheel_spin'):d.qvel[m.joint(name).dofadr[0]]=10.
    d.qvel[m.joint('crank_spin').dofadr[0]]=4.
    mujoco.mj_forward(m,d)
    contacts=RiderContactApplier(m,pose,cfg.articulated);contacts.reset(m,d)
    drive=DrivetrainForceApplier(m,cfg.drive,cfg.drive_mode);drive.reset(m,d)
    suspension=SuspensionForceApplier(m,*build_suspension_components(specs),physics_config=cfg)
    initial=system_momentum(m,d)
    duration=.1;count=round(duration/dt_s)
    if count<1 or abs(count*dt_s-duration)>1e-10:raise ValueError('momentum duration needs integer steps')
    max_p=max_l=max_ly=0.;max_out_of_plane=0.;external_impulse=np.zeros(3);motor_work=joint_work=power_error=peak=0.
    root_actuation=0.;ground_contacts=0.;max_defect=0.
    for i in range(count):
        torque=20. if active and i<count//2 else 0.
        mujoco.mj_forward(m,d)
        contact_force=contacts.compute_qfrc(m,d,dt_s)
        observed=contacts.diagnostics
        availability={side:contacts.enabled[side+'_pedal'] and observed.get(side+'_pedal',{}).get('in_platform',False) for side in ('front','rear')}
        availability.update(grip=contacts.enabled['grip'],saddle=observed.get('saddle',{}).get('in_platform',False))
        commands=controller.compute(m,d,RiderCommand(0.,enabled=rider_active),
                                    support_available=availability,dt_s=dt_s)
        controller.write(d,commands)
        sensor_fixture_nm = 10. if torque > 0. else 0.
        components=drive.compute_components(m,d,dt_s,speed_mps=0.,
            sensed_human_nm=sensor_fixture_nm,
            control=RideControl(motor_torque_nm=torque,human_torque_nm=sensor_fixture_nm))
        d.qfrc_applied[:]=sum(components.values(),np.zeros(m.nv))+contact_force+suspension.compute_qfrc(m,d)
        velocity=d.qvel.copy();mujoco.mj_step(m,d);drive.settle_actuation(m,d)
        for name in ('root_x','root_z','root_pitch','rider_root_x','rider_root_z','rider_root_pitch'):
            root_actuation=max(root_actuation,abs(float(d.qfrc_actuator[m.joint(name).dofadr[0]])))
        actual=drive.last['motor_torque_nm'];shaft=velocity[m.joint('crank_spin').dofadr[0]]
        power_error=max(power_error,abs(drive.last['motor_shaft_power_w']-actual*shaft))
        motor_work+=actual*shaft*dt_s;peak=max(peak,actual)
        joint_work+=sum(max(0.,float(d.actuator_force[aid]*velocity[dof])) for _,dof,aid in controller.joints.values())*dt_s
        max_defect=max(max_defect,abs(drive.last.get('transmission_constraint_defect_m',0.)))
        mujoco.mj_forward(m,d);external_impulse+=float(m.body_mass.sum())*m.opt.gravity*dt_s
        _,linear,angular=system_momentum(m,d)
        max_p=max(max_p,float(np.linalg.norm(linear-initial[1]-external_impulse)))
        delta=angular-initial[2]
        max_l=max(max_l,float(np.linalg.norm(delta)))
        max_ly=max(max_ly,abs(float(delta[1])))
        max_out_of_plane=max(max_out_of_plane,float(np.linalg.norm(delta[[0,2]])))
        # Airborne internal rider contacts are permitted, terrain/catch contact is not.
        for c in d.contact[:d.ncon]:
            names=[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,int(g)) or '' for g in c.geom]
            ground_contacts+=int(any(n in ('terrain','catch_plane') for n in names))
    mass=float(m.body_mass.sum());impulse_scale=max(1.,mass*np.linalg.norm(m.opt.gravity)*duration)
    angular_scale=max(1.,float(np.linalg.norm(initial[2])),impulse_scale*1.)
    return {'impulse_residual_ns':max_p,'impulse_residual_ratio':max_p/impulse_scale,
            'angular_momentum_residual_kg_m2_s':max_l,'angular_residual_ratio':max_ly/angular_scale,
            'pitch_angular_momentum_residual_kg_m2_s':max_ly,
            'planar_roll_yaw_constraint_impulse_required_kg_m2_s':max_out_of_plane,
            'impulse_floor_ns':1.,'angular_scale_length_m':1.,'impulse_scale_ns':impulse_scale,
            'motor_shaft_work_j':motor_work,'joint_positive_work_j':joint_work,'peak_motor_torque_nm':peak,
            'shaft_power_identity_error_w':power_error,'root_actuator_force_n':root_actuation,
            'ground_contact_count':ground_contacts,'constraint_defect_m':max_defect,
            'duration_s':duration,'mass_kg':mass,
            'synthetic_sensor_torque_nm':10. if active else 0.,
            'synthetic_rider_intent_nm':10. if active else 0.}, {'impulse_residual_ratio':(0.,.001),
            'angular_residual_ratio':(0.,.001),'shaft_power_identity_error_w':(0.,1e-10),
            'root_actuator_force_n':(0.,0.),'ground_contact_count':(0.,0.),
            'peak_motor_torque_nm':(1e-8,20.) if active else (0.,0.),
            'motor_shaft_work_j':(1e-8,np.inf) if active else (0.,0.)}
