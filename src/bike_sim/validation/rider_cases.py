"""Articulated-rider checks using actual compiled bodies and joint actuation."""
import mujoco
import numpy as np
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.physics.rider import RiderSpecs
from bike_sim.physics.rider_segments import geometry_pose
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.sim.ride.rider_control import ArticulatedRiderController, RiderCommand
from bike_sim.sim.ride.physical_energy import mass_observations


def airborne_internal_actuation(dt):
    """A18 with moving limbs: no support force, no root actuator, 2 seconds.

    The reference articulated pose is the declared airborne initial condition,
    not a claimed static equilibrium. Gravity is uniform; the selected joint
    actuators and engine bearing losses act only internally.
    """
    cfg=SimulationPhysicsConfig('physical',drive_mode='articulated_effort',timestep_s=dt)
    specs=BikeSpecs();rider=RiderSpecs(variant='articulated_planar')
    pose=geometry_pose(rider,specs)
    model=mujoco.MjModel.from_xml_string(generate_mujoco_xml(mode='ride',rider=rider,physics_config=cfg))
    data=mujoco.MjData(model)
    controller=ArticulatedRiderController(model,pose,cfg.articulated,specs.crank_length/1000.)
    controller.initialize(model,data)
    for name in ('root_z','rider_root_z'):
        data.qpos[model.joint(name).qposadr[0]]+=30.
    data.qpos[controller.joints['rider_torso_hinge'][0]]+=.03
    for name in ('front_wheel_spin','rear_wheel_spin'):
        data.qvel[model.joint(name).dofadr[0]]=10.
    mujoco.mj_forward(model,data)
    initial=mass_observations(model,data);q0=data.qpos.copy()
    scale=max(1.,abs(float(initial['angular_momentum_kg_m2_s'][1])))
    maximum=work=absolute_work=peak=0.
    for _ in range(round(2./dt)):
        mujoco.mj_forward(model,data)
        commands=controller.compute(model,data,RiderCommand(20.),contact_loads={},support_available={})
        controller.write(data,commands)
        velocity=data.qvel.copy()
        mujoco.mj_step(model,data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise ArithmeticError('non-finite articulated flight')
        power=sum(float(data.actuator_force[aid]*velocity[dof]) for _,dof,aid in controller.joints.values())
        work+=power*dt;absolute_work+=abs(power)*dt
        peak=max(peak,max(abs(float(data.actuator_force[aid])) for _,_,aid in controller.joints.values()))
        mujoco.mj_forward(model,data)
        observation=mass_observations(model,data)
        maximum=max(maximum,abs(float(observation['angular_momentum_kg_m2_s'][1])-initial['angular_momentum_kg_m2_s'][1]))
    joint_motion=max(abs(float(data.qpos[qa]-q0[qa])) for qa,_,_ in controller.joints.values())
    return {'relative_Ly_drift':maximum/scale,'joint_motion_rad':joint_motion,
            'actual_joint_work_j':work,'absolute_joint_work_j':absolute_work,'peak_joint_torque_nm':peak}, {
            'relative_Ly_drift':(0.,.001),'joint_motion_rad':(.01,np.inf),
            'absolute_joint_work_j':(.01,np.inf),'peak_joint_torque_nm':(0.,cfg.articulated.joint_limit_nm+1e-9)}
