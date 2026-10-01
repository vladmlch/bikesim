"""Serialized physical contact, force and energy observations (no state advance)."""
import mujoco
import numpy as np
from bike_sim.sim.ride.physical_mapping import body_angular_velocity, point_velocity, point_jacobian
from bike_sim.sim.ride.physical_energy import mass_observations, suspension_energy


def tire_channels(runtime, snapshots, *, qvel=None):
    sim = runtime.sim
    model, data = sim.model, sim.data
    result = {}
    for side, snapshot in snapshots.items():
        body = runtime.drive.ids.get(side+'_wheel')
        if body is None:
            body = int(model.geom_bodyid[sim.contact_query.front_id if side=='front' else sim.contact_query.rear_id])
        _, dof = runtime.address(side+'_wheel_spin')
        velocity_q = data.qvel if qvel is None else qvel
        jp, jr = point_jacobian(model,data,body,snapshot.wheel_axis_m)
        omega = float((jr@velocity_q)[1])
        # Called before solve: all these values belong to q_n, v_n.
        velocity = jp@velocity_q
        patches = [{'point_m': p.point_m, 'normal': p.normal, 'normal_load_n': p.normal_load_n,
                    'tangent_force_n': p.tangent_force_n, 'slip_mps': p.slip_mps,
                    'world_force_n': p.world_force_n, 'couple_world_nm':p.couple_world_nm,
                    'source_geom':p.source_geom} for p in snapshot.patches]
        slip = (sum(p.slip_mps*p.normal_load_n for p in snapshot.patches)/snapshot.normal_load_n
                if snapshot.normal_load_n > 0 else 0.)
        roll_speed = (sum(float(velocity @ p.tangent) * p.normal_load_n for p in snapshot.patches)
                      / snapshot.normal_load_n if snapshot.normal_load_n > 0 else float(velocity[0]))
        diagnostics = runtime.tire.diagnostics.get(side,{}) if runtime.tire is not None else {}
        radius=(runtime.tire.radii[side] if runtime.tire is not None else
                float(model.geom_size[getattr(sim.contact_query,side+'_id'),0]))
        result[side] = dict(diagnostics, backend=snapshot.backend,
            unloaded_radius_m=radius,
            supports_multiple_contacts=bool(diagnostics.get('supports_multiple_contacts',False)),
            outside_material_load_range=bool(diagnostics.get('outside_material_load_range',False)),
            patches=patches, geometric_contact=snapshot.geometric_contact,
            raw_contact=snapshot.road_loaded_contact, normal_load_n=snapshot.normal_load_n,
            world_force_n=snapshot.world_force_n, vertical_force_n=snapshot.vertical_force_n,
            normal_vertical_n=snapshot.normal_vertical_n, tangent_force_n=snapshot.tangent_force_n,
            effective_radius_m=snapshot.effective_radius_m, wheel_axis_m=snapshot.wheel_axis_m,
            wheel_axis_moment_nm=snapshot.wheel_axis_moment_nm, omega_abs_rad_s=omega,
            omega_rel_rad_s=float(velocity_q[dof]), slip_mps=slip,
            slip_ratio=-slip/max(abs(roll_speed),.1),
            tangent_center_speed_mps=roll_speed,
            controller_grounded=bool(getattr(sim.contacts,side+'_controller_grounded',False)),
            multi_support=bool(diagnostics.get('multi_support',False)))
    return result


def stored_terms(runtime):
    sim = runtime.sim
    terms = suspension_energy(sim.applier,sim.data)
    terms.update(runtime.drive.stored_energy(sim.model,sim.data))
    if runtime.tire is not None:
        terms['tires'] = runtime.tire.stored_energy(sim.model,sim.data)
    if runtime.rider_control is not None:
        terms['rider_joint_envelope']=runtime.rider_control.envelope_forces(sim.model,sim.data)[1]
    if runtime.rider_contacts is not None:
        terms['rider_interfaces'] = runtime.rider_contacts.stored_energy(sim.model,sim.data)
    for path in sim.rider_forces._paths:
        depth = path.body.preload_deflection_m+path.offset_m-float(sim.data.qpos[path.qposadr])
        if path.body.unilateral:
            depth = max(depth,0.)
        terms['seated_'+path.body.name] = .5*path.body.stiffness_n_m*depth**2
    # Explicit engine springs are stored separately from applied-force springs.
    for j, stiffness in enumerate(sim.model.jnt_stiffness):
        if stiffness:
            q = sim.data.qpos[sim.model.jnt_qposadr[j]]-sim.model.qpos_spring[sim.model.jnt_qposadr[j]]
            terms['engine_joint_'+str(j)] = .5*float(stiffness)*float(q*q)
    return terms


def energy_state(runtime):
    mass = mass_observations(runtime.sim.model,runtime.sim.data)
    elastic = stored_terms(runtime)
    total = mass['kinetic_energy_j']+mass['gravitational_energy_j']+sum(elastic.values())
    return mass,elastic,float(total)


def actuator_components(model,data):
    """All physical actuators are scalar joint motors with explicit gear=1."""
    result={}
    for aid in range(model.nu):
        name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_ACTUATOR,aid)
        jid=int(model.actuator_trnid[aid,0])
        if int(model.actuator_trntype[aid]) != int(mujoco.mjtTrn.mjTRN_JOINT):
            raise ValueError('physical actuation requires direct joint motors')
        force=np.zeros(model.nv)
        force[model.jnt_dofadr[jid]]=data.actuator_force[aid]*model.actuator_gear[aid,0]
        result[name]=force
    return result


def constraint_components(model,data):
    """Separate contacts, joint limits and closure work using solved EFC rows."""
    groups={'native_contact':[], 'joint_limits':[], 'closure':[]}
    for i,t in enumerate(data.efc_type[:data.nefc]):
        if t in (mujoco.mjtConstraint.mjCNSTR_CONTACT_FRICTIONLESS,
                 mujoco.mjtConstraint.mjCNSTR_CONTACT_PYRAMIDAL,
                 mujoco.mjtConstraint.mjCNSTR_CONTACT_ELLIPTIC):
            groups['native_contact'].append(i)
        elif t in (mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT,mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON):
            groups['joint_limits'].append(i)
        elif t==mujoco.mjtConstraint.mjCNSTR_EQUALITY:
            groups['closure'].append(i)
    result={}
    for name,indices in groups.items():
        force=np.zeros(model.nv)
        weights=np.zeros(data.nefc)
        weights[indices]=data.efc_force[indices]
        if indices:
            mujoco.mj_mulJacTVec(model,data,force,weights)
        result[name]=force
    return result


def sensor_channels(runtime, *, qvel=None, drive_channels=None):
    """Raw input-state proper acceleration, gyro, encoders and shaft sensors.

    Called after the constraint solve and before endpoint forward kinematics.
    MuJoCo accelerometers report specific force: gravity is not subtracted a
    second time. The research wrapper adds sensor noise and transport delay.
    """
    d = runtime.sim.data
    drive = runtime.drive.last if drive_channels is None else drive_channels
    v = d.qvel if qvel is None else qvel
    encoders = {side: float(v[runtime.address(side+'_spin')[1]])
                for side in ('front_wheel', 'rear_wheel', 'crank')}
    return dict(frame_specific_force_body_mps2=d.sensor('sensor_frame_accel').data.copy(),
        frame_gyro_body_rad_s=d.sensor('sensor_frame_gyro').data.copy(), encoders_rad_s=encoders,
        motor_torque_nm=float(drive.get('motor_torque_nm', 0.)),
        human_torque_nm=float(drive.get('human_sensor_nm', 0.)))
