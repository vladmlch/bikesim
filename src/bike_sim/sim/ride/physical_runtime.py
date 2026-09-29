"""Single owner of the physical initialization, force step and work ledger."""
from dataclasses import replace
import copy
import mujoco
import numpy as np
from bike_sim.physics.checks import scalar
from bike_sim.physics.rider_segments import geometry_pose
from bike_sim.sim.ride.contacts import TerrainContactQuery, TerrainContacts
from bike_sim.sim.ride.contact_filter import GroundedFilter
from bike_sim.sim.ride.tire_forces import TireForceApplier, compiled_profile_vertices
from bike_sim.sim.ride.drivetrain_forces import DrivetrainForceApplier
from bike_sim.sim.ride.physical_resistance import ExternalResistanceApplier
from bike_sim.sim.ride.static_braking import StaticBrakeApplier
from bike_sim.sim.ride.rider_contacts import RiderContactApplier
from bike_sim.sim.ride.rider_control import ArticulatedRiderController, RiderCommand
from bike_sim.sim.ride.physical_samples import PhysicalSample, WorkHistory
from bike_sim.sim.ride.telemetry_v2 import ForceSample
from bike_sim.sim.ride.constraint_forces import ConstraintForceSnapshot, shock_joint_limit_qfrc
from bike_sim.sim.ride.physical_observations import (
    tire_channels, energy_state, actuator_components, constraint_components,
)


class PhysicalRuntime:
    def __init__(self, sim):
        self.sim = sim
        self.cfg = sim.physics_config
        m, d = sim.model, sim.data
        if np.any(m.dof_armature):
            raise ValueError('physical momentum needs explicit rotor bodies, not hidden armature')
        mujoco.mj_forward(m,d)
        self.vertices = compiled_profile_vertices(m,d)
        self.tire = (TireForceApplier(m,self.vertices,self.cfg.tires)
                     if self.cfg.tires.backend == 'compliant_2d' else None)
        self.drive = DrivetrainForceApplier(m,self.cfg.drive,self.cfg.drive_mode)
        self.resistance = ExternalResistanceApplier(m,self.cfg.resistance)
        self.brake = StaticBrakeApplier(self.address('front_wheel_spin')[1],
            self.address('rear_wheel_spin')[1],self.cfg.drive.brake_ceiling_nm)
        self.rider_contacts = self.rider_control = None
        if sim.rider.variant == 'articulated_planar':
            pose = geometry_pose(sim.rider,sim.specs)
            self.rider_contacts = RiderContactApplier(m,pose,self.cfg.articulated)
            self.rider_control = ArticulatedRiderController(m,pose,self.cfg.articulated,sim.crank_length_m)
        self.probe_query = TerrainContactQuery(m)
        self.filters = {s:GroundedFilter(.005) for s in ('front','rear')}
        self.history = WorkHistory()
        self.sample = None
        self.initial_energy_j = None
        self.loss_j = 0.
        self.active_work_j = self.external_work_j = self.solver_work_j = 0.
        self.snapshots = {}
        self.generation = 0

    def address(self, name):
        m = self.sim.model
        jid = mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,name)
        if jid < 0:
            raise ValueError(f'physical model needs joint {name!r}')
        return int(m.jnt_qposadr[jid]),int(m.jnt_dofadr[jid])

    def _contacts(self, *, final=False, time_s=None, qvel=None, update_grounded=False):
        sim = self.sim
        query = sim.contact_query if final else self.probe_query
        raw = query.query(sim.model,sim.data,time_s=time_s,qvel=qvel)
        if self.tire is None:
            snapshots = {'front':raw.front_snapshot,'rear':raw.rear_snapshot}
        else:
            snapshots = self.tire.snapshots
            if not snapshots:
                probe = copy.copy(self.tire)
                probe.states = copy.deepcopy(self.tire.states)
                probe.last_time_s = None
                probe.compute_qfrc(sim.model,sim.data,float(sim.model.opt.timestep))
                snapshots = probe.snapshots
        t = float(sim.data.time) if time_s is None else time_s
        grounded = {}
        for side, snapshot in snapshots.items():
            loaded = sum(p.normal_load_n for p in snapshot.patches if p.working_surface) > 1.
            # One bool-only physical filter owner for both backends. The second
            # observation of this interval is idempotent at its equal timestamp.
            grounded[side] = (self.filters[side].update(loaded,t)
                              if final or update_grounded else loaded)
        if self.tire is None:
            return replace(raw,front_controller_grounded=grounded['front'],
                           rear_controller_grounded=grounded['rear']),snapshots
        front,rear = snapshots['front'],snapshots['rear']
        contacts = TerrainContacts(front_load_n=front.normal_load_n,rear_load_n=rear.normal_load_n,
            front_support_n=front.normal_vertical_n,rear_support_n=rear.normal_vertical_n,
            handlebar_load_n=raw.handlebar_load_n,front_snapshot=front,rear_snapshot=rear,
            front_controller_grounded=grounded['front'],rear_controller_grounded=grounded['rear'])
        return contacts,snapshots

    def apply_forces(self, *, active=True, advance=True, front=0., rear=0., external=None):
        """Evaluate all writers once; a non-advancing probe copies contact states."""
        sim = self.sim
        m,d = sim.model,sim.data
        dt = float(m.opt.timestep)
        d.qfrc_applied.fill(0.)
        d.xfrc_applied.fill(0.)
        d.ctrl.fill(0.)
        self.brake.apply(m,d,front,rear)
        mujoco.mj_forward(m,d)
        acc = sim.force_accumulator
        acc.clear()
        for name,force in sim.applier.compute_qfrc_components(m,d).items():
            acc.add(name,force)
        if external is not None:
            acc.add('external',external)
        if self.tire is not None:
            acc.add('tires',self.tire.compute_qfrc(m,d,dt,advance=advance))
        if sim.rider_forces.active:
            sim.rider_forces.apply(m,d)
            acc.add('seated_interfaces',d.qfrc_applied.copy())
            d.qfrc_applied.fill(0.)
        if self.rider_contacts is not None:
            acc.add('rider_interfaces',self.rider_contacts.compute_qfrc(m,d,dt,advance=advance))
            observed=(self.rider_contacts.diagnostics if advance else self.rider_contacts.probe_diagnostics)
            enabled=(self.rider_contacts.enabled if advance else self.rider_contacts.probe_enabled)
            loads = {side:observed.get(side+'_pedal',{}).get('normal_load_n',0.) for side in ('front','rear')}
            loads['grip'] = enabled.get('grip',False)
            loads['saddle'] = observed.get('saddle',{}).get('normal_load_n',0.)
            command = RiderCommand(self.cfg.drive.human_torque_nm
                if active and self.cfg.drive_mode=='articulated_effort' else 0.)
            availability={side:bool(enabled.get(side+'_pedal',False)
                and observed.get(side+'_pedal',{}).get('in_platform',False)) for side in ('front','rear')}
            # A geometrically available saddle is a posture goal even before
            # it carries load. This does not synthesize a normal force; the
            # pelvis must actually settle onto the unilateral surface.
            availability['saddle']=bool(enabled.get('saddle',False) and observed.get('saddle',{}).get('in_platform',False))
            availability['grip']=bool(enabled.get('grip',False))
            self.rider_control.write(d,self.rider_control.compute(m,d,command,contact_loads=loads,
                support_available=availability))
        sensed = self.rider_contacts.delivered_crank_torque_nm if self.rider_contacts is not None else 0.
        for name,force in self.drive.compute_components(m,d,dt,speed_mps=sim.speed_mps,
            braking=front>0 or rear>0,sensed_human_nm=sensed,active=active,advance=advance).items():
            acc.add(name,force)
        # Native contact loads must see the current suspension/chain/rider forces.
        d.qfrc_applied[:] = acc.total()
        mujoco.mj_forward(m,d)
        contacts,snapshots = self._contacts(update_grounded=active and advance)
        if active:
            for name,force in self.resistance.compute_components(m,d,snapshots).items():
                acc.add(name,force)
        sim.cruise.torque_nm = 0.
        if active and self.cfg.drive_mode == 'ideal_speed_control':
            torque = sim.cruise.compute(m,d,contacts,controller_grounded=contacts.rear_controller_grounded)
            if (front>0 or rear>0) and torque>0:
                torque = 0.
                sim.cruise.torque_nm = 0.
            d.ctrl[sim.drive_ctrl_adr] = torque
        d.qfrc_applied[:] = acc.total()
        return contacts,snapshots

    def reset(self):
        from bike_sim.sim.ride.physical_equilibrium import solve_physical_equilibrium
        sim = self.sim
        sim.equilibrium = solve_physical_equilibrium(self)
        self.generation += 1
        d,m = sim.data,sim.model
        d.time = 0.
        sim.steps = 0
        sim.contact_query.reset()
        self.probe_query.reset()
        for f in self.filters.values():
            f.reset()
        sim.cruise.reset(); sim.brakes.reset(); sim.stabilizer.reset(); sim.crash_detector.reset()
        sim.brake_source_cruise = False
        self.drive.restart_clock()
        if self.tire is not None:
            self.tire.restart_clock()
        if self.rider_contacts is not None:
            self.rider_contacts.restart_clock()
        self.history.reset()
        self.loss_j = self.active_work_j = self.external_work_j = self.solver_work_j = 0.
        self.sample = None
        self.initial_energy_j = None
        sim.last_force_sample = sim.last_force_snapshot = sim.last_constraint_snapshot = None
        sim.force_accumulator.clear()
        d.qfrc_applied.fill(0.); d.ctrl.fill(0.)
        mujoco.mj_forward(m,d)
        self.apply_forces(active=False,advance=False,front=self.cfg.initial_front_brake,rear=self.cfg.initial_rear_brake)
        mujoco.mj_forward(m,d)
        # The optimizer may have refined pose/shear since the last advancing
        # evaluation. Canonicalize the initial material state and loss datum;
        # these transitions belong to initialization, not a running interval.
        if self.tire is not None:
            self.tire.compute_qfrc(m,d,float(m.opt.timestep))
            self.tire.brush_loss_step_j=0.
            self.tire.restart_clock()
        if self.rider_contacts is not None:
            self.rider_contacts.initialize_settled_state(m,d)
        sim.contacts,self.snapshots = self._contacts(final=False)
        sim.equilibrium.update(static_front_load_n=self.snapshots['front'].normal_load_n,
                               static_rear_load_n=self.snapshots['rear'].normal_load_n,
                               total_vertical_force_n=sum(s.vertical_force_n for s in self.snapshots.values()))
        self._initial_speed()
        mujoco.mj_forward(m,d)
        mass,elastic,total=energy_state(self)
        self.initial_energy_j=total
        self.energy_scale_j=max(1.,mass['kinetic_energy_j']+sum(elastic.values()))
        self.initial_battery_j=self.drive.battery.energy_j
        self.electrical_work_j=0.
        self.energy = {'mechanical_energy_j':total,'elastic_energy_j':elastic,
                      'residual_j':0.,'energy_scale_j':self.energy_scale_j}
        sim._update_compiled_com_marker()

    def _initial_speed(self):
        sim=self.sim; m,d=sim.model,sim.data
        speed=self.cfg.initial_speed_mps
        # Initial speed is along the working road, not into an inclined surface.
        # Both independent roots get the same translational velocity; wheel spin
        # below cancels contact slip after subtracting each carrier's rotation.
        tangent = sum((p.normal_load_n*p.tangent
                       for snap in self.snapshots.values() for p in snap.patches
                       if p.working_surface), np.zeros(3))
        tangent = tangent/np.linalg.norm(tangent) if np.linalg.norm(tangent)>0 else np.array([1.,0.,0.])
        for root in ('root', 'rider_root') if self.rider_control is not None else ('root',):
            d.qvel[self.address(root+'_x')[1]]=speed*tangent[0]
            d.qvel[self.address(root+'_z')[1]]=speed*tangent[2]
        mujoco.mj_forward(m,d)
        from bike_sim.sim.ride.physical_mapping import body_angular_velocity, point_velocity
        for side in ('front','rear'):
            gid=sim.contact_query.front_id if side=='front' else sim.contact_query.rear_id
            bid=int(m.geom_bodyid[gid]); dof=self.address(side+'_wheel_spin')[1]
            snap=self.snapshots[side]
            tangent=snap.patches[0].tangent if snap.patches else np.array([1.,0.,0.])
            radius=snap.effective_radius_m if snap.effective_radius_m>0 else float(m.geom_size[gid,0])
            roll=float(point_velocity(m,d,bid,d.geom_xpos[gid])@tangent)
            parent_omega=float(body_angular_velocity(m,d,bid)[1])-float(d.qvel[dof])
            d.qvel[dof]=roll/radius-parent_omega
        mujoco.mj_forward(m,d)

    def _loss_increment(self, forces, velocity, dt):
        """Material losses only; road/aero/native-solver work stay signed external."""
        sim=self.sim
        loss=sum(max(0.,-float(forces[name]@velocity))*dt for name in
                 ('fork_damper','shock_damper','shock_hbo','drive_bearings') if name in forces)
        if 'engine_passive' in forces:
            from bike_sim.sim.ride.physical_energy import engine_passive_loss_power
            loss += engine_passive_loss_power(sim.model, sim.last_force_sample.qpos,
                                              velocity, forces['engine_passive'])*dt
        for side in ('front','rear'):
            loss+=max(0.,-float(forces.get(side+'_static_brake',np.zeros_like(velocity))@velocity))*dt
        loss+=self.drive.last.get('chain_dissipation_power_w',0.)*dt
        loss+=self.drive.last.get('freehub_dissipation_power_w',0.)*dt
        if self.tire is not None:
            loss+=self.tire.brush_loss_step_j+self.tire.radial_dissipation_power_w*dt
        if self.rider_contacts is not None:
            loss+=self.rider_contacts.loss_step_j+self.rider_contacts.radial_dissipation_power_w*dt
        for path in sim.rider_forces._paths:
            b=path.body
            depth=b.preload_deflection_m+path.offset_m-float(sim.last_force_sample.qpos[path.qposadr])
            elastic=b.stiffness_n_m*(max(depth,0.) if b.unilateral else depth)
            loss+=max(0.,-(path.force_n-elastic)*velocity[path.dofadr])*dt
        # End-stop damping is superimposed on an explicitly integrated spring.
        x=float(sim.last_force_sample.qpos[sim.applier.shock_qposadr]); v=velocity[sim.applier.shock_dofadr]
        cfg=self.cfg.end_stops; nominal=sim.applier.coil_shock.specs.stroke_mm/1000.
        if x<0:
            elastic=-cfg.stiffness_n_m*x
            loss+=max(0.,-(forces['shock_top_out'][sim.applier.shock_dofadr]-elastic)*v)*dt
        if x>nominal:
            elastic=-(sim.applier.coil_shock.specs.bumper_peak_n+cfg.stiffness_n_m*(x-nominal))
            loss+=max(0.,-(forces['shock_upper_stop'][sim.applier.shock_dofadr]-elastic)*v)*dt
        return float(loss)

    def step(self, front=0., rear=0., external=None):
        front=scalar(front,'front brake demand'); rear=scalar(rear,'rear brake demand')
        sim=self.sim; m,d=sim.model,sim.data; dt=float(m.opt.timestep)
        if external is not None:
            external=np.array(external,dtype=float,copy=True)
            if external.shape!=(m.nv,) or not np.isfinite(external).all():
                raise ValueError('invalid generalized force')
        t=float(d.time); q=d.qpos.copy(); v=d.qvel.copy()
        self.apply_forces(front=front,rear=rear,external=external)
        # Save incoming auxiliary energies before the solve overwrites no state.
        mass0,_,_=energy_state(self)
        sim.last_force_sample=ForceSample(t,q,v,sim.force_accumulator.components)
        sim.last_force_snapshot=(t,sim.last_force_sample.qpos,sim.last_force_sample.qvel,sim.last_force_sample.components)
        components={k:np.array(f,copy=True) for k,f in sim.force_accumulator.components.items()}
        warning_counts=np.array([w.number for w in d.warning],copy=True)
        mujoco.mj_step(m,d)
        for warning in (mujoco.mjtWarning.mjWARN_BADQPOS,mujoco.mjtWarning.mjWARN_BADQVEL,mujoco.mjtWarning.mjWARN_BADQACC):
            if d.warning[int(warning)].number>warning_counts[int(warning)]:
                raise RuntimeError(f'MuJoCo numerical failure: {warning.name}')
        from bike_sim.sim.ride.physical_crash import physical_contact_crash
        contact_crash=physical_contact_crash(m,d)
        self.drive.settle_actuation(m,d)
        components.update(actuator_components(m,d))
        constraints=constraint_components(m,d)
        shock_limit=shock_joint_limit_qfrc(m,d)
        # Split the shock limit out of aggregate joint limits; never count it twice.
        constraints['joint_limits']-=shock_limit
        constraints['shock_solver_limit']=shock_limit
        components.update(constraints)
        components.update(self.brake.solved_components(m,d))
        components['engine_passive']=d.qfrc_passive.copy()
        sim.last_constraint_snapshot=ConstraintForceSnapshot(t,float(d.time),v,
            {'shock_solver_limit':shock_limit,**self.brake.solved_components(m,d)})
        sim.contacts,self.snapshots=self._contacts(final=True,time_s=t,qvel=v)
        tires=tire_channels(self,self.snapshots,qvel=v)
        loss_step=self._loss_increment(components,v,dt)
        self.loss_j+=loss_step
        active_names={'human_crank','mid_drive'}|{n for n in components if n.startswith('act_rider_')}
        external_names={'external','rear_drive','road_rolling','aerodynamic','native_contact'}
        self.active_work_j+=sum(float(components[n]@v)*dt for n in active_names if n in components)
        self.external_work_j+=sum(float(components[n]@v)*dt for n in external_names if n in components)
        self.solver_work_j+=sum(float(components[n]@v)*dt for n in ('joint_limits','shock_solver_limit','closure') if n in components)
        self.electrical_work_j+=self.drive.last.get('electrical_power_w',0.)*dt
        rider={} if self.rider_contacts is None else copy.deepcopy(self.rider_contacts.diagnostics)
        drive=dict(self.drive.last,crank_phase_rad=float(q[self.address('crank_spin')[0]]),
                   gear_ratio=self.cfg.drive.gearing.front_teeth/self.cfg.drive.gearing.rear_teeth)
        # Capture all solved quantities before refreshing the endpoint kinematics.
        equality_rows=d.efc_type[:d.nefc]==mujoco.mjtConstraint.mjCNSTR_EQUALITY
        linkage_error=float(np.max(np.abs(d.efc_pos[:d.nefc][equality_rows]))) if np.any(equality_rows) else 0.
        shock_limit_power=float(sim.last_constraint_snapshot.components['shock_solver_limit']@v)
        mujoco.mj_forward(m,d)
        if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():
            raise RuntimeError('non-finite physical simulation state')
        mass,elastic,total=energy_state(self)
        self.energy={'mechanical_energy_j':total,'elastic_energy_j':elastic,
            'active_work_j':self.active_work_j,'external_work_j':self.external_work_j,
            'loss_j':self.loss_j,'loss_step_j':loss_step,'solver_constraint_work_j':self.solver_work_j,'energy_scale_j':self.energy_scale_j,
            'residual_j':total-self.initial_energy_j-self.active_work_j-self.external_work_j+self.loss_j,
            'electrical_work_j':self.electrical_work_j,
            'electrical_residual_j':self.initial_battery_j-self.drive.battery.energy_j-self.electrical_work_j}
        drive.update(chain_power_w=float(components['chain']@v),freehub_power_w=float(components['freehub']@v),
            front_brake_power_w=float(components['front_static_brake']@v),rear_brake_power_w=float(components['rear_static_brake']@v),
            front_brake_torque_nm=float(components['front_static_brake'][self.address('front_wheel_spin')[1]]),
            rear_brake_torque_nm=float(components['rear_static_brake'][self.address('rear_wheel_spin')[1]]),
            human_joint_power_w=sum(float(f@v) for n,f in components.items() if n.startswith('act_rider_')))
        suspension={'fork_travel_m':float(q[sim.applier.fork_qposadr]),
            'shock_stroke_m':float(q[sim.applier.shock_qposadr]),
            'fork_velocity_mps':float(v[sim.applier.fork_dofadr]),
            'shock_velocity_mps':float(v[sim.applier.shock_dofadr]),
            'shock_solver_limit_power_w':shock_limit_power,'linkage_closure_max_m':linkage_error,
            'generalized_force_components_n':{n:float(f[sim.applier.fork_dofadr if n.startswith('fork') else sim.applier.shock_dofadr])
                for n,f in components.items() if n.startswith(('fork_','shock_'))}}
        channels={'tires':tires,'drive':drive,'rider':rider,'suspension':suspension,
                  'mass':mass0,'endpoint_mass':mass,'energy':self.energy,
                  'component_work_j':{n:self.history.work_j.get(n,0.)+float(f@v)*dt for n,f in components.items()},
                  'rider_control':{} if self.rider_control is None else self.rider_control.last_terms,
                  'rider_ik_saturation':{} if self.rider_control is None else self.rider_control.saturated_ik,
                  'rider_support_targets':{} if self.rider_control is None else self.rider_control.support_diagnostics,
                  'contact_crash_cause':contact_crash}
        self.sample=PhysicalSample(sim.steps,t,float(d.time),q,v,components,channels)
        self.history.add(self.sample)
        sim.steps+=1
        if contact_crash is not None and sim.crash_detector.event is None:
            from bike_sim.sim.ride.virtual_rider import CrashEvent
            sim.crash_detector.event=CrashEvent(contact_crash,t,float(q[sim.root_x_qposadr]),float(q[sim.root_pitch_qposadr]))
        sim.crash_detector.check(d,sim.contacts)
        sim._update_compiled_com_marker()
