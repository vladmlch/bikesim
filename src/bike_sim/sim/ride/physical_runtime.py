"""Single owner of the physical initialization, force step and work ledger."""
from dataclasses import replace, asdict
from contextlib import contextmanager
from bike_sim.sim.ride.control import RideControl
from bike_sim.physics.rider_posture import RiderPosture
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
    tire_channels, energy_state, actuator_components, constraint_components, sensor_channels,
)
from bike_sim.sim.ride.physical_energy import mass_observations
from bike_sim.sim.ride.rider_intent import RiderIntentResolver, signals_from_channels
from bike_sim.physics.seated_climb import SeatedClimbSignals


class PhysicalRuntime:
    def __init__(self, sim):
        self.sim = sim
        self.cfg = sim.physics_config
        if self.cfg.seated_climb.enabled and sim.rider.variant != 'articulated_planar':
            raise ValueError('seated climb needs an articulated planar rider')
        self.rider_intent = RiderIntentResolver(self.cfg.seated_climb, float(sim.model.opt.timestep))
        self.rider_intent_signals = SeatedClimbSignals()
        self.applied_control = RideControl()
        m, d = sim.model, sim.data
        if np.any(m.dof_armature):
            raise ValueError('physical momentum needs explicit rotor bodies, not hidden armature')
        mujoco.mj_forward(m,d)
        self.vertices = compiled_profile_vertices(m,d)
        self.tire = (TireForceApplier(m,self.vertices,self.cfg.tires,sim.track.surface_map)
                     if self.cfg.tires.backend == 'compliant_2d' else None)
        if self.cfg.tires.backend == 'distributed_2d_reference':
            from bike_sim.sim.ride.distributed_tire_forces import DistributedTireForceApplier
            self.tire = DistributedTireForceApplier(m,self.vertices,self.cfg.tires,sim.track.surface_map)
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
        from bike_sim.sim.ride.model_status import ModelStatus
        self.model_status = ModelStatus()
        self.initial_energy_j = None
        self.loss_j = 0.
        self.active_work_j = self.external_work_j = self.solver_work_j = 0.
        self.snapshots = {}
        self.generation = 0
        self.interactive_preview = False
        self.preview_real_time_factor = None
        self.research_accounting_valid = True
        self._rollback_hold = False

    @contextmanager
    def preview_mode(self):
        """Scope viewer diagnostics; reset before resuming research accounting."""
        previous=self.interactive_preview
        self.interactive_preview=True
        try:
            yield
        finally:
            self.interactive_preview=previous

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

    def _rollback_brake_demand(self, speed_mps, control=None):
        """Hill-hold reflex: sustained rollback engages the wheel brakes.

        The hold releases only once the bike has nearly stopped, so a stalled
        climb cannot run away downhill. It is a physical restraint, never a
        rider intent signal: pedaling and assist keep working while it holds.
        """
        cfg = self.cfg.drive.pedaling
        if (not cfg.rollback_brake or cfg.rollback_demand <= 0.
                or self.cfg.drive_mode not in ('crank_effort', 'articulated_effort')
                or (control is not None and not control.rider_enabled)):
            self._rollback_hold = False
            return 0.
        if self._rollback_hold:
            self._rollback_hold = speed_mps <= -cfg.rollback_release_mps
        else:
            self._rollback_hold = speed_mps < -cfg.rollback_engage_mps
        return cfg.rollback_demand if self._rollback_hold else 0.

    def apply_forces(self, *, active=True, advance=True, front=0., rear=0., external=None,
                     control=None, braking=None):
        """Evaluate all writers once; a non-advancing probe copies contact states.

        ``braking`` overrides rider brake intent when the demands already carry
        a reflex hold (hill-hold must not read as the rider grabbing brakes).
        """
        sim = self.sim
        control = RideControl() if control is None else control
        if not isinstance(control, RideControl):
            raise ValueError('expected an immutable RideControl')
        control.validate_for(self.cfg, sim.rider.variant)
        automatic_effort = (active and self.cfg.seated_climb.enabled
                            and control.human_torque_nm is None and control.rider_enabled)
        if self.cfg.seated_climb.enabled:
            control = self.rider_intent.resolve(control, self.rider_intent_signals,
                step=sim.steps, active=active, advance=advance)
        if advance:
            self.applied_control = control
        if braking is None:
            braking = front > 0. or rear > 0.
        elif not isinstance(braking, bool):
            raise ValueError('brake intent must be a bool')
        m,d = sim.model,sim.data
        dt = float(m.opt.timestep)
        d.qfrc_applied.fill(0.)
        d.xfrc_applied.fill(0.)
        d.ctrl.fill(0.)
        self.brake.apply(m,d,front,rear)
        rear_snapshot = sim.contacts.rear_snapshot
        pedaling = self.drive.prepare_pedaling(d, dt, control, active=active,
            advance=advance, braking=braking, model=m,
            rear_in_contact=bool(sim.contacts.rear_controller_grounded),
            rear_slip_mps=None if rear_snapshot is None else float(rear_snapshot.slip_mps),
            effort_ceiling_nm=control.human_torque_nm if automatic_effort else None)
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
            acc.add('rider_interfaces',self.rider_contacts.compute_qfrc(m,d,dt,advance=advance,
                detailed=not self.interactive_preview))
            observed=(self.rider_contacts.diagnostics if advance else self.rider_contacts.probe_diagnostics)
            enabled=(self.rider_contacts.enabled if advance else self.rider_contacts.probe_enabled)
            loads = {side:observed.get(side+'_pedal',{}).get('normal_load_n',0.) for side in ('front','rear')}
            loads['grip'] = enabled.get('grip',False)
            loads['saddle'] = observed.get('saddle',{}).get('normal_load_n',0.)
            crank_goal = pedaling.target_phase_rad
            if not active and self.drive.ideal_hub is not None:
                crank_goal = self.cfg.drive.crank_phase_rad
            command = RiderCommand(pedaling.effort_nm if self.cfg.drive_mode=='articulated_effort' else 0.,
                enabled=control.rider_enabled, posture=control.posture or RiderPosture(),
                crank_target_phase_rad=crank_goal,
                crank_target_rate_rad_s=pedaling.target_rate_rad_s)
            availability={side:bool(enabled.get(side+'_pedal',False)
                and observed.get(side+'_pedal',{}).get('in_platform',False)) for side in ('front','rear')}
            # A geometrically available saddle is a posture goal even before
            # it carries load. This does not synthesize a normal force; the
            # pelvis must actually settle onto the unilateral surface.
            availability['saddle']=bool(enabled.get('saddle',False) and observed.get('saddle',{}).get('in_platform',False))
            availability['grip']=bool(enabled.get('grip',False))
            self.rider_control.write(d,self.rider_control.compute(m,d,command,contact_loads=loads,
                support_available=availability,advance=advance,dt_s=dt,steady_state=not active,
                support_states=observed if self.cfg.seated_climb.enabled else None))
            acc.add('rider_joint_envelope',self.rider_control.envelope_forces(m,d)[0])
        if self.rider_contacts is None:
            sensed = 0.
        elif advance:
            sensed = self.rider_contacts.delivered_crank_torque_nm
        else:
            sensed = getattr(self.rider_contacts, 'probe_delivered_crank_torque_nm',
                             self.rider_contacts.delivered_crank_torque_nm)
        for name,force in self.drive.compute_components(m,d,dt,speed_mps=sim.speed_mps,
            braking=braking,sensed_human_nm=sensed,active=active,advance=advance,
            control=control,pedaling_state=pedaling).items():
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
        self.rider_intent.reset()
        self.rider_intent_signals = SeatedClimbSignals()
        self.applied_control = RideControl()
        self.research_accounting_valid=False
        self.preview_real_time_factor=None
        self.initializing=True
        try:
            seed = sim.physical_initial_state
            sim.equilibrium = solve_physical_equilibrium(self) if seed is None else seed.prepare(self)
        finally:
            self.initializing=False
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
        if self.drive.ideal_hub is not None:
            self.drive.ideal_hub.reset(m, d)
        if self.tire is not None:
            self.tire.restart_clock()
        if self.rider_contacts is not None:
            self.rider_contacts.restart_clock()
        self.history.reset()
        self.loss_j = self.active_work_j = self.external_work_j = self.solver_work_j = 0.
        self.sample = None
        from bike_sim.sim.ride.model_status import ModelStatus
        self.model_status = ModelStatus()
        self.initial_energy_j = None
        sim.last_force_sample = sim.last_force_snapshot = sim.last_constraint_snapshot = None
        sim.force_accumulator.clear()
        d.qfrc_applied.fill(0.); d.ctrl.fill(0.)
        self._rollback_hold = False
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
        if self.rider_control is not None:
            c=self.rider_control
            requests=c.effort_diagnostics.get('rider_active_request_nm',{})
            c.active_state=np.array([requests.get(n,0.) for n in c.joints])
            c.activation_time_s=None
        self._initial_speed()
        mujoco.mj_forward(m,d)
        if sim.physical_initial_state is not None:
            sim.physical_initial_state.restore(self)
        mass,elastic,total=energy_state(self)
        self.initial_energy_j=total
        self.energy_scale_j=max(1.,mass['kinetic_energy_j']+sum(elastic.values()))
        self.initial_battery_j=self.drive.battery.energy_j
        self.electrical_work_j=0.
        self.energy = {'mechanical_energy_j':total,'elastic_energy_j':elastic,
                      'residual_j':0.,'energy_scale_j':self.energy_scale_j}
        sim._update_compiled_com_marker()
        self.research_accounting_valid=True
        if self.cfg.seated_climb.enabled:
            self.rider_intent_signals = signals_from_channels(sensor_channels(self,
                drive_channels=getattr(self.drive, 'probe_last', self.drive.last)))

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
        pedaling = (self.cfg.drive_mode in ('crank_effort', 'articulated_effort')
                    and self.cfg.drive.human_torque_nm > 0.)
        ratio = self.cfg.drive.gearing.front_teeth / self.cfg.drive.gearing.rear_teeth
        rate = d.qvel[self.address('rear_wheel_spin')[1]] / ratio
        if self.cfg.drive.pedaling.enabled and abs(rate) * 60. / (2. * np.pi) >= self.cfg.drive.pedaling.coast_above_rpm:
            pedaling = False
        if pedaling:
            # A rolling, already pedaling initial condition must not kick a
            # stationary crank/legs up to wheel speed through the transmission.
            # A genuinely coasting elastic drivetrain remains freewheeling.
            d.qvel[self.address('crank_spin')[1]] = rate
            cassette = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'cassette_spin')
            if cassette >= 0:
                d.qvel[int(m.jnt_dofadr[cassette])] = d.qvel[self.address('rear_wheel_spin')[1]]
            frame = m.body('frame').id
            frame_pitch_rate = float(body_angular_velocity(m, d, frame)[1])
            for side in ('front', 'rear'):
                d.qvel[self.address(f'pedal_{side}_spin')[1]] = -rate-frame_pitch_rate
            if self.rider_control is not None:
                self.rider_control.initialize_velocity(m, d)
        mujoco.mj_forward(m,d)

    def _loss_increment(self, forces, velocity, dt):
        """Material losses only; road/aero/native-solver work stay signed external."""
        sim=self.sim
        loss=sum(max(0.,-float(forces[name]@velocity))*dt for name in
                 ('fork_damper','shock_damper','shock_hbo','drive_bearings','rider_passive_damping') if name in forces)
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

    def step(self, front=0., rear=0., external=None, *, control=None):
        if not self.interactive_preview and not self.research_accounting_valid:
            raise RuntimeError('reset is required after interactive preview before research accounting')
        control = RideControl() if control is None else control
        if not isinstance(control, RideControl):
            raise ValueError('expected an immutable RideControl')
        control.validate_for(self.cfg, self.sim.rider.variant)
        front=scalar(front,'front brake demand'); rear=scalar(rear,'rear brake demand')
        sim=self.sim; m,d=sim.model,sim.data; dt=float(m.opt.timestep)
        if external is not None:
            external=np.array(external,dtype=float,copy=True)
            if external.shape!=(m.nv,) or not np.isfinite(external).all():
                raise ValueError('invalid generalized force')
        if self.interactive_preview:
            return self._step_preview(front,rear,external,control)
        t=float(d.time); q=d.qpos.copy(); v=d.qvel.copy()
        braking = front > 0. or rear > 0.
        hold = self._rollback_brake_demand(sim.speed_mps, control)
        if hold:
            front = max(front, hold); rear = max(rear, hold)
        self.apply_forces(front=front,rear=rear,external=external,control=control,braking=braking)
        # Save incoming auxiliary energies before the solve overwrites no state.
        mass0=mass_observations(m,d)
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
        transmission = self.drive.settle_actuation(m,d)
        effort={}
        if self.rider_control is not None:
            from bike_sim.sim.ride.rider_effort import solved_effort
            effort=solved_effort(self.rider_control,d,v,dt)
        sensors = sensor_channels(self, qvel=v)
        if self.cfg.seated_climb.enabled:
            self.rider_intent_signals = signals_from_channels(sensors)
        components.update(actuator_components(m,d))
        if self.rider_control is not None:
            passive=np.zeros(m.nv)
            for name,(_,dof,aid) in self.rider_control.joints.items():
                damping=self.rider_control.last_terms[name]['solved_passive_nm']
                passive[dof]=damping
                components['act_'+name][dof]-=damping
            components['rider_passive_damping']=passive
        constraints=constraint_components(m,d)
        if self.drive.ideal_hub is not None:
            constraints['joint_limits'] -= transmission
            constraints['ideal_transmission'] = transmission
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
        self.solver_work_j+=sum(float(components[n]@v)*dt for n in ('joint_limits','shock_solver_limit','closure','ideal_transmission') if n in components)
        self.electrical_work_j+=self.drive.last.get('electrical_power_w',0.)*dt
        rider={} if self.rider_contacts is None else copy.deepcopy(self.rider_contacts.diagnostics)
        drive=dict(self.drive.last,crank_phase_rad=float(q[self.address('crank_spin')[0]]),
                   front_brake_demand=front,rear_brake_demand=rear,rollback_brake_demand=hold)
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
            'electrical_residual_j':(self.initial_battery_j-self.drive.battery.energy_j-self.electrical_work_j
                                    if self.cfg.drive.battery.enabled else 0.)}
        drive.update(chain_power_w=float(components['chain']@v),
            freehub_power_w=float((components['freehub']+components.get('ideal_transmission',np.zeros(m.nv)))@v),
            front_brake_power_w=float(components['front_static_brake']@v),rear_brake_power_w=float(components['rear_static_brake']@v),
            front_brake_torque_nm=float(components['front_static_brake'][self.address('front_wheel_spin')[1]]),
            rear_brake_torque_nm=float(components['rear_static_brake'][self.address('rear_wheel_spin')[1]]),
            human_active_power_w=sum(float(f@v) for n,f in components.items() if n.startswith('act_rider_')),
            human_joint_power_w=sum(float(f@v) for n,f in components.items() if n.startswith('act_rider_'))
                +float(components.get('rider_passive_damping',np.zeros(m.nv))@v))
        suspension={'fork_travel_m':float(q[sim.applier.fork_qposadr]),
            'shock_stroke_m':float(q[sim.applier.shock_qposadr]),
            'fork_velocity_mps':float(v[sim.applier.fork_dofadr]),
            'shock_velocity_mps':float(v[sim.applier.shock_dofadr]),
            'shock_solver_limit_power_w':shock_limit_power,'linkage_closure_max_m':linkage_error,
            'generalized_force_components_n':{n:float(f[sim.applier.fork_dofadr if n.startswith('fork') else sim.applier.shock_dofadr])
                for n,f in components.items() if n.startswith(('fork_','shock_'))}}
        channels={**effort,'tires':tires,'drive':drive,'rider':rider,'suspension':suspension,
                  'control':asdict(self.applied_control), 'sensors':sensors,
                  'rider_intent':(dict(asdict(self.rider_intent.intent),
                      inclination_rad=self.rider_intent.policy.inclination_rad)
                      if self.cfg.seated_climb.enabled else {}),
                  'mass':mass0,'endpoint_mass':mass,'energy':self.energy,
                  'component_work_j':{n:self.history.work_j.get(n,0.)+float(f@v)*dt for n,f in components.items()},
                  'rider_control':{} if self.rider_control is None else self.rider_control.last_terms,
                  'rider_ik_saturation':{} if self.rider_control is None else self.rider_control.saturated_ik,
                  'rider_support_targets':{} if self.rider_control is None else self.rider_control.support_diagnostics,
                  'contact_crash_cause':contact_crash}
        self.model_status.observe(sim.steps,t,channels)
        channels['model_status']=self.model_status.as_dict()
        self.sample=PhysicalSample(sim.steps,t,float(d.time),q,v,components,channels)
        self.history.add(self.sample)
        sim.steps+=1
        if contact_crash is not None and sim.crash_detector.event is None:
            from bike_sim.sim.ride.virtual_rider import CrashEvent
            sim.crash_detector.event=CrashEvent(contact_crash,t,float(q[sim.root_x_qposadr]),float(q[sim.root_pitch_qposadr]))
        sim.crash_detector.check(d,sim.contacts)
        sim._update_compiled_com_marker()

    def _step_preview(self, front, rear, external, control):
        """Same force/integration path without research samples or energy audits."""
        sim=self.sim
        self.research_accounting_valid=False
        model,data=sim.model,sim.data
        time_s=float(data.time)
        incoming_velocity=data.qvel.copy()
        position_m=sim.position_m
        pitch_rad=sim.pitch_rad
        braking = front > 0. or rear > 0.
        hold = self._rollback_brake_demand(sim.speed_mps, control)
        if hold:
            front = max(front, hold); rear = max(rear, hold)
        self.apply_forces(front=front,rear=rear,external=external,control=control,braking=braking)
        warning_counts=np.array([warning.number for warning in data.warning],copy=True)
        mujoco.mj_step(model,data)
        for warning in (mujoco.mjtWarning.mjWARN_BADQPOS,mujoco.mjtWarning.mjWARN_BADQVEL,mujoco.mjtWarning.mjWARN_BADQACC):
            if data.warning[int(warning)].number>warning_counts[int(warning)]:
                raise RuntimeError(f'MuJoCo numerical failure: {warning.name}')
        from bike_sim.sim.ride.physical_crash import physical_contact_crash
        contact_crash=physical_contact_crash(model,data)
        self.drive.settle_actuation(model,data)
        if self.cfg.seated_climb.enabled:
            self.rider_intent_signals = signals_from_channels(sensor_channels(self, qvel=incoming_velocity))
        if self.rider_control is not None:
            from bike_sim.sim.ride.rider_effort import solved_effort
            solved_effort(self.rider_control,data,incoming_velocity,float(model.opt.timestep))
        sim.contacts,self.snapshots=self._contacts(final=True,time_s=time_s,qvel=incoming_velocity)
        equality=data.efc_type[:data.nefc]==mujoco.mjtConstraint.mjCNSTR_EQUALITY
        closure=float(np.max(np.abs(data.efc_pos[:data.nefc][equality]))) if np.any(equality) else 0.
        preview_channels={'tires':tire_channels(self,self.snapshots,qvel=incoming_velocity),
                          'suspension':{'linkage_closure_max_m':closure}}
        self.model_status.observe(sim.steps,time_s,preview_channels)
        mujoco.mj_forward(model,data)
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RuntimeError('non-finite physical simulation state')
        sim.steps+=1
        if contact_crash is not None and sim.crash_detector.event is None:
            from bike_sim.sim.ride.virtual_rider import CrashEvent
            sim.crash_detector.event=CrashEvent(contact_crash,time_s,position_m,pitch_rad)
        sim.crash_detector.check(data,sim.contacts)
        sim._update_compiled_com_marker()
