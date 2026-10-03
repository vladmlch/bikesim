"""Independent scalar integration oracle captured before P3 batching.

G5 corrects its constraint-work inputs to individual physical constraints, just
as the real ledger requires; scalar stepping and scalar wrench recovery remain
independent of the batched runtime. Never called by runtime.
"""
from dataclasses import replace, asdict
from contextlib import contextmanager
from bike_sim.sim.ride.control import RideControl
from bike_sim.sim.ride.control_clock import ControlClock
from bike_sim.physics.rider_posture import RiderPosture
import copy
import mujoco
import numpy as np
from bike_sim.physics.checks import scalar
from bike_sim.physics.energy_ledger import step_work
from bike_sim.physics.rider_segments import geometry_pose
from bike_sim.sim.ride.contacts import TerrainContactQuery, TerrainContacts
from bike_sim.sim.ride.contact_filter import GroundedFilter
from bike_sim.sim.ride.tire_forces import TireForceApplier, compiled_profile_vertices
from bike_sim.sim.ride.drivetrain_forces import DrivetrainForceApplier
from bike_sim.sim.ride.physical_resistance import ExternalResistanceApplier
from bike_sim.sim.ride.static_braking import StaticBrakeApplier
from bike_sim.sim.ride.rider_contacts import RiderContactApplier
from bike_sim.sim.ride.rider_control import ArticulatedRiderController, RiderCommand
from bike_sim.sim.ride.physical_mapping import resolve_id
from bike_sim.sim.ride.rider_state import rider_kinematic_state
from bike_sim.sim.ride.physical_samples import PhysicalSample, WorkHistory, freeze
from bike_sim.sim.ride.telemetry_v2 import ForceSample
from bike_sim.sim.ride.constraint_forces import ConstraintForceSnapshot, shock_joint_limit_qfrc
from bike_sim.sim.ride.physical_observations import (
    tire_channels, preview_tire_channels, energy_state, actuator_components,
    constraint_components, sensor_channels, numerical_constraint_powers,
)
from bike_sim.sim.ride.physical_energy import mass_observations
from bike_sim.sim.ride.rider_intent import RiderIntentResolver, signals_from_channels
from bike_sim.physics.seated_climb import SeatedClimbSignals



from bike_sim.sim.ride.physical_runtime import _connect_equality_rows

def step_reference(self, front=0., rear=0., external=None, *, control=None):
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
    constraint_powers=numerical_constraint_powers(m,d,v)
    for warning in (mujoco.mjtWarning.mjWARN_BADQPOS,mujoco.mjtWarning.mjWARN_BADQVEL,mujoco.mjtWarning.mjWARN_BADQACC):
        if d.warning[int(warning)].number>warning_counts[int(warning)]:
            raise RuntimeError(f'MuJoCo numerical failure: {warning.name}')
    # Read first: efc_force and poses still belong to this solved interval.
    # (q, v) are the interval's start state, where MuJoCo built efc_J.
    welds={} if self.rider_contacts is None else self.rider_contacts.settle_welds(m,d,(q,v))
    if self.rider_contacts is None:
        self.attachment_samples,self.attachment_errors={},()
    else:
        self.attachment_samples=self.rider_contacts.last_attachment_samples
        self.attachment_errors=self.rider_contacts.last_attachment_errors
    self.step_violations=self._attachment_violations()
    from bike_sim.sim.ride.physical_crash import physical_contact_crash
    contact_crash=physical_contact_crash(m,d)
    transmission = self.drive.settle_actuation(m,d)
    effort={}
    if self.rider_control is not None:
        from bike_sim.sim.ride.rider_effort import solved_effort
        effort=solved_effort(self.rider_control,d,(q,v),dt)
        self.step_violations+=tuple(
            f'rider_strength.{name}'
            for name in effort.get('rider_strength_violations',()))
        # A QP that cannot find any budget-holding command is a controller
        # fault: physics may stall, but a silent infeasible command is not
        # a legitimate stall.
        if self.rider_control.allocation_diagnostics.get('invalid_controller'):
            self.step_violations+=('rider_controller.infeasible',)
    sensors = sensor_channels(self, qvel=v)
    if self.cfg.seated_climb.enabled:
        self.rider_intent_signals = signals_from_channels(sensors)
    components.update(actuator_components(m,d))
    if self.rider_control is not None:
        passive=np.zeros(m.nv)
        for name,(_,dof,aid) in self.rider_control.joints.items():
            passive[dof]=self.rider_control.last_terms[name]['solved_passive_nm']
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
    # Rider tissue damping is a DOF-damping row inside qfrc_passive; the
    # rider_passive_damping component already accounts for it. Removing
    # the rows here keeps the loss ledger free of double counting.
    if self.rider_control is not None:
        components['engine_passive']-=components['rider_passive_damping']
    sim.last_constraint_snapshot=ConstraintForceSnapshot(t,float(d.time),v,
        {'shock_solver_limit':shock_limit,**self.brake.solved_components(m,d)})
    sim.contacts,self.snapshots=self._contacts(final=True,time_s=t,qvel=v)
    tires=tire_channels(self,self.snapshots,qvel=v)
    loss_step=self._loss_increment(components,v,dt)
    self.loss_j+=loss_step
    external_names={'external','rear_drive','road_rolling','aerodynamic','native_contact'}
    muscle_power=np.array([float(components[n]@v) for n in components
                           if n=='human_crank' or n.startswith('act_rider_')])
    motor_power=float(components.get('mid_drive',np.zeros(m.nv))@v)
    constraint_power=np.array(list(constraint_powers.values()))
    work=step_work(muscle_power,motor_power,constraint_power,dt)
    self.muscle_signed_j+=work.muscle_signed_j
    self.muscle_positive_j+=work.muscle_positive_j
    self.motor_signed_j+=work.motor_signed_j
    self.motor_positive_j+=work.motor_positive_j
    self.constraint_absolute_j+=work.constraint_absolute_j
    self.solver_work_j+=work.constraint_signed_j
    self.active_work_j+=work.muscle_signed_j+work.motor_signed_j
    self.external_work_j+=sum(float(components[n]@v)*dt for n in external_names if n in components)
    self.electrical_work_j+=self.drive.last.get('electrical_power_w',0.)*dt
    rider={} if self.rider_contacts is None else copy.deepcopy(self.rider_contacts.diagnostics)
    drive=dict(self.drive.last,crank_phase_rad=float(q[self.address('crank_spin')[0]]),
               front_brake_demand=front,rear_brake_demand=rear,rollback_brake_demand=hold)
    # Capture all solved quantities before refreshing the endpoint kinematics.
    # Only connect-equality rows measure linkage closure; weld rows 3-5 are
    # rotational residuals in radians and must stay out of a metre metric.
    connect_rows=_connect_equality_rows(m,d)
    linkage_error=float(np.max(np.abs(d.efc_pos[:d.nefc][connect_rows]))) if np.any(connect_rows) else 0.
    shock_limit_power=float(sim.last_constraint_snapshot.components['shock_solver_limit']@v)
    mujoco.mj_forward(m,d)
    if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():
        raise RuntimeError('non-finite physical simulation state')
    mass,elastic,total=energy_state(self)
    self.energy={'mechanical_energy_j':total,'elastic_energy_j':elastic,
        'active_work_j':self.active_work_j,'external_work_j':self.external_work_j,
        'loss_j':self.loss_j,'loss_step_j':loss_step,'solver_constraint_work_j':self.solver_work_j,'energy_scale_j':self.energy_scale_j,
        'muscle_signed_j':self.muscle_signed_j,'muscle_positive_j':self.muscle_positive_j,
        'motor_signed_j':self.motor_signed_j,'motor_positive_j':self.motor_positive_j,
        'source_positive_work_j':self.muscle_positive_j+self.motor_positive_j,
        'constraint_signed_j':self.solver_work_j,'constraint_absolute_j':self.constraint_absolute_j,
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
    channels={**effort,'tires':tires,'drive':drive,'rider':rider,'rider_welds':welds,'suspension':suspension,
              'control':asdict(self.applied_control), 'sensors':sensors,
              'rider_intent':(dict(asdict(self.rider_intent.intent),
                  inclination_rad=self.rider_intent.policy.inclination_rad)
                  if self.cfg.seated_climb.enabled else {}),
              'mass':mass0,'endpoint_mass':mass,'energy':self.energy,
              'attachment_violations':self.step_violations,
              'attachment_samples':{n:asdict(s) for n,s in self.attachment_samples.items()},
              'component_work_j':{n:self.history.work_j.get(n,0.)+float(f@v)*dt for n,f in components.items()},
              'rider_control':{} if self.rider_control is None else self.rider_control.last_terms,
              'rider_allocation':{} if self.rider_control is None else {
                  k: (tuple(float(x) for x in v) if isinstance(v, np.ndarray)
                      else v)
                  for k, v in self.rider_control.allocation_diagnostics.items()
                  if not k.startswith('solution_')},
              'rider_ik_saturation':{} if self.rider_control is None else self.rider_control.saturated_ik,
              'rider_support_targets':{} if self.rider_control is None else self.rider_control.support_diagnostics,
              'contact_crash_cause':contact_crash}
    self.model_status.observe(sim.steps,t,channels)
    channels['model_status']=self.model_status.as_dict()
    # Preserve original scalar-channel snapshotting after P4 transfers runtime
    # ownership instead of freezing channels in PhysicalSample itself.
    self.sample=PhysicalSample(sim.steps,t,float(d.time),q,v,components,freeze(channels))
    self.history.add(self.sample)
    sim.steps+=1
    if contact_crash is not None and sim.crash_detector.event is None:
        from bike_sim.sim.ride.virtual_rider import CrashEvent
        sim.crash_detector.event=CrashEvent(contact_crash,t,float(q[sim.root_x_qposadr]),float(q[sim.root_pitch_qposadr]))
    sim.crash_detector.check(d,sim.contacts)
    sim._update_compiled_com_marker()
