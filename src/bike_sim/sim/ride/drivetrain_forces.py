"""Physical drivetrain paths and independently metered crank sources."""
import copy
from dataclasses import asdict, replace
from math import atan2, pi
import numpy as np
from bike_sim.physics.chain import chain_extension, chain_geometry, chain_jacobian, chain_tension, chain_center_gradient
from bike_sim.physics.freehub import Freehub
from bike_sim.physics.motor import AssistController
from bike_sim.physics.pedaling import human_crank_torque, PedalingPolicy
from bike_sim.physics.shifting import CadenceShifter
from bike_sim.physics.battery import Battery, limit_torque_by_energy, motor_electrical_power
from bike_sim.physics.checks import scalar
from bike_sim.sim.ride.physical_mapping import resolve_id
from bike_sim.sim.ride.ideal_freehub import IdealFreehubConstraint


def _unwrap(value, reference):
    return reference + atan2(np.sin(value-reference), np.cos(value-reference))


class DrivetrainForceApplier:
    """Stateful force producer; finite-difference probes never mutate live data."""
    def __init__(self, model, config, drive_mode):
        import mujoco
        if model.nq != model.nv:
            raise ValueError('physical chain supports scalar planar coordinates only (nq == nv)')
        self.config, self.drive_mode = config, drive_mode
        self.simplified = config.transmission_model in ('ideal_mid_drive','geometric_ideal_mid_drive')
        self.motor_clutch = bool(config.motor_clutch)
        self.rotor = config.rotor_inertia_kgm2 > 0.
        self.scratch = mujoco.MjData(model)
        self.ids = {name:resolve_id(model, mujoco.mjtObj.mjOBJ_BODY, name)
                    for name in ('frame','crank','rear_wheel')}
        if not self.simplified:
            self.ids['cassette']=resolve_id(model, mujoco.mjtObj.mjOBJ_BODY, 'cassette')
            if model.body_parentid[self.ids['cassette']] != model.body_parentid[self.ids['rear_wheel']]:
                raise ValueError('cassette and wheel must have the same carrier body')
        self.joints = {}
        joint_names=('crank_spin','rear_wheel_spin','front_wheel_spin',
                     'pedal_front_spin','pedal_rear_spin')
        if not self.simplified:
            joint_names=('crank_spin','cassette_spin','rear_wheel_spin','front_wheel_spin',
                         'pedal_front_spin','pedal_rear_spin')
        if self.motor_clutch:
            joint_names += ('drive_shaft_spin',)
        if self.rotor:
            joint_names += ('rotor_spin',)
        for name in joint_names:
            jid = resolve_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.joints[name] = (int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid]))
        self.actuators = {name:int(mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR,name))
                          for name in ('human_crank','mid_drive')}
        if drive_mode == 'articulated_effort' and self.actuators['human_crank'] >= 0:
            raise ValueError('articulated effort must not have a human_crank actuator')
        if drive_mode in ('crank_effort','articulated_effort') and self.actuators['mid_drive'] < 0:
            raise ValueError('physical effort needs a mid_drive actuator')
        self.hub = None if self.simplified else Freehub(config.freehub_k_nm_rad, config.freehub_c_nms_rad)
        ratio = config.gearing.front_teeth / config.gearing.rear_teeth
        self.ideal_hub = None
        self.clutch = None
        self.freewheel = None
        if self.simplified and drive_mode in ('crank_effort','articulated_effort'):
            driver = 'drive_shaft_spin' if self.motor_clutch else 'crank_spin'
            if config.transmission_model == 'geometric_ideal_mid_drive':
                from bike_sim.sim.ride.geometric_freehub import GeometricFreehubConstraint
                self.ideal_hub = GeometricFreehubConstraint(model,config.gearing,
                    driver=driver, driver_body='drive_shaft' if self.motor_clutch else 'crank')
            else:
                self.ideal_hub = IdealFreehubConstraint(model,ratio,driver=driver)
            if self.motor_clutch:
                self.clutch = IdealFreehubConstraint(model, 1.,
                    tendon_name='crank_clutch', driver='crank_spin', driven='drive_shaft_spin')
            if self.rotor:
                self.freewheel = IdealFreehubConstraint(model, 1.,
                    tendon_name='motor_freewheel',driver='rotor_spin',driven='crank_spin')
        self.pedaling = PedalingPolicy(config.pedaling)
        self.shifting = CadenceShifter(config.gearing, config.shifting)
        self.shift_time_s = None
        self.shift_motor_limit_nm = None
        self.assist = AssistController(**asdict(config.assist))
        self.battery = Battery(config.battery.energy_j)
        self.last_time_s = None
        self.reference = None
        self.last = {}
        self.pending_actuation = None

    def _angle(self, data, name, reference=None):
        R = data.xmat[self.ids[name]].reshape(3, 3)
        value = atan2(-R[2, 0], R[0, 0])
        return value if reference is None else _unwrap(value, reference)

    def _geometry(self, data, theta_reference, psi_reference):
        cf = data.xpos[self.ids['crank']][[0, 2]]
        cr = data.xpos[self.ids['cassette']][[0, 2]]
        up = data.xmat[self.ids['frame']].reshape(3,3)[[0,2],2]
        theta_f = self._angle(data, 'crank', theta_reference[0])
        theta_r = self._angle(data, 'cassette', theta_reference[1])
        rf, rr = self.config.gearing.front_radius_m, self.config.gearing.rear_radius_m
        return cf, cr, rf, rr, theta_f, theta_r, up

    def jacobian(self,model,data,*,psi_reference=None):
        """Analytic centre gradient mapped through engine body Jacobians.

        The full finite-difference implementation remains an independent test
        oracle. World rotational Jacobians include carrier and frame reactions.
        """
        import mujoco
        cf,cr,rf,rr,tf,tr,up=self._geometry(data,self.angles,self.psi)
        gradient=chain_center_gradient(cf,cr,rf,rr,up_xz=up,
                                       psi_reference=self.psi if psi_reference is None else psi_reference)
        jp_f,jr_f=np.zeros((3,model.nv)),np.zeros((3,model.nv))
        jp_r,jr_r=np.zeros((3,model.nv)),np.zeros((3,model.nv))
        mujoco.mj_jac(model,data,jp_f,jr_f,data.xpos[self.ids['crank']],self.ids['crank'])
        mujoco.mj_jac(model,data,jp_r,jr_r,data.xpos[self.ids['cassette']],self.ids['cassette'])
        return gradient@(jp_r[[0,2]]-jp_f[[0,2]])+rf*jr_f[1]-rr*jr_r[1]

    def finite_difference_jacobian(self,model,data):
        """Read-only oracle; perturb only scratch data with a frozen angle branch."""
        import mujoco
        angles=(self._angle(data,'crank',self.angles[0]),self._angle(data,'cassette',self.angles[1]))
        psi=self.psi
        def evaluate(q):
            self.scratch.qpos[:]=q
            mujoco.mj_kinematics(model,self.scratch)
            cf,cr,rf,rr,tf,tr,up=self._geometry(self.scratch,angles,psi)
            return chain_extension(cf,cr,rf,rr,tf,tr,self.reference,up_xz=up,psi_reference=psi)
        return chain_jacobian(data.qpos,evaluate)

    def reset(self, model, data):
        """Reset drivetrain state after initial crank/rider posing."""
        import mujoco
        mujoco.mj_kinematics(model, data)
        self.shifting.reset()
        self.shift_time_s = None
        self.shift_motor_limit_nm = None
        if self.simplified:
            self.angles = (self._angle(data,'crank'),)
            self.reference = 0.
            self.psi = None
            if self.ideal_hub is not None:
                self.ideal_hub.set_ratio(model, data, self.shifting.gear_ratio)
                self.ideal_hub.reset(model, data)
            if self.clutch is not None:
                self.clutch.reset(model, data)
            if self.freewheel is not None:
                self.freewheel.reset(model, data)
        else:
            self.angles = (self._angle(data,'crank'), self._angle(data,'cassette'))
            cf, cr, rf, rr, tf, tr, up = self._geometry(data, self.angles, None)
            _, self.psi = chain_geometry(cf, cr, rf, rr, up_xz=up)
            self.reference = chain_extension(cf,cr,rf,rr,tf,tr,0.,up_xz=up,psi_reference=self.psi)
            self.hub.reset()
        self.assist.reset(); self.battery.reset()
        self.pedaling.reset()
        self.pending_actuation = None
        self.last_time_s = None
        self.last = {'chain_energy_j':0., 'freehub_energy_j':0., 'motor_torque_nm':0.,
                     'human_torque_nm':0., 'electrical_power_w':0., 'freehub_torque_nm':0.,
                     'motor_freewheel_engaged':False, 'motor_freewheel_torque_nm':0.,
                     'motor_freewheel_dissipation_power_w':0.}
        self.last.update(self._shift_diagnostics())

    def restart_clock(self):
        self.last_time_s = None
        self.assist.reset()
        self.pedaling.reset()

    def _shift_diagnostics(self):
        return {
            'gear_front_teeth':self.config.gearing.front_teeth,
            'gear_rear_teeth':self.shifting.rear_teeth,
            'gear_ratio':self.shifting.gear_ratio,
            'shift_active':self.shifting.cut_remaining_s > 0.,
            'shift_direction':self.shifting.direction,
            'shift_from_teeth':self.shifting.from_teeth,
            'shift_count':self.shifting.shift_count,
            'shift_time_s':self.shift_time_s,
            'shift_torque_factor':self.shifting.torque_factor,
            'shift_motor_limit_nm':(self.shift_motor_limit_nm
                                    if self.shifting.torque_factor < 1. else None),
        }

    def prepare_pedaling(self, data, dt, control, *, active=True, advance=True, braking=False,
                         model=None, rear_in_contact=True, rear_slip_mps=None, effort_ceiling_nm=None):
        if advance and self.last_time_s is not None and float(data.time) <= self.last_time_s:
            raise ValueError('drivetrain state can advance only once per timestamp')
        crank_qpos, crank_dof = self.joints['crank_spin']
        _, wheel_dof = self.joints['rear_wheel_spin']
        required = float(data.qvel[wheel_dof]) / self.shifting.gear_ratio * 60. / (2. * pi)
        effort = self.config.human_torque_nm if control.human_torque_nm is None else control.human_torque_nm
        enabled = active and control.rider_enabled and self.drive_mode in ('crank_effort', 'articulated_effort')
        if (advance and active and self.config.shifting.enabled
                and self.drive_mode in ('crank_effort', 'articulated_effort')):
            if model is None or self.ideal_hub is None:
                raise ValueError('automatic shifting needs the live ideal_mid_drive model')
            shifted = self.shifting.update(float(data.qvel[crank_dof]) * 60. / (2. * pi),
                required, dt, pedaling=enabled and effort > 0., braking=braking,
                rear_in_contact=rear_in_contact, rear_slip_mps=rear_slip_mps)
            if shifted:
                self.ideal_hub.set_ratio(model, data, self.shifting.gear_ratio)
                self.shift_time_s = float(data.time)
                self.shift_motor_limit_nm = self.assist.torque * self.config.shifting.torque_factor
            required = float(data.qvel[wheel_dof]) / self.shifting.gear_ratio * 60. / (2. * pi)
        policy = self.pedaling if advance else copy.deepcopy(self.pedaling)
        state = policy.update(float(data.qpos[crank_qpos]), float(data.qvel[crank_dof]),
            required, effort, dt, braking=braking,
            enabled=enabled, reposition=control.crank_reposition)
        if effort_ceiling_nm is not None:
            ceiling = scalar(effort_ceiling_nm, 'automatic rider effort ceiling', minimum=0.)
            state = replace(state, effort_nm=min(state.effort_nm, ceiling))
        return replace(state, effort_nm=state.effort_nm * self.shifting.torque_factor)

    def stored_energy(self, model, data):
        if self.simplified:
            return {}
        cf, cr, rf, rr, tf, tr, up = self._geometry(data, self.angles, self.psi)
        e = chain_extension(cf, cr, rf, rr, tf, tr, self.reference, up_xz=up, psi_reference=self.psi)
        qc, _ = self.joints['cassette_spin']; qw, _ = self.joints['rear_wheel_spin']
        phi = float(data.qpos[qc]-data.qpos[qw])
        boundary = phi if self.hub.boundary is None else self.hub.boundary
        return {'chain': .5*self.config.chain_k_n_m*max(e, 0.)**2,
                'freehub': .5*self.hub.k*max(phi-boundary, 0.)**2}

    def settle_actuation(self, model, data):
        """Debit only delivered, solved shaft effort at the saved input velocity."""
        transmission = (np.zeros(model.nv) if self.ideal_hub is None
                        else self.ideal_hub.solved_qfrc(model, data))
        if self.clutch is not None:
            clutch_force = self.clutch.solved_qfrc(model, data)
            transmission = transmission + clutch_force
            clutch_torque = float(clutch_force[self.clutch.driven_dof])
            # A one-sided catch is inelastic: force x overrun rate is
            # dissipated by the solver and must be debited as loss.
            self.last.update(
                crank_clutch_torque_nm=clutch_torque,
                crank_clutch_engaged=bool(clutch_torque > 1e-8),
                crank_clutch_dissipation_power_w=max(0., clutch_torque
                                                     * self.clutch.relative_rate(data)))
        if self.freewheel is not None:
            freewheel_force = self.freewheel.solved_qfrc(model, data)
            transmission = transmission + freewheel_force
            freewheel_torque = float(freewheel_force[self.freewheel.driven_dof])
            self.last.update(
                motor_freewheel_torque_nm=freewheel_torque,
                motor_freewheel_engaged=bool(freewheel_torque > 1e-8),
                motor_freewheel_dissipation_power_w=max(
                    0., freewheel_torque*self.freewheel.relative_rate(data)))
        if self.ideal_hub is not None:
            torque = float(transmission[self.ideal_hub.driven_dof])
            self.last.update(freehub_torque_nm=torque, freehub_engaged=torque > 1e-8)
            if self.motor_clutch:
                self.last['freehub_dissipation_power_w'] = max(
                    0., torque * self.ideal_hub.relative_rate(data))
            self.last.update(getattr(self.ideal_hub,'diagnostics',{}))
        if self.pending_actuation is None:
            return transmission
        requested, omega, dt, enabled = self.pending_actuation
        aid = self.actuators['mid_drive']
        torque = float(data.actuator_force[aid]) if aid >= 0 else 0.
        if torque < -1e-10 or torque > requested+1e-8:
            raise ArithmeticError('solved motor effort violates the reserved effort ceiling')
        torque = max(torque, 0.)
        cfg = self.config.battery
        power = motor_electrical_power(torque, omega, cfg.copper_w_per_nm2,
            cfg.speed_w_per_rad_s2, cfg.idle_w, enabled and torque > 0.)
        if cfg.enabled:
            if power*dt > self.battery.energy_j+max(1e-10, self.battery.energy_j*1e-12):
                raise ArithmeticError('solved motor energy exceeds available battery storage')
            delivered_power = self.battery.draw(power, dt)
        else:
            delivered_power = power
        self.last.update(motor_torque_nm=torque, motor_shaft_power_w=torque*omega,
                         electrical_power_w=delivered_power, battery_energy_j=self.battery.energy_j,
                         motor_enabled=enabled and torque > 0., battery_empty=self.battery.energy_j == 0.)
        self.assist.torque = torque
        self.pending_actuation = None
        return transmission

    def compute_components(self, model, data, dt, *, speed_mps, braking=False,
                           sensed_human_nm=0., active=True, advance=True, control=None,
                           pedaling_state=None):
        from bike_sim.sim.ride.control import RideControl
        control = RideControl() if control is None else control
        if not isinstance(control, RideControl):
            raise ValueError('expected an immutable RideControl')
        dt = scalar(dt,'drivetrain interval',positive=True)
        speed_mps = scalar(speed_mps,'bike speed')
        sensed_human_nm = scalar(sensed_human_nm,'measured pedal torque')
        if not isinstance(braking,bool) or not isinstance(active,bool):
            raise ValueError('drive enable and brake status must be booleans')
        if self.reference is None:
            raise RuntimeError('initialize the drivetrain before applying forces')
        time = float(data.time)
        if advance and self.last_time_s is not None and time <= self.last_time_s:
            raise ValueError('drivetrain state can advance only once per timestamp')
        if not advance:
            probe = copy.copy(self)
            probe.hub = copy.deepcopy(self.hub)
            probe.assist = copy.deepcopy(self.assist)
            probe.battery = copy.deepcopy(self.battery)
            probe.pedaling = copy.deepcopy(self.pedaling)
            probe.shifting = copy.deepcopy(self.shifting)
            probe.pending_actuation = None
            probe.last_time_s = None
            components = probe._compute_components(model, data, dt, speed_mps=speed_mps, braking=braking,
                sensed_human_nm=sensed_human_nm, active=active, advance=False,
                control=control, pedaling_state=pedaling_state)
            self.probe_last = dict(probe.last)
            return components
        return self._compute_components(model, data, dt, speed_mps=speed_mps, braking=braking,
            sensed_human_nm=sensed_human_nm, active=active, advance=True,
            control=control, pedaling_state=pedaling_state)

    def _compute_components(self, model, data, dt, *, speed_mps, braking,
                            sensed_human_nm, active, advance, control, pedaling_state):
        time = float(data.time)
        if advance:
            if self.ideal_hub is not None:
                self.ideal_hub.prepare(model, data)
            if self.clutch is not None:
                self.clutch.prepare(model, data)
            if self.freewheel is not None:
                self.freewheel.prepare(model, data)
        if pedaling_state is None:
            pedaling_state = self.prepare_pedaling(data, dt, control,
                active=active, advance=advance, braking=braking, model=model)
        qf, vf = self.joints['crank_spin']
        qw, vw = self.joints['rear_wheel_spin']
        human=0.
        chain_energy=extension=tension=extension_rate=0.
        torque=deflection=relative_rate=0.
        components={}
        if self.simplified:
            components.update(chain=np.zeros(model.nv), freehub=np.zeros(model.nv))
        else:
            angles = (self._angle(data,'crank',self.angles[0]),self._angle(data,'cassette',self.angles[1]))
            cf, cr, rf, rr, tf, tr, up = self._geometry(data,angles,self.psi)
            _, psi = chain_geometry(cf,cr,rf,rr,up_xz=up,psi_reference=self.psi)
            extension = chain_extension(cf,cr,rf,rr,tf,tr,self.reference,up_xz=up,psi_reference=psi)
            J = self.jacobian(model,data,psi_reference=psi)
            extension_rate = float(J @ data.qvel)
            tension, chain_energy = chain_tension(extension,extension_rate,
                                                   self.config.chain_k_n_m,self.config.chain_c_ns_m)
            components['chain']=-tension*J
            qc, vc = self.joints['cassette_spin']
            torque = self.hub.update(float(data.qpos[qc]),float(data.qpos[qw]),
                                     float(data.qvel[vc]),float(data.qvel[vw]))
            hub_force = np.zeros(model.nv); hub_force[vw] = torque; hub_force[vc] = -torque
            components['freehub'] = hub_force
        bearing = np.zeros(model.nv)
        for _, va in self.joints.values():
            bearing[va] = -self.config.bearing_c_nms_rad*data.qvel[va]
        components['drive_bearings'] = bearing
        # Legacy shaft or inertial rotor supplies the motor's own coordinate;
        # otherwise the motor shaft is the crank. Permission always uses crank.
        omega_crank = float(data.qvel[vf]); cadence = omega_crank*60/(2*pi)
        shaft = self.joints.get('drive_shaft_spin') or self.joints.get('rotor_spin')
        omega_shaft = omega_crank if shaft is None else float(data.qvel[shaft[1]])
        shaft_rpm = omega_shaft*60/(2*pi)
        mean_human = pedaling_state.effort_nm
        human = (human_crank_torque(mean_human,float(data.qpos[qf]),self.config.torque_ripple)
                 if active and self.drive_mode=='crank_effort' else 0.)
        sensor = human if self.drive_mode=='crank_effort' else sensed_human_nm
        assist_sensor = sensor if pedaling_state.mode == 'pedaling' else 0.
        request = (self.assist.step(assist_sensor,cadence,speed_mps,braking,dt,
                                    torque_request_nm=control.motor_torque_nm,shaft_rpm=shaft_rpm)
                   if active and self.drive_mode in ('crank_effort','articulated_effort') else 0.)
        battery_cfg = self.config.battery
        a,b,idle = battery_cfg.copper_w_per_nm2,battery_cfg.speed_w_per_rad_s2,battery_cfg.idle_w
        budget = self.battery.energy_j/dt
        safety_request = request if control.motor_limit_nm is None else min(request, control.motor_limit_nm)
        limited_request = safety_request
        if self.shifting.torque_factor < 1. and self.shift_motor_limit_nm is not None:
            limited_request = min(limited_request, self.shift_motor_limit_nm)
        delivered = (limit_torque_by_energy(limited_request,omega_shaft,a,b,idle,budget)
                     if battery_cfg.enabled else limited_request)
        enabled = active and delivered > 0. and not braking
        if not enabled:
            delivered = 0.
        electrical = motor_electrical_power(delivered,omega_shaft,a,b,idle,enabled)
        if battery_cfg.enabled and electrical > budget+max(1e-10,abs(budget)*1e-12):
            raise ArithmeticError('delivered motor torque exceeds the battery budget')
        actual_electrical = 0.  # settled from data.actuator_force after mj_step
        if active and self.pending_actuation is not None:
            raise RuntimeError('previous motor interval was not settled')
        self.pending_actuation = (delivered, omega_shaft, dt, enabled) if active else None
        for name,value in (('human_crank',human),('mid_drive',delivered)):
            aid = self.actuators[name]
            if aid >= 0:
                data.ctrl[aid] = value
        if self.simplified:
            components['ideal_transmission']=np.zeros(model.nv)
        if self.drive_mode in ('crank_effort','articulated_effort'):
            self.assist.torque = delivered
        if not self.simplified:
            relative_rate = float(data.qvel[vc]-data.qvel[vw])
            deflection = max(float(data.qpos[qc]-data.qpos[qw])-self.hub.boundary,0.)
        self.last = {
            'transmission_model':self.config.transmission_model,
            'omits_suspension_coupling':self.config.transmission_model=='ideal_mid_drive',
            'chain_extension_m':extension, 'chain_extension_rate_mps':0. if self.simplified else extension_rate,
            'chain_tension_n':tension, 'chain_energy_j':chain_energy,
            'chain_dissipation_power_w':max(0.,(tension-self.config.chain_k_n_m*max(extension,0.))*extension_rate),
            'freehub_torque_nm':0. if self.simplified else torque, 'freehub_energy_j':0. if self.simplified else self.hub.energy_j,
            'freehub_engaged':False if self.simplified else torque > 0., 'freehub_deflection_rad':0. if self.simplified else deflection,
            'freehub_dissipation_power_w':(0. if self.simplified else
                max(0.,(torque-self.hub.k*deflection)*relative_rate)),
            'cadence_rpm':cadence, 'crank_rad_s':omega_crank, 'drive_shaft_rad_s':omega_shaft,
            'human_torque_nm':human, 'human_sensor_nm':sensor,
            'human_setpoint_nm':control.human_torque_nm,
            'assist_demand_gated':bool(braking or assist_sensor<=self.assist.engage_torque_nm
                                       or omega_crank<=self.assist.gate_min_crank_rad_s),
            'assist_sensor_nm':assist_sensor, 'human_command_nm':mean_human,
            'rider_mode':pedaling_state.mode, 'coasting_reason':pedaling_state.reason,
            'required_cadence_rpm':pedaling_state.required_cadence_rpm,
            'crank_target_phase_rad':pedaling_state.target_phase_rad,
            'crank_target_rate_rad_s':pedaling_state.target_rate_rad_s,
            'motor_request_nm':request, 'motor_torque_nm':delivered,
            'motor_freewheel_engaged':delivered > 0., 'motor_freewheel_torque_nm':delivered,
            'motor_freewheel_dissipation_power_w':0.,
            'motor_limited_request_nm':limited_request,
            'motor_setpoint_nm':control.motor_torque_nm,
            'motor_limit_nm':control.motor_limit_nm,
            'motor_control_source':'assist' if control.motor_torque_nm is None else 'external_request',
            'safety_limited':safety_request < request,
            'shift_limited':limited_request < safety_request,
            'motor_shaft_power_w':delivered*omega_shaft, 'electrical_power_w':actual_electrical,
            'battery_energy_j':self.battery.energy_j, 'motor_enabled':enabled,
            'energy_limited':delivered < limited_request, 'battery_empty':self.battery.energy_j==0.,
            'assist_mode':self.assist.mode, 'assist_gain':self.assist.last_gain,
        }
        self.last.update(self._shift_diagnostics())
        if not self.simplified:
            self.angles,self.psi = angles,psi
        else:
            self.angles=(self._angle(data,'crank',self.angles[0]),)
        self.last_time_s = time
        return components
