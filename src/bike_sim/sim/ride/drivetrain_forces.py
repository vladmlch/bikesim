"""Physical chain/freehub path and independently metered crank sources."""
import copy
from dataclasses import asdict
from math import atan2, pi
import numpy as np
from bike_sim.physics.chain import chain_extension, chain_geometry, chain_jacobian, chain_tension
from bike_sim.physics.freehub import Freehub
from bike_sim.physics.motor import AssistController
from bike_sim.physics.pedaling import human_crank_torque
from bike_sim.physics.battery import Battery, limit_torque_by_energy, motor_electrical_power
from bike_sim.physics.checks import scalar
from bike_sim.sim.ride.physical_mapping import resolve_id


def _unwrap(value, reference):
    return reference + atan2(np.sin(value-reference), np.cos(value-reference))


class DrivetrainForceApplier:
    """Stateful force producer; finite-difference probes never mutate live data."""
    def __init__(self, model, config, drive_mode):
        import mujoco
        if model.nq != model.nv:
            raise ValueError('physical chain supports scalar planar coordinates only (nq == nv)')
        self.config, self.drive_mode = config, drive_mode
        self.scratch = mujoco.MjData(model)
        self.ids = {name:resolve_id(model, mujoco.mjtObj.mjOBJ_BODY, name)
                    for name in ('frame','crank','cassette','rear_wheel')}
        if model.body_parentid[self.ids['cassette']] != model.body_parentid[self.ids['rear_wheel']]:
            raise ValueError('cassette and wheel must have the same carrier body')
        self.joints = {}
        for name in ('crank_spin','cassette_spin','rear_wheel_spin','front_wheel_spin',
                     'pedal_front_spin','pedal_rear_spin'):
            jid = resolve_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            self.joints[name] = (int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid]))
        self.actuators = {name:int(mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR,name))
                          for name in ('human_crank','mid_drive')}
        if drive_mode == 'articulated_effort' and self.actuators['human_crank'] >= 0:
            raise ValueError('articulated effort must not have a human_crank actuator')
        if drive_mode in ('crank_effort','articulated_effort') and self.actuators['mid_drive'] < 0:
            raise ValueError('physical effort needs a mid_drive actuator')
        self.hub = Freehub(config.freehub_k_nm_rad, config.freehub_c_nms_rad)
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

    def reset(self, model, data):
        """Bind the stress-free chain datum after initial crank/rider posing."""
        import mujoco
        mujoco.mj_kinematics(model, data)
        self.angles = (self._angle(data,'crank'), self._angle(data,'cassette'))
        cf, cr, rf, rr, tf, tr, up = self._geometry(data, self.angles, None)
        _, self.psi = chain_geometry(cf, cr, rf, rr, up_xz=up)
        self.reference = chain_extension(cf,cr,rf,rr,tf,tr,0.,up_xz=up,psi_reference=self.psi)
        self.hub.reset(); self.assist.reset(); self.battery.reset()
        self.pending_actuation = None
        self.last_time_s = None
        self.last = {'chain_energy_j':0., 'freehub_energy_j':0., 'motor_torque_nm':0.,
                     'human_torque_nm':0., 'electrical_power_w':0., 'freehub_torque_nm':0.}

    def restart_clock(self):
        self.last_time_s = None
        self.assist.reset()

    def stored_energy(self, model, data):
        cf, cr, rf, rr, tf, tr, up = self._geometry(data, self.angles, self.psi)
        e = chain_extension(cf, cr, rf, rr, tf, tr, self.reference, up_xz=up, psi_reference=self.psi)
        qc, _ = self.joints['cassette_spin']; qw, _ = self.joints['rear_wheel_spin']
        phi = float(data.qpos[qc]-data.qpos[qw])
        boundary = phi if self.hub.boundary is None else self.hub.boundary
        return {'chain': .5*self.config.chain_k_n_m*max(e, 0.)**2,
                'freehub': .5*self.hub.k*max(phi-boundary, 0.)**2}

    def settle_actuation(self, model, data):
        """Debit only delivered, solved shaft effort at the saved input velocity."""
        if self.pending_actuation is None:
            return
        requested, omega, dt, enabled = self.pending_actuation
        aid = self.actuators['mid_drive']
        torque = float(data.actuator_force[aid]) if aid >= 0 else 0.
        if torque < -1e-10 or torque > requested+1e-8:
            raise ArithmeticError('solved motor effort violates the reserved effort ceiling')
        torque = max(torque, 0.)
        cfg = self.config.battery
        power = motor_electrical_power(torque, omega, cfg.copper_w_per_nm2,
            cfg.speed_w_per_rad_s2, cfg.idle_w, enabled and torque > 0.)
        if power*dt > self.battery.energy_j+max(1e-10, self.battery.energy_j*1e-12):
            raise ArithmeticError('solved motor energy exceeds available battery storage')
        delivered_power = self.battery.draw(power, dt)
        self.last.update(motor_torque_nm=torque, motor_shaft_power_w=torque*omega,
                         electrical_power_w=delivered_power, battery_energy_j=self.battery.energy_j,
                         motor_enabled=enabled and torque > 0., battery_empty=self.battery.energy_j == 0.)
        self.assist.torque = torque
        self.pending_actuation = None

    def compute_components(self, model, data, dt, *, speed_mps, braking=False,
                           sensed_human_nm=0., active=True, advance=True):
        import mujoco
        dt = scalar(dt,'drivetrain interval',positive=True)
        speed_mps = scalar(speed_mps,'bike speed')
        sensed_human_nm = scalar(sensed_human_nm,'measured pedal torque')
        if not isinstance(braking,bool) or not isinstance(active,bool):
            raise ValueError('drive enable and brake status must be booleans')
        if self.reference is None:
            raise RuntimeError('initialize the chain datum before applying forces')
        time = float(data.time)
        if advance and self.last_time_s is not None and time <= self.last_time_s:
            raise ValueError('drivetrain state can advance only once per timestamp')
        if not advance:
            probe = copy.copy(self)
            probe.hub = copy.deepcopy(self.hub)
            probe.assist = copy.deepcopy(self.assist)
            probe.battery = copy.deepcopy(self.battery)
            probe.last_time_s = None
            return probe.compute_components(model,data,dt,speed_mps=speed_mps,braking=braking,
                                            sensed_human_nm=sensed_human_nm,active=active)
        angles = (self._angle(data,'crank',self.angles[0]),self._angle(data,'cassette',self.angles[1]))
        cf, cr, rf, rr, tf, tr, up = self._geometry(data,angles,self.psi)
        _, psi = chain_geometry(cf,cr,rf,rr,up_xz=up,psi_reference=self.psi)
        extension = chain_extension(cf,cr,rf,rr,tf,tr,self.reference,up_xz=up,psi_reference=psi)

        def evaluate(q):
            self.scratch.qpos[:] = q
            # Pure forward kinematics, no live-state writes, filters, contacts or
            # control callbacks. Unwrap references are frozen for the entire J.
            mujoco.mj_kinematics(model,self.scratch)
            af, ar, r_f, r_r, t_f, t_r, up_q = self._geometry(self.scratch,angles,psi)
            return chain_extension(af,ar,r_f,r_r,t_f,t_r,self.reference,up_xz=up_q,psi_reference=psi)

        J = chain_jacobian(data.qpos,evaluate)
        extension_rate = float(J @ data.qvel)
        tension, chain_energy = chain_tension(extension,extension_rate,
                                               self.config.chain_k_n_m,self.config.chain_c_ns_m)
        components = {'chain':-tension*J}
        qc, vc = self.joints['cassette_spin']
        qw, vw = self.joints['rear_wheel_spin']
        torque = self.hub.update(float(data.qpos[qc]),float(data.qpos[qw]),
                                 float(data.qvel[vc]),float(data.qvel[vw]))
        hub_force = np.zeros(model.nv); hub_force[vw] = torque; hub_force[vc] = -torque
        components['freehub'] = hub_force
        bearing = np.zeros(model.nv)
        for _, va in self.joints.values():
            bearing[va] = -self.config.bearing_c_nms_rad*data.qvel[va]
        components['drive_bearings'] = bearing
        qf, vf = self.joints['crank_spin']
        omega = float(data.qvel[vf]); cadence = omega*60/(2*pi)
        human = (human_crank_torque(self.config.human_torque_nm,float(data.qpos[qf]),self.config.torque_ripple)
                 if active and self.drive_mode=='crank_effort' else 0.)
        sensor = human if self.drive_mode=='crank_effort' else sensed_human_nm
        request = (self.assist.step(sensor,cadence,speed_mps,braking,dt)
                   if active and self.drive_mode in ('crank_effort','articulated_effort') else 0.)
        battery_cfg = self.config.battery
        a,b,idle = battery_cfg.copper_w_per_nm2,battery_cfg.speed_w_per_rad_s2,battery_cfg.idle_w
        budget = self.battery.energy_j/dt
        delivered = limit_torque_by_energy(request,omega,a,b,idle,budget)
        enabled = active and delivered > 0. and not braking
        if not enabled:
            delivered = 0.
        electrical = motor_electrical_power(delivered,omega,a,b,idle,enabled)
        if electrical > budget+max(1e-10,abs(budget)*1e-12):
            raise ArithmeticError('delivered motor torque exceeds the battery budget')
        actual_electrical = 0.  # settled from data.actuator_force after mj_step
        if active and self.pending_actuation is not None:
            raise RuntimeError('previous motor interval was not settled')
        self.pending_actuation = (delivered, omega, dt, enabled) if active else None
        for name,value in (('human_crank',human),('mid_drive',delivered)):
            aid = self.actuators[name]
            if aid >= 0:
                data.ctrl[aid] = value
        if self.drive_mode in ('crank_effort','articulated_effort'):
            self.assist.torque = delivered
        relative_rate = float(data.qvel[vc]-data.qvel[vw])
        deflection = max(float(data.qpos[qc]-data.qpos[qw])-self.hub.boundary,0.)
        self.last = {
            'chain_extension_m':extension, 'chain_extension_rate_mps':extension_rate,
            'chain_tension_n':tension, 'chain_energy_j':chain_energy,
            'chain_dissipation_power_w':max(0.,(tension-self.config.chain_k_n_m*max(extension,0.))*extension_rate),
            'freehub_torque_nm':torque, 'freehub_energy_j':self.hub.energy_j,
            'freehub_dissipation_power_w':max(0.,(torque-self.hub.k*deflection)*relative_rate),
            'cadence_rpm':cadence, 'human_torque_nm':human, 'human_sensor_nm':sensor,
            'motor_request_nm':request, 'motor_torque_nm':delivered,
            'motor_shaft_power_w':delivered*omega, 'electrical_power_w':actual_electrical,
            'battery_energy_j':self.battery.energy_j, 'motor_enabled':enabled,
            'energy_limited':delivered < request, 'battery_empty':self.battery.energy_j==0.,
        }
        self.angles,self.psi,self.last_time_s = angles,psi,time
        return components
