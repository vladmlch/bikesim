"""Torque-sensing mid-drive assist: support factor through a first-order lag,
bounded by instantaneous shaft limits and gated as a binary permission.

With a profile (``profile='bosch_cx_gen4'``) the numbers are the recalled,
unverified Bosch CX Gen 4 set from ``motor_profile``; without one the plain
``gain`` and explicit limits form a synthetic test motor.

Order per step: demand = gain(mode, human) x human  ->  first-order lag (tau)
and slew  ->  min with the instantaneous ceiling (peak torque, torque curve
over shaft rpm, peak power / omega, speed taper)  ->  min with the pedelec
permission (0 when braking, crank not turning forward past the gate, or no
measured rider torque above ``engage_torque_nm``; the external request is a
ceiling). A zero permission also zeroes the lag state so that a restored
permission ramps up from zero instead of restoring a stale torque.

There is no stall timer, no boost hold and no stop delay: a real Gen 4 keeps
assisting as long as the cranks turn forward under load; thermal derating is
out of scope.
"""
from math import expm1, pi, radians
import numpy as np
from bike_sim.physics.checks import array, scalar
from bike_sim.physics.motor_profile import PROFILES, MotorProfile, assist_gain, pedelec_cap


class AssistController:
    def __init__(self,*,gain=2.,max_torque=80.,max_power=500.,tau=.05,slew=400.,
                 engage_torque_nm=4.,gate_min_crank_rad_s=radians(5.),
                 cutoff_mps=25/3.6,taper_width_mps=2/3.6,torque_curve=None,
                 profile=None,mode='turbo'):
        self.profile = None
        if profile is not None:
            if isinstance(profile, MotorProfile):
                self.profile = profile
            elif isinstance(profile, str) and profile in PROFILES:
                self.profile = PROFILES[profile]
            else:
                raise ValueError(f'unknown motor profile {profile!r}')
            p = self.profile
            max_torque,max_power,tau = p.peak_torque_nm,p.peak_power_w,p.torque_tau_s
            cutoff_mps,taper_width_mps = p.cutoff_mps,p.taper_width_mps
            gate_min_crank_rad_s = p.gate_min_crank_rad_s
            if not isinstance(mode, str) or mode not in p.mode_gains:
                raise ValueError(f'unknown assist mode {mode!r} for profile {p.name}')
        if not isinstance(mode, str):
            raise ValueError('assist mode must be a string')
        self.mode = mode
        for name,value in (('gain',gain),('max_torque',max_torque),('max_power',max_power),
                           ('engage_torque_nm',engage_torque_nm),
                           ('gate_min_crank_rad_s',gate_min_crank_rad_s),('cutoff_mps',cutoff_mps)):
            scalar(value,name,minimum=0)
        tau = scalar(tau, 'assist lag', positive=True)
        slew = scalar(slew, 'assist slew', positive=True)
        taper_width_mps = scalar(taper_width_mps, 'assist taper width', positive=True)
        if taper_width_mps > cutoff_mps:
            raise ValueError('speed taper exceeds the cutoff speed')
        if self.profile is not None:
            # The declared 40 ms lag must dominate: a 400 N.m/s slew would turn
            # the first-order response into a 170 ms ramp to 68 N.m.
            slew = max(slew, self.profile.peak_torque_nm/self.profile.torque_tau_s)
        self.gain,self.max_torque,self.max_power = gain,max_torque,max_power
        self.tau,self.slew,self.engage_torque_nm = tau,slew,engage_torque_nm
        self.gate_min_crank_rad_s = gate_min_crank_rad_s
        self.cutoff,self.width = cutoff_mps,taper_width_mps
        self.torque_curve = None
        if torque_curve is not None:
            curve = array(torque_curve,'torque curve',readonly=True)
            if curve.ndim != 2 or curve.shape[1] != 2 or len(curve) < 2:
                raise ValueError('torque curve needs at least two rpm/torque points')
            if np.any(curve < 0) or np.any(np.diff(curve[:,0]) <= 0):
                raise ValueError('torque curve rpm must increase strictly; limits must be nonnegative')
            self.torque_curve = curve
        self.reset()

    def reset(self):
        self.torque = 0.
        self.pedaling = False
        self.last_gain = 0.

    def ceiling(self, shaft_rpm, speed_mps):
        """Instantaneous shaft limit: peak, curve over shaft rpm, power, speed taper."""
        omega = shaft_rpm*2*pi/60
        ceiling = self.max_torque
        if self.torque_curve is not None:
            ceiling = min(ceiling,float(np.interp(shaft_rpm,self.torque_curve[:,0],self.torque_curve[:,1])))
        if omega > 0:
            ceiling = min(ceiling,self.max_power/omega)
        taper = max(0.,min(1.,(self.cutoff-abs(speed_mps))/self.width))
        return ceiling*taper, taper

    def step(self,human_nm,cadence_rpm,speed_mps,braking,dt, *,
             torque_request_nm=None,shaft_rpm=None):
        human = scalar(human_nm,'human torque')
        rpm = scalar(cadence_rpm,'cadence')
        speed = scalar(speed_mps,'road speed')
        dt = scalar(dt,'assist timestep',positive=True)
        if not isinstance(braking,(bool,np.bool_)):
            raise ValueError('braking must be a bool')
        if torque_request_nm is not None:
            torque_request_nm = scalar(torque_request_nm,'motor setpoint',minimum=0.)
        # Rigid crank/chainring: the shaft is the crank. The legacy crank-side
        # clutch passes its own shaft rpm for the ceiling and power accounting.
        shaft = rpm if shaft_rpm is None else scalar(shaft_rpm,'motor shaft rpm')
        if braking:
            self.reset()
            return 0.
        sensed = human if human > self.engage_torque_nm else 0.
        gain = assist_gain(self.profile,self.mode,sensed) if self.profile is not None else self.gain
        self.last_gain = gain
        ceiling,taper = self.ceiling(shaft,speed)
        # Support itself fades toward the cutoff, not only the hard ceiling.
        target = min(gain*sensed*taper,ceiling)
        candidate = self.torque-expm1(-dt/self.tau)*(target-self.torque)
        candidate = max(self.torque-self.slew*dt,min(self.torque+self.slew*dt,candidate))
        cap = pedelec_cap(sensed,rpm*2*pi/60,torque_request_nm,braking=bool(braking),
                          gate_min_crank_rad_s=self.gate_min_crank_rad_s)
        torque = scalar(max(0.,min(candidate,ceiling,cap)),'delivered assist torque')
        self.torque = torque
        self.pedaling = torque > 0.
        return torque
