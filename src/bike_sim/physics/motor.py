"""Torque/cadence assist with brake priority and instantaneous shaft limits.

All defaults are a synthetic test profile, not a measured motor or a legal
classification. The optional torque curve is piecewise linear in rpm; its end
values are held outside its specified range, an explicit synthetic convention.

Engagement follows the torque sensor, not crank rotation: a rider pressing a
pedal at standstill (hill start) is assisted immediately, like torque-sensing
mid-drives. Two protections bound that freedom:

- `stall_timeout_s` caps sustained torque while the shaft is not turning
  (|rpm| < `spin_rpm`), a thermal/antihold guard. The latch clears once the
  shaft rotates again or the rider releases.
- `boost_s` is an extended-boost overrun: while the cranks keep turning
  forward the motor holds the last requested target briefly after the rider
  torque drops, which carries the wheel over ledges between pedal strokes.

Rolling backwards (`rpm < 0`) no longer locks the controller out: rider torque
is still sensed, and motor torque on the back-driven crank brakes the rollback
through the engaged drivetrain. With no rider torque there is still no assist.
"""
from math import expm1, pi
import numpy as np
from bike_sim.physics.checks import array, scalar


class AssistController:
    def __init__(self,*,gain=2.,max_torque=80.,max_power=500.,tau=.05,
                 slew=400.,stop_delay=.2,engage_torque_nm=4.,spin_rpm=15.,
                 stall_timeout_s=1.,boost_s=.4,
                 cutoff_mps=25/3.6,taper_width_mps=2/3.6,torque_curve=None):
        values = {}
        for name,value in locals().copy().items():
            if name not in ('self','torque_curve','values'):
                values[name] = scalar(value,name,minimum=0)
        if tau <= 0 or slew <= 0 or taper_width_mps <= 0 or spin_rpm <= 0:
            raise ValueError('invalid assist time constants or thresholds')
        if taper_width_mps > cutoff_mps:
            raise ValueError('speed taper exceeds the cutoff speed')
        self.gain,self.max_torque,self.max_power = gain,max_torque,max_power
        self.tau,self.slew,self.stop_delay = tau,slew,stop_delay
        self.engage_torque_nm,self.spin_rpm = engage_torque_nm,spin_rpm
        self.stall_timeout_s,self.boost_s = stall_timeout_s,boost_s
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
        self.age = float('inf')
        self.stall_s = 0.
        self.stalled = False
        self._boost_target = None

    def step(self,human_nm,cadence_rpm,speed_mps,braking,dt, *, torque_request_nm=None):
        human = scalar(human_nm,'human torque')
        rpm = scalar(cadence_rpm,'cadence')
        speed = scalar(speed_mps,'road speed')
        dt = scalar(dt,'assist timestep',positive=True)
        if not isinstance(braking,(bool,np.bool_)):
            raise ValueError('braking must be a bool')
        if torque_request_nm is not None:
            torque_request_nm = scalar(torque_request_nm, 'motor setpoint', minimum=0.)
        if braking:
            self.reset()
            return 0.
        pressing = torque_request_nm is not None or human > self.engage_torque_nm
        age = 0. if pressing else self.age+dt
        # Extended boost also defers release: as long as the cranks keep
        # turning forward the pedal stroke is treated as ongoing.
        boosting = (not pressing and self._boost_target is not None
                    and age < self.boost_s and rpm >= self.spin_rpm)
        if pressing:
            self.pedaling = True
        elif age >= self.stop_delay and not boosting:
            self.pedaling,self.age,self.torque = False,age,0.
            self.stall_s,self.stalled,self._boost_target = 0.,False,None
            return 0.
        if abs(rpm) >= self.spin_rpm:
            self.stall_s = 0.
            self.stalled = False
        elif pressing:
            self.stall_s += dt
            if self.stall_s > self.stall_timeout_s:
                self.stalled = True
        omega = rpm*2*pi/60
        ceiling = self.max_torque
        if self.torque_curve is not None:
            ceiling = min(ceiling,float(np.interp(rpm,self.torque_curve[:,0],self.torque_curve[:,1])))
        if omega > 0:
            ceiling = min(ceiling,self.max_power/omega)
        taper = max(0.,min(1.,(self.cutoff-abs(speed))/self.width))
        ceiling *= taper
        if self.stalled:
            target = 0.
        else:
            demand = self.gain*max(human,0.) if torque_request_nm is None else torque_request_nm
            target = min(demand*taper,ceiling)
            if pressing:
                self._boost_target = target
            elif boosting:
                target = self._boost_target
        candidate = self.torque-expm1(-dt/self.tau)*(target-self.torque)
        candidate = max(self.torque-self.slew*dt,min(self.torque+self.slew*dt,candidate))
        torque = scalar(max(0.,min(candidate,ceiling)),'delivered assist torque')
        self.pedaling,self.age,self.torque = self.pedaling,age,torque
        return torque
