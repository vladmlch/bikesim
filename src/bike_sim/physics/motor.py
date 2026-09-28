"""Cadence/torque assist with brake priority and instantaneous shaft limits.

All defaults are a synthetic test profile, not a measured motor or a legal
classification. The optional torque curve is piecewise linear in rpm; its end
values are held outside its specified range, an explicit synthetic convention.
"""
from math import expm1, pi
import numpy as np
from bike_sim.physics.checks import array, scalar


class AssistController:
    def __init__(self,*,gain=2.,max_torque=80.,max_power=500.,tau=.05,
                 slew=400.,stop_delay=.1,on_rpm=10.,off_rpm=5.,
                 cutoff_mps=25/3.6,taper_width_mps=2/3.6,torque_curve=None):
        values = {}
        for name,value in locals().copy().items():
            if name not in ('self','torque_curve','values'):
                values[name] = scalar(value,name,minimum=0)
        if tau <= 0 or slew <= 0 or taper_width_mps <= 0 or on_rpm <= off_rpm:
            raise ValueError('invalid assist time constants or thresholds')
        if taper_width_mps > cutoff_mps:
            raise ValueError('speed taper exceeds the cutoff speed')
        self.gain,self.max_torque,self.max_power = gain,max_torque,max_power
        self.tau,self.slew,self.stop_delay = tau,slew,stop_delay
        self.on_rpm,self.off_rpm = on_rpm,off_rpm
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

    def step(self,human_nm,cadence_rpm,speed_mps,braking,dt):
        human = scalar(human_nm,'human torque')
        rpm = scalar(cadence_rpm,'cadence')
        speed = scalar(speed_mps,'road speed')
        dt = scalar(dt,'assist timestep',positive=True)
        if not isinstance(braking,(bool,np.bool_)):
            raise ValueError('braking must be a bool')
        if braking or rpm < 0:
            self.reset()
            return 0.
        threshold = self.off_rpm if self.pedaling else self.on_rpm
        pedaling = human > 0 and rpm >= threshold
        age = 0. if pedaling else self.age+dt
        if age >= self.stop_delay and not pedaling:
            self.pedaling,self.age,self.torque = pedaling,age,0.
            return 0.
        omega = rpm*2*pi/60
        ceiling = self.max_torque
        if self.torque_curve is not None:
            ceiling = min(ceiling,float(np.interp(rpm,self.torque_curve[:,0],self.torque_curve[:,1])))
        if omega > 0:
            ceiling = min(ceiling,self.max_power/omega)
        taper = max(0.,min(1.,(self.cutoff-abs(speed))/self.width))
        ceiling *= taper
        target = min(self.gain*max(human,0.)*taper,ceiling)
        candidate = self.torque-expm1(-dt/self.tau)*(target-self.torque)
        candidate = max(self.torque-self.slew*dt,min(self.torque+self.slew*dt,candidate))
        torque = scalar(max(0.,min(candidate,ceiling)),'delivered assist torque')
        self.pedaling,self.age,self.torque = pedaling,age,torque
        return torque
