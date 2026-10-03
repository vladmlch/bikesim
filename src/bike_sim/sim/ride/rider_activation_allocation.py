"""Bounded excitation and the existing first-order active torque mechanics.

Excitation and latent activation state are torques in Nm. Excitation is bounded
by the current directional strength and actuator envelope. Delivered torque is
the first-order activation output after the physical per-joint and whole-body
positive-power clamps. The allocator solves this law, rather than bypassing it.
"""
import numpy as np
from scipy.optimize import NonlinearConstraint
from bike_sim.physics.rider_activation import activation_step,limit_positive_power


class ActivationLaw:
    def __init__(self,state,speeds,lower,upper,*,dt_s,tau_s,power_limit):
        self.state=np.asarray(state).copy()
        self.speeds=np.asarray(speeds).copy()
        self.lower=np.asarray(lower).copy();self.upper=np.asarray(upper).copy()
        self.dt_s=dt_s;self.tau_s=tau_s;self.power_limit=power_limit
        self.gain=1. if tau_s==0. else float(-np.expm1(-dt_s/tau_s))

    def latent(self,excitation):
        return activation_step(self.state,excitation,self.dt_s,self.tau_s)

    def delivered(self,excitation):
        active=np.clip(self.latent(excitation),self.lower,self.upper)
        return active if self.power_limit is None else limit_positive_power(active,self.speeds,self.power_limit)

    def derivative(self,excitation):
        latent=self.latent(excitation)
        active=np.clip(latent,self.lower,self.upper)
        jac=np.diag(((latent>=self.lower)&(latent<=self.upper))*self.gain)
        producing=active*self.speeds>0.
        power=float(np.maximum(active*self.speeds,0.).sum())
        if self.power_limit is not None and power>self.power_limit:
            indices=np.flatnonzero(producing)
            projection=np.eye(len(active))
            projection[np.ix_(indices,indices)]=(self.power_limit/power*np.eye(len(indices))
                -self.power_limit/power**2*np.outer(active[indices],self.speeds[indices]))
            jac=projection@jac
        return jac

    def constraint(self,torque_indices,excitation_indices,scales):
        ti=np.asarray(torque_indices);ei=np.asarray(excitation_indices);scales=np.asarray(scales)
        def value(x):return x[ti]*scales[ti]-self.delivered(x[ei]*scales[ei])
        def jacobian(x):
            result=np.zeros((len(ti),len(x)))
            result[np.arange(len(ti)),ti]=scales[ti]
            result[:,ei]=-self.derivative(x[ei]*scales[ei])*scales[ei]
            return result
        return NonlinearConstraint(value,0.,0.,jac=jacobian)

    def project(self,x,torque_indices,excitation_indices,scales):
        result=np.asarray(x).copy()
        result[torque_indices]=self.delivered(result[excitation_indices]*scales[excitation_indices])/scales[torque_indices]
        return result
