"""Time-based active effort and a shared positive-power budget.

These helpers never filter passive damping and never alter mechanical state.
"""
import numpy as np


def activation_step(previous, target, dt_s: float, tau_s: float):
    previous,target=np.asarray(previous,float),np.asarray(target,float)
    if previous.ndim!=1 or previous.shape!=target.shape or not np.isfinite(np.r_[previous,target,dt_s,tau_s]).all():
        raise ValueError('invalid activation state')
    if dt_s<=0 or tau_s<0:
        raise ValueError('invalid activation time constants')
    return target.copy() if tau_s==0 else target+(previous-target)*np.exp(-dt_s/tau_s)


def limit_positive_power(torque, velocity, limit_w: float):
    torque,velocity=np.asarray(torque,float),np.asarray(velocity,float)
    if torque.ndim!=1 or torque.shape!=velocity.shape:
        raise ValueError('power budget vector mismatch')
    if not np.isfinite(np.r_[torque,velocity,limit_w]).all() or limit_w<0:
        raise ValueError('invalid power budget')
    result=torque.copy()
    positive=result*velocity>0
    power=float(np.sum(result[positive]*velocity[positive]))
    if power>limit_w:
        result[positive]*=limit_w/power
    return result
