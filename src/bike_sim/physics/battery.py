"""Energy-limited shaft torque and synthetic electrical losses (joules/watts).

Limit torque BEFORE applying it. Then draw only the power of that delivered
command. An insufficient idle budget disables the motor without erasing the
nonzero battery remainder. There is no implicit regenerative braking.
"""
from math import sqrt
from bike_sim.physics.checks import scalar


class Battery:
    def __init__(self,energy_j):
        self.initial_energy_j = scalar(energy_j,'battery energy',minimum=0)
        self.energy_j = self.initial_energy_j
        self.drawn_energy_j = 0.

    def reset(self):
        self.energy_j = self.initial_energy_j
        self.drawn_energy_j = 0.

    def draw(self,requested_power_w,dt):
        power = scalar(requested_power_w,'battery power',minimum=0)
        dt = scalar(dt,'battery timestep',positive=True)
        requested = scalar(power*dt,'requested battery energy',minimum=0)
        delivered = min(self.energy_j,requested)
        self.energy_j = max(0.,self.energy_j-delivered)
        self.drawn_energy_j += delivered
        return delivered/dt


def motor_electrical_power(torque,omega,a,b,idle,enabled):
    torque,omega = scalar(torque,'shaft torque'),scalar(omega,'shaft speed')
    a,b,idle = (scalar(v,n,minimum=0) for v,n in zip((a,b,idle),('copper coefficient','speed loss coefficient','idle loss')))
    if not isinstance(enabled,bool):
        raise ValueError('motor enable must be a bool')
    if not enabled:
        return 0.
    return scalar(max(torque*omega,0.)+a*torque**2+b*omega**2+idle,'electrical power')


def limit_torque_by_energy(request,omega,a,b,idle,budget_w):
    request = scalar(request,'requested motor torque',minimum=0)
    omega = scalar(omega,'shaft speed')
    a,b,idle,budget = (scalar(v,n,minimum=0) for v,n in zip(
        (a,b,idle,budget_w),('copper coefficient','speed loss coefficient','idle loss','power budget')))
    overhead = scalar(b*omega**2+idle,'motor overhead')
    available = budget-overhead
    if available <= 0:
        return 0.
    w = max(omega,0.)
    if a > 0:
        discriminant = scalar(w*w+4*a*available,'torque budget discriminant')
        cap = 2*available/(w+sqrt(discriminant))
    elif w > 0:
        cap = available/w
    else:
        cap = request
    return min(request,cap)
