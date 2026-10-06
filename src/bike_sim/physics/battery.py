"""Energy-limited shaft torque and synthetic electrical losses (joules/watts).

Limit torque BEFORE applying it. Then draw only the power of that delivered
command. An insufficient idle budget disables the motor without erasing the
nonzero battery remainder. There is no implicit regenerative braking.
"""
from math import sqrt
from bike_sim.physics.checks import derived, scalar


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
        requested = derived(power*dt, 'Battery.requested_energy')
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
    return derived(max(derived(torque*omega, 'motor_electrical_power.mechanical_power'),0.)+a*torque**2+b*omega**2+idle, 'motor_electrical_power.power')


def limit_torque_by_energy(request,omega,a,b,idle,budget_w):
    request = scalar(request,'requested motor torque',minimum=0)
    omega = scalar(omega,'shaft speed')
    a,b,idle,budget = (scalar(v,n,minimum=0) for v,n in zip(
        (a,b,idle,budget_w),('copper coefficient','speed loss coefficient','idle loss','power budget')))
    overhead = derived(b*omega**2+idle, 'limit_torque_by_energy.overhead')
    available = budget-overhead
    if available <= 0:
        return 0.
    w = max(omega,0.)
    if a > 0:
        discriminant = derived(w*w+4*a*available, 'limit_torque_by_energy.discriminant')
        numerator = derived(2*available, 'limit_torque_by_energy.numerator')
        denominator = derived(w+sqrt(discriminant), 'limit_torque_by_energy.denominator')
        cap = derived(numerator/denominator, 'limit_torque_by_energy.cap')
    elif w > 0:
        cap = derived(available/w, 'limit_torque_by_energy.cap')
    else:
        cap = request
    return min(request,cap)
