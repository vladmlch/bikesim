"""Passive zero-backlash synthetic ratchet between sibling cassette and wheel.

The torque returned acts positively on the wheel and negatively on the cassette.
A moving engagement boundary follows overrun, never a forced velocity reset.
"""
from bike_sim.physics.checks import scalar


class Freehub:
    def __init__(self,stiffness_nm_rad,damping_nms_rad):
        self.k = scalar(stiffness_nm_rad,'freehub stiffness',positive=True)
        self.c = scalar(damping_nms_rad,'freehub damping',minimum=0)
        self.reset()

    def reset(self):
        self.boundary = None
        self.energy_j = 0.
        self.torque_nm = 0.

    def update(self,phi_c,phi_w,omega_c,omega_w):
        pc,pw,wc,ww = (scalar(x,n) for x,n in zip(
            (phi_c,phi_w,omega_c,omega_w),
            ('cassette angle','wheel angle','cassette speed','wheel speed')))
        relative = scalar(pc-pw,'freehub relative angle')
        boundary = relative if self.boundary is None else min(self.boundary,relative)
        deflection = max(0.,relative-boundary)
        energy = scalar(.5*self.k*deflection**2,'freehub energy')
        torque = scalar(max(0.,self.k*deflection+self.c*(wc-ww)),'freehub torque')
        self.boundary,self.energy_j,self.torque_nm = boundary,energy,torque
        return torque
