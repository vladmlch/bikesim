"""Passive zero-backlash synthetic ratchet between sibling cassette and wheel.

The torque returned acts positively on the wheel and negatively on the cassette.
A moving engagement boundary follows overrun, never a forced velocity reset.
"""
from bike_sim.physics.checks import derived, scalar


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
        scalar(self.k, "Freehub.stiffness", positive=True)
        scalar(self.c, "Freehub.damping", minimum=0.)
        pc,pw,wc,ww = (scalar(x,n) for x,n in zip(
            (phi_c,phi_w,omega_c,omega_w),
            ('cassette angle','wheel angle','cassette speed','wheel speed')))
        relative = derived(pc-pw, 'Freehub.relative_angle')
        boundary = relative if self.boundary is None else min(self.boundary,relative)
        deflection = max(0.,relative-boundary)
        energy = derived(.5*self.k*deflection**2, 'Freehub.energy')
        torque = max(0., derived(self.k*deflection+self.c*(wc-ww), 'Freehub.torque'))
        self.boundary,self.energy_j,self.torque_nm = boundary,energy,torque
        return torque
