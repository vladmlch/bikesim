"""External road rolling moment and planar aerodynamic force, in SI units.

The wheel speed is SIGNED ABSOLUTE angular speed about its world axis, not
relative hinge speed or the magnitude of speed. A caller maps the moment to the
wheel body using mj_applyFT; applying it only on the hinge loses root reaction.
"""
from math import tanh
import numpy as np
from bike_sim.physics.checks import array, scalar


def rolling_moment(crr, Fn, radius, omega_abs, taper):
    crr = scalar(crr, 'rolling coefficient', minimum=0)
    load = scalar(Fn, 'normal load', minimum=0)
    radius = scalar(radius, 'wheel radius', positive=True)
    speed = scalar(omega_abs, 'signed absolute wheel speed')
    taper = scalar(taper, 'rolling regularization speed', positive=True)
    return scalar(-crr*load*radius*tanh(speed/taper), 'rolling torque')


def drag_force(velocity_relative, rho, cda):
    v = array(velocity_relative, 'air-relative velocity', (3,))
    rho = scalar(rho, 'air density', minimum=0)
    cda = scalar(cda, 'drag area', minimum=0)
    if v[1] != 0:
        raise ValueError('drag velocity must be planar X-Z')
    with np.errstate(over='ignore', invalid='ignore'):
        force = -.5*rho*cda*np.linalg.norm(v)*v
    return array(force, 'aerodynamic force', (3,))
