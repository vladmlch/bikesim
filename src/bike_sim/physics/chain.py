"""Unilateral chain extension in SI, including moving sprocket centres.

Angles are absolute, unwrapped rotations about world +Y. The tangent unwrap
reference must be held fixed while differentiating. Chain forces are -T*J; do
not add a second T*r wheel or crank torque after this mapping.
"""
from dataclasses import dataclass
from math import atan2, cos, pi, sin, sqrt
import numpy as np
from bike_sim.physics.checks import array, scalar


@dataclass(frozen=True)
class DrivetrainSpecs:
    front_teeth: int = 34
    rear_teeth: int = 24
    chain_pitch_m: float = .0127

    def __post_init__(self):
        if any(type(z) is not int or z < 3 for z in (self.front_teeth,self.rear_teeth)):
            raise ValueError('sprockets need integer tooth counts of at least three')
        object.__setattr__(self,'chain_pitch_m',scalar(self.chain_pitch_m,'chain pitch',positive=True))

    @property
    def front_radius_m(self):
        return self.chain_pitch_m*self.front_teeth/(2*pi)

    @property
    def rear_radius_m(self):
        return self.chain_pitch_m*self.rear_teeth/(2*pi)


def chain_geometry(cf,cr,rf,rr,*,up_xz=None,psi_reference=None):
    """Return selected tangent length plus arc term, and unwrapped tangent angle."""
    cf,cr = array(cf,'front centre',(2,)),array(cr,'rear centre',(2,))
    up = array([0.,1.] if up_xz is None else up_xz,'chain up',(2,))
    rf,rr = scalar(rf,'front radius',positive=True),scalar(rr,'rear radius',positive=True)
    if np.linalg.norm(up) == 0:
        raise ValueError('chain up cannot be zero')
    difference = rf-rr
    vector = cr-cf
    distance = float(np.linalg.norm(vector))
    if not np.isfinite(distance) or distance <= abs(difference):
        raise ValueError('sprockets have no valid external tangent')
    direction = vector/distance
    perpendicular = np.array([direction[1],-direction[0]])
    ratio = difference/distance
    term = sqrt(max(0.,1-ratio*ratio))*perpendicular
    first,second = ratio*direction+term,ratio*direction-term
    normal = first if first@up >= second@up else second
    psi = atan2(normal[1],normal[0])
    if psi_reference is not None:
        ref = scalar(psi_reference,'angular unwrap reference')
        psi = ref+atan2(sin(psi-ref),cos(psi-ref))
    length = sqrt(distance*distance-difference*difference)+difference*psi
    return scalar(length,'chain geometric coordinate'),psi


def chain_extension(cf,cr,rf,rr,theta_f,theta_r,reference,*,up_xz=None,psi_reference=None):
    theta_f = scalar(theta_f,'absolute crank angle')
    theta_r = scalar(theta_r,'absolute cassette angle')
    reference = scalar(reference,'chain reference')
    length,_ = chain_geometry(cf,cr,rf,rr,up_xz=up_xz,psi_reference=psi_reference)
    return scalar(length+rf*theta_f-rr*theta_r-reference,'chain extension')


def chain_jacobian(q,evaluate,epsilon=1e-7):
    q = array(q,'generalized coordinates')
    if q.ndim != 1:
        raise ValueError('generalized coordinates must be a vector')
    epsilon = scalar(epsilon,'difference step',positive=True)
    result = np.zeros_like(q)
    for i in range(len(q)):
        step = np.zeros_like(q)
        step[i] = epsilon
        plus = scalar(evaluate(q+step),'positive chain perturbation')
        minus = scalar(evaluate(q-step),'negative chain perturbation')
        result[i] = (plus-minus)/(2*epsilon)
    return array(result,'chain Jacobian')


def chain_tension(extension_m,extension_rate_mps,stiffness_n_m,damping_ns_m):
    e = scalar(extension_m,'chain extension')
    rate = scalar(extension_rate_mps,'chain extension rate')
    k = scalar(stiffness_n_m,'chain stiffness',positive=True)
    c = scalar(damping_ns_m,'chain damping',minimum=0)
    if e <= 0:
        return 0.,0.
    return scalar(max(0.,k*e+c*rate),'chain tension'), scalar(.5*k*e*e,'chain energy')


def chain_center_gradient(cf, cr, rf, rr, *, up_xz=None, psi_reference=None):
    """Derivative of the selected geometric branch with respect to rear centre.

    The front-centre derivative is its negative. The branch is held fixed, just
    as in the independent finite-difference oracle; the unwrap adds a constant.
    """
    cf,cr=array(cf,'front centre',(2,)),array(cr,'rear centre',(2,))
    _,psi=chain_geometry(cf,cr,rf,rr,up_xz=up_xz,psi_reference=psi_reference)
    a=cr-cf;D=float(np.linalg.norm(a));difference=rf-rr
    root=float(np.sqrt(D*D-difference*difference))
    perpendicular=np.array([a[1],-a[0]])/D
    sign=1. if np.array([np.cos(psi),np.sin(psi)])@perpendicular>=0 else -1.
    radial=D/root-sign*difference*difference/(D*root)
    return radial*a/D+difference*np.array([-a[1],a[0]])/(D*D)
