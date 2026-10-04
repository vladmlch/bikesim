"""Closed-form virtual-work requests; solved reactions remain post-hoc gates."""
from math import cos
from typing import Callable

from bike_sim.physics.checks import array, scalar


def leg_shares(phase_rad: float) -> dict[str, float]:
    downward = cos(scalar(phase_rad, 'crank phase'))
    if abs(downward) <= 1e-12:
        return {'front': .5, 'rear': .5}
    return {'front': 1. if downward > 0. else 0.,
            'rear': 0. if downward > 0. else 1.}


def preload_torque_nm(preload_n: float, spindle_xz, crank_xz) -> float:
    preload = scalar(preload_n, 'return foot preload', minimum=0.)
    spindle = array(spindle_xz, 'spindle position', (2,))
    crank = array(crank_xz, 'crank position', (2,))
    # F=(0,-preload) and dr/dphi=(z,-x) give F.dr/dphi=+preload*x.
    return float(preload*(spindle[0]-crank[0]))


def leg_crank_targets(total_nm: float, phase_rad: float, spindles: dict,
                      crank_xz, preload_n: float) -> dict[str, float]:
    total = scalar(total_nm, 'total crank torque')
    shares = leg_shares(phase_rad)
    if shares['front'] == shares['rear']:
        return {'front': total/2., 'rear': total/2.}
    rising = min(shares, key=shares.get)
    driving = 'rear' if rising == 'front' else 'front'
    recovery = preload_torque_nm(preload_n, spindles[rising], crank_xz)
    return {rising: recovery, driving: total-recovery}


def split_joint_torques(tau_leg_nm: float, jac: tuple[float, float],
                        capacity: Callable[[str, float], float]) -> tuple[float, float]:
    torque = scalar(tau_leg_nm, 'leg crank torque')
    j = array(jac, 'leg virtual-work Jacobian', (2,))
    names = ('hip', 'knee')
    def cap(name, sign):
        return scalar(capacity(name, sign), name+' directional capacity', minimum=0.)
    limits = [max(cap(name, 1.), cap(name, -1.)) for name in names]
    for _ in range(2):
        weights = [c*c*derivative for c,derivative in zip(limits, j)]
        denominator = float(sum(a*b for a,b in zip(j, weights)))
        if denominator < 1e-9:
            return (0., 0.)
        torques = [torque*w/denominator for w in weights]
        limits = [cap(name, 1. if t >= 0. else -1.) for name,t in zip(names,torques)]
    return tuple(max(-limit, min(limit, t)) for limit,t in zip(limits,torques))


def scale_to_power_budget(torques: dict[str, float], velocities: dict[str, float],
                          per_joint_w: float, total_w: float) -> dict[str, float]:
    per_joint = scalar(per_joint_w, 'joint positive-power budget', minimum=0.)
    total = scalar(total_w, 'whole-body positive-power budget', minimum=0.)
    result = {}
    for name, torque in torques.items():
        torque = scalar(torque, name+' torque')
        speed = scalar(velocities[name], name+' speed')
        result[name] = per_joint/speed if torque*speed > per_joint else torque
    positive = sum(max(t*velocities[n], 0.) for n,t in result.items())
    if positive > total:
        scale = total/positive
        result = {n: t*scale if t*velocities[n] > 0. else t for n,t in result.items()}
    return result
