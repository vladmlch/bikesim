"""Postural force requests balanced against the current anatomical gravity wrench.

These are targets for internal joint actuators, not imposed support loads. Actual
unilateral contacts, friction and actuator limits still decide delivered forces.
"""
import numpy as np
from bike_sim.physics.checks import array, scalar


NAMES = ('saddle', 'front', 'rear', 'grip')


def gravity_support_targets(weight_n, com_x_m, points_x_m, crank_x_m,
                            pedal_fraction, bar_fraction, enabled, *, pitch_moment_nm=0., vertical_target_n=None):
    """Closest preferred vertical load split with zero coasting crank torque.

    Force and world moment balance are explicit. Nonnegative saddle/pedal
    requests and a bilateral hand force are solved by a small active set. If
    the remaining contacts cannot support that wrench, the residual is exposed
    rather than creating an unmodelled root force.
    """
    weight = scalar(weight_n, 'rider weight', minimum=0)
    com = scalar(com_x_m, 'rider CoM x')
    crank = scalar(crank_x_m, 'crank x')
    moment = scalar(pitch_moment_nm, 'postural pitch moment')
    vertical = weight if vertical_target_n is None else scalar(vertical_target_n,'vertical support target')
    points = array(points_x_m, 'support x coordinates', (4,))
    pedal = scalar(pedal_fraction, 'preferred pedal share', minimum=0)
    bar = scalar(bar_fraction, 'preferred bar share', minimum=0)
    if pedal+bar >= 1 or len(enabled) != 4 or any(type(value) is not bool for value in enabled):
        raise ValueError('invalid postural support distribution')
    preferred = weight*np.array([1-pedal-bar, pedal/2, pedal/2, bar])
    # Work around the CoM for good conditioning at large track coordinates.
    matrix = np.array([np.ones(4), points-com,
                       [0., points[1]-crank, points[2]-crank, 0.]])
    # A vertical force has world-Y moment -r_x*Fz. This is only a
    # requested contact wrench; no floating root coordinate is actuated.
    target = np.array([vertical, -moment, 0.])
    available = np.array(enabled, dtype=bool)
    forces = np.zeros(4)
    for _ in range(4):
        columns = np.flatnonzero(available)
        if not len(columns):
            forces[:] = 0.
            break
        selected = matrix[:, columns]
        desired = preferred[columns]
        correction = np.linalg.lstsq(selected, target-selected@desired, rcond=1e-10)[0]
        forces[:] = 0.
        forces[columns] = desired+correction
        negative = [index for index in columns if index != 3 and forces[index] < -1e-9]
        if not negative:
            forces[:3] = np.maximum(forces[:3], 0.)
            break
        available[min(negative, key=lambda index: forces[index])] = False
    residual = matrix@forces-target
    return dict(zip(NAMES, map(float, forces))), {
        'requested_pitch_moment_nm': moment,
        'vertical_force_error_n': float(residual[0]),
        'gravity_moment_error_nm': float(residual[1]),
        'coasting_crank_torque_nm': float(residual[2]),
        'feasible': bool(np.max(np.abs(residual)) <= max(1e-8, weight*1e-8)),
    }


def pedaling_support_targets(weight_n, com_m, points_m, crank_x_m,
                             pedal_fraction, bar_fraction, enabled, pedal_forces,
                             *, pitch_moment_nm=0.):
    """Return forces requested *on the bike*, including the pedaling request.

    Pedal reaction is part of supporting rider weight, not an additional upward
    external force. The remaining support request compensates its full wrench.
    The bilateral hand may balance horizontal pedal reaction; unilateral pad
    limits remain in the contact model and no requested force is imposed there.
    """
    com=array(com_m,'rider CoM',(3,))
    points=array(points_m,'support points',(4,3))
    pedal={side:array(pedal_forces[side],side+' pedal force',(3,)) for side in ('front','rear')}
    if any(abs(force[1])>1e-12 for force in pedal.values()):
        raise ValueError('pedaling request must be planar')
    extra=np.zeros((4,3))
    extra[1]=-pedal['front']; extra[2]=-pedal['rear']
    if enabled[3]:
        extra[3,0]=-np.sum(extra[:3,0])
    arms=points-com
    extra_moment=float(np.sum(arms[:,2]*extra[:,0]-arms[:,0]*extra[:,2]))
    loads,diagnostics=gravity_support_targets(weight_n,com[0],points[:,0],crank_x_m,
        pedal_fraction,bar_fraction,enabled,pitch_moment_nm=pitch_moment_nm-extra_moment,
        vertical_target_n=weight_n-float(np.sum(extra[:,2])))
    reactions=extra.copy()
    reactions[:,2]+=np.array(list(loads.values()))
    diagnostics.update(requested_vertical_forces_n=loads,
        requested_pitch_moment_nm=float(pitch_moment_nm),
        total_requested_force_on_rider_n=np.sum(reactions,axis=0).tolist(),
        total_requested_pitch_moment_nm=float(np.sum(arms[:,2]*reactions[:,0]-arms[:,0]*reactions[:,2])))
    diagnostics['feasible']=bool(diagnostics['feasible'] and abs(np.sum(reactions[:,0]))<=1e-8)
    return {name:-force for name,force in zip(NAMES,reactions)},diagnostics
