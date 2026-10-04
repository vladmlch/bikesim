"""Forward lean permitted by the saddle/bar geometry and anatomical ROM."""
from math import atan2, cos, pi, sin

import numpy as np

from bike_sim.physics.checks import scalar
from bike_sim.physics.rider_envelope import joint_q_range
from bike_sim.sim.ride.leg_loop import leg_joint_q, leg_loop_geometry, spindle_xz


def lean_limit_rad(pose, envelopes, *, crank_m, tdc_phase_rad, max_rad=.8, tol=1e-3):
    """Keep torso/arms and both hips at their own top dead centre within ROM.

    Angles use native +y hinges with the built pose at q=0. Hip envelopes
    describe flexion relative to the torso, so forward torso lean subtracts
    from the hip coordinate used for that anatomical check. This calculation
    only bounds an actuator target; it never moves a live model.
    """
    from bike_sim.sim.ride.rider_control import two_link_ik
    maximum = scalar(max_rad, 'lean search maximum', minimum=0.)
    tolerance = scalar(tol, 'lean search tolerance', positive=True)
    radius = scalar(crank_m, 'lean crank length', positive=True)
    phase = scalar(tdc_phase_rad, 'top dead centre phase')
    hip = pose.hip[[0, 2]]
    trunk = (pose.shoulder-pose.hip)[[0, 2]]
    upper = (pose.elbow-pose.shoulder)[[0, 2]]
    lower = (pose.grip-pose.elbow)[[0, 2]]
    arm_lengths = float(np.linalg.norm(upper)), float(np.linalg.norm(lower))
    upper0 = atan2(upper[1], upper[0])
    lower0 = atan2(lower[1], lower[0])
    branch = 1 if sin(lower0-upper0) >= 0. else -1
    crank = ((pose.pedal_front+pose.pedal_rear)/2)[[0, 2]]
    hips_at_tdc = {}
    for side in ('front', 'rear'):
        geometry = leg_loop_geometry(pose, side)
        tdc = phase if side == 'front' else phase+pi
        spindle = spindle_xz(crank, radius, tdc, geometry.side_sign)
        hips_at_tdc[side] = leg_joint_q(geometry, hip, spindle, 0.)[0]

    def within(name, q):
        lo, hi = joint_q_range(envelopes[name])
        return lo-1e-12 <= q <= hi+1e-12

    def feasible(lean):
        rotation = np.array([[cos(lean), sin(lean)], [-sin(lean), cos(lean)]])
        shoulder = hip+rotation@trunk
        arm_goal = rotation.T@(pose.grip[[0, 2]]-shoulder)
        angles, saturated = two_link_ik(arm_goal, *arm_lengths, elbow_sign=branch)
        if saturated or not within('rider_torso_hinge', lean):
            return False
        for side in ('left', 'right'):
            if not within('rider_shoulder_'+side, upper0-angles[0]):
                return False
            if not within('rider_elbow_'+side, (lower0-upper0)-angles[1]):
                return False
        return all(within('rider_hip_'+side, q-lean) for side, q in hips_at_tdc.items())

    if not feasible(0.):
        raise ValueError('neutral seated posture lies outside reachable joint ROM')
    if feasible(maximum):
        return maximum
    low, high = 0., maximum
    while high-low > tolerance:
        middle = (low+high)/2
        if feasible(middle):
            low = middle
        else:
            high = middle
    return low
