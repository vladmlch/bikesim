"""Planar two-link leg with a rigid shank and foot, in native +y rotation."""
from dataclasses import dataclass
from math import acos, atan2, cos, pi, sin

import numpy as np

from bike_sim.physics.checks import array, scalar


@dataclass(frozen=True)
class LegLoopGeometry:
    thigh_m: float
    link_m: float
    knee_sign: float
    thigh_angle0_rad: float
    link_angle0_rad: float
    side_sign: float


def planar_angle(direction: np.ndarray) -> float:
    """Angle from -z toward -x: positive is rotation about native +y."""
    d = array(direction, 'planar direction', (2,))
    return atan2(-d[0], -d[1])


def leg_loop_geometry(pose, side: str) -> LegLoopGeometry:
    if side not in ('front', 'rear'):
        raise ValueError('unknown leg side')
    hip = np.asarray(pose.hip)[[0, 2]]
    knee = np.asarray(getattr(pose, 'knee_'+side))[[0, 2]]
    pedal = np.asarray(getattr(pose, 'pedal_'+side))[[0, 2]]
    thigh, link, chord = knee-hip, pedal-knee, pedal-hip
    ahead = chord[0]*thigh[1]-chord[1]*thigh[0]
    return LegLoopGeometry(float(np.linalg.norm(thigh)), float(np.linalg.norm(link)),
        1. if ahead >= 0. else -1., planar_angle(thigh), planar_angle(link),
        1. if side == 'front' else -1.)


def spindle_xz(crank_xz, crank_m: float, phase_rad: float, side_sign: float,
               *, frame_pitch_rad=0., phase_offset_rad=0.) -> np.ndarray:
    """Spindle in the coordinates of crank_xz; include frame pitch in world coordinates.

    In bike coordinates leave frame_pitch_rad at zero. phase_offset_rad is
    the authored crank orientation at q=0, independently of its joint q.
    """
    crank = array(crank_xz, 'crank position', (2,))
    radius = scalar(crank_m, 'crank length', positive=True)
    phase = (scalar(phase_rad, 'crank phase') + scalar(frame_pitch_rad, 'frame pitch')
             + scalar(phase_offset_rad, 'crank phase offset'))
    sign = scalar(side_sign, 'leg side sign')
    if sign not in (-1., 1.):
        raise ValueError('leg side sign must be plus or minus one')
    return crank + sign*radius*np.array([cos(phase), -sin(phase)])


def _wrap(angle):
    return atan2(sin(angle), cos(angle))


def leg_joint_q(g: LegLoopGeometry, hip_xz, spindle_xz,
                pelvis_pitch_rad: float) -> tuple[float, float]:
    hip = array(hip_xz, 'hip position', (2,))
    spindle = array(spindle_xz, 'spindle position', (2,))
    thigh = scalar(g.thigh_m, 'thigh length', positive=True)
    link = scalar(g.link_m, 'rigid shank-foot length', positive=True)
    pitch = scalar(pelvis_pitch_rad, 'pelvis pitch')
    direction = spindle-hip
    length = float(np.linalg.norm(direction))
    if length <= 1e-12 or length > thigh+link+1e-9 or length < abs(thigh-link)-1e-9:
        raise ValueError('leg loop unreachable')
    knee_cos = (thigh*thigh+link*link-length*length)/(2.*thigh*link)
    hip_cos = (thigh*thigh+length*length-link*link)/(2.*thigh*length)
    if not (-1.-1e-9 <= knee_cos <= 1.+1e-9 and -1.-1e-9 <= hip_cos <= 1.+1e-9):
        raise ValueError('leg loop unreachable')
    thigh_angle = planar_angle(direction)-g.knee_sign*acos(float(np.clip(hip_cos, -1., 1.)))
    bend = g.knee_sign*(pi-acos(float(np.clip(knee_cos, -1., 1.))))
    return (_wrap(thigh_angle-pitch-g.thigh_angle0_rad),
            _wrap(bend-(g.link_angle0_rad-g.thigh_angle0_rad)))


def leg_loop_jacobian(g, hip_xz, crank_xz, crank_m, phase_rad, pelvis_pitch_rad,
                      *, delta_rad=1e-4, frame_pitch_rad=0., phase_offset_rad=0.) -> tuple[float, float]:
    delta = scalar(delta_rad, 'leg derivative interval', positive=True)
    def sample(phase):
        return leg_joint_q(g, hip_xz, spindle_xz(crank_xz, crank_m, phase,
            g.side_sign, frame_pitch_rad=frame_pitch_rad,
            phase_offset_rad=phase_offset_rad), pelvis_pitch_rad)
    plus, minus = sample(phase_rad+delta), sample(phase_rad-delta)
    return tuple(_wrap(a-b)/(2.*delta) for a,b in zip(plus, minus))
