"""Finite planar pad/box geometry shared by support forces and foot targets.

Each sole is sampled by two circular pressure pads. The pad radius is a
synthetic contact-shape parameter, not extra body mass or a hidden constraint.
A pedal has both broad faces and its finite edges; flipping it cannot turn it
into an attractive, infinite half-space. Lateral face contact is outside the
planar model and is explicitly excluded by ``within_width``.
"""
from dataclasses import dataclass
import numpy as np
from bike_sim.physics.checks import array, scalar


@dataclass(frozen=True)
class BoxPadContact:
    point_m: np.ndarray
    normal: np.ndarray
    gap_m: float
    within_width: bool
    within_footprint: bool

    def __post_init__(self):
        for name in ('point_m', 'normal'):
            value = np.asarray(getattr(self, name), dtype=float)
            object.__setattr__(self, name, np.frombuffer(value.tobytes(), dtype=float))

    @property
    def tangent(self):
        return np.array([self.normal[2], 0., -self.normal[0]])


def _box(origin, rotation, half_size):
    origin = array(origin, 'box origin', (3,))
    rotation = array(rotation, 'box orientation', (3, 3))
    half = array(half_size, 'box half size', (3,))
    if np.any(half <= 0.):
        raise ValueError('box dimensions must be positive')
    if (not np.allclose(rotation.T @ rotation, np.eye(3), rtol=0., atol=1e-9)
            or abs(np.linalg.det(rotation)-1.) > 1e-9
            or not np.allclose(rotation[:, 1], [0., 1., 0.], rtol=0., atol=1e-9)):
        raise ValueError('support box needs a planar proper rotation')
    return origin, rotation, half


def box_pad_contact(center_m, radius_m, box_origin_m, box_rotation, half_size_m):
    """Signed gap and closest box point for a circular pad in the X-Z plane.

    Positive gap is separation. Outside the box, corner normals are radial.
    Inside it, the nearest finite face supplies the signed distance. Unlike a
    half-space, a point far below or beyond either end has positive distance.
    Normal and tangent velocities must be evaluated at the returned common
    force point on both bodies for virtual work to match this gap.
    """
    center = array(center_m, 'pad center', (3,))
    radius = scalar(radius_m, 'pad radius', positive=True)
    origin, rotation, half = _box(box_origin_m, box_rotation, half_size_m)
    local = rotation.T @ (center-origin)
    planar = local[[0, 2]]
    extent = half[[0, 2]]
    closest = np.clip(planar, -extent, extent)
    delta = planar-closest
    distance = float(np.linalg.norm(delta))
    if distance > 1e-14:
        n = delta/distance
        signed_distance = distance
    else:
        clearance = extent-np.abs(planar)
        axis = int(np.argmin(clearance))
        sign = np.sign(planar[axis])
        if sign == 0.:
            # At a medial-axis tie choose the upward face deterministically.
            sign = 1. if rotation[2, (0, 2)[axis]] >= 0. else -1.
        n = np.zeros(2)
        n[axis] = sign
        closest[axis] = sign*extent[axis]
        signed_distance = -float(clearance[axis])
    point = origin+rotation @ np.array([closest[0], local[1], closest[1]])
    normal = rotation @ np.array([n[0], 0., n[1]])
    width = bool(abs(local[1]) <= half[1]+1e-9)
    footprint = width and bool(abs(local[0]) <= half[0]+radius)
    return BoxPadContact(point, normal, signed_distance-radius, width, footprint)


def upper_box_face(box_origin_m, box_rotation, half_size_m):
    """Return the upward-facing finite face used by the foot's posture target.

    Both broad faces can support a shoe. Near a vertical pedal the narrow end
    face is the upper face. This is only an actuator target; actual contact is
    evaluated independently by ``box_pad_contact``.
    """
    origin, rotation, half = _box(box_origin_m, box_rotation, half_size_m)
    # Intersect a vertical ray through the spindle with the finite box.
    # Choosing the face with the most upward normal jumps by centimetres at
    # 45 degrees even though the physical top of a thin pedal is continuous.
    distances = {i: half[i]/abs(rotation[2, i]) if abs(rotation[2, i])>1e-14
                 else float('inf') for i in (0, 2)}
    axis = min(distances, key=distances.get)
    sign = 1. if rotation[2, axis] >= 0. else -1.
    normal = sign*rotation[:, axis]
    point = origin+np.array([0., 0., distances[axis]])
    tangent = np.array([normal[2], 0., -normal[0]])
    return point, normal, tangent
