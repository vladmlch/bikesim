"""Finite planar pad/box geometry shared by support forces and foot targets.

Each sole is sampled by two circular pressure pads. The pad radius is a
synthetic contact-shape parameter, not extra body mass or a hidden constraint.
A pedal has both broad faces and its finite edges; flipping it cannot turn it
into an attractive, infinite half-space. Lateral face contact is outside the
planar model and is explicitly excluded by ``within_width``.
"""
from dataclasses import dataclass
from math import hypot
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
    if (np.any(np.abs(rotation.T @ rotation-np.eye(3))>1e-9)
            or abs(np.linalg.det(rotation)-1.) > 1e-9
            or np.any(np.abs(rotation[:, 1]-[0., 1., 0.])>1e-9)):
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
    return _box_pad_contact(center,radius,origin,rotation,half)


def _box_pad_contact(center, radius, origin, rotation, half):
    """Evaluate already validated compiled planar support geometry."""
    local = rotation.T @ (center-origin)
    planar_x,planar_z=float(local[0]),float(local[2])
    extent_x,extent_z=float(half[0]),float(half[2])
    closest_x=min(max(planar_x,-extent_x),extent_x)
    closest_z=min(max(planar_z,-extent_z),extent_z)
    delta_x,delta_z=planar_x-closest_x,planar_z-closest_z
    distance=hypot(delta_x,delta_z)
    if distance > 1e-14:
        normal_x,normal_z=delta_x/distance,delta_z/distance
        signed_distance = distance
    else:
        clearance_x,clearance_z=extent_x-abs(planar_x),extent_z-abs(planar_z)
        axis=0 if clearance_x<=clearance_z else 2
        coordinate=planar_x if axis==0 else planar_z
        sign=1. if coordinate>0. else -1. if coordinate<0. else 0.
        if sign == 0.:
            # At a medial-axis tie choose the upward face deterministically.
            sign = 1. if rotation[2,axis] >= 0. else -1.
        normal_x,normal_z=(sign,0.) if axis==0 else (0.,sign)
        if axis==0:
            closest_x=sign*extent_x
        else:
            closest_z=sign*extent_z
        signed_distance=-(clearance_x if axis==0 else clearance_z)
    point = origin+rotation @ np.array([closest_x, local[1], closest_z])
    normal = rotation @ np.array([normal_x, 0., normal_z])
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
    return _upper_box_face(origin,rotation,half)


def _upper_box_face(origin, rotation, half):
    """Evaluate an upward face after validating the compiled planar model."""
    # Intersect a vertical ray through the spindle with the finite box.
    # Choosing the face with the most upward normal jumps by centimetres at
    # 45 degrees even though the physical top of a thin pedal is continuous.
    d0 = half[0]/abs(rotation[2, 0]) if abs(rotation[2, 0]) > 1e-14 else float('inf')
    d2 = half[2]/abs(rotation[2, 2]) if abs(rotation[2, 2]) > 1e-14 else float('inf')
    axis = 2 if d2 < d0 else 0
    distance = d2 if axis == 2 else d0
    sign = 1. if rotation[2, axis] >= 0. else -1.
    normal = sign*rotation[:, axis]
    point = origin+np.array([0., 0., distance])
    tangent = np.array([normal[2], 0., -normal[0]])
    return point, normal, tangent


class UnreachableSoleTarget(ValueError):
    """The finite support cannot realize the requested actuator goal."""


def sole_target_height(origin, rotation, half, sole_x_m, pad_half_length_m,
                       pad_radius_m, compression_m):
    """Match the combined pad compression rather than the spindle's clearance.

    Tilt changes how many sole pads carry load. Matching their summed
    penetration retains the declared two-pad stiffness without moving a foot
    through the platform or doubling its support request.
    """
    face_normal = rotation[:,2].copy()
    if face_normal[2] < 0.:
        face_normal = -face_normal
    if face_normal[2] > 1e-8:
        spread = abs(face_normal[0])*pad_half_length_m
        effective = compression_m if compression_m <= 0. or spread <= compression_m else 2.*compression_m-spread
        height = (origin[2]+(half[2]+pad_radius_m-effective
                  -face_normal[0]*(sole_x_m-origin[0]))/face_normal[2]-pad_radius_m)
        valid = True
        for offset in (-pad_half_length_m, pad_half_length_m):
            center = np.array([sole_x_m+offset, origin[1], height+pad_radius_m])
            local = rotation.T@(center-origin)
            if (abs(local[0]) > half[0] or abs(local[1]) > half[1]
                    or local[2]*np.sign(face_normal@rotation[:,2]) < 0.):
                valid = False
                break
        if valid:
            return float(height)
    radius = pad_radius_m-min(compression_m, 0.)
    along = rotation[[0, 2], 0]*half[0]
    across = rotation[[0, 2], 2]*half[2]
    corners = [first*along+second*across for first, second in
               ((-1., -1.), (-1., 1.), (1., 1.), (1., -1.))]
    normals = [-rotation[[0, 2], 0], rotation[[0, 2], 2],
               rotation[[0, 2], 0], -rotation[[0, 2], 2]]
    heights = []
    for offset in (-pad_half_length_m, pad_half_length_m):
        query_x = sole_x_m+offset-origin[0]
        for corner in corners:
            horizontal = query_x-corner[0]
            if abs(horizontal) <= radius+1e-12:
                heights.append(corner[1]+np.sqrt(max(0., radius*radius-horizontal*horizontal)))
        for index, normal in enumerate(normals):
            start = corners[index]+radius*normal
            end = corners[(index+1)%4]+radius*normal
            span = end[0]-start[0]
            if abs(span) > 1e-12:
                fraction = (query_x-start[0])/span
                if -1e-12 <= fraction <= 1.+1e-12:
                    heights.append(start[1]+fraction*(end[1]-start[1]))
    if not heights:
        raise UnreachableSoleTarget('sole target is outside the finite pedal footprint')
    upper = float(origin[2]+max(heights)-pad_radius_m)
    if compression_m <= 0.:
        return upper

    def penetration(height):
        total = derivative = 0.
        for offset in (-pad_half_length_m, pad_half_length_m):
            center = np.array([sole_x_m+offset, origin[1], height+pad_radius_m])
            contact = _box_pad_contact(center, pad_radius_m, origin, rotation, half)
            if contact.within_footprint and contact.gap_m < 0.:
                if center[2] < origin[2] and contact.normal[2] < -.5:
                    raise UnreachableSoleTarget('requested sole compression crosses the pedal surface')
                total -= contact.gap_m
                derivative += contact.normal[2]
        return total, derivative

    requested = 2.*compression_m
    lower = upper-max(requested, .001)
    for attempt in range(32):
        value, derivative = penetration(lower)
        if value >= requested:
            break
        lower -= max(requested, .001)
    else:
        raise UnreachableSoleTarget('requested pedal support load is not reachable')
    height = (lower+upper)/2.
    for attempt in range(32):
        value, derivative = penetration(height)
        difference = value-requested
        if abs(difference) <= 1e-12:
            return height
        if difference > 0.:
            lower = height
        else:
            upper = height
        candidate = height+difference/derivative if derivative > 1e-12 else upper
        height = candidate if lower < candidate < upper else (lower+upper)/2.
    raise RuntimeError('finite pedal support target did not converge')


def project_sole_goal(origin, rotation, half, pad_half_length_m, pad_radius_m,
                      compression_m, shear_m):
    """Project only the desired sole goal; actual material forces remain untouched."""
    surface, normal, tangent = _upper_box_face(origin, rotation, half)

    def evaluate(compression, shear):
        goal = surface+normal*(pad_radius_m-compression)+shear*tangent
        goal[2] = sole_target_height(origin, rotation, half, goal[0],
            pad_half_length_m, pad_radius_m, compression)
        return goal

    applied_compression = compression_m
    applied_shear = shear_m
    reasons = []
    try:
        goal = evaluate(applied_compression, applied_shear)
    except UnreachableSoleTarget:
        baseline = min(compression_m, 0.)
        for attempt in range(18):
            try:
                goal = evaluate(baseline, applied_shear)
                break
            except UnreachableSoleTarget:
                applied_shear *= .5
        else:
            applied_shear = 0.
            goal = evaluate(baseline, applied_shear)
        if applied_shear != shear_m:
            reasons.append('finite_shear_footprint')
        applied_compression = baseline
        if compression_m > 0.:
            lower, upper = 0., compression_m
            for attempt in range(18):
                candidate = (lower+upper)/2.
                try:
                    tested = evaluate(candidate, applied_shear)
                except UnreachableSoleTarget:
                    upper = candidate
                else:
                    lower = candidate
                    goal = tested
            applied_compression = lower
            reasons.append('finite_compression_capacity')
    return goal, {
        'requested_compression_m': float(compression_m),
        'applied_compression_m': float(applied_compression),
        'requested_shear_m': float(shear_m), 'applied_shear_m': float(applied_shear),
        'saturated': bool(reasons), 'limiting_reasons': reasons,
    }


def validate_planar_support_model(model, data, geoms):
    """Validate static topology once; MuJoCo preserves these planar rotations."""
    import mujoco
    geoms=tuple(geoms)
    ancestors=set()
    for geom in geoms:
        body=int(model.geom_bodyid[geom])
        while body:
            ancestors.add(body)
            body=int(model.body_parentid[body])
    joints=np.flatnonzero(np.isin(model.jnt_bodyid,tuple(ancestors)))
    hinge=model.jnt_type[joints]==mujoco.mjtJoint.mjJNT_HINGE
    slide=model.jnt_type[joints]==mujoco.mjtJoint.mjJNT_SLIDE
    if (np.any(~(hinge|slide))
            or np.any(np.abs(np.abs(data.xaxis[joints[hinge]])-[0.,1.,0.])>1e-9)
            or np.any(np.abs(data.xaxis[joints[slide],1])>1e-9)):
        raise ValueError('rider supports require scalar planar joint topology')
    for geom in geoms:
        if model.geom_type[geom]!=mujoco.mjtGeom.mjGEOM_BOX:
            raise ValueError('rider support geometry must be a box')
        _box(data.geom_xpos[geom],data.geom_xmat[geom].reshape(3,3),model.geom_size[geom])
