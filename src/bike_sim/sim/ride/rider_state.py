"""Kinematic-only input contract for rider planning.

The rider's action planner receives a point-in-time kinematic snapshot plus a
bounded road preview window. It never receives MjData internals, solved
constraint reactions, or contact truth beyond what external sensing could
provide. Copies are taken at build time: later solver state cannot leak into
an already-issued state.
"""
from dataclasses import dataclass
import numpy as np
from bike_sim.physics.checks import array, scalar


@dataclass(frozen=True)
class RoadSample:
    """One read of the road profile at a fixed longitudinal position."""
    x_m: float
    height_m: float
    grade: float

    def __post_init__(self):
        scalar(self.x_m, 'road sample position')
        scalar(self.height_m, 'road sample height')
        scalar(self.grade, 'road sample grade')


def road_grade_for_posture(road: tuple[RoadSample, ...]) -> float:
    """Uniform sample mean, including both wheel locations and the preview."""
    if not road:
        raise ValueError('posture program needs at least one road sample')
    return float(sum(sample.grade for sample in road)/len(road))


def road_grade_preview(road) -> float:
    """Grade at the far endpoint of the allowed road preview window."""
    if not road:
        raise ValueError('posture program needs at least one road sample')
    return float(road[-1].grade)


@dataclass(frozen=True)
class RiderKinematicState:
    """Point-in-time rider input: generalized kinematics and road preview."""
    qpos: tuple[float, ...]
    qvel: tuple[float, ...]
    rider_time_s: float
    road: tuple[RoadSample, ...]

    def __post_init__(self):
        object.__setattr__(self, 'qpos',
            tuple(scalar(v, 'rider state qpos') for v in self.qpos))
        object.__setattr__(self, 'qvel',
            tuple(scalar(v, 'rider state qvel') for v in self.qvel))
        object.__setattr__(self, 'road', tuple(self.road))
        scalar(self.rider_time_s, 'rider state time', minimum=0.)
        if not all(isinstance(s, RoadSample) for s in self.road):
            raise ValueError('rider state road must be RoadSample entries')


def _profile_height(vertices, x):
    return float(np.interp(x, vertices[:, 0], vertices[:, 1]))


def _profile_grade(vertices, x):
    xs, zs = vertices[:, 0], vertices[:, 1]
    seg = int(np.clip(np.searchsorted(xs, x, side='right')-1, 0, len(xs)-2))
    dx = xs[seg+1]-xs[seg]
    return 0. if dx <= 0. else float((zs[seg+1]-zs[seg])/dx)


def road_samples(vertices, wheel_x_m, lookahead_m, *, spacing_m=.05):
    """Sample the rigid profile under the wheels and ahead of the front wheel.

    The preview window is exactly [front wheel x, front wheel x + lookahead];
    nothing further out is disclosed to the rider.
    """
    verts = array(vertices, 'compiled road vertices')
    if verts.ndim != 2 or verts.shape[1] != 2 or len(verts) < 2:
        raise ValueError('road vertices must be an (N,2) X-Z profile')
    if np.any(np.diff(verts[:, 0]) <= 0.):
        raise ValueError('road profile x must strictly increase')
    lookahead = scalar(lookahead_m, 'road lookahead', minimum=0.)
    spacing = scalar(spacing_m, 'road sample spacing', positive=True)
    wheels = tuple(scalar(x, 'wheel x') for x in wheel_x_m)
    if not wheels:
        raise ValueError('rider road preview needs wheel positions')
    front = max(wheels)
    xs = set(wheels)
    count = int(np.floor(lookahead/spacing+1e-12))
    xs.update(front+(k+1)*spacing for k in range(count))
    xs.add(front+lookahead)
    return tuple(RoadSample(x, _profile_height(verts, x), _profile_grade(verts, x))
                 for x in sorted(xs))


def rider_kinematic_state(model, data, *, vertices, wheel_x_m,
                          lookahead_m=0., spacing_m=.05):
    """Snapshot the solved-agnostic rider input for one physical instant."""
    if model is None or data is None:
        raise ValueError('kinematic state needs a model and data')
    qpos = np.asarray(data.qpos).reshape(-1)
    qvel = np.asarray(data.qvel).reshape(-1)
    if len(qpos) != model.nq or len(qvel) != model.nv:
        raise ValueError('kinematic state and model sizes disagree')
    return RiderKinematicState(
        qpos=tuple(map(float, qpos)), qvel=tuple(map(float, qvel)),
        rider_time_s=float(data.time),
        road=road_samples(vertices, wheel_x_m, lookahead_m, spacing_m=spacing_m))
