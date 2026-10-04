"""Circle contact with an immutable, piecewise-linear world X-Z road.

No analytic road function is reconstructed here. Feed the vertices actually used
by the heightfield. A single equivalent support is chosen. Two significant local
supports with normals more than 20 degrees apart are flagged, not combined or
silently replaced by native contact. Significance defaults (0.1 mm and 5% of the
largest penetration) are explicit synthetic geometry criteria, not calibration.
"""
from dataclasses import dataclass
from math import hypot, isclose
import numpy as np
from bike_sim.physics.checks import array, scalar


@dataclass(frozen=True)
class ProfileContact:
    point: np.ndarray
    normal: np.ndarray
    delta: float
    segment_id: int
    multi_support: bool

    def __post_init__(self):
        object.__setattr__(self, 'point', array(self.point, 'contact point', (2,), readonly=True))
        n = array(self.normal, 'contact normal', (2,), readonly=True)
        if not isclose(hypot(float(n[0]),float(n[1])),1.,rel_tol=0.,abs_tol=1e-10):
            raise ValueError('contact normal must be a unit vector')
        object.__setattr__(self, 'normal', n)
        object.__setattr__(self, 'delta', scalar(self.delta, 'penetration'))


class ProfileQuery:
    """Validated profile with local search and stable global segment IDs."""
    def __init__(self, vertices_xz, *, significant_delta_m=.0001,
                 significance_fraction=.05, normal_angle_deg=20.):
        vertices = array(vertices_xz, 'profile vertices')
        if vertices.ndim != 2 or vertices.shape[1] != 2 or len(vertices) < 2:
            raise ValueError('profile must contain at least two X-Z vertices')
        if np.any(np.diff(vertices[:, 0]) <= 0):
            raise ValueError('profile x must increase strictly')
        self.vertices = array(vertices, 'profile', readonly=True)
        # Scalar interpolation otherwise copies both strided Nx2 columns on
        # every tire query. These immutable contiguous axes have identical data.
        self._profile_x = array(self.vertices[:, 0], 'profile x', readonly=True)
        self._profile_z = array(self.vertices[:, 1], 'profile z', readonly=True)
        self._segments = np.diff(self.vertices,axis=0)
        self._segment_lengths_sq = np.einsum('ij,ij->i',self._segments,self._segments)
        self._height_changes = np.r_[0,np.cumsum(self._segments[:,1]!=0.)]
        self._left_directions=np.vstack((np.zeros(2),-self._segments))
        self._right_directions=np.vstack((self._segments,np.zeros(2)))
        self._maximum_z = float(np.max(vertices[:, 1]))
        self.significant_delta_m = scalar(significant_delta_m, 'significant penetration', minimum=0)
        self.significance_fraction = scalar(significance_fraction, 'significance fraction', minimum=0)
        angle = scalar(normal_angle_deg, 'normal angle', positive=True)
        if self.significance_fraction > 1 or angle >= 180:
            raise ValueError('invalid multi-support threshold')
        self.normal_cosine = np.cos(np.deg2rad(angle))

    def _candidates(self, c, lo, hi):
        """Project the centre onto every window segment; pruning is the caller's."""
        ids = np.arange(lo, hi)
        a = self.vertices[lo:hi]
        d = self._segments[lo:hi]
        t = np.clip(np.einsum('ij,ij->i', c-a, d)/self._segment_lengths_sq[lo:hi], 0., 1.)
        points = a+t[:, None]*d
        diff = c-points
        distances = np.sqrt(np.einsum('ij,ij->i', diff, diff))
        return ids, t, points, distances

    def _endpoint_keep(self, c, ids, t):
        keep = np.ones(len(ids), dtype=bool)
        # A projection on a segment endpoint is not a separate support when
        # continuing along its neighbour moves closer to the wheel. In particular,
        # subdividing a flat road must not manufacture hundreds of supports.
        endpoint = np.flatnonzero((t == 0.) | (t == 1.))
        if not len(endpoint):
            return keep
        vertex_ids = ids[endpoint] + (t[endpoint] == 1.).astype(int)
        v = self.vertices[vertex_ids]
        offsets = c-v
        for neighbour_directions in (self._left_directions,self._right_directions):
            directions=neighbour_directions[vertex_ids]
            closer = np.einsum('ij,ij->i',offsets,directions) > 1e-14
            keep[endpoint[closer]] = False
        return keep

    def contact(self, center_xz, radius, previous_segment=None):
        c = array(center_xz, 'wheel center', (2,))
        radius = scalar(radius, 'wheel radius', positive=True)
        count = len(self.vertices)-1
        if previous_segment is not None and (
            isinstance(previous_segment, bool) or not isinstance(previous_segment, (int, np.integer))
            or not 0 <= previous_segment < count
        ):
            raise ValueError('invalid previous contact segment')
        x = self._profile_x
        if not x[0] <= c[0] <= x[-1]:
            raise ValueError('wheel center is outside profile domain')
        if c[1] <= np.interp(c[0], x, self._profile_z):
            raise ValueError('wheel center reached or entered solid road')
        lo = max(0, int(np.searchsorted(x, c[0]-radius, side='right'))-2)
        hi = min(count, int(np.searchsorted(x, c[0]+radius, side='right'))+1)
        height=float(c[1]-self.vertices[lo,1])
        if height<radius and self._height_changes[hi]==self._height_changes[lo]:
            segment=max(0,int(np.searchsorted(x,c[0],side='left'))-1)
            point=np.array([c[0],self.vertices[lo,1]])
            distance=height
            if previous_segment is not None and lo<=previous_segment<hi:
                previous_x=min(max(float(c[0]),float(x[previous_segment])),float(x[previous_segment+1]))
                previous_distance=hypot(float(c[0])-previous_x,height)
                vertex=previous_segment if previous_x==x[previous_segment] else previous_segment+1
                endpoint=previous_x==x[previous_segment] or previous_x==x[previous_segment+1]
                eligible=not endpoint or not any(float((c-self.vertices[vertex])@directions[vertex])>1e-14
                    for directions in (self._left_directions,self._right_directions))
                if eligible and abs(previous_distance-distance)<=1e-12:
                    segment=previous_segment
                    point[0]=previous_x
                    distance=previous_distance
            if distance<=1e-12:
                raise ValueError('unsupported or degenerate contact geometry')
            return ProfileContact(point,(c-point)/distance,radius-distance,segment,False)
        ids, t, points, distances = self._candidates(c, lo, hi)
        if len(ids) == 0 or np.min(distances) >= radius:
            # Outside contact, retain an exact signed nearest gap rather than a
            # radius-window artefact; no force is applied in either case.
            # A segment farther away horizontally than this bound cannot beat
            # the current distance, even at the highest point of the terrain.
            # This is exact branch-and-bound, not a contact radius heuristic.
            # In particular a high airborne wheel over flat road need not scan
            # all 24,000 segments four times in every physical interval.
            if len(ids):
                best = float(np.min(distances))
                vertical_lower = max(0., float(c[1]) - self._maximum_z)
                reach = np.sqrt(max(0., best*best - vertical_lower*vertical_lower)) + 1e-8
                search_lo = max(0, int(np.searchsorted(x, c[0]-reach, side='right'))-2)
                search_hi = min(count, int(np.searchsorted(x, c[0]+reach, side='right'))+1)
            else:
                search_lo, search_hi = 0, count
            ids, t, points, distances = self._candidates(c, search_lo, search_hi)
        if len(ids) == 0 or np.min(distances) <= 1e-12:
            raise ValueError('unsupported or degenerate contact geometry')
        # An endpoint-pruned segment always has a closer neighbour inside the
        # window, so it can never be the winner; pruning is only required to
        # interpret the previous segment and the multi-support test.
        keep = self._endpoint_keep(c, ids, t)
        winner = int(np.argmin(distances))
        if previous_segment is not None:
            same = np.flatnonzero((ids == previous_segment) & keep)
            if len(same) and abs(distances[same[0]]-distances[winner]) <= 1e-12:
                winner = int(same[0])
        delta = radius-distances
        normal_winner = (c-points[winner])/distances[winner]
        # Normals and the multi-support tests are needed only for supports that
        # could be significant; skipped segments never reach them.
        near = np.flatnonzero((delta >= max(self.significant_delta_m,
            self.significance_fraction*max(float(delta[winner]), 0.))) & keep)
        if len(near):
            normals = (c-points[near])/distances[near, None]
            different = normals @ normal_winner < self.normal_cosine
            separated = np.linalg.norm(points[near]-points[winner], axis=1) > 1e-10
            multi = bool(np.any(different & separated))
        else:
            multi = False
        return ProfileContact(points[winner], normal_winner, float(delta[winner]),
                              int(ids[winner]), multi)


def closest_profile_contact(center_xz, radius, vertices_xz, previous_segment=None):
    return ProfileQuery(vertices_xz).contact(center_xz, radius, previous_segment)
