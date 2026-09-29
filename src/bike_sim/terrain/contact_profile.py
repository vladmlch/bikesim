"""Circle contact with an immutable, piecewise-linear world X-Z road.

No analytic road function is reconstructed here. Feed the vertices actually used
by the heightfield. A single equivalent support is chosen. Two significant local
supports with normals more than 20 degrees apart are flagged, not combined or
silently replaced by native contact. Significance defaults (0.1 mm and 5% of the
largest penetration) are explicit synthetic geometry criteria, not calibration.
"""
from dataclasses import dataclass
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
        if not np.isclose(np.linalg.norm(n), 1., rtol=0., atol=1e-10):
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
        self._maximum_z = float(np.max(vertices[:, 1]))
        self.significant_delta_m = scalar(significant_delta_m, 'significant penetration', minimum=0)
        self.significance_fraction = scalar(significance_fraction, 'significance fraction', minimum=0)
        angle = scalar(normal_angle_deg, 'normal angle', positive=True)
        if self.significance_fraction > 1 or angle >= 180:
            raise ValueError('invalid multi-support threshold')
        self.normal_cosine = np.cos(np.deg2rad(angle))

    def _candidates(self, c, lo, hi):
        ids = np.arange(lo, hi)
        a, b = self.vertices[ids], self.vertices[ids+1]
        d = b-a
        t = np.clip(np.einsum('ij,ij->i', c-a, d)/np.einsum('ij,ij->i', d, d), 0., 1.)
        points = a+t[:, None]*d
        keep = np.ones(len(ids), dtype=bool)
        # A projection on a segment endpoint is not a separate support when
        # continuing along its neighbour moves closer to the wheel. In particular,
        # subdividing a flat road must not manufacture hundreds of supports.
        endpoint = np.flatnonzero((t == 0.) | (t == 1.))
        vertex_ids = ids[endpoint] + (t[endpoint] == 1.).astype(int)
        v = self.vertices[vertex_ids]
        offsets = c-v
        for shift in (-1, 1):
            neighbours = vertex_ids+shift
            valid = (neighbours >= 0) & (neighbours < len(self.vertices))
            directions = self.vertices[np.clip(neighbours,0,len(self.vertices)-1)]-v
            closer = np.einsum('ij,ij->i',offsets,directions) > 1e-14
            keep[endpoint[valid & closer]] = False
        points, ids = points[keep], ids[keep]
        distances = np.linalg.norm(c-points, axis=1)
        return ids, points, distances

    def contact(self, center_xz, radius, previous_segment=None):
        c = array(center_xz, 'wheel center', (2,))
        radius = scalar(radius, 'wheel radius', positive=True)
        count = len(self.vertices)-1
        if previous_segment is not None and (
            isinstance(previous_segment, bool) or not isinstance(previous_segment, (int, np.integer))
            or not 0 <= previous_segment < count
        ):
            raise ValueError('invalid previous contact segment')
        x = self.vertices[:, 0]
        if not x[0] <= c[0] <= x[-1]:
            raise ValueError('wheel center is outside profile domain')
        if c[1] <= np.interp(c[0], x, self.vertices[:, 1]):
            raise ValueError('wheel center reached or entered solid road')
        lo = max(0, int(np.searchsorted(x, c[0]-radius, side='right'))-2)
        hi = min(count, int(np.searchsorted(x, c[0]+radius, side='right'))+1)
        ids, points, distances = self._candidates(c, lo, hi)
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
            ids, points, distances = self._candidates(c, search_lo, search_hi)
        if len(ids) == 0 or np.min(distances) <= 1e-12:
            raise ValueError('unsupported or degenerate contact geometry')
        winner = int(np.argmin(distances))
        if previous_segment is not None:
            same = np.flatnonzero(ids == previous_segment)
            if len(same) and abs(distances[same[0]]-distances[winner]) <= 1e-12:
                winner = int(same[0])
        normals = (c-points)/distances[:, None]
        delta = radius-distances
        significant = delta >= max(self.significant_delta_m,
                                   self.significance_fraction*max(float(delta[winner]), 0.))
        different = normals @ normals[winner] < self.normal_cosine
        separated = np.linalg.norm(points-points[winner], axis=1) > 1e-10
        multi = bool(np.any(significant & different & separated))
        return ProfileContact(points[winner], normals[winner], float(delta[winner]),
                              int(ids[winner]), multi)


def closest_profile_contact(center_xz, radius, vertices_xz, previous_segment=None):
    return ProfileQuery(vertices_xz).contact(center_xz, radius, previous_segment)
