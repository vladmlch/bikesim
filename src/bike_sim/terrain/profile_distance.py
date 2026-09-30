"""Exact signed distance to the compiled, monotone-X, piecewise linear road.

There is no second smoothed obstacle geometry. Ties use the lowest segment ID;
``nonsmooth`` flags points where a unique differential cannot be asserted.
The horizontal search bound follows from the first candidate's Euclidean
length, so restricting the search cannot discard a closer segment.
"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class ProfileDistance:
    distance: np.ndarray
    normal: np.ndarray
    point: np.ndarray
    segment: np.ndarray
    nonsmooth: np.ndarray


class SignedProfile:
    def __init__(self, vertices_xz):
        vertices = np.array(vertices_xz, dtype=float, copy=True)
        if (vertices.ndim != 2 or vertices.shape[1] != 2 or len(vertices) < 2
                or not np.isfinite(vertices).all() or np.any(np.diff(vertices[:, 0]) <= 0)):
            raise ValueError('road vertices must be finite and strictly increasing in X')
        self.vertices = vertices
        self.edges = np.diff(vertices, axis=0)
        self.length2 = np.sum(self.edges**2, axis=1)
        self.normals = np.column_stack((-self.edges[:, 1], self.edges[:, 0])) / np.sqrt(self.length2)[:, None]
        for value in (self.vertices, self.edges, self.length2, self.normals):
            value.setflags(write=False)

    def height(self, x):
        x = np.asarray(x, dtype=float)
        if not np.isfinite(x).all() or np.any(x < self.vertices[0, 0]) or np.any(x > self.vertices[-1, 0]):
            raise ValueError('query lies outside the compiled road domain')
        return np.interp(x, self.vertices[:, 0], self.vertices[:, 1])

    def query(self, points_xz) -> ProfileDistance:
        p = np.asarray(points_xz, dtype=float)
        if p.ndim != 2 or p.shape[1] != 2 or not np.isfinite(p).all():
            raise ValueError('distance queries must be a finite (N, 2) array')
        height = self.height(p[:, 0])
        dist = np.empty(len(p)); normal = np.empty_like(p); closest = np.empty_like(p)
        segment = np.empty(len(p), dtype=int); nonsmooth = np.zeros(len(p), dtype=bool)
        local = np.clip(np.searchsorted(self.vertices[:, 0], p[:, 0], side='right')-1, 0, len(self.edges)-1)
        for i, (point, first) in enumerate(zip(p, local)):
            edge = self.edges[first]
            t = np.clip(np.dot(point-self.vertices[first], edge)/self.length2[first], 0., 1.)
            upper = np.linalg.norm(point-(self.vertices[first]+t*edge))
            left = max(0, np.searchsorted(self.vertices[:, 0], point[0]-upper, side='left')-1)
            right = min(len(self.edges), np.searchsorted(self.vertices[:, 0], point[0]+upper, side='right'))
            ids = np.arange(left, max(left+1, right))
            offsets = point-self.vertices[ids]
            fractions = np.clip(np.sum(offsets*self.edges[ids], axis=1)/self.length2[ids], 0., 1.)
            projections = self.vertices[ids]+fractions[:, None]*self.edges[ids]
            vectors = point-projections
            squared = np.sum(vectors*vectors, axis=1)
            # Only numerical ties, not a geometric smoothing length.
            best = float(np.min(squared))
            ties = np.flatnonzero(squared <= best+2e-15*max(1., best))
            winner = int(ties[0]); sid = int(ids[winner])
            length = float(np.sqrt(squared[winner]))
            sign = 1. if point[1] >= height[i] else -1.
            dist[i] = sign*length; closest[i] = projections[winner]; segment[i] = sid
            normal[i] = sign*vectors[winner]/length if length > 1e-12 else self.normals[sid]
            # Collinear mesh subdivisions do not introduce physical corners.
            tie_normals = self.normals[ids[ties]]
            nonsmooth[i] = bool(np.any(np.linalg.norm(tie_normals-self.normals[sid], axis=1) > 1e-10))
            if length <= 1e-12 and (fractions[winner] <= 1e-12 or fractions[winner] >= 1.-1e-12):
                vertex = sid if fractions[winner] <= 1e-12 else sid+1
                if 0 < vertex < len(self.vertices)-1:
                    nonsmooth[i] |= np.linalg.norm(self.normals[vertex-1]-self.normals[vertex]) > 1e-10
        return ProfileDistance(dist, normal, closest, segment, nonsmooth)


def signed_profile_distance(vertices_xz, points_xz):
    result = SignedProfile(vertices_xz).query(points_xz)
    return result.distance, result.normal, result.point, result.segment
