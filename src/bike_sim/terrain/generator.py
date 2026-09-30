"""Seeded procedural rough-terrain tracks for anti-wheelie episodes.

A generated track is an ordinary validated TrackSpec (grade profile, obstacles,
surface sections), so it runs through the same heightfield and can be frozen
with save_track. One numpy Generator drives every draw, in a fixed order, so
(spec, seed) fully determines the track; nothing here touches global RNG state.

Grade is uphill-only (0..grade_max, with an optional steeper stretch up to
grade_peak): the research question is front-load loss while climbing under
torque. The first lead_in_m and last lead_out_m are flat and obstacle-free so
the static equilibrium and the finish are not contaminated by terrain.
"""
from dataclasses import asdict, dataclass, fields
import hashlib
import json
import numpy as np
from bike_sim.physics.checks import scalar
from bike_sim.terrain.grade import GradeProfile
from bike_sim.terrain.obstacles import Bump, Drop, RoadRoughness, Roots, SquareEdge
from bike_sim.terrain.profile import TrackSpec
from bike_sim.terrain.surface import SURFACES, SurfaceSection

# Minimum clear distance between placed features and sections, in metres.
CLEARANCE_M = 1.
PLACEMENT_TRIES = 200
FEATURE_TYPES = ('bump', 'edge', 'drop', 'roots')


def _range(value, name, *, minimum=0., integer=False):
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise ValueError(f'{name} must be a (low, high) pair')
    lo, hi = value
    if integer:
        if any(type(v) is not int for v in (lo, hi)):
            raise ValueError(f'{name} must hold integers')
    else:
        lo, hi = scalar(lo, name, minimum=minimum), scalar(hi, name, minimum=minimum)
    if lo > hi or lo < minimum:
        raise ValueError(f'{name} must satisfy {minimum} <= low <= high')
    return (lo, hi)


@dataclass(frozen=True)
class TerrainGenSpec:
    """Ranges a generated track is drawn from (lengths m, heights m, grades dz/dx)."""
    length_m: tuple = (60., 120.)
    grade_max: float = .25          # nominal knots uniform in [0, grade_max]
    grade_peak: float = .30         # one optional steep stretch
    grade_peak_prob: float = .35
    roughness_amp_m: tuple = (.005, .030)
    roughness_len_m: tuple = (3., 10.)
    roughness_corr_m: tuple = (.25, .5)
    bump_height_m: tuple = (.02, .10)
    bump_length_m: tuple = (.5, 1.)
    edge_height_m: tuple = (.02, .08)
    drop_height_m: tuple = (.02, .10)
    roots_len_m: tuple = (3., 6.)
    base_surfaces: tuple = ('hardpack', 'asphalt', 'loose')
    section_surfaces: tuple = ('wet', 'loose')
    section_len_m: tuple = (3., 15.)
    n_roughness: tuple = (2, 6)
    n_features: tuple = (1, 4)
    n_surface_sections: tuple = (0, 2)
    lead_in_m: float = 4.           # flat, clean start for equilibrium
    lead_out_m: float = 5.          # flat finish

    def __post_init__(self):
        # JSON/TOML give lists; freeze them so the dataclass stays hashable/immutable.
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, list):
                object.__setattr__(self, f.name, tuple(value))
        for name in ('length_m', 'roughness_amp_m', 'roughness_len_m', 'roughness_corr_m', 'bump_height_m',
                     'bump_length_m', 'edge_height_m', 'drop_height_m', 'roots_len_m', 'section_len_m'):
            lo, hi = _range(getattr(self, name), name)
            object.__setattr__(self, name, (float(lo), float(hi)))
        for name in ('n_roughness', 'n_features', 'n_surface_sections'):
            _range(getattr(self, name), name, integer=True)
        for name in ('grade_max', 'grade_peak', 'lead_in_m', 'lead_out_m'):
            scalar(getattr(self, name), name, minimum=0.)
        if self.grade_peak < self.grade_max:
            raise ValueError('grade_peak must not be below grade_max')
        if scalar(self.grade_peak_prob, 'grade_peak_prob', minimum=0.) > 1.:
            raise ValueError('grade_peak_prob must lie in [0, 1]')
        for name, allowed in (('base_surfaces', SURFACES), ('section_surfaces', SURFACES)):
            value = getattr(self, name)
            if not isinstance(value, tuple) or not value or any(v not in allowed for v in value):
                raise ValueError(f'{name} must be a nonempty tuple of known surfaces')
        if self.length_m[0] < self.lead_in_m+self.lead_out_m+2.*CLEARANCE_M:
            raise ValueError('shortest track must leave a usable body after lead-in/out')

    @classmethod
    def from_dict(cls, data):
        known = {f.name for f in fields(cls)}
        unknown = sorted(set(data)-known)
        if unknown:
            raise ValueError(f'unknown terrain generator keys: {unknown}')
        return cls(**data)

    def to_dict(self):
        return {k: list(v) if isinstance(v, tuple) else v for k, v in asdict(self).items()}

    def sha256(self):
        return hashlib.sha256(json.dumps(self.to_dict(), sort_keys=True).encode()).hexdigest()


def _r(value, digits=4):
    # Round so saved TOML files reproduce the track bit-for-bit.
    return round(float(value), digits)


def _free(start, end, taken):
    return all(end+CLEARANCE_M <= a or start >= b+CLEARANCE_M for a, b in taken)


def _place(rng, length, lo, hi, taken, *, shrink=False):
    """Random start for an interval of `length` inside [lo, hi] clear of `taken`.

    With shrink, the interval is halved (down to 1 m) when it cannot be placed.
    Returns (start, length) or None.
    """
    while True:
        if length <= hi-lo:
            for _ in range(PLACEMENT_TRIES):
                start = _r(rng.uniform(lo, hi-length), 2)
                if _free(start, start+length, taken):
                    return start, length
        if not shrink or length/2. < 1.:
            return None
        length = _r(length/2., 2)


def _grade(spec, rng, length):
    n = int(rng.integers(3, 6))
    a, b = spec.lead_in_m, length-spec.lead_out_m
    width = (b-a)/(n+1)
    # One knot per cell boundary, jittered within +-0.3 of a cell: strictly
    # increasing and at least 0.4 cells apart, whatever the draw.
    stations = [_r(a+(i+1)*width+(rng.uniform()-.5)*.6*width, 2) for i in range(n)]
    grades = [_r(rng.uniform(0., spec.grade_max)) for _ in range(n)]
    if rng.uniform() < spec.grade_peak_prob:
        i = int(rng.integers(0, n-1))
        grades[i], grades[i+1] = (_r(rng.uniform(spec.grade_max, spec.grade_peak)) for _ in range(2))
    knots = [(0., 0.), (spec.lead_in_m, 0.), *zip(stations, grades), (_r(b, 2), 0.), (length, 0.)]
    return GradeProfile(tuple(knots))


def _feature(kind, rng, spec, grade, taken, lo, hi):
    """Draw one feature of `kind`; returns (obstacle, start, end) or None if it cannot be placed."""
    if kind == 'bump':
        size = _r(rng.uniform(*spec.bump_length_m), 2)
        height = _r(rng.uniform(*spec.bump_height_m))
    elif kind == 'edge':
        size = _r(rng.uniform(.2, .6), 2)
        height = _r(rng.uniform(*spec.edge_height_m))
    elif kind == 'drop':
        size = 0.
        height = _r(rng.uniform(*spec.drop_height_m))
    else:
        size = _r(rng.uniform(*spec.roots_len_m), 2)
        height = _r(rng.uniform(spec.bump_height_m[0], min(spec.bump_height_m[1], .08)))
    placed = _place(rng, max(size, .01), lo, hi, taken)
    if placed is None:
        return None
    start = placed[0]
    if kind == 'drop' and grade.slope(start) < 0.:
        return None  # a drop on a descent would compound into a ski-jump
    if kind == 'bump':
        return Bump(start_m=start, height_m=height, bump_length_m=size), start, start+size
    if kind == 'edge':
        return SquareEdge(start_m=start, height_m=height, ledge_length_m=size), start, start+size
    if kind == 'drop':
        return Drop(start_m=start, height_m=height), start, start
    n_bumps = int(rng.integers(4, 9))
    return (Roots(start_m=start, n_bumps=n_bumps, height_m=height, section_length_m=size,
                  seed=int(rng.integers(0, 2**31))), start, start+size)


def generate_track(spec, *, seed, name=None):
    if not isinstance(spec, TerrainGenSpec):
        raise ValueError('expected a TerrainGenSpec')
    if type(seed) is not int or seed < 0:
        raise ValueError('terrain seed must be a nonnegative integer')
    rng = np.random.default_rng(seed)
    length = _r(rng.uniform(*spec.length_m), 1)
    lo, hi = spec.lead_in_m, length-spec.lead_out_m
    grade = _grade(spec, rng, length)

    taken, obstacles = [], []
    for _ in range(int(rng.integers(spec.n_roughness[0], spec.n_roughness[1]+1))):
        wanted = _r(rng.uniform(*spec.roughness_len_m), 2)
        amplitude = _r(rng.uniform(*spec.roughness_amp_m))
        correlation = _r(rng.uniform(*spec.roughness_corr_m), 3)
        rough_seed = int(rng.integers(0, 2**31))
        placed = _place(rng, wanted, lo, hi, taken, shrink=True)
        if placed is not None:
            start, size = placed
            obstacles.append(RoadRoughness(start_m=start, section_length_m=size, amplitude_m=amplitude,
                                           correlation_length_m=correlation, seed=rough_seed))
            taken.append((start, start+size))
    for _ in range(int(rng.integers(spec.n_features[0], spec.n_features[1]+1))):
        kind = FEATURE_TYPES[int(rng.integers(0, len(FEATURE_TYPES)))]
        feature = _feature(kind, rng, spec, grade, taken, lo, hi)
        if feature is not None:
            obstacle, start, end = feature
            obstacles.append(obstacle)
            taken.append((start, end))

    surface = spec.base_surfaces[int(rng.integers(0, len(spec.base_surfaces)))]
    sections, used = [], []
    for _ in range(int(rng.integers(spec.n_surface_sections[0], spec.n_surface_sections[1]+1))):
        wanted = _r(rng.uniform(*spec.section_len_m), 2)
        material = spec.section_surfaces[int(rng.integers(0, len(spec.section_surfaces)))]
        placed = _place(rng, wanted, lo, hi, used, shrink=True)
        if placed is not None:
            start, size = placed
            sections.append(SurfaceSection(start, _r(start+size, 2), material))
            used.append((start, start+size))

    track = TrackSpec(name or f'generated_{seed}', length, obstacles=obstacles, surface=surface,
        grade_profile=grade, surface_sections=tuple(sorted(sections, key=lambda s: s.start_m)),
        description=f'Generated rough climb; seed={seed}; spec_sha256={spec.sha256()}')
    track.validate()
    return track
