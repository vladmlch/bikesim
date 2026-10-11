"""
Seeded Alpine Scree-Trail Generator.

A high-mountain MTB climb above the treeline, built for traction, wheel-slip and
front-wheel-lift research: a trail rhythm of steep ramps and switchback shelves, scree
sections (a low-friction surface plus fine texture), embedded boulders, small rock gardens
and drainage bowls on hardpack.

The result is an ordinary validated TrackSpec, so it runs through the same heightfield,
dumps to TOML and loads back. One numpy Generator drives every draw in a fixed order:
(spec, seed) fully determines the track; nothing here touches global RNG state.

With ``single_contact_safe`` (the default) every obstacle is drawn to satisfy the
``compliant_2d`` tyre's one-support geometry (:mod:`bike_sim.terrain.contact_safety`), so a
strict research run never ends in ``multi_support``. Switching it off removes the guards for
a multi-contact tyre; the ranges are then the caller's responsibility.
Design: docs/superpowers/specs/2026-10-11-alpine-scree-track-design.md.
"""
from dataclasses import asdict, dataclass, fields
import hashlib
import json
from math import ceil

import numpy as np

from bike_sim.physics.checks import scalar
from bike_sim.terrain.contact_safety import concave_span_deg, max_bowl_depth_m, min_bump_length_m
from bike_sim.terrain.grade import GradeProfile
from bike_sim.terrain.obstacles import BowlPothole, Bump, RoadRoughness, RockGarden, SquareEdge
from bike_sim.terrain.placement import PLACEMENT_TRIES, check_range, free_intervals, place, rounded as _r
from bike_sim.terrain.profile import TrackSpec
from bike_sim.terrain.surface import SURFACES, SurfaceSection

ALPINE_PRESET_SEED = 41
MIN_BACKGROUND_CORRELATIONS = 4.
"""Free stretches shorter than this many correlation lengths get no background texture."""
MIN_GRID_M = 0.01
"""The centimetre grid the body, placements and corr draws are rounded onto."""
_FACETED_REDRAWS = 25
"""Amplitude-shrink rounds a faceted section gets before its spec is declared impossible."""

_RANGE_FIELDS = ('ramp_len_m', 'ramp_grade', 'shelf_len_m', 'shelf_grade', 'wall_grade', 'counter_len_m',
                 'scree_len_m', 'scree_amp_m', 'scree_corr_m', 'wet_len_m', 'background_amp_m',
                 'background_corr_m', 'boulder_height_m', 'boulder_len_m', 'rock_len_m', 'rock_amp_m',
                 'rock_corr_m', 'water_bar_depth_m', 'water_bar_len_m', 'sharp_edge_height_m')
_GRID_RANGE_FIELDS = ('ramp_len_m', 'shelf_len_m', 'counter_len_m', 'scree_len_m', 'scree_corr_m',
                      'wet_len_m', 'background_corr_m', 'boulder_len_m', 'rock_len_m', 'rock_corr_m',
                      'water_bar_len_m')
"""Length-like ranges whose low bound must still round to a positive grid step."""
_COUNT_FIELDS = ('n_scree', 'n_wet', 'n_boulders', 'n_rock_gardens', 'n_water_bars', 'n_sharp_edges')
_POSITIVE_FIELDS = ('length_m', 'lead_in_m', 'lead_out_m', 'transition_m', 'min_contact_radius_m',
                    'max_kink_deg', 'contact_window_m')


@dataclass(frozen=True)
class AlpineGenSpec:
    """Ranges an alpine scree climb is drawn from (lengths m, heights m, grades dz/dx)."""
    length_m: float = 120.
    lead_in_m: float = 10.
    lead_out_m: float = 15.
    transition_m: float = 2.
    ramp_len_m: tuple = (15., 30.)
    ramp_grade: tuple = (.15, .25)
    shelf_len_m: tuple = (4., 8.)
    shelf_grade: tuple = (.02, .06)
    wall_grade: tuple = (.26, .32)
    n_walls: int = 1
    counter_len_m: tuple = (3., 6.)
    counter_grade: tuple = (-.08, -.03)
    counter_prob: float = .3
    base_surface: str = 'hardpack'
    n_scree: tuple = (2, 3)
    scree_len_m: tuple = (6., 15.)
    scree_ramp_bias: float = .7
    scree_amp_m: tuple = (.010, .020)
    scree_corr_m: tuple = (.15, .20)
    n_wet: tuple = (0, 0)
    wet_len_m: tuple = (2., 5.)
    background_amp_m: tuple = (.005, .010)
    background_corr_m: tuple = (.20, .30)
    n_boulders: tuple = (2, 5)
    boulder_height_m: tuple = (.05, .15)
    boulder_len_m: tuple = (.6, 1.2)
    n_rock_gardens: tuple = (1, 2)
    rock_len_m: tuple = (4., 8.)
    rock_amp_m: tuple = (.03, .06)
    rock_corr_m: tuple = (.30, .45)
    n_water_bars: tuple = (1, 3)
    water_bar_depth_m: tuple = (.08, .15)
    water_bar_len_m: tuple = (1.0, 1.5)
    n_sharp_edges: tuple = (0, 0)
    sharp_edge_height_m: tuple = (.03, .08)
    single_contact_safe: bool = True
    min_contact_radius_m: float = .45
    max_kink_deg: float = 17.
    contact_window_m: float = .25

    def __post_init__(self):
        # JSON/TOML give lists; freeze them so the dataclass stays hashable/immutable.
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, list):
                object.__setattr__(self, f.name, tuple(value))
        for name in _RANGE_FIELDS:
            lo, hi = check_range(getattr(self, name), name,
                                 minimum=MIN_GRID_M if name in _GRID_RANGE_FIELDS else 0.)
            object.__setattr__(self, name, (float(lo), float(hi)))
        lo, hi = check_range(self.counter_grade, 'counter_grade', minimum=-1.)
        if hi >= 0.:
            raise ValueError('counter_grade must be negative: it is a short descent')
        object.__setattr__(self, 'counter_grade', (float(lo), float(hi)))
        for name in _COUNT_FIELDS:
            check_range(getattr(self, name), name, integer=True)
        for name in _POSITIVE_FIELDS:
            object.__setattr__(self, name, scalar(getattr(self, name), name, positive=True))
        for name in ('counter_prob', 'scree_ramp_bias'):
            value = scalar(getattr(self, name), name, minimum=0.)
            if value > 1.:
                raise ValueError(f'{name} must lie in [0, 1]')
            object.__setattr__(self, name, value)
        if type(self.n_walls) is not int or not 0 <= self.n_walls <= 1:
            raise ValueError('n_walls must be 0 or 1')
        if type(self.single_contact_safe) is not bool:
            raise ValueError('single_contact_safe must be a bool')
        if self.wall_grade[0] < self.ramp_grade[1]:
            raise ValueError('wall_grade must start at or above the top of ramp_grade')
        if self.base_surface not in SURFACES:
            raise ValueError(f'unknown base_surface {self.base_surface!r}; available: {", ".join(SURFACES)}')
        if self.transition_m < MIN_GRID_M:
            raise ValueError(f'transition_m must be at least {MIN_GRID_M} m: segments are spaced on a centimetre grid')
        # _segments walks the body on the rounded centimetre grid, so the fit is checked
        # on the same rounded terms rather than on the raw spec values.
        cursor = _r(self.lead_in_m + self.transition_m, 2)
        limit = _r(_r(self.length_m - self.lead_out_m, 2) - self.transition_m, 2)
        if _r(limit - cursor, 2) < self.ramp_len_m[0]:
            raise ValueError('the body must hold at least one shortest ramp between lead-in and lead-out')
        if self.single_contact_safe and self.n_sharp_edges[1] > 0:
            raise ValueError('sharp edges always put the 2D tyre on two supports; '
                             'set single_contact_safe = false to opt in')

    @classmethod
    def from_dict(cls, data):
        known = {f.name for f in fields(cls)}
        unknown = sorted(set(data) - known)
        if unknown:
            raise ValueError(f'unknown alpine generator keys: {unknown}')
        return cls(**data)

    def to_dict(self):
        return {k: list(v) if isinstance(v, tuple) else v for k, v in asdict(self).items()}

    def sha256(self):
        return hashlib.sha256(json.dumps(self.to_dict(), sort_keys=True).encode()).hexdigest()


def _segments(spec, rng):
    """Fill the body with ramp -> shelf [-> counter-slope] cycles; rows are [kind, start, end, grade].

    Each segment ends ``transition_m`` before the next starts so the grade profile ramps
    linearly between them. When the remaining body is too short for a ramp, the last
    segment runs on to the body end; one ramp becomes the wall when ``n_walls`` is 1.
    """
    body_end = _r(spec.length_m - spec.lead_out_m, 2)
    limit = _r(body_end - spec.transition_m, 2)
    cursor = _r(spec.lead_in_m + spec.transition_m, 2)
    segments = []

    def push(kind, length, grade):
        nonlocal cursor
        if cursor >= limit:  # no room left: the draws are spent but nothing is appended
            return False
        end = _r(min(cursor + length, limit), 2)
        segments.append([kind, cursor, end, grade])
        cursor = _r(end + spec.transition_m, 2)
        return end < limit

    while cursor < limit:
        length, grade = _r(rng.uniform(*spec.ramp_len_m), 2), _r(rng.uniform(*spec.ramp_grade))
        if _r(limit - cursor, 2) < spec.ramp_len_m[0]:
            segments[-1][2] = limit
            break
        if not push('ramp', length, grade):
            break
        if not push('shelf', _r(rng.uniform(*spec.shelf_len_m), 2), _r(rng.uniform(*spec.shelf_grade))):
            break
        if rng.uniform() < spec.counter_prob and not push(
                'counter', _r(rng.uniform(*spec.counter_len_m), 2), _r(rng.uniform(*spec.counter_grade))):
            break
    if spec.n_walls:
        ramps = [i for i, s in enumerate(segments) if s[0] == 'ramp']
        i = ramps[int(rng.integers(0, len(ramps)))]
        segments[i][0], segments[i][3] = 'wall', _r(rng.uniform(*spec.wall_grade))
    return segments


def _rhythm(spec, rng):
    """Grade profile of the trail rhythm plus the (start, end) of every ramp or wall."""
    segments = _segments(spec, rng)
    body_end = _r(spec.length_m - spec.lead_out_m, 2)
    knots = [(0., 0.), (spec.lead_in_m, 0.)]
    knots += [knot for _, s, e, g in segments for knot in ((s, g), (e, g))]
    knots += [(body_end, 0.), (spec.length_m, 0.)]
    return GradeProfile(tuple(knots)), [(s, e) for kind, s, e, _ in segments if kind in ('ramp', 'wall')]


def _count(rng, bounds):
    return int(rng.integers(bounds[0], bounds[1] + 1))


def _faceted(spec, rng, cls, start, length, amp, corr):
    """A seeded faceted section whose concave span respects ``max_kink_deg``.

    New seeds are drawn first; if none of PLACEMENT_TRIES passes, the amplitude shrinks by
    a fifth and the draw repeats. The shrink rounds onto the placement grid and reaches a
    fixed point, so after ``_FACETED_REDRAWS`` rounds the spec is rejected rather than left
    redrawing forever. Without ``single_contact_safe`` the first seed is kept.
    """
    for _ in range(_FACETED_REDRAWS):
        for _ in range(PLACEMENT_TRIES):
            obstacle = cls(start_m=start, section_length_m=length, amplitude_m=amp,
                           correlation_length_m=corr, seed=int(rng.integers(0, 2**31)))
            if not spec.single_contact_safe or concave_span_deg(obstacle, spec.contact_window_m) < spec.max_kink_deg:
                return obstacle
        amp = _r(amp * .8)
    raise ValueError(f'concave-span redraw exhausted for {cls.__name__} at {start:g} m: '
                     'amplitude reached the grid floor without passing max_kink_deg')


def generate_alpine_track(spec, *, seed, name=None):
    """Draws one alpine scree climb; ``(spec, seed)`` fully determines the result."""
    if not isinstance(spec, AlpineGenSpec):
        raise ValueError('expected an AlpineGenSpec')
    if type(seed) is not int or seed < 0:
        raise ValueError('terrain seed must be a nonnegative integer')
    rng = np.random.default_rng(seed)
    lo, hi = spec.lead_in_m, _r(spec.length_m - spec.lead_out_m, 2)
    grade, ramps = _rhythm(spec, rng)
    taken, used, obstacles, sections = [], [], [], []

    for _ in range(_count(rng, spec.n_rock_gardens)):
        wanted = _r(rng.uniform(*spec.rock_len_m), 2)
        amp, corr = _r(rng.uniform(*spec.rock_amp_m)), _r(rng.uniform(*spec.rock_corr_m), 3)
        placed = place(rng, wanted, lo, hi, taken, shrink=True)
        if placed is not None:
            start, length = placed
            obstacles.append(_faceted(spec, rng, RockGarden, start, length, amp, corr))
            taken.append((start, start + length))

    for _ in range(_count(rng, spec.n_scree)):
        wanted = _r(rng.uniform(*spec.scree_len_m), 2)
        amp, corr = _r(rng.uniform(*spec.scree_amp_m)), _r(rng.uniform(*spec.scree_corr_m), 3)
        a, b = ramps[int(rng.integers(0, len(ramps)))] if rng.uniform() < spec.scree_ramp_bias else (lo, hi)
        placed = place(rng, wanted, a, b, taken, shrink=True)
        if placed is not None:
            start, length = placed
            obstacles.append(_faceted(spec, rng, RoadRoughness, start, length, amp, corr))
            sections.append(SurfaceSection(start, _r(start + length, 2), 'scree'))
            taken.append((start, start + length))
            used.append((start, start + length))

    for _ in range(_count(rng, spec.n_boulders)):
        height, length = _r(rng.uniform(*spec.boulder_height_m)), _r(rng.uniform(*spec.boulder_len_m), 2)
        if spec.single_contact_safe:
            length = max(length, _r(ceil(min_bump_length_m(height, spec.min_contact_radius_m) * 100.) / 100., 2))
        placed = place(rng, length, lo, hi, taken)
        if placed is not None:
            start, _ = placed
            obstacles.append(Bump(start_m=start, height_m=height, bump_length_m=length))
            taken.append((start, start + length))

    for _ in range(_count(rng, spec.n_water_bars)):
        depth, length = _r(rng.uniform(*spec.water_bar_depth_m)), _r(rng.uniform(*spec.water_bar_len_m), 2)
        if spec.single_contact_safe:
            depth = min(depth, _r(max_bowl_depth_m(length, spec.min_contact_radius_m) * .98))
        placed = place(rng, length, lo, hi, taken)
        if placed is not None:
            start, _ = placed
            obstacles.append(BowlPothole(start_m=start, depth_m=depth, hole_length_m=length))
            taken.append((start, start + length))

    for _ in range(_count(rng, spec.n_sharp_edges)):
        height, length = _r(rng.uniform(*spec.sharp_edge_height_m)), _r(rng.uniform(.2, .6), 2)
        placed = place(rng, length, lo, hi, taken)
        if placed is not None:
            start, _ = placed
            obstacles.append(SquareEdge(start_m=start, height_m=height, ledge_length_m=length))
            taken.append((start, start + length))

    for _ in range(_count(rng, spec.n_wet)):
        wanted = _r(rng.uniform(*spec.wet_len_m), 2)
        placed = place(rng, wanted, lo, hi, used, shrink=True)
        if placed is not None:
            start, length = placed
            sections.append(SurfaceSection(start, _r(start + length, 2), 'wet'))
            used.append((start, start + length))

    amp, corr = _r(rng.uniform(*spec.background_amp_m)), _r(rng.uniform(*spec.background_corr_m), 3)
    for a, b in free_intervals((lo, hi), taken, 0.):
        if b - a >= MIN_BACKGROUND_CORRELATIONS * corr:
            # Trim float noise from the segment length without crossing into the neighbour.
            length = round(b - a, 6)
            while a + length > b:
                length = round(length - 1e-6, 6)
            obstacles.append(_faceted(spec, rng, RoadRoughness, a, length, amp, corr))

    track = TrackSpec(name or f'alpine_{seed}', spec.length_m, obstacles=obstacles, surface=spec.base_surface,
                      grade_profile=grade, surface_sections=tuple(sorted(sections, key=lambda s: s.start_m)),
                      description=f'Alpine scree climb; seed={seed}; spec_sha256={spec.sha256()}')
    track.validate()
    return track


def alpine_scree(seed=ALPINE_PRESET_SEED):
    """Builds the ``alpine_scree`` preset: the default spec at the pinned seed, or another seed."""
    name = 'alpine_scree' if seed == ALPINE_PRESET_SEED else f'alpine_scree_{seed}'
    return generate_alpine_track(AlpineGenSpec(), seed=seed, name=name)


__all__ = ['ALPINE_PRESET_SEED', 'AlpineGenSpec', 'generate_alpine_track', 'alpine_scree']
