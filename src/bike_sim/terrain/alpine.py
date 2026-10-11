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

_RANGE_FIELDS = ('ramp_len_m', 'ramp_grade', 'shelf_len_m', 'shelf_grade', 'wall_grade', 'counter_len_m',
                 'scree_len_m', 'scree_amp_m', 'scree_corr_m', 'wet_len_m', 'background_amp_m',
                 'background_corr_m', 'boulder_height_m', 'boulder_len_m', 'rock_len_m', 'rock_amp_m',
                 'rock_corr_m', 'water_bar_depth_m', 'water_bar_len_m', 'sharp_edge_height_m')
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
            lo, hi = check_range(getattr(self, name), name)
            object.__setattr__(self, name, (float(lo), float(hi)))
        lo, hi = check_range(self.counter_grade, 'counter_grade', minimum=-1.)
        if hi >= 0.:
            raise ValueError('counter_grade must be negative: it is a short descent')
        object.__setattr__(self, 'counter_grade', (float(lo), float(hi)))
        for name in _COUNT_FIELDS:
            check_range(getattr(self, name), name, integer=True)
        for name in _POSITIVE_FIELDS:
            scalar(getattr(self, name), name, positive=True)
        for name in ('counter_prob', 'scree_ramp_bias'):
            if scalar(getattr(self, name), name, minimum=0.) > 1.:
                raise ValueError(f'{name} must lie in [0, 1]')
        if type(self.n_walls) is not int or not 0 <= self.n_walls <= 1:
            raise ValueError('n_walls must be 0 or 1')
        if type(self.single_contact_safe) is not bool:
            raise ValueError('single_contact_safe must be a bool')
        if self.wall_grade[0] < self.ramp_grade[1]:
            raise ValueError('wall_grade must start at or above the top of ramp_grade')
        if self.base_surface not in SURFACES:
            raise ValueError(f'unknown base_surface {self.base_surface!r}; available: {", ".join(SURFACES)}')
        if self.lead_in_m + self.lead_out_m + 2. * self.transition_m + self.ramp_len_m[0] > self.length_m:
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
