"""Closed-form single-contact guards reproduce the rolling-circle spike (spec section 4)."""
from math import atan, degrees

import numpy as np
import pytest
import _golden

from bike_sim.terrain.contact_safety import (
    concave_span_deg, max_bowl_depth_m, min_bump_length_m, single_contact_hazards,
)
from bike_sim.terrain.obstacles import BowlPothole, Bump, RoadRoughness, RockGarden, SlopedPothole, SquareEdge
from bike_sim.terrain.profile import TrackSpec


def test_bump_length_floor_matches_spike():
    floor_10 = min_bump_length_m(0.10)
    floor_05 = min_bump_length_m(0.05)
    floor_15 = min_bump_length_m(0.15)
    _golden.record(np.float64(floor_10), 'bump floor 0.10')
    _golden.record(np.float64(floor_05), 'bump floor 0.05')
    _golden.record(np.float64(floor_15), 'bump floor 0.15')
    assert floor_10 == pytest.approx(0.9425, abs=1e-4)
    assert floor_05 < 0.67
    assert floor_15 < 1.16


def test_bowl_depth_ceiling_matches_spike():
    depth_08 = max_bowl_depth_m(0.8)
    depth_12 = max_bowl_depth_m(1.2)
    depth_15 = max_bowl_depth_m(1.5)
    _golden.record(np.float64(depth_08), 'bowl depth 0.8')
    _golden.record(np.float64(depth_12), 'bowl depth 1.2')
    _golden.record(np.float64(depth_15), 'bowl depth 1.5')
    assert depth_08 == pytest.approx(0.07205, abs=1e-5)
    assert 0.07 <= depth_08 < 0.10
    assert 0.15 <= depth_12
    assert 0.20 <= depth_15


def garden(amp, corr, seed, length=8., start=0.):
    return RockGarden(start_m=start, section_length_m=length, amplitude_m=amp, correlation_length_m=corr, seed=seed)


def _garden_span(obstacle, label):
    # The smoothed node grid is a platform oracle (scipy's correlate loop
    # lowers differently across lanes by ~1 ulp): record the concrete
    # facets so the C++ port replays this platform's inputs through the
    # raw-facet span overload instead of regenerating the garden.
    s, z = obstacle._nodes()
    _golden.record(np.asarray(s, dtype=np.float64), f'input.{label}.s')
    _golden.record(np.asarray(z, dtype=np.float64), f'input.{label}.z')
    return concave_span_deg(obstacle)


def test_concave_span_flags_the_dirty_spike_seed_and_passes_clean_ones():
    dirty = _garden_span(garden(0.07, 0.45, 5), 'dirty')
    clean = _garden_span(garden(0.07, 0.45, 0), 'clean')
    spans_a = [_garden_span(garden(0.03, 0.25, s), f'0.03 s{s}') for s in range(6)]
    spans_b = [_garden_span(garden(0.05, 0.30, s), f'0.05 s{s}') for s in range(6)]
    spans_c = [_garden_span(garden(0.10, 0.15, s), f'0.10 s{s}') for s in range(6)]
    _golden.record(np.float64(dirty), 'span dirty')
    _golden.record(np.float64(clean), 'span clean')
    for i, span in enumerate(spans_a):
        _golden.record(np.float64(span), f'span 0.03 s{i}')
    for i, span in enumerate(spans_b):
        _golden.record(np.float64(span), f'span 0.05 s{i}')
    for i, span in enumerate(spans_c):
        _golden.record(np.float64(span), f'span 0.10 s{i}')
    assert dirty >= 20.
    assert clean < 20.
    assert max(spans_a) < 20.
    assert max(spans_b) < 20.
    assert max(spans_c) >= 20.


def test_concave_span_ignores_convex_crests():
    # This seed's sharpest kink (20.46 deg between adjacent facets) is a crest; the rolling
    # spike found it clean, so the concave-only span must stay below 20 deg.
    span = _garden_span(garden(0.05, 0.30, 3), 'crest')
    _golden.record(np.float64(span), 'span crest')
    assert span < 20.


class StubGarden:
    """Minimal ``_nodes()`` stand-in: crafted facets without RockGarden's taper/noise."""

    def __init__(self, s, z):
        self._s = np.asarray(s, dtype=float)
        self._z = np.asarray(z, dtype=float)

    def _nodes(self):
        return self._s, self._z


# Facet mids at 0.10, 0.40, 1.00, 1.60, 2.10, 2.70 -- every pair of adjacent mids is more
# than the 0.25 m window apart, so interior pairs contribute nothing and boundary joins
# are isolated. Only the first facet mid (0.10) sits in the entry window; no mid reaches
# the exit window (s[-1] - 0.25 = 2.75).
S = [0.0, 0.2, 0.6, 1.4, 1.8, 2.4, 3.0]


def test_concave_span_counts_a_steep_entry_join():
    # First facet rises at atan(0.4) ~ 21.8 deg off flat ground; the rest are gentle.
    stub = StubGarden(S, [0.0, 0.08, 0.10, 0.10, 0.10, 0.05, 0.0])
    span = concave_span_deg(stub)
    _golden.record(np.float64(span), 'span entry concave')
    assert span == pytest.approx(degrees(atan(0.4)), abs=1e-9)
    assert span >= 20.


def test_concave_span_ignores_a_convex_entry_join():
    # First facet descends at atan(-0.4) ~ -21.8 deg: a drop-off edge, not a concave join.
    # With no other kink anywhere, the span must stay on the floor.
    stub = StubGarden(S, [0.0, -0.08, -0.10, -0.10, -0.10, -0.05, 0.0])
    span = concave_span_deg(stub)
    _golden.record(np.float64(span), 'span entry convex')
    assert span == pytest.approx(0., abs=1e-9)


def test_concave_span_control_gentle_facets_stay_clean():
    # All facets under ~6 deg, joins included: nothing for the guard to report.
    stub = StubGarden(S, [0.0, 0.01, 0.02, 0.02, 0.02, 0.01, 0.0])
    span = concave_span_deg(stub)
    _golden.record(np.float64(span), 'span gentle')
    assert span < 17.


# Same isolation trick mirrored to the far end: mids at 0.30, 1.00, 1.70, 2.30, 2.80 --
# all interior gaps exceed the window, no mid reaches the entry window (s[0] + 0.25),
# and only the last facet mid (2.80) sits in the exit window (s[-1] - 0.25 = 2.75).
S_EXIT = [0.0, 0.6, 1.4, 2.0, 2.6, 3.0]


def test_concave_span_counts_a_steep_exit_join_and_ignores_a_convex_one():
    # Final facet descends at atan(0.4) ~ 21.8 deg onto flat ground: a concave exit
    # join, counted. Everything else is gentle or unpaired, so the exit term drives it.
    stub = StubGarden(S_EXIT, [0.0, 0.02, 0.16, 0.16, 0.16, 0.0])
    concave = concave_span_deg(stub)
    _golden.record(np.float64(concave), 'span exit concave')
    assert concave == pytest.approx(degrees(atan(0.4)), abs=1e-9)
    # A final facet rising at atan(0.4) to meet flat ground is a convex exit join:
    # the contribution is negative, so it must not flag.
    stub = StubGarden(S_EXIT, [0.0, -0.02, -0.16, -0.16, -0.16, 0.0])
    convex = concave_span_deg(stub)
    _golden.record(np.float64(convex), 'span exit convex')
    assert convex == pytest.approx(0., abs=1e-9)


def test_hazards_report_every_unsafe_obstacle_kind():
    track = TrackSpec('t', 60., obstacles=[
        Bump(start_m=5., height_m=.10, bump_length_m=.66),
        Bump(start_m=8., height_m=.10, bump_length_m=.95),
        BowlPothole(start_m=12., depth_m=.10, hole_length_m=.8),
        BowlPothole(start_m=16., depth_m=.07, hole_length_m=.8),
        SlopedPothole(start_m=20., depth_m=.05, hole_length_m=.8),
        SquareEdge(start_m=24., height_m=.05, ledge_length_m=.3),
        garden(0.07, 0.45, 5, start=30.),
        RoadRoughness(start_m=40., section_length_m=8., amplitude_m=.01, correlation_length_m=.10, seed=0),
    ])
    hazards = single_contact_hazards(track)
    for i, hazard in enumerate(hazards):
        _golden.record(np.frombuffer(hazard.encode(), np.uint8),
                       f'hazard {i}')
    labels = [h.split(':')[0] for h in hazards]
    assert labels == ['Bump@5m', 'BowlPothole@12m', 'SlopedPothole@20m', 'SquareEdge@24m', 'RockGarden@30m']


def test_clean_track_has_no_hazards():
    track = TrackSpec('t', 30., obstacles=[Bump(start_m=5., height_m=.05, bump_length_m=.7),
                                          garden(0.03, 0.25, 2, start=10.)])
    assert single_contact_hazards(track) == []
