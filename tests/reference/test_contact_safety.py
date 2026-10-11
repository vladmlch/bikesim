"""Closed-form single-contact guards reproduce the rolling-circle spike (spec section 4)."""
from math import atan, degrees

import numpy as np
import pytest

from bike_sim.terrain.contact_safety import (
    concave_span_deg, max_bowl_depth_m, min_bump_length_m, single_contact_hazards,
)
from bike_sim.terrain.obstacles import BowlPothole, Bump, RoadRoughness, RockGarden, SlopedPothole, SquareEdge
from bike_sim.terrain.profile import TrackSpec


def test_bump_length_floor_matches_spike():
    assert min_bump_length_m(0.10) == pytest.approx(0.9425, abs=1e-4)
    assert min_bump_length_m(0.05) < 0.67
    assert min_bump_length_m(0.15) < 1.16


def test_bowl_depth_ceiling_matches_spike():
    assert max_bowl_depth_m(0.8) == pytest.approx(0.07205, abs=1e-5)
    assert 0.07 <= max_bowl_depth_m(0.8) < 0.10
    assert 0.15 <= max_bowl_depth_m(1.2)
    assert 0.20 <= max_bowl_depth_m(1.5)


def garden(amp, corr, seed, length=8., start=0.):
    return RockGarden(start_m=start, section_length_m=length, amplitude_m=amp, correlation_length_m=corr, seed=seed)


def test_concave_span_flags_the_dirty_spike_seed_and_passes_clean_ones():
    assert concave_span_deg(garden(0.07, 0.45, 5)) >= 20.
    assert concave_span_deg(garden(0.07, 0.45, 0)) < 20.
    assert max(concave_span_deg(garden(0.03, 0.25, s)) for s in range(6)) < 20.
    assert max(concave_span_deg(garden(0.05, 0.30, s)) for s in range(6)) < 20.
    assert max(concave_span_deg(garden(0.10, 0.15, s)) for s in range(6)) >= 20.


def test_concave_span_ignores_convex_crests():
    # This seed's sharpest kink (20.46 deg between adjacent facets) is a crest; the rolling
    # spike found it clean, so the concave-only span must stay below 20 deg.
    assert concave_span_deg(garden(0.05, 0.30, 3)) < 20.


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
    assert concave_span_deg(stub) == pytest.approx(degrees(atan(0.4)), abs=1e-9)
    assert concave_span_deg(stub) >= 20.


def test_concave_span_ignores_a_convex_entry_join():
    # First facet descends at atan(-0.4) ~ -21.8 deg: a drop-off edge, not a concave join.
    # With no other kink anywhere, the span must stay on the floor.
    stub = StubGarden(S, [0.0, -0.08, -0.10, -0.10, -0.10, -0.05, 0.0])
    assert concave_span_deg(stub) == pytest.approx(0., abs=1e-9)


def test_concave_span_control_gentle_facets_stay_clean():
    # All facets under ~6 deg, joins included: nothing for the guard to report.
    stub = StubGarden(S, [0.0, 0.01, 0.02, 0.02, 0.02, 0.01, 0.0])
    assert concave_span_deg(stub) < 17.


# Same isolation trick mirrored to the far end: mids at 0.30, 1.00, 1.70, 2.30, 2.80 --
# all interior gaps exceed the window, no mid reaches the entry window (s[0] + 0.25),
# and only the last facet mid (2.80) sits in the exit window (s[-1] - 0.25 = 2.75).
S_EXIT = [0.0, 0.6, 1.4, 2.0, 2.6, 3.0]


def test_concave_span_counts_a_steep_exit_join_and_ignores_a_convex_one():
    # Final facet descends at atan(0.4) ~ 21.8 deg onto flat ground: a concave exit
    # join, counted. Everything else is gentle or unpaired, so the exit term drives it.
    stub = StubGarden(S_EXIT, [0.0, 0.02, 0.16, 0.16, 0.16, 0.0])
    assert concave_span_deg(stub) == pytest.approx(degrees(atan(0.4)), abs=1e-9)
    # A final facet rising at atan(0.4) to meet flat ground is a convex exit join:
    # the contribution is negative, so it must not flag.
    stub = StubGarden(S_EXIT, [0.0, -0.02, -0.16, -0.16, -0.16, 0.0])
    assert concave_span_deg(stub) == pytest.approx(0., abs=1e-9)


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
    labels = [h.split(':')[0] for h in hazards]
    assert labels == ['Bump@5m', 'BowlPothole@12m', 'SlopedPothole@20m', 'SquareEdge@24m', 'RockGarden@30m']


def test_clean_track_has_no_hazards():
    track = TrackSpec('t', 30., obstacles=[Bump(start_m=5., height_m=.05, bump_length_m=.7),
                                          garden(0.03, 0.25, 2, start=10.)])
    assert single_contact_hazards(track) == []
