"""Closed-form single-contact guards reproduce the rolling-circle spike (spec section 4)."""
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
