"""Savage isolates climb power and geometry with asphalt end to end."""
from pathlib import Path

from bike_sim.terrain.trackfile import load_track


def test_savage_is_asphalt_everywhere():
    track = load_track(Path('examples/research/rough_uphill_savage.toml'))
    assert track.surface == 'asphalt'
    assert track.surface_sections == ()
    assert track.length_m == 120.0
