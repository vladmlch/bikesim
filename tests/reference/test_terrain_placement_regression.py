"""The procedural generators must stay byte-identical across the placement refactor."""
import hashlib

import pytest

from bike_sim.terrain.generator import TerrainGenSpec, generate_track
from bike_sim.terrain.presets import get_preset
from bike_sim.terrain.trackfile import dump_track

GENERATED = {
    0: 'b298d8170a8391daf50b671f41b9dc7e33855354fe770d3cb894af70400f544d',
    1: '7ac544f21b89f27a03a176e5d017e0a0e175af8ec80119601dd05a652133848a',
    2: '4837293048585ed234201595833d3e8ed7851b6c6e8a384ab41745840c7f5622',
    3: 'be8c67f11a7961b2bcb012612b6d16ed0b72394edcae09f9b12fc0e1b73857c0',
}
ROADS = {
    'road_smooth': 'd05d8a6b82f179bdb80f66ee5cfa71bdb8c74237135456f673e74dfc74eafb1d',
    'road_worn': '455af2ec2c2665629898e2c05b7ae29cf0b2f766a4cf2d9a1e0e77fd0017288c',
    'road_broken': 'f6eb8883c2f83196328a06cb7eaf2d7eb1736f9b5de49206dc25dffcc1aa029e',
}
SPEC_SHA = '45d11d386fb651dff55d7af3292ea3e2a972d9354e1bef677227f2399bfcf8e7'


def digest(track):
    return hashlib.sha256(dump_track(track).encode()).hexdigest()


def test_default_terrain_spec_hash_is_pinned():
    assert TerrainGenSpec().sha256() == SPEC_SHA


@pytest.mark.parametrize('seed', sorted(GENERATED))
def test_generated_tracks_are_byte_identical(seed):
    assert digest(generate_track(TerrainGenSpec(), seed=seed)) == GENERATED[seed]


@pytest.mark.parametrize('name', sorted(ROADS))
def test_road_presets_are_byte_identical(name):
    assert digest(get_preset(name)) == ROADS[name]
