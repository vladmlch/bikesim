# tests/reference/test_pneumatic_tire_config.py
"""The ``tires.backend = "pneumatic"`` config plumbing: section, gates, TOML merge."""
import pytest

from bike_sim.physics.physical_config import PneumaticTireConfig, TireBackendConfig
from bike_sim.physics.resolution import resolve_physics_config
from bike_sim.physics.tyre import FRONT_TYRE, REAR_TYRE, TIERS


def test_pneumatic_section_defaults_to_fast_stock_tyres():
    cfg = PneumaticTireConfig()
    assert cfg.tier == 'fast' and cfg.surface is None
    assert cfg.front == FRONT_TYRE and cfg.rear == REAR_TYRE
    assert cfg.tier in TIERS


def test_pneumatic_is_opt_in_and_the_section_always_resolves():
    cfg = TireBackendConfig()
    assert cfg.backend == 'native_reference'
    assert isinstance(cfg.pneumatic, PneumaticTireConfig)
    assert cfg.pneumatic.front == FRONT_TYRE


def test_pneumatic_backend_resolves_from_toml():
    cfg = resolve_physics_config({'physics_mode': 'physical', 'tires': {
        'backend': 'pneumatic',
        'pneumatic': {'surface': 'asphalt'}}})
    tires = cfg.tires
    assert tires.backend == 'pneumatic' and tires.pneumatic.surface == 'asphalt'
    assert tires.pneumatic.tier == 'fast'
    assert tires.pneumatic.front == FRONT_TYRE and tires.pneumatic.rear == REAR_TYRE


def test_pneumatic_backend_resolves_under_track_surface_mode():
    cfg = resolve_physics_config({'physics_mode': 'physical', 'tires': {
        'backend': 'pneumatic',
        'surface_mode': 'track',
        'pneumatic': {'tier': 'detailed'}}})
    assert cfg.tires.surface_mode == 'track' and cfg.tires.pneumatic.surface is None
    assert cfg.tires.pneumatic.tier == 'detailed'


def test_partial_tyre_tables_extend_the_default_specs():
    cfg = resolve_physics_config({'physics_mode': 'physical', 'tires': {
        'backend': 'pneumatic',
        'pneumatic': {'surface': 'asphalt',
                      'front': {'pressure_bar': 1.9},
                      'rear': {'pressure_bar': 2.1, 'name': 'custom rear'}}}})
    front, rear = cfg.tires.pneumatic.front, cfg.tires.pneumatic.rear
    assert front.pressure_bar == 1.9 and front.name == FRONT_TYRE.name
    assert front.outer_radius_mm == FRONT_TYRE.outer_radius_mm
    assert front.area_factor == FRONT_TYRE.area_factor
    assert rear.pressure_bar == 2.1 and rear.name == 'custom rear'
    assert rear.outer_radius_mm == REAR_TYRE.outer_radius_mm


def test_pneumatic_rejects_configured_mode_without_a_surface():
    with pytest.raises(ValueError, match='pneumatic'):
        resolve_physics_config({'tires': {'backend': 'pneumatic'}})
    with pytest.raises(ValueError, match='surface'):
        resolve_physics_config({'tires': {'backend': 'pneumatic',
                                          'surface_mode': 'configured',
                                          'pneumatic': {'tier': 'fast'}}})


def test_pneumatic_rejects_an_unknown_tier():
    with pytest.raises(ValueError, match='tier'):
        resolve_physics_config({'tires': {'backend': 'pneumatic',
                                          'pneumatic': {'surface': 'asphalt',
                                                        'tier': 'ultra'}}})
    with pytest.raises(ValueError, match='tier'):
        PneumaticTireConfig(tier='ultra')


def test_pneumatic_rejects_an_unknown_surface_name():
    with pytest.raises(ValueError, match='surface'):
        resolve_physics_config({'tires': {'backend': 'pneumatic',
                                          'pneumatic': {'surface': 'ice'}}})
    with pytest.raises(ValueError, match='surface'):
        PneumaticTireConfig(surface='ice')


def test_pneumatic_rejects_an_unresolved_section():
    with pytest.raises(ValueError, match='pneumatic tire configuration'):
        TireBackendConfig(backend='pneumatic', pneumatic=None)
    with pytest.raises(ValueError, match='pneumatic tire configuration'):
        TireBackendConfig(backend='pneumatic', pneumatic={'surface': 'asphalt'})
    with pytest.raises(ValueError, match='requires a TOML table'):
        resolve_physics_config({'tires': {'backend': 'pneumatic',
                                          'pneumatic': 'asphalt'}})


def test_pneumatic_rejects_unknown_tyre_keys():
    with pytest.raises(ValueError, match='unknown TyreSpecs parameter'):
        resolve_physics_config({'tires': {'backend': 'pneumatic',
                                          'pneumatic': {'surface': 'asphalt',
                                                        'front': {'pressure_baar': 1.9}}}})


def test_existing_backend_gates_are_unchanged():
    with pytest.raises(ValueError, match='unknown physical tire backend'):
        TireBackendConfig(backend='pneumtic')
    with pytest.raises(ValueError, match='track material resolution requires compliant_2d tires'):
        TireBackendConfig(backend='native_reference', surface_mode='track')
    with pytest.raises(ValueError, match='unknown TireBackendConfig parameter'):
        resolve_physics_config({'tires': {'pneumtic': {}}})
