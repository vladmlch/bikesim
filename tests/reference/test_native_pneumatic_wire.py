"""The pneumatic backend's native wire: projection shape, schema
round-trip, the configured/track surface modes, and the C++ parser
boundary.

``project()`` is exercised end-to-end on a real physical RideSimulation;
the structural and rejection checks mutate the emitted ``tire`` section
and re-run ``validate_config``, and the ``selected_native_parser_*``
tests pin where the selected artifact accepts the wire — the C++ config
layer (Task 6) parses and validates it while the pneumatic writer lands
with the Stepper dispatch task.
"""
import copy
from dataclasses import asdict

import numpy as np
import pytest

from native_loader import load_native

from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.physical_config import (
    PneumaticTireConfig, TireBackendConfig)
from bike_sim.physics.tyre import FRONT_TYRE, REAR_TYRE, TIERS
from bike_sim.sim.ride.tyre.geometry import RoadProfile
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset
from bike_sim.terrain.surface import SurfaceMap, get_surface
from tools.native_config import project, validate_config, _surface_spec

TYRE_SPEC_KEYS = tuple(asdict(FRONT_TYRE))


def _sim(pneumatic=None, surface_mode='track', **tires_kwargs):
    cfg = SimulationPhysicsConfig(
        'physical', drive_mode='coast',
        tires=TireBackendConfig(backend='pneumatic',
                                surface_mode=surface_mode,
                                pneumatic=(PneumaticTireConfig()
                                           if pneumatic is None else pneumatic),
                                **tires_kwargs))
    sim = RideSimulation(track=get_preset('flat'), rider='lumped',
                         physics_config=cfg)
    # A coasting lumped rider does no muscle or motor work; the strict
    # monitor's absolute 1e-8 J zero-source budget fails on
    # linkage-closure numerics -- identically under compliant_2d.
    sim.physical.set_strict(False)
    return sim


@pytest.fixture(scope='module')
def track_sim():
    return _sim()


@pytest.fixture(scope='module')
def track_wire(track_sim):
    return project(track_sim)


def _profile():
    return RoadProfile.from_samples(np.array([0., .5, 1., 1.5]),
                                    np.array([.4, .42, .38, .4]))


def _section(pneumatic=None, surface_mode='track', surface_map=None):
    tires = TireBackendConfig(
        backend='pneumatic', surface_mode=surface_mode,
        pneumatic=PneumaticTireConfig() if pneumatic is None else pneumatic)
    if surface_mode == 'track' and surface_map is None:
        surface_map = SurfaceMap.uniform('hardpack')
    from tools.native_config import _tire_section
    return _tire_section(tires, surface_map, _profile())


def _wire(section):
    return {'schema': 2, 'tire': section}


def _assert_builtin_leaves(value, path='config'):
    """Every wire leaf is a builtin int/float/bool/str or None — nothing
    numpy crosses nanobind."""
    if isinstance(value, dict):
        for key, item in value.items():
            assert type(key) is str, f'{path}: non-string key'
            _assert_builtin_leaves(item, f'{path}.{key}')
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            _assert_builtin_leaves(item, f'{path}.{index}')
    else:
        assert type(value) in (int, float, bool, str) or value is None, \
            f'{path}: {type(value).__name__}'


def test_pneumatic_projection_carries_the_full_section(track_sim, track_wire):
    tire = track_wire['tire']
    assert tire['backend'] == 'pneumatic' and tire['surface_mode'] == 'track'
    # The TireParameters sides keep the shared tangential fields; the
    # analytic material belongs to compliant_2d only.
    for side in ('front', 'rear'):
        assert set(tire[side]) == {'tangent_k_n_m', 'mu', 'relaxation_length_m'}
    assert 'distributed' not in tire
    pneu = tire['pneumatic']
    assert set(pneu) == {'tier', 'n_rays', 'half_angle_deg',
                       'discretised_brush', 'timestep_s',
                       'front', 'rear', 'profile'}
    # The named surface stays off the wire while pneumatic.surface is None.
    assert 'surface' not in pneu
    tier = TIERS['fast']
    assert pneu['tier'] == 'fast'
    assert type(pneu['n_rays']) is int and pneu['n_rays'] == tier.n_rays
    assert pneu['discretised_brush'] is tier.discretised_brush
    assert type(pneu['half_angle_deg']) is float
    assert pneu['half_angle_deg'] == tier.half_angle_deg
    assert pneu['timestep_s'] == tier.timestep_s
    # Each side carries the 18 TyreSpecs dataclass fields — nothing else.
    assert pneu['front'] == asdict(FRONT_TYRE)
    assert pneu['rear'] == asdict(REAR_TYRE)
    for side in ('front', 'rear'):
        assert tuple(pneu[side]) == TYRE_SPEC_KEYS
        assert 'rim_strike_deflection_mm' not in pneu[side]
    _assert_builtin_leaves(track_wire)


def test_pneumatic_profile_matches_the_appliers_road_bitwise(
        track_sim, track_wire):
    profile = track_wire['tire']['pneumatic']['profile']
    assert set(profile) == {'x0_m', 'dx_m', 'z_m'}
    assert type(profile['x0_m']) is float and type(profile['dx_m']) is float
    assert type(profile['z_m']) is list
    assert all(type(z) is float for z in profile['z_m'])
    applier_profile = track_sim.physical.tire.profile
    assert profile['x0_m'] == applier_profile.x0_m
    assert profile['dx_m'] == applier_profile.dx_m
    assert np.array_equal(np.asarray(profile['z_m']), applier_profile.z_m)


def test_pneumatic_track_mode_keeps_the_surface_map(track_wire):
    tire = track_wire['tire']
    assert set(tire['surface_map']) == {'surface', 'sections'}
    assert tire['surface_map']['surface']['name'] == get_preset('flat').surface
    validate_config(track_wire)


def test_pneumatic_wire_round_trips_through_the_schema(track_wire):
    validate_config(track_wire)


def test_configured_surface_projects_the_resolved_spec():
    """surface_mode='configured' + a named pneumatic surface ships the
    resolved SurfaceSpec under tire.pneumatic.surface and no surface_map —
    the native side builds SurfaceMap.uniform from it."""
    section = _section(pneumatic=PneumaticTireConfig(surface='asphalt'),
                       surface_mode='configured')
    assert 'surface_map' not in section
    assert section['pneumatic']['surface'] == _surface_spec(
        get_surface('asphalt'))
    validate_config(_wire(section))


def test_configured_surface_projects_through_a_real_sim():
    sim = _sim(pneumatic=PneumaticTireConfig(surface='wet'),
               surface_mode='configured')
    wire = project(sim)
    tire = wire['tire']
    assert 'surface_map' not in tire
    assert tire['pneumatic']['surface']['name'] == 'wet'
    validate_config(wire)


def test_named_surface_overrides_the_track_map_on_the_wire():
    """A named surface under surface_mode='track' ships BOTH: the map the
    runtime passed and the pneumatic.surface the applier prefers."""
    section = _section(pneumatic=PneumaticTireConfig(surface='scree'))
    assert 'surface_map' in section
    assert section['pneumatic']['surface']['name'] == 'scree'
    validate_config(_wire(section))


def test_detailed_tier_projects_its_resolved_parameters():
    section = _section(pneumatic=PneumaticTireConfig(tier='detailed'))
    pneu = section['pneumatic']
    tier = TIERS['detailed']
    assert pneu['tier'] == 'detailed'
    assert pneu['n_rays'] == tier.n_rays == 256
    assert pneu['discretised_brush'] is True
    assert pneu['timestep_s'] == tier.timestep_s
    validate_config(_wire(section))


def test_pneumatic_section_requires_the_road_profile():
    from tools.native_config import _tire_section
    tires = TireBackendConfig(backend='pneumatic', surface_mode='track',
                              pneumatic=PneumaticTireConfig())
    with pytest.raises(ValueError, match='RoadProfile'):
        _tire_section(tires, SurfaceMap.uniform('hardpack'), None)


def test_projection_rejects_a_non_pneumatic_section():
    from tools.native_config import _pneumatic_section
    with pytest.raises(ValueError, match='PneumaticTireConfig'):
        _pneumatic_section(None, _profile())


def _good_track_wire():
    return _wire(_section())


@pytest.mark.parametrize('mutate,match', [
    # Presence rules: the section is required iff backend == 'pneumatic'.
    (lambda c: c['tire'].pop('pneumatic'), 'tire.pneumatic'),
    (lambda c: c['tire'].__setitem__('backend', 'compliant_2d'),
     'tire.pneumatic'),
    # The compliant-only material stays forbidden on the sides.
    (lambda c: c['tire']['front'].__setitem__(
        'material', {'radial_k_n_m': 1., 'radial_c_ns_m': 1.,
                     'pressure_pa_gauge': 1., 'provenance': 'x',
                     'valid_load_range_n': [0., 1.]}),
     'tire.front.material'),
    # Section shape: unknown keys, missing required keys, and the one
    # nullable-'profile' skip not extending to this schema.
    (lambda c: c['tire']['pneumatic'].__setitem__('misspelled', 1.),
     'tire.pneumatic.misspelled'),
    (lambda c: c['tire']['pneumatic'].pop('profile'),
     'tire.pneumatic.profile'),
    (lambda c: c['tire']['pneumatic'].__setitem__('profile', None),
     'tire.pneumatic.profile'),
    (lambda c: c['tire']['pneumatic'].__setitem__('profile', {}),
     'tire.pneumatic.profile'),
    # Resolved tier values must match the registry row.
    (lambda c: c['tire']['pneumatic'].__setitem__('tier', 'ultra'),
     'tire.pneumatic.tier'),
    (lambda c: c['tire']['pneumatic'].__setitem__('n_rays', 128),
     'tire.pneumatic.n_rays'),
    (lambda c: c['tire']['pneumatic'].__setitem__('n_rays', 64.5),
     'tire.pneumatic.n_rays'),
    (lambda c: c['tire']['pneumatic'].__setitem__('discretised_brush', 'yes'),
     'tire.pneumatic.discretised_brush'),
    (lambda c: c['tire']['pneumatic'].__setitem__('timestep_s', .001),
     'tire.pneumatic.timestep_s'),
    # TyreSpecs domains: missing field, bad geometry, bad factors.
    (lambda c: c['tire']['pneumatic']['front'].pop('pressure_bar'),
     'tire.pneumatic.front'),
    (lambda c: c['tire']['pneumatic']['front'].__setitem__('pressure_bar', -1.),
     'tire.pneumatic.front.pressure_bar'),
    (lambda c: c['tire']['pneumatic']['front'].__setitem__('rim_radius_mm', 400.),
     'tire.pneumatic.front.rim_radius_mm'),
    (lambda c: c['tire']['pneumatic']['front'].__setitem__('area_factor', 1.5),
     'tire.pneumatic.front'),
    (lambda c: c['tire']['pneumatic']['front'].__setitem__('loss_factor', 1.),
     'tire.pneumatic.front.loss_factor'),
    (lambda c: c['tire']['pneumatic']['front'].__setitem__('extra', 0.),
     'tire.pneumatic.front'),
    # The road profile holds the uniform-grid invariants.
    (lambda c: c['tire']['pneumatic']['profile'].__setitem__('dx_m', 0.),
     'profile.dx_m'),
    (lambda c: c['tire']['pneumatic']['profile'].__setitem__('z_m', []),
     'profile.z_m'),
    (lambda c: c['tire']['pneumatic']['profile'].__setitem__('z_m', [.4]),
     'profile.z_m'),
    (lambda c: c['tire']['pneumatic']['profile']['z_m'].__setitem__(1, np.nan),
     'profile.z_m'),
    (lambda c: c['tire']['pneumatic']['profile'].__setitem__('z_m', 1.),
     'profile.z_m'),
])
def test_pneumatic_wire_rejection_cases(mutate, match):
    wire = _good_track_wire()
    mutate(wire)
    with pytest.raises(ValueError, match=match):
        validate_config(wire)


def test_pneumatic_configured_wire_requires_the_surface():
    wire = _wire(_section(pneumatic=PneumaticTireConfig(surface='asphalt'),
                          surface_mode='configured'))
    del wire['tire']['pneumatic']['surface']
    with pytest.raises(ValueError, match='tire.pneumatic.surface'):
        validate_config(wire)


def test_pneumatic_configured_wire_rejects_a_malformed_surface():
    wire = _wire(_section(pneumatic=PneumaticTireConfig(surface='asphalt'),
                          surface_mode='configured'))
    del wire['tire']['pneumatic']['surface']['mu_slide']
    with pytest.raises(ValueError, match='mu_slide'):
        validate_config(wire)


def test_pneumatic_track_wire_still_requires_the_map():
    wire = _good_track_wire()
    del wire['tire']['surface_map']
    with pytest.raises(ValueError, match='tire.surface_map'):
        validate_config(wire)


def test_pneumatic_backend_reaches_the_backend_allow_list():
    wire = _good_track_wire()
    wire['tire']['backend'] = 'pneumtic'
    with pytest.raises(ValueError, match='tire.backend'):
        validate_config(wire)
    wire['tire']['backend'] = 'native_reference'
    with pytest.raises(ValueError, match='tire.backend'):
        validate_config(wire)


def test_selected_native_parser_accepts_the_pneumatic_wire(
        track_sim, track_wire, tmp_path):
    """Task 6 lands the C++ parser: the selected artifact parses and
    domain-validates the pneumatic wire, so construction proceeds past
    the config layer and stops at the writer boundary — a RuntimeError
    naming the missing pneumatic writer (the Stepper dispatch lands with
    Task 11) instead of a parser ValueError."""
    import mujoco
    mjb = tmp_path / 'model.mjb'
    mujoco.mj_saveModel(track_sim.model, str(mjb))
    with pytest.raises(RuntimeError, match='pneumatic'):
        load_native().Stepper(str(mjb), track_wire)


@pytest.mark.parametrize('mutate,match', [
    # Presence rules: the section is required iff backend == 'pneumatic'.
    (lambda c: c['tire'].pop('pneumatic'), 'tire.pneumatic'),
    (lambda c: c['tire'].__setitem__('backend', 'compliant_2d'),
     'tire.pneumatic'),
    # The compliant-only material stays forbidden on the sides.
    (lambda c: c['tire']['front'].__setitem__(
        'material', {'radial_k_n_m': 1., 'radial_c_ns_m': 1.,
                     'pressure_pa_gauge': 1., 'provenance': 'x',
                     'valid_load_range_n': [0., 1.]}),
     'tire.front.material'),
    # Section shape and resolved-tier integrity.
    (lambda c: c['tire']['pneumatic'].__setitem__('misspelled', 1.),
     'tire.pneumatic.misspelled'),
    (lambda c: c['tire']['pneumatic'].__setitem__('tier', 'ultra'),
     'tire.pneumatic.tier'),
    (lambda c: c['tire']['pneumatic'].__setitem__('n_rays', 128),
     'tire.pneumatic.n_rays'),
    (lambda c: c['tire']['pneumatic'].__setitem__('discretised_brush', 'yes'),
     'tire.pneumatic.discretised_brush'),
    (lambda c: c['tire']['pneumatic'].__setitem__('timestep_s', .001),
     'tire.pneumatic.timestep_s'),
    # TyreSpecs domains on both sides.
    (lambda c: c['tire']['pneumatic']['front'].__setitem__('pressure_bar', -1.),
     'tire.pneumatic.front.pressure_bar'),
    (lambda c: c['tire']['pneumatic']['front'].__setitem__('rim_radius_mm', 400.),
     'tire.pneumatic.front.rim_radius_mm'),
    (lambda c: c['tire']['pneumatic']['rear'].__setitem__('area_factor', 1.5),
     'tire.pneumatic.rear'),
    (lambda c: c['tire']['pneumatic']['rear'].__setitem__('loss_factor', 1.),
     'tire.pneumatic.rear.loss_factor'),
    # The road profile holds the uniform-grid invariants.
    (lambda c: c['tire']['pneumatic']['profile'].__setitem__('dx_m', 0.),
     'profile.dx_m'),
    (lambda c: c['tire']['pneumatic']['profile'].__setitem__('z_m', []),
     'profile.z_m'),
    (lambda c: c['tire']['pneumatic']['profile'].__setitem__('z_m', [.4]),
     'profile.z_m'),
    (lambda c: c['tire']['pneumatic']['profile'].__setitem__('z_m', [.4, np.nan]),
     'profile.z_m'),
    # A configured (non-track) surface must ship resolved in the section.
    (lambda c: c['tire'].__setitem__('surface_mode', 'configured'),
     'tire.pneumatic.surface'),
])
def test_selected_native_parser_rejects_malformed_pneumatic_wires(
        track_sim, track_wire, tmp_path, mutate, match):
    """Malformed pneumatic wires still fail closed at the selected
    artifact — a ValueError naming the offending field, raised by the
    parser or the shared config validator before the writer boundary."""
    import mujoco
    mjb = tmp_path / 'model.mjb'
    mujoco.mj_saveModel(track_sim.model, str(mjb))
    wire = copy.deepcopy(track_wire)
    mutate(wire)
    with pytest.raises(ValueError, match=match):
        load_native().Stepper(str(mjb), wire)
