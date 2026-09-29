"""Road grade, obstacles and material must be independent physical inputs."""
import tomllib
import numpy as np
import pytest
from bike_sim.terrain.grade import GradeProfile
from bike_sim.terrain.profile import TrackSpec, build_profile
from bike_sim.terrain.obstacles import SquareEdge
from bike_sim.terrain.surface import SurfaceMap, SurfaceSection, get_surface
from bike_sim.terrain.trackfile import dump_track, track_from_dict, TrackFileError
from bike_sim.physics.physical_config import TireBackendConfig, TireParameters
from bike_sim.sim.ride.tire_forces import effective_friction


def test_grade_is_integral_of_continuous_slope():
    p = GradeProfile(((0., 0.), (2., .2), (4., .2)))
    np.testing.assert_allclose(p.elevation([0., 2., 4.]), [0., .2, .6])
    np.testing.assert_allclose(p.slope([0., 1., 2., 3., 5.]), [0., .1, .2, .2, .2])
    eps = 1e-5
    assert (p.elevation(2.+eps)-p.elevation(2.-eps))/(2*eps) == pytest.approx(.2, abs=1e-6)


def test_obstacle_is_added_to_grade_not_replaced():
    obstacle = SquareEdge(start_m=2., height_m=.05, ledge_length_m=.5)
    flat = TrackSpec('flat', 6., [obstacle])
    grade = GradeProfile(((0., .1), (6., .1)))
    uphill = TrackSpec('hill', 6., [obstacle], grade_profile=grade)
    x = np.linspace(0., 6., 101)
    np.testing.assert_allclose(build_profile(uphill, x)-build_profile(flat, x), .1*x)
    assert uphill.datum_shift_m == pytest.approx(flat.datum_shift_m + .6)


@pytest.mark.parametrize('knots', [(), ((0., 0.),), ((1., 0.), (2., .1)),
    ((0., 0.), (0., .1)), ((0., 0.), (2., float('nan'))), ((0., True), (2., .1))])
def test_invalid_grade_rejected(knots):
    with pytest.raises(ValueError):
        GradeProfile(knots)


def test_surface_sections_are_half_open_and_nonoverlapping():
    zones = (SurfaceSection(2., 4., 'wet'), SurfaceSection(4., 6., 'loose'))
    mapping = SurfaceMap(get_surface('asphalt'), zones)
    assert [mapping.at(x).name for x in (1., 2., 3.99, 4., 6.)] == [
        'asphalt', 'wet', 'wet', 'loose', 'asphalt']
    assert {s.name for s in mapping.surfaces} == {'asphalt', 'wet', 'loose'}
    with pytest.raises(ValueError):
        SurfaceMap(get_surface('asphalt'), zones + (SurfaceSection(3., 5., 'hardpack'),))
    with pytest.raises(ValueError):
        TrackSpec('bad', 5., surface_sections=zones).validate()


def test_extended_track_roundtrips_toml():
    t = TrackSpec('graded', 10., grade_profile=GradeProfile(((0., 0.), (3., .15), (10., 0.))),
                  surface_sections=(SurfaceSection(5., 7., 'wet'),))
    restored = track_from_dict(tomllib.loads(dump_track(t)))
    assert restored == t
    x = np.linspace(-1., 11., 100)
    np.testing.assert_array_equal(build_profile(t, x), build_profile(restored, x))


@pytest.mark.parametrize('extra', [
    {'grade_profile': {'knots': [[0., 0.], [12., .1]]}},
    {'surface_sections': [{'start_m': 1., 'end_m': 2., 'surface': 'unknown'}]},
    {'grade_profile': {'knots': [[0., 0.], [5., .1]], 'ignored_typo': 1}},
])
def test_bad_extended_track_is_not_silently_accepted(extra):
    with pytest.raises(TrackFileError):
        track_from_dict({'name': 'bad', 'length_m': 10., **extra})


def test_contact_friction_uses_material_and_retains_tire_ceiling():
    mapping = SurfaceMap(get_surface('asphalt'), (SurfaceSection(2., 4., 'wet'),))
    tire = TireParameters(mu=.9)
    assert effective_friction(tire, mapping, 1., 0.) == (.9, 'asphalt')
    assert effective_friction(tire, mapping, 2., 0.) == (.5, 'wet')
    assert effective_friction(tire, mapping, 2., 10.)[0] == pytest.approx(get_surface('wet').mu(10.))
    assert effective_friction(tire, None, 2., 10.) == (.9, 'configured')
    with pytest.raises(ValueError, match='compliant'):
        TireBackendConfig(surface_mode='track')


def test_research_tracks_are_reproducible_and_roundtrip():
    from bike_sim.terrain.research import RESEARCH_SCENARIOS, build_research_track
    for name in RESEARCH_SCENARIOS:
        track = build_research_track(name, seed=13)
        x = np.linspace(0., track.length_m, 2001)
        restored = track_from_dict(tomllib.loads(dump_track(track)))
        np.testing.assert_array_equal(build_profile(track, x), build_profile(restored, x))
    a = build_research_track('rough_uphill', seed=13)
    b = build_research_track('rough_uphill', seed=14)
    assert not np.array_equal(build_profile(a, x), build_profile(b, x))


def test_constant_grade_has_flat_field_runout_not_unbounded_extrapolation():
    from bike_sim.terrain.heightfield import HeightFieldSpec, build_field_data
    track = TrackSpec('constant_25_percent', 12.,
        grade_profile=GradeProfile(((0., .25), (12., .25))))
    np.testing.assert_allclose(build_profile(track, np.array([-2., 0., 2., 12., 120.])),
                               [0., 0., .5, 3., 3.])
    spec = HeightFieldSpec.for_track(track)
    field = build_field_data(track, spec)
    assert np.isfinite(field).all() and field.min() >= 0. and field.max() <= 1.
    assert spec.max_profile_m == pytest.approx(4.)
