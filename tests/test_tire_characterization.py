import pytest
from bike_sim.physics.physical_config import TireParameters
from bike_sim.terrain.surface import SurfaceSpec
from bike_sim.validation.tire_characterization import steady_brush_force


def test_steady_small_slip_gain_matches_existing_brush_model():
    tire = TireParameters(tangent_k_n_m=20000., relaxation_length_m=.2, mu=1.1)
    surface = SurfaceSpec('synthetic', .8, .6, 7., 4.5)
    a = steady_brush_force(tire, surface, 1e-4, 600.)
    b = steady_brush_force(tire, surface, -1e-4, 600.)
    slope = (a['force_n'] - b['force_n']) / 2e-4
    assert slope == pytest.approx(4000., rel=.002)
    assert a['force_n'] > 0 > b['force_n']


def test_high_slip_remains_surface_limited():
    tire = TireParameters(mu=1.1)
    surface = SurfaceSpec('synthetic-loose', .55, .45, 7., 4.5)
    r = steady_brush_force(tire, surface, 2., 600.)
    assert abs(r['force_n']) <= r['mu_used'] * 600. + 1e-9
    assert r['mu_used'] <= .55
    assert r['loss_j'] >= 0.


def test_zero_slip_yields_zero_force_and_zero_loss():
    tire = TireParameters()
    surface = SurfaceSpec('synthetic', .8, .6, 7., 4.5)
    r = steady_brush_force(tire, surface, 0., 500.)
    assert r['force_n'] == 0.
    assert r['slip_mps'] == 0.
    assert r['loss_j'] == 0.
    assert r['stored_energy_j'] == 0.


def test_tire_mu_limits_surface_mu_when_tire_mu_is_lower():
    tire = TireParameters(mu=0.4)
    surface = SurfaceSpec('synthetic-high-grip', .9, .7, 10., 4.5)
    r = steady_brush_force(tire, surface, 0.5, 600.)
    assert r['mu_used'] == pytest.approx(0.4)
    assert abs(r['force_n']) <= 0.4 * 600. + 1e-9


def test_metadata_and_configured_stiffness_fields():
    tire = TireParameters(tangent_k_n_m=25000., relaxation_length_m=.15)
    surface = SurfaceSpec('synthetic', .8, .6, 12., 4.5)
    r = steady_brush_force(tire, surface, 0.01, 700.)
    assert r['configured_ckappa_n'] == pytest.approx(25000. * 0.15)
    assert r['surface_ckappa_per_load_metadata'] == pytest.approx(12.)
    assert r['normal_load_n'] == 700.
    assert r['stored_energy_j'] >= 0.


def test_invalid_arguments_raise_value_error():
    tire = TireParameters()
    surface = SurfaceSpec('synthetic', .8, .6, 7., 4.5)
    with pytest.raises(ValueError):
        steady_brush_force(tire, surface, 0.1, -100.)  # non-positive load
    with pytest.raises(ValueError):
        steady_brush_force(tire, surface, 0.1, 600., speed_mps=-1.)  # non-positive speed
    with pytest.raises(ValueError):
        steady_brush_force(tire, surface, 0.1, 600., dt_s=-0.001)  # non-positive dt
    with pytest.raises(ValueError):
        steady_brush_force(tire, surface, 0.1, 600., duration_s=-1.)  # non-positive duration
    with pytest.raises(ValueError, match='integer physical steps'):
        steady_brush_force(tire, surface, 0.1, 600., dt_s=0.003, duration_s=0.01)  # non-integer steps
