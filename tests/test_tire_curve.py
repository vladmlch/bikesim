from dataclasses import asdict
import numpy as np
import pytest
from bike_sim.physics.tire import TireSpec
from bike_sim.physics.tire_curve import TabulatedTireSpec, MaterialRangeError
from bike_sim.physics.physical_config import TireParameters
from bike_sim.physics.resolution import resolve_physics_config


def curve(**overrides):
    values = dict(deflection_m=(0., .01, .03), force_n=(0., 1000., 5000.),
                  radial_c_ns_m=100., pressure_pa_gauge=200000.,
                  provenance='synthetic test', valid_load_range_n=(0., 5000.))
    return TabulatedTireSpec(**(values | overrides))


@pytest.mark.parametrize('x', [.001, .005, .0099, .01, .011, .02, .029])
def test_force_is_derivative_of_exact_elastic_energy(x):
    c = curve()
    f, e = c.elastic_response(x)
    eps = 1e-8
    numerical = (c.elastic_response(x+eps)[1]-c.elastic_response(x-eps)[1])/(2*eps)
    assert numerical == pytest.approx(f, rel=1e-6)
    assert e >= 0.
    assert c.elastic_response(.01) == pytest.approx((1000., 5.))
    assert c.elastic_response(.03) == pytest.approx((5000., 65.))


@pytest.mark.parametrize('rate', [-100., -3., -.1, 0., 1., 100.])
def test_unilateral_damping_is_passive(rate):
    c = curve()
    f, e = c.normal_contact(.02, rate)
    spring, stored = c.elastic_response(.02)
    assert f >= 0. and e == stored
    assert (f-spring)*rate >= -1e-12
    assert c.normal_contact(-.001, rate) == (0., 0.)


def test_linear_and_tabulated_laws_agree_for_linear_data():
    a = TireSpec(100000., 100., 200000., 'synthetic', (0., 3000.))
    b = curve(force_n=(0., 1000., 3000.), valid_load_range_n=(0., 3000.))
    for x in np.linspace(0., .03, 15):
        assert a.elastic_response(x) == pytest.approx(b.elastic_response(x))
        assert a.normal_contact(x, -.1) == pytest.approx(b.normal_contact(x, -.1))


def test_no_extrapolation_or_false_calibration_and_immutable_inputs():
    xs, fs = [0., .01, .03], [0., 1000., 5000.]
    c = curve(deflection_m=xs, force_n=fs)
    xs[1] = 123.; fs[1] = -123.
    assert c.deflection_m == (0., .01, .03)
    with pytest.raises(MaterialRangeError, match='deflection'):
        c.elastic_response(.03001)
    with pytest.raises(MaterialRangeError):
        c.normal_contact(.04, 0.)
    assert not c.is_load_in_valid_range(6000.)
    assert not c.is_load_in_valid_range(-1.)


@pytest.mark.parametrize('change', [
    {'deflection_m': (0., .01)}, {'deflection_m': (0., .01, .01)},
    {'deflection_m': (.001, .01, .03)}, {'force_n': (1., 1000., 5000.)},
    {'force_n': (0., 2000., 1000.)}, {'force_n': (0., float('inf'), 5000.)},
    {'radial_c_ns_m': -1.}, {'pressure_pa_gauge': True}, {'provenance': ''},
    {'valid_load_range_n': (0., 6000.)}, {'force_n': (0., 0., 0.)},
])
def test_invalid_table_rejected(change):
    with pytest.raises(ValueError):
        curve(**change)


def test_material_config_roundtrip_and_unknown_fields():
    c = curve()
    assert TireParameters(material=c).material == c
    data = {'physics_mode': 'physical', 'tires': {'backend': 'compliant_2d',
            'front': {'material': asdict(c)}}}
    resolved = resolve_physics_config(data)
    assert resolved.tires.front.material == c
    assert resolve_physics_config(asdict(resolved)) == resolved
    data['tires']['front']['material']['radial_k_n_m'] = 10000.
    with pytest.raises(ValueError, match='unknown'):
        resolve_physics_config(data)
