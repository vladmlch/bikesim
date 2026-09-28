"""Synthetic material-law checks for C3; these are not tire calibration."""

from dataclasses import FrozenInstanceError, replace
from math import fsum, inf, nan

import pytest

from bike_sim.physics.tire import TireSpec, normal_contact


@pytest.fixture
def spec():
    return TireSpec(
        radial_k_n_m=130000.0,
        radial_c_ns_m=800.0,
        pressure_pa_gauge=160000.0,
        provenance="synthetic",
        valid_load_range_n=(0.0, 1500.0),
    )


@pytest.mark.parametrize("load", [100.0, 300.0, 500.0, 600.0, 1000.0])
def test_material_stiffness_gives_known_deflection(load):
    delta = load / 130000.0
    force, energy = normal_contact(delta, 0.0, 130000.0, 800.0)
    assert force == pytest.approx(load)
    assert energy == pytest.approx(0.5 * load * delta)


@pytest.mark.parametrize("delta", [-1.0, -0.001, 0.0])
@pytest.mark.parametrize("speed", [-10.0, 0.0, 2.0, 10.0])
def test_no_force_or_energy_outside_contact(delta, speed):
    assert normal_contact(delta, speed, 130000.0, 800.0) == (0.0, 0.0)


@pytest.mark.parametrize("speed,expected", [(0.2, 810.0), (-0.2, 490.0), (-2.0, 0.0)])
def test_damper_and_unilateral_unloading(speed, expected):
    force, energy = normal_contact(0.005, speed, 130000.0, 800.0)
    assert force == pytest.approx(expected)
    # Damping and tensile-force clipping never change stored spring energy.
    assert energy == pytest.approx(0.5 * 130000.0 * 0.005**2)


@pytest.mark.parametrize("delta", [0.0001, 0.004, 0.01])
def test_elastic_energy_is_force_displacement_integral(delta):
    n = 1000
    dx = delta / n
    forces = [normal_contact(i * dx, 0.0, 130000.0, 800.0)[0] for i in range(n + 1)]
    integral = dx * (0.5 * (forces[0] + forces[-1]) + fsum(forces[1:-1]))
    assert normal_contact(delta, 0.0, 130000.0, 800.0)[1] == pytest.approx(integral, rel=1e-12)


@pytest.mark.parametrize("index", range(4))
@pytest.mark.parametrize("value", [nan, inf, -inf])
def test_nonfinite_radial_inputs_are_rejected(index, value):
    args = [0.001, 0.1, 130000.0, 800.0]
    args[index] = value
    with pytest.raises(ValueError):
        normal_contact(*args)


@pytest.mark.parametrize("k,c", [(0.0, 0.0), (-1.0, 800.0), (130000.0, -1.0)])
def test_invalid_material_parameters_rejected_even_airborne(k, c):
    with pytest.raises(ValueError):
        normal_contact(-0.01, 0.0, k, c)


def test_zero_damping_is_valid():
    assert normal_contact(0.001, 100.0, 1000.0, 0.0) == pytest.approx((1.0, 0.0005))


def test_spec_is_the_explicit_material_input(spec):
    assert spec.normal_contact(0.005, 0.1) == normal_contact(0.005, 0.1, 130000.0, 800.0)
    assert spec.provenance == "synthetic"
    with pytest.raises(FrozenInstanceError):
        spec.radial_k_n_m = 1.0


def test_pressure_does_not_invent_a_stiffness_scaling(spec):
    high_pressure = replace(spec, pressure_pa_gauge=2.0 * spec.pressure_pa_gauge)
    assert high_pressure.normal_contact(0.005, 0.1) == spec.normal_contact(0.005, 0.1)


def test_spec_does_not_alias_mutable_load_range(spec):
    bounds = [0.0, 1500.0]
    configured = replace(spec, valid_load_range_n=bounds)
    bounds[1] = 1.0
    assert configured.valid_load_range_n == (0.0, 1500.0)


@pytest.mark.parametrize(
    "changes",
    [
        {"radial_k_n_m": 0.0}, {"radial_k_n_m": -1.0},
        {"radial_c_ns_m": -1.0}, {"pressure_pa_gauge": -1.0},
        {"radial_k_n_m": nan}, {"radial_c_ns_m": inf},
        {"pressure_pa_gauge": nan}, {"provenance": ""},
        {"provenance": "   "}, {"provenance": None},
        {"valid_load_range_n": (-1.0, 1000.0)},
        {"valid_load_range_n": (1000.0, 100.0)},
        {"valid_load_range_n": (100.0, 100.0)},
        {"valid_load_range_n": (0.0, inf)},
        {"valid_load_range_n": (nan, 1000.0)},
        {"valid_load_range_n": (0.0,)},
        {"valid_load_range_n": (0.0, 1.0, 2.0)},
        {"valid_load_range_n": None},
    ],
)
def test_invalid_spec_is_rejected(spec, changes):
    with pytest.raises(ValueError):
        replace(spec, **changes)


@pytest.mark.parametrize("load,expected", [(0.0, True), (1500.0, True), (1500.1, False), (-1.0, False)])
def test_explicit_applicability_range(spec, load, expected):
    assert spec.is_load_in_valid_range(load) is expected


@pytest.mark.parametrize("load", [nan, inf, -inf])
def test_nonfinite_load_query_is_rejected(spec, load):
    with pytest.raises(ValueError):
        spec.is_load_in_valid_range(load)


def test_declared_range_is_not_a_hidden_force_limiter(spec):
    force, _ = spec.normal_contact(0.02, 0.0)
    assert force == pytest.approx(2600.0)
    assert not spec.is_load_in_valid_range(force)


def test_native_solver_parameters_are_not_material_parameters(spec):
    with pytest.raises(TypeError):
        replace(spec, solref=(-130000.0, -800.0))


@pytest.mark.parametrize("args", [(1e200, 0.0, 1e200, 0.0), (0.01, 1e308, 1.0, 1e308)])
def test_radial_overflow_is_not_returned_as_physical_data(args):
    with pytest.raises(ArithmeticError):
        normal_contact(*args)


@pytest.mark.parametrize("value", [True, "0.001", None, [0.001]])
def test_radial_rejects_non_scalar_inputs(value):
    with pytest.raises(ValueError):
        normal_contact(value, 0.1, 130000.0, 800.0)
