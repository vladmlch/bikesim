"""Physical sag tuning uses kinetic energy and measured ride equilibrium."""

import dataclasses

import numpy as np
import pytest

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.rider import RiderSpecs
from bike_sim.physics.tuning import reflected_shock_mass
from bike_sim.sim.ride.sag_fit import build_equilibrium_evaluator, fit_sag
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset


def test_reflected_mass_follows_kinetic_energy():
    # 1/2 m v_wheel^2 = 1/2 m_eff v_shock^2, v_wheel = LR * v_shock.
    assert reflected_shock_mass(50.0, 3.0) == 450.0


@pytest.mark.parametrize("mass, ratio", [
    (float("nan"), 3.0), (float("inf"), 3.0), (-1.0, 3.0), (0.0, 3.0),
    (50.0, float("nan")), (50.0, float("inf")), (50.0, -3.0), (50.0, 0.0),
])
def test_reflected_mass_rejects_invalid_inputs(mass, ratio):
    with pytest.raises(ValueError):
        reflected_shock_mass(mass, ratio)


def test_sag_fit_solves_two_positive_parameters():
    def evaluate(psi, rate):
        return 5000.0 / psi, 5e6 / rate

    result = fit_sag(
        evaluate, (50.0, 50.0), (80.0, 90000.0),
        ((20.0, 20000.0), (200.0, 300000.0)),
    )
    np.testing.assert_allclose(result, (100.0, 100000.0), rtol=1e-4)


@pytest.mark.parametrize("target, initial, bounds", [
    ((float("nan"), 50.0), (80.0, 90000.0), ((20.0, 20000.0), (200.0, 300000.0))),
    ((float("inf"), 50.0), (80.0, 90000.0), ((20.0, 20000.0), (200.0, 300000.0))),
    ((-1.0, 50.0), (80.0, 90000.0), ((20.0, 20000.0), (200.0, 300000.0))),
    ((50.0, 50.0), (float("nan"), 90000.0), ((20.0, 20000.0), (200.0, 300000.0))),
    ((50.0, 50.0), (float("inf"), 90000.0), ((20.0, 20000.0), (200.0, 300000.0))),
    ((50.0, 50.0), (-80.0, 90000.0), ((20.0, 20000.0), (200.0, 300000.0))),
    ((50.0, 50.0), (80.0, 90000.0), ((float("nan"), 20000.0), (200.0, 300000.0))),
    ((50.0, 50.0), (80.0, 90000.0), ((20.0, float("inf")), (200.0, 300000.0))),
    ((50.0, 50.0), (80.0, 90000.0), ((-20.0, 20000.0), (200.0, 300000.0))),
    ((50.0, 50.0), (80.0, 90000.0), ((20.0, 20000.0), (0.0, 300000.0))),
    ((50.0, 50.0), (80.0, 90000.0), ((20.0, 20000.0), (20.0, 300000.0))),
])
def test_sag_fit_rejects_invalid_inputs(target, initial, bounds):
    with pytest.raises(ValueError):
        fit_sag(lambda *_: (50.0, 50.0), target, initial, bounds)


def test_sag_fit_rejects_invalid_equilibrium_output():
    with pytest.raises(ValueError, match="equilibrium"):
        fit_sag(lambda *_: (float("nan"), 50.0), (50.0, 50.0),
                (80.0, 90000.0), ((20.0, 20000.0), (200.0, 300000.0)))


def test_sag_fit_reports_unattainable_target():
    with pytest.raises(RuntimeError, match="not achievable"):
        fit_sag(lambda *_: (10.0, 10.0), (50.0, 50.0),
                (80.0, 90000.0), ((20.0, 20000.0), (200.0, 300000.0)))


def test_equilibrium_evaluator_matches_flat_track_ride_simulation():
    specs = dataclasses.replace(BikeSpecs(), fork_initial_psi=90.0, shock_stiffness=120000.0)
    track = get_preset("flat")
    rider = RiderSpecs(variant="lumped", mass_kg=76.0)
    config = SimulationPhysicsConfig(physics_mode="physical")
    evaluate = build_equilibrium_evaluator(
        specs=specs, track=track, rider=rider, physics_config=config,
    )

    measured = evaluate(105.0, 130000.0)
    reference = RideSimulation(
        track=track,
        specs=dataclasses.replace(specs, fork_initial_psi=105.0, shock_stiffness=130000.0),
        rider=rider,
        physics_config=config,
    ).equilibrium
    assert measured == pytest.approx(
        (reference["fork_travel_mm"], reference["rear_travel_mm"]), abs=1e-9
    )
    assert specs.fork_initial_psi == 90.0
    assert specs.shock_stiffness == 120000.0
