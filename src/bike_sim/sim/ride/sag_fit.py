"""Fit physical suspension sag from the ride model's settled wheel travel."""

from dataclasses import replace
from typing import Callable

import numpy as np
from scipy.optimize import least_squares

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.physics.rider import RiderSpecs
from bike_sim.physics.suspension_config import build_suspension_components
from bike_sim.physics.tyre import TyreConfig
from bike_sim.sim.ride_sim import DEFAULT_START_X_M, RideSimulation
from bike_sim.terrain import HeightFieldSpec, TrackSpec


SagEvaluator = Callable[[float, float], tuple[float, float]]


def build_equilibrium_evaluator(
    *,
    specs: BikeSpecs,
    track: TrackSpec,
    rider: RiderSpecs,
    physics_config: SimulationPhysicsConfig,
    field: HeightFieldSpec | None = None,
    tyre: TyreConfig | None = None,
    start_x_m: float = DEFAULT_START_X_M,
    target_speed_kmh: float = 25.0,
) -> SagEvaluator:
    """Bind a physical ride scenario; vary only fork PSI and rear coil rate.

    A fresh physical suspension and ride model are built for every candidate. The
    constructor settles the compiled system with ``solve_static_equilibrium``; the
    measured wheel travels in that result are the fitter's only observations.
    """
    if physics_config.physics_mode != "physical":
        raise ValueError("equilibrium sag evaluator requires physical mode")

    def evaluate(psi: float, rate: float) -> tuple[float, float]:
        if not np.isfinite(psi) or not np.isfinite(rate) or psi <= 0 or rate <= 0:
            raise ValueError("fork pressure and coil rate must be finite and positive")
        candidate = replace(specs, fork_initial_psi=float(psi), shock_stiffness=float(rate))
        controller, coil = build_suspension_components(candidate)
        simulation = RideSimulation(
            specs=candidate,
            track=track,
            rider=rider,
            field=field,
            tyre=tyre,
            start_x_m=start_x_m,
            target_speed_kmh=target_speed_kmh,
            controller=controller,
            coil_shock=coil,
            physics_config=physics_config,
        )
        equilibrium = simulation.equilibrium
        return equilibrium["fork_travel_mm"], equilibrium["rear_travel_mm"]

    return evaluate


def fit_sag(
    evaluate: SagEvaluator,
    target_mm: tuple[float, float],
    initial: tuple[float, float],
    bounds: tuple[tuple[float, float], tuple[float, float]],
) -> np.ndarray:
    """Solve for positive fork PSI and coil N/m with <=0.5 mm sag error."""
    target = np.asarray(target_mm, dtype=float)
    x0 = np.asarray(initial, dtype=float)
    lower, upper = (np.asarray(bound, dtype=float) for bound in bounds)
    if any(array.shape != (2,) or not np.isfinite(array).all()
           for array in (target, x0, lower, upper)):
        raise ValueError("expected finite two-element sag arrays")
    if np.any(target <= 0) or np.any(lower <= 0) or np.any(upper <= lower):
        raise ValueError("invalid sag target or bounds")
    if np.any(x0 < lower) or np.any(x0 > upper):
        raise ValueError("initial sag parameters lie outside bounds")

    def residual(log_values: np.ndarray) -> np.ndarray:
        values = np.exp(log_values)
        measured = np.asarray(evaluate(float(values[0]), float(values[1])), dtype=float)
        if measured.shape != (2,) or not np.isfinite(measured).all():
            raise ValueError("invalid equilibrium output")
        return measured - target

    result = least_squares(
        residual, np.log(x0), bounds=(np.log(lower), np.log(upper)),
    )
    error_mm = float(np.max(np.abs(residual(result.x))))
    if not result.success or error_mm > 0.5:
        raise RuntimeError(
            "requested sag is not achievable within parameter bounds: "
            f"best error {error_mm:.3f} mm"
        )
    return np.exp(result.x)
