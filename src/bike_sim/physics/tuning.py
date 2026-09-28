"""
Suspension Sag Calibration & Damper Tuning Module.

Computes spring stiffness, air pressure (PSI), and damping coefficients
required to achieve target static sag under rider weight.
"""

from math import isfinite
from typing import Any, Dict, Optional
import numpy as np

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.physics.mass import BikeMassSpecs, RiderSpecs, compute_static_system_cg


def reflected_shock_mass(mass_kg: float, leverage_ratio: float) -> float:
    """Reflect wheel-side mass to the shock coordinate by kinetic energy."""
    if (
        not all(isfinite(value) for value in (mass_kg, leverage_ratio))
        or mass_kg <= 0
        or leverage_ratio <= 0
    ):
        raise ValueError("mass and leverage ratio must be finite and positive")
    return mass_kg * leverage_ratio**2


def _compute_fork_tuning(
    specs: BikeSpecs,
    mass_specs: BikeMassSpecs,
    total_mass: float,
    front_weight_fraction: float,
    f_front_vert: float,
    target_front_sag_pct: float,
) -> Dict[str, Any]:
    """Calculates front fork spring stiffness, critical damping, air calibration, and bottom-out metrics."""
    theta = np.radians(specs.head_angle_deg)
    target_fork_travel_m = (target_front_sag_pct / 100.0) * (specs.fork_travel / 1000.0)
    f_fork_axial = f_front_vert * np.sin(theta)
    k_fork = f_fork_axial / target_fork_travel_m

    m_sprung_front = total_mass * front_weight_fraction - (mass_specs.fork_lowers_mass + mass_specs.front_wheel_mass)
    c_crit_front = 2.0 * np.sqrt(k_fork * max(m_sprung_front, 1.0))
    c_fork = 0.50 * c_crit_front

    from bike_sim.physics.air_spring import AirSpringSpecs, ForkAirSpring
    air_specs = AirSpringSpecs(total_travel_mm=specs.fork_travel)
    tokens = specs.fork_air_tokens
    air_spring = ForkAirSpring(specs=air_specs, num_tokens=tokens)
    calibrated_psi = air_spring.calibrate_psi_for_sag(
        target_sag_mm=target_fork_travel_m * 1000.0,
        target_axial_force_n=f_fork_axial,
        num_tokens=tokens,
    )
    air_curve = air_spring.compute_force_curve(num_tokens=tokens)
    air_max_axial_force = air_curve["max_force_n"]
    air_front_vert_max = air_max_axial_force / np.sin(theta)
    air_front_g_bottom = air_front_vert_max / f_front_vert

    f_fork_axial_max = k_fork * (specs.fork_travel / 1000.0)
    f_front_vert_max = f_fork_axial_max / np.sin(theta)
    front_g_bottom = f_front_vert_max / f_front_vert

    return {
        "target_front_sag_mm": float(target_fork_travel_m * 1000.0),
        "fork_stiffness_n_m": float(k_fork),
        "fork_damping_n_s_m": float(c_fork),
        "fork_air_tokens": int(tokens),
        "fork_calibrated_psi": float(calibrated_psi),
        "fork_air_force_full_travel_n": float(air_front_vert_max),
        "fork_air_bottom_out_g": float(air_front_g_bottom),
        "fork_air_progressivity_pct": float(air_curve["progressivity_pct"]),
        "front_spring_force_full_travel_n": float(f_front_vert_max),
        "front_bottom_out_g": float(front_g_bottom),
    }


def _compute_shock_tuning(
    specs: BikeSpecs,
    mass_specs: BikeMassSpecs,
    solver: Any,
    total_mass: float,
    front_weight_fraction: float,
    f_rear_vert: float,
    target_rear_sag_pct: float,
) -> Dict[str, Any]:
    """Calculates rear shock spring stiffness, leverage ratio at sag, damping, and bottom-out metrics."""
    target_rear_wheel_travel_mm = (target_rear_sag_pct / 100.0) * specs.rear_wheel_travel
    st_sag = solver.solve_state_from_wheel_travel(target_rear_wheel_travel_mm)
    shock_stroke_sag_m = st_sag["shock_stroke"] / 1000.0
    lr_at_sag = st_sag["leverage_ratio"]

    f_shock_sag = f_rear_vert * lr_at_sag
    k_shock = f_shock_sag / shock_stroke_sag_m

    m_sprung_rear = total_mass * (1.0 - front_weight_fraction) - (
        mass_specs.chainstay_mass + mass_specs.seatstay_mass + mass_specs.rear_wheel_mass
    )
    m_eff_shock = reflected_shock_mass(m_sprung_rear, lr_at_sag)
    c_crit_shock = 2.0 * np.sqrt(k_shock * max(m_eff_shock, 1.0))
    c_shock = 0.65 * c_crit_shock

    st180 = solver.solve_state_from_wheel_travel(specs.rear_wheel_travel)
    f_shock_max = k_shock * (st180["shock_stroke"] / 1000.0)
    f_rear_wheel_max = f_shock_max / st180["leverage_ratio"]
    rear_g_bottom = f_rear_wheel_max / f_rear_vert

    return {
        "target_rear_sag_mm": float(target_rear_wheel_travel_mm),
        "shock_stroke_at_sag_mm": float(shock_stroke_sag_m * 1000.0),
        "lr_at_sag": float(lr_at_sag),
        "shock_stiffness_n_m": float(k_shock),
        "shock_damping_n_s_m": float(c_shock),
        "rear_spring_force_full_travel_n": float(f_rear_wheel_max),
        "rear_bottom_out_g": float(rear_g_bottom),
    }


def compute_suspension_tuning_for_sag(
    specs: Optional[BikeSpecs] = None,
    mass_specs: Optional[BikeMassSpecs] = None,
    rider_specs: Optional[RiderSpecs] = None,
    rider_mass_kg: Optional[float] = None,
    target_front_sag_pct: float = 30.0,
    target_rear_sag_pct: float = 30.0,
    front_weight_fraction: Optional[float] = None,
) -> Dict[str, Any]:
    """
    Computes exact suspension spring stiffness (k), air pressure (PSI), and damping coefficients (c)
    required to achieve target static sag under rider weight.
    """
    from bike_sim.kinematics.solver import HorstLinkageSolver

    if specs is None:
        specs = BikeSpecs()
    if mass_specs is None:
        mass_specs = BikeMassSpecs()
    if rider_specs is None:
        # The historical default of this calculator is the standing lumped rider, scaled to
        # the requested mass; a seated rider is passed in explicitly as `rider_specs`.
        rider_specs = RiderSpecs(
            variant="lumped",
            mass_kg=rider_mass_kg if rider_mass_kg is not None else 80.0,
        )

    solver = HorstLinkageSolver(specs)
    cg_info = compute_static_system_cg(specs=specs, mass_specs=mass_specs, rider_specs=rider_specs, solver=solver)

    total_mass = cg_info["total_mass_kg"]
    g = 9.81
    total_weight_n = total_mass * g

    if front_weight_fraction is None:
        front_weight_fraction = cg_info["front_load_pct"] / 100.0

    f_front_vert = total_weight_n * front_weight_fraction
    f_rear_vert = total_weight_n * (1.0 - front_weight_fraction)

    fork_tuning = _compute_fork_tuning(specs, mass_specs, total_mass, front_weight_fraction, f_front_vert, target_front_sag_pct)
    shock_tuning = _compute_shock_tuning(specs, mass_specs, solver, total_mass, front_weight_fraction, f_rear_vert, target_rear_sag_pct)

    return {
        "total_system_mass_kg": float(total_mass),
        "front_load_fraction": float(front_weight_fraction),
        **fork_tuning,
        **shock_tuning,
    }
