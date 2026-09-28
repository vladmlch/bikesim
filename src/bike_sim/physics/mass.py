"""
Physical Mass Distribution and Center of Gravity (CG) Profile for eMTB.

This module defines the mass specifications and analytical mass distribution
computations for a 29"/27.5" mullet full-power eMTB. It computes component centers
of mass, moments of inertia, overall system CG, and static axle load distributions.
"""

from dataclasses import dataclass, fields
from typing import Any, Dict, Optional, Tuple
import numpy as np

from bike_sim.geometry.cockpit import saddle_geometry, saddle_post_center_of_mass
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.geometry.hardpoints import (
    compute_front_axle,
    compute_rear_axle,
    get_fixed_frame_points,
)
from bike_sim.physics.rider import RiderSpecs


@dataclass
class BikeMassSpecs:
    """
    Mass breakdown for a high-performance full-power eMTB (24.40 kg total; the
    compiled MJCF is 24.35 kg). All masses are in kilograms (kg).
    """

    # Powertrain & Energy
    motor_mass: float = 2.90          # Mid-drive motor casing & internals around BB
    battery_mass: float = 4.30        # 750 Wh integrated battery inside lower downtube

    # Main Frame & Cockpit
    frame_structure_mass: float = 3.20 # Front triangle tubes, headtube, pivot mounts
    saddle_post_mass: float = 0.90     # Dropper post, remote cable, enduro saddle
    crank_pedals_mass: float = 0.85    # Crank arms, spindle, chainring, platform pedals
    steer_assembly_mass: float = 1.10  # Handlebar, stem, steerer tube, crown, headset, controls
    stanchions_mass: float = 0.85      # 38mm upper fork stanchions

    # Front Suspension Unsprung
    fork_lowers_mass: float = 1.50     # Magnesium lowers, arch, axle, 4-piston caliper

    # Rear Suspension Linkage
    chainstay_mass: float = 1.25       # Chainstay arms, main pivot sleeve, Horst pivots
    seatstay_mass: float = 1.05        # Seatstay arms, rear dropouts, rear brake caliper
    rocker_mass: float = 0.40          # Forged aluminum rocker link
    shock_yoke_mass: float = 0.30      # Shock driving yoke
    shock_damper_mass: float = 0.60    # Metric trunnion 205x65mm air damper body & shaft

    # Wheels & Rotational Inertia Breakdown
    front_wheel_mass: float = 2.40     # 29" front wheel, 2.4" enduro tire, sealant, 203mm rotor
    rear_wheel_mass: float = 2.80      # 27.5" rear wheel, 2.5" DH tire, 12-spd cassette, rotor

    # Wheel mass distribution fractions (Rim+Tire outer ring vs Hub+Rotor inner core)
    wheel_rim_tire_fraction: float = 0.75
    wheel_hub_core_fraction: float = 0.25

    @property
    def component_masses(self) -> Dict[str, float]:
        """The fifteen physical component budgets, excluding wheel-shape fractions."""
        return {
            field.name.removesuffix("_mass"): getattr(self, field.name)
            for field in fields(self)
            if field.name.endswith("_mass")
        }

    @property
    def component_provenance(self) -> Dict[str, Dict[str, str]]:
        """Source labels for each physical budget and its geom distribution.

        These authored values and shapes have no measured component dataset yet.
        The stable component IDs let release metadata preserve that distinction.
        """
        return {
            component_id: {"mass": "synthetic", "distribution": "synthetic"}
            for component_id in self.component_masses
        }

    @property
    def total_bike_mass(self) -> float:
        """Computes total mass of the bicycle in kg."""
        return sum(self.component_masses.values())

    def compute_wheel_rotational_inertia(
        self, wheel_mass: float, outer_radius: float, hub_radius: float = 0.045
    ) -> float:
        """
        Computes accurate pitch rotational inertia I_yy (kg*m^2) for a bicycle wheel.
        """
        m_rim = self.wheel_rim_tire_fraction * wheel_mass
        m_hub = self.wheel_hub_core_fraction * wheel_mass
        i_yy = m_rim * (outer_radius ** 2) + 0.5 * m_hub * (hub_radius ** 2)
        return float(i_yy)


# `RiderSpecs` lives in `physics/rider.py` together with the anthropometry and the seated
# pose solver; it is re-exported here because this is where callers historically found it.


def saddle_top_z_for_rider(specs: BikeSpecs, rider_specs: Optional[RiderSpecs]) -> Optional[float]:
    """
    Saddle top height a rider needs, or None for the photograph's default saddle.

    Only the seated rider moves the saddle: its height follows the rider's inseam. The
    lumped rider stands and leaves the saddle where the photograph has it.
    """
    if rider_specs is not None and rider_specs.variant == "seated":
        return rider_specs.seated_pose(specs).saddle.top_z_m
    return None


def compute_component_centers_of_mass(
    specs: Optional[BikeSpecs] = None,
    mass_specs: Optional[BikeMassSpecs] = None,
    solver: Optional[Any] = None,
    rider_specs: Optional[RiderSpecs] = None,
) -> Dict[str, Tuple[np.ndarray, float]]:
    """
    Computes the 3D Center of Mass (CoM) position in meters [X, Y, Z] and mass (kg)
    for each functional component of the bicycle in uncompressed geometry.
    """
    if specs is None:
        specs = BikeSpecs()
    if mass_specs is None:
        mass_specs = BikeMassSpecs()
    if solver is None:
        from bike_sim.kinematics.solver import HorstLinkageSolver
        solver = HorstLinkageSolver(specs)

    st0 = solver.solve_state_from_wheel_travel(0.0)
    fixed = get_fixed_frame_points(specs)

    p_bb = np.array([0.0, 0.0, 0.0])
    p_fa = compute_front_axle(specs) / 1000.0
    p_ra = compute_rear_axle(specs) / 1000.0

    p0 = np.array(fixed["P0"]) / 1000.0
    p2 = np.array(st0["P2"]) / 1000.0
    p3 = np.array(st0["P3"]) / 1000.0
    p4 = np.array(st0["P4"]) / 1000.0
    p5 = np.array(fixed["P5"]) / 1000.0
    p6 = np.array(st0["P6"]) / 1000.0
    p7 = np.array(fixed["P7"]) / 1000.0
    p9 = np.array(fixed["P9"]) / 1000.0
    p10 = np.array(fixed["P10"]) / 1000.0
    p11 = np.array(fixed["P11"]) / 1000.0
    ht_bot = np.array(fixed["P_HT_bot"]) / 1000.0

    pos_motor = p_bb + np.array([0.030, 0.0, 0.0275])
    pos_battery = np.array([0.172834, 0.0, 0.226886])
    pos_frame = np.array([0.180925, 0.0, 0.335899])
    # The post and saddle follow the rider: a seated rider sets the saddle for their inseam,
    # so the component's centre is read from the same cockpit geometry the builder emits.
    saddle = saddle_geometry(p9, saddle_top_z_for_rider(specs, rider_specs))
    pos_saddle = saddle_post_center_of_mass(mass_specs.saddle_post_mass, p10, p9, saddle)
    # Horizontal cranks with a pedal on each end are symmetric about the spindle.
    pos_cranks = p_bb.copy()
    pos_steer = p11 + np.array([-0.015, 0.0, 0.025])
    pos_stanchions = 0.5 * (ht_bot + 0.5 * (ht_bot + p_fa))
    pos_lowers = 0.5 * (0.5 * (ht_bot + p_fa) + p_fa)
    pos_front_wheel = p_fa.copy()
    pos_chainstay = 0.55 * p0 + 0.45 * p2
    pos_seatstay = 0.50 * p_ra + 0.30 * p2 + 0.20 * p3
    pos_rocker = p5 + np.array([-0.008, 0.0, 0.018])
    pos_yoke = 0.5 * (p4 + p6)
    pos_damper = 0.5 * (p7 + p6)
    pos_rear_wheel = p_ra.copy()

    components = {
        "motor": (pos_motor, mass_specs.motor_mass),
        "battery": (pos_battery, mass_specs.battery_mass),
        "frame_structure": (pos_frame, mass_specs.frame_structure_mass),
        "saddle_post": (pos_saddle, mass_specs.saddle_post_mass),
        "crank_pedals": (pos_cranks, mass_specs.crank_pedals_mass),
        "steer_assembly": (pos_steer, mass_specs.steer_assembly_mass),
        "stanchions": (pos_stanchions, mass_specs.stanchions_mass),
        "fork_lowers": (pos_lowers, mass_specs.fork_lowers_mass),
        "front_wheel": (pos_front_wheel, mass_specs.front_wheel_mass),
        "chainstay": (pos_chainstay, mass_specs.chainstay_mass),
        "seatstay": (pos_seatstay, mass_specs.seatstay_mass),
        "rocker": (pos_rocker, mass_specs.rocker_mass),
        "shock_yoke": (pos_yoke, mass_specs.shock_yoke_mass),
        "shock_damper": (pos_damper, mass_specs.shock_damper_mass),
        "rear_wheel": (pos_rear_wheel, mass_specs.rear_wheel_mass),
    }

    if rider_specs is not None:
        components.update(rider_specs.compute_rider_centers_of_mass(specs))

    return components


def compute_static_system_cg(
    specs: Optional[BikeSpecs] = None,
    mass_specs: Optional[BikeMassSpecs] = None,
    rider_specs: Optional[RiderSpecs] = None,
    solver: Optional[Any] = None,
) -> Dict[str, Any]:
    """
    Computes the unloaded analytic CoM and nominal axle load distribution.
    """
    if specs is None:
        specs = BikeSpecs()
    if mass_specs is None:
        mass_specs = BikeMassSpecs()

    components = compute_component_centers_of_mass(
        specs=specs, mass_specs=mass_specs, solver=solver, rider_specs=rider_specs
    )

    total_mass = 0.0
    weighted_pos = np.zeros(3)

    for name, (pos, mass) in components.items():
        total_mass += mass
        weighted_pos += mass * pos

    cg_pos = weighted_pos / total_mass

    p_fa = compute_front_axle(specs) / 1000.0
    p_ra = compute_rear_axle(specs) / 1000.0
    wheelbase = p_fa[0] - p_ra[0]

    g = 9.81
    total_weight_n = total_mass * g
    dist_to_front = p_fa[0] - cg_pos[0]
    dist_to_rear = cg_pos[0] - p_ra[0]

    front_load_n = total_weight_n * (dist_to_rear / wheelbase)
    rear_load_n = total_weight_n * (dist_to_front / wheelbase)

    front_pct = (front_load_n / total_weight_n) * 100.0
    rear_pct = (rear_load_n / total_weight_n) * 100.0

    bike_mass = mass_specs.total_bike_mass
    rider_mass = rider_specs.total_rider_mass if rider_specs is not None else 0.0

    return {
        "total_mass_kg": float(total_mass),
        "bike_mass_kg": float(bike_mass),
        "rider_mass_kg": float(rider_mass),
        "cg_pos_m": cg_pos,
        "wheelbase_m": float(wheelbase),
        "front_axle_x": float(p_fa[0]),
        "rear_axle_x": float(p_ra[0]),
        "front_load_n": float(front_load_n),
        "rear_load_n": float(rear_load_n),
        "front_load_pct": float(front_pct),
        "rear_load_pct": float(rear_pct),
    }


# Explicit name for the unloaded reference calculation. Dynamic displays use the
# compiled CoM from sim.ride.mass_properties instead.
compute_unloaded_analytic_system_cg = compute_static_system_cg


# Re-export suspension sag tuning for backward compatibility
from bike_sim.physics.tuning import compute_suspension_tuning_for_sag
