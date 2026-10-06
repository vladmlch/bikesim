"""
Simulation Controllers: Suspension Forces & Cruise Control.

Provides helper controllers for applying non-linear thermodynamic pneumatic fork spring
and hydrodynamic damper forces into MuJoCo's `qfrc_applied`, plus closed-loop cruise control.
"""

from typing import Tuple
import numpy as np

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.physics.air_spring import ForkAirSpring
from bike_sim.physics.damper import BikeSuspensionSystem


class SuspensionController:
    """
    Computes and integrates non-linear suspension forces (thermodynamic pneumatic air spring,
    hydrodynamic Charger 3 fork damping, linear/custom shock spring, and Super Deluxe RC2T rear damping).
    """

    def __init__(
        self,
        specs: BikeSpecs,
        air_spring: ForkAirSpring,
        suspension_system: BikeSuspensionSystem,
    ) -> None:
        self.specs = specs
        self.air_spring = air_spring
        self.suspension_system = suspension_system

    def compute_fork_force(self, travel_mm: float, velocity_mps: float) -> Tuple[float, float, float]:
        """
        Computes net axial fork resistance force (air spring + hydraulic damping).

        Returns:
            Tuple of (f_total, f_air_spring, f_damper).
        """
        f_air = self.air_spring.compute_axial_force(travel_mm)
        f_damp = self.suspension_system.fork_damper.compute_damping_force(velocity_mps, travel_mm)
        f_total = f_air + f_damp
        from bike_sim.physics.checks import derived
        return derived(f_total, 'SuspensionController.fork_total'), f_air, f_damp

    def compute_shock_force(self, stroke_mm: float, velocity_mps: float) -> Tuple[float, float, float]:
        """
        Computes net axial shock resistance force (coil/air spring + hydrodynamic damping with HBO).

        Returns:
            Tuple of (f_total, f_spring, f_damper).
        """
        f_spring = (stroke_mm / 1000.0) * float(self.specs.shock_stiffness)
        f_damp = self.suspension_system.shock_damper.compute_damping_force(velocity_mps, stroke_mm)
        f_total = f_spring + f_damp
        return f_total, f_spring, f_damp
