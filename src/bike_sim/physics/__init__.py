"""
Physics & Dynamics Package.
"""

from bike_sim.physics.mass import (
    BikeMassSpecs,
    RiderSpecs,
    compute_component_centers_of_mass,
    compute_static_system_cg,
)
from bike_sim.physics.tuning import compute_suspension_tuning_for_sag
from bike_sim.physics.air_spring import AirSpringSpecs, ForkAirSpring
from bike_sim.physics.coil_shock import CoilShock, CoilShockSpecs
from bike_sim.physics.damper import (
    BikeSuspensionSystem,
    Charger3Damper,
    DamperClickConfig,
    DamperPreset,
    SuperDeluxeDamper,
    DAMPER_PRESETS,
)
from bike_sim.physics.tyre import FRONT_TYRE, REAR_TYRE, TIERS, TierSpec, TyreConfig, TyreSpecs

__all__ = [
    "BikeMassSpecs",
    "RiderSpecs",
    "compute_component_centers_of_mass",
    "compute_static_system_cg",
    "compute_suspension_tuning_for_sag",
    "AirSpringSpecs",
    "ForkAirSpring",
    "CoilShock",
    "CoilShockSpecs",
    "BikeSuspensionSystem",
    "Charger3Damper",
    "DamperClickConfig",
    "DamperPreset",
    "SuperDeluxeDamper",
    "DAMPER_PRESETS",
    "TyreSpecs",
    "TyreConfig",
    "TierSpec",
    "TIERS",
    "FRONT_TYRE",
    "REAR_TYRE",
]
