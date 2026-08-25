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
from bike_sim.physics.damper import (
    BikeSuspensionSystem,
    Charger3Damper,
    DamperClickConfig,
    DamperPreset,
    SuperDeluxeDamper,
    DAMPER_PRESETS,
)

__all__ = [
    "BikeMassSpecs",
    "RiderSpecs",
    "compute_component_centers_of_mass",
    "compute_static_system_cg",
    "compute_suspension_tuning_for_sag",
    "AirSpringSpecs",
    "ForkAirSpring",
    "BikeSuspensionSystem",
    "Charger3Damper",
    "DamperClickConfig",
    "DamperPreset",
    "SuperDeluxeDamper",
    "DAMPER_PRESETS",
]
