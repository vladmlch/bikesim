"""
bike_sim: Modular Bicycle Frame Geometry, 4-Bar Suspension Kinematics, Physics & MuJoCo Simulation Suite.
"""

__version__ = "0.2.0"

from bike_sim.config import BikeConfig
from bike_sim.geometry.hardpoints import (
    compute_frame_angles,
    compute_front_axle,
    compute_headtube_points,
    compute_rear_axle,
    compute_trail,
    get_fixed_frame_points,
)
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.geometry.validation import validate_geometry
from bike_sim.kinematics.curves import compute_instant_centers
from bike_sim.kinematics.solver import HorstLinkageSolver
from bike_sim.mujoco.builder import generate_mujoco_xml
from bike_sim.mujoco.exporter import (
    export_json,
    export_mujoco,
    export_playground_models,
)
from bike_sim.physics.air_spring import AirSpringSpecs, ForkAirSpring
from bike_sim.physics.damper import (
    BikeSuspensionSystem,
    Charger3Damper,
    DamperClickConfig,
    DamperPreset,
    SuperDeluxeDamper,
)
from bike_sim.physics.mass import BikeMassSpecs, RiderSpecs
from bike_sim.sim.playground import SuspensionPlayground, run_interactive_playground
from bike_sim.viz.plots import (
    plot_leverage_ratio,
    plot_linkage_geometry,
    plot_suspension_compressed_comparison,
)
from bike_sim.viz.tables import print_tables

__all__ = [
    "__version__",
    "BikeConfig",
    "BikeSpecs",
    "compute_frame_angles",
    "compute_front_axle",
    "compute_headtube_points",
    "compute_rear_axle",
    "compute_trail",
    "get_fixed_frame_points",
    "validate_geometry",
    "compute_instant_centers",
    "HorstLinkageSolver",
    "generate_mujoco_xml",
    "export_json",
    "export_mujoco",
    "export_playground_models",
    "AirSpringSpecs",
    "ForkAirSpring",
    "BikeSuspensionSystem",
    "Charger3Damper",
    "DamperClickConfig",
    "DamperPreset",
    "SuperDeluxeDamper",
    "BikeMassSpecs",
    "RiderSpecs",
    "SuspensionPlayground",
    "run_interactive_playground",
    "plot_leverage_ratio",
    "plot_linkage_geometry",
    "plot_suspension_compressed_comparison",
    "print_tables",
]
