"""
Bicycle Geometry Package.
"""

from bike_sim.geometry.specs import BikeSpecs, FrameGeometrySpecs, SuspensionHardwareSpecs
from bike_sim.geometry.hardpoints import (
    compute_frame_angles,
    compute_front_axle,
    compute_ground_z,
    compute_headtube_points,
    compute_rear_axle,
    compute_trail,
    get_fixed_frame_points,
)
from bike_sim.geometry.validation import validate_geometry

__all__ = [
    "BikeSpecs",
    "FrameGeometrySpecs",
    "SuspensionHardwareSpecs",
    "compute_frame_angles",
    "compute_front_axle",
    "compute_ground_z",
    "compute_headtube_points",
    "compute_rear_axle",
    "compute_trail",
    "get_fixed_frame_points",
    "validate_geometry",
]
