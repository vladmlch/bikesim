"""
MuJoCo Kinematic Linkage, Steering, Fork, and Constraint Builder (Facade).

Re-exports builder functions from specialized modules for full backward compatibility:
- build_steering_and_fork (from bike_sim.mujoco.steering_fork)
- build_rear_linkage (from bike_sim.mujoco.rear_linkage)
- build_equality_constraints (from bike_sim.mujoco.rear_linkage)
- build_contact_exclusions (from bike_sim.mujoco.rear_linkage)
"""

from bike_sim.mujoco.steering_fork import build_steering_and_fork
from bike_sim.mujoco.rear_linkage import (
    build_rear_linkage,
    build_equality_constraints,
    build_contact_exclusions,
)

__all__ = [
    "build_steering_and_fork",
    "build_rear_linkage",
    "build_equality_constraints",
    "build_contact_exclusions",
]
