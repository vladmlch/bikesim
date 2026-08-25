"""
Bicycle Geometry Validation.

Provides sanity checks and structural validation for BikeSpecs.
"""

from bike_sim.geometry.specs import BikeSpecs


def validate_geometry(specs: BikeSpecs) -> bool:
    """
    Validates geometric specifications for sanity and physical viability.

    Args:
        specs: Bicycle specifications.

    Returns:
        True if all checks pass.

    Raises:
        ValueError: If geometry parameters are unphysical.
    """
    if specs.reach <= 0 or specs.stack <= 0:
        raise ValueError("Reach and Stack must be strictly positive.")
    if not (50.0 <= specs.head_angle_deg <= 80.0):
        raise ValueError(f"Unphysical head angle: {specs.head_angle_deg} deg")
    if specs.wheelbase <= specs.reach:
        raise ValueError("Wheelbase must be greater than reach.")
    if specs.front_wheel_radius <= 0 or specs.rear_wheel_radius <= 0:
        raise ValueError("Wheel radii must be positive.")
    return True
