"""
Cockpit Geometry: Saddle, Seatpost, Handlebar Grip and Pedals.

The three points a rider touches the bike at -- saddle top, handlebar grip, pedal spindles
-- and the seatpost that carries the saddle, expressed in the BB-origin frame in metres.

This module exists so that the MJCF builder (`mujoco/frame.py`, `mujoco/steering_fork.py`,
`mujoco/drivetrain.py`), the analytic mass table (`physics/mass.py`) and the seated-rider
pose solver (`physics/rider.py`) read one set of formulas. Before it, the seatpost
arithmetic lived in the frame builder and its result was restated as a constant in the mass
table; a saddle that moves with the rider's inseam would have pulled the two apart.

**Provenance.** The seatpost axis, the photo saddle height and the saddle box dimensions are
authored styling measured off the reference photograph (README, "Measured vs. Authored").
Nothing here is a published number.
"""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from bike_sim.geometry.hardpoints import get_fixed_frame_points
from bike_sim.geometry.specs import BikeSpecs

# Seatpost direction, from the photograph's post pixels (-83, 430) -> (-146, 624) mm.
_SEATPOST_AXIS_RAW = np.array([-146.0 - -83.0, 0.0, 624.0 - 430.0])
SEATPOST_AXIS = _SEATPOST_AXIS_RAW / float(np.linalg.norm(_SEATPOST_AXIS_RAW))

# Saddle top surface height above the BB in the reference photograph. The default saddle
# position of every mode that does not derive the saddle from a rider.
PHOTO_SADDLE_TOP_Z_M = 0.690

SADDLE_HALF_THICKNESS_M = 0.015
SADDLE_SETBACK_FROM_POST_TOP_M = 0.024
SADDLE_HALF_LENGTH_M = 0.13
SADDLE_HALF_WIDTH_M = 0.065

# Dropper post: 0.35 kg of the 0.90 kg `saddle_post_mass`; the saddle is the remainder.
SEATPOST_MASS_KG = 0.35
SADDLE_MIN_MASS_KG = 0.20

# Exposed post length the frame accepts, collar to saddle rail. Below the minimum the saddle
# sits on the collar; above the maximum a 200 mm dropper has run out of stanchion.
MIN_EXPOSED_SEATPOST_M = 0.050
MAX_EXPOSED_SEATPOST_M = 0.250

# Stem: the handlebar clamp sits 40 mm forward of and 20 mm above the head tube top
# (`mujoco/steering_fork.py`). The rider's hands close on the bar at its centre plane.
STEM_TOP_OFFSET_M = np.array([0.04, 0.0, 0.02])

# Cranks: horizontal, 3 and 9 o'clock. Right arm forward by convention; the plane is
# sagittal so which side leads is a drawing choice, not a dynamic one.
CRANK_ARM_LATERAL_OFFSET_M = 0.075
PEDAL_LATERAL_OFFSET_M = 0.115
PEDAL_HALF_SIZE_M = (0.050, 0.040, 0.008)


@dataclass(frozen=True)
class SaddleGeometry:
    """
    The seatpost and saddle placement for one saddle height.

    Attributes:
        seatpost_top: Top of the exposed post, on the seatpost axis (m, BB frame).
        saddle_pos: Centre of the saddle box (m, BB frame).
        top_center: Centre of the saddle's top surface (m, BB frame).
        top_z_m: Height of the top surface above the BB.
        height_m: Saddle height in the cycling sense: BB centre to the top-surface centre.
        exposed_post_m: Length of post between the collar and the post top.
    """

    seatpost_top: np.ndarray
    saddle_pos: np.ndarray
    top_center: np.ndarray
    top_z_m: float
    height_m: float
    exposed_post_m: float


def seatpost_top_for_saddle_z(p9_m: np.ndarray, saddle_top_z_m: float) -> np.ndarray:
    """
    Point on the seatpost axis whose height puts the saddle's top surface at ``saddle_top_z_m``.

    Reproduces the frame builder's construction exactly: the saddle underside is two half
    thicknesses below the top surface, and the post top sits on the axis at that height.
    """
    saddle_underside_z = saddle_top_z_m - 2.0 * SADDLE_HALF_THICKNESS_M
    return p9_m + SEATPOST_AXIS * ((saddle_underside_z - p9_m[2]) / SEATPOST_AXIS[2])


def saddle_geometry(p9_m: np.ndarray, saddle_top_z_m: Optional[float] = None) -> SaddleGeometry:
    """
    Places the seatpost and saddle for a saddle top height.

    Args:
        p9_m: Seatpost clamp (collar) hardpoint P9, in metres, BB frame.
        saddle_top_z_m: Height of the saddle's top surface above the BB. Defaults to the
            photograph's 0.690 m.

    Returns:
        The saddle placement.
    """
    top_z = PHOTO_SADDLE_TOP_Z_M if saddle_top_z_m is None else float(saddle_top_z_m)
    seatpost_top = seatpost_top_for_saddle_z(p9_m, top_z)
    saddle_pos = np.array(
        [
            seatpost_top[0] - SADDLE_SETBACK_FROM_POST_TOP_M,
            0.0,
            seatpost_top[2] + SADDLE_HALF_THICKNESS_M,
        ]
    )
    top_center = saddle_pos + np.array([0.0, 0.0, SADDLE_HALF_THICKNESS_M])
    return SaddleGeometry(
        seatpost_top=seatpost_top,
        saddle_pos=saddle_pos,
        top_center=top_center,
        top_z_m=top_z,
        height_m=float(np.linalg.norm(top_center)),
        exposed_post_m=float(np.linalg.norm(seatpost_top - p9_m)),
    )


def saddle_top_z_for_height(p9_m: np.ndarray, saddle_height_m: float) -> float:
    """
    Saddle top height that gives a required saddle height (BB centre to top-surface centre).

    The top-surface centre moves along a line as the post extends, so the height is a
    quadratic in the top z; the root above the BB is the one wanted.

    Args:
        p9_m: Seatpost clamp hardpoint P9, in metres.
        saddle_height_m: Required BB-to-saddle-top distance, in metres.

    Returns:
        The saddle top surface height above the BB, in metres.

    Raises:
        ValueError: If no saddle position on the post axis reaches that height.
    """
    s = SEATPOST_AXIS[0] / SEATPOST_AXIS[2]
    x0 = p9_m[0] - SADDLE_SETBACK_FROM_POST_TOP_M - s * (2.0 * SADDLE_HALF_THICKNESS_M + p9_m[2])
    a = 1.0 + s * s
    b = 2.0 * x0 * s
    c = x0 * x0 - saddle_height_m * saddle_height_m
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        raise ValueError(f"no saddle position on the seatpost axis is {saddle_height_m:.3f} m from the BB")
    return float((-b + np.sqrt(disc)) / (2.0 * a))


def saddle_post_masses(saddle_post_mass_kg: float, p10_m: np.ndarray, p9_m: np.ndarray,
                       geometry: SaddleGeometry) -> Tuple[float, float, float]:
    """
    Splits the saddle-and-post budget the way the frame builder does.

    Args:
        saddle_post_mass_kg: `BikeMassSpecs.saddle_post_mass`.
        p10_m: Seat tube junction P10 (post bottom), in metres.
        p9_m: Seatpost collar P9, in metres.
        geometry: Saddle placement.

    Returns:
        Tuple of (lower post mass, upper post mass, saddle mass), in kg.
    """
    lower_len = float(np.linalg.norm(p9_m - p10_m))
    upper_len = geometry.exposed_post_m
    lower = round(SEATPOST_MASS_KG * lower_len / (lower_len + upper_len), 6)
    upper = SEATPOST_MASS_KG - lower
    saddle = max(SADDLE_MIN_MASS_KG, saddle_post_mass_kg - SEATPOST_MASS_KG)
    return lower, upper, saddle


def saddle_post_center_of_mass(saddle_post_mass_kg: float, p10_m: np.ndarray, p9_m: np.ndarray,
                               geometry: SaddleGeometry) -> np.ndarray:
    """Mass-weighted centre of the two post cylinders and the saddle box, in metres."""
    lower, upper, saddle = saddle_post_masses(saddle_post_mass_kg, p10_m, p9_m, geometry)
    centre = (
        lower * 0.5 * (p10_m + p9_m)
        + upper * 0.5 * (p9_m + geometry.seatpost_top)
        + saddle * geometry.saddle_pos
    )
    return centre / (lower + upper + saddle)


def handlebar_grip_point(specs: BikeSpecs) -> np.ndarray:
    """
    Centre of the handlebar, where the planar rider's hands close, in metres, BB frame.

    The bar is a straight cylinder through the stem top; its centre plane is the sagittal
    plane, so the grip point is the stem top itself.
    """
    fixed = get_fixed_frame_points(specs)
    p11_m = np.array(fixed["P11"], dtype=float) / 1000.0
    return p11_m + STEM_TOP_OFFSET_M


def pedal_points(crank_length_m: float) -> Tuple[np.ndarray, np.ndarray]:
    """
    Pedal spindle centres for horizontal cranks, in metres, BB frame.

    Returns:
        Tuple of (front pedal, rear pedal): +x and -x by one crank length, on the BB axis.
    """
    front = np.array([crank_length_m, 0.0, 0.0])
    rear = np.array([-crank_length_m, 0.0, 0.0])
    return front, rear


__all__ = [
    "SEATPOST_AXIS",
    "PHOTO_SADDLE_TOP_Z_M",
    "SADDLE_HALF_THICKNESS_M",
    "SADDLE_SETBACK_FROM_POST_TOP_M",
    "SADDLE_HALF_LENGTH_M",
    "SADDLE_HALF_WIDTH_M",
    "SEATPOST_MASS_KG",
    "SADDLE_MIN_MASS_KG",
    "MIN_EXPOSED_SEATPOST_M",
    "MAX_EXPOSED_SEATPOST_M",
    "STEM_TOP_OFFSET_M",
    "CRANK_ARM_LATERAL_OFFSET_M",
    "PEDAL_LATERAL_OFFSET_M",
    "PEDAL_HALF_SIZE_M",
    "SaddleGeometry",
    "seatpost_top_for_saddle_z",
    "saddle_geometry",
    "saddle_top_z_for_height",
    "saddle_post_masses",
    "saddle_post_center_of_mass",
    "handlebar_grip_point",
    "pedal_points",
]
