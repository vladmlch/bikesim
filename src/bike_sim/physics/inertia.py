"""Body-frame inertia tensors for the synthetic physical wheel profile."""

import math
import xml.etree.ElementTree as ET
from typing import NamedTuple

import numpy as np


class WheelComponent(NamedTuple):
    """Annular cylinder centered at an offset from its wheel body's axle."""

    mass_fraction: float
    inner_radius_m: float
    outer_radius_m: float
    width_m: float
    offset_m: tuple[float, float, float]


# Synthetic profile: ring/tire assembly and core (including rotor and, for the
# rear wheel until D1, cassette). Replace these tuples as a whole when measured.
FRONT_WHEEL_PROFILE = (
    WheelComponent(0.75, 0.320, 0.372, 0.060, (0.0, 0.0, 0.0)),
    WheelComponent(0.25, 0.000, 0.045, 0.110, (0.0, 0.0, 0.0)),
)
REAR_WHEEL_PROFILE = (
    WheelComponent(0.75, 0.300, 0.352, 0.064, (0.0, 0.0, 0.0)),
    WheelComponent(0.25, 0.000, 0.045, 0.148, (0.0, 0.0, 0.0)),
)


def ring_inertia(mass_kg: float, inner_m: float, outer_m: float, width_m: float) -> np.ndarray:
    """Inertia of a uniform annular cylinder about its center, with axis Y."""
    if not all(math.isfinite(v) for v in (mass_kg, inner_m, outer_m, width_m)):
        raise ValueError("non-finite inertia input")
    if mass_kg <= 0 or inner_m < 0 or outer_m <= inner_m or width_m <= 0:
        raise ValueError("invalid annular cylinder")
    radial = inner_m**2 + outer_m**2
    transverse = mass_kg * (3 * radial + width_m**2) / 12
    return np.diag([transverse, mass_kg * radial / 2, transverse])


def parallel_axis(mass_kg: float, offset_m: tuple[float, float, float] | np.ndarray) -> np.ndarray:
    """Move an inertia tensor through a displacement in body coordinates."""
    r = np.asarray(offset_m, dtype=float)
    if r.shape != (3,) or not np.isfinite(r).all() or not math.isfinite(mass_kg) or mass_kg < 0:
        raise ValueError("invalid parallel-axis input")
    return mass_kg * (float(r @ r) * np.eye(3) - np.outer(r, r))


def wheel_body_inertia(
    mass_kg: float, components: tuple[WheelComponent, ...]
) -> tuple[np.ndarray, np.ndarray]:
    """Return the combined wheel CoM and tensor in the wheel body's axes."""
    if not math.isfinite(mass_kg) or mass_kg <= 0 or not components:
        raise ValueError("invalid wheel mass or empty profile")
    if not all(math.isfinite(component.mass_fraction) and component.mass_fraction > 0 for component in components):
        raise ValueError("invalid wheel mass fractions")
    if not math.isclose(sum(component.mass_fraction for component in components), 1.0, rel_tol=0, abs_tol=1e-12):
        raise ValueError("wheel mass fractions must sum to one")

    centers = [np.asarray(component.offset_m, dtype=float) for component in components]
    if any(center.shape != (3,) or not np.isfinite(center).all() for center in centers):
        raise ValueError("invalid wheel component offset")
    com = sum((component.mass_fraction * center for component, center in zip(components, centers)), np.zeros(3))
    tensor = np.zeros((3, 3))
    for component, center in zip(components, centers):
        component_mass = mass_kg * component.mass_fraction
        tensor += ring_inertia(
            component_mass, component.inner_radius_m, component.outer_radius_m, component.width_m
        )
        tensor += parallel_axis(component_mass, center - com)
    return com, tensor


def add_body_inertial(
    body: ET.Element, mass_kg: float, com_m: tuple[float, float, float] | np.ndarray,
    tensor: np.ndarray,
) -> None:
    """Write one explicit physical inertial using MuJoCo's full tensor order."""
    com_m = np.asarray(com_m, dtype=float)
    tensor = np.asarray(tensor, dtype=float)
    if not math.isfinite(mass_kg) or mass_kg <= 0 or com_m.shape != (3,) or tensor.shape != (3, 3):
        raise ValueError("invalid body inertial dimensions or mass")
    if not np.isfinite(com_m).all() or not np.isfinite(tensor).all():
        raise ValueError("non-finite body inertial")
    if not np.allclose(tensor, tensor.T, rtol=0, atol=1e-12):
        raise ValueError("inertia tensor must be symmetric")
    eig = np.linalg.eigvalsh(tensor)
    if eig[0] <= 0 or eig[2] > eig[0] + eig[1] + 1e-12:
        raise ValueError("inertia violates positivity or triangle inequality")
    if body.find("inertial") is not None:
        raise ValueError("body already has an explicit inertial")
    values = (tensor[0, 0], tensor[1, 1], tensor[2, 2], tensor[0, 1], tensor[0, 2], tensor[1, 2])
    ET.SubElement(body, "inertial", {
        "mass": format(mass_kg, ".17g"),
        "pos": " ".join(format(float(x), ".17g") for x in com_m),
        "fullinertia": " ".join(format(float(x), ".17g") for x in values),
    })
