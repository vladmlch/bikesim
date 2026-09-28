"""Distribute a component's mass over its existing MJCF geometry."""

import math
import xml.etree.ElementTree as ET


def assign_component_mass(geoms: list[ET.Element], total_kg: float) -> None:
    """Use authored geom masses as relative weights for one physical component."""
    if not math.isfinite(total_kg) or total_kg <= 0:
        raise ValueError("component mass must be finite and positive")
    weights = [float(geom.get("mass", "0")) for geom in geoms]
    if not weights or any(not math.isfinite(weight) or weight < 0 for weight in weights):
        raise ValueError("component has no valid mass weights")
    denominator = sum(weights)
    if not math.isfinite(denominator) or denominator <= 0:
        raise ValueError("component has no valid mass weights")
    for geom, weight in zip(geoms, weights):
        geom.set("mass", format(total_kg * weight / denominator, ".17g"))


def register_component(
    registry: dict[str, list[ET.Element]], component_id: str, geoms: list[ET.Element]
) -> None:
    """Register a physical component exactly once, excluding zero-mass visual geoms."""
    if component_id in registry:
        raise ValueError(f"component {component_id!r} was registered twice")
    positive_geoms = [geom for geom in geoms if float(geom.get("mass", "0")) > 0]
    if not positive_geoms:
        raise ValueError(f"component {component_id!r} has no mass-bearing geoms")
    registry[component_id] = positive_geoms
