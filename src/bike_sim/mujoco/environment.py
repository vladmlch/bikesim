"""
MuJoCo Simulation Environment.

Builds ground plane, lighting, workshop test stand fixtures, and the ride-mode catch plane.
"""

from typing import Optional
import xml.etree.ElementTree as ET
import numpy as np

from bike_sim.terrain.heightfield import FIELD, HeightFieldSpec


def build_environment(
    worldbody: ET.Element,
    mode: str,
    ground_z_m: float,
    P10: np.ndarray,
    field: HeightFieldSpec = FIELD,
) -> None:
    """
    Builds lighting, ground plane, workshop stand fixtures, and (in ride mode) the catch plane.

    Args:
        worldbody: <worldbody> element to append to.
        mode: Simulation mode string.
        ground_z_m: World height of the road surface at the track start, in metres.
        P10: Rear axle hardpoint, used to place the stand fixtures.
        field: Ride-mode heightfield geometry; the catch plane sits below its floor.
    """
    # Lights
    ET.SubElement(
        worldbody,
        "light",
        {
            "directional": "true",
            "diffuse": "0.85 0.85 0.85",
            "specular": "0.25 0.25 0.25",
            "pos": "1.5 -2.5 3.5",
            "dir": "-0.3 0.6 -1.0",
        },
    )
    ET.SubElement(
        worldbody,
        "light",
        {
            "directional": "true",
            "diffuse": "0.45 0.45 0.45",
            "specular": "0.1 0.1 0.1",
            "pos": "-2.5 2.5 3.0",
            "dir": "0.6 -0.6 -1.0",
        },
    )

    if mode == "ride":
        # No floor plane at road level: a MuJoCo plane is an infinite half-space for
        # collision and would bridge every pothole. Only a runaway catch plane, backed
        # off below the heightfield's floor, closes out the world.
        catch_plane_z_m = field.catch_plane_z_m(ground_z_m)
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": "catch_plane",
                "type": "plane",
                "pos": f"0 0 {catch_plane_z_m:.6f}",
                "size": "50 50 0.1",
                "material": "mat_floor",
            },
        )
        return

    def _add_stand_fixtures() -> None:
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": "geom_stand_base",
                "type": "box",
                "pos": f"{P10[0]:.6f} -0.35 {ground_z_m + 0.015:.6f}",
                "size": "0.25 0.20 0.015",
                "material": "mat_stand",
            },
        )
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": "geom_stand_post",
                "type": "cylinder",
                "fromto": f"{P10[0]:.6f} -0.35 {ground_z_m + 0.03:.6f} {P10[0]:.6f} -0.35 {P10[2]:.6f}",
                "size": "0.026",
                "material": "mat_stand",
            },
        )
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": "geom_stand_arm",
                "type": "cylinder",
                "fromto": f"{P10[0]:.6f} -0.35 {P10[2]:.6f} {P10[0]:.6f} -0.05 {P10[2]:.6f}",
                "size": "0.022",
                "material": "mat_stand",
            },
        )
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": "geom_stand_clamp",
                "type": "cylinder",
                "fromto": f"{P10[0]:.6f} -0.055 {P10[2]:.6f} {P10[0]:.6f} 0.055 {P10[2]:.6f}",
                "size": "0.028",
                "material": "mat_metal",
            },
        )

    # Floor plane
    floor_z = ground_z_m
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "floor",
            "type": "plane",
            "pos": f"0 0 {floor_z:.6f}",
            "size": "50 50 0.1",
            "material": "mat_floor",
            "conaffinity": "1",
            "contype": "1",
            "friction": "1.3 0.005 0.0001",
        },
    )

    if mode in ("stand", "playground"):
        _add_stand_fixtures()

