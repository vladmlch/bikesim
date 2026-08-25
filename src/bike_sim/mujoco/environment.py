"""
MuJoCo Simulation Environment.

Builds ground plane, lighting, and workshop test stand fixtures.
"""

from typing import Optional
import xml.etree.ElementTree as ET
import numpy as np


def build_environment(
    worldbody: ET.Element,
    mode: str,
    ground_z_m: float,
    P10: np.ndarray,
) -> None:
    """
    Builds lighting, ground plane, and workshop stand fixtures.
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

