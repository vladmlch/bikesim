"""
MuJoCo Visual & Asset Configuration.

Constructs <visual> global quality settings and <asset> definitions (textures, materials).
"""

import xml.etree.ElementTree as ET
from typing import Dict, List

TEXTURE_DEFINITIONS: List[Dict[str, str]] = [
    {
        "name": "grid_tex",
        "type": "2d",
        "builtin": "checker",
        "width": "512",
        "height": "512",
        "rgb1": "0.85 0.85 0.88",
        "rgb2": "0.75 0.75 0.78",
    },
    {
        "name": "skybox",
        "type": "skybox",
        "builtin": "gradient",
        "rgb1": "0.4 0.6 0.8",
        "rgb2": "0.1 0.15 0.25",
        "width": "512",
        "height": "512",
    },
]

MATERIAL_DEFINITIONS: List[Dict[str, str]] = [
    {"name": "mat_floor", "texture": "grid_tex", "texrepeat": "25 25", "reflectance": "0.08"},
    {"name": "mat_frame", "rgba": "0.14 0.18 0.22 1.0", "specular": "0.4", "shininess": "0.5"},
    {"name": "mat_linkage", "rgba": "0.88 0.22 0.16 1.0", "specular": "0.7", "shininess": "0.8"},
    {"name": "mat_yoke", "rgba": "0.55 0.22 0.75 1.0", "specular": "0.6", "shininess": "0.7"},
    {"name": "mat_kashima", "rgba": "0.82 0.60 0.28 1.0", "specular": "0.95", "shininess": "0.95"},
    {"name": "mat_damper_body", "rgba": "0.12 0.12 0.12 1.0", "specular": "0.8", "shininess": "0.85"},
    {"name": "mat_fork_lower", "rgba": "0.16 0.16 0.18 1.0", "specular": "0.3", "shininess": "0.4"},
    {"name": "mat_tire", "rgba": "0.08 0.08 0.08 1.0", "specular": "0.05", "shininess": "0.1"},
    {"name": "mat_rim", "rgba": "0.22 0.24 0.26 1.0", "specular": "0.5", "shininess": "0.6"},
    {"name": "mat_metal", "rgba": "0.75 0.78 0.82 1.0", "specular": "0.85", "shininess": "0.9"},
    {"name": "mat_saddle", "rgba": "0.06 0.06 0.06 1.0", "specular": "0.1", "shininess": "0.2"},
    {"name": "mat_handlebar", "rgba": "0.12 0.12 0.14 1.0", "specular": "0.4", "shininess": "0.5"},
    {"name": "mat_pivot_yellow", "rgba": "0.98 0.85 0.12 1.0", "specular": "0.9", "shininess": "0.9"},
    {"name": "mat_stand", "rgba": "0.22 0.25 0.30 1.0", "specular": "0.5", "shininess": "0.6"},
    {"name": "mat_obstacle", "rgba": "0.85 0.50 0.15 1.0", "specular": "0.4", "shininess": "0.4"},
    {"name": "mat_pit", "rgba": "0.22 0.35 0.55 1.0", "specular": "0.4", "shininess": "0.5"},
    {"name": "mat_motor", "rgba": "0.10 0.11 0.13 1.0", "specular": "0.6", "shininess": "0.7"},
    {"name": "mat_battery", "rgba": "0.08 0.08 0.10 1.0", "specular": "0.5", "shininess": "0.6"},
    {"name": "mat_cg_marker", "rgba": "0.95 0.20 0.85 0.9", "specular": "0.9", "shininess": "0.9"},
    {"name": "mat_rider", "rgba": "0.18 0.38 0.68 0.95", "specular": "0.4", "shininess": "0.5"},
]


def build_visual_and_assets(root: ET.Element) -> None:
    """
    Appends <visual> and <asset> elements to the MJCF root.

    Args:
        root: Root <mujoco> XML element.
    """
    visual = ET.SubElement(root, "visual")
    ET.SubElement(
        visual,
        "global",
        {"elevation": "-20", "azimuth": "140", "offwidth": "1920", "offheight": "1080"},
    )
    ET.SubElement(visual, "quality", {"shadowsize": "4096"})

    asset = ET.SubElement(root, "asset")
    for tex in TEXTURE_DEFINITIONS:
        ET.SubElement(asset, "texture", tex)
    for mat in MATERIAL_DEFINITIONS:
        ET.SubElement(asset, "material", mat)
