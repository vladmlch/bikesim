"""
MuJoCo Ride-Mode Terrain Builder.

Emits the fixed road heightfield asset and its worldbody placement geom for `ride` mode.
No elevation data goes into the XML: the hfield asset is declared with zeroed data and
`model.hfield_data` is filled in from Python later, once per track preset.
"""

import xml.etree.ElementTree as ET

from bike_sim.terrain.heightfield import FIELD, HeightFieldSpec


def build_terrain(
    root: ET.Element,
    worldbody: ET.Element,
    ground_z_m: float,
    spec: HeightFieldSpec = FIELD,
) -> None:
    """
    Appends the ride-mode road hfield asset and its collision/visual geom.

    Args:
        root: Root <mujoco> XML element, used to locate the <asset> section.
        worldbody: <worldbody> element to append the terrain geom to.
        ground_z_m: World height at which the road surface should start, in metres.
        spec: Fixed heightfield geometry shared by every track preset.
    """
    asset = root.find("asset")
    ET.SubElement(
        asset,
        "hfield",
        {
            "name": "road",
            "nrow": str(spec.nrow),
            "ncol": str(spec.ncol),
            "size": spec.size_attr,
        },
    )

    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "terrain",
            "type": "hfield",
            "hfield": "road",
            "pos": f"{spec.geom_x_m():.6f} 0 {spec.geom_z_m(ground_z_m):.6f}",
            "condim": "3",
            "friction": "1.2 0.005 0.0001",
            "solref": "-130000 -800",
            "material": "mat_floor",
        },
    )
