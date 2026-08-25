"""
MuJoCo Rider Geometry & Mass Breakdown.

Builds visual and mass capsules for an 80.0 kg rider in neutral attack standing position.
"""

import xml.etree.ElementTree as ET


def build_rider(frame: ET.Element, include_rider: bool = True) -> None:
    """
    Appends rider body capsules (torso, legs, arms) to the frame body.
    """
    torso_sz = "0.13" if include_rider else "0.0001"
    legs_sz = "0.075" if include_rider else "0.0001"
    arms_sz = "0.055" if include_rider else "0.0001"
    torso_m = "55.0" if include_rider else "0.0"
    legs_m = "18.0" if include_rider else "0.0"
    arms_m = "7.0" if include_rider else "0.0"
    rider_rgba = "0.5 0.5 0.5 1.0" if include_rider else "0.5 0.5 0.5 0.0"

    # 1. Torso & Helmet (55.0 kg)
    ET.SubElement(
        frame,
        "geom",
        {
            "name": "geom_rider_torso",
            "type": "capsule",
            "fromto": "0.10 0 0.48 0.22 0 0.82",
            "size": torso_sz,
            "mass": torso_m,
            "rgba": rider_rgba,
            "material": "mat_rider",
            "contype": "0",
            "conaffinity": "0",
        },
    )
    # 2. Rider Legs (18.0 kg, from pedals/BB to hips)
    ET.SubElement(
        frame,
        "geom",
        {
            "name": "geom_rider_legs",
            "type": "capsule",
            "fromto": "0.0 0 0.0 0.10 0 0.48",
            "size": legs_sz,
            "mass": legs_m,
            "rgba": rider_rgba,
            "material": "mat_rider",
            "contype": "0",
            "conaffinity": "0",
        },
    )
    # 3. Rider Arms (7.0 kg, from shoulders to handlebar grips)
    ET.SubElement(
        frame,
        "geom",
        {
            "name": "geom_rider_arms",
            "type": "capsule",
            "fromto": "0.22 0 0.78 0.45 0 0.66",
            "size": arms_sz,
            "mass": arms_m,
            "rgba": rider_rgba,
            "material": "mat_rider",
            "contype": "0",
            "conaffinity": "0",
        },
    )
