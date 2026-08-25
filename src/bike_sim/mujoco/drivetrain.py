"""
MuJoCo Drivetrain, Brakes, and Wheel Assembly Builder.

Provides modular builders for:
1. Bottom bracket shell, motor core, crankset, and pedals
2. Rear wheel, hub, rim, tire, cassette, and brake disc
3. Front and rear disc brake calipers and rotors
"""

import xml.etree.ElementTree as ET
import numpy as np

from bike_sim.physics.mass import BikeMassSpecs
from bike_sim.mujoco._xml_format import _format_vec


def build_bb_and_motor(
    frame: ET.Element,
    mass_specs: BikeMassSpecs,
) -> None:
    """
    Builds the bottom bracket shell, electric motor core, and crankset/pedals on the main frame.
    """
    ET.SubElement(
        frame,
        "geom",
        {
            "name": "geom_bb_shell",
            "type": "cylinder",
            "fromto": "0 -0.0365 0 0 0.0365 0",
            "size": "0.024",
            "mass": "0.30",
            "material": "mat_metal",
        },
    )
    ET.SubElement(
        frame,
        "geom",
        {
            "name": "geom_motor_core",
            "type": "cylinder",
            "fromto": "0.015 -0.055 0.020 0.045 0.055 0.035",
            "size": "0.045",
            "mass": f"{mass_specs.motor_mass:.2f}",
            "material": "mat_motor",
        },
    )
    ET.SubElement(
        frame,
        "geom",
        {
            "name": "geom_crank_arms",
            "type": "cylinder",
            "fromto": "0 -0.090 0 0 0.090 0",
            "size": "0.016",
            "mass": f"{mass_specs.crank_pedals_mass:.2f}",
            "material": "mat_metal",
        },
    )


def build_rear_wheel(
    seatstay: ET.Element,
    mode: str,
    p1_rel_p2: np.ndarray,
    rear_wheel_radius_m: float,
) -> ET.Element:
    """
    Builds the rear wheel body (hub, rim, tire, cassette, brake rotor) mounted on the seatstay at P1 (Rear Axle).
    """
    rear_wheel = ET.SubElement(seatstay, "body", {"name": "rear_wheel", "pos": _format_vec(p1_rel_p2)})
    ET.SubElement(
        rear_wheel,
        "joint",
        {
            "name": "rear_wheel_spin",
            "type": "hinge",
            "pos": "0 0 0",
            "axis": "0 1 0",
            "damping": "0.01",
        },
    )
    ET.SubElement(
        rear_wheel,
        "geom",
        {
            "name": "geom_rear_hub",
            "type": "cylinder",
            "fromto": "0 -0.074 0 0 0.074 0",
            "size": "0.020",
            "mass": "0.30",
            "material": "mat_metal",
        },
    )
    ET.SubElement(
        rear_wheel,
        "geom",
        {
            "name": "geom_rear_rim",
            "type": "cylinder",
            "fromto": "0 -0.015 0 0 0.015 0",
            "size": "0.300",
            "mass": "0.40",
            "material": "mat_rim",
            "contype": "0",
            "conaffinity": "0",
        },
    )
    tire_contype = "0" if mode == "ride" else "1"
    ET.SubElement(
        rear_wheel,
        "geom",
        {
            "name": "geom_rear_tire",
            "type": "cylinder",
            "fromto": "0 -0.032 0 0 0.032 0",
            "size": f"{rear_wheel_radius_m:.6f}",
            "mass": "1.50",
            "material": "mat_tire",
            "contype": tire_contype,
            "conaffinity": tire_contype,
            "friction": "1.2 0.005 0.0001",
        },
    )
    if mode == "ride":
        # Sphere contact patch: see geom_front_contact in steering_fork.py for rationale.
        ET.SubElement(
            rear_wheel,
            "geom",
            {
                "name": "geom_rear_contact",
                "type": "sphere",
                "pos": "0 0 0",
                "size": "0.352",
                "mass": "0",
                "condim": "3",
                "friction": "1.2 0.005 0.0001",
                "solref": "-130000 -800",
                "contype": "1",
                "conaffinity": "1",
                "rgba": "0.08 0.08 0.08 0",
            },
        )
    ET.SubElement(
        rear_wheel,
        "geom",
        {
            "name": "geom_rear_cassette",
            "type": "cylinder",
            "fromto": "0 -0.055 0 0 -0.035 0",
            "size": "0.095",
            "mass": "0.40",
            "material": "mat_metal",
            "contype": "0",
            "conaffinity": "0",
        },
    )
    ET.SubElement(
        rear_wheel,
        "geom",
        {
            "name": "geom_rear_rotor",
            "type": "cylinder",
            "fromto": "0 0.038 0 0 0.040 0",
            "size": "0.1015",
            "mass": "0.20",
            "material": "mat_metal",
            "contype": "0",
            "conaffinity": "0",
        },
    )
    return rear_wheel
