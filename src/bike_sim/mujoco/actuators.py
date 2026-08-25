"""
MuJoCo Actuator Configuration.

Configures position servos and motors for test stand and standard simulation modes.
"""

import xml.etree.ElementTree as ET


def build_actuators(root: ET.Element, mode: str) -> None:
    """
    Appends mode-appropriate actuators to the root MJCF element.
    """
    actuator = ET.SubElement(root, "actuator")
    if mode in ("stand", "playground"):
        # Interactive Test Stand Mode: position servos to precisely sweep & hold wheel travel
        ET.SubElement(
            actuator,
            "position",
            {
                "name": "rear_travel_actuator",
                "joint": "main_pivot",
                "kp": "150000",
                "kv": "5000",
                "ctrlrange": "-0.60 0.60",
            },
        )
        ET.SubElement(
            actuator,
            "position",
            {
                "name": "fork_travel_actuator",
                "joint": "fork_travel",
                "kp": "150000",
                "kv": "5000",
                "ctrlrange": "0 0.30",
            },
        )
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": "front_wheel_spin",
                "joint": "front_wheel_spin",
                "gear": "1",
                "ctrlrange": "-50 50",
            },
        )
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": "rear_wheel_spin",
                "joint": "rear_wheel_spin",
                "gear": "1",
                "ctrlrange": "-50 50",
            },
        )
    elif mode == "ride":
        # Planar ride mode: propulsion and braking torque applied directly to the wheel
        # spin joints. Brake ranges are two-sided because the controller computes the
        # sign; the actuator does not. See docs/RIDE.md section 6.
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": "rear_drive",
                "joint": "rear_wheel_spin",
                "gear": "1",
                "ctrlrange": "-150 150",
            },
        )
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": "front_brake",
                "joint": "front_wheel_spin",
                "gear": "1",
                "ctrlrange": "-200 200",
            },
        )
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": "rear_brake",
                "joint": "rear_wheel_spin",
                "gear": "1",
                "ctrlrange": "-200 200",
            },
        )
    else:
        ET.SubElement(
            actuator,
            "position",
            {
                "name": "steer_servo",
                "joint": "steer_joint",
                "kp": "150",
                "kv": "15",
                "ctrlrange": "-45 45",
            },
        )
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": "rear_drive_motor",
                "joint": "rear_wheel_spin",
                "gear": "1",
                "ctrlrange": "-150 150",
            },
        )
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": "front_brake",
                "joint": "front_wheel_spin",
                "gear": "1",
                "ctrlrange": "-200 0",
            },
        )
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": "rear_brake",
                "joint": "rear_wheel_spin",
                "gear": "1",
                "ctrlrange": "-200 0",
            },
        )

