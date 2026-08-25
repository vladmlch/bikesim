"""
MuJoCo Sensor Suite Configuration.

Configures telemetry sensors: shock stroke, fork travel, joint angles, wheel velocities, IMU, and CoM.
"""

import xml.etree.ElementTree as ET


def build_sensors(root: ET.Element, mode: str = "standard") -> None:
    """
    Appends sensor suite (<sensor>) to the root MJCF element.

    Args:
        root: Root <mujoco> XML element.
        mode: Simulation mode. In "ride" mode, real accelerometers are added at the
            handlebar and saddle sites, reporting the proper acceleration the rider feels.
    """
    sensor = ET.SubElement(root, "sensor")

    # Suspension Travel & Stroke
    ET.SubElement(sensor, "jointpos", {"name": "sensor_shock_stroke", "joint": "shock_stroke"})
    ET.SubElement(sensor, "jointvel", {"name": "sensor_shock_velocity", "joint": "shock_stroke"})
    ET.SubElement(sensor, "jointpos", {"name": "sensor_fork_travel", "joint": "fork_travel"})
    ET.SubElement(sensor, "jointvel", {"name": "sensor_fork_velocity", "joint": "fork_travel"})

    # Steering & Linkage Angles
    ET.SubElement(sensor, "jointpos", {"name": "sensor_steer_angle", "joint": "steer_joint"})
    ET.SubElement(sensor, "jointvel", {"name": "sensor_steer_velocity", "joint": "steer_joint"})
    ET.SubElement(sensor, "jointpos", {"name": "sensor_chainstay_angle", "joint": "main_pivot"})
    ET.SubElement(sensor, "jointpos", {"name": "sensor_horst_angle", "joint": "horst_pivot"})
    ET.SubElement(sensor, "jointpos", {"name": "sensor_rocker_angle", "joint": "rocker_frame_pivot"})
    ET.SubElement(sensor, "jointpos", {"name": "sensor_yoke_angle", "joint": "yoke_pivot"})

    # Wheel Speeds
    ET.SubElement(sensor, "jointvel", {"name": "sensor_front_wheel_speed", "joint": "front_wheel_spin"})
    ET.SubElement(sensor, "jointvel", {"name": "sensor_rear_wheel_speed", "joint": "rear_wheel_spin"})

    # Frame Inertial Measurement Unit (IMU) & Subtree CoM Telemetry
    ET.SubElement(sensor, "accelerometer", {"name": "sensor_frame_accel", "site": "site_BB"})
    ET.SubElement(sensor, "gyro", {"name": "sensor_frame_gyro", "site": "site_BB"})
    ET.SubElement(sensor, "framepos", {"name": "sensor_rear_axle_pos", "objtype": "site", "objname": "site_PRA"})
    ET.SubElement(sensor, "framepos", {"name": "sensor_front_axle_pos", "objtype": "site", "objname": "site_PFA"})
    ET.SubElement(sensor, "subtreecom", {"name": "sensor_subtree_com", "body": "frame"})

    if mode == "ride":
        # MuJoCo accelerometers report proper acceleration (0 in free fall, 9.81 at rest)
        # -- what the rider feels -- rather than finite-differenced site positions.
        ET.SubElement(sensor, "accelerometer", {"name": "sensor_bar_accel", "site": "site_handlebar"})
        ET.SubElement(sensor, "accelerometer", {"name": "sensor_saddle_accel", "site": "site_seatpost_top"})
