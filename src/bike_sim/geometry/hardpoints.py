"""
Bicycle Hardpoints & Analytical Geometry Calculations.

Calculates key frame, steering, wheel, and hardpoint coordinates
in the MuJoCo 3D coordinate system (+X: Forward, +Y: Lateral/Left, +Z: Up, Origin at BB (0,0,0)).
"""

from typing import Dict, Optional, Tuple
import numpy as np

from bike_sim.geometry.specs import BikeSpecs


def compute_headtube_points(specs: BikeSpecs) -> Tuple[np.ndarray, np.ndarray]:
    """
    Computes top and bottom headtube 3D coordinates.

    Top headtube (P11) corresponds to Reach (X) and Stack (Z) relative to BB.
    Bottom headtube (P_HT_bot) extends along the head tube angle downwards.

    Args:
        specs: Bicycle specifications.

    Returns:
        Tuple of (P11_top, P_HT_bot) as numpy arrays in mm.
    """
    theta = np.radians(specs.head_angle_deg)
    u_steer = np.array([np.cos(theta), 0.0, -np.sin(theta)], dtype=float)

    p11_top = np.array([specs.reach, 0.0, specs.stack], dtype=float)
    p_ht_bot = p11_top + specs.headtube_length * u_steer

    return p11_top, p_ht_bot


def compute_front_axle(specs: BikeSpecs) -> np.ndarray:
    """
    Computes the front axle 3D coordinate (P_FA) in mm.

    Steering axis equation in XZ plane: (z - stack) = -tan(theta) * (x - reach).
    At axle height Z = bb_drop:
        x_axis = reach + (stack - bb_drop) / tan(theta)
    With fork offset along normal n = (sin(theta), 0, cos(theta)):
        x_fa = x_axis + fork_offset / sin(theta)

    Args:
        specs: Bicycle specifications.

    Returns:
        Front axle coordinate np.ndarray [X, Y, Z] in mm.
    """
    theta = np.radians(specs.head_angle_deg)
    tan_theta = np.tan(theta)
    sin_theta = np.sin(theta)

    x_axis = specs.reach + (specs.stack - specs.bb_drop) / tan_theta
    x_fa = x_axis + specs.fork_offset / sin_theta

    return np.array([x_fa, 0.0, specs.bb_drop], dtype=float)


def compute_ground_z(specs: BikeSpecs) -> float:
    """
    Computes the ground plane Z coordinate in the BB origin frame.

    Ground contact Z is bb_drop - front_wheel_radius (e.g. 22.5 - 372.0 = -349.5 mm).

    Args:
        specs: Bicycle specifications.

    Returns:
        Z coordinate of flat ground plane in mm.
    """
    return specs.bb_drop - specs.front_wheel_radius


def compute_rear_axle(specs: BikeSpecs, front_axle: Optional[np.ndarray] = None) -> np.ndarray:
    """
    Computes the rear axle 3D coordinate (P_RA / P1) in mm for Mullet (29" Front / 27.5" Rear).

    For Mullet configuration on a level ground plane:
        Z_ground = bb_drop - front_wheel_radius = 22.5 - 372.0 = -349.5 mm
        Z_RA = Z_ground + rear_wheel_radius = -349.5 + 352.0 = +2.5 mm
        X_RA = X_FA - wheelbase

    Args:
        specs: Bicycle specifications.
        front_axle: Optional precomputed front axle coordinate.

    Returns:
        Rear axle coordinate np.ndarray [X, Y, Z] in mm.
    """
    if front_axle is None:
        front_axle = compute_front_axle(specs)

    x_ra = front_axle[0] - specs.wheelbase
    z_ground = compute_ground_z(specs)
    z_ra = z_ground + specs.rear_wheel_radius
    return np.array([x_ra, 0.0, z_ra], dtype=float)


def compute_trail(specs: BikeSpecs) -> Dict[str, float]:
    """
    Computes ground trail, mechanical trail, and axle-to-crown distance in mm.

    - Ground contact Z is compute_ground_z(specs) = -349.5 mm.
    - Ground Trail (Tg) is the horizontal distance between steer axis ground intersection and front axle.
    - Mechanical Trail (Tm) is the perpendicular distance: Tg * sin(head_angle).
    - Axle-to-Crown (A2C) is distance from bottom headtube to axle projected onto steer axis.

    Args:
        specs: Bicycle specifications.

    Returns:
        Dictionary with 'ground_trail', 'mechanical_trail', 'axle_to_crown',
        'steer_ground_intersection_x', and 'ground_z'.
    """
    theta = np.radians(specs.head_angle_deg)
    tan_theta = np.tan(theta)
    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)

    ground_z = compute_ground_z(specs)
    x_g = specs.reach + (specs.stack - ground_z) / tan_theta

    fa = compute_front_axle(specs)
    ground_trail = x_g - fa[0]
    mechanical_trail = ground_trail * sin_theta

    _, p_ht_bot = compute_headtube_points(specs)
    u_steer = np.array([cos_theta, 0.0, -sin_theta], dtype=float)
    v_fork = fa - p_ht_bot
    axle_to_crown = float(np.dot(v_fork, u_steer))

    return {
        "ground_trail": float(ground_trail),
        "mechanical_trail": float(mechanical_trail),
        "axle_to_crown": axle_to_crown,
        "steer_ground_intersection_x": float(x_g),
        "ground_z": float(ground_z),
    }


def get_fixed_frame_points(specs: BikeSpecs) -> Dict[str, np.ndarray]:
    """
    Returns fixed frame pivot points and reference coordinates in mm.

    Includes:
        P0: Main pivot (chainstay to front triangle)
        P5: Rocker frame pivot
        P7: Shock upper mount
        P8: Top tube kink / bend before headtube
        P9: Seatpost clamp / collar
        P10: Seat tube junction (Tube 1 / Seat tube meeting point)
        P11: Top headtube (Reach, Stack)
        P_HT_bot: Bottom headtube
        BB: Bottom bracket origin (0, 0, 0)

    Args:
        specs: Bicycle specifications.

    Returns:
        Dictionary mapping point names to 3D np.ndarray coordinates in mm.
    """
    p11_top, p_ht_bot = compute_headtube_points(specs)

    p10_st_junc = np.array([-60.0, 0.0, 400.0], dtype=float)
    p8_kink = np.array([410.0, 0.0, 635.0], dtype=float)
    p9_clamp = np.array([-85.0, 0.0, 490.0], dtype=float)

    points = {
        "P0": np.array([-44.902156, 0.0, 46.392796], dtype=float),
        "P5": np.array([41.321097, 0.0, 184.456762], dtype=float),
        "P7": np.array([173.220585, 0.0, 403.650931], dtype=float),
        "P8": p8_kink,
        "P9": p9_clamp,
        "P10": p10_st_junc,
        "P11": p11_top,
        "P_HT_bot": p_ht_bot,
        "BB": np.array([0.0, 0.0, 0.0], dtype=float),
    }
    return points


def compute_frame_angles(specs: BikeSpecs) -> Dict[str, float]:
    """
    Computes frame tube inclination angles and Angle 1 (seat tube to Tube 1).

    Angles computed (in degrees):
        - angle_x_tube1: Inclination of Tube 1 (P10 -> P8) with horizontal +X axis.
        - angle_x_tube2: Inclination of Tube 2 (BB -> P_HT_bot) with horizontal +X axis.
        - angle_seat_tube_x: Inclination of Seat Tube (P10 -> P9) with horizontal +X axis.
        - effective_seat_tube_angle: Actual Seat Tube Angle from horizontal -X axis (180 - angle_seat_tube_x).
        - angle_1: Interior angle between Seat Tube (P10 -> P9) and Tube 1 (P10 -> P8).
        - head_angle: Head tube angle (specs.head_angle_deg).

    Args:
        specs: Bicycle specifications.

    Returns:
        Dictionary of frame angles in degrees.
    """
    pts = get_fixed_frame_points(specs)
    p10 = pts["P10"]
    p8 = pts["P8"]
    p9 = pts["P9"]
    p_ht_bot = pts["P_HT_bot"]
    bb = pts["BB"]

    # Tube 1: P10 -> P8
    v_tube1 = p8 - p10
    angle_x_tube1 = float(np.degrees(np.arctan2(v_tube1[2], v_tube1[0])))

    # Tube 2: BB -> P_HT_bot
    v_tube2 = p_ht_bot - bb
    angle_x_tube2 = float(np.degrees(np.arctan2(v_tube2[2], v_tube2[0])))

    # Seat tube: P10 -> P9
    v_st = p9 - p10
    angle_seat_tube_x = float(np.degrees(np.arctan2(v_st[2], v_st[0])))
    sta = float(180.0 - angle_seat_tube_x)

    # Angle 1: Interior angle between Seat Tube (upward-backward) and Tube 1 (upward-forward)
    cos_a1 = float(
        np.dot(v_st[[0, 2]], v_tube1[[0, 2]])
        / (np.linalg.norm(v_st[[0, 2]]) * np.linalg.norm(v_tube1[[0, 2]]))
    )
    angle_1 = float(np.degrees(np.arccos(np.clip(cos_a1, -1.0, 1.0))))

    return {
        "angle_x_tube1_deg": angle_x_tube1,
        "angle_x_tube2_deg": angle_x_tube2,
        "angle_seat_tube_x_deg": angle_seat_tube_x,
        "effective_seat_tube_angle_deg": sta,
        "angle_1_deg": angle_1,
        "head_angle_deg": float(specs.head_angle_deg),
    }
