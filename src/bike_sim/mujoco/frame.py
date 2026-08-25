"""
MuJoCo Frame Body & Front Triangle Builder.

Constructs the base `frame` body with front triangle tubes, downtube battery box,
headtube, toptube, seattube, casting, seatpost, saddle, motor, cranks, mount bosses,
and hardpoint sites.
"""

from typing import Dict
import xml.etree.ElementTree as ET
import numpy as np

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.physics.mass import BikeMassSpecs
from bike_sim.mujoco.rider import build_rider
from bike_sim.mujoco.drivetrain import build_bb_and_motor
from bike_sim.mujoco._xml_format import (
    _format_fromto,
    _format_vec,
    add_geom,
    add_joint,
    add_site,
    add_marker,
)


def _build_front_triangle_tubes(
    frame: ET.Element,
    BB: np.ndarray,
    P_HT_bot: np.ndarray,
    P11: np.ndarray,
    P8: np.ndarray,
    P10: np.ndarray,
    mass_specs: BikeMassSpecs,
) -> None:
    """Builds downtube, battery pack, headtube, and top tube sections."""
    downtube_mid = (BB + P_HT_bot) / 2.0
    downtube_vec = P_HT_bot - BB
    downtube_len = float(np.linalg.norm(downtube_vec))
    downtube_angle_deg = float(np.degrees(np.arctan2(downtube_vec[2], downtube_vec[0])))

    _dt_sin = downtube_vec[2] / downtube_len
    _dt_cos = downtube_vec[0] / downtube_len
    _dt_bands_mm = ((255.0, 150.0, 266.0), (323.0, 208.0, 305.0))
    downtube_half_thickness = float(
        np.mean([(x_hi - x_lo) * _dt_sin / 2.0 for _, x_lo, x_hi in _dt_bands_mm])
    ) / 1000.0
    downtube_perp_offset = float(
        np.mean(
            [
                (x_lo + x_hi) / 2.0 * -_dt_sin + z * _dt_cos
                for z, x_lo, x_hi in _dt_bands_mm
            ]
        )
    ) / 1000.0
    downtube_perp_axis = np.array([-_dt_sin, 0.0, _dt_cos])
    downtube_pos = downtube_mid + downtube_perp_offset * downtube_perp_axis

    add_geom(
        frame,
        "geom_downtube",
        "box",
        pos=_format_vec(downtube_pos),
        size=f"{downtube_len / 2.0:.6f} 0.038 {downtube_half_thickness:.6f}",
        euler=f"0 {-downtube_angle_deg:.6f} 0",
        mass="0.60",
        material="mat_frame",
    )

    battery_axis_a = np.array([0.080, 0.0, 0.080]) + downtube_perp_offset * downtube_perp_axis
    battery_axis_b = np.array([0.320, 0.0, 0.320]) + downtube_perp_offset * downtube_perp_axis
    add_geom(
        frame,
        "geom_battery_pack",
        "capsule",
        fromto=_format_fromto(battery_axis_a, battery_axis_b),
        size="0.032",
        mass=f"{mass_specs.battery_mass:.2f}",
        material="mat_battery",
    )
    add_geom(frame, "geom_headtube", "cylinder", fromto=_format_fromto(P_HT_bot, P11), size="0.027", mass="0.40", material="mat_frame")
    add_geom(frame, "geom_toptube_bridge", "capsule", fromto=_format_fromto(P11, P8), size="0.024", mass="0.25", material="mat_frame")
    add_geom(frame, "geom_toptube_main", "capsule", fromto=_format_fromto(P8, P10), size="0.023", mass="0.45", material="mat_frame")


def _build_seatpost_and_saddle(
    frame: ET.Element,
    P10: np.ndarray,
    P9: np.ndarray,
    P8: np.ndarray,
    P7: np.ndarray,
    P5: np.ndarray,
    mass_specs: BikeMassSpecs,
) -> np.ndarray:
    """Builds seat tube, bottom casting, seatpost, gusset, and saddle. Returns seatpost_top pos."""
    seattube_end = np.array([-0.030, 0.0, 0.258])
    SEATTUBE_CASTING_MASS_KG = 0.65
    seattube_mass_kg = 0.05
    casting_mass_kg = SEATTUBE_CASTING_MASS_KG - seattube_mass_kg

    add_geom(frame, "geom_seattube", "capsule", fromto=_format_fromto(P10, seattube_end), size="0.020", mass=f"{seattube_mass_kg:.2f}", material="mat_frame")
    add_geom(
        frame,
        "geom_frame_casting",
        "box",
        pos=f"{(seattube_end[0] + P5[0]) / 2.0:.6f} 0 {(seattube_end[2] + 0.0) / 2.0:.6f}",
        size="0.075 0.036 0.130",
        mass=f"{casting_mass_kg:.2f}",
        material="mat_frame",
        contype="0",
        conaffinity="0",
    )

    PHOTO_SADDLE_TOP_Z_M = 0.690
    SADDLE_HALF_THICKNESS_M = 0.015
    SADDLE_SETBACK_FROM_POST_TOP_M = 0.024
    SEATPOST_MASS_KG = 0.35

    seatpost_axis = np.array([-146.0 - -83.0, 0.0, 624.0 - 430.0])
    seatpost_axis = seatpost_axis / float(np.linalg.norm(seatpost_axis))
    saddle_underside_z = PHOTO_SADDLE_TOP_Z_M - 2.0 * SADDLE_HALF_THICKNESS_M
    seatpost_top = P9 + seatpost_axis * ((saddle_underside_z - P9[2]) / seatpost_axis[2])
    saddle_pos = np.array(
        [
            seatpost_top[0] - SADDLE_SETBACK_FROM_POST_TOP_M,
            0.0,
            seatpost_top[2] + SADDLE_HALF_THICKNESS_M,
        ]
    )

    seatpost_lower_len = float(np.linalg.norm(P9 - P10))
    seatpost_upper_len = float(np.linalg.norm(seatpost_top - P9))
    seatpost_lower_mass = round(
        SEATPOST_MASS_KG * seatpost_lower_len / (seatpost_lower_len + seatpost_upper_len), 6
    )
    seatpost_upper_mass = SEATPOST_MASS_KG - seatpost_lower_mass

    add_geom(frame, "geom_seatpost", "cylinder", fromto=_format_fromto(P10, P9), size="0.016", mass=f"{seatpost_lower_mass:.6f}", material="mat_metal")
    add_geom(frame, "geom_seatpost_upper", "cylinder", fromto=_format_fromto(P9, seatpost_top), size="0.016", mass=f"{seatpost_upper_mass:.6f}", material="mat_metal")

    p_gusset_mid = P10 + (P8 - P10) * 0.22
    add_geom(frame, "geom_seat_gusset", "capsule", fromto=_format_fromto(P9, p_gusset_mid), size="0.015", mass="0.10", material="mat_frame")

    p_junction_top = P10 + (P8 - P10) * 0.52
    add_geom(frame, "geom_frame_junction", "capsule", fromto=_format_fromto(p_junction_top, P7), size="0.042", mass="0.10", material="mat_frame")

    saddle_mass = max(0.20, mass_specs.saddle_post_mass - 0.35)
    add_geom(
        frame,
        "geom_saddle",
        "box",
        pos=_format_vec(saddle_pos),
        size=f"0.13 0.065 {SADDLE_HALF_THICKNESS_M:.3f}",
        mass=f"{saddle_mass:.2f}",
        material="mat_saddle",
    )
    return seatpost_top


def _build_mounts_and_sites(
    frame: ET.Element,
    fixed_points_m: Dict[str, np.ndarray],
    BB: np.ndarray,
    cg_pos: np.ndarray,
    seatpost_top: np.ndarray,
    debug_markers: bool = False,
) -> None:
    """Builds pivot mount bosses, hardpoint sites, and debug spheres."""
    P0 = fixed_points_m["P0"]
    P5 = fixed_points_m["P5"]
    P7 = fixed_points_m["P7"]
    P8 = fixed_points_m["P8"]
    P9 = fixed_points_m["P9"]
    P10 = fixed_points_m["P10"]
    P11 = fixed_points_m["P11"]
    P_HT_bot = fixed_points_m["P_HT_bot"]

    # Mount Bosses
    add_geom(frame, "geom_p0_mount", "cylinder", fromto=f"{P0[0]:.6f} -0.035 {P0[2]:.6f} {P0[0]:.6f} 0.035 {P0[2]:.6f}", size="0.016", mass="0.10", material="mat_metal")
    add_geom(frame, "geom_p5_mount", "cylinder", fromto=f"{P5[0]:.6f} -0.048 {P5[2]:.6f} {P5[0]:.6f} 0.048 {P5[2]:.6f}", size="0.013", mass="0.10", material="mat_metal")
    add_geom(frame, "geom_p7_mount", "cylinder", fromto=f"{P7[0]:.6f} -0.020 {P7[2]:.6f} {P7[0]:.6f} 0.020 {P7[2]:.6f}", size="0.012", mass="0.10", material="mat_metal")

    # Hardpoint Sites
    add_site(frame, "site_BB", pos=BB, size="0.008", rgba="0.9 0.6 0.1 1.0")
    add_site(frame, "site_P0", pos=P0, size="0.008", rgba="0.2 0.8 0.2 1.0")
    add_site(frame, "site_P5", pos=P5, size="0.008", rgba="0.8 0.2 0.2 1.0")
    add_site(frame, "site_P7", pos=P7, size="0.008", rgba="0.1 0.6 0.9 1.0")
    add_site(frame, "site_P8", pos=P8, size="0.006", rgba="0.5 0.5 0.5 1.0")
    add_site(frame, "site_P9", pos=P9, size="0.006", rgba="0.5 0.5 0.5 1.0")
    add_site(frame, "site_P10", pos=P10, size="0.006", rgba="0.5 0.5 0.5 1.0")
    add_site(frame, "site_P11", pos=P11, size="0.008", rgba="0.9 0.1 0.1 1.0")
    add_site(frame, "site_HT_bot", pos=P_HT_bot, size="0.008", rgba="0.9 0.1 0.1 1.0")
    add_site(frame, "site_CG", pos=cg_pos, size="0.012", rgba="0.95 0.20 0.85 0.9")
    add_site(frame, "site_top_tube_kink", pos=P8, size="0.005", rgba="0.5 0.5 0.5 0.8")
    add_site(frame, "site_seat_collar", pos=P9, size="0.005", rgba="0.5 0.5 0.5 0.8")
    add_site(frame, "site_seatpost_top", pos=seatpost_top, size="0.005", rgba="0.5 0.5 0.5 0.8")
    add_site(frame, "site_seat_tube_junction", pos=P10, size="0.005", rgba="0.5 0.5 0.5 0.8")
    add_site(frame, "site_headtube_top", pos=P11, size="0.006", rgba="0.9 0.1 0.1 0.8")
    add_site(frame, "site_headtube_bottom", pos=P_HT_bot, size="0.006", rgba="0.9 0.1 0.1 0.8")

    if debug_markers:
        add_marker(frame, "marker_bb", pos=BB, size="0.013")
        add_marker(frame, "marker_p0", pos=P0, size="0.013")
        add_marker(frame, "marker_p5_l", pos=f"{P5[0]:.6f} 0.048 {P5[2]:.6f}", size="0.013")
        add_marker(frame, "marker_p5_r", pos=f"{P5[0]:.6f} -0.048 {P5[2]:.6f}", size="0.013")
        add_marker(frame, "marker_p7", pos=P7, size="0.013")
        add_marker(frame, "marker_p8", pos=P8, size="0.011")
        add_marker(frame, "marker_p9", pos=P9, size="0.011")
        add_marker(frame, "marker_p10", pos=P10, size="0.011")
        add_marker(frame, "marker_p11", pos=P11, size="0.013")
        add_marker(frame, "marker_cg", pos=cg_pos, size="0.014", material="mat_cg_marker")


def build_frame_body(
    worldbody: ET.Element,
    mode: str,
    specs: BikeSpecs,
    mass_specs: BikeMassSpecs,
    fixed_points: Dict[str, np.ndarray],
    cg_pos: np.ndarray,
    include_rider: bool = True,
    debug_markers: bool = False,
) -> ET.Element:
    """
    Constructs the base frame body in worldbody and adds all frame geometry.
    """
    fixed_points_m = {k: np.array(v, dtype=float) / 1000.0 for k, v in fixed_points.items()}
    BB = np.array([0.0, 0.0, 0.0], dtype=float)

    frame = ET.SubElement(worldbody, "body", {"name": "frame", "pos": "0 0 0"})
    if mode == "ride":
        # Planar chassis root: longitudinal, vertical, and pitch, in that order, before
        # the rider and any geometry. See docs/RIDE.md sections 0-1.
        add_joint(frame, "root_x", "slide", axis="1 0 0")
        add_joint(frame, "root_z", "slide", axis="0 0 1")
        add_joint(frame, "root_pitch", "hinge", axis="0 1 0")
    if mode in ("stand", "playground", "ride"):
        build_rider(frame, include_rider=include_rider)

    build_bb_and_motor(frame, mass_specs)
    _build_front_triangle_tubes(
        frame=frame,
        BB=BB,
        P_HT_bot=fixed_points_m["P_HT_bot"],
        P11=fixed_points_m["P11"],
        P8=fixed_points_m["P8"],
        P10=fixed_points_m["P10"],
        mass_specs=mass_specs,
    )
    seatpost_top = _build_seatpost_and_saddle(
        frame=frame,
        P10=fixed_points_m["P10"],
        P9=fixed_points_m["P9"],
        P8=fixed_points_m["P8"],
        P7=fixed_points_m["P7"],
        P5=fixed_points_m["P5"],
        mass_specs=mass_specs,
    )
    _build_mounts_and_sites(
        frame=frame,
        fixed_points_m=fixed_points_m,
        BB=BB,
        cg_pos=cg_pos,
        seatpost_top=seatpost_top,
        debug_markers=debug_markers,
    )

    return frame

