"""
MuJoCo Rear Suspension Linkage, Shock, and Constraints Builder.

Constructs:
1. Chainstay body (main pivot P0, chainstay tubes, bridge, P2 site)
2. Seatstay body (Horst pivot P2, seatstay tubes, dropouts, P3/P12/PRA sites)
3. Rear Wheel assembly (via bike_sim.mujoco.drivetrain)
4. Rocker link body (frame pivot P5, rocker arms, P3/P4 sites)
5. Shock yoke body (yoke pivot P4, yoke arms, bridge, P6 site)
6. Shock shaft & shock body (shock lower eyelet, damper can, piggyback reservoir, P7 site)
7. Equality constraints and Contact exclusions
"""

from typing import Dict
import xml.etree.ElementTree as ET
import numpy as np

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.physics.model_config import SimulationPhysicsConfig
from bike_sim.mujoco._xml_format import (
    _format_vec,
    add_geom,
    add_joint,
    add_site,
    add_marker,
)
from bike_sim.mujoco.drivetrain import build_rear_wheel


def _build_chainstay(
    frame: ET.Element,
    p0: np.ndarray,
    p2_rel_p0: np.ndarray,
    debug_markers: bool = False,
) -> ET.Element:
    """Builds the chainstay body, main pivot joint, and chainstay tubes/bridge."""
    chainstay = ET.SubElement(frame, "body", {"name": "chainstay", "pos": _format_vec(p0)})
    add_joint(chainstay, "main_pivot", "hinge", axis="0 1 0", damping="0.5")
    add_geom(chainstay, "geom_cs_sleeve", "cylinder", fromto="0 -0.040 0 0 0.040 0", size="0.018", mass="0.25", material="mat_metal")
    add_geom(chainstay, "geom_cs_left", "capsule", fromto=f"0 0.045 0 {p2_rel_p0[0]:.6f} 0.074 {p2_rel_p0[2]:.6f}", size="0.016", mass="0.40", material="mat_frame")
    add_geom(chainstay, "geom_cs_right", "capsule", fromto=f"0 -0.045 0 {p2_rel_p0[0]:.6f} -0.074 {p2_rel_p0[2]:.6f}", size="0.016", mass="0.40", material="mat_frame")

    cs_bridge_pos = p2_rel_p0 * 0.40
    add_geom(chainstay, "geom_cs_bridge", "capsule", fromto=f"{cs_bridge_pos[0]:.6f} -0.055 {cs_bridge_pos[2]:.6f} {cs_bridge_pos[0]:.6f} 0.055 {cs_bridge_pos[2]:.6f}", size="0.014", mass="0.20", material="mat_frame")
    add_site(chainstay, "site_P2_cs", pos=p2_rel_p0, size="0.008", rgba="0.1 0.9 0.1 1.0")

    if debug_markers:
        add_marker(chainstay, "marker_p2", pos=p2_rel_p0, size="0.013")
    return chainstay


def _build_seatstay(
    chainstay: ET.Element,
    mode: str,
    p2_rel_p0: np.ndarray,
    p1_rel_p2: np.ndarray,
    p3_rel_p2: np.ndarray,
    p12_rel_p2: np.ndarray,
    rear_wheel_radius_m: float,
    debug_markers: bool = False,
    tyre_model: str = "sphere",
) -> ET.Element:
    """Builds the seatstay body, Horst pivot joint, dropout truss, and rear wheel."""
    seatstay = ET.SubElement(chainstay, "body", {"name": "seatstay", "pos": _format_vec(p2_rel_p0)})
    add_joint(seatstay, "horst_pivot", "hinge", axis="0 1 0", damping="0.3")

    # Horst pivot sleeves
    add_geom(seatstay, "geom_horst_sleeve_l", "cylinder", fromto="0 0.068 0 0 0.080 0", size="0.013", mass="0.075", material="mat_metal")
    add_geom(seatstay, "geom_horst_sleeve_r", "cylinder", fromto="0 -0.080 0 0 -0.068 0", size="0.013", mass="0.075", material="mat_metal")

    # Main seatstay tubes
    add_geom(seatstay, "geom_ss_left", "capsule", fromto=f"{p12_rel_p2[0]:.6f} 0.074 {p12_rel_p2[2]:.6f} {p3_rel_p2[0]:.6f} 0.045 {p3_rel_p2[2]:.6f}", size="0.015", mass="0.25", material="mat_frame")
    add_geom(seatstay, "geom_ss_right", "capsule", fromto=f"{p12_rel_p2[0]:.6f} -0.074 {p12_rel_p2[2]:.6f} {p3_rel_p2[0]:.6f} -0.045 {p3_rel_p2[2]:.6f}", size="0.015", mass="0.25", material="mat_frame")

    # Left & Right Dropout triangular truss
    for side, y in [("_l", 0.074), ("_r", -0.074)]:
        add_geom(seatstay, f"geom_dropout_bottom{side}", "capsule", fromto=f"0 {y} 0 {p1_rel_p2[0]:.6f} {y} {p1_rel_p2[2]:.6f}", size="0.016", mass="0.045", material="mat_frame")
        add_geom(seatstay, f"geom_dropout_rear{side}", "capsule", fromto=f"{p1_rel_p2[0]:.6f} {y} {p1_rel_p2[2]:.6f} {p12_rel_p2[0]:.6f} {y} {p12_rel_p2[2]:.6f}", size="0.016", mass="0.045", material="mat_frame")
        add_geom(seatstay, f"geom_dropout_front{side}", "capsule", fromto=f"0 {y} 0 {p12_rel_p2[0]:.6f} {y} {p12_rel_p2[2]:.6f}", size="0.015", mass="0.045", material="mat_frame")

    # Seatstay cross bridge
    ss_bridge_pos = p3_rel_p2 * 0.70
    add_geom(seatstay, "geom_ss_bridge", "capsule", fromto=f"{ss_bridge_pos[0]:.6f} -0.050 {ss_bridge_pos[2]:.6f} {ss_bridge_pos[0]:.6f} 0.050 {ss_bridge_pos[2]:.6f}", size="0.013", mass="0.13", material="mat_frame")

    # Sites
    add_site(seatstay, "site_P3_ss", pos=p3_rel_p2, size="0.008", rgba="0.9 0.5 0.1 1.0")
    add_site(seatstay, "site_P12_ss", pos=p12_rel_p2, size="0.007", rgba="0.9 0.6 0.1 1.0")
    if debug_markers:
        add_marker(seatstay, "marker_p12", pos=p12_rel_p2, size="0.011")

    add_site(seatstay, "site_PRA", pos=p1_rel_p2, size="0.008", rgba="0.2 0.4 0.9 1.0")
    if debug_markers:
        add_marker(seatstay, "marker_pra", pos=p1_rel_p2, size="0.013")

    # Rear Wheel Assembly
    build_rear_wheel(seatstay, mode, p1_rel_p2, rear_wheel_radius_m, tyre_model)
    return seatstay


def _build_rocker(
    frame: ET.Element,
    p5: np.ndarray,
    p3_rel_p5: np.ndarray,
    p4_rel_p5: np.ndarray,
    debug_markers: bool = False,
) -> ET.Element:
    """Builds the rocker link body, frame pivot, rocker triangular arms, and sites."""
    rocker = ET.SubElement(frame, "body", {"name": "rocker", "pos": _format_vec(p5)})
    add_joint(rocker, "rocker_frame_pivot", "hinge", axis="0 1 0", damping="0.3")
    add_geom(rocker, "geom_rocker_sleeve", "cylinder", fromto="0 -0.045 0 0 0.045 0", size="0.016", mass="0.16", material="mat_metal")

    # Rocker triangular arms (Left & Right pairs)
    add_geom(rocker, "geom_roc_arm_l1", "capsule", fromto=f"0 0.045 0 {p3_rel_p5[0]:.6f} 0.045 {p3_rel_p5[2]:.6f}", size="0.014", mass="0.04", material="mat_linkage")
    add_geom(rocker, "geom_roc_arm_r1", "capsule", fromto=f"0 -0.045 0 {p3_rel_p5[0]:.6f} -0.045 {p3_rel_p5[2]:.6f}", size="0.014", mass="0.04", material="mat_linkage")
    add_geom(rocker, "geom_roc_arm_l2", "capsule", fromto=f"0 0.045 0 {p4_rel_p5[0]:.6f} 0.045 {p4_rel_p5[2]:.6f}", size="0.014", mass="0.04", material="mat_linkage")
    add_geom(rocker, "geom_roc_arm_r2", "capsule", fromto=f"0 -0.045 0 {p4_rel_p5[0]:.6f} -0.045 {p4_rel_p5[2]:.6f}", size="0.014", mass="0.04", material="mat_linkage")
    add_geom(rocker, "geom_roc_arm_l3", "capsule", fromto=f"{p3_rel_p5[0]:.6f} 0.045 {p3_rel_p5[2]:.6f} {p4_rel_p5[0]:.6f} 0.045 {p4_rel_p5[2]:.6f}", size="0.012", mass="0.04", material="mat_linkage")
    add_geom(rocker, "geom_roc_arm_r3", "capsule", fromto=f"{p3_rel_p5[0]:.6f} -0.045 {p3_rel_p5[2]:.6f} {p4_rel_p5[0]:.6f} -0.045 {p4_rel_p5[2]:.6f}", size="0.012", mass="0.04", material="mat_linkage")


    add_site(rocker, "site_P3_rocker", pos=p3_rel_p5, size="0.008", rgba="0.9 0.5 0.1 1.0")
    if debug_markers:
        add_marker(rocker, "marker_p3_roc_l", pos=f"{p3_rel_p5[0]:.6f} 0.045 {p3_rel_p5[2]:.6f}", size="0.011")
        add_marker(rocker, "marker_p3_roc_r", pos=f"{p3_rel_p5[0]:.6f} -0.045 {p3_rel_p5[2]:.6f}", size="0.011")

    add_site(rocker, "site_P4_rocker", pos=p4_rel_p5, size="0.008", rgba="0.7 0.2 0.8 1.0")
    if debug_markers:
        add_marker(rocker, "marker_p4_l", pos=f"{p4_rel_p5[0]:.6f} 0.045 {p4_rel_p5[2]:.6f}", size="0.013")
        add_marker(rocker, "marker_p4_r", pos=f"{p4_rel_p5[0]:.6f} -0.045 {p4_rel_p5[2]:.6f}", size="0.013")

    return rocker


def _build_shock_yoke(
    rocker: ET.Element,
    p4_rel_p5: np.ndarray,
    p6_rel_p4: np.ndarray,
    debug_markers: bool = False,
) -> ET.Element:
    """Builds the shock yoke body, pivot, arms, bridge, and P6 eyelet."""
    shock_yoke = ET.SubElement(rocker, "body", {"name": "shock_yoke", "pos": _format_vec(p4_rel_p5)})
    add_joint(shock_yoke, "yoke_pivot", "hinge", axis="0 1 0", damping="0.2")
    add_geom(shock_yoke, "geom_yoke_pivot_p4", "cylinder", fromto="0 -0.044 0 0 0.044 0", size="0.011", mass="0.06", material="mat_metal")
    add_geom(shock_yoke, "geom_yoke_arm_l", "capsule", fromto=f"0 0.040 0 {p6_rel_p4[0]:.6f} 0.038 {p6_rel_p4[2]:.6f}", size="0.011", mass="0.08", material="mat_yoke")
    add_geom(shock_yoke, "geom_yoke_arm_r", "capsule", fromto=f"0 -0.040 0 {p6_rel_p4[0]:.6f} -0.038 {p6_rel_p4[2]:.6f}", size="0.011", mass="0.08", material="mat_yoke")

    bridge_pos = p6_rel_p4 * 0.75
    add_geom(shock_yoke, "geom_yoke_bridge", "capsule", fromto=f"{bridge_pos[0]:.6f} -0.038 {bridge_pos[2]:.6f} {bridge_pos[0]:.6f} 0.038 {bridge_pos[2]:.6f}", size="0.009", mass="0.04", material="mat_yoke")
    add_geom(shock_yoke, "geom_yoke_eyelet", "cylinder", fromto=f"{p6_rel_p4[0]:.6f} -0.040 {p6_rel_p4[2]:.6f} {p6_rel_p4[0]:.6f} 0.040 {p6_rel_p4[2]:.6f}", size="0.011", mass="0.04", material="mat_metal")
    add_site(shock_yoke, "site_P6_yoke", pos=p6_rel_p4, size="0.008", rgba="0.6 0.1 0.7 1.0")

    if debug_markers:
        add_marker(shock_yoke, "marker_p6_l", pos=f"{p6_rel_p4[0]:.6f} 0.038 {p6_rel_p4[2]:.6f}", size="0.013")
        add_marker(shock_yoke, "marker_p6_r", pos=f"{p6_rel_p4[0]:.6f} -0.038 {p6_rel_p4[2]:.6f}", size="0.013")

    return shock_yoke


def _build_shock_assembly(
    shock_yoke: ET.Element,
    mode: str,
    p6_rel_p4: np.ndarray,
    p7_rel_p6: np.ndarray,
    specs: BikeSpecs,
    physics_config: SimulationPhysicsConfig | None = None,
) -> None:
    """Builds the shock shaft and damper body with trunnion mount, canister, and piggyback."""
    shock_len_m = float(np.linalg.norm(p7_rel_p6))
    shock_slide_axis = -p7_rel_p6 / shock_len_m
    shock_stroke_m = specs.shock_stroke / 1000.0
    if mode == "ride" and physics_config is not None and physics_config.physics_mode == "physical":
        stop_travel_m = physics_config.end_stops.overtravel_m
        joint_range = f"{-stop_travel_m:.6f} {shock_stroke_m + stop_travel_m:.6f}"
    else:
        joint_range = f"0 {shock_stroke_m:.6f}"

    u_shock = p7_rel_p6 / shock_len_m
    shaft_len = 0.145
    shaft_tip_rel_p6 = u_shock * shaft_len
    bottom_out_clearance_m = 0.002
    body_can_len = (specs.shock_eye_to_eye - specs.shock_stroke) / 1000.0 - bottom_out_clearance_m
    body_can_end_rel_p7 = shock_slide_axis * body_can_len
    trunnion_overhang = 0.014
    trunnion_cap_rel_p7 = -shock_slide_axis * trunnion_overhang
    trunnion_half_width = 0.027

    perp = np.array([-shock_slide_axis[2], 0.0, shock_slide_axis[0]])
    piggy_offset = perp * 0.034 + np.array([0.0, 0.008, 0.0])
    piggy_start = (shock_slide_axis * 0.018) + piggy_offset
    piggy_end = (shock_slide_axis * 0.108) + piggy_offset
    piggy_bridge_1 = shock_slide_axis * 0.030
    piggy_bridge_2 = (shock_slide_axis * 0.030) + piggy_offset

    # Shock shaft
    shock_shaft = ET.SubElement(shock_yoke, "body", {"name": "shock_shaft", "pos": _format_vec(p6_rel_p4)})
    add_geom(shock_shaft, "geom_shock_lower_eyelet", "cylinder", fromto="0 -0.016 0 0 0.016 0", size="0.011", mass="0.06", material="mat_metal")
    add_geom(shock_shaft, "geom_shock_shaft", "cylinder", fromto=f"0 0 0 {_format_vec(shaft_tip_rel_p6)}", size="0.014", mass="0.14", material="mat_kashima")
    add_geom(
        shock_shaft,
        "geom_shock_travel_indicator",
        "cylinder",
        fromto=f"{_format_vec(u_shock * 0.016)} {_format_vec(u_shock * 0.021)}",
        size="0.0165",
        mass="0.0",
        material="mat_linkage",
        contype="0",
        conaffinity="0",
    )

    # Shock body
    shock_body = ET.SubElement(shock_shaft, "body", {"name": "shock_body", "pos": _format_vec(p7_rel_p6)})
    _shock_body_child_geoms = [
        (0.02, np.array([0.0, 0.019, 0.0]), np.array([0.0, trunnion_half_width, 0.0])),
        (0.02, np.array([0.0, -0.019, 0.0]), np.array([0.0, -trunnion_half_width, 0.0])),
        (0.04, np.zeros(3), trunnion_cap_rel_p7),
        (0.18, np.zeros(3), body_can_end_rel_p7),
        (0.10, piggy_start, piggy_end),
        (0.04, piggy_bridge_1, piggy_bridge_2),
    ]
    _shock_body_pinned_mass = sum(m for m, _, _ in _shock_body_child_geoms)
    ET.SubElement(
        shock_body,
        "inertial",
        {
            "pos": "-0.033395 0.011400 -0.028962",
            "quat": "-0.10166406 0.38183084 -0.06020627 0.91664870",
            "mass": f"{_shock_body_pinned_mass:.6f}",
            "diaginertia": "0.00084836 0.00071226 0.00023249",
        },
    )
    add_joint(
        shock_body,
        "shock_stroke",
        "slide",
        axis=_format_vec(shock_slide_axis),
        range=joint_range,
        stiffness="0",
        damping="0",
        springref="0",
        solreflimit="0.01 1" if mode == "ride" else None,
    )

    add_geom(shock_body, "geom_shock_trunnion_boss_l", "cylinder", fromto=f"0 0.019 0 0 {trunnion_half_width:.6f} 0", size="0.013", mass="0.02", material="mat_metal")
    add_geom(shock_body, "geom_shock_trunnion_boss_r", "cylinder", fromto=f"0 -0.019 0 0 {-trunnion_half_width:.6f} 0", size="0.013", mass="0.02", material="mat_metal")
    add_geom(shock_body, "geom_shock_trunnion_overhang", "cylinder", fromto=f"0 0 0 {_format_vec(trunnion_cap_rel_p7)}", size="0.024", mass="0.04", material="mat_damper_body")
    add_geom(shock_body, "geom_shock_body", "cylinder", fromto=f"0 0 0 {_format_vec(body_can_end_rel_p7)}", size="0.026", mass="0.18", material="mat_damper_body")
    add_geom(shock_body, "geom_shock_piggyback", "cylinder", fromto=f"{_format_vec(piggy_start)} {_format_vec(piggy_end)}", size="0.018", mass="0.10", material="mat_damper_body")
    add_geom(shock_body, "geom_shock_piggy_bridge", "cylinder", fromto=f"{_format_vec(piggy_bridge_1)} {_format_vec(piggy_bridge_2)}", size="0.010", mass="0.04", material="mat_damper_body")
    add_site(shock_body, "site_P7_shock", pos="0 0 0", size="0.008", rgba="0.1 0.6 0.9 1.0")


def build_rear_linkage(
    frame: ET.Element,
    mode: str,
    specs: BikeSpecs,
    fixed_points: Dict[str, np.ndarray],
    solved_points: Dict[str, np.ndarray],
    debug_markers: bool = False,
    tyre_model: str = "sphere",
    physics_config: SimulationPhysicsConfig | None = None,
) -> None:
    """
    Builds the 4-bar rear suspension linkage, shock yoke, shock shaft, shock body, and rear wheel.
    """
    P0 = np.array(fixed_points["P0"], dtype=float) / 1000.0
    P5 = np.array(fixed_points["P5"], dtype=float) / 1000.0
    P7 = np.array(fixed_points["P7"], dtype=float) / 1000.0

    P1 = np.array(solved_points["P1"], dtype=float) / 1000.0
    P2 = np.array(solved_points["P2"], dtype=float) / 1000.0
    P3 = np.array(solved_points["P3"], dtype=float) / 1000.0
    P4 = np.array(solved_points["P4"], dtype=float) / 1000.0
    P6 = np.array(solved_points["P6"], dtype=float) / 1000.0
    P12 = np.array(solved_points["P12"], dtype=float) / 1000.0

    rear_wheel_radius_m = specs.rear_wheel_radius / 1000.0

    p2_rel_p0 = P2 - P0
    p3_rel_p2 = P3 - P2
    p1_rel_p2 = P1 - P2
    p12_rel_p2 = P12 - P2

    p3_rel_p5 = P3 - P5
    p4_rel_p5 = P4 - P5
    p6_rel_p4 = P6 - P4
    p7_rel_p6 = P7 - P6

    # 1. Chainstay
    chainstay = _build_chainstay(frame, P0, p2_rel_p0, debug_markers)

    # 2. Seatstay & Rear Wheel
    _build_seatstay(
        chainstay,
        mode,
        p2_rel_p0,
        p1_rel_p2,
        p3_rel_p2,
        p12_rel_p2,
        rear_wheel_radius_m,
        debug_markers,
        tyre_model,
    )

    # 3. Rocker Link
    rocker = _build_rocker(frame, P5, p3_rel_p5, p4_rel_p5, debug_markers)

    # 4. Shock Yoke
    shock_yoke = _build_shock_yoke(rocker, p4_rel_p5, p6_rel_p4, debug_markers)

    # 5. Shock Shaft & Shock Body
    _build_shock_assembly(shock_yoke, mode, p6_rel_p4, p7_rel_p6, specs, physics_config)


def build_equality_constraints(root: ET.Element, mode: str) -> None:
    """
    Appends loop closure equality constraints (<equality>) at P3 and P7, plus stand clamp if in playground mode.
    """
    equality = ET.SubElement(root, "equality")
    ET.SubElement(
        equality,
        "connect",
        {
            "name": "seatstay_rocker_joint",
            "site1": "site_P3_ss",
            "site2": "site_P3_rocker",
            "solref": "0.0005 1",
            "solimp": "0.99 0.999 0.0001 0.5 2",
        },
    )
    ET.SubElement(
        equality,
        "connect",
        {
            "name": "shock_frame_joint",
            "site1": "site_P7_shock",
            "site2": "site_P7",
            "solref": "0.0005 1",
            "solimp": "0.99 0.999 0.0001 0.5 2",
        },
    )
    if mode == "playground":
        ET.SubElement(
            equality,
            "weld",
            {
                "name": "stand_clamp",
                "body1": "world",
                "body2": "frame",
                "active": "true",
            },
        )


def build_contact_exclusions(root: ET.Element) -> None:
    """
    Appends contact exclusion pairs between all internal bicycle bodies.
    """
    contact = ET.SubElement(root, "contact")
    link_bodies = [
        "frame",
        "steer",
        "fork_lower",
        "chainstay",
        "seatstay",
        "rocker",
        "shock_yoke",
        "shock_shaft",
        "shock_body",
        "front_wheel",
        "rear_wheel",
    ]
    for i in range(len(link_bodies)):
        for j in range(i + 1, len(link_bodies)):
            ET.SubElement(contact, "exclude", {"body1": link_bodies[i], "body2": link_bodies[j]})
