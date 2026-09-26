"""
MuJoCo Steering & Fork Subsystem Builder.

Constructs:
1. Steering Assembly (steer hinge, steerer tube, stem, handlebars)
2. Fork Assembly (crown, stanchions, lower slider, arch, dropouts, axle site)
3. Front Wheel (hub, rim, tire, brake rotor)
"""

from typing import Dict, Tuple
import xml.etree.ElementTree as ET
import numpy as np

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.physics.mass import BikeMassSpecs
from bike_sim.mujoco._xml_format import (
    _format_vec,
    add_geom,
    add_joint,
    add_site,
    add_marker,
)


def _build_steerer_and_cockpit(
    frame: ET.Element,
    mode: str,
    p_ht_bot: np.ndarray,
    steer_axis_up: np.ndarray,
    ht_top_rel_steer: np.ndarray,
) -> Tuple[ET.Element, np.ndarray]:
    """Builds the steerer tube body, steering joint, stem, and handlebars."""
    steer = ET.SubElement(frame, "body", {"name": "steer", "pos": _format_vec(p_ht_bot)})
    if mode in ("stand", "dynamic", "playground", "ride"):
        add_joint(steer, "steer_joint", "hinge", axis=steer_axis_up, range="-0.01 0.01", stiffness="50000", damping="1000")
    else:
        add_joint(steer, "steer_joint", "hinge", axis=steer_axis_up, range="-60 60", damping="1.5")

    add_geom(steer, "geom_steerer_tube", "cylinder", fromto=f"0 0 0 {_format_vec(ht_top_rel_steer)}", size="0.014", mass="0.25", material="mat_metal")

    stem_top = ht_top_rel_steer + np.array([0.04, 0.0, 0.02])
    add_geom(steer, "geom_stem", "cylinder", fromto=f"{_format_vec(ht_top_rel_steer)} {_format_vec(stem_top)}", size="0.016", mass="0.15", material="mat_handlebar")
    add_geom(steer, "geom_handlebar", "cylinder", fromto=f"{stem_top[0]:.6f} -0.40 {stem_top[2]:.6f} {stem_top[0]:.6f} 0.40 {stem_top[2]:.6f}", size="0.0155", mass="0.40", material="mat_handlebar")
    if mode == "ride":
        add_site(steer, "site_handlebar", pos=stem_top, size="0.008", rgba="0.9 0.6 0.1 1.0")
    return steer, stem_top


def _build_fork_crown_and_stanchions(
    steer: ET.Element,
    steer_axis_up: np.ndarray,
    mass_specs: BikeMassSpecs,
) -> None:
    """Builds fork crown box and dual stanchion tubes."""
    add_geom(steer, "geom_fork_crown", "box", pos="0 0 0", size="0.038 0.085 0.018", mass="0.30", material="mat_fork_lower")

    stanchion_len = 0.320
    stanchion_tip = -stanchion_len * steer_axis_up
    stanchion_each_mass = mass_specs.stanchions_mass / 2.0
    for side, y in [("l", 0.065), ("r", -0.065)]:
        add_geom(
            steer,
            f"geom_stanchion_{side}",
            "cylinder",
            fromto=f"0 {y} 0 {stanchion_tip[0]:.6f} {y} {stanchion_tip[2]:.6f}",
            size="0.019",
            mass=f"{stanchion_each_mass:.3f}",
            material="mat_kashima",
        )


def _build_fork_lowers(
    steer: ET.Element,
    mode: str,
    fork_slide_axis: np.ndarray,
    steer_axis_up: np.ndarray,
    fa_rel_steer: np.ndarray,
    fork_travel_m: float,
    debug_markers: bool = False,
) -> ET.Element:
    """Builds fork lower slider body, sliding joint, magnesium lowers, arch, and dropouts."""
    fork_lower = ET.SubElement(steer, "body", {"name": "fork_lower", "pos": "0 0 0"})
    add_joint(
        fork_lower,
        "fork_travel",
        "slide",
        axis=fork_slide_axis,
        range=f"0 {fork_travel_m:.6f}",
        stiffness="0",
        damping="0",
        springref="0",
        solreflimit="0.01 1" if mode == "ride" else None,
    )

    lower_top = fa_rel_steer + (0.330 * steer_axis_up)
    add_geom(fork_lower, "geom_fork_lower_l", "capsule", fromto=f"{lower_top[0]:.6f} 0.065 {lower_top[2]:.6f} {fa_rel_steer[0]:.6f} 0.065 {fa_rel_steer[2]:.6f}", size="0.022", mass="0.55", material="mat_fork_lower")
    add_geom(fork_lower, "geom_fork_lower_r", "capsule", fromto=f"{lower_top[0]:.6f} -0.065 {lower_top[2]:.6f} {fa_rel_steer[0]:.6f} -0.065 {fa_rel_steer[2]:.6f}", size="0.022", mass="0.55", material="mat_fork_lower")
    add_geom(fork_lower, "geom_fork_arch", "capsule", fromto=f"{lower_top[0]:.6f} -0.065 {lower_top[2]:.6f} {lower_top[0]:.6f} 0.065 {lower_top[2]:.6f}", size="0.016", mass="0.25", material="mat_fork_lower")
    add_geom(fork_lower, "geom_front_dropout", "cylinder", fromto=f"{fa_rel_steer[0]:.6f} -0.065 {fa_rel_steer[2]:.6f} {fa_rel_steer[0]:.6f} 0.065 {fa_rel_steer[2]:.6f}", size="0.012", mass="0.15", material="mat_metal")
    add_site(fork_lower, "site_PFA", pos=fa_rel_steer, size="0.008", rgba="0.1 0.7 0.9 1.0")

    if debug_markers:
        add_marker(fork_lower, "marker_pfa", pos=fa_rel_steer, size="0.013")

    return fork_lower


def _build_front_wheel(
    fork_lower: ET.Element,
    mode: str,
    fa_rel_steer: np.ndarray,
    front_wheel_radius_m: float,
    tyre_model: str = "sphere",
) -> ET.Element:
    """Builds front wheel assembly (hub, rim, tire, brake rotor)."""
    front_wheel = ET.SubElement(fork_lower, "body", {"name": "front_wheel", "pos": _format_vec(fa_rel_steer)})
    add_joint(front_wheel, "front_wheel_spin", "hinge", axis="0 1 0", damping="0.01")
    add_geom(front_wheel, "geom_front_hub", "cylinder", fromto="0 -0.055 0 0 0.055 0", size="0.018", mass="0.35", material="mat_metal")
    add_geom(front_wheel, "geom_front_rim", "cylinder", fromto="0 -0.015 0 0 0.015 0", size="0.320", mass="0.45", material="mat_rim", contype="0", conaffinity="0")
    tire_contype = "0" if mode == "ride" else "1"
    add_geom(front_wheel, "geom_front_tire", "cylinder", fromto="0 -0.030 0 0 0.030 0", size=f"{front_wheel_radius_m:.6f}", mass="1.35", material="mat_tire", contype=tire_contype, conaffinity=tire_contype, friction="1.2 0.005 0.0001")
    add_geom(front_wheel, "geom_front_rotor", "cylinder", fromto="0 0.032 0 0 0.034 0", size="0.1015", mass="0.25", material="mat_metal", contype="0", conaffinity="0")
    if mode == "ride":
        # Sphere contact patch: geometrically identical to the cylinder in the sagittal
        # plane, but with no flat end faces to catch on a heightfield prism edge. See
        # docs/RIDE.md section 3. Alpha 0 so it does not render over the tyre.
        add_geom(
            front_wheel,
            "geom_front_contact",
            "sphere",
            pos="0 0 0",
            size=f"{front_wheel_radius_m:.6f}",
            mass="0",
            condim="3",
            friction="1.2 0.005 0.0001",
            solref="-130000 -800",
            contype="0" if tyre_model == "pneumatic" else "1",
            conaffinity="0" if tyre_model == "pneumatic" else "1",
            rgba="0.08 0.08 0.08 0",
        )
    return front_wheel


def build_steering_and_fork(
    frame: ET.Element,
    mode: str,
    specs: BikeSpecs,
    mass_specs: BikeMassSpecs,
    fixed_points: Dict[str, np.ndarray],
    front_axle: np.ndarray,
    debug_markers: bool = False,
    tyre_model: str = "sphere",
) -> Tuple[ET.Element, ET.Element, ET.Element]:
    """
    Builds the steering body, fork lower slider, and front wheel.
    """
    P11 = np.array(fixed_points["P11"], dtype=float) / 1000.0
    P_HT_bot = np.array(fixed_points["P_HT_bot"], dtype=float) / 1000.0
    P_FA = np.array(front_axle, dtype=float) / 1000.0

    front_wheel_radius_m = specs.front_wheel_radius / 1000.0
    fork_travel_m = specs.fork_travel / 1000.0

    v_ht = P11 - P_HT_bot
    steer_axis_up = v_ht / np.linalg.norm(v_ht)
    fork_slide_axis = steer_axis_up
    fa_rel_steer = P_FA - P_HT_bot
    ht_top_rel_steer = P11 - P_HT_bot

    # 1. Steerer & Cockpit
    steer, stem_top = _build_steerer_and_cockpit(frame, mode, P_HT_bot, steer_axis_up, ht_top_rel_steer)

    # 2. Fork Crown & Stanchions
    _build_fork_crown_and_stanchions(steer, steer_axis_up, mass_specs)

    # 3. Fork Lowers Slider
    fork_lower = _build_fork_lowers(steer, mode, fork_slide_axis, steer_axis_up, fa_rel_steer, fork_travel_m, debug_markers)

    # 4. Front Wheel
    front_wheel = _build_front_wheel(
        fork_lower, mode, fa_rel_steer, front_wheel_radius_m, tyre_model
    )

    return steer, fork_lower, front_wheel
