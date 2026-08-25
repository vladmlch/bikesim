"""
MuJoCo MJCF Model and Geometry Exporters.

Provides file-export utilities for standard/stand/dynamic/playground models and
hardpoint JSON metadata.
"""

import json
from pathlib import Path
from typing import Any, Dict, Optional, Union
import xml.etree.ElementTree as ET
import numpy as np

from bike_sim.geometry.hardpoints import (
    compute_frame_angles,
    compute_front_axle,
    compute_trail,
    get_fixed_frame_points,
)
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.kinematics.solver import HorstLinkageSolver
from bike_sim.physics.air_spring import AirSpringSpecs
from bike_sim.physics.damper import DamperClickConfig
from bike_sim.physics.mass import BikeMassSpecs
from bike_sim.mujoco.builder import generate_mujoco_xml


def export_mujoco(
    output_path: Union[str, Path] = "bike_model.xml",
    specs: Optional[BikeSpecs] = None,
    solver: Optional[HorstLinkageSolver] = None,
    mode: str = "standard",
    mass_specs: Optional[BikeMassSpecs] = None,
    include_rider: bool = False,
    debug_markers: bool = False,
) -> str:
    """
    Exports the generated MuJoCo MJCF XML model to a file.
    """
    xml_content = generate_mujoco_xml(
        specs=specs,
        solver=solver,
        mode=mode,
        mass_specs=mass_specs,
        include_rider=include_rider,
        debug_markers=debug_markers,
    )

    # Validate XML structure with ElementTree before writing
    try:
        ET.fromstring(xml_content)
    except ET.ParseError as e:
        raise ValueError(f"Generated MuJoCo XML failed ElementTree validation: {e}") from e

    target_file = Path(output_path)
    target_file.parent.mkdir(parents=True, exist_ok=True)
    with open(target_file, "w", encoding="utf-8") as f:
        f.write(xml_content)

    print(f"[SUCCESS] Exported MuJoCo MJCF XML model ({mode} mode) to {target_file.resolve()} ({len(xml_content)} bytes)")
    return xml_content


def export_playground_models(
    output_dir: Union[str, Path] = "output/models",
    specs: Optional[BikeSpecs] = None,
    solver: Optional[HorstLinkageSolver] = None,
    include_rider: bool = False,
) -> Dict[str, str]:
    """
    Exports standard, test stand, and unified interactive playground MJCF models.
    """
    out_path = Path(output_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    stand_file = out_path / "bike_playground_stand.xml"
    std_file = out_path / "bike_model.xml"
    playground_file = out_path / "bike_playground.xml"
    ride_file = out_path / "bike_ride.xml"

    xml_stand = export_mujoco(output_path=stand_file, specs=specs, solver=solver, mode="stand", include_rider=include_rider)
    xml_std = export_mujoco(output_path=std_file, specs=specs, solver=solver, mode="standard")
    xml_playground = export_mujoco(
        output_path=playground_file,
        specs=specs,
        solver=solver,
        mode="playground",
        include_rider=include_rider,
        debug_markers=True,
    )
    xml_ride = export_mujoco(
        output_path=ride_file,
        specs=specs,
        solver=solver,
        mode="ride",
        include_rider=True,
    )
    return {
        "stand": xml_stand,
        "standard": xml_std,
        "playground": xml_playground,
        "dynamic": xml_playground,
        "ride": xml_ride,
    }


def _collect_hardpoint_dict(fixed: Dict[str, np.ndarray], state: Dict[str, Any], specs: BikeSpecs) -> Dict[str, np.ndarray]:
    """Assembles all moving and fixed hardpoints for a given kinematic state."""
    return {
        "P0": fixed["P0"],
        "P1": state["P1"],
        "P2": state["P2"],
        "P3": state["P3"],
        "P4": state["P4"],
        "P5": fixed["P5"],
        "P6": state["P6"],
        "P7": fixed["P7"],
        "P8": fixed["P8"],
        "P9": fixed["P9"],
        "P10": fixed["P10"],
        "P11": fixed["P11"],
        "P12": state["P12"],
        "P_HT_bot": fixed["P_HT_bot"],
        "P_FA": compute_front_axle(specs),
        "BB": fixed["BB"],
    }


def _format_point_map(pts: Dict[str, np.ndarray], scale: float = 1.0, decimals: int = 4) -> Dict[str, list[float]]:
    """Converts a dict of 3D points to rounded floating point lists."""
    return {k: [round(float(c) * scale, decimals) for c in v] for k, v in pts.items()}


def _extract_link_invariants(
    solver: HorstLinkageSolver,
    uncomp_pts: Dict[str, np.ndarray],
    st180: Dict[str, Any],
    specs: BikeSpecs,
) -> Dict[str, float]:
    """Computes rigid link invariant lengths and shock dimensions."""
    return {
        "chainstay_p0_p2_mm": round(float(solver.l_cs), 4),
        "seatstay_p2_p3_mm": round(float(solver.l_ss), 4),
        "seatstay_tube_p3_p12_mm": round(float(solver.l_ss_tube), 4),
        "dropout_height_p12_p1_mm": round(float(solver.l_dropout_height), 4),
        "dropout_base_p2_p1_mm": round(float(solver.r_21), 4),
        "dropout_front_p2_p12_mm": round(float(solver.r_2_12), 4),
        "rocker_p5_p3_mm": round(float(solver.r_53), 4),
        "rocker_p5_p4_mm": round(float(solver.r_54), 4),
        "rocker_span_p3_p4_mm": round(float(np.linalg.norm(uncomp_pts["P3"] - uncomp_pts["P4"])), 4),
        "rear_axle_offset_p2_p1_mm": round(float(solver.r_21), 4),
        "shock_yoke_p4_p6_mm": round(float(solver.l_yoke), 4),
        "shock_eye_to_eye_uncompressed_mm": round(float(solver.l_shock_0), 4),
        "shock_eye_to_eye_compressed_mm": round(float(st180["shock_length"]), 4),
        "shock_stroke_mm": round(float(st180["shock_stroke"]), 4),
        "headtube_length_mm": round(float(specs.headtube_length), 4),
    }


def _extract_geometry_summary(
    specs: BikeSpecs,
    trail_info: Dict[str, Any],
    frame_angles: Dict[str, Any],
    traj: Dict[str, Any],
) -> Dict[str, Any]:
    """Builds comprehensive bicycle geometry and kinematic summary."""
    return {
        "reach_mm": float(specs.reach),
        "stack_mm": float(specs.stack),
        "head_angle_deg": float(specs.head_angle_deg),
        "effective_seat_angle_deg": float(specs.effective_seat_angle_deg),
        "wheelbase_mm": float(specs.wheelbase),
        "bb_drop_mm": float(specs.bb_drop),
        "wheel_radius_mm": float(specs.wheel_radius),
        "fork_travel_mm": float(specs.fork_travel),
        "fork_offset_mm": float(specs.fork_offset),
        "ground_trail_mm": round(float(trail_info["ground_trail"]), 4),
        "mechanical_trail_mm": round(float(trail_info["mechanical_trail"]), 4),
        "axle_to_crown_mm": round(float(trail_info["axle_to_crown"]), 4),
        "rear_wheel_travel_mm": float(specs.rear_wheel_travel),
        "angle_x_tube1_deg": round(float(frame_angles["angle_x_tube1_deg"]), 3),
        "angle_x_tube2_deg": round(float(frame_angles["angle_x_tube2_deg"]), 3),
        "angle_1_deg": round(float(frame_angles["angle_1_deg"]), 3),
        "initial_leverage_ratio": round(float(traj["initial_leverage_ratio"]), 4),
        "final_leverage_ratio": round(float(traj["final_leverage_ratio"]), 4),
        "progressivity_pct": round(float(traj["progressivity_pct"]), 3),
        "initial_transmission_angle_deg": round(float(traj["transmission_angle_deg"][0]), 3),
        "final_transmission_angle_deg": round(float(traj["transmission_angle_deg"][-1]), 3),
        "max_link_error_mm": float(traj["max_link_error"]),
    }


def _extract_damper_specs(specs: BikeSpecs) -> Dict[str, Any]:
    """Builds metadata for fork and rear shock damping configurations."""
    damper_cfg = DamperClickConfig()
    return {
        "fork": {
            "model": "RockShox ZEB Ultimate (Charger 3 RC2 + DebonAir+)",
            "travel_mm": float(specs.fork_travel),
            "air_pressure_psi": 85.2,
            "volume_tokens": 2,
            "hsc_clicks": int(damper_cfg.fork_hsc),
            "hsc_range": "0..4 clicks (0=Open, 2=Neutral, 4=Firm)",
            "lsc_clicks": int(damper_cfg.fork_lsc),
            "lsc_range": "0..14 clicks (0=Open, 7=Neutral, 14=Firm)",
            "rebound_clicks": int(damper_cfg.fork_rebound),
            "rebound_range": "0..17 clicks (0=Fast/Open, 9=Neutral, 17=Slow/Firm)",
            "hbo": "Fixed Hydraulic Bottom-Out (active last 20 mm)",
        },
        "rear_shock": {
            "model": "RockShox Super Deluxe Ultimate (RC2T + HBO)",
            "eye_to_eye_mm": float(specs.shock_eye_to_eye),
            "stroke_mm": float(specs.shock_stroke),
            "hsc_clicks": int(damper_cfg.shock_hsc),
            "hsc_range": "0..4 clicks (0=Open, 2=Neutral, 4=Firm)",
            "lsc_clicks": int(damper_cfg.shock_lsc),
            "lsc_range": "0..14 clicks (0=Open, 7=Neutral, 14=Firm)",
            "rebound_clicks": int(damper_cfg.shock_rebound),
            "rebound_range": "0..14 clicks (0=Fast/Open, 7=Neutral, 14=Slow/Firm)",
            "hbo_clicks": int(damper_cfg.shock_hbo),
            "hbo_range": "0..4 clicks (0=Min HBO, 2=Neutral, 4=Max HBO)",
            "lockout_state": "Firm" if damper_cfg.shock_lockout else "Open",
        },
    }


def export_json(
    output_path: Union[str, Path] = "coordinates.json",
    specs: Optional[BikeSpecs] = None,
    solver: Optional[HorstLinkageSolver] = None,
) -> str:
    """
    Exports complete bicycle hardpoint coordinates (in mm and meters), rigid link
    invariants, and suspension geometry metadata to a formatted JSON file.
    """
    if specs is None:
        specs = BikeSpecs()
    if solver is None:
        solver = HorstLinkageSolver(specs)

    fixed = get_fixed_frame_points(specs)
    st0 = solver.solve_state_from_wheel_travel(0.0)
    st180 = solver.solve_state_from_wheel_travel(specs.rear_wheel_travel)
    trail_info = compute_trail(specs)
    frame_angles = compute_frame_angles(specs)
    traj = solver.solve_trajectory(n_points=101, max_travel=specs.rear_wheel_travel)

    uncomp_pts = _collect_hardpoint_dict(fixed, st0, specs)
    comp_pts = _collect_hardpoint_dict(fixed, st180, specs)

    data = {
        "coordinate_frame": {
            "origin": "Bottom Bracket (BB) [0, 0, 0]",
            "axes": {"+X": "Forward", "+Y": "Lateral (Left)", "+Z": "Upward"},
            "unit": "mm and meters",
        },
        "uncompressed_points_mm": _format_point_map(uncomp_pts, scale=1.0, decimals=4),
        "uncompressed_points_m": _format_point_map(uncomp_pts, scale=0.001, decimals=6),
        "compressed_points_mm": _format_point_map(comp_pts, scale=1.0, decimals=4),
        "compressed_points_m": _format_point_map(comp_pts, scale=0.001, decimals=6),
        "link_lengths_mm": _extract_link_invariants(solver, uncomp_pts, st180, specs),
        "geometry_summary": _extract_geometry_summary(specs, trail_info, frame_angles, traj),
        "damper_specifications": _extract_damper_specs(specs),
    }

    target_file = Path(output_path)
    target_file.parent.mkdir(parents=True, exist_ok=True)
    json_str = json.dumps(data, indent=2)
    with open(target_file, "w", encoding="utf-8") as f:
        f.write(json_str)

    print(f"[SUCCESS] Exported hardpoints and geometry data to {target_file.resolve()}")
    return json_str
