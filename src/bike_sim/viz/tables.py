"""
ASCII Tables & Formatting Utilities for Bicycle Kinematics & Geometry.
"""

from typing import Dict
import numpy as np

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.geometry.hardpoints import (
    compute_frame_angles,
    compute_front_axle,
    compute_trail,
    get_fixed_frame_points,
)
from bike_sim.kinematics.solver import HorstLinkageSolver


POINT_DESCRIPTIONS: Dict[str, str] = {
    "P0": "Main Pivot (Frame / Chainstay)",
    "P1": "Rear Wheel Axle",
    "P2": "Horst Pivot (Chainstay / Seatstay)",
    "P3": "Seatstay-Rocker Pivot",
    "P4": "Rocker-Yoke Pivot",
    "P5": "Rocker Frame Pivot (Seat Tube)",
    "P6": "Shock Lower Eyelet",
    "P7": "Shock Upper Mount (Frame Tab)",
    "P8": "Top Tube Kink / Bend",
    "P9": "Seatpost Clamp / Saddle Base",
    "P10": "Seat Tube Junction (Seat Cluster)",
    "P11": "Headtube Top (Reach, Stack)",
    "P12": "Seatstay Dropout Upper Kink",
    "P_HT_bot": "Headtube Bottom",
    "P_FA": "Front Wheel Axle",
    "BB": "Bottom Bracket Origin (0,0,0)",
}


def get_all_points(
    specs: BikeSpecs, solver: HorstLinkageSolver, wheel_travel: float = 0.0
) -> Dict[str, np.ndarray]:
    """
    Computes a complete dictionary of 3D coordinates for all frame, linkage, and wheel hardpoints
    at a given wheel travel (mm).
    """
    fixed = get_fixed_frame_points(specs)
    st = solver.solve_state_from_wheel_travel(wheel_travel)

    points: Dict[str, np.ndarray] = {
        "P0": np.array(fixed["P0"], dtype=float),
        "P1": np.array(st["P1"], dtype=float),
        "P2": np.array(st["P2"], dtype=float),
        "P3": np.array(st["P3"], dtype=float),
        "P4": np.array(st["P4"], dtype=float),
        "P5": np.array(fixed["P5"], dtype=float),
        "P6": np.array(st["P6"], dtype=float),
        "P7": np.array(fixed["P7"], dtype=float),
        "P8": np.array(fixed["P8"], dtype=float),
        "P9": np.array(fixed["P9"], dtype=float),
        "P10": np.array(fixed["P10"], dtype=float),
        "P11": np.array(fixed["P11"], dtype=float),
        "P12": np.array(st["P12"], dtype=float),
        "P_HT_bot": np.array(fixed["P_HT_bot"], dtype=float),
        "P_FA": np.array(fixed["P_FA"] if "P_FA" in fixed else compute_front_axle(specs), dtype=float),
        "BB": np.array(fixed["BB"], dtype=float),
    }
    return points


def print_tables(specs: BikeSpecs, solver: HorstLinkageSolver) -> None:
    """
    Prints clean, formatted ASCII tables of coordinates (mm and meters) for uncompressed (0mm)
    and compressed (180mm) suspension states, along with a comprehensive summary of bicycle
    geometry and kinematic metrics.
    """
    uncomp_pts = get_all_points(specs, solver, wheel_travel=0.0)
    comp_pts = get_all_points(specs, solver, wheel_travel=180.0)
    trail_info = compute_trail(specs)
    traj = solver.solve_trajectory(n_points=101, max_travel=specs.rear_wheel_travel)
    st180 = solver.solve_state_from_wheel_travel(180.0)

    sep_line = "=" * 98
    sub_sep = "-" * 98

    # Table 1: Uncompressed State Coordinates (0 mm Wheel Travel)
    print("\n" + sep_line)
    print(" UNCOMPRESSED STATE HARDPOINTS (Wheel Travel = 0.0 mm, Shock Stroke = 0.0 mm)")
    print(sep_line)
    header1 = f"{'Point':<10} | {'Description':<32} | {'X (mm)':>9} {'Y (mm)':>7} {'Z (mm)':>9} | {'X (m)':>8} {'Y (m)':>7} {'Z (m)':>8}"
    print(header1)
    print(sub_sep)
    for name, p_mm in uncomp_pts.items():
        desc = POINT_DESCRIPTIONS.get(name, "")
        p_m = p_mm / 1000.0
        print(
            f"{name:<10} | {desc:<32} | {p_mm[0]:>9.3f} {p_mm[1]:>7.3f} {p_mm[2]:>9.3f} | {p_m[0]:>8.4f} {p_m[1]:>7.4f} {p_m[2]:>8.4f}"
        )
    print(sep_line)

    # Table 2: Fully Compressed State Coordinates (180 mm Wheel Travel)
    print("\n" + sep_line)
    print(f" FULLY COMPRESSED STATE HARDPOINTS (Wheel Travel = 180.0 mm, Shock Stroke = {st180['shock_stroke']:.2f} mm)")
    print(sep_line)
    header2 = f"{'Point':<10} | {'Description':<32} | {'X (mm)':>9} {'Y (mm)':>7} {'Z (mm)':>9} | {'dX (mm)':>8} {'dZ (mm)':>8}"
    print(header2)
    print(sub_sep)
    for name, p_mm in comp_pts.items():
        desc = POINT_DESCRIPTIONS.get(name, "")
        dx = p_mm[0] - uncomp_pts[name][0]
        dz = p_mm[2] - uncomp_pts[name][2]
        print(
            f"{name:<10} | {desc:<32} | {p_mm[0]:>9.3f} {p_mm[1]:>7.3f} {p_mm[2]:>9.3f} | {dx:>+8.3f} {dz:>+8.3f}"
        )
    print(sep_line)

    # Table 3: Summary of Bicycle Frame Geometry & Kinematics
    print("\n" + sep_line)
    print(" BICYCLE FRAME GEOMETRY & SUSPENSION KINEMATICS SUMMARY")
    print(sep_line)
    header3 = f"{'Parameter / Metric':<36} | {'Value':>16} | {'Unit':<8} | {'Specification / Notes':<30}"
    print(header3)
    print(sub_sep)

    frame_angles = compute_frame_angles(specs)

    metrics = [
        ("Reach", f"{specs.reach:.1f}", "mm", "BB to Headtube Top (Horizontal)"),
        ("Stack", f"{specs.stack:.1f}", "mm", "BB to Headtube Top (Vertical)"),
        ("Head Tube Angle (HA)", f"{specs.head_angle_deg:.1f}", "deg", "Steering Axis Inclination"),
        ("Effective Seat Tube Angle", f"{specs.effective_seat_angle_deg:.1f}", "deg", "Virtual Seat Angle"),
        ("Actual Seat Tube Angle (STA)", f"{frame_angles['effective_seat_tube_angle_deg']:.2f}", "deg", "Seat Tube Axis relative to -X"),
        ("Angle 1 (Seat Tube to Tube 1)", f"{frame_angles['angle_1_deg']:.2f}", "deg", "Interior angle at Seat Cluster"),
        ("Tube 1 Inclination (X to Tube 1)", f"{frame_angles['angle_x_tube1_deg']:.2f}", "deg", "P10 -> P8 relative to +X"),
        ("Tube 2 Inclination (X to Tube 2)", f"{frame_angles['angle_x_tube2_deg']:.2f}", "deg", "BB -> P_HT_bot relative to +X"),
        ("Wheelbase", f"{specs.wheelbase:.1f}", "mm", "Front Axle to Rear Axle"),
        ("Bottom Bracket Drop", f"{specs.bb_drop:.1f}", "mm", "Axle Plane to BB Height"),
        ("Front Wheel Radius (29\")", f"{specs.front_wheel_radius:.1f}", "mm", "29\" Tire (Diameter 744 mm)"),
        ("Rear Wheel Radius (27.5\")", f"{specs.rear_wheel_radius:.1f}", "mm", "27.5\" Tire (Diameter 704 mm)"),
        ("Ground Plane Z", f"{trail_info['ground_z']:.1f}", "mm", "Flat ground contact plane"),
        ("Fork Travel", f"{specs.fork_travel:.1f}", "mm", "Front Suspension Travel"),
        ("Fork Offset (Rake)", f"{specs.fork_offset:.1f}", "mm", "Perpendicular Offset"),
        ("Ground Trail", f"{trail_info['ground_trail']:.2f}", "mm", "Steer Ground Contact Offset"),
        ("Mechanical Trail", f"{trail_info['mechanical_trail']:.2f}", "mm", "Torque Arm for Self-Centering"),
        ("Axle-to-Crown (A2C)", f"{trail_info['axle_to_crown']:.2f}", "mm", "Fork Length (Uncompressed)"),
        ("Rear Wheel Travel", f"{specs.rear_wheel_travel:.1f}", "mm", "Vertical Axle Travel (+Z)"),
        ("Shock Eye-to-Eye (Uncompressed)", f"{solver.l_shock_0:.2f}", "mm", "Target: 205.0 mm (trunnion)"),
        ("Shock Eye-to-Eye (Compressed)", f"{st180['shock_length']:.2f}", "mm", "Target: 140.0 mm"),
        ("Shock Stroke", f"{st180['shock_stroke']:.2f}", "mm", "Target: 65.00 mm (+-0.05 mm)"),
        ("Initial Leverage Ratio", f"{traj['initial_leverage_ratio']:.3f}", "ratio", "At 0 mm Wheel Travel"),
        ("Final Leverage Ratio", f"{traj['final_leverage_ratio']:.3f}", "ratio", "At 180 mm Wheel Travel"),
        ("Suspension Progressivity", f"+{traj['progressivity_pct']:.2f}", "%", "Progressive Leverage Curve"),
        ("Initial Transmission Angle", f"{traj['transmission_angle_deg'][0]:.2f}", "deg", "Safe Range (50 deg - 140 deg)"),
        ("Final Transmission Angle", f"{traj['transmission_angle_deg'][-1]:.2f}", "deg", "Safe Range (50 deg - 140 deg)"),
        ("Max Rigid Link Invariant Error", f"{traj['max_link_error']:.2e}", "mm", "Requirement < 1e-10 mm"),
        ("Fork Damper Model", "Charger 3 RC2", "model", "RockShox ZEB Ultimate (38mm)"),
        ("Fork HSC Clicks", f"{specs.fork_hsc} / 4", "clicks", "0=Open, 2=Neutral, 4=Firm (+2)"),
        ("Fork LSC Clicks", f"{specs.fork_lsc} / 14", "clicks", "0=Open, 7=Neutral, 14=Firm"),
        ("Fork Rebound Clicks", f"{specs.fork_rebound} / 17", "clicks", "0=Fast, 9=Neutral, 17=Slow"),
        ("Fork Air Pressure & Tokens", f"{specs.fork_initial_psi:.1f} PSI, {specs.fork_air_tokens} tok", "psi/tok", "DebonAir+ Pneumatic Spring"),
        ("Rear Shock Damper Model", "Super Deluxe RC2T", "model", "RockShox Ultimate + HBO"),
        ("Rear Shock HSC Clicks", f"{specs.shock_hsc} / 4", "clicks", "0=Open, 2=Neutral, 4=Firm"),
        ("Rear Shock LSC Clicks", f"{specs.shock_lsc} / 14", "clicks", "0=Open, 7=Neutral, 14=Firm"),
        ("Rear Shock Rebound Clicks", f"{specs.shock_rebound} / 14", "clicks", "0=Fast, 7=Neutral, 14=Slow"),
        ("Hydraulic Bottom-Out (HBO)", f"{specs.shock_hbo} / 4", "clicks", "Active in last 13mm stroke"),
        ("Shock Lockout State", "Firm Platform" if specs.shock_lockout else "Open / Active", "state", "Threshold 2-Pos Lever"),
    ]

    for label, val, unit, notes in metrics:
        print(f"{label:<36} | {val:>16} | {unit:<8} | {notes:<30}")
    print(sep_line + "\n")
