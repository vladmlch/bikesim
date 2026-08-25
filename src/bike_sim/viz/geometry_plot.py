"""
Bicycle Linkage Geometry & Frame Layout Plotter.
"""

from pathlib import Path
from typing import Any, Dict, Tuple
import numpy as np

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.geometry.hardpoints import compute_ground_z, compute_trail
from bike_sim.kinematics.solver import HorstLinkageSolver
from bike_sim.viz.tables import get_all_points
from bike_sim.viz.theme import setup_matplotlib, LINKAGE_COLORS


def _draw_ground_and_wheels(
    ax: Any,
    plt: Any,
    pts: Dict[str, np.ndarray],
    specs: BikeSpecs,
    c_ground: str,
    c_wheel: str,
    ground_z: float,
) -> None:
    """Draws ground horizontal line and wheel/rim circles."""
    ax.axhline(
        ground_z,
        color=c_ground,
        linestyle="--",
        linewidth=1.8,
        label=f"Ground Line ($Z = {ground_z:.1f}$ mm)",
        zorder=1,
    )
    rear_wheel = plt.Circle(
        (pts["P1"][0], pts["P1"][2]),
        specs.rear_wheel_radius,
        color=c_wheel,
        fill=True,
        alpha=0.08,
        linewidth=1.5,
        linestyle="--",
        zorder=2,
    )
    front_wheel = plt.Circle(
        (pts["P_FA"][0], pts["P_FA"][2]),
        specs.front_wheel_radius,
        color=c_wheel,
        fill=True,
        alpha=0.08,
        linewidth=1.5,
        linestyle="--",
        zorder=2,
    )
    ax.add_patch(rear_wheel)
    ax.add_patch(front_wheel)

    rear_rim = plt.Circle(
        (pts["P1"][0], pts["P1"][2]),
        specs.rear_wheel_radius - 25.0,
        color="#7F8C8D",
        fill=False,
        linewidth=1.0,
        linestyle=":",
        zorder=2,
    )
    front_rim = plt.Circle(
        (pts["P_FA"][0], pts["P_FA"][2]),
        specs.front_wheel_radius - 25.0,
        color="#7F8C8D",
        fill=False,
        linewidth=1.0,
        linestyle=":",
        zorder=2,
    )
    ax.add_patch(rear_rim)
    ax.add_patch(front_rim)


def _draw_front_triangle(
    ax: Any,
    plt: Any,
    patches: Any,
    pts: Dict[str, np.ndarray],
    c_frame: str,
) -> None:
    """Draws front triangle tubes, gusset, shock tab, angle arcs, and tube labels."""
    # Down Tube & Head Tube
    ax.plot([pts["P_HT_bot"][0], pts["BB"][0]], [pts["P_HT_bot"][2], pts["BB"][2]], color="#1C2833", linewidth=6.0, solid_capstyle="round", label="Down Tube (Tube 2, Frame)", zorder=3)
    ax.plot([pts["P11"][0], pts["P_HT_bot"][0]], [pts["P11"][2], pts["P_HT_bot"][2]], color="#1A252F", linewidth=7.0, solid_capstyle="round", zorder=4)

    # Top Tube & Seat Tube
    ax.plot([pts["P11"][0], pts["P8"][0]], [pts["P11"][2], pts["P8"][2]], color=c_frame, linewidth=4.5, solid_capstyle="round", zorder=3)
    ax.plot([pts["P8"][0], pts["P10"][0]], [pts["P8"][2], pts["P10"][2]], color=c_frame, linewidth=4.5, solid_capstyle="round", label="Top Tube (Tube 1, Frame)", zorder=3)
    ax.plot([pts["BB"][0], pts["P10"][0], pts["P9"][0]], [pts["BB"][2], pts["P10"][2], pts["P9"][2]], color=c_frame, linewidth=4.5, solid_capstyle="round", label="Seat Tube (STA = 77.2°)", zorder=3)

    # Gusset & Shock Tab Polygons
    v_t1 = pts["P8"] - pts["P10"]
    p_gusset = pts["P10"] + v_t1 * 0.22
    gusset_poly = plt.Polygon([[pts["P9"][0], pts["P9"][2]], [pts["P10"][0], pts["P10"][2]], [p_gusset[0], p_gusset[2]]], closed=True, facecolor="#2C3E50", edgecolor="#1A252F", alpha=0.35, linewidth=1.8, label="Seat Cluster Gusset", zorder=3)
    ax.add_patch(gusset_poly)

    p_tab = pts["P10"] + v_t1 * 0.52
    tab_poly = plt.Polygon([[p_tab[0] - 18, p_tab[2] - 7], [p_tab[0] + 18, p_tab[2] + 7], [pts["P7"][0], pts["P7"][2]]], closed=True, facecolor="#7F8C8D", edgecolor="#34495E", alpha=0.45, linewidth=1.5, zorder=4)
    ax.add_patch(tab_poly)
    ax.plot([p_tab[0], pts["P7"][0]], [p_tab[2], pts["P7"][2]], color="#34495E", linewidth=2.0, zorder=5)

    # Angle Arc
    arc_rad = 55.0
    theta_t1 = float(np.degrees(np.arctan2(v_t1[2], v_t1[0])))
    v_st_up = pts["P9"] - pts["P10"]
    theta_st = float(np.degrees(np.arctan2(v_st_up[2], v_st_up[0])))
    angle_arc = patches.Arc((pts["P10"][0], pts["P10"][2]), 2 * arc_rad, 2 * arc_rad, angle=0.0, theta1=theta_t1, theta2=theta_st, color="#27AE60", linewidth=2.2, zorder=8)
    ax.add_patch(angle_arc)
    mid_th = float(np.radians((theta_t1 + theta_st) / 2.0))
    ax.text(
        pts["P10"][0] + (arc_rad + 25) * np.cos(mid_th),
        pts["P10"][2] + (arc_rad + 25) * np.sin(mid_th),
        f"Angle_1\n{theta_st - theta_t1:.1f}°",
        ha="center",
        va="center",
        fontsize=8.5,
        fontweight="bold",
        color="#1E8449",
        bbox=dict(boxstyle="round,pad=0.2", facecolor="#E8F8F5", edgecolor="#27AE60", alpha=0.9),
        zorder=12,
    )

    # Tube Angle Labels
    mid_t1 = pts["P10"] + v_t1 * 0.58
    ax.text(mid_t1[0] - 10, mid_t1[2] + 16, f"Tube 1 ({theta_t1:.1f}°)", ha="center", va="bottom", fontsize=9.5, fontweight="bold", color="#2980B9", rotation=theta_t1, zorder=12)
    mid_t2 = pts["BB"] + (pts["P_HT_bot"] - pts["BB"]) * 0.55
    theta_t2 = float(np.degrees(np.arctan2(pts["P_HT_bot"][2] - pts["BB"][2], pts["P_HT_bot"][0] - pts["BB"][0])))
    ax.text(mid_t2[0] + 15, mid_t2[2] - 25, f"Tube 2 ({theta_t2:.1f}°)", ha="center", va="top", fontsize=9.5, fontweight="bold", color="#2980B9", rotation=theta_t2, zorder=12)


def _draw_fork_and_steering(
    ax: Any,
    pts: Dict[str, np.ndarray],
    specs: BikeSpecs,
    trail_info: Dict[str, float],
    ground_z: float,
    c_fork: str,
) -> None:
    """Draws front fork and steering axis line."""
    ax.plot([pts["P_HT_bot"][0], pts["P_FA"][0]], [pts["P_HT_bot"][2], pts["P_FA"][2]], color=c_fork, linewidth=3.5, solid_capstyle="round", label="Front Fork (180 mm Travel)", zorder=3)
    x_g = trail_info["steer_ground_intersection_x"]
    ax.plot(
        [pts["P11"][0], x_g],
        [pts["P11"][2], ground_z],
        color="#E74C3C",
        linestyle=":",
        linewidth=1.2,
        label=f"Steering Axis (HA = {specs.head_angle_deg:.0f}°, Trail = {trail_info['ground_trail']:.1f} mm)",
        zorder=2,
    )


def _draw_rear_suspension(
    ax: Any,
    plt: Any,
    pts: Dict[str, np.ndarray],
    solver: HorstLinkageSolver,
    c_cs: str,
    c_ss: str,
    c_rocker: str,
    c_yoke: str,
    c_shock: str,
) -> None:
    """Draws 4-bar rear suspension linkages, dropout plate, rocker link, yoke, and shock."""
    # Chainstay
    ax.plot([pts["P0"][0], pts["P2"][0]], [pts["P0"][2], pts["P2"][2]], color=c_cs, linewidth=4.5, solid_capstyle="round", label=f"Chainstay ($P_0 \\to P_2$, $L={solver.l_cs:.1f}$ mm)", zorder=5)

    # Dropout triangular plate & edges
    dropout_poly = plt.Polygon([[pts["P12"][0], pts["P12"][2]], [pts["P1"][0], pts["P1"][2]], [pts["P2"][0], pts["P2"][2]]], closed=True, facecolor=c_ss, edgecolor="#D35400", alpha=0.45, linewidth=2.5, label="Rear Dropout Plate (Solid $\\Delta_{12-1-2}$)", zorder=5)
    ax.add_patch(dropout_poly)
    ax.plot([pts["P12"][0], pts["P1"][0], pts["P2"][0], pts["P12"][0]], [pts["P12"][2], pts["P1"][2], pts["P2"][2], pts["P12"][2]], color="#D35400", linewidth=2.2, solid_capstyle="round", zorder=6)

    # Seatstay
    ax.plot([pts["P3"][0], pts["P12"][0]], [pts["P3"][2], pts["P12"][2]], color=c_ss, linewidth=4.5, solid_capstyle="round", label=f"Seatstay ($P_3 \\to P_{{12}}$, $L={solver.l_ss_tube:.1f}$ mm)", zorder=5)

    # Rocker link & polygon
    rocker_poly = plt.Polygon([[pts["P5"][0], pts["P5"][2]], [pts["P3"][0], pts["P3"][2]], [pts["P4"][0], pts["P4"][2]]], closed=True, facecolor=c_rocker, edgecolor="#962D22", alpha=0.35, linewidth=2.5, label=f"Rocker Link ($P_5-P_3-P_4$, $R_{{53}}={solver.r_53:.1f}$ mm)", zorder=6)
    ax.add_patch(rocker_poly)
    ax.plot([pts["P5"][0], pts["P3"][0], pts["P4"][0], pts["P5"][0]], [pts["P5"][2], pts["P3"][2], pts["P4"][2], pts["P5"][2]], color=c_rocker, linewidth=2.8, solid_capstyle="round", zorder=7)

    # Shock Yoke & Shock Absorber
    ax.plot([pts["P4"][0], pts["P6"][0]], [pts["P4"][2], pts["P6"][2]], color=c_yoke, linewidth=3.8, solid_capstyle="round", label=f"Shock Yoke ($P_4 \\to P_6$, $L={solver.l_yoke:.1f}$ mm)", zorder=6)
    ax.plot([pts["P6"][0], pts["P7"][0]], [pts["P6"][2], pts["P7"][2]], color=c_shock, linewidth=5.5, solid_capstyle="round", label=f"Shock Absorber ($P_6 \\to P_7$, Eye-to-Eye = {solver.l_shock_0:.1f} mm)", zorder=6)


def _draw_pivot_markers_and_labels(ax: Any, pts: Dict[str, np.ndarray]) -> None:
    """Draws hardpoint circles and callout annotation labels."""
    point_callouts: Dict[str, Tuple[Tuple[float, float], str, str]] = {
        "P0": ((30, -15), "left", "center"),
        "P1": ((-75, -28), "right", "top"),
        "P2": ((-70, 18), "right", "bottom"),
        "P3": ((-85, 30), "right", "bottom"),
        "P4": ((-80, -32), "right", "top"),
        "P5": ((35, -22), "left", "top"),
        "P6": ((32, 22), "left", "bottom"),
        "P7": ((32, 18), "left", "bottom"),
        "P8": ((25, 20), "left", "bottom"),
        "P9": ((-65, 10), "right", "bottom"),
        "P10": ((-75, -22), "right", "top"),
        "P11": ((30, 20), "left", "bottom"),
        "P12": ((-75, 20), "right", "bottom"),
        "P_HT_bot": ((30, -20), "left", "top"),
        "P_FA": ((30, -22), "left", "top"),
        "BB": ((25, -35), "left", "top"),
    }

    for name, p_mm in pts.items():
        is_fixed = name in ["P0", "P5", "P7", "P8", "P9", "P10", "P11", "P_HT_bot", "BB"]
        marker_color = "#34495E" if is_fixed else "#E74C3C"
        ax.scatter(p_mm[0], p_mm[2], s=85 if name != "BB" else 115, color=marker_color, edgecolors="#FFFFFF", linewidth=1.8, zorder=10)

        offset, ha, va = point_callouts.get(name, ((15, 15), "left", "bottom"))
        ax.annotate(
            f"{name}\n({p_mm[0]:.1f}, {p_mm[2]:.1f})",
            xy=(p_mm[0], p_mm[2]),
            xytext=(p_mm[0] + offset[0], p_mm[2] + offset[1]),
            ha=ha,
            va=va,
            fontsize=8.5,
            fontweight="bold",
            color="#1C2833",
            bbox=dict(boxstyle="round,pad=0.25", facecolor="white", alpha=0.92, edgecolor="#BDC3C7", linewidth=0.8),
            arrowprops=dict(arrowstyle="->", color="#7F8C8D", lw=0.8, shrinkA=3, shrinkB=4),
            zorder=11,
        )

    # Origin BB
    ax.scatter(0.0, 0.0, s=130, color="#F39C12", edgecolors="black", linewidth=2.0, zorder=12)


def _draw_geometry_dimensions(ax: Any, pts: Dict[str, np.ndarray], specs: BikeSpecs, ground_z: float) -> None:
    """Draws reach, stack, and wheelbase dimensions."""
    # Reach
    ax.plot([0, specs.reach], [specs.stack + 35, specs.stack + 35], color="#7F8C8D", linestyle="-.", linewidth=1.2)
    ax.plot([specs.reach, specs.reach], [specs.stack, specs.stack + 35], color="#7F8C8D", linestyle=":", linewidth=1.0)
    ax.plot([0, 0], [specs.stack, specs.stack + 35], color="#7F8C8D", linestyle=":", linewidth=1.0)
    ax.text(specs.reach / 2.0, specs.stack + 45, f"Reach = {specs.reach:.0f} mm", ha="center", fontsize=9, color="#34495E", fontweight="bold")

    # Stack
    ax.plot([-30, -30], [0, specs.stack], color="#7F8C8D", linestyle="-.", linewidth=1.2)
    ax.plot([-30, 0], [0, 0], color="#7F8C8D", linestyle=":", linewidth=1.0)
    ax.plot([-30, 0], [specs.stack, specs.stack], color="#7F8C8D", linestyle=":", linewidth=1.0)
    ax.text(-45, specs.stack / 2.0, f"Stack = {specs.stack:.0f} mm", va="center", ha="right", rotation=90, fontsize=9, color="#34495E", fontweight="bold")

    # Wheelbase
    ax.annotate("", xy=(pts["P1"][0], ground_z - 35), xytext=(pts["P_FA"][0], ground_z - 35), arrowprops=dict(arrowstyle="<->", color="#2C3E50", lw=1.5))
    ax.text((pts["P1"][0] + pts["P_FA"][0]) / 2.0, ground_z - 65, f"Wheelbase = {specs.wheelbase:.0f} mm", ha="center", fontsize=9, fontweight="bold", color="#2C3E50")


def plot_linkage_geometry(
    specs: BikeSpecs,
    solver: HorstLinkageSolver,
    output_path: str = "output/plots/linkage_geometry.png",
) -> Path:
    """
    Generates a publication-quality diagram of the bicycle frame geometry and Horst-Link
    suspension layout with clearly labeled pivot points and dimensions in the MuJoCo coordinate frame.
    """
    plt, patches = setup_matplotlib()
    pts = get_all_points(specs, solver, wheel_travel=0.0)
    trail_info = compute_trail(specs)

    fig, ax = plt.subplots(figsize=(16, 9.2), dpi=200)
    ground_z = compute_ground_z(specs)

    # 1. Ground and Wheels
    _draw_ground_and_wheels(ax, plt, pts, specs, LINKAGE_COLORS["ground"], LINKAGE_COLORS["wheel"], ground_z)

    # 2. Front Triangle
    _draw_front_triangle(ax, plt, patches, pts, LINKAGE_COLORS["frame"])

    # 3. Fork & Steering
    _draw_fork_and_steering(ax, pts, specs, trail_info, ground_z, LINKAGE_COLORS["fork"])

    # 4. Rear Suspension Linkages
    _draw_rear_suspension(
        ax,
        plt,
        pts,
        solver,
        LINKAGE_COLORS["chainstay"],
        LINKAGE_COLORS["seatstay"],
        LINKAGE_COLORS["rocker"],
        LINKAGE_COLORS["yoke"],
        LINKAGE_COLORS["shock"],
    )

    # 5. Pivot Markers & Labels
    _draw_pivot_markers_and_labels(ax, pts)

    # 6. Dimensions (Reach, Stack, Wheelbase)
    _draw_geometry_dimensions(ax, pts, specs, ground_z)

    # 7. Axes Setup & Save
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X (mm) [Forward ->]", fontsize=11, fontweight="bold")
    ax.set_ylabel("Z (mm) [Upward ^]", fontsize=11, fontweight="bold")
    ax.set_title(
        "Bicycle Geometry & Horst-Link Suspension Layout\n"
        "(Enduro 29\" | 180 mm Travel | Reach 480 mm | Stack 646 mm | HA 64°)",
        fontsize=13,
        fontweight="bold",
        pad=15,
    )
    ax.grid(True, linestyle="--", alpha=0.5, color=LINKAGE_COLORS["grid"])
    ax.set_xlim(-620, 1020)
    ax.set_ylim(ground_z - 90, 780)
    ax.legend(loc="upper left", framealpha=0.94, fontsize=8.5, edgecolor="#BDC3C7")

    plt.tight_layout()
    p = Path(output_path)
    p.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(p, dpi=200)
    plt.close(fig)
    print(f"[SUCCESS] Saved linkage geometry plot to {p.resolve()}")
    return p

