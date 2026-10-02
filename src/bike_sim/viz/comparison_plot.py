"""
Suspension Compressed Comparison Kinematic Overlay Plotter.
"""

from pathlib import Path
from typing import Any, Dict, Tuple
import numpy as np

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.geometry.hardpoints import compute_ground_z
from bike_sim.kinematics.solver import HorstLinkageSolver
from bike_sim.viz.tables import get_all_points
from bike_sim.viz.theme import setup_matplotlib, COMPARISON_COLORS


def _draw_fixed_frame_and_wheels(
    ax: Any,
    plt: Any,
    p0_pts: Dict[str, np.ndarray],
    c_frame: str,
    fw_radius: float,
    ground_z: float,
) -> None:
    """Draws ground line, rigid front triangle tubes, gusset, shock tab, and front wheel."""
    ax.axhline(ground_z, color="#BDC3C7", linestyle="--", linewidth=1.5, zorder=1)

    # Frame tubes
    ax.plot([p0_pts["P_HT_bot"][0], p0_pts["BB"][0]], [p0_pts["P_HT_bot"][2], p0_pts["BB"][2]], color="#1C2833", linewidth=5.5, zorder=2)
    ax.plot([p0_pts["P11"][0], p0_pts["P_HT_bot"][0]], [p0_pts["P11"][2], p0_pts["P_HT_bot"][2]], color="#1A252F", linewidth=6.5, zorder=2)
    ax.plot([p0_pts["P11"][0], p0_pts["P8"][0]], [p0_pts["P11"][2], p0_pts["P8"][2]], color=c_frame, linewidth=4.0, zorder=2)
    ax.plot([p0_pts["P8"][0], p0_pts["P10"][0]], [p0_pts["P8"][2], p0_pts["P10"][2]], color=c_frame, linewidth=4.0, zorder=2)
    ax.plot([p0_pts["BB"][0], p0_pts["P10"][0], p0_pts["P9"][0]], [p0_pts["BB"][2], p0_pts["P10"][2], p0_pts["P9"][2]], color=c_frame, linewidth=4.0, zorder=2)

    # Gusset & shock tab
    v_t1_p0 = p0_pts["P8"] - p0_pts["P10"]
    p_gusset_p0 = p0_pts["P10"] + v_t1_p0 * 0.22
    gusset_poly = plt.Polygon([[p0_pts["P9"][0], p0_pts["P9"][2]], [p0_pts["P10"][0], p0_pts["P10"][2]], [p_gusset_p0[0], p_gusset_p0[2]]], closed=True, facecolor="#2C3E50", edgecolor="#1A252F", alpha=0.35, linewidth=1.5, zorder=2)
    ax.add_patch(gusset_poly)

    p_tab_p0 = p0_pts["P10"] + v_t1_p0 * 0.52
    ax.plot([p_tab_p0[0], p0_pts["P7"][0]], [p_tab_p0[2], p0_pts["P7"][2]], color="#34495E", linewidth=2.0, zorder=2)
    ax.plot([p0_pts["P_HT_bot"][0], p0_pts["P_FA"][0]], [p0_pts["P_HT_bot"][2], p0_pts["P_FA"][2]], color="#7F8C8D", linewidth=3.0, zorder=2)

    front_wheel = plt.Circle((p0_pts["P_FA"][0], p0_pts["P_FA"][2]), fw_radius, color="#BDC3C7", fill=False, linewidth=1.2, linestyle="--", zorder=2)
    ax.add_patch(front_wheel)


def _draw_trajectory_arcs(ax: Any, traj: Dict[str, np.ndarray], c_arc: str) -> None:
    """Draws motion trajectory arcs for rear axle and suspension pivots."""
    ax.plot(traj["P1"][:, 0], traj["P1"][:, 2], color="#E67E22", linestyle=":", linewidth=2.2, label="Rear Axle Path ($P_1$)", zorder=3)
    for p_name in ["P2", "P3", "P4", "P6", "P12"]:
        ax.plot(traj[p_name][:, 0], traj[p_name][:, 2], color=c_arc, linestyle=":", linewidth=1.5, zorder=3)


def _draw_linkage_state(
    ax: Any,
    plt: Any,
    pts: Dict[str, np.ndarray],
    rw_radius: float,
    *,
    label_suffix: str,
    shock_label: str,
    linestyle: str,
    zorder_base: int,
    wheel_alpha: float,
    chainstay: Tuple[str, float],
    seatstay: Tuple[str, float],
    dropout_face: str,
    dropout_edge: str,
    rocker: str,
    yoke: Tuple[str, float],
    shock: Tuple[str, float],
    wheel_color: str,
) -> None:
    """
    Draws one suspension linkage state (members + rear wheel).

    The uncompressed and compressed overlays are the same seven primitives in the
    same order; only palette, line weights, dash style, and z-layering differ.
    """
    def xz(*names: str) -> Tuple[list, list]:
        return [pts[n][0] for n in names], [pts[n][2] for n in names]

    cs_color, cs_lw = chainstay
    ss_color, ss_lw = seatstay
    yoke_color, yoke_lw = yoke
    shock_color, shock_lw = shock

    ax.plot(*xz("P0", "P2"), color=cs_color, linewidth=cs_lw, linestyle=linestyle, solid_capstyle="round", label=f"Chainstay ({label_suffix})", zorder=zorder_base)
    ax.plot(*xz("P3", "P12"), color=ss_color, linewidth=ss_lw, linestyle=linestyle, solid_capstyle="round", label=f"Seatstay ({label_suffix})", zorder=zorder_base)

    dropout_poly = plt.Polygon(
        [[pts["P12"][0], pts["P12"][2]], [pts["P1"][0], pts["P1"][2]], [pts["P2"][0], pts["P2"][2]]],
        closed=True, facecolor=dropout_face, edgecolor=dropout_edge, alpha=0.35, linewidth=1.8, linestyle=linestyle, zorder=zorder_base,
    )
    ax.add_patch(dropout_poly)
    ax.plot(*xz("P12", "P1", "P2", "P12"), color=dropout_edge, linewidth=1.8, linestyle=linestyle, zorder=zorder_base + 1)
    ax.plot(*xz("P5", "P3", "P4", "P5"), color=rocker, linewidth=2.5, linestyle=linestyle, label=f"Rocker Link ({label_suffix})", zorder=zorder_base + 1)
    ax.plot(*xz("P4", "P6"), color=yoke_color, linewidth=yoke_lw, linestyle=linestyle, label=f"Yoke ({label_suffix})", zorder=zorder_base + 1)
    ax.plot(*xz("P6", "P7"), color=shock_color, linewidth=shock_lw, linestyle=linestyle, label=shock_label, zorder=zorder_base + 1)

    ax.add_patch(plt.Circle(
        (pts["P1"][0], pts["P1"][2]), rw_radius,
        color=wheel_color, fill=False, linewidth=1.2, linestyle=linestyle, alpha=wheel_alpha, zorder=2,
    ))


def _draw_uncompressed_state(ax: Any, plt: Any, p0_pts: Dict[str, np.ndarray], rw_radius: float, c_uncomp: str) -> None:
    """Draws uncompressed (0 mm travel) suspension linkage members and rear wheel."""
    _draw_linkage_state(
        ax, plt, p0_pts, rw_radius,
        label_suffix="0 mm",
        shock_label="Shock (0 mm, trunnion L=205 mm)",
        linestyle="-", zorder_base=4, wheel_alpha=0.5,
        chainstay=(c_uncomp, 4.0), seatstay=("#3498DB", 3.5),
        dropout_face="#3498DB", dropout_edge="#2980B9", rocker="#2980B9",
        yoke=("#8E44AD", 3.5), shock=("#16A085", 5.0), wheel_color="#2980B9",
    )


def _draw_compressed_state(ax: Any, plt: Any, p180_pts: Dict[str, np.ndarray], rw_radius: float, c_comp: str) -> None:
    """Draws fully compressed (180 mm travel) suspension linkage members and rear wheel."""
    _draw_linkage_state(
        ax, plt, p180_pts, rw_radius,
        label_suffix="180 mm",
        shock_label="Shock (180 mm, L=165 mm)",
        linestyle="--", zorder_base=6, wheel_alpha=0.6,
        chainstay=(c_comp, 3.5), seatstay=("#E67E22", 3.0),
        dropout_face="#E67E22", dropout_edge="#D35400", rocker=c_comp,
        yoke=("#9B59B6", 3.0), shock=("#1ABC9C", 4.5), wheel_color=c_comp,
    )


def _draw_motion_annotations(ax: Any, p0_pts: Dict[str, np.ndarray], p180_pts: Dict[str, np.ndarray]) -> None:
    """Draws pivot comparison markers, fixed points, and travel/stroke deltas."""
    for name in ["P1", "P2", "P3", "P4", "P6", "P12"]:
        ax.scatter(p0_pts[name][0], p0_pts[name][2], s=55, color="#2980B9", edgecolors="white", linewidth=1.2, zorder=8)
        ax.scatter(p180_pts[name][0], p180_pts[name][2], s=55, color="#E74C3C", marker="s", edgecolors="white", linewidth=1.2, zorder=8)

    fixed_label_positions = {
        "P0": ((25, -15), "left", "center"),
        "P5": ((30, -18), "left", "top"),
        "P7": ((25, 18), "left", "bottom"),
        "BB": ((20, -25), "left", "top"),
    }
    for name in ["P0", "P5", "P7", "BB"]:
        pf = p0_pts[name]
        ax.scatter(pf[0], pf[2], s=75, color="#2C3E50", edgecolors="white", linewidth=1.5, zorder=9)
        offset, ha, va = fixed_label_positions[name]
        ax.annotate(
            f"{name} (fixed)",
            xy=(pf[0], pf[2]),
            xytext=(pf[0] + offset[0], pf[2] + offset[1]),
            ha=ha,
            va=va,
            fontsize=8.5,
            fontweight="bold",
            color="#2C3E50",
            bbox=dict(boxstyle="round,pad=0.2", facecolor="white", alpha=0.88, edgecolor="#BDC3C7"),
        )

    # Wheel Travel Delta
    ax.annotate("", xy=(p180_pts["P1"][0], p180_pts["P1"][2]), xytext=(p0_pts["P1"][0], p0_pts["P1"][2]), arrowprops=dict(arrowstyle="->", color="#C0392B", lw=2.2), zorder=10)
    ax.text(
        p0_pts["P1"][0] - 80,
        (p0_pts["P1"][2] + p180_pts["P1"][2]) / 2.0,
        "ΔZ = +180.0 mm\n(Wheel Travel)",
        fontsize=9,
        fontweight="bold",
        color="#C0392B",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.92, edgecolor="#C0392B"),
    )

    # Shock Stroke Delta
    d_stroke = np.linalg.norm(p0_pts["P7"] - p0_pts["P6"]) - np.linalg.norm(p180_pts["P7"] - p180_pts["P6"])
    ax.text(
        p180_pts["P6"][0] - 120,
        p180_pts["P6"][2] + 25,
        f"Shock Stroke:\nΔS = {d_stroke:.2f} mm",
        fontsize=9,
        fontweight="bold",
        color="#16A085",
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.92, edgecolor="#16A085"),
    )


def plot_suspension_compressed_comparison(
    specs: BikeSpecs,
    solver: HorstLinkageSolver,
    output_path: str = "output/plots/suspension_compressed_comparison.png",
) -> Path:
    """
    Generates a publication-quality visual kinematic overlay diagram comparing the uncompressed
    state (0 mm travel, solid lines) against the fully compressed state (180 mm travel, dashed lines),
    with motion trajectory arcs and delta annotations.
    """
    plt, _ = setup_matplotlib()
    p0_pts = get_all_points(specs, solver, wheel_travel=0.0)
    p180_pts = get_all_points(specs, solver, wheel_travel=180.0)
    traj = solver.solve_trajectory(n_points=101, max_travel=180.0)

    fig, ax = plt.subplots(figsize=(16, 9.2), dpi=200)

    ground_z = compute_ground_z(specs)

    # 1. Rigid frame and front wheel
    _draw_fixed_frame_and_wheels(ax, plt, p0_pts, COMPARISON_COLORS["frame"], specs.front_wheel_radius, ground_z)

    # 2. Motion Trajectory Arcs
    _draw_trajectory_arcs(ax, traj, COMPARISON_COLORS["arc"])

    # 3. Uncompressed state (0 mm)
    _draw_uncompressed_state(ax, plt, p0_pts, specs.rear_wheel_radius, COMPARISON_COLORS["uncompressed"])

    # 4. Compressed state (180 mm)
    _draw_compressed_state(ax, plt, p180_pts, specs.rear_wheel_radius, COMPARISON_COLORS["compressed"])

    # 5. Delta callouts and markers
    _draw_motion_annotations(ax, p0_pts, p180_pts)

    # 6. Axes layout and save
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X (mm) [Forward ->]", fontsize=11, fontweight="bold")
    ax.set_ylabel("Z (mm) [Upward ^]", fontsize=11, fontweight="bold")
    ax.set_title(
        "Suspension Kinematics Overlay: 0 mm (Solid Blue) vs 180 mm (Dashed Red)\n"
        "(Horst-Link 4-Bar + Yoke Mechanism Deformation)",
        fontsize=13,
        fontweight="bold",
        pad=15,
    )
    ax.grid(True, linestyle="--", alpha=0.5, color="#D5D8DC")
    ax.set_xlim(-620, 1020)
    ax.set_ylim(ground_z - 60, 780)
    ax.legend(loc="upper left", framealpha=0.94, fontsize=8.0, edgecolor="#BDC3C7", ncol=2)

    plt.tight_layout()
    p = Path(output_path)
    p.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(p, dpi=200)
    plt.close(fig)
    print(f"[SUCCESS] Saved compressed comparison plot to {p.resolve()}")
    return p

