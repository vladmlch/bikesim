"""
Leverage Ratio, Shock Stroke, and Axle Path Kinematic Plotter.
"""

from pathlib import Path
from typing import Any, Dict
import numpy as np

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.kinematics.solver import HorstLinkageSolver
from bike_sim.viz.theme import setup_matplotlib


def _render_leverage_ratio_panel(ax: Any, travel: np.ndarray, lr: np.ndarray, traj: Dict[str, Any]) -> None:
    """Renders instantaneous leverage ratio curve and progressivity annotations."""
    ax.plot(travel, lr, color="#1F77B4", linewidth=2.8, label="Instantaneous Leverage Ratio")
    ax.fill_between(travel, lr, alpha=0.15, color="#1F77B4")

    lr_0 = traj["initial_leverage_ratio"]
    lr_180 = traj["final_leverage_ratio"]
    prog = traj["progressivity_pct"]

    ax.scatter([0.0, 180.0], [lr_0, lr_180], color="#D9534F", s=60, zorder=5)
    ax.annotate(f"0 mm: {lr_0:.3f}", xy=(0.0, lr_0), xytext=(15, 6), textcoords="offset points", fontsize=9, fontweight="bold", color="#2C3E50")
    ax.annotate(f"180 mm: {lr_180:.3f}", xy=(180.0, lr_180), xytext=(-95, 14), textcoords="offset points", fontsize=9, fontweight="bold", color="#2C3E50")

    info_box = f"Initial LR: {lr_0:.3f}\nFinal LR: {lr_180:.3f}\nProgressivity: +{prog:.1f}%"
    ax.text(0.06, 0.12, info_box, transform=ax.transAxes, fontsize=9.5, fontweight="bold", verticalalignment="bottom", bbox=dict(boxstyle="round,pad=0.5", facecolor="white", edgecolor="#1F77B4", alpha=0.92))

    ax.set_xlabel("Rear Wheel Vertical Travel (mm)", fontsize=10.5, fontweight="bold")
    ax.set_ylabel("Leverage Ratio (dTravel / dStroke)", fontsize=10.5, fontweight="bold")
    ax.set_title("1. Leverage Ratio Curve", fontsize=12, fontweight="bold")
    ax.set_xlim(0, 180)
    ax.set_ylim(2.2, 3.4)
    ax.grid(True, linestyle="--", alpha=0.6)


def _render_shock_stroke_panel(ax: Any, travel: np.ndarray, stroke: np.ndarray, traj: Dict[str, Any]) -> None:
    """Renders shock stroke vs wheel travel panel."""
    ax.plot(travel, stroke, color="#E67E22", linewidth=2.8, label="Shock Compression")
    ax.fill_between(travel, stroke, alpha=0.15, color="#E67E22")

    ax.axhline(65.0, color="#C0392B", linestyle="--", linewidth=1.5, label="Target Stroke (65.0 mm)")
    max_stroke = traj["max_stroke"]
    ax.scatter([180.0], [max_stroke], color="#D9534F", s=60, zorder=5)
    ax.annotate(f"Max Stroke: {max_stroke:.2f} mm", xy=(180.0, max_stroke), xytext=(-125, -20), textcoords="offset points", fontsize=9, fontweight="bold", color="#2C3E50")

    ax.set_xlabel("Rear Wheel Vertical Travel (mm)", fontsize=10.5, fontweight="bold")
    ax.set_ylabel("Shock Stroke (mm)", fontsize=10.5, fontweight="bold")
    ax.set_title("2. Shock Stroke vs Wheel Travel", fontsize=12, fontweight="bold")
    ax.set_xlim(0, 180)
    ax.set_ylim(0, 72)
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper left", framealpha=0.92)


def _render_axle_path_panel(ax: Any, axle_x: np.ndarray, axle_z: np.ndarray) -> None:
    """Renders rear axle trajectory in X-Z space."""
    ax.plot(axle_x, axle_z, color="#27AE60", linewidth=2.8, label="Rear Axle Trajectory")
    ax.scatter([axle_x[0]], [axle_z[0]], color="#2980B9", s=70, zorder=5, label="Uncompressed (0 mm)")
    ax.scatter([axle_x[-1]], [axle_z[-1]], color="#C0392B", s=70, zorder=5, label="Compressed (180 mm)")

    ax.annotate(f"Start ({axle_x[0]:.1f}, {axle_z[0]:.1f})", xy=(axle_x[0], axle_z[0]), xytext=(-125, 6), textcoords="offset points", fontsize=8.5, fontweight="bold")
    ax.annotate(f"End ({axle_x[-1]:.1f}, {axle_z[-1]:.1f})", xy=(axle_x[-1], axle_z[-1]), xytext=(-115, 6), textcoords="offset points", fontsize=8.5, fontweight="bold")

    dx_total = axle_x[-1] - axle_x[0]
    dz_total = axle_z[-1] - axle_z[0]
    axle_info = f"Rear Axle Travel:\n  dX = {dx_total:+.2f} mm\n  dZ = {dz_total:+.2f} mm"
    ax.text(0.06, 0.72, axle_info, transform=ax.transAxes, fontsize=9.5, fontweight="bold", bbox=dict(boxstyle="round,pad=0.5", facecolor="white", edgecolor="#27AE60", alpha=0.92))

    ax.set_xlabel("Rear Axle X Coordinate (mm)", fontsize=10.5, fontweight="bold")
    ax.set_ylabel("Rear Axle Z Coordinate (mm)", fontsize=10.5, fontweight="bold")
    ax.set_title("3. Rear Axle Path (X vs Z Trajectory)", fontsize=12, fontweight="bold")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="lower right", framealpha=0.92)


def plot_leverage_ratio(
    specs: BikeSpecs,
    solver: HorstLinkageSolver,
    output_path: str = "output/plots/leverage_ratio.png",
) -> Path:
    """
    Generates a publication-grade 3-panel kinematic analysis figure showing:
    1. Leverage Ratio vs Wheel Travel (0 to 180 mm).
    2. Shock Stroke vs Wheel Travel (0 to 65 mm).
    3. Rear Axle Path trajectory (X_RA vs Z_RA).
    """
    plt, _ = setup_matplotlib()
    traj = solver.solve_trajectory(n_points=101, max_travel=specs.rear_wheel_travel)

    fig, axes = plt.subplots(1, 3, figsize=(18, 5.8), dpi=200)

    _render_leverage_ratio_panel(axes[0], traj["wheel_travel"], traj["leverage_ratio"], traj)
    _render_shock_stroke_panel(axes[1], traj["wheel_travel"], traj["shock_stroke"], traj)
    _render_axle_path_panel(axes[2], traj["axle_path_x"], traj["axle_path_z"])

    plt.suptitle(
        "Horst-Link Rear Suspension Kinematics Analysis (180 mm Travel / Trunnion 205x65 mm Shock)",
        fontsize=14,
        fontweight="bold",
        y=0.98,
    )
    fig.subplots_adjust(top=0.88, bottom=0.12, left=0.06, right=0.96, wspace=0.25)
    p = Path(output_path)
    p.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(p, dpi=200)
    plt.close(fig)
    print(f"[SUCCESS] Saved leverage ratio & kinematics plot to {p.resolve()}")
    return p
