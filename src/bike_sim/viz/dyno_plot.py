"""
Suspension Damper Dyno Curve Plotter for RockShox ZEB Ultimate & Super Deluxe Ultimate.
"""

from pathlib import Path
from typing import Any
import numpy as np

from bike_sim.physics.damper import (
    BikeSuspensionSystem,
    Charger3Damper,
    DamperPreset,
    SuperDeluxeDamper,
)
from bike_sim.viz.theme import setup_matplotlib


def _style_ax(ax: Any, title: str, xlabel: str, ylabel: str) -> None:
    """Applies high-contrast dark theme styling to a subplot."""
    ax.set_facecolor("#22272e")
    ax.set_title(title, color="#e6edf3", fontsize=12, fontweight="bold", pad=10)
    ax.set_xlabel(xlabel, color="#8b949e", fontsize=10)
    ax.set_ylabel(ylabel, color="#8b949e", fontsize=10)
    ax.tick_params(colors="#8b949e", labelsize=9)
    ax.grid(True, linestyle="--", alpha=0.3, color="#484f58")
    for spine in ax.spines.values():
        spine.set_color("#444c56")
    ax.axhline(0, color="#6e7681", linestyle="-", linewidth=0.8, alpha=0.7)
    ax.axvline(0, color="#6e7681", linestyle="-", linewidth=0.8, alpha=0.7)


def _render_fork_lsc_panel(ax: Any, v_sweep: np.ndarray) -> None:
    """Renders Fork LSC sweeps across different click settings."""
    _style_ax(ax, "RockShox ZEB Ultimate: LSC Sweep (HSC=2, Reb=9)", "Shaft Velocity v (m/s)", "Damping Force F (N)")
    colors = ["#38bdf8", "#34d399", "#fbbf24", "#f97316", "#ef4444"]
    clicks = [0, 3, 7, 11, 14]
    labels = ["Click 0 (Fully Open)", "Click 3 (Soft)", "Click 7 (Neutral)", "Click 11 (Firm)", "Click 14 (Max LSC)"]

    for lsc, col, lab in zip(clicks, colors, labels):
        damp = Charger3Damper(hsc_clicks=2, lsc_clicks=lsc, rebound_clicks=9)
        f_arr = np.array([damp.compute_damping_force(v) for v in v_sweep])
        ax.plot(v_sweep, f_arr, label=lab, color=col, linewidth=2.0)

    ax.annotate("Knee v = 0.18 m/s\n(Shim Stack Opens)", xy=(0.18, 140), xytext=(0.45, 450),
                arrowprops=dict(facecolor="#e6edf3", shrink=0.08, width=1.0, headwidth=5),
                color="#e6edf3", fontsize=8.5, backgroundcolor="#1a1e24")
    ax.legend(loc="upper left", facecolor="#161b22", edgecolor="#444c56", labelcolor="#e6edf3", fontsize=8.5)


def _render_fork_hsc_panel(ax: Any, v_sweep: np.ndarray) -> None:
    """Renders Fork HSC sweeps."""
    _style_ax(ax, "RockShox ZEB Ultimate: HSC Sweep (LSC=7, Reb=9)", "Shaft Velocity v (m/s)", "Damping Force F (N)")
    colors = ["#60a5fa", "#3b82f6", "#a855f7", "#ec4899", "#f43f5e"]
    labels = ["HSC 0 (Open / -2)", "HSC 1 (-1)", "HSC 2 (Neutral / 0)", "HSC 3 (+1)", "HSC 4 (Firm / +2)"]

    for hsc, col, lab in zip(range(5), colors, labels):
        damp = Charger3Damper(hsc_clicks=hsc, lsc_clicks=7, rebound_clicks=9)
        f_arr = np.array([damp.compute_damping_force(v) for v in v_sweep])
        ax.plot(v_sweep, f_arr, label=lab, color=col, linewidth=2.0)

    ax.annotate("High-Speed Blow-Off Slope\n(Shim Stack Stiffness)", xy=(1.4, 520), xytext=(0.7, 750),
                arrowprops=dict(facecolor="#e6edf3", shrink=0.08, width=1.0, headwidth=5),
                color="#e6edf3", fontsize=8.5, backgroundcolor="#1a1e24")
    ax.legend(loc="upper left", facecolor="#161b22", edgecolor="#444c56", labelcolor="#e6edf3", fontsize=8.5)


def _render_fork_rebound_panel(ax: Any, v_sweep: np.ndarray) -> None:
    """Renders Fork Rebound sweeps."""
    _style_ax(ax, "RockShox ZEB Ultimate: Rebound Sweep (HSC=2, LSC=7)", "Shaft Velocity v (m/s)", "Damping Force F (N)")
    colors = ["#38bdf8", "#06b6d4", "#10b981", "#eab308", "#ef4444"]
    clicks = [0, 5, 9, 13, 17]
    labels = ["Reb 0 (Fast / Open)", "Reb 5 (Fast-Mid)", "Reb 9 (Neutral)", "Reb 13 (Slow-Mid)", "Reb 17 (Slow / Closed)"]

    for reb, col, lab in zip(clicks, colors, labels):
        damp = Charger3Damper(hsc_clicks=2, lsc_clicks=7, rebound_clicks=reb)
        f_arr = np.array([damp.compute_damping_force(v) for v in v_sweep])
        ax.plot(v_sweep, f_arr, label=lab, color=col, linewidth=2.0)

    ax.annotate("Rebound Extension Force", xy=(-1.2, -650), xytext=(-1.8, -250),
                arrowprops=dict(facecolor="#e6edf3", shrink=0.08, width=1.0, headwidth=5),
                color="#e6edf3", fontsize=8.5, backgroundcolor="#1a1e24")
    ax.legend(loc="lower right", facecolor="#161b22", edgecolor="#444c56", labelcolor="#e6edf3", fontsize=8.5)


def _render_shock_compression_panel(ax: Any, v_sweep: np.ndarray) -> None:
    """Renders Rear Shock damping curves and pedal platform threshold."""
    _style_ax(ax, "Super Deluxe Ultimate: Damping & Threshold Lockout", "Shaft Velocity v (m/s)", "Damping Force F (N)")

    shk_base = SuperDeluxeDamper(hsc_clicks=2, lsc_clicks=7, rebound_clicks=7, lockout_firm=False)
    ax.plot(v_sweep, [shk_base.compute_damping_force(v) for v in v_sweep], label="Open: Base Tune (HSC=2, LSC=7)", color="#38bdf8", linewidth=2.2)

    shk_firm = SuperDeluxeDamper(hsc_clicks=4, lsc_clicks=14, rebound_clicks=12, lockout_firm=False)
    ax.plot(v_sweep, [shk_firm.compute_damping_force(v) for v in v_sweep], label="Open: Park/Firm (HSC=4, LSC=14)", color="#f97316", linewidth=2.0)

    shk_plush = SuperDeluxeDamper(hsc_clicks=0, lsc_clicks=3, rebound_clicks=4, lockout_firm=False)
    ax.plot(v_sweep, [shk_plush.compute_damping_force(v) for v in v_sweep], label="Open: Plush (HSC=0, LSC=3)", color="#34d399", linewidth=2.0)

    shk_lock = SuperDeluxeDamper(hsc_clicks=2, lsc_clicks=7, rebound_clicks=7, lockout_firm=True)
    ax.plot(v_sweep, [shk_lock.compute_damping_force(v) for v in v_sweep], label="Firm: Threshold Lockout Platform", color="#ef4444", linewidth=2.5, linestyle="--")

    ax.annotate("Pedal Platform Blow-off\n(~480 N Threshold)", xy=(0.04, 500), xytext=(0.35, 1800),
                arrowprops=dict(facecolor="#ef4444", shrink=0.08, width=1.0, headwidth=5),
                color="#ef4444", fontsize=8.5, backgroundcolor="#1a1e24")
    ax.legend(loc="upper left", facecolor="#161b22", edgecolor="#444c56", labelcolor="#e6edf3", fontsize=8.0)


def _render_shock_hbo_panel(ax: Any) -> None:
    """Renders Hydraulic Bottom-Out resistance ramp-up curves."""
    _style_ax(ax, "Super Deluxe HBO: Damping vs Stroke (v = 1.5 m/s)", "Shock Stroke (mm)", "Total Damper Force (N)")
    stroke_arr = np.linspace(0, 65.0, 201)
    colors = ["#38bdf8", "#34d399", "#fbbf24", "#f97316", "#ef4444"]
    for hbo, col in zip(range(5), colors):
        shk_hbo = SuperDeluxeDamper(hsc_clicks=2, lsc_clicks=7, rebound_clicks=7, hbo_clicks=hbo)
        f_stroke = np.array([shk_hbo.compute_damping_force(velocity_mps=1.5, stroke_mm=s) for s in stroke_arr])
        ax.plot(stroke_arr, f_stroke, label=f"HBO Click {hbo} ({'Min' if hbo==0 else 'Max' if hbo==4 else 'Mid'})", color=col, linewidth=2.0)

    ax.axvline(52.0, color="#ec4899", linestyle=":", linewidth=1.5, label="HBO Start (52mm / 80%)")
    ax.annotate("Tapered HBO Needle\nRamps Up Fluid Resistance", xy=(60.0, 2100), xytext=(20.0, 2600),
                arrowprops=dict(facecolor="#e6edf3", shrink=0.08, width=1.0, headwidth=5),
                color="#e6edf3", fontsize=8.5, backgroundcolor="#1a1e24")
    ax.legend(loc="upper left", facecolor="#161b22", edgecolor="#444c56", labelcolor="#e6edf3", fontsize=8.5)


def _render_factory_presets_panel(ax: Any, v_sweep: np.ndarray) -> None:
    """Renders factory damping presets comparing fork and shock balance."""
    _style_ax(ax, "Factory Presets: Fork vs Rear Force Balance", "Shaft Velocity v (m/s)", "Damping Force F (N)")
    preset_colors = {
        DamperPreset.BASE: "#38bdf8",
        DamperPreset.PLUSH: "#34d399",
        DamperPreset.ENDURO: "#f59e0b",
        DamperPreset.PARK: "#ef4444",
    }

    for preset_enum, col in preset_colors.items():
        sys_tune = BikeSuspensionSystem()
        sys_tune.apply_preset(preset_enum)
        f_fork = np.array([sys_tune.fork_damper.compute_damping_force(v) for v in v_sweep])
        f_shock = np.array([sys_tune.shock_damper.compute_damping_force(v) for v in v_sweep])
        ax.plot(v_sweep, f_fork, label=f"Fork: {preset_enum.value.split(' ')[0]}", color=col, linewidth=2.0)
        ax.plot(v_sweep, f_shock, label=f"Rear: {preset_enum.value.split(' ')[0]}", color=col, linewidth=1.8, linestyle="--")

    ax.legend(loc="upper left", facecolor="#161b22", edgecolor="#444c56", labelcolor="#e6edf3", fontsize=7.8, ncol=2)


def plot_damper_dyno_curves(output_file: str = "output/plots/damper_dyno_curves.png") -> str:
    """
    Generates a 6-panel comprehensive dyno analysis figure and saves it to disk.
    """
    plt, _ = setup_matplotlib()
    fig, axs = plt.subplots(2, 3, figsize=(18, 11), dpi=200)
    fig.patch.set_facecolor("#1a1e24")

    v_sweep = np.linspace(-2.0, 2.0, 201)

    _render_fork_lsc_panel(axs[0, 0], v_sweep)
    _render_fork_hsc_panel(axs[0, 1], v_sweep)
    _render_fork_rebound_panel(axs[0, 2], v_sweep)
    _render_shock_compression_panel(axs[1, 0], v_sweep)
    _render_shock_hbo_panel(axs[1, 1])
    _render_factory_presets_panel(axs[1, 2], v_sweep)

    fig.suptitle(
        "RockShox ZEB Ultimate (Charger 3 RC2) & Super Deluxe Ultimate Hydrodynamic Damper Curves",
        color="#e6edf3",
        fontsize=15,
        fontweight="bold",
        y=0.98,
    )
    plt.tight_layout(rect=[0.02, 0.02, 0.98, 0.95])

    out_path = Path(output_file)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, facecolor=fig.get_facecolor(), edgecolor="none")
    plt.close()
    print(f"[SUCCESS] Saved Damper Dyno Curves figure to {out_path.resolve()}")
    return str(out_path.resolve())
