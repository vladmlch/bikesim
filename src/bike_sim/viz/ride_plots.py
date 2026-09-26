"""
Ride-Mode Telemetry Plots and Track Preview.

Three figures from a recorded ride, plus a preview of the road itself:

- ``travel``: fork travel and shock stroke against track X with obstacle markers, so a
  bottom-out can be read against the feature that caused it.
- ``shaft_velocity``: a histogram of shaft velocity for each end, split into compression
  and rebound. This is the only view against which HSC/LSC/rebound clicks can be tuned
  sensibly: the damper is a force-versus-velocity device.
- ``acceleration``: vertical acceleration at the handlebar and saddle, low-passed as the
  summary reports it, with the raw peak annotated so the solver transients are neither
  hidden nor mistaken for what a rider feels.
- ``profile``: the road height along X with markers and, for each pothole, the wheel's
  effective drop -- the figure ``--preview`` renders without simulating.

Style matches ``viz/dyno_plot.py`` (dark theme, Agg backend via ``setup_matplotlib``).
"""

from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple, Union

import numpy as np

from bike_sim.sim.ride.metrics import ACCEL_FILTER_HZ, GRAVITY_MPS2, lowpass
from bike_sim.terrain.obstacles import POTHOLE_TYPES
from bike_sim.terrain.profile import TrackSpec, build_profile
from bike_sim.terrain.wheelpath import FRONT_WHEEL_RADIUS_M, REAR_WHEEL_RADIUS_M, effective_drop_m
from bike_sim.viz.theme import setup_matplotlib

BG = "#1a1e24"
PANEL = "#22272e"
FG = "#e6edf3"
MUTED = "#8b949e"
GRID = "#484f58"
SPINE = "#444c56"
FORK = "#38bdf8"
SHOCK = "#fbbf24"
BAR = "#34d399"
SADDLE = "#f97316"
RIDER = "#c084fc"
MARKER = "#a855f7"
RAW = "#ef4444"


def _style_ax(ax: Any, title: str, xlabel: str, ylabel: str) -> None:
    ax.set_facecolor(PANEL)
    ax.set_title(title, color=FG, fontsize=12, fontweight="bold", pad=10)
    ax.set_xlabel(xlabel, color=MUTED, fontsize=10)
    ax.set_ylabel(ylabel, color=MUTED, fontsize=10)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(True, linestyle="--", alpha=0.3, color=GRID)
    for spine in ax.spines.values():
        spine.set_color(SPINE)


def _legend(ax: Any, **kwargs: Any) -> None:
    ax.legend(facecolor="#161b22", edgecolor=SPINE, labelcolor=FG, fontsize=8.5, **kwargs)


def _draw_markers(ax: Any, markers: Sequence[Tuple[float, str]], y_frac: float = 0.96) -> None:
    """Draws vertical obstacle markers with staggered labels."""
    for i, (x, label) in enumerate(markers):
        ax.axvline(x, color=MARKER, linestyle=":", linewidth=1.0, alpha=0.8)
        ax.text(
            x, y_frac - 0.06 * (i % 3), label, transform=ax.get_xaxis_transform(),
            rotation=90, va="top", ha="right", color=MARKER, fontsize=7,
        )


def _save(plt: Any, fig: Any, path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(path, facecolor=fig.get_facecolor(), edgecolor="none")
    plt.close(fig)
    return path


# --------------------------------------------------------------------------------------
# Telemetry figures
# --------------------------------------------------------------------------------------


def plot_travel(channels: Dict[str, np.ndarray], track: TrackSpec, fork_travel_mm: float,
                shock_stroke_mm: float, path: Union[str, Path]) -> Path:
    """
    Plots fork travel and shock stroke against track X with obstacle markers.

    Args:
        channels: Recorder channels.
        track: Track ridden, for markers.
        fork_travel_mm: Full fork travel, drawn as a limit line.
        shock_stroke_mm: Full shock stroke, drawn as a limit line.
        path: Output PNG.
    """
    plt, _ = setup_matplotlib()
    fig, axs = plt.subplots(2, 1, figsize=(16, 8), dpi=150, sharex=True)
    fig.patch.set_facecolor(BG)
    x = channels["x_m"]

    _style_ax(axs[0], f"Fork travel along {track.name}", "", "travel (mm)")
    axs[0].plot(x, channels["fork_travel_mm"], color=FORK, linewidth=0.9, label="fork travel")
    axs[0].axhline(fork_travel_mm, color=RAW, linestyle="--", linewidth=0.8, label=f"full travel {fork_travel_mm:.0f} mm")
    _draw_markers(axs[0], track.markers)
    _legend(axs[0], loc="upper left")

    _style_ax(axs[1], "Shock stroke and rear wheel travel", "track x (m)", "mm")
    axs[1].plot(x, channels["shock_stroke_mm"], color=SHOCK, linewidth=0.9, label="shock stroke")
    axs[1].plot(x, channels["rear_wheel_mm"], color=SADDLE, linewidth=0.7, alpha=0.7, label="rear wheel travel")
    axs[1].axhline(shock_stroke_mm, color=RAW, linestyle="--", linewidth=0.8, label=f"full stroke {shock_stroke_mm:.0f} mm")
    _draw_markers(axs[1], track.markers)
    _legend(axs[1], loc="upper left")

    plt.tight_layout()
    return _save(plt, fig, Path(path))


def plot_shaft_velocity(channels: Dict[str, np.ndarray], window: Optional[np.ndarray],
                        path: Union[str, Path], bins: int = 80) -> Path:
    """
    Plots a shaft-velocity histogram for each end, compression to the right.

    Args:
        channels: Recorder channels.
        window: Boolean mask selecting the rows to histogram (None for all).
        path: Output PNG.
        bins: Histogram bins per panel.
    """
    plt, _ = setup_matplotlib()
    fig, axs = plt.subplots(1, 2, figsize=(14, 6), dpi=150)
    fig.patch.set_facecolor(BG)
    mask = np.ones_like(channels["x_m"], dtype=bool) if window is None else window

    for ax, name, color, title in (
        (axs[0], "fork_shaft_mps", FORK, "Fork shaft velocity"),
        (axs[1], "shock_shaft_mps", SHOCK, "Shock shaft velocity"),
    ):
        v = channels[name][mask]
        _style_ax(ax, title, "shaft velocity (m/s), + compression / - rebound", "time share")
        if v.size:
            limit = max(0.5, float(np.percentile(np.abs(v), 99.5)) * 1.1)
            ax.hist(v, bins=bins, range=(-limit, limit), color=color, alpha=0.85, density=True)
            ax.axvline(0.0, color=MUTED, linewidth=0.8)
            comp = v[v > 0]
            reb = v[v < 0]
            note = (
                f"compression: p50 {np.percentile(comp, 50) if comp.size else 0:.3f}, "
                f"p95 {np.percentile(comp, 95) if comp.size else 0:.3f}, max {comp.max() if comp.size else 0:.3f} m/s\n"
                f"rebound: p50 {-np.percentile(-reb, 50) if reb.size else 0:.3f}, "
                f"p95 {-np.percentile(-reb, 95) if reb.size else 0:.3f}, max {reb.min() if reb.size else 0:.3f} m/s"
            )
            ax.text(0.02, 0.97, note, transform=ax.transAxes, va="top", color=FG, fontsize=8.5,
                    bbox=dict(facecolor="#161b22", edgecolor=SPINE))
        ax.set_yscale("log")

    plt.tight_layout()
    return _save(plt, fig, Path(path))


def plot_acceleration(channels: Dict[str, np.ndarray], sample_interval_s: float, track: TrackSpec,
                      path: Union[str, Path]) -> Path:
    """
    Plots low-passed vertical acceleration at bar and saddle against track X.

    Gravity is removed. The raw (unfiltered) peak is annotated per channel so the solver
    transients are visible as a number without dominating the axis.

    Args:
        channels: Recorder channels.
        sample_interval_s: Time between rows, for the filter.
        track: Track ridden, for markers.
        path: Output PNG.
    """
    plt, _ = setup_matplotlib()
    fig, axs = plt.subplots(2, 1, figsize=(16, 8), dpi=150, sharex=True)
    fig.patch.set_facecolor(BG)
    x = channels["x_m"]

    for ax, name, color, title in (
        (axs[0], "bar_acc_vert_mps2", BAR, "Handlebar vertical acceleration"),
        (axs[1], "saddle_acc_vert_mps2", SADDLE, "Saddle vertical acceleration"),
    ):
        raw = channels[name] - GRAVITY_MPS2
        filtered = lowpass(raw, sample_interval_s)
        _style_ax(ax, title, "track x (m)" if ax is axs[1] else "", "m/s^2 (g removed)")
        ax.plot(x, filtered, color=color, linewidth=0.8, label=f"{ACCEL_FILTER_HZ:.0f} Hz low-pass")
        ax.axhline(0.0, color=MUTED, linewidth=0.6)
        if raw.size:
            i = int(np.argmax(np.abs(raw)))
            ax.annotate(
                f"raw peak {raw[i]:+.0f} m/s^2 at x = {x[i]:.1f} m (solver transient, unfiltered)",
                xy=(x[i], filtered[i]), xytext=(0.62, 0.9), textcoords="axes fraction",
                color=RAW, fontsize=8.5, arrowprops=dict(color=RAW, arrowstyle="->", lw=0.8),
                bbox=dict(facecolor="#161b22", edgecolor=SPINE),
            )
        _draw_markers(ax, track.markers)

    # The seated rider's torso, downstream of the saddle and the rider's own compliance,
    # goes on the saddle panel: the two together are the transmissibility of the body.
    torso = channels.get("rider_torso_acc_vert_mps2")
    if torso is not None and torso.size and np.any(torso != 0.0):
        filtered = lowpass(torso - GRAVITY_MPS2, sample_interval_s)
        axs[1].plot(x, filtered, color=RIDER, linewidth=0.8, alpha=0.9,
                    label=f"rider torso, {ACCEL_FILTER_HZ:.0f} Hz low-pass")
    for ax in axs:
        _legend(ax, loc="upper left")

    plt.tight_layout()
    return _save(plt, fig, Path(path))


def plot_ride(channels: Dict[str, np.ndarray], sample_interval_s: float, track: TrackSpec,
              fork_travel_mm: float, shock_stroke_mm: float, out_dir: Union[str, Path],
              window: Optional[np.ndarray] = None) -> List[Path]:
    """
    Renders the three telemetry figures into a directory.

    Returns:
        Paths written: ``travel.png``, ``shaft_velocity.png``, ``acceleration.png``.
    """
    out = Path(out_dir)
    return [
        plot_travel(channels, track, fork_travel_mm, shock_stroke_mm, out / "travel.png"),
        plot_shaft_velocity(channels, window, out / "shaft_velocity.png"),
        plot_acceleration(channels, sample_interval_s, track, out / "acceleration.png"),
    ]


# --------------------------------------------------------------------------------------
# Track preview
# --------------------------------------------------------------------------------------


def plot_track_profile(track: TrackSpec, path: Union[str, Path], resolution_m: float = 0.005) -> Path:
    """
    Draws the road profile with obstacle markers and effective pothole drops.

    Args:
        track: Track to draw.
        path: Output PNG.
        resolution_m: Sampling interval for the profile.
    """
    plt, _ = setup_matplotlib()
    x = np.arange(0.0, track.length_m + resolution_m, resolution_m)
    z = build_profile(track, x)

    fig, axs = plt.subplots(2, 1, figsize=(16, 8), dpi=150, gridspec_kw={"height_ratios": [2, 1]})
    fig.patch.set_facecolor(BG)

    _style_ax(axs[0], f"Road profile: {track.name} ({track.length_m:.0f} m)", "track x (m)", "height (m)")
    axs[0].plot(x, z, color=FG, linewidth=0.9)
    axs[0].fill_between(x, z, np.min(z) - 0.05, color="#30363d", alpha=0.8)
    _draw_markers(axs[0], track.markers)
    if track.description:
        axs[0].text(0.99, 0.04, track.description, transform=axs[0].transAxes, ha="right",
                    color=MUTED, fontsize=8.5)

    _style_ax(axs[1], "Potholes: declared depth vs effective wheel drop", "track x (m)", "mm")
    holes = [o for o in track.sorted_obstacles if isinstance(o, POTHOLE_TYPES)]
    if holes:
        xs = np.array([o.start_m + o.length_m / 2.0 for o in holes])
        declared = np.array([o.depth_m * 1000.0 for o in holes])
        front = np.array([effective_drop_m(o, FRONT_WHEEL_RADIUS_M) * 1000.0 for o in holes])
        rear = np.array([effective_drop_m(o, REAR_WHEEL_RADIUS_M) * 1000.0 for o in holes])
        width = max(0.4, track.length_m / 150.0)
        axs[1].bar(xs - width, declared, width=width, color=MUTED, label="declared depth")
        axs[1].bar(xs, front, width=width, color=FORK, label=f"front wheel drop (r = {FRONT_WHEEL_RADIUS_M*1000:.0f} mm)")
        axs[1].bar(xs + width, rear, width=width, color=SHOCK, label=f"rear wheel drop (r = {REAR_WHEEL_RADIUS_M*1000:.0f} mm)")
        _legend(axs[1], loc="upper left")
    else:
        axs[1].text(0.5, 0.5, "no potholes on this track", transform=axs[1].transAxes, ha="center",
                    color=MUTED, fontsize=10)
    axs[1].set_xlim(axs[0].get_xlim())

    plt.tight_layout()
    return _save(plt, fig, Path(path))


__all__ = [
    "plot_travel",
    "plot_shaft_velocity",
    "plot_acceleration",
    "plot_ride",
    "plot_track_profile",
]
