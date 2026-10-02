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
from bike_sim.physics.tyre import FRONT_TYRE, REAR_TYRE
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


def plot_tyres(channels: Dict[str, np.ndarray], track: TrackSpec,
               path: Union[str, Path]) -> Path:
    """Plots pneumatic tyre load, deflection/rim strikes, slip and patch length."""
    plt, _ = setup_matplotlib()
    fig, axs = plt.subplots(2, 2, figsize=(16, 9), dpi=150, sharex=True)
    fig.patch.set_facecolor(BG)
    x = channels["x_m"]
    wheel_data = (
        ("front", FORK, FRONT_TYRE),
        ("rear", SHOCK, REAR_TYRE),
    )

    def _event_x(wheel: str) -> np.ndarray:
        flag = np.asarray(channels[f"{wheel}_rim_strike"], dtype=float) > 0.5
        if not flag.size:
            return np.zeros(0)
        starts = flag & ~np.r_[False, flag[:-1]]
        return x[starts]

    _style_ax(axs[0, 0], "Tyre vertical load", "", "load (N)")
    _style_ax(axs[0, 1], "Tyre deflection and rim strikes", "", "deflection (mm)")
    _style_ax(axs[1, 0], "Longitudinal slip", "track x (m)", "slip ratio κ")
    _style_ax(axs[1, 1], "Contact patch length", "track x (m)", "length (mm)")
    axs[1, 0].axhline(0.0, color=MUTED, linewidth=0.7)
    axs[1, 0].axhline(-1.0, color=RAW, linestyle="--", linewidth=0.8, label="locked wheel κ = −1")

    for wheel, color, specs in wheel_data:
        axs[0, 0].plot(x, channels[f"{wheel}_tyre_fz_n"], color=color, linewidth=0.8,
                       label=f"{wheel} Fz")
        axs[0, 1].plot(x, channels[f"{wheel}_tyre_deflection_mm"], color=color, linewidth=0.8,
                       label=f"{wheel} deflection")
        axs[0, 1].axhline(specs.rim_strike_deflection_mm, color=color, linestyle=":", linewidth=0.8,
                           label=f"{wheel} rim threshold")
        axs[1, 0].plot(x, channels[f"{wheel}_slip_ratio"], color=color, linewidth=0.8,
                       label=f"{wheel} κ")
        axs[1, 1].plot(x, channels[f"{wheel}_patch_length_mm"], color=color, linewidth=0.8,
                       label=f"{wheel} patch")
        for event_x in _event_x(wheel):
            for ax in (axs[0, 0], axs[0, 1], axs[1, 1]):
                ax.axvline(event_x, color=RAW, linestyle="--", linewidth=0.8, alpha=0.65)

    _draw_markers(axs[0, 0], track.markers)
    _draw_markers(axs[0, 1], track.markers)
    _draw_markers(axs[1, 0], track.markers)
    _draw_markers(axs[1, 1], track.markers)
    for ax in axs.flat:
        _legend(ax, loc="upper left")
    plt.tight_layout()
    return _save(plt, fig, Path(path))


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
    "plot_tyres",
    "plot_track_profile",
    "load_ride_csv",
    "plot_physical_ride_html",
]


def plot_physical_ride(channels, output_dir):
    """Plot recorded schema-2 observations; no reconstructed wheel/battery power."""
    from pathlib import Path
    import matplotlib.pyplot as plt
    out=Path(output_dir);out.mkdir(parents=True,exist_ok=True)
    time=channels['time_s']
    groups=(('speed',('speed_mps',),'Speed (m/s)'),
            ('normal_load',('tires.front.normal_load_n','tires.rear.normal_load_n'),'Normal force (N)'),
            ('power',('drive.motor_shaft_power_w','drive.electrical_power_w'),'Power (W)'),
            ('energy_residual',('energy.residual_j',),'Mechanical residual (J)'))
    paths=[]
    for name,keys,label in groups:
        fig,ax=plt.subplots(figsize=(9,4))
        for key in keys:
            if key in channels: ax.plot(time,channels[key],label=key)
        ax.set_xlabel('Force interval start (s)');ax.set_ylabel(label)
        ax.legend();ax.grid(True);fig.tight_layout()
        path=out/(name+'.png');fig.savefig(path,dpi=130);plt.close(fig);paths.append(path)
    return paths


# --------------------------------------------------------------------------------------
# Interactive HTML figures (physical preview.csv schema)
# --------------------------------------------------------------------------------------


def load_ride_csv(path: Union[str, Path]) -> Dict[str, np.ndarray]:
    """
    Loads a ride CSV into a channels dict, matching ``recorder.columns()``.

    Numeric columns become float arrays (missing entries become NaN); columns
    containing any non-numeric value become object arrays of strings.
    """
    import csv

    with Path(path).open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    channels: Dict[str, np.ndarray] = {}
    for name in (rows[0].keys() if rows else []):
        values = [row.get(name) or "" for row in rows]
        try:
            channels[name] = np.array(
                [float(v) if v != "" else np.nan for v in values], dtype=float
            )
        except ValueError:
            channels[name] = np.array(values, dtype=object)
    return channels


def _series(channels: Dict[str, np.ndarray], name: str) -> Optional[np.ndarray]:
    values = channels.get(name)
    if values is None or values.dtype == object:
        return None
    return values


def _obstacle_positions(channels: Dict[str, np.ndarray]) -> List[Tuple[float, str]]:
    """Parses unique ``Name@Xm`` obstacle labels into (x, label) marker pairs."""
    import re

    labels = channels.get("obstacle")
    if labels is None:
        return []
    out = []
    seen = set()
    for label in labels:
        match = re.search(r"@(-?\d+(?:\.\d+)?)m\b", str(label))
        if match and label not in seen:
            seen.add(label)
            out.append((float(match.group(1)), str(label)))
    return sorted(out)


def _shift_events(channels: Dict[str, np.ndarray]) -> List[Tuple[float, str]]:
    """Returns (time_s, label) for each step change of ``shift_count``."""
    count, time = _series(channels, "shift_count"), _series(channels, "time_s")
    if count is None or time is None:
        return []
    direction = channels.get("last_shift_direction")
    from_t, to_t = channels.get("last_shift_from_teeth"), channels.get("last_shift_to_teeth")
    events = []
    for i in range(1, len(count)):
        if count[i] != count[i - 1] and np.isfinite(count[i]):
            label = ""
            if direction is not None and from_t is not None and to_t is not None:
                label = f"{direction[i]}:{int(from_t[i])}->{int(to_t[i])}"
            events.append((float(time[i]), label))
    return events


def _html_layout(fig: Any, title: str, height: int) -> None:
    """Applies the dark ride-plots theme to a plotly figure."""
    fig.update_layout(
        title=dict(text=title, font=dict(color=FG, size=14)),
        paper_bgcolor=BG, plot_bgcolor=PANEL, font=dict(color=FG, size=11),
        hovermode="x unified", height=height,
        legend=dict(bgcolor="#161b22", bordercolor=SPINE, font=dict(size=10)),
        margin=dict(l=60, r=60, t=50, b=40),
    )
    fig.update_xaxes(gridcolor=GRID, zerolinecolor=GRID)
    fig.update_yaxes(gridcolor=GRID, zerolinecolor=GRID)


def plot_physical_ride_html(channels: Dict[str, np.ndarray],
                            output_dir: Union[str, Path],
                            filename: str = "ride.html") -> Path:
    """
    Interactive HTML dashboards for a physical ``preview.csv`` recording.

    Two figures share one file: an effort/drivetrain view over time (speed and
    grade, motor vs human power, cadence tracking with shift-cut shading, gear
    steps with shift markers, motor request vs delivered torque) and a
    suspension/load view over track position with obstacle markers. Unified
    hover reports every curve at the cursor's x.

    Args:
        channels: Channels keyed by CSV column name (``load_ride_csv`` or
            ``PhysicalRecorder.columns()``).
        output_dir: Directory for the output file.
        filename: Output file name.

    Requires plotly (``uv run --with plotly``); imported lazily so the package
    does not depend on it.
    """
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    time = _series(channels, "time_s")
    x_m = _series(channels, "x_m")
    if time is None or x_m is None:
        raise ValueError("channels lack time_s/x_m; is this a physical preview.csv?")

    def add(fig: Any, xs: np.ndarray, name: str, row: int, color: str,
            secondary: bool = False, step: bool = False, text: Optional[Sequence[str]] = None) -> None:
        ys = _series(channels, name)
        if ys is None:
            return
        fig.add_trace(go.Scatter(
            x=xs, y=ys, name=name, text=text,
            line=dict(color=color, width=1.4, shape="hv" if step else "linear"),
        ), row=row, col=1, secondary_y=secondary)

    # Figure 1 -- effort and drivetrain over time
    fig1 = make_subplots(
        rows=5, cols=1, shared_xaxes=True, vertical_spacing=0.035,
        specs=[[{"secondary_y": True}], [{"secondary_y": False}], [{"secondary_y": False}],
               [{"secondary_y": True}], [{"secondary_y": False}]],
        subplot_titles=("Speed and grade", "Power: motor vs human", "Cadence vs required",
                        "Rear gear and freehub torque", "Motor request vs delivered"),
    )
    add(fig1, time, "speed_kmh", 1, FORK)
    add(fig1, time, "grade_pct", 1, MUTED, secondary=True)
    cadence = _series(channels, "cadence_rpm")
    human_nm = _series(channels, "human_sensor_nm")
    add(fig1, time, "motor_shaft_power_w", 2, SHOCK)
    if cadence is not None and human_nm is not None:
        human_w = human_nm * cadence * (2.0 * np.pi / 60.0)
        fig1.add_trace(go.Scatter(x=time, y=human_w, name="human_power_w (sensor Nm x cadence)",
                                  line=dict(color=RIDER, width=1.4)), row=2, col=1)
    add(fig1, time, "cadence_rpm", 3, BAR)
    add(fig1, time, "required_cadence_rpm", 3, RAW)
    cut = _series(channels, "shift_torque_factor")
    if cut is not None:
        in_cut = cut < 0.999
        edges = np.diff(in_cut.astype(int))
        for start, end in zip(np.where(edges == 1)[0], np.where(edges == -1)[0]):
            fig1.add_vrect(float(time[start]), float(time[end]), row=3, col=1,
                           fillcolor=MARKER, opacity=0.18, line_width=0)
    add(fig1, time, "gear_rear_teeth", 4, SADDLE, step=True)
    add(fig1, time, "gear_front_teeth", 4, MUTED, step=True)
    add(fig1, time, "freehub_torque_nm", 4, RIDER, secondary=True)
    for t, label in _shift_events(channels):
        fig1.add_vline(t, row=4, col=1, line=dict(color=MARKER, dash="dot", width=1))
        fig1.add_annotation(x=t, y=1.0, yref="y4 domain", text=label, showarrow=False,
                            font=dict(color=MARKER, size=9), textangle=-90,
                            xanchor="left", yanchor="top", row=4, col=1)
    add(fig1, time, "motor_request_nm", 5, MUTED)
    add(fig1, time, "motor_torque_nm", 5, SHOCK)
    fig1.update_yaxes(title_text="km/h | %", row=1, col=1)
    fig1.update_yaxes(title_text="W", row=2, col=1)
    fig1.update_yaxes(title_text="rpm", row=3, col=1)
    fig1.update_yaxes(title_text="teeth", row=4, col=1)
    fig1.update_yaxes(title_text="Nm", row=4, col=1, secondary_y=True)
    fig1.update_yaxes(title_text="Nm", row=5, col=1)
    fig1.update_xaxes(title_text="time (s)", row=5, col=1)
    _html_layout(fig1, "Effort and drivetrain", height=1100)

    # Figure 2 -- suspension and wheel load over track position
    fig2 = make_subplots(rows=2, cols=1, shared_xaxes=True, vertical_spacing=0.06,
                         subplot_titles=("Suspension travel", "Wheel normal load"))
    add(fig2, x_m, "fork_travel_mm", 1, FORK)
    add(fig2, x_m, "shock_stroke_mm", 1, SHOCK)
    add(fig2, x_m, "front_load_n", 2, BAR)
    add(fig2, x_m, "rear_load_n", 2, SADDLE)
    for pos, label in _obstacle_positions(channels):
        fig2.add_vline(pos, line=dict(color=MARKER, dash="dot", width=1))
        fig2.add_annotation(x=pos, y=1.0, yref="paper", text=label, showarrow=False,
                            font=dict(color=MARKER, size=9), textangle=-90,
                            xanchor="left", yanchor="top")
    fig2.update_yaxes(title_text="mm", row=1, col=1)
    fig2.update_yaxes(title_text="N", row=2, col=1)
    fig2.update_xaxes(title_text="track x (m)", row=2, col=1)
    _html_layout(fig2, "Suspension and wheel load", height=650)

    path = out / filename
    path.write_text(
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        f"<title>{path.stem}</title>"
        f"<style>body{{background:{BG};margin:0;padding:12px}}</style></head><body>"
        + fig1.to_html(full_html=False, include_plotlyjs=True)
        + fig2.to_html(full_html=False, include_plotlyjs=False)
        + "</body></html>",
        encoding="utf-8",
    )
    return path
