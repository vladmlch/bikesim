"""
Uphill Climbing and Wheelspin Analysis Tool.

Executes a simulation on a climbing track (default: `climb_steps`) with a 12-speed
adaptive auto-shifting cassette and pneumatic tyres on hardpack ground.
Generates a multi-panel kinematics figure and a JSON summary of wheelspin metrics.
"""

from __future__ import annotations

import argparse
import dataclasses
from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Dict, Optional, Union

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.physics.tyre import TyreConfig
from bike_sim.sim.ride.recorder import RideRecorder
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import SteppedClimb, TrackSpec, build_profile, get_preset

DEFAULT_OUT_DIR = Path("output/wheelspin")


@dataclass
class WheelspinReport:
    """Summary of wheelspin dynamics during an uphill climb."""

    track_name: str
    total_time_s: float
    total_distance_m: float
    critical_gradient_pct: Optional[float]
    total_wheelspin_time_s: float
    total_dissipated_energy_j: float
    step_wheelspin_duration_s: Dict[str, float]
    completed: bool
    outcome: str


def run_wheelspin_analysis(
    track: Union[str, TrackSpec] = "climb_steps",
    out_dir: Union[str, Path] = DEFAULT_OUT_DIR,
    max_seconds: Optional[float] = None,
) -> WheelspinReport:
    """
    Simulates an uphill climb and computes wheelspin / slip telemetry metrics.

    Args:
        track: Preset name or TrackSpec instance.
        out_dir: Directory where plots and summary JSON are written.
        max_seconds: Optional duration cutoff for simulation.

    Returns:
        WheelspinReport with summary statistics.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    track_spec = get_preset(track) if isinstance(track, str) else track

    sim = RideSimulation(
        track=track_spec,
        target_speed_kmh=16.0,
        tyre=TyreConfig(model="pneumatic", tier="fast"),
        drive_mode="pedelec",
        assist="turbo",
    )

    limits = sim.default_limits()
    if max_seconds is not None:
        max_steps = max(1, int(max_seconds / sim.model.opt.timestep))
        limits = dataclasses.replace(limits, max_steps=max_steps)

    recorder = RideRecorder(sim, decimate=1)
    recorder.record(sim)
    outcome = sim.run(limits=limits, on_step=recorder.record)
    channels = recorder.columns()

    time_s = channels["time_s"]
    x_m = channels["x_m"]
    speed_mps = channels["speed_mps"]
    rear_slip = channels["rear_slip_ratio"]
    rear_sliding = channels["rear_tyre_full_sliding"]
    rear_loss_w = channels["rear_tyre_loss_w"]
    rear_fz_n = channels["rear_tyre_fz_n"]
    rear_fx_n = channels["rear_tyre_fx_n"]
    cadence_rpm = channels["cadence_rpm"]
    gear_teeth = channels["gear_teeth"]

    dt = recorder.sample_interval_s

    climb_obs = next((o for o in track_spec.obstacles if isinstance(o, SteppedClimb)), None)
    climb_start_x = climb_obs.start_m if climb_obs is not None else 0.0

    # Wheel linear speed V_tread = Vx * (1 + kappa)
    tread_speed_mps = speed_mps * (1.0 + np.maximum(rear_slip, 0.0))

    # Wheelspin detection: contact patch sliding or exceeding slip threshold while driving forward on climb
    wheelspin_mask = (x_m >= climb_start_x) & (speed_mps > 0.5) & ((rear_sliding > 0.5) | (rear_slip > 0.05))
    total_wheelspin_s = float(np.sum(wheelspin_mask) * dt)
    total_dissipated_j = float(np.sum(rear_loss_w) * dt)

    # Road profile and grade along x
    profile_z = build_profile(track_spec, x_m)
    dx = np.diff(x_m)
    dz = np.diff(profile_z)
    grade_pct = np.zeros_like(x_m)
    valid_dx = dx > 1e-4
    grade_pct[:-1][valid_dx] = (dz[valid_dx] / dx[valid_dx]) * 100.0
    if len(grade_pct) > 1:
        grade_pct[-1] = grade_pct[-2]

    # Critical gradient of first wheelspin onset
    spin_indices = np.where(wheelspin_mask & (grade_pct > 0.5))[0]
    if len(spin_indices) > 0:
        first_idx = spin_indices[0]
        critical_gradient_pct = float(round(float(grade_pct[first_idx]), 1))
    else:
        critical_gradient_pct = None

    # Step wheelspin breakdown
    step_durations: Dict[str, float] = {}
    if climb_obs is not None:
        curr_x = climb_obs.start_m
        for grade, length in climb_obs.steps:
            curr_x += climb_obs.transition_m
            step_mask = (x_m >= curr_x) & (x_m < curr_x + length)
            duration = float(np.sum(wheelspin_mask[step_mask]) * dt) if np.any(step_mask) else 0.0
            step_durations[f"{grade:g}%"] = round(duration, 3)
            curr_x += length
    else:
        step_durations["general"] = round(total_wheelspin_s, 3)

    report = WheelspinReport(
        track_name=track_spec.name,
        total_time_s=float(time_s[-1]) if len(time_s) > 0 else 0.0,
        total_distance_m=float(x_m[-1]) if len(x_m) > 0 else 0.0,
        critical_gradient_pct=critical_gradient_pct,
        total_wheelspin_time_s=round(total_wheelspin_s, 3),
        total_dissipated_energy_j=round(total_dissipated_j, 1),
        step_wheelspin_duration_s=step_durations,
        completed=bool(outcome.completed),
        outcome=str(outcome.reason),
    )

    # Write summary JSON
    json_path = out_dir / "summary.json"
    json_path.write_text(json.dumps(asdict(report), indent=2), encoding="utf-8")

    # Plot multi-panel figure
    fig, axes = plt.subplots(5, 1, figsize=(11, 13), sharex=True)

    # Panel 1: Speeds
    ax1 = axes[0]
    ax1.plot(x_m, speed_mps * 3.6, "b-", label="Chassis Speed $V_x$ (km/h)", linewidth=1.5)
    ax1.plot(x_m, tread_speed_mps * 3.6, "r--", label="Rear Wheel Linear Speed $\\omega R_e$ (km/h)", linewidth=1.2)
    # Highlight wheelspin bursts
    if np.any(wheelspin_mask):
        ax1.fill_between(x_m, 0, np.max(tread_speed_mps * 3.6) * 1.1, where=wheelspin_mask, color="red", alpha=0.2, label="Wheelspin")
    ax1.set_ylabel("Speed (km/h)")
    ax1.set_title(f"Climb Kinematics & Wheelspin Analysis: {track_spec.name}")
    ax1.grid(True, linestyle=":", alpha=0.6)
    ax1.legend(loc="upper left")

    # Panel 2: Profile & Grade
    ax2 = axes[1]
    ax2.plot(x_m, profile_z, "k-", label="Elevation (m)", linewidth=1.5)
    ax2.set_ylabel("Elevation (m)", color="black")
    ax2_twin = ax2.twinx()
    ax2_twin.plot(x_m, grade_pct, "g--", label="Grade (%)", alpha=0.7)
    ax2_twin.set_ylabel("Grade (%)", color="green")
    ax2.grid(True, linestyle=":", alpha=0.6)

    # Panel 3: Gear & Cadence
    ax3 = axes[2]
    ax3.plot(x_m, cadence_rpm, "m-", label="Cadence (RPM)", linewidth=1.5)
    ax3.axhspan(65, 85, color="magenta", alpha=0.1, label="Target Cadence Band (65-85 RPM)")
    ax3.set_ylabel("Cadence (RPM)", color="purple")
    ax3_twin = ax3.twinx()
    ax3_twin.step(x_m, gear_teeth, "tab:orange", where="post", label="Cog Teeth (T)", linewidth=1.5)
    ax3_twin.set_ylabel("Cog Teeth (T)", color="tab:orange")
    ax3.grid(True, linestyle=":", alpha=0.6)
    ax3.legend(loc="lower left")

    # Panel 4: Slip Ratio
    ax4 = axes[3]
    ax4.plot(x_m, rear_slip, color="firebrick", label="Rear Slip Ratio $\\kappa$", linewidth=1.2)
    ax4.axhline(0.05, color="grey", linestyle="--", alpha=0.7, label="Sliding Slip Threshold")
    ax4.set_ylabel("Slip Ratio $\\kappa$")
    ax4.grid(True, linestyle=":", alpha=0.6)
    ax4.legend(loc="upper left")

    # Panel 5: Traction Limit & Drive Force
    ax5 = axes[4]
    mu_peak = 0.80  # Hardpack surface mu_peak
    max_traction_n = mu_peak * np.maximum(rear_fz_n, 0.0)
    ax5.plot(x_m, rear_fx_n, "b-", label="Tractive Force $F_x$ (N)", linewidth=1.2)
    ax5.plot(x_m, max_traction_n, "k--", label=f"Traction Limit $\\mu F_z$ ($\\mu={mu_peak:.2f}$)", linewidth=1.2)
    ax5.set_xlabel("Track Position $x$ (m)")
    ax5.set_ylabel("Force (N)")
    ax5.grid(True, linestyle=":", alpha=0.6)
    ax5.legend(loc="upper left")

    plt.tight_layout()
    fig.savefig(out_dir / "climb_kinematics.png", dpi=150)
    plt.close(fig)

    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze wheelspin during uphill climbing.")
    parser.add_argument("--track", default="climb_steps", help="track preset (default: climb_steps)")
    parser.add_argument("--out", default=str(DEFAULT_OUT_DIR), help="output directory")
    parser.add_argument("--max-seconds", type=float, default=None, help="max simulation time")
    args = parser.parse_args()

    print(f"Running wheelspin analysis on '{args.track}' -> {args.out}...")
    report = run_wheelspin_analysis(track=args.track, out_dir=args.out, max_seconds=args.max_seconds)
    print("\nWheelspin Analysis Report:")
    print(f"  Track:                     {report.track_name}")
    print(f"  Total Distance:            {report.total_distance_m:.1f} m in {report.total_time_s:.2f} s")
    print(f"  Critical Gradient:         {report.critical_gradient_pct}%" if report.critical_gradient_pct else "  Critical Gradient:         None (no wheelspin)")
    print(f"  Total Wheelspin Duration:  {report.total_wheelspin_time_s:.3f} s")
    print(f"  Total Dissipated Energy:   {report.total_dissipated_energy_j:.1f} J")
    print("  Step Durations:")
    for step, dur in report.step_wheelspin_duration_s.items():
        print(f"    {step:5s}: {dur:.3f} s")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
