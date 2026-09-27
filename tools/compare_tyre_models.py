"""Compare the legacy sphere tyre with the pneumatic ride models.

Runs matching tracks, seeds, rider settings and target speeds, then writes the full
``RideSummary`` objects and a Markdown comparison. Run with ``uv run python -m
tools.compare_tyre_models``; add ``--detailed`` to include the higher-fidelity tier.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from bike_sim.cli.ride import resolve_track, track_seed
from bike_sim.physics.tyre import TyreConfig
from bike_sim.sim.ride.metrics import RideSummary, summarize_ride
from bike_sim.sim.ride.recorder import RideRecorder
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import ROAD_LEVEL_SPECS, TrackSpec, available_presets

DEFAULT_OUT_DIR = Path("output/tyre_compare")
DEFAULT_SPEED_KMH = 25.0


def _run_summary(
    track: TrackSpec,
    tyre: TyreConfig,
    target_speed_kmh: float,
    rider: str,
    seed: Optional[int],
) -> RideSummary:
    """Runs one recorded traverse and returns the CLI's normal ride summary."""
    sim = RideSimulation(
        track=track,
        target_speed_kmh=target_speed_kmh,
        rider=rider,
        tyre=tyre,
    )
    # Keep a common 1 ms reporting grid for 0.5 ms and 0.25 ms runs. This remains 10×
    # faster than the 100 Hz acceleration cutoff and preserves exact event accounting.
    decimate = max(1, round(0.001 / float(sim.model.opt.timestep)))
    recorder = RideRecorder(sim, decimate=decimate)
    recorder.record(sim)
    outcome = sim.run(on_step=recorder.record)
    surface = tyre.surface or track.surface
    extras: Dict[str, Any] = {
        "seed": seed,
        "tyre_model": tyre.model,
        "tyre_tier": tyre.tier,
        "tyre_pressure_front_bar": tyre.front.pressure_bar,
        "tyre_pressure_rear_bar": tyre.rear.pressure_bar,
        "surface": surface,
    }
    return summarize_ride(
        recorder.columns(),
        recorder.sample_interval_s,
        track,
        outcome,
        target_speed_kmh=target_speed_kmh,
        fork_travel_mm=sim.specs.fork_travel,
        shock_stroke_mm=sim.specs.shock_stroke,
        shock_bumper_engage_mm=sim.applier.coil_shock.specs.bumper_engage_mm,
        start_x_m=sim.start_x_m,
        extras=extras,
        rider_variant=sim.rider_variant,
        rider_mass_kg=sim.rider.mass_kg,
        tyre_rim_events=recorder.tyre_rim_events,
        tyre_rim_starts=recorder.tyre_rim_starts,
        tyre_times_s=recorder.tyre_times_s,
    )


def compare_tracks(
    tracks: Sequence[TrackSpec],
    *,
    target_speed_kmh: float = DEFAULT_SPEED_KMH,
    rider: str = "seated",
    seeds: Optional[Dict[str, Optional[int]]] = None,
    include_detailed: bool = False,
) -> List[Dict[str, Any]]:
    """Runs matching tyre variants over each supplied track.

    Args:
        tracks: Tracks to compare. Reuse the same instance for every variant so generated
            roads have identical obstacles and roughness.
        target_speed_kmh: Cruise target, shared by every run.
        rider: Rider variant, shared by every run.
        seeds: Effective generator seed keyed by track name, carried into each summary.
        include_detailed: Also run ``pneumatic/detailed``.

    Returns:
        One record per track with a summary for each selected tyre configuration.
    """
    configs: List[Tuple[str, TyreConfig]] = [
        ("sphere", TyreConfig(model="sphere")),
        ("pneumatic/fast", TyreConfig(model="pneumatic", tier="fast")),
    ]
    if include_detailed:
        configs.append(("pneumatic/detailed", TyreConfig(model="pneumatic", tier="detailed")))

    records = []
    for track in tracks:
        seed = (seeds or {}).get(track.name)
        summaries = {
            label: _run_summary(track, config, target_speed_kmh, rider, seed)
            for label, config in configs
        }
        records.append({"track": track.name, "surface": track.surface, "variants": summaries})
    return records


def _relative_change(reference: float, comparison: float) -> float:
    """Relative change from the sphere value, as a fraction."""
    return (comparison - reference) / max(abs(reference), 1e-9)


def _rim_count(summary: RideSummary) -> int:
    """Total front and rear rim strikes in one summary."""
    return sum(summary.tyres[wheel].rim_strikes for wheel in ("front", "rear"))


def _mean_tyre_loss_w(summary: RideSummary) -> float:
    """Mean per-wheel dissipated power, zero for the sphere model."""
    return sum(summary.tyres[w].mean_dissipated_power_w for w in ("front", "rear")) / 2.0


def generated_reading(records: Sequence[Dict[str, Any]]) -> str:
    """Builds a one-paragraph, data-only reading of the sphere/fast results."""
    if not records:
        return "No tracks were run."

    bar_changes = []
    saddle_changes = []
    rim_tracks = []
    sphere_rims = fast_rims = 0
    sphere_complete = fast_complete = 0
    for record in records:
        sphere: RideSummary = record["variants"]["sphere"]
        fast: RideSummary = record["variants"]["pneumatic/fast"]
        sphere_complete += int(sphere.completed)
        fast_complete += int(fast.completed)
        bar_changes.append((record["track"], _relative_change(
            sphere.bar.rms_filtered_mps2, fast.bar.rms_filtered_mps2
        )))
        saddle_changes.append((record["track"], _relative_change(
            sphere.saddle.rms_filtered_mps2, fast.saddle.rms_filtered_mps2
        )))
        sphere_count, fast_count = _rim_count(sphere), _rim_count(fast)
        sphere_rims += sphere_count
        fast_rims += fast_count
        if sphere_count or fast_count:
            rim_tracks.append(f"{record['track']} ({sphere_count}/{fast_count})")

    bar_lower = sum(change < 0.0 for _, change in bar_changes)
    saddle_lower = sum(change < 0.0 for _, change in saddle_changes)
    largest_bar_change = max(bar_changes, key=lambda item: abs(item[1]))
    largest_saddle_change = max(saddle_changes, key=lambda item: abs(item[1]))
    rim_text = ", ".join(rim_tracks) if rim_tracks else "none"
    paragraph = (
        f"Sphere completed {sphere_complete}/{len(records)} presets and pneumatic/fast "
        f"completed {fast_complete}/{len(records)}. Pneumatic/fast lowered bar RMS on "
        f"{bar_lower}/{len(records)} presets and saddle RMS on {saddle_lower}/{len(records)}; "
        f"the largest relative RMS changes were {largest_bar_change[0]} bar "
        f"({_percent(largest_bar_change[1])}) and {largest_saddle_change[0]} saddle "
        f"({_percent(largest_saddle_change[1])}). Total rim strikes were "
        f"{sphere_rims} for sphere and {fast_rims} for pneumatic/fast; affected presets "
        f"(sphere/fast) were {rim_text}."
    )
    return paragraph


def _percent(value: float) -> str:
    """Formats a relative difference with its sign."""
    return f"{100.0 * value:+.1f}%"


def markdown_table(records: Sequence[Dict[str, Any]]) -> str:
    """Formats the per-track summaries for Markdown review."""
    headers = (
        "Track", "Tyre", "Outcome", "Bar RMS / peak", "Saddle RMS / peak",
        "Fork p95 mm", "Shock p95 mm", "Time s", "Wheelspin / lock s",
        "Rim strikes", "Mean tyre loss W",
    )
    lines = ["| " + " | ".join(headers) + " |", "|" + "---|" * len(headers)]
    for record in records:
        for label, summary in record["variants"].items():
            spin = sum(summary.tyres[w].wheelspin_time_s for w in ("front", "rear"))
            locked = sum(summary.tyres[w].locked_time_s for w in ("front", "rear"))
            cells = (
                record["track"],
                label,
                summary.outcome,
                f"{summary.bar.rms_filtered_mps2:.2f} / {summary.bar.peak_filtered_mps2:.2f}",
                f"{summary.saddle.rms_filtered_mps2:.2f} / {summary.saddle.peak_filtered_mps2:.2f}",
                f"{summary.fork.p95_mm:.1f}",
                f"{summary.shock.p95_mm:.1f}",
                f"{summary.sim_time_s:.3f}",
                f"{spin:.3f} / {locked:.3f}",
                str(_rim_count(summary)),
                f"{_mean_tyre_loss_w(summary):.1f}",
            )
            lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def write_report(records: Sequence[Dict[str, Any]], output_dir: Path,
                 target_speed_kmh: float, rider: str) -> Tuple[Path, Path]:
    """Writes summary JSON and a generated Markdown comparison table."""
    output_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "metadata": {
            "target_speed_kmh": float(target_speed_kmh),
            "rider_variant": rider,
            "variants": list(records[0]["variants"]) if records else [],
        },
        "reading": generated_reading(records),
        "tracks": [
            {
                "track": record["track"],
                "surface": record["surface"],
                "variants": {
                    label: summary.to_dict()
                    for label, summary in record["variants"].items()
                },
            }
            for record in records
        ],
    }
    summary_path = output_dir / "summary.json"
    summary_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    table_path = output_dir / "table.md"
    table_path.write_text(
        "# Tyre model comparison\n\n"
        + generated_reading(records)
        + "\n\n"
        + markdown_table(records)
        + "\n",
        encoding="utf-8",
    )
    return summary_path, table_path


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Runs the configured comparison and writes its review artifacts."""
    from bike_sim.physics.rider import RIDER_VARIANTS

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--tracks", nargs="*", default=None, help="presets to compare (default: all)")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED_KMH, metavar="KMH")
    parser.add_argument("--rider", choices=RIDER_VARIANTS, default="seated")
    parser.add_argument("--seed", type=int, default=None, help="override seeds for generated road presets")
    parser.add_argument("--length", type=float, default=None, metavar="M",
                        help="override generated-road length")
    parser.add_argument("--detailed", action="store_true", help="also run pneumatic/detailed")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT_DIR)
    args = parser.parse_args(argv)

    names = args.tracks or available_presets()
    try:
        tracks = [
            resolve_track(
                name,
                seed=args.seed if name in ROAD_LEVEL_SPECS else None,
                length_m=args.length if name in ROAD_LEVEL_SPECS else None,
            )
            for name in names
        ]
    except (KeyError, ValueError) as exc:
        parser.error(str(exc))
    seeds = {
        name: track_seed(name, args.seed if name in ROAD_LEVEL_SPECS else None)
        for name in names
    }
    records = compare_tracks(
        tracks,
        target_speed_kmh=args.speed,
        rider=args.rider,
        seeds=seeds,
        include_detailed=args.detailed,
    )
    summary_path, table_path = write_report(records, args.out, args.speed, args.rider)
    print(generated_reading(records))
    print(f"Wrote {summary_path}")
    print(f"Wrote {table_path}")
    return 0


__all__ = ["compare_tracks", "generated_reading", "markdown_table", "write_report", "main"]


if __name__ == "__main__":
    raise SystemExit(main())
