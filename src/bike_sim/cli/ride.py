"""
CLI Entrypoint for Ride Mode: ``bike-ride``.

Rides a track -- a built-in preset or a TOML track file -- either interactively in the
MuJoCo viewer or headlessly with telemetry, summary metrics and plots written to disk.
Also previews a track's profile without simulating, and dumps any preset as a track file
to edit.

Geometry has one source of truth, the track (preset or file); the command line carries
only run-level knobs: seed and length overrides for generated roads, speed, sag target,
output directory and decimation.
"""

import argparse
from pathlib import Path
import sys
from typing import List, Optional, Sequence

from bike_sim.geometry.specs import BikeSpecs
from bike_sim.sim.ride.cruise import (
    DEFAULT_TARGET_SPEED_KMH,
    MAX_TARGET_SPEED_KMH,
    MIN_TARGET_SPEED_KMH,
)
from bike_sim.terrain import (
    DEFAULT_ROAD_PRESET,
    PRESETS,
    ROAD_LEVEL_SPECS,
    TrackFileError,
    TrackSpec,
    available_presets,
    build_road,
    dump_track,
    get_preset,
    load_track,
)

DEFAULT_OUT_DIR = "output/ride"
LONG_TRACK_WARNING_M = 500.0
PREFIX = "[bike-ride]"


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """Builds the argument parser and parses ``argv``."""
    parser = argparse.ArgumentParser(
        prog="bike-ride",
        description=(
            "Ride the bike over a track in MuJoCo. Default: the interactive viewer on the "
            f"'{DEFAULT_ROAD_PRESET}' road. --headless writes telemetry.csv, summary.json and "
            "plots instead."
        ),
        epilog=(
            "Track sizes and shapes live in the track file ([[obstacles]] and [generator]); "
            "see docs/RIDE.md. `bike-ride --dump-track road_worn > my.toml` gives you one to edit."
        ),
    )
    parser.add_argument(
        "--track", default=DEFAULT_ROAD_PRESET,
        help=f"preset name or path to a .toml track file (default: {DEFAULT_ROAD_PRESET}); "
             f"presets: {', '.join(available_presets())}",
    )
    parser.add_argument("--seed", type=int, default=None,
                        help="override the generator seed of a road preset or track file")
    parser.add_argument("--length", type=float, default=None, metavar="M",
                        help="override the track length in metres of a road preset or track file")
    parser.add_argument("--speed", type=float, default=DEFAULT_TARGET_SPEED_KMH, metavar="KMH",
                        help=f"cruise target in km/h, {MIN_TARGET_SPEED_KMH:.0f}-{MAX_TARGET_SPEED_KMH:.0f} "
                             f"(default {DEFAULT_TARGET_SPEED_KMH:.0f})")
    parser.add_argument("--headless", action="store_true",
                        help="run without a viewer and write telemetry, summary and plots")
    parser.add_argument("--sag", type=float, default=None, metavar="PCT",
                        help="fit fork pressure and coil rate to this static sag (both ends) instead of "
                             "the shipped tune; headless only")
    parser.add_argument("--out", default=DEFAULT_OUT_DIR, metavar="DIR",
                        help=f"root directory for run artifacts (default {DEFAULT_OUT_DIR})")
    parser.add_argument("--decimate", type=int, default=1, metavar="N",
                        help="store every N-th step in telemetry.csv (default 1 = every step)")
    parser.add_argument("--no-rider", action="store_true", help="ride without the rider mass")
    parser.add_argument("--preview", action="store_true",
                        help="render the track profile with effective pothole drops and exit")
    parser.add_argument("--dump-track", metavar="NAME", default=None,
                        help="print a preset as a TOML track file and exit")
    parser.add_argument("--list-tracks", action="store_true", help="list the built-in presets and exit")
    parser.add_argument("--no-plots", action="store_true", help="headless: skip the PNG figures")
    return parser.parse_args(argv)


# --------------------------------------------------------------------------------------
# Track resolution
# --------------------------------------------------------------------------------------


def resolve_track(name_or_path: str, seed: Optional[int] = None, length_m: Optional[float] = None) -> TrackSpec:
    """
    Turns the ``--track`` argument into a validated track.

    Args:
        name_or_path: Registered preset name, or a path to a TOML track file.
        seed: Generator seed override; only meaningful for road presets and files with a
            ``[generator]`` block.
        length_m: Track length override, same applicability.

    Returns:
        The track.

    Raises:
        TrackFileError: On a malformed file.
        ValueError: On an unknown preset, or an override that does not apply.
    """
    path = Path(name_or_path)
    if name_or_path.endswith(".toml") or path.is_file():
        if not path.is_file():
            raise ValueError(f"track file not found: {path}")
        return load_track(path, seed=seed, length_m=length_m)

    if name_or_path in ROAD_LEVEL_SPECS:
        spec = ROAD_LEVEL_SPECS[name_or_path]()
        if seed is not None:
            spec.seed = int(seed)
        kwargs = {"length_m": float(length_m)} if length_m is not None else {}
        return build_road(name_or_path, spec, description=get_preset(name_or_path).description, **kwargs)

    if name_or_path not in PRESETS:
        raise ValueError(f"unknown track '{name_or_path}'; presets: {', '.join(available_presets())} "
                         f"-- or give a path to a .toml file")
    if seed is not None or length_m is not None:
        raise ValueError(f"--seed/--length only apply to generated roads (road_*) or track files "
                         f"with a [generator] block; '{name_or_path}' is authored")
    return get_preset(name_or_path)


def track_seed(name_or_path: str, seed: Optional[int]) -> Optional[int]:
    """
    The generator seed a run should be labelled with, or None when none applies.

    Args:
        name_or_path: The ``--track`` argument.
        seed: The ``--seed`` override.

    Returns:
        The override if given; else a road preset's built-in seed; else a track file's
        ``[generator] seed`` (0 when the block omits it); else None for authored tracks.
    """
    if seed is not None:
        return int(seed)
    if name_or_path in ROAD_LEVEL_SPECS:
        return int(ROAD_LEVEL_SPECS[name_or_path]().seed)
    path = Path(name_or_path)
    if path.is_file():
        import tomllib

        try:
            with open(path, "rb") as handle:
                data = tomllib.load(handle)
        except (OSError, tomllib.TOMLDecodeError):
            return None
        generator = data.get("generator")
        if isinstance(generator, dict):
            return int(generator.get("seed", 0))
    return None


def run_dir_name(track: TrackSpec, speed_kmh: float, seed: Optional[int]) -> str:
    """``<track>_<speed>_s<seed>`` (seed suffix only when one applies)."""
    name = f"{track.name}_{speed_kmh:g}"
    return f"{name}_s{seed}" if seed is not None else name


# --------------------------------------------------------------------------------------
# Modes
# --------------------------------------------------------------------------------------


def _list_tracks() -> int:
    for name in available_presets():
        track = get_preset(name)
        kind = "generated" if name in ROAD_LEVEL_SPECS else "authored"
        print(f"  {name:20s} {track.length_m:6.0f} m  {kind:9s}  {track.description}")
    return 0


def _dump_track(name: str) -> int:
    if name not in PRESETS:
        print(f"{PREFIX} unknown preset '{name}'; presets: {', '.join(available_presets())}", file=sys.stderr)
        return 1
    sys.stdout.write(dump_track(get_preset(name)))
    return 0


def _preview(track: TrackSpec, out_root: Path, seed: Optional[int]) -> int:
    from bike_sim.viz.ride_plots import plot_track_profile

    suffix = f"_s{seed}" if seed is not None else ""
    path = plot_track_profile(track, out_root / f"preview_{track.name}{suffix}.png")
    print(f"{PREFIX} {track.name}: {track.length_m:.0f} m, {len(track.markers)} marked obstacles")
    for x, label in track.markers:
        print(f"    {x:7.2f} m  {label}")
    print(f"{PREFIX} wrote {path}")
    return 0


def _fit_sag(target_pct: float, specs: BikeSpecs):
    """Fits both ends to ``target_pct`` against the model's own centre of mass."""
    from bike_sim.physics.air_spring import AirSpringSpecs, ForkAirSpring
    from bike_sim.physics.coil_shock import CoilShock, CoilShockSpecs
    from bike_sim.physics.damper import BikeSuspensionSystem
    from bike_sim.physics.tuning import compute_suspension_tuning_for_sag
    from bike_sim.sim.controllers import SuspensionController

    fit = compute_suspension_tuning_for_sag(
        specs=specs, target_front_sag_pct=target_pct, target_rear_sag_pct=target_pct,
        front_weight_fraction=None,
    )
    psi = float(fit["fork_calibrated_psi"])
    tokens = int(fit["fork_air_tokens"])
    rate = float(fit["shock_stiffness_n_m"])
    controller = SuspensionController(
        specs=specs,
        air_spring=ForkAirSpring(
            specs=AirSpringSpecs(total_travel_mm=specs.fork_travel), num_tokens=tokens, gauge_pressure_psi=psi,
        ),
        suspension_system=BikeSuspensionSystem(),
    )
    coil = CoilShock(CoilShockSpecs(rate_n_m=rate))
    print(f"{PREFIX} --sag {target_pct:g}%: fork {psi:.1f} psi ({tokens} tokens) instead of "
          f"{specs.fork_initial_psi:.1f}; coil {rate:,.0f} N/m instead of {specs.shock_stiffness:,.0f}")
    return controller, coil, {"sag_target_pct": target_pct, "fork_psi": psi, "coil_rate_n_m": rate}


def _headless(track: TrackSpec, args: argparse.Namespace, seed: Optional[int]) -> int:
    from bike_sim.sim.ride.metrics import RAMP_EXCLUSION_M, summarize_ride
    from bike_sim.sim.ride.recorder import RideRecorder
    from bike_sim.sim.ride_sim import RideSimulation

    specs = BikeSpecs()
    controller = coil = None
    extras = {}
    if seed is not None:
        extras["seed"] = float(seed)
    if args.sag is not None:
        controller, coil, extras_sag = _fit_sag(args.sag, specs)
        extras.update(extras_sag)

    run_dir = Path(args.out) / run_dir_name(track, args.speed, seed)
    print(f"{PREFIX} {track.name}: {track.length_m:.0f} m, {len(track.markers)} marked obstacles, "
          f"target {args.speed:g} km/h -> {run_dir}")

    sim = RideSimulation(
        track=track, specs=specs, target_speed_kmh=args.speed, include_rider=not args.no_rider,
        controller=controller, coil_shock=coil,
    )
    eq = sim.equilibrium
    print(f"{PREFIX} start equilibrium: fork {eq['fork_travel_mm']:.1f} mm "
          f"({100 * eq['fork_travel_mm'] / specs.fork_travel:.1f} %), shock {eq['shock_stroke_mm']:.1f} mm "
          f"({100 * eq['shock_stroke_mm'] / specs.shock_stroke:.1f} %)")
    extras["start_fork_travel_mm"] = float(eq["fork_travel_mm"])
    extras["start_shock_stroke_mm"] = float(eq["shock_stroke_mm"])

    recorder = RideRecorder(sim, decimate=args.decimate)
    recorder.record(sim)
    outcome = sim.run(on_step=recorder.record)
    print(f"{PREFIX} {outcome.describe()}")

    channels = recorder.columns()
    summary = summarize_ride(
        channels, recorder.sample_interval_s, track, outcome, args.speed,
        fork_travel_mm=specs.fork_travel, shock_stroke_mm=specs.shock_stroke,
        shock_bumper_engage_mm=sim.applier.coil_shock.specs.bumper_engage_mm,
        start_x_m=sim.start_x_m, extras=extras,
    )

    written: List[Path] = [recorder.write_csv(run_dir / "telemetry.csv"), summary.write_json(run_dir / "summary.json")]
    if not args.no_plots:
        from bike_sim.viz.ride_plots import plot_ride, plot_track_profile

        window = channels["x_m"] >= sim.start_x_m + RAMP_EXCLUSION_M
        written += plot_ride(channels, recorder.sample_interval_s, track, specs.fork_travel,
                             specs.shock_stroke, run_dir, window=window)
        written.append(plot_track_profile(track, run_dir / "profile.png"))

    print()
    print(summary.format_table())
    print()
    for path in written:
        print(f"{PREFIX} wrote {path}")
    return 0


def _interactive(track: TrackSpec, args: argparse.Namespace) -> int:
    if args.sag is not None:
        print(f"{PREFIX} --sag applies to headless runs only; the viewer uses the shipped tune "
              f"(P cycles damper presets, -/= change pressure)", file=sys.stderr)
    from bike_sim.sim.ride.viewer import run_interactive_ride

    run_interactive_ride(track=track, target_speed_kmh=args.speed, include_rider=not args.no_rider)
    return 0


# --------------------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------------------


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point; returns the process exit code."""
    args = parse_args(argv)

    if args.list_tracks:
        return _list_tracks()
    if args.dump_track is not None:
        return _dump_track(args.dump_track)
    if args.decimate < 1:
        print(f"{PREFIX} --decimate must be >= 1", file=sys.stderr)
        return 2
    if not MIN_TARGET_SPEED_KMH <= args.speed <= MAX_TARGET_SPEED_KMH:
        print(f"{PREFIX} --speed must be within {MIN_TARGET_SPEED_KMH:.0f}-{MAX_TARGET_SPEED_KMH:.0f} km/h",
              file=sys.stderr)
        return 2

    try:
        track = resolve_track(args.track, seed=args.seed, length_m=args.length)
    except (TrackFileError, ValueError, FileNotFoundError) as exc:
        print(f"{PREFIX} {exc}", file=sys.stderr)
        return 1

    if track.length_m > LONG_TRACK_WARNING_M:
        print(f"{PREFIX} note: a {track.length_m:.0f} m track is a large heightfield for the interactive "
              f"viewer; headless runs are unaffected", file=sys.stderr)

    seed = track_seed(args.track, args.seed)
    if args.preview:
        return _preview(track, Path(args.out), seed)
    if args.headless:
        return _headless(track, args, seed)
    return _interactive(track, args)


if __name__ == "__main__":
    sys.exit(main())
