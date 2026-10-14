"""CLI entrypoint ``bike-settle``: settle a physical scenario once, write the pose.

The run relaxes the fully coupled physical model to its validated initial
equilibrium — the same solve ``solve_physical_equilibrium`` performs at the
start of every physical ride — and writes the settled pose plus the committed
material state to ``--out`` as an npz pose file. A run whose physics config sets
``equilibrium_state_file`` to that file loads the pose instead of re-settling;
the embedded configuration fingerprint, the strict material-state restore and
a re-measured residual are all enforced at load time.

Every runtime-selection flag is forwarded verbatim to ``bike-ride``'s parser so
the settled runtime is identical to the one a ride with the same flags builds.
The pose file is pinned to the resolved configuration — track, rider, physics
config and compiled model — not to the mechanism that produced it: settle and
load must run the same scenario, while ``equilibrium_cache_enabled`` stays a
local mechanism choice on each side.
"""

import argparse
from math import isfinite
from pathlib import Path
import sys
import time
from typing import Optional, Sequence

from bike_sim.cli import ride
from bike_sim.physics.drivetrain import ASSIST_ORDER
from bike_sim.physics.rider import RIDER_VARIANTS
from bike_sim.terrain import DEFAULT_ROAD_PRESET, TrackFileError, available_presets

PREFIX = "[bike-settle]"
DEFAULT_MAX_STEPS = 400000

# Flags forwarded to ride.parse_args: name -> (metavar help for the settle parser).
_FORWARDED = {
    "physics-config": "physical-mode TOML profile; overrides below win over its values",
    "physics": "'physical' selects the physical model without a TOML",
    "track": f"preset name or path to a .toml track file (default: {DEFAULT_ROAD_PRESET}); "
             f"presets: {', '.join(available_presets())}",
    "seed": "generator seed override for road presets and seeded track files",
    "length": "track length override in metres, same applicability as --seed",
    "rider": f"rider model (default: articulated_planar for articulated_effort, "
             "lumped otherwise)",
    "rider-mass": "rider mass including helmet and kit, kg (default 80)",
    "rider-height": "rider stature in metres (default 1.80)",
    "rider-inseam": "rider inseam in metres (default 0.47 x height)",
    "speed": "cruise target in km/h; only an ideal_speed_control target",
    "drive": "physical drive mode override: coast, ideal_speed_control, "
             "crank_effort, articulated_effort",
    "timestep": "integration step in seconds",
    "initial-speed": "initial speed in km/h",
    "human-torque": "rider crank torque for effort drive modes, N m",
    "assist-gain": "assist gain override",
    "assist": "assist mode override",
    "crank-phase": "crank angle at the start of the run, degrees",
}

def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """Builds the settle argument parser and parses ``argv``."""
    parser = argparse.ArgumentParser(
        prog="bike-settle",
        description=(
            "Settle a physical-mode scenario into its validated initial equilibrium and "
            "write the pose to --out. Runs whose physics config sets equilibrium_state_file "
            "to the written file start from that pose instead of re-settling."
        ),
        epilog=(
            "The pose file embeds the fingerprint of the resolved configuration; a run "
            "loads it only when track, rider, physics config and compiled model all match. "
            "bike-settle never runs the ride itself."
        ),
    )
    parser.add_argument(
        "--physics-config", default=None, metavar="PATH",
        help=_FORWARDED["physics-config"])
    parser.add_argument("--physics", choices=("legacy", "physical"), default=None,
                        help=_FORWARDED["physics"])
    parser.add_argument("--track", default=DEFAULT_ROAD_PRESET,
                        help=_FORWARDED["track"])
    parser.add_argument("--seed", type=int, default=None, help=_FORWARDED["seed"])
    parser.add_argument("--length", type=float, default=None, metavar="M",
                        help=_FORWARDED["length"])
    parser.add_argument("--rider", choices=RIDER_VARIANTS, default=None,
                        help=_FORWARDED["rider"])
    parser.add_argument("--rider-mass", type=float, default=None, metavar="KG",
                        help=_FORWARDED["rider-mass"])
    parser.add_argument("--rider-height", type=float, default=None, metavar="M",
                        help=_FORWARDED["rider-height"])
    parser.add_argument("--rider-inseam", type=float, default=None, metavar="M",
                        help=_FORWARDED["rider-inseam"])
    parser.add_argument("--no-rider", action="store_true",
                        help="settle the bike alone (alias for --rider none)")
    parser.add_argument("--speed", type=float, default=None, metavar="KMH",
                        help=_FORWARDED["speed"])
    parser.add_argument("--drive",
                        choices=("coast", "ideal_speed_control", "crank_effort",
                                 "articulated_effort"),
                        default=None, help=_FORWARDED["drive"])
    parser.add_argument("--timestep", type=float, default=None, metavar="S",
                        help=_FORWARDED["timestep"])
    parser.add_argument("--initial-speed", type=float, default=None, metavar="KMH",
                        help=_FORWARDED["initial-speed"])
    parser.add_argument("--human-torque", type=float, default=None, metavar="NM",
                        help=_FORWARDED["human-torque"])
    parser.add_argument("--assist-gain", type=float, default=None,
                        help=_FORWARDED["assist-gain"])
    parser.add_argument("--assist", choices=(*ASSIST_ORDER, "emtb"), default=None,
                        help=_FORWARDED["assist"])
    parser.add_argument("--crank-phase", type=float, default=None, metavar="DEG",
                        help=_FORWARDED["crank-phase"])
    parser.add_argument("--max-steps", type=int, default=DEFAULT_MAX_STEPS,
                        help=f"settle budget in integration steps (default {DEFAULT_MAX_STEPS})")
    parser.add_argument("--tolerance", type=float, default=.05,
                        help="convergence tolerance on max |qacc| (default .05)")
    parser.add_argument("--out", required=True, metavar="PATH",
                        help="the pose file to write (.npz)")
    args = parser.parse_args(argv)
    if args.max_steps <= 0:
        parser.error("--max-steps must be a positive integer")
    if not isfinite(args.tolerance) or args.tolerance <= 0.:
        parser.error("--tolerance must be finite and positive")
    out = Path(args.out)
    if out.exists() and out.is_dir():
        parser.error("--out must be a file path, not a directory")

    ride_argv = []
    for name in _FORWARDED:
        value = getattr(args, name.replace("-", "_"))
        if value is not None:
            ride_argv += [f"--{name}", str(value)]
    if args.no_rider:
        ride_argv.append("--no-rider")
    args.ride_args = ride.parse_args(ride_argv)
    if args.ride_args.physics != "physical":
        parser.error(
            'bike-settle requires physics_mode="physical" — set it in --physics-config '
            "or pass --physics physical")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point; returns the process exit code."""
    args = parse_args(argv)
    ride_args = args.ride_args
    try:
        rider = ride.resolve_rider(ride_args)
        track = ride.resolve_track(ride_args.track, seed=ride_args.seed,
                                   length_m=ride_args.length)
    except (TrackFileError, ValueError, FileNotFoundError) as exc:
        print(f"{PREFIX} {exc}", file=sys.stderr)
        return 2

    from bike_sim.sim.ride.physical_session import build_physical_simulation
    print(f"{PREFIX} {track.name}: settling "
          f"(budget {args.max_steps} steps, tolerance {args.tolerance:g}) -> {args.out}",
          flush=True)
    started = time.perf_counter()
    try:
        sim = build_physical_simulation(
            track, ride_args, rider,
            equilibrium_max_steps=args.max_steps,
            equilibrium_tolerance=args.tolerance)
    except (RuntimeError, ValueError) as exc:
        print(f"{PREFIX} settle failed: {exc}", file=sys.stderr)
        return 3
    wall_s = time.perf_counter() - started

    equilibrium = sim.equilibrium
    out = Path(args.out)
    try:
        from bike_sim.sim.ride.equilibrium_cache import save_state_file
        save_state_file(out, sim.physical, sim.data.qpos,
                        equilibrium["steps"], equilibrium["residual_qacc"])
    except (OSError, ValueError) as exc:
        print(f"{PREFIX} could not write {out}: {exc}", file=sys.stderr)
        return 4
    origin = "auto-cache hit" if equilibrium.get("cache_hit") else "freshly settled"
    print(f"{PREFIX} {track.name}: {origin}, steps={equilibrium['steps']} "
          f"residual={equilibrium['residual_qacc']:.6g} wall={wall_s:.1f}s", flush=True)
    print(f"{PREFIX} fork {equilibrium['fork_travel_mm']:.1f} mm, "
          f"shock {equilibrium['shock_stroke_mm']:.1f} mm, "
          f"rear {equilibrium['rear_travel_mm']:.1f} mm", flush=True)
    print(f"{PREFIX} wrote {out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
