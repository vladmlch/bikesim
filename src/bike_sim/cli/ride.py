"""
CLI Entrypoint for Ride Mode: ``bike-ride``.

Rides a track -- a built-in preset or a TOML track file -- either interactively in the
MuJoCo viewer or headlessly with telemetry, summary metrics and plots written to disk.
Also previews a track's profile without simulating, and dumps any preset as a track file
to edit.

Geometry has one source of truth, the track (preset or file); the command line carries
only run-level knobs: seed and length overrides for generated roads, speed, sag target,
output directory, decimation, and the rider (variant, mass, height, inseam, legs).
"""

import argparse
from pathlib import Path
import sys
from typing import List, NamedTuple, Optional, Sequence

from bike_sim.physics.drivetrain import ASSIST_ORDER, DRIVE_MODES, DrivetrainSpecs
from bike_sim.geometry.specs import BikeSpecs
from bike_sim.physics.rider import DEFAULT_RIDER_VARIANT, RIDER_VARIANTS, RiderSpecs
from bike_sim.physics.tyre import (
    DEFAULT_TYRE_MODEL,
    DEFAULT_TYRE_TIER,
    FRONT_TYRE,
    PRESSURE_MAX_BAR,
    PRESSURE_MIN_BAR,
    REAR_TYRE,
    TYRE_MODELS,
    TYRE_TIERS,
    TyreConfig,
)
from bike_sim.sim.ride.cruise import (
    DEFAULT_TARGET_SPEED_KMH,
    MAX_TARGET_SPEED_KMH,
    MIN_TARGET_SPEED_KMH,
)
from bike_sim.terrain.obstacles import Drop, Kicker
from bike_sim.terrain.trackfile import FILE_SUFFIX
from bike_sim.terrain import (
    DEFAULT_ROAD_PRESET,
    PRESETS,
    ROAD_LEVEL_SPECS,
    SURFACES,
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

# Obstacles that put the bike in the air. The seated rider is a rough-road model; a rider
# does not sit through a drop, so a seated run over these is flagged (docs/RIDE.md section 7).
FLIGHT_OBSTACLE_TYPES = (Drop, Kicker)


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
    parser.add_argument("--speed", type=float, default=None, metavar="KMH",
                        help=f"cruise target in km/h, {MIN_TARGET_SPEED_KMH:.0f}-{MAX_TARGET_SPEED_KMH:.0f} "
                             f"(default {DEFAULT_TARGET_SPEED_KMH:.0f}, or 16 for climb_steps)")
    parser.add_argument("--headless", action="store_true",
                        help="run without a viewer and write telemetry, summary and plots")
    parser.add_argument("--sag", type=float, default=None, metavar="PCT",
                        help="fit fork pressure and coil rate to this static sag (both ends) instead of "
                             "the shipped tune; headless only")
    parser.add_argument("--out", default=DEFAULT_OUT_DIR, metavar="DIR",
                        help=f"root directory for run artifacts (default {DEFAULT_OUT_DIR})")
    parser.add_argument("--decimate", type=int, default=1, metavar="N",
                        help="store every N-th step in telemetry.csv (default 1 = every step)")
    parser.add_argument("--rider", choices=RIDER_VARIANTS, default=None,
                        help=f"rider model: seated biodynamic rider (default), the lumped standing rider, "
                             f"or none (default {DEFAULT_RIDER_VARIANT})")
    parser.add_argument("--rider-mass", type=float, default=80.0, metavar="KG",
                        help="rider mass including helmet and kit (default 80)")
    parser.add_argument("--rider-height", type=float, default=1.80, metavar="M",
                        help="rider stature; sets segment lengths and, for the seated rider, the saddle "
                             "height via the inseam (default 1.80)")
    parser.add_argument("--rider-inseam", type=float, default=None, metavar="M",
                        help="rider inseam (crotch height); default 0.47 x height")
    parser.add_argument("--no-rider", action="store_true", help="alias for --rider none")
    parser.add_argument(
        "--legs", choices=("articulated", "rigid"), default=None,
        help="seated rider's legs: articulated hip/knee/ankle chains welded to the pedals, "
             "or rigid slide-mounted masses (default: articulated when the crankset turns "
             "-- pedal/pedelec or --visual-pedalling --, rigid otherwise)",
    )
    parser.add_argument("--preview", action="store_true",
                        help="render the track profile with effective pothole drops and exit")
    parser.add_argument("--dump-track", metavar="NAME", default=None,
                        help="print a preset as a TOML track file and exit")
    parser.add_argument("--list-tracks", action="store_true", help="list the built-in presets and exit")
    parser.add_argument("--no-plots", action="store_true", help="headless: skip the PNG figures")
    parser.add_argument("--html", metavar="DIR", default=None,
                        help="render an interactive ride.html from DIR/preview.csv and exit "
                             "(needs plotly: uv run --with plotly)")
    parser.add_argument(
        "--tyre-model", choices=TYRE_MODELS, default=None,
        help=f"wheel contact model: sphere or pneumatic (default {DEFAULT_TYRE_MODEL}, or pneumatic for climb_steps)",
    )
    parser.add_argument(
        "--tyre-tier", choices=TYRE_TIERS, default=None,
        help=f"pneumatic brush fidelity (default {DEFAULT_TYRE_TIER})",
    )
    parser.add_argument(
        "--tyre-pressure", type=_parse_tyre_pressures, default=None,
        metavar="FRONT/REAR", help="front/rear tyre pressure in bar (default 1.5/1.7)",
    )
    parser.add_argument(
        "--drive-mode", choices=DRIVE_MODES, default=None,
        help="propulsion: motor (ideal wheel torque, the baseline), pedal (rider cranks), "
             "pedelec (rider plus mid-drive) (default motor, or pedelec for climb_steps)",
    )
    parser.add_argument(
        "--assist", choices=ASSIST_ORDER, default=None,
        help="mid-drive assist level for --drive-mode pedelec (default tour, or turbo for climb_steps); E cycles it in the viewer",
    )
    parser.add_argument(
        "--gearing", type=_parse_gearing, default=None, metavar="CHAINRINGxCOG",
        help="single gear for the pedalled drivetrain (default 32x14, which is 82 rpm at 25 km/h)",
    )
    parser.add_argument(
        "--ripple-depth", type=float, default=None, metavar="D",
        help="depth of the crank torque pulse in [0, 1] (default 0.85); 0 is a smooth crank "
             "and reproduces the motor baseline",
    )
    parser.add_argument(
        "--crank-phase", type=float, default=None, metavar="DEG",
        help="crank angle at the start of the run (default 0, the built 3/9 o'clock pose). "
             "Deliberately not tied to --seed: the phase a jump is met in is its own variable",
    )
    parser.add_argument(
        "--visual-pedalling", action="store_true",
        help="motor mode only: build the turning crankset and let the chain equality spin it "
             "off the driven wheel, so the articulated legs visibly pedal while the ideal "
             "wheel actuator does the driving",
    )
    parser.add_argument(
        "--surface", choices=tuple(SURFACES), default=None,
        help="override the track surface friction preset (default: track surface)",
    )
    parser.add_argument("--physics", choices=("legacy","physical"), default=None)
    parser.add_argument("--drive", choices=("coast","ideal_speed_control","crank_effort","articulated_effort"), default=None)
    parser.add_argument("--physics-config", default=None, metavar="PATH")
    parser.add_argument("--initial-speed", type=float, default=None, metavar="KMH")
    parser.add_argument("--human-torque", type=float, default=None, metavar="NM")
    parser.add_argument("--assist-gain", type=float, default=None)
    parser.add_argument("--timestep", type=float, default=None, metavar="S")
    parser.add_argument("--duration", type=float, default=None, metavar="S", help="physical run duration; permits stationary rigs")
    args = parser.parse_args(argv)
    from bike_sim.physics.resolution import load_physics_config
    from math import isfinite, radians
    overrides={"physics_mode":args.physics,"drive_mode":args.drive,"timestep_s":args.timestep}
    if args.initial_speed is not None:
        overrides["initial_speed_mps"]=args.initial_speed/3.6
    drive_values={}
    if args.human_torque is not None: drive_values["human_torque_nm"]=args.human_torque
    if args.assist_gain is not None: drive_values["assist"]={"gain":args.assist_gain}
    if args.crank_phase is not None: drive_values["crank_phase_rad"]=radians(args.crank_phase)
    if drive_values: overrides["drive"]=drive_values
    try:
        args.resolved_physics=load_physics_config(args.physics_config,overrides)
    except (ValueError,OSError) as exc:
        parser.error(str(exc))
    args.physics=args.resolved_physics.physics_mode
    args.drive=args.resolved_physics.drive_mode
    if args.duration is not None and (not isfinite(args.duration) or args.duration<=0):
        parser.error("--duration must be finite and positive")
    if args.physics=="physical":
        if args.speed is not None and args.drive!="ideal_speed_control":
            parser.error("--speed is only an ideal controller target; use --initial-speed for coast/effort")
        if args.drive_mode not in (None,"motor") or args.assist is not None or args.visual_pedalling:
            parser.error("legacy drive/assist/visual flags cannot select physical drivetrain settings; use --drive and TOML")
        if (args.tyre_model not in (None,"sphere") or args.surface is not None
                or args.tyre_tier is not None or args.tyre_pressure is not None):
            parser.error("physical tire backend and friction must be selected in --physics-config")
        if args.gearing is not None or args.ripple_depth is not None:
            parser.error("physical gearing and torque_ripple are configured in TOML")
        if args.legs is not None:
            parser.error("use --rider articulated_planar instead of legacy --legs")
        if args.drive=="ideal_speed_control" and args.speed is None: args.speed=DEFAULT_TARGET_SPEED_KMH
        args.tyre_model="sphere"
        args.drive_mode="motor"
        args.assist="tour"
        if args.rider is None and not args.no_rider:
            args.rider="articulated_planar" if args.drive=="articulated_effort" else "lumped"
        if args.drive=="articulated_effort" and (args.no_rider or args.rider!="articulated_planar"):
            parser.error("articulated_effort requires --rider articulated_planar")
        if args.headless and args.drive=="coast" and args.resolved_physics.initial_speed_mps==0 and args.duration is None:
            parser.error("a stationary physical coast run needs --duration or nonzero --initial-speed")
    else:
        if any(v is not None for v in (args.initial_speed,args.human_torque,args.assist_gain,args.duration)):
            parser.error("initial-speed, human-torque, assist-gain and duration require --physics physical")
        if args.track == "climb_steps":
            if args.speed is None:
                args.speed = 16.0
            if args.tyre_model is None:
                args.tyre_model = "pneumatic"
            if args.drive_mode is None:
                args.drive_mode = "pedelec"
            if args.assist is None:
                args.assist = "turbo"
        else:
            if args.speed is None:
                args.speed = DEFAULT_TARGET_SPEED_KMH
            if args.tyre_model is None:
                args.tyre_model = DEFAULT_TYRE_MODEL
            if args.drive_mode is None:
                args.drive_mode = "motor"
            if args.assist is None:
                args.assist = "tour"
    if args.tyre_tier is None: args.tyre_tier=DEFAULT_TYRE_TIER
    if args.tyre_pressure is None: args.tyre_pressure=(FRONT_TYRE.pressure_bar,REAR_TYRE.pressure_bar)
    if args.crank_phase is None: args.crank_phase=0.
    return args


def _parse_tyre_pressures(value: str) -> tuple[float, float]:
    """Parses the CLI's ``FRONT/REAR`` bar pair and enforces the live pressure limits."""
    parts = value.split("/")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("--tyre-pressure must be FRONT/REAR in bar")
    try:
        front, rear = (float(part) for part in parts)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("--tyre-pressure values must be numbers") from exc
    if not all(PRESSURE_MIN_BAR <= pressure <= PRESSURE_MAX_BAR for pressure in (front, rear)):
        raise argparse.ArgumentTypeError(
            f"--tyre-pressure values must be in [{PRESSURE_MIN_BAR:.1f}, {PRESSURE_MAX_BAR:.1f}] bar"
        )
    return front, rear


def _parse_gearing(value: str) -> tuple:
    """Parses the CLI's ``CHAINRINGxCOG`` tooth pair."""
    parts = value.lower().split("x")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("--gearing must be CHAINRINGxCOG, e.g. 32x14")
    try:
        chainring, cog = (int(part) for part in parts)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("--gearing teeth must be whole numbers") from exc
    return chainring, cog


def _drivetrain_config(args: argparse.Namespace) -> DrivetrainSpecs:
    """
    Builds the drivetrain specs from parsed command-line values.

    Raises:
        SystemExit: Via argparse, if a value is outside the physical range.
    """
    kwargs = {"crank_phase_deg": float(args.crank_phase)}
    if args.gearing is not None:
        kwargs["chainring_teeth"], kwargs["cog_teeth"] = args.gearing
        kwargs["auto_shift"] = False
    if args.ripple_depth is not None:
        kwargs["ripple_depth"] = float(args.ripple_depth)
    try:
        return DrivetrainSpecs(**kwargs)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def _tyre_config(args: argparse.Namespace) -> TyreConfig:
    """Builds the selected wheel model from parsed command-line values."""
    front_bar, rear_bar = args.tyre_pressure
    return TyreConfig(
        model=args.tyre_model,
        tier=args.tyre_tier,
        front=FRONT_TYRE.with_pressure(front_bar),
        rear=REAR_TYRE.with_pressure(rear_bar),
        surface=args.surface,
    )


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
    if _is_file_argument(name_or_path):
        path = Path(name_or_path)
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


def _is_file_argument(name_or_path: str) -> bool:
    """A preset name is a preset even if a file of that name happens to exist in cwd."""
    if name_or_path in PRESETS:
        return False
    return name_or_path.endswith(FILE_SUFFIX) or Path(name_or_path).is_file()


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
    if _is_file_argument(name_or_path) and path.is_file():
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


def run_dir_name(
    track: TrackSpec,
    speed_kmh: float,
    seed: Optional[int],
    tyre: Optional[TyreConfig] = None,
    *, drive_settings: Optional[dict] = None,
) -> str:
    """Builds a unique artifact directory, adding tyre settings for pneumatic runs."""
    name = f"{track.name}_{speed_kmh:g}"
    if seed is not None:
        name = f"{name}_s{seed}"
    if tyre is not None and tyre.pneumatic:
        surface = tyre.surface or track.surface
        name = (
            f"{name}_pneumatic-{tyre.tier}_{tyre.front.pressure_bar:.2f}-"
            f"{tyre.rear.pressure_bar:.2f}bar_{surface}"
        )
    if drive_settings is not None:
        import hashlib
        import json
        encoded = json.dumps(drive_settings, sort_keys=True, separators=(",", ":"), allow_nan=False)
        digest = hashlib.sha256(encoded.encode()).hexdigest()[:16]
        name += "_" + str(drive_settings["mode"]) + "_" + digest
    return name


# --------------------------------------------------------------------------------------
# Rider resolution
# --------------------------------------------------------------------------------------


class LegConfig(NamedTuple):
    """
    The resolved leg options for a run.

    Attributes:
        legs: ``"articulated"`` or ``"rigid"`` -- which legs the seated rider gets.
        visual_pedalling: Whether the crankset is built to spin in ``motor`` mode so the
            legs visibly pedal while the wheel actuator drives.
    """

    legs: str
    visual_pedalling: bool


def resolve_leg_config(args: argparse.Namespace) -> LegConfig:
    """
    Resolves ``--legs`` and ``--visual-pedalling`` against the drive mode.

    The legs default follows the crankset: ``pedal`` and ``pedelec`` turn it, so the
    rider's legs articulate and pump; ``motor`` leaves them rigid on their slides unless
    ``--visual-pedalling`` asks for the visual stroke anyway. The result lands on both
    `RiderSpecs.legs` -- which `seated_pose` reads -- and the `RideSimulation` kwargs.

    Args:
        args: Parsed command line.

    Raises:
        ValueError: If ``--visual-pedalling`` is set in a pedalled mode (redundant --
            the cranks turn anyway), is combined with ``--legs rigid`` (nothing would
            follow the pedals), or if ``--legs articulated`` is asked for in ``motor``
            mode without it, which would leave the feet with no pedal bodies to weld to.
    """
    if getattr(args,"physics","legacy")=="physical":
        return LegConfig(legs="rigid",visual_pedalling=False)
    visual = bool(args.visual_pedalling)
    if visual and args.drive_mode != "motor":
        raise ValueError(
            f"--visual-pedalling only applies to --drive-mode motor; "
            f"{args.drive_mode} already turns the crankset"
        )
    if visual and args.legs == "rigid":
        raise ValueError("--visual-pedalling needs the articulated legs; drop --legs rigid")
    if args.legs == "articulated" and args.drive_mode == "motor" and not visual:
        raise ValueError(
            "--legs articulated needs the crankset to turn; add --visual-pedalling "
            "or use --drive-mode pedal/pedelec"
        )
    legs = args.legs or ("articulated" if args.drive_mode != "motor" or visual else "rigid")
    return LegConfig(legs=legs, visual_pedalling=visual)


def resolve_rider(args: argparse.Namespace) -> RiderSpecs:
    """
    Turns the rider arguments into a `RiderSpecs`.

    Args:
        args: Parsed command line.

    Raises:
        ValueError: On contradictory or out-of-range rider arguments.
    """
    if args.no_rider and args.rider not in (None, "none"):
        raise ValueError(f"--no-rider contradicts --rider {args.rider}")
    variant = "none" if args.no_rider else (args.rider or DEFAULT_RIDER_VARIANT)
    return RiderSpecs(
        variant=variant,
        legs=resolve_leg_config(args).legs,
        mass_kg=float(args.rider_mass),
        height_m=float(args.rider_height),
        inseam_m=float(args.rider_inseam) if args.rider_inseam is not None else None,
    )


def flight_obstacles(track: TrackSpec) -> List[str]:
    """Labels of the track's drops and kickers, which a seated rider is not a model for."""
    return [o.label for o in track.sorted_obstacles if isinstance(o, FLIGHT_OBSTACLE_TYPES)]


def describe_rider(rider: RiderSpecs, specs: BikeSpecs) -> str:
    """One line on the rider for the run log."""
    if rider.variant == "none":
        return "rider: none (bike alone)"
    if rider.variant == "lumped":
        return f"rider: lumped, {rider.mass_kg:g} kg standing attack pose (rigid in frame)"
    if rider.variant == "articulated_planar":
        return f"rider: articulated_planar, {rider.mass_kg:g} kg, independent planar root"
    pose = rider.seated_pose(specs)
    return (
        f"rider: seated, {rider.mass_kg:g} kg, {rider.height_m:.2f} m (inseam {rider.inseam:.3f} m), "
        f"{rider.legs} legs; "
        f"saddle {pose.saddle.height_m:.3f} m (top +{pose.saddle.top_z_m * 1000 - 690:.0f} mm vs photo), "
        f"torso {pose.torso_lean_deg:.0f} deg from vertical, knee at BDC {pose.knee_flexion_bdc_deg:.0f} deg; "
        f"static split saddle {100 * rider.saddle_share:.0f} / pedals {100 * rider.pedal_share:.0f} / "
        f"bar {100 * rider.bar_share:.0f} %"
    )


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


def _html_report(run_dir: Path) -> int:
    from bike_sim.viz.ride_plots import load_ride_csv, plot_physical_ride_html

    csv_path = run_dir / "preview.csv"
    if not csv_path.is_file():
        print(f"{PREFIX} no preview.csv in {run_dir}", file=sys.stderr)
        return 1
    try:
        path = plot_physical_ride_html(load_ride_csv(csv_path), run_dir)
    except ImportError:
        print(f"{PREFIX} --html needs plotly: uv run --with plotly bike-ride --html {run_dir}",
              file=sys.stderr)
        return 1
    print(f"{PREFIX} wrote {path}")
    return 0


def _fit_sag(target_pct: float, specs: BikeSpecs, rider: RiderSpecs):
    """Fits both ends to ``target_pct`` against the model's own centre of mass, rider included."""
    from bike_sim.physics.air_spring import AirSpringSpecs, ForkAirSpring
    from bike_sim.physics.coil_shock import CoilShock, CoilShockSpecs
    from bike_sim.physics.damper import BikeSuspensionSystem
    from bike_sim.physics.tuning import compute_suspension_tuning_for_sag
    from bike_sim.sim.controllers import SuspensionController

    fit = compute_suspension_tuning_for_sag(
        specs=specs, rider_specs=rider, target_front_sag_pct=target_pct, target_rear_sag_pct=target_pct,
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
        suspension_system=BikeSuspensionSystem(legacy_behavior=True),
    )
    coil = CoilShock(CoilShockSpecs(rate_n_m=rate))
    print(f"{PREFIX} --sag {target_pct:g}%: fork {psi:.1f} psi ({tokens} tokens) instead of "
          f"{specs.fork_initial_psi:.1f}; coil {rate:,.0f} N/m instead of {specs.shock_stiffness:,.0f}")
    return controller, coil, {"sag_target_pct": target_pct, "fork_psi": psi, "coil_rate_n_m": rate}


def _headless(
    track: TrackSpec,
    args: argparse.Namespace,
    seed: Optional[int],
    rider: RiderSpecs,
    tyre: TyreConfig,
) -> int:
    if args.physics == "physical":
        from bike_sim.sim.ride.physical_session import run_physical_headless
        return run_physical_headless(track,args,seed,rider)
    from bike_sim.sim.ride.metrics import RAMP_EXCLUSION_M, summarize_ride
    from bike_sim.sim.ride.recorder import RideRecorder
    from bike_sim.sim.ride_sim import RideSimulation

    specs = BikeSpecs()
    controller = coil = None
    extras = {"rider_mass_kg": float(rider.total_rider_mass)}
    if seed is not None:
        extras["seed"] = float(seed)
    if args.sag is not None:
        controller, coil, extras_sag = _fit_sag(args.sag, specs, rider)
        extras.update(extras_sag)

    from dataclasses import asdict
    drivetrain = _drivetrain_config(args)
    leg_config = resolve_leg_config(args)
    drive_settings = {"mode": args.drive_mode, "assist": args.assist if args.drive_mode == "pedelec" else "off",
                      "drivetrain": asdict(drivetrain), "legs": leg_config.legs,
                      "visual_pedalling": leg_config.visual_pedalling}
    drive_settings["physics"] = asdict(args.resolved_physics)
    from bike_sim.physics.model_config import SimulationPhysicsConfig
    default_drive = (args.drive_mode == "motor" and drivetrain == DrivetrainSpecs()
                     and not leg_config.visual_pedalling and leg_config.legs == "rigid"
                     and args.resolved_physics == SimulationPhysicsConfig())
    run_dir = Path(args.out) / run_dir_name(track, args.speed, seed, tyre,
        drive_settings=None if default_drive else drive_settings)
    print(f"{PREFIX} {track.name}: {track.length_m:.0f} m, {len(track.markers)} marked obstacles, "
          f"target {args.speed:g} km/h -> {run_dir}")
    print(f"{PREFIX} {describe_rider(rider, specs)}")
    tyre_surface = tyre.surface or track.surface
    print(
        f"{PREFIX} tyres: {tyre.model}/{tyre.tier}, pressure "
        f"{tyre.front.pressure_bar:.2f}/{tyre.rear.pressure_bar:.2f} bar, surface {tyre_surface}"
    )

    if args.drive_mode != "motor":
        cadence = drivetrain.cadence_rpm_at(args.speed / 3.6, specs.rear_wheel_radius / 1000.0)
        assist = args.assist if args.drive_mode == "pedelec" else "off"
        print(f"{PREFIX} drive: {args.drive_mode}, assist {assist}, "
              f"{drivetrain.chainring_teeth}x{drivetrain.cog_teeth} "
              f"({cadence:.0f} rpm at {args.speed:g} km/h), ripple depth {drivetrain.ripple_depth:g}")
        if not 60.0 <= cadence <= 110.0:
            print(f"{PREFIX} warning: {cadence:.0f} rpm is outside the 60-110 rpm band a rider "
                  f"actually pedals in; change --gearing or --speed", file=sys.stderr)
    sim = RideSimulation(
        track=track, specs=specs, target_speed_kmh=args.speed, rider=rider,
        controller=controller, coil_shock=coil, tyre=tyre,
        drive_mode=args.drive_mode, assist=args.assist, drivetrain=drivetrain,
        legs=leg_config.legs, visual_pedalling=leg_config.visual_pedalling,
        physics_config=args.resolved_physics,
    )
    eq = sim.equilibrium
    print(f"{PREFIX} start equilibrium: fork {eq['fork_travel_mm']:.1f} mm "
          f"({100 * eq['fork_travel_mm'] / specs.fork_travel:.1f} %), shock {eq['shock_stroke_mm']:.1f} mm "
          f"({100 * eq['shock_stroke_mm'] / specs.shock_stroke:.1f} %)")
    if "rider_saddle_share" in eq:
        print(f"{PREFIX} solved rider load split: saddle {100 * eq['rider_saddle_share']:.1f} % / "
              f"pedals {100 * eq['rider_pedals_share']:.1f} % / bar {100 * eq['rider_bar_share']:.1f} % "
              f"({eq['rider_saddle_load_n']:.0f} / {eq['rider_pedals_load_n']:.0f} / {eq['rider_bar_load_n']:.0f} N)")
    extras["start_fork_travel_mm"] = float(eq["fork_travel_mm"])
    extras["start_shock_stroke_mm"] = float(eq["shock_stroke_mm"])
    extras.update({
        "drive_mode": args.drive_mode,
        "assist": args.assist if args.drive_mode == "pedelec" else "off",
        "legs": leg_config.legs,
        "visual_pedalling": leg_config.visual_pedalling,
        "gear_ratio": float(drivetrain.gear_ratio),
        "ripple_depth": float(drivetrain.ripple_depth),
        "tyre_model": tyre.model,
        "tyre_tier": tyre.tier,
        "tyre_pressure_front_bar": tyre.front.pressure_bar,
        "tyre_pressure_rear_bar": tyre.rear.pressure_bar,
        "surface": tyre_surface,
    })

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
        rider_variant=rider.variant, rider_mass_kg=rider.mass_kg,
        tyre_rim_events=recorder.tyre_rim_events,
        tyre_rim_starts=recorder.tyre_rim_starts,
        tyre_times_s=recorder.tyre_times_s,
    )

    written: List[Path] = [recorder.write_csv(run_dir / "telemetry.csv"), summary.write_json(run_dir / "summary.json")]
    if not args.no_plots:
        from bike_sim.viz.ride_plots import plot_ride, plot_track_profile, plot_tyres

        window = channels["x_m"] >= sim.start_x_m + RAMP_EXCLUSION_M
        written += plot_ride(channels, recorder.sample_interval_s, track, specs.fork_travel,
                             specs.shock_stroke, run_dir, window=window)
        if sim.tyre_applier is not None:
            written.append(plot_tyres(channels, track, run_dir / "tyres.png"))
        written.append(plot_track_profile(track, run_dir / "profile.png"))

    print()
    print(summary.format_table())
    print()
    for path in written:
        print(f"{PREFIX} wrote {path}")
    return 0


def _interactive(track: TrackSpec, args: argparse.Namespace, rider: RiderSpecs, tyre: TyreConfig) -> int:
    if args.physics == "physical":
        from bike_sim.sim.playground import ensure_macos_mjpython
        ensure_macos_mjpython()
        from bike_sim.sim.ride.physical_session import build_physical_simulation
        from bike_sim.sim.ride.viewer import run_physical_viewer
        return run_physical_viewer(build_physical_simulation(track,args,rider), out_root=args.out)
    if args.sag is not None:
        print(f"{PREFIX} --sag applies to headless runs only; the viewer uses the shipped tune "
              f"(P cycles damper presets, -/= change pressure)", file=sys.stderr)
    from bike_sim.sim.ride.viewer import run_interactive_ride

    print(f"{PREFIX} {describe_rider(rider, BikeSpecs())}")
    print(
        f"{PREFIX} tyres: {tyre.model}/{tyre.tier}, pressure "
        f"{tyre.front.pressure_bar:.2f}/{tyre.rear.pressure_bar:.2f} bar, "
        f"surface {tyre.surface or track.surface}"
    )
    leg_config = resolve_leg_config(args)
    run_interactive_ride(
        track=track, target_speed_kmh=args.speed, rider=rider, tyre=tyre,
        drive_mode=args.drive_mode, assist=args.assist, drivetrain=_drivetrain_config(args),
        legs=leg_config.legs, visual_pedalling=leg_config.visual_pedalling,
    )
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
    if args.html is not None:
        return _html_report(Path(args.html))
    if args.decimate < 1:
        print(f"{PREFIX} --decimate must be >= 1", file=sys.stderr)
        return 2
    if args.speed is not None and not MIN_TARGET_SPEED_KMH <= args.speed <= MAX_TARGET_SPEED_KMH:
        print(f"{PREFIX} --speed must be within {MIN_TARGET_SPEED_KMH:.0f}-{MAX_TARGET_SPEED_KMH:.0f} km/h",
              file=sys.stderr)
        return 2
    try:
        rider = resolve_rider(args)
    except ValueError as exc:
        print(f"{PREFIX} {exc}", file=sys.stderr)
        return 2
    if args.sag is not None and not rider.present:
        print(f"{PREFIX} --sag fits the springs for the rider's weight; it cannot be combined with --no-rider "
              f"or --rider none", file=sys.stderr)
        return 2

    try:
        track = resolve_track(args.track, seed=args.seed, length_m=args.length)
    except (TrackFileError, ValueError, FileNotFoundError) as exc:
        print(f"{PREFIX} {exc}", file=sys.stderr)
        return 1

    if track.length_m > LONG_TRACK_WARNING_M:
        print(f"{PREFIX} note: a {track.length_m:.0f} m track is a large heightfield for the interactive "
              f"viewer; headless runs are unaffected", file=sys.stderr)
    if rider.variant == "seated":
        flights = flight_obstacles(track)
        if flights:
            print(f"{PREFIX} warning: '{track.name}' has {len(flights)} drop/kicker obstacle(s) "
                  f"({', '.join(flights)}). The seated rider is a rough-road model: a rider does not sit "
                  f"through a drop, and in flight the pitch is held by the external virtual-rider moment "
                  f"(docs/RIDE.md section 12). Use --rider lumped for the aggressive presets.", file=sys.stderr)
    if rider.variant == "seated":
        try:
            rider.seated_pose(BikeSpecs())
        except ValueError as exc:
            print(f"{PREFIX} {exc}", file=sys.stderr)
            return 2

    seed = track_seed(args.track, args.seed)
    try:
        tyre = _tyre_config(args)
    except ValueError as exc:
        print(f"{PREFIX} {exc}", file=sys.stderr)
        return 2
    if args.preview:
        return _preview(track, Path(args.out), seed)
    if args.headless:
        return _headless(track, args, seed, rider, tyre)
    return _interactive(track, args, rider, tyre)


if __name__ == "__main__":
    sys.exit(main())
