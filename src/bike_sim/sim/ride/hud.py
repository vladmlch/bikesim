"""
Ride-Mode Terminal HUD.

The single-line live readout of a rolling run, and the interactive key map's help screen.
Separate from `sim/hud.py`, which belongs to the test stand: the stand reports a commanded
travel against a static load and the two HUDs share no channel, so folding them together
would produce one formatter with two disjoint halves and a mode flag.

Everything the line needs is derived here from the simulation's own state, rather than
assembled by the viewer loop and passed in as a dict, so the derivation is covered by tests
while the viewer stays a loop that prints a string.

**Pitch is reported with its direction spelled out.** `qpos[root_pitch]` is positive
nose-down -- a positive rotation about world +Y tips the bike's forward axis toward the
ground -- and a signed number in degrees is ambiguous to a reader who does not know that.
"""

import sys
from math import degrees
from typing import TYPE_CHECKING, Optional, Tuple

import mujoco

from bike_sim.sim.ride.cruise import KMH_PER_MPS, MAX_TARGET_SPEED_KMH, MIN_TARGET_SPEED_KMH
from bike_sim.sim.ride.wheels import resolve_wheel_spin

if TYPE_CHECKING:
    # Type-only: importing the orchestrator at runtime would make the ride package import
    # itself through `ride_sim`.
    from bike_sim.sim.ride_sim import RideSimulation

PITCH_NOSE_DOWN = "nose-dn"
PITCH_NOSE_UP = "nose-up"
PITCH_LEVEL = "level"


def pitch_label(pitch_rad: float) -> str:
    """
    Names the direction a pitch angle leans.

    Args:
        pitch_rad: Chassis pitch in radians, positive nose-down.

    Returns:
        `nose-dn`, `nose-up`, or `level` at exactly zero.
    """
    if pitch_rad > 0.0:
        return PITCH_NOSE_DOWN
    if pitch_rad < 0.0:
        return PITCH_NOSE_UP
    return PITCH_LEVEL


def nearest_obstacle(track, position_m: float) -> Optional[Tuple[str, float]]:
    """
    Finds the track feature closest to a position along the track.

    Args:
        track: Track being ridden.
        position_m: Current track position, in metres.

    Returns:
        Tuple of (label, signed distance in metres, positive ahead of the bike), or None if
        the track has no obstacles at all.
    """
    markers = track.markers
    if not markers:
        return None
    start_m, label = min(markers, key=lambda m: abs(m[0] - position_m))
    return label, float(start_m - position_m)


class RideHUD:
    """
    Formats the ride-mode HUD line and owns the interactive help text.

    Wheel-spin handles are resolved once at construction, because the reported wheel power is
    a torque times an angular velocity and neither is available as a simulation property.
    """

    def __init__(self, model: mujoco.MjModel) -> None:
        """
        Args:
            model: Compiled ride-mode model, used to resolve the wheel-spin coordinates.

        Raises:
            ValueError: If a wheel-spin joint or contact sphere is missing.
        """
        self.rear_wheel = resolve_wheel_spin(model, "rear_wheel_spin", "geom_rear_contact")

    def line(
        self,
        sim: "RideSimulation",
        braking: bool = False,
        brake_strength: float = 0.0,
    ) -> str:
        """
        Builds the single-line HUD for the simulation's current state.

        Args:
            sim: Simulation to read. Nothing is written.
            braking: Whether the brake toggle is currently engaged.
            brake_strength: Brake lever position the toggle applies, in [0, 1].

        Returns:
            One line of text, without a trailing newline.
        """
        obstacle = nearest_obstacle(sim.track, sim.position_m)
        obstacle_str = "--" if obstacle is None else f"{obstacle[0]} {obstacle[1]:+.1f}m"

        fork_pct = 100.0 * sim.fork_travel_mm / float(sim.specs.fork_travel)
        shock_pct = 100.0 * sim.shock_stroke_mm / float(sim.specs.shock_stroke)
        fork_shaft_mps = float(sim.data.qvel[sim.applier.fork_dofadr])
        shock_shaft_mps = float(sim.data.qvel[sim.applier.shock_dofadr])

        brake_str = (
            f"Brake:ON {brake_strength * 100.0:3.0f}%"
            if braking
            else f"Brake:off({brake_strength * 100.0:3.0f}%)"
        )
        grip_str = (
            f"{'F' if sim.contacts.front_in_contact else '-'}"
            f"{'R' if sim.contacts.rear_in_contact else '-'}"
        )
        power_w = sim.wheel_drive_torque_nm * self.rear_wheel.omega_radps(sim.data)
        command = sim.pedal_command
        if command is None:
            drive_str = f"Pwr:{power_w:+7.1f}W"
        else:
            assist = sim.drivetrain.assist_mode.upper()
            state = "FREEWHEEL" if command.freewheel else f"{command.cadence_rpm:5.1f}rpm"
            cutoff = " CUT" if command.cutoff_active else ""
            drive_str = (
                f"Drive:{assist}{cutoff} {state} "
                f"legs{command.rider_power_w:+6.1f}W motor{command.motor_power_w:+6.1f}W "
                f"| Pwr:{power_w:+7.1f}W"
            )
        tyre_str = ""
        if sim.tyre_applier is not None:
            front = sim.tyre_applier.front_outputs
            rear = sim.tyre_applier.rear_outputs
            rim_flash = (
                " | RIM STRIKE"
                if any(o.rim_strike_active or o.rim_event is not None for o in (front, rear))
                else ""
            )
            tyre_str = (
                f" | Tyre F/R {front.pressure_bar:.2f}/{rear.pressure_bar:.2f}bar "
                f"κ {front.slip_ratio:+.2f}/{rear.slip_ratio:+.2f}{rim_flash}"
            )

        return (
            f"[RIDE|{sim.track.name}] "
            f"x{sim.position_m:7.2f}m ({obstacle_str}) | "
            f"v {sim.speed_mps * KMH_PER_MPS:5.2f}/{sim.cruise.target_speed_kmh:4.1f}km/h | "
            f"Fork:{sim.fork_travel_mm:6.1f}mm {fork_pct:5.1f}% ({fork_shaft_mps:+6.3f}m/s) | "
            f"Shock:{sim.shock_stroke_mm:5.1f}mm {shock_pct:5.1f}% ({shock_shaft_mps:+6.3f}m/s) | "
            f"{brake_str} | Grip:{grip_str} | "
            f"Pitch:{degrees(sim.pitch_rad):+6.2f}deg {pitch_label(sim.pitch_rad)} | "
            f"{drive_str}"
            f"{tyre_str}"
        )

    def print_line(
        self,
        sim: "RideSimulation",
        braking: bool = False,
        brake_strength: float = 0.0,
    ) -> None:
        """
        Writes the HUD line over the current terminal line.

        Args:
            sim: Simulation to read.
            braking: Whether the brake toggle is currently engaged.
            brake_strength: Brake lever position the toggle applies, in [0, 1].
        """
        sys.stdout.write("\r" + self.line(sim, braking, brake_strength))
        sys.stdout.flush()

    @staticmethod
    def get_help_text() -> str:
        """Returns the interactive control help string for ride mode."""
        return f"""
================================================================================
              MUJOCO RIDE MODE - ROLLING TRACK CONTROLS
================================================================================
  --- SPEED, BRAKES & RUN ---
    W / S           : Cruise Target Speed + / - 1 km/h \
({MIN_TARGET_SPEED_KMH:.0f}-{MAX_TARGET_SPEED_KMH:.0f} km/h band)
    SPACE           : Brakes TOGGLE (a passive viewer delivers presses, not key state)
    , / .           : Brake Strength - / + 10 %
    R               : Restart the run from the solved static equilibrium

  --- PNEUMATIC AIR SPRING (FORK) ---
    [ / ]           : Bottomless Tokens DECREASE / INCREASE (Ramp-up)
    - / =           : Fork Air Pressure -/+ 2.0 PSI

  --- PNEUMATIC TYRES (pneumatic model only) ---
    N / M           : Front tyre pressure -/+ 0.05 bar
    ; / '           : Rear tyre pressure -/+ 0.05 bar

  --- ROCKSHOX CHARGER 3 DAMPER (FORK) ---
    J / H           : Fork High-Speed Compression (HSC) + / - (0 to 4 clicks)
    L / K           : Fork Low-Speed Compression (LSC) + / - (0 to 14 clicks)
    U / Y           : Fork Rebound (Reb) Slower / Faster (0 to 17 clicks)

  --- ROCKSHOX SUPER DELUXE DAMPER (REAR SHOCK) ---
    8 / 7           : Shock High-Speed Compression (HSC) + / - (0 to 4 clicks)
    0 / 9           : Shock Low-Speed Compression (LSC) + / - (0 to 14 clicks)
    6 / 5           : Shock Rebound (Reb) Slower / Faster (0 to 14 clicks)
    4 / 3           : Shock Hydraulic Bottom-Out (HBO) + / - (0 to 4 clicks)
    X               : Shock Threshold Lockout TOGGLE (Open / Firm Platform)
    P               : Factory Damper Preset CYCLE (Base / Plush / Firm / Climb)

  --- VISUALIZATION & VIEWPORT ---
    C / 1 / 2       : Camera Angle (Cycle / 2D Side View / 3D Isometric)
    G               : Toggle Pivot Markers (Yellow sphere debug indicators)
    T               : Toggle Live Telemetry Stream
    ? / /           : Show This Help Screen
    ESC             : Close Viewer & Exit Ride Mode

  The rider (none / lumped / seated) is chosen with `bike-ride --rider`, not a key.
================================================================================
"""

    @classmethod
    def print_help(cls) -> None:
        """Displays the ride-mode control help in the terminal."""
        print(cls.get_help_text())


__all__ = [
    "RideHUD",
    "nearest_obstacle",
    "pitch_label",
    "PITCH_NOSE_DOWN",
    "PITCH_NOSE_UP",
    "PITCH_LEVEL",
]
