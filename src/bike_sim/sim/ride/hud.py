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
from math import atan2, degrees
from typing import TYPE_CHECKING, Optional, Tuple

import mujoco
import numpy as np

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
        if getattr(sim,"physical",None) is not None:
            if sim.physical.interactive_preview:
                drive=sim.physical.drive.last
                factor=sim.physical.preview_real_time_factor
                rate='measuring' if factor is None else f'{factor:.2f}x'
                return (f"[PHYSICAL PREVIEW|{sim.physics_config.drive_mode}] t={sim.time_s:.3f}s "
                        f"RTF={rate} "
                        f"v={sim.speed_mps*KMH_PER_MPS:.2f}km/h "
                        f"Fn={sim.contacts.front_load_n:.1f}/{sim.contacts.rear_load_n:.1f}N "
                        f"cadence={drive.get('cadence_rpm',0.):.1f}rpm "
                        f"motor={drive.get('motor_shaft_power_w',0.):.1f}W "
                        f"model_valid={sim.physical.model_status.as_dict()['model_valid']} "
                        f"UNVALIDATED planar/experimental; energy audit: off")
            sample=sim.physical.sample
            if sample is None:
                return f"[PHYSICAL|{sim.physics_config.drive_mode}] initial condition"
            channels=sample.channels
            tires=channels["tires"];drive=channels["drive"];energy=channels["energy"]
            return (f"[PHYSICAL|{sim.physics_config.drive_mode}] t={sample.time_s:.4f}s "
                    f"v={sample.qvel[sim.root_x_dofadr]*3.6:.2f}km/h "
                    f"Fn={tires['front']['normal_load_n']:.1f}/{tires['rear']['normal_load_n']:.1f}N "
                    f"cadence={drive.get('cadence_rpm',0.):.1f}rpm "
                    f"motor={drive.get('motor_shaft_power_w',0.):.1f}W "
                    f"battery={drive.get('battery_energy_j',0.)/3600.:.2f}Wh "
                    f"energy residual={energy['residual_j']:.5f}J "
                    f"model_valid={channels.get('model_status',{}).get('model_valid','not_evaluated')} UNVALIDATED")
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

    def preview_log_row(self, sim: "RideSimulation") -> dict:
        """Build one CSV row of the physical preview's key state channels.

        Columns absent from this build (no rider contacts, no tire model) are
        emitted as empty strings, so every preview CSV shares one schema.
        """
        row = {"time_s": float(sim.time_s)}
        if getattr(sim, "physical", None) is None or not sim.physical.interactive_preview:
            return row
        drive = sim.physical.drive.last
        assist = sim.physical.drive.assist
        obstacle = nearest_obstacle(sim.track, sim.position_m)
        force_components = sim.force_accumulator.components
        shift_count = int(drive.get("shift_count") or 0)
        crank_rate = drive.get("crank_target_rate_rad_s")
        grade = (0.0 if sim.track.grade_profile is None
                 else 100.0 * sim.track.grade_profile.slope(sim.position_m))
        row.update({
            "x_m": float(sim.position_m),
            "rtf": sim.physical.preview_real_time_factor,
            "speed_kmh": float(sim.speed_mps) * KMH_PER_MPS,
            "pitch_deg": float(degrees(sim.pitch_rad)),
            "grade_pct": float(grade),
            "obstacle": "" if obstacle is None else obstacle[0],
            "obstacle_distance_m": "" if obstacle is None else float(obstacle[1]),
            "front_load_n": float(sim.contacts.front_load_n),
            "rear_load_n": float(sim.contacts.rear_load_n),
            "fork_travel_mm": float(sim.fork_travel_mm),
            "fork_velocity_mps": float(sim.data.qvel[sim.applier.fork_dofadr]),
            "fork_force_n": float(force_components.get(
                "fork_spring", np.zeros(sim.model.nv))[sim.applier.fork_dofadr]),
            "shock_stroke_mm": float(sim.shock_stroke_mm),
            "shock_velocity_mps": float(sim.data.qvel[sim.applier.shock_dofadr]),
            "shock_force_n": float(force_components.get(
                "shock_coil", np.zeros(sim.model.nv))[sim.applier.shock_dofadr]),
            "crank_phase_rad": float(sim.data.qpos[sim.physical.address("crank_spin")[0]]),
            "crank_target_rate_rpm": (
                "" if drive.get("crank_target_phase_rad") is None or crank_rate is None
                else float(crank_rate) * 60.0 / (2.0 * np.pi)),
            "cadence_rpm": float(drive.get("cadence_rpm", 0.0)),
            "required_cadence_rpm": float(drive.get("required_cadence_rpm", 0.0)),
            "rider_mode": drive.get("rider_mode", "unknown"),
            "coast_reason": drive.get("coasting_reason") or "",
            "gear_front_teeth": int(drive.get("gear_front_teeth", 0)),
            "gear_rear_teeth": int(drive.get("gear_rear_teeth", 0)),
            "shift_count": shift_count,
            "shift_torque_factor": float(drive.get("shift_torque_factor", 1.0)),
            "last_shift_direction": "" if not shift_count else drive.get("shift_direction", ""),
            "last_shift_from_teeth": "" if not shift_count else drive.get("shift_from_teeth", ""),
            "last_shift_to_teeth": "" if not shift_count else drive.get("gear_rear_teeth", ""),
            "last_shift_time_s": "" if not shift_count else float(drive.get("shift_time_s", 0.0)),
            "freehub_engaged": int(bool(drive.get("freehub_engaged"))),
            "freehub_torque_nm": float(drive.get("freehub_torque_nm", 0.0)),
            "human_sensor_nm": float(drive.get("human_sensor_nm", 0.0)),
            "human_command_nm": float(drive.get("human_command_nm", 0.0)),
            "assist_sensor_nm": float(drive.get("assist_sensor_nm", 0.0)),
            "motor_request_nm": float(drive.get("motor_request_nm", 0.0)),
            "motor_torque_nm": float(drive.get("motor_torque_nm", 0.0)),
            "motor_shaft_power_w": float(drive.get("motor_shaft_power_w", 0.0)),
            "motor_enabled": int(bool(drive.get("motor_enabled", False))),
            "assist_pedaling": int(bool(assist.pedaling)),
            "assist_age_s": float(assist.age),
            "assist_stall_s": float(drive.get("assist_stall_s", 0.0)),
        })

        rider_columns = {
            "rider_grip_enabled": "", "rider_grip_reachable": "",
            "rider_pedal_front": "", "rider_pedal_rear": "",
            "rider_ik_saturated": "",
            "rider_stance_front": "", "rider_stance_rear": "",
            "grip_gap_m": "", "front_pedal_load_n": "", "front_pedal_gap_m": "",
            "rear_pedal_load_n": "", "rear_pedal_gap_m": "", "saddle_load_n": "",
            "rider_root_pitch_deg": "", "rider_pelvis_pitch_deg": "",
            "rider_torso_pitch_deg": "", "rider_rel_pitch_deg": "",
            "rider_joints_saturated": "",
        }
        rider = sim.physical.rider_contacts
        if rider is not None:
            diagnostics = rider.diagnostics
            front = diagnostics.get("front_pedal", {})
            rear_pedal = diagnostics.get("rear_pedal", {})
            saddle = diagnostics.get("saddle", {})
            grip = diagnostics.get("grip", {})
            control = sim.physical.rider_control
            stance = {} if control is None else control.support_diagnostics.get("stance", {})
            ik_saturated = "" if control is None else ",".join(
                name for name, active in control.saturated_ik.items() if active)
            joints_saturated = "" if control is None else ",".join(
                name for name, terms in control.last_terms.items() if terms.get("saturated"))
            root_pitch = degrees(float(sim.data.qpos[sim.physical.address("rider_root_pitch")[0]]))
            body_pitch = {}
            for name in ("rider_pelvis", "rider_torso"):
                body_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, name)
                rotation = sim.data.xmat[body_id].reshape(3, 3)
                body_pitch[name] = degrees(atan2(-rotation[2, 0], rotation[0, 0]))
            rider_columns.update({
                "rider_grip_enabled": int(bool(grip.get("enabled"))),
                "rider_grip_reachable": int(bool(grip.get("reachable"))),
                "rider_pedal_front": int(bool(front.get("in_platform"))),
                "rider_pedal_rear": int(bool(rear_pedal.get("in_platform"))),
                "rider_ik_saturated": ik_saturated,
                "rider_stance_front": "" if control is None else int(bool(stance.get("front"))),
                "rider_stance_rear": "" if control is None else int(bool(stance.get("rear"))),
                "grip_gap_m": float(grip.get("hand_gap_m", 0.0)),
                "front_pedal_load_n": float(front.get("normal_load_n", 0.0)),
                "front_pedal_gap_m": float(front.get("gap_m", 0.0)),
                "rear_pedal_load_n": float(rear_pedal.get("normal_load_n", 0.0)),
                "rear_pedal_gap_m": float(rear_pedal.get("gap_m", 0.0)),
                "saddle_load_n": float(saddle.get("normal_load_n", 0.0)),
                "rider_root_pitch_deg": float(root_pitch),
                "rider_pelvis_pitch_deg": float(body_pitch["rider_pelvis"]),
                "rider_torso_pitch_deg": float(body_pitch["rider_torso"]),
                "rider_rel_pitch_deg": float(root_pitch - degrees(sim.pitch_rad)),
                "rider_joints_saturated": joints_saturated,
            })
        row.update(rider_columns)

        tire_columns = {
            "front_slip_mps": "", "rear_slip_mps": "", "front_mu": "", "rear_mu": "",
        }
        tire = sim.physical.tire
        if tire is not None:
            front_tire = tire.diagnostics.get("front", {})
            rear_tire = tire.diagnostics.get("rear", {})
            tire_columns.update({
                "front_slip_mps": float(front_tire.get("slip_mps", 0.0)),
                "rear_slip_mps": float(rear_tire.get("slip_mps", 0.0)),
                "front_mu": float(front_tire.get("friction_coefficient", 0.0)),
                "rear_mu": float(rear_tire.get("friction_coefficient", 0.0)),
            })
        row.update(tire_columns)
        return row

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
