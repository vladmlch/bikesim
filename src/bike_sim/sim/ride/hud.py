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
from rich.text import Text

from bike_sim.sim.ride.console import console, event, styled
from bike_sim.sim.ride.cruise import KMH_PER_MPS, MAX_TARGET_SPEED_KMH, MIN_TARGET_SPEED_KMH
from bike_sim.sim.ride.wheels import resolve_wheel_spin

if TYPE_CHECKING:
    # Type-only: importing the orchestrator at runtime would make the ride package import
    # itself through `ride_sim`.
    from bike_sim.sim.ride_sim import RideSimulation

PITCH_NOSE_DOWN = "nose-dn"
PITCH_NOSE_UP = "nose-up"
PITCH_LEVEL = "level"

# Dynamic-flag thresholds on the physical HUD: a wheel pressing with less than
# this is airborne for display purposes; a |kappa| past this is sliding.
_AIRBORNE_N = 5.0
_SLIP_WARN = 0.25
# Sentinel for "no legend printed yet": physical.generation may be 0, which a
# falsy default would treat as already seen.
_UNSET = object()

_FORCE_ARROWS = ("→", "↘", "↓", "↙", "←", "↖", "↑", "↗")


def _force_arrow(forward_n: float, vertical_n: float) -> str:
    """
    Picks the nearest of the eight compass arrows for a planar force.

    Args:
        forward_n: Forward (+x) component of the force, in newtons.
        vertical_n: Up (+z) component of the force, in newtons.

    Returns:
        One arrow; ``↓`` is weight pressing down, ``↑`` is a pull upward.
    """
    sector = round(degrees(atan2(-vertical_n, forward_n)) / 45.0) % 8
    return _FORCE_ARROWS[sector]


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
            return "".join(
                text for _, segments in self._physical_columns(sim) for text, _ in segments
            )
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

    def _physical_columns(self, sim: "RideSimulation") -> list:
        """Ordered (legend, [(text, style)]) columns of the physical HUD line.

        Every column has a fixed width so each value sits under the value of
        the line above; an over-range value overflows rather than being
        clipped, and a missing channel is a same-width dash placeholder. The
        styles are a channel map -- tires blue, crank cyan, rider green, drive
        magenta, slip yellow -- plus dynamic flags: RTF grades green/yellow/
        red, an airborne wheel is red, |kappa| past SLIP_WARN is red, a support
        pulling against its weld is orange1, a slipping one red.
        """
        physical = sim.physical
        factor = physical.live_real_time_factor
        rate = "----x" if factor is None else f"{factor:.2f}x"
        rtf_style = ("dim" if factor is None else "green" if factor >= .95
                     else "yellow" if factor >= .5 else "red")
        if physical.sample is None:
            return [("PHYS", [(f"[PHYS|{sim.physics_config.drive_mode}]", "bold cyan")]),
                    ("", [(" initial condition", ""), (f" RTF={rate}", rtf_style)])]
        channels = physical.sample.channels
        tires = channels["tires"]; drive = channels["drive"]
        rider = channels.get("rider", {})
        front_angle = degrees(drive.get("crank_phase_rad", 0.)) % 360.
        columns = []
        sep = ("", [(" | ", "")])

        def col(legend, *segments):
            # One space between columns of a group; " | " between groups.
            leading = [(" ", "")] if columns and columns[-1] is not sep else []
            columns.append((legend, leading + list(segments)))

        col("PHYS", (f"[PHYS|{sim.physics_config.drive_mode}]", "bold cyan"))
        col("time", (f"t={physical.sample.time_s:7.2f}s", ""))
        col("rtf", (f"RTF={rate}", rtf_style))
        col("speed", (f"v={physical.sample.qvel[sim.root_x_dofadr]*3.6:+6.2f}km/h", ""))
        normal_f = float(tires["front"].get("normal_load_n", 0.))
        normal_r = float(tires["rear"].get("normal_load_n", 0.))
        col("Fn f/r",
            ("Fn=", "blue"),
            (f"{normal_f:4.0f}", "red" if normal_f < _AIRBORNE_N else "blue"),
            ("/", "blue"),
            (f"{normal_r:4.0f}", "red" if normal_r < _AIRBORNE_N else "blue"),
            ("N", "blue"))
        columns.append(sep)
        col("crank f/r",
            (f"crk F@{front_angle:3.0f}deg R@{(front_angle+180.)%360.:3.0f}deg", "cyan"))
        pedal_values = []
        for side in ("front", "rear"):
            pedal = rider.get(f"{side}_pedal") or {}
            force = pedal.get("vertical_force_on_rider_n")
            if force is None:
                pedal_values.append(("----", "dim"))
                continue
            normal = float(pedal.get("normal_load_n", 0.))
            style = ("red" if pedal.get("would_slip") else
                     "orange1" if normal < 0. else "green")
            pedal_values.append((f"{float(force):+4.0f}", style))
        col("pedals f/r",
            ("ped=", "green"), pedal_values[0], ("/", "green"),
            pedal_values[1], ("N", "green"))
        col("cadence", (f"cad={drive.get('cadence_rpm',0.):+5.1f}rpm", "cyan"))
        gear_f, gear_r = drive.get("gear_front_teeth"), drive.get("gear_rear_teeth")
        col("gear", (f"gear={f'{gear_f:2d}/{gear_r:2d}' if gear_f and gear_r else '--/--'}T", "cyan"))
        col("human", (f"hum={drive.get('human_sensor_nm',0.):5.1f}Nm", "magenta"))
        col("motor", (f"mot={drive.get('motor_torque_nm',0.):5.1f}Nm", "magenta"))
        columns.append(sep)
        bar_segs = [("bar=", "green")]
        for index, side in enumerate(("left", "right")):
            force = (rider.get(f"grip_{side}") or {}).get("force_on_bike_n")
            if index:
                bar_segs.append((" ", "green"))
            bar_segs.append((f"{side[0].upper()}", "green"))
            if force is None:
                bar_segs.append(("-----N", "dim"))
                continue
            arrow = _force_arrow(force[0], force[2])
            # An up-pulling hand means the grip is carrying weight, like a
            # clipless pedal pulling on the backstroke: same color as the
            # weld-pull flags on the other two supports.
            bar_segs.append((arrow, "orange1" if force[2] > 0. else "green"))
            bar_segs.append((f"{np.hypot(force[0],force[2]):3.0f}N", "green"))
        col("bar l/r", *bar_segs)
        saddle = rider.get("saddle") or {}
        saddle_n = saddle.get("normal_load_n")
        col("saddle",
            ("saddle=", "green"),
            (f"{saddle_n:4.0f}" if saddle_n is not None else "----",
             "orange1" if (saddle_n or 0.) < 0. else "green"),
            ("N", "green"))
        torso_pitch = self._rider_body_pitch_deg(sim, "rider_torso")
        lean = None if torso_pitch is None else torso_pitch - degrees(sim.pitch_rad)
        col("lean",
            ("lean=", "green"),
            (f"{lean:+5.1f}" if lean is not None else "-----", "green"),
            ("deg", "green"))
        columns.append(sep)
        kappa_f = tires["front"].get("slip_ratio")
        kappa_r = tires["rear"].get("slip_ratio")
        kappa_segs = [("κ=", "yellow")]
        for index, kappa in enumerate((kappa_f, kappa_r)):
            if index:
                kappa_segs.append(("/", "yellow"))
            if kappa is None:
                kappa_segs.append(("-----", "dim"))
            else:
                kappa_segs.append((f"{kappa:+5.2f}",
                                   "red" if abs(kappa) > _SLIP_WARN else "yellow"))
        col("slip f/r", *kappa_segs)
        coast = (drive.get("coasting_reason")
                 or ("" if drive.get("freehub_engaged", True) else "freehub"))
        col("coast", (f"coast={(coast or '-'):<12}", "yellow" if coast else "dim"))
        columns.append(sep)
        col("shaft W", (f"motor={drive.get('motor_shaft_power_w',0.):+5.0f}W", "magenta"))
        return columns

    def styled_line(self, sim: "RideSimulation") -> "Text":
        """The physical HUD line as styled rich Text."""
        return styled(
            segment for _, segments in self._physical_columns(sim) for segment in segments
        )

    def physical_legend(self, sim: "RideSimulation") -> "Text":
        """Dim legend row whose names sit at the start of each column."""
        parts = []
        for legend, segments in self._physical_columns(sim):
            width = sum(len(text) for text, _ in segments)
            blanks = 0
            for text, _ in segments:
                if text.strip():
                    break
                blanks += len(text)
            parts.append(" " * blanks + legend.ljust(width - blanks))
        return Text("".join(parts), style="dim")

    def print_line(
        self,
        sim: "RideSimulation",
        braking: bool = False,
        brake_strength: float = 0.0,
    ) -> None:
        """
        Writes the HUD line; physical runs log one styled line per refresh,
        legacy runs rewrite the current terminal line in place.

        Args:
            sim: Simulation to read.
            braking: Whether the brake toggle is currently engaged.
            brake_strength: Brake lever position the toggle applies, in [0, 1].
        """
        physical = getattr(sim, "physical", None)
        if physical is not None:
            self._report_physical_events(sim)
            generation = getattr(physical, "generation", None)
            if generation != getattr(self, "_legend_generation", _UNSET):
                # A new run re-prints its column legend so a scrolled-back log
                # still carries its key.
                self._legend_generation = generation
                console.print(self.physical_legend(sim))
            if sys.stdout.isatty():
                # Line-per-refresh log, not an in-place HUD: the physical viewer
                # doubles as a console trace, and a \r-rewritten line is
                # impossible to scroll back, grep or copy.
                console.print(self.styled_line(sim))
                return
            text = self.line(sim, braking, brake_strength)
            t = physical.sample.time_s if physical.sample is not None else 0.0
            last = getattr(self, "_file_print_s", None)
            if last is None or t < last or t - last >= 1.0:
                self._file_print_s = t
                sys.stdout.write(text + "\n")
                sys.stdout.flush()
            return
        sys.stdout.write("\r" + self.line(sim, braking, brake_strength))
        sys.stdout.flush()

    def _report_physical_events(self, sim: "RideSimulation") -> None:
        """Prints one latched line for the reference monitor's first failure."""
        monitor = getattr(sim.physical, "reference_monitor", None)
        failure = getattr(monitor, "first_failure", None)
        if failure is None or getattr(self, "_reported_failure", None) is failure:
            return
        self._reported_failure = failure
        event(f"[PHYS|event] first failure at t={failure[0]:.6f}s: {', '.join(failure[1])}")

    def _rider_body_pitch_deg(self, sim: "RideSimulation", body_name: str):
        """World pitch of an articulated-rider body, or None when it is absent."""
        try:
            body_id = self._cached_lookup(
                sim, ("body", body_name),
                lambda: int(mujoco.mj_name2id(
                    sim.model, mujoco.mjtObj.mjOBJ_BODY, body_name)))
        except Exception:
            return None
        if body_id < 0:
            return None
        rotation = sim.data.xmat[body_id].reshape(3, 3)
        return degrees(atan2(-rotation[2, 0], rotation[0, 0]))

    def _cached_lookup(self, sim: "RideSimulation", key, resolve):
        """Memoize a model-name resolution; the compiled model never changes."""
        cache = getattr(self, "_lookup_cache", None)
        if cache is None or cache[0] is not sim.model:
            cache = (sim.model, {})
            self._lookup_cache = cache
        ids = cache[1]
        if key not in ids:
            ids[key] = resolve()
        return ids[key]

    def preview_log_row(self, sim: "RideSimulation") -> dict:
        """Build one CSV row of the physical preview's key state channels.

        Columns absent from this build (no rider contacts, no tire model) are
        emitted as empty strings, so every preview CSV shares one schema.
        """
        row = {"time_s": float(sim.time_s)}
        if getattr(sim, "physical", None) is None:
            return row
        drive = sim.physical.drive.last
        assist = sim.physical.drive.assist
        obstacle = nearest_obstacle(sim.track, sim.position_m)
        fork_spring = sim.force_accumulator.component("fork_spring")
        shock_coil = sim.force_accumulator.component("shock_coil")
        shift_count = int(drive.get("shift_count") or 0)
        crank_rate = drive.get("crank_target_rate_rad_s")
        grade = (0.0 if sim.track.grade_profile is None
                 else 100.0 * sim.track.grade_profile.slope(sim.position_m))
        row.update({
            "x_m": float(sim.position_m),
            "rtf": sim.physical.live_real_time_factor,
            "speed_kmh": float(sim.speed_mps) * KMH_PER_MPS,
            "pitch_deg": float(degrees(sim.pitch_rad)),
            "grade_pct": float(grade),
            "obstacle": "" if obstacle is None else obstacle[0],
            "obstacle_distance_m": "" if obstacle is None else float(obstacle[1]),
            "front_load_n": float(sim.contacts.front_load_n),
            "rear_load_n": float(sim.contacts.rear_load_n),
            "fork_travel_mm": float(sim.fork_travel_mm),
            "fork_velocity_mps": float(sim.data.qvel[sim.applier.fork_dofadr]),
            "fork_force_n": float(0.0 if fork_spring is None
                                  else fork_spring[sim.applier.fork_dofadr]),
            "shock_stroke_mm": float(sim.shock_stroke_mm),
            "shock_velocity_mps": float(sim.data.qvel[sim.applier.shock_dofadr]),
            "shock_force_n": float(0.0 if shock_coil is None
                                   else shock_coil[sim.applier.shock_dofadr]),
            "crank_phase_rad": float(sim.data.qpos[self._cached_lookup(
                sim, "crank_spin", lambda: sim.physical.address("crank_spin"))[0]]),
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
            root_pitch = degrees(float(sim.data.qpos[self._cached_lookup(
                sim, "rider_root_pitch", lambda: sim.physical.address("rider_root_pitch"))[0]]))
            body_pitch = {}
            for name in ("rider_pelvis", "rider_torso"):
                body_id = self._cached_lookup(
                    sim, ("body", name),
                    lambda name=name: int(mujoco.mj_name2id(
                        sim.model, mujoco.mjtObj.mjOBJ_BODY, name)))
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
    V               : Crank reposition (backpedal to a power phase; needs drive.motor_clutch)

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
