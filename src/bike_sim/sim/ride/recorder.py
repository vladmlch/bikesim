"""
Ride-Mode Telemetry Recorder.

Collects one row of channels per recorded step from a :class:`RideSimulation` and writes
them as CSV with the standard library. Nothing here is a pandas frame: the recorder holds
a chunked float64 buffer and hands back a plain NumPy array.

Channel conventions:

- Positions and travel in metres / millimetres as named; velocities in m/s; forces in N.
- ``*_contact`` flags are 0 / 1 as the contact query reports them (debounced).
- Accelerometer channels are **proper acceleration** resolved into world axes: at rest
  the vertical channel reads +9.81 m/s^2, in free fall 0. ``vert`` is world Z, ``long``
  world X. The raw solver transients on sharp edges are recorded as they are; filtering
  is the summary's job, not the recorder's.
- The virtual rider's moment, accumulated angular impulse and accumulated work are
  recorded so an energy audit can account for what the rider injected.
- The seated rider's interface loads (saddle, front and rear pedal, bar) are the forces the
  bike exerts on the rider, in N; ``saddle_gap_m`` is the pelvis-to-saddle separation, zero
  while seated. The ``rider_*_acc`` channels are the torso's and pelvis's proper
  acceleration in world axes, like the bar and saddle ones. All read zero without a seated
  rider, so a telemetry file has the same columns whichever rider rode.
"""

import csv
from pathlib import Path
from typing import TYPE_CHECKING, Dict, List, Sequence, Union

import mujoco
import numpy as np

from bike_sim.sim.ride.metrics import RAMP_EXCLUSION_M
from bike_sim.sim.ride.wheels import resolve_wheel_spin

if TYPE_CHECKING:  # pragma: no cover
    from bike_sim.sim.ride_sim import RideSimulation

CHANNELS: Sequence[str] = (
    "time_s",
    "x_m",
    "speed_mps",
    "fork_travel_mm",
    "fork_shaft_mps",
    "shock_stroke_mm",
    "rear_wheel_mm",
    "shock_shaft_mps",
    "fork_spring_n",
    "fork_damper_n",
    "shock_spring_n",
    "shock_bumper_n",
    "shock_damper_n",
    "pitch_rad",
    "front_contact",
    "rear_contact",
    "front_load_n",
    "rear_load_n",
    "drive_torque_nm",
    "wheel_power_w",
    "bar_acc_vert_mps2",
    "bar_acc_long_mps2",
    "saddle_acc_vert_mps2",
    "saddle_acc_long_mps2",
    "rider_moment_nm",
    "rider_impulse_nms",
    "rider_work_j",
    "saddle_load_n",
    "saddle_gap_m",
    "bar_hand_load_n",
    "pedal_load_front_n",
    "pedal_load_rear_n",
    "rider_torso_acc_vert_mps2",
    "rider_torso_acc_long_mps2",
    "rider_pelvis_acc_vert_mps2",
    "rider_pelvis_acc_long_mps2",
    "front_tyre_fz_n",
    "front_tyre_fx_n",
    "front_tyre_deflection_mm",
    "front_patch_length_mm",
    "front_slip_ratio",
    "front_tyre_full_sliding",
    "front_rim_strike",
    "front_tyre_pressure_bar",
    "front_tyre_loss_w",
    "rear_tyre_fz_n",
    "rear_tyre_fx_n",
    "rear_tyre_deflection_mm",
    "rear_patch_length_mm",
    "rear_slip_ratio",
    "rear_tyre_full_sliding",
    "rear_rim_strike",
    "rear_tyre_pressure_bar",
    "rear_tyre_loss_w",
)

# Channels the seated rider fills; every other rider variant records zeros here.
SEATED_RIDER_CHANNELS: Sequence[str] = CHANNELS[
    CHANNELS.index("saddle_load_n"):CHANNELS.index("rider_pelvis_acc_long_mps2") + 1
]
TYRE_CHANNELS: Sequence[str] = CHANNELS[CHANNELS.index("front_tyre_fz_n"):]

CSV_FLOAT_FORMAT = "%.10g"
_CHUNK_ROWS = 4096


class RideRecorder:
    """
    Per-step telemetry buffer for one ride.

    Use ``record`` directly or pass it as ``RideSimulation.run(on_step=...)``. With
    ``decimate = n`` only every n-th call is stored; the first call is always stored.
    """

    def __init__(self, sim: "RideSimulation", decimate: int = 1) -> None:
        """
        Args:
            sim: Simulation whose channels will be read. Resolves sensor and wheel handles
                once here, so recording itself does no name lookups.
            decimate: Store every n-th call; 1 stores every step.

        Raises:
            ValueError: If ``decimate`` is not positive or a needed sensor is missing.
        """
        if decimate < 1:
            raise ValueError(f"decimate must be >= 1, got {decimate}")
        self.decimate = int(decimate)
        self.timestep_s = float(sim.model.opt.timestep)

        self._bar = _Accelerometer(sim.model, "sensor_bar_accel", "site_handlebar")
        self._saddle = _Accelerometer(sim.model, "sensor_saddle_accel", "site_seatpost_top")
        self._rear_wheel = resolve_wheel_spin(sim.model, "rear_wheel_spin", "geom_rear_contact")
        # The rider's own accelerometers exist only on the seated rider's bodies.
        self._seated = bool(sim.rider_forces.active)
        if self._seated:
            self._torso = _Accelerometer(sim.model, "sensor_rider_torso_accel", "site_rider_torso")
            self._pelvis = _Accelerometer(sim.model, "sensor_rider_pelvis_accel", "site_rider_pelvis")

        # Rear wheel travel is a smooth function of shock stroke; tabulating the analytical
        # solver once at 0.1 mm and interpolating is ~30x cheaper per step than solving.
        stroke_limit_mm = float(sim.specs.shock_stroke)
        self._stroke_table_mm = np.linspace(0.0, stroke_limit_mm, int(round(stroke_limit_mm / 0.1)) + 1)
        self._travel_table_mm = np.array(
            [float(sim.solver.solve_state_from_shock_stroke(s)["wheel_travel"]) for s in self._stroke_table_mm]
        )

        self._chunks: List[np.ndarray] = []
        self._current = np.empty((_CHUNK_ROWS, len(CHANNELS)))
        self._fill = 0
        self._calls = 0
        self.tyre_rim_events: Dict[str, List[Dict[str, float]]] = {"front": [], "rear": []}
        self.tyre_rim_strike_counts: Dict[str, int] = {"front": 0, "rear": 0}
        self.tyre_rim_starts: Dict[str, List[float]] = {"front": [], "rear": []}
        self.tyre_times_s: Dict[str, Dict[str, float]] = {
            "front": {"wheelspin": 0.0, "locked": 0.0},
            "rear": {"wheelspin": 0.0, "locked": 0.0},
        }
        self._previous_rim_active: Dict[str, bool] = {"front": False, "rear": False}

    # ------------------------------------------------------------------ recording

    def record(self, sim: "RideSimulation") -> None:
        """
        Stores one row if this call falls on the decimation grid.

        Args:
            sim: Simulation to read. Nothing is written to it.
        """
        self._observe_tyre_events(sim)
        take = self._calls % self.decimate == 0
        self._calls += 1
        if not take:
            return
        if self._fill == _CHUNK_ROWS:
            self._chunks.append(self._current)
            self._current = np.empty((_CHUNK_ROWS, len(CHANNELS)))
            self._fill = 0
        self._current[self._fill] = self._row(sim)
        self._fill += 1

    def _observe_tyre_events(self, sim: "RideSimulation") -> None:
        """Captures rim events on every callback, even when CSV rows are decimated."""
        if sim.tyre_applier is None:
            return
        for wheel, outputs in (
            ("front", sim.tyre_applier.front_outputs),
            ("rear", sim.tyre_applier.rear_outputs),
        ):
            if outputs.rim_strike_active and not self._previous_rim_active[wheel]:
                self.tyre_rim_strike_counts[wheel] += 1
                self.tyre_rim_starts[wheel].append(float(outputs.hub_position_world_m[0]))
            self._previous_rim_active[wheel] = outputs.rim_strike_active
            if sim.position_m >= sim.start_x_m + RAMP_EXCLUSION_M:
                if any(patch.fully_sliding and patch.slip_ratio > 0.0 for patch in outputs.patches):
                    self.tyre_times_s[wheel]["wheelspin"] += float(sim.model.opt.timestep)
                if any(patch.fully_sliding and patch.slip_ratio < 0.0 for patch in outputs.patches):
                    self.tyre_times_s[wheel]["locked"] += float(sim.model.opt.timestep)
            event = outputs.rim_event
            if event is not None:
                self.tyre_rim_events[wheel].append({
                    "x_m": float(event.x_m),
                    "speed_mps": float(event.speed_mps),
                    "peak_load_n": float(event.peak_load_n),
                    "peak_rim_force_n": float(event.peak_rim_force_n),
                    "absorbed_energy_j": float(event.absorbed_energy_j),
                })

    def _row(self, sim: "RideSimulation") -> np.ndarray:
        data = sim.data
        applier = sim.applier
        contacts = sim.contacts
        bar_vert, bar_long = self._bar.world_components(data)
        saddle_vert, saddle_long = self._saddle.world_components(data)
        drive_torque = float(sim.cruise.torque_nm)
        stroke_mm = sim.shock_stroke_mm
        rider = sim.rider_forces
        if self._seated:
            torso_vert, torso_long = self._torso.world_components(data)
            pelvis_vert, pelvis_long = self._pelvis.world_components(data)
        else:
            torso_vert = torso_long = pelvis_vert = pelvis_long = 0.0
        if sim.tyre_applier is None:
            front_tyre = rear_tyre = None
        else:
            front_tyre = sim.tyre_applier.front_outputs
            rear_tyre = sim.tyre_applier.rear_outputs

        tyre_channels = []
        for outputs in (front_tyre, rear_tyre):
            if outputs is None:
                tyre_channels.extend((0.0,) * 9)
            else:
                tyre_channels.extend((
                    float(outputs.support_n),
                    float(outputs.force_world_n[0]),
                    float(outputs.mean_deflection_m * 1000.0),
                    float(outputs.contact_length_m * 1000.0),
                    float(outputs.slip_ratio),
                    float(outputs.fully_sliding),
                    float(outputs.rim_strike_active or outputs.rim_event is not None),
                    float(outputs.pressure_bar),
                    float(outputs.dissipated_power_w),
                ))
        return np.array(
            [
                float(data.time),
                sim.position_m,
                sim.speed_mps,
                sim.fork_travel_mm,
                float(data.qvel[applier.fork_dofadr]),
                stroke_mm,
                self.rear_wheel_mm(stroke_mm),
                float(data.qvel[applier.shock_dofadr]),
                applier.fork_spring_n,
                applier.fork_damper_n,
                applier.shock_spring_n,
                applier.shock_bumper_n,
                applier.shock_damper_n,
                sim.pitch_rad,
                1.0 if contacts.front_in_contact else 0.0,
                1.0 if contacts.rear_in_contact else 0.0,
                contacts.front_load_n,
                contacts.rear_load_n,
                drive_torque,
                drive_torque * self._rear_wheel.omega_radps(data),
                bar_vert,
                bar_long,
                saddle_vert,
                saddle_long,
                float(sim.stabilizer.moment_nm),
                float(sim.stabilizer.angular_impulse_nms),
                float(sim.stabilizer.work_j),
                float(rider.saddle_load_n),
                float(rider.saddle_gap_m),
                float(rider.bar_load_n),
                float(rider.pedal_load_front_n),
                float(rider.pedal_load_rear_n),
                torso_vert,
                torso_long,
                pelvis_vert,
                pelvis_long,
                *tyre_channels,
            ]
        )

    def rear_wheel_mm(self, stroke_mm: float) -> float:
        """
        Rear wheel travel for a shock stroke, from the tabulated leverage curve.

        Args:
            stroke_mm: Shock shaft stroke in millimetres; values outside the stroke range
                are extrapolated linearly from the nearest table segment.
        """
        table_s, table_t = self._stroke_table_mm, self._travel_table_mm
        if stroke_mm <= table_s[0]:
            slope = (table_t[1] - table_t[0]) / (table_s[1] - table_s[0])
            return float(table_t[0] + slope * (stroke_mm - table_s[0]))
        if stroke_mm >= table_s[-1]:
            slope = (table_t[-1] - table_t[-2]) / (table_s[-1] - table_s[-2])
            return float(table_t[-1] + slope * (stroke_mm - table_s[-1]))
        return float(np.interp(stroke_mm, table_s, table_t))

    # ------------------------------------------------------------------ access

    @property
    def rows(self) -> int:
        """Number of rows stored so far."""
        return len(self._chunks) * _CHUNK_ROWS + self._fill

    @property
    def sample_interval_s(self) -> float:
        """Time between stored rows, in seconds."""
        return self.timestep_s * self.decimate

    def array(self) -> np.ndarray:
        """Returns all stored rows as an array of shape ``(rows, len(CHANNELS))``."""
        parts = self._chunks + [self._current[: self._fill]]
        return np.vstack(parts) if parts else np.empty((0, len(CHANNELS)))

    def column(self, name: str) -> np.ndarray:
        """
        Returns one channel as a 1-D array.

        Args:
            name: Channel name from ``CHANNELS``.

        Raises:
            KeyError: On an unknown channel.
        """
        if name not in CHANNELS:
            raise KeyError(f"unknown channel '{name}'")
        return self.array()[:, CHANNELS.index(name)]

    def columns(self) -> Dict[str, np.ndarray]:
        """Returns every channel keyed by name."""
        table = self.array()
        return {name: table[:, i] for i, name in enumerate(CHANNELS)}

    # ------------------------------------------------------------------ output

    def write_csv(self, path: Union[str, Path]) -> Path:
        """
        Writes the stored rows as CSV with a header row.

        Args:
            path: Destination file.

        Returns:
            The path written.
        """
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        table = self.array()
        with open(destination, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(CHANNELS)
            for row in table:
                writer.writerow([CSV_FLOAT_FORMAT % v for v in row])
        return destination


class _Accelerometer:
    """Resolves one accelerometer and rotates its reading into world axes."""

    def __init__(self, model: mujoco.MjModel, sensor_name: str, site_name: str) -> None:
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
        if sid < 0:
            raise ValueError(f"model has no sensor '{sensor_name}'; ride mode telemetry needs it")
        self.adr = int(model.sensor_adr[sid])
        site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if site < 0:
            raise ValueError(f"model has no site '{site_name}'")
        self.site = int(site)

    def world_components(self, data: mujoco.MjData):
        """Returns (vertical, longitudinal) proper acceleration in world axes, m/s^2."""
        local = data.sensordata[self.adr : self.adr + 3]
        rotation = data.site_xmat[self.site].reshape(3, 3)
        world = rotation @ local
        return float(world[2]), float(world[0])


def read_csv(path: Union[str, Path]) -> Dict[str, np.ndarray]:
    """
    Reads a telemetry CSV back into channel arrays.

    Args:
        path: File written by :meth:`RideRecorder.write_csv`.

    Returns:
        Channels keyed by header name.
    """
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        header = next(reader)
        rows = [[float(v) for v in row] for row in reader]
    table = np.array(rows, dtype=float) if rows else np.empty((0, len(header)))
    return {name: table[:, i] for i, name in enumerate(header)}


__all__ = [
    "CHANNELS",
    "SEATED_RIDER_CHANNELS",
    "TYRE_CHANNELS",
    "TYRE_CHANNELS",
    "CSV_FLOAT_FORMAT",
    "RideRecorder",
    "read_csv",
]
