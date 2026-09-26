"""
Unit tests for the ride-mode controllers and one full `single_edge` traverse.

Tests include:
- A 40 m `single_edge` run at 25 km/h: it reaches the edge, is worked by it, finishes the
  track, and never trips the crash detector.
- Mean speed over the flat run-up against the 25 km/h target.
- Cruise control closing on chassis speed rather than wheel speed, its airborne gate, its
  anti-wind-up, and the adjustable speed band.
- Brake torque opposing rotation for both signs of wheel speed, and vanishing at rest.
- Rolling-resistance torque scaling with the instantaneous normal load, reading the raw
  vertical support channel rather than the bridged gate, and capped against solver spikes.
- The pitch stabilizer's ceiling, its both-wheels-airborne gate, and that it does not leave a
  stale moment latched in `qfrc_applied` after touchdown.
- The crash detector's two causes and its latching.
- The contact query bridging MuJoCo's single-step sphere-heightfield collision dropouts, and
  its raw support channel summing to system weight on level road.
"""

from typing import List, NamedTuple, Optional, Tuple

import mujoco
import numpy as np
import pytest

from bike_sim.sim.ride.braking import BRAKE_TAPER_RADPS, BRAKE_TORQUE_CEILING_NM, BrakeController
from bike_sim.sim.ride.contacts import TerrainContacts
from bike_sim.sim.ride.cruise import (
    DRIVE_TORQUE_CEILING_NM,
    MAX_TARGET_SPEED_KMH,
    MIN_TARGET_SPEED_KMH,
    CruiseController,
)
from bike_sim.sim.ride.resistance import (
    CRR,
    LOAD_CEILING_WEIGHTS,
    ROLLING_TAPER_RADPS,
    RollingResistance,
)
from bike_sim.sim.ride.virtual_rider import (
    CAUSE_HANDLEBAR_CONTACT,
    CAUSE_PITCH_OVER,
    CRASH_PITCH_LIMIT_DEG,
    PITCH_MOMENT_CEILING_NM,
    CrashDetector,
    CrashEvent,
    PitchStabilizer,
)
from bike_sim.sim.ride_sim import RideSimulation
from bike_sim.terrain import get_preset

TARGET_SPEED_KMH = 25.0
TARGET_SPEED_MPS = TARGET_SPEED_KMH / 3.6

# The square edge of the `single_edge` preset is a 90 mm ledge from 20.00 to 20.25 m; the
# window covers the approach, the ledge, and the settle after the step back down.
EDGE_WINDOW_M: Tuple[float, float] = (19.0, 22.5)

# Window over which mean speed is compared with the target. The bike starts from the solved
# static equilibrium at rest, so the first 8 m of the 20 m run-up are the acceleration to
# target; measuring across those would compare an acceleration ramp with a cruise speed. The
# window is flat road, clear of the ramp and 2 m clear of the edge.
RUNUP_WINDOW_M: Tuple[float, float] = (10.0, 18.0)

# Traverse step cap: 40 m at 25 km/h is 5.8 s, i.e. 11 600 steps at the ride timestep. The
# cap is generous enough to absorb the acceleration ramp and a stumble at the edge, and
# tight enough that a bike which has stopped moving fails instead of hanging.
MAX_TRAVERSE_STEPS = 40000


class Traverse(NamedTuple):
    """Per-step trace of one full track traverse, one array per channel."""

    steps: int
    sim_time_s: float
    crash: Optional[CrashEvent]
    x_m: np.ndarray
    speed_mps: np.ndarray
    fork_mm: np.ndarray
    shock_mm: np.ndarray
    front_load_n: np.ndarray
    rear_load_n: np.ndarray
    front_support_n: np.ndarray
    rear_support_n: np.ndarray
    front_in_contact: np.ndarray
    rear_in_contact: np.ndarray
    rider_moment_nm: np.ndarray
    front_rolling_nm: np.ndarray
    rear_rolling_nm: np.ndarray

    def window(self, lo_m: float, hi_m: float) -> np.ndarray:
        """Returns the boolean mask of samples with track position in [lo_m, hi_m]."""
        return (self.x_m >= lo_m) & (self.x_m <= hi_m)


@pytest.fixture(scope="module")
def sim() -> RideSimulation:
    """Compiles the ride model on `single_edge` once: compilation and the sag solve are slow."""
    return RideSimulation(track=get_preset("single_edge"), target_speed_kmh=TARGET_SPEED_KMH)


@pytest.fixture(scope="module")
def model(sim: RideSimulation) -> mujoco.MjModel:
    """The compiled ride model, for controller unit tests that drive their own `MjData`."""
    return sim.model


@pytest.fixture(scope="module")
def traverse(sim: RideSimulation) -> Traverse:
    """Rides `single_edge` from the start to the end of the track, recording every step."""
    sim.reset()
    channels: List[Tuple[float, ...]] = []
    while sim.position_m < sim.track.length_m and sim.steps < MAX_TRAVERSE_STEPS:
        sim.step()
        channels.append(
            (
                sim.position_m,
                sim.speed_mps,
                sim.fork_travel_mm,
                sim.shock_stroke_mm,
                sim.contacts.front_load_n,
                sim.contacts.rear_load_n,
                sim.contacts.front_support_n,
                sim.contacts.rear_support_n,
                float(sim.contacts.front_in_contact),
                float(sim.contacts.rear_in_contact),
                sim.stabilizer.moment_nm,
                sim.resistance.front_torque_nm,
                sim.resistance.rear_torque_nm,
            )
        )

    trace = np.array(channels, dtype=float)
    return Traverse(
        steps=sim.steps,
        sim_time_s=sim.time_s,
        crash=sim.crash,
        x_m=trace[:, 0],
        speed_mps=trace[:, 1],
        fork_mm=trace[:, 2],
        shock_mm=trace[:, 3],
        front_load_n=trace[:, 4],
        rear_load_n=trace[:, 5],
        front_support_n=trace[:, 6],
        rear_support_n=trace[:, 7],
        front_in_contact=trace[:, 8].astype(bool),
        rear_in_contact=trace[:, 9].astype(bool),
        rider_moment_nm=trace[:, 10],
        front_rolling_nm=trace[:, 11],
        rear_rolling_nm=trace[:, 12],
    )


def _contacts(
    front_n: float,
    rear_n: float,
    handlebar_n: float = 0.0,
    front_support_n: Optional[float] = None,
    rear_support_n: Optional[float] = None,
) -> TerrainContacts:
    """
    Builds a synthetic contact snapshot, so a controller can be driven without a track.

    The support channels default to the gating loads, which is the flat-ground case where the
    two are equal. Pass them explicitly to drive the two apart, as a collision dropout or a
    square-edge face impact does.
    """
    return TerrainContacts(
        front_load_n=front_n,
        rear_load_n=rear_n,
        front_support_n=front_n if front_support_n is None else front_support_n,
        rear_support_n=rear_n if rear_support_n is None else rear_support_n,
        handlebar_load_n=handlebar_n,
    )


def _dofadr(model: mujoco.MjModel, joint_name: str) -> int:
    """Returns the dof address of a named joint."""
    return int(model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)])


def _qposadr(model: mujoco.MjModel, joint_name: str) -> int:
    """Returns the qpos address of a named joint."""
    return int(model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)])


# --------------------------------------------------------------------------------------
# The traverse
# --------------------------------------------------------------------------------------


def test_single_edge_traverse_finishes_the_track(sim: RideSimulation, traverse: Traverse):
    """The bike rides the whole 40 m track, inside the step cap, without crashing."""
    assert traverse.crash is None
    assert traverse.x_m[-1] >= sim.track.length_m
    assert traverse.steps < MAX_TRAVERSE_STEPS


def test_traverse_is_worked_by_the_edge(traverse: Traverse):
    """
    The 90 mm edge shows up as a large compression, so the bike really crossed it.

    This is also the check that the heightfield was filled before the run: on an unfilled
    field the road is dead flat and the fork would never leave its sag band.
    """
    runup_peak_mm = traverse.fork_mm[traverse.window(*RUNUP_WINDOW_M)].max()
    edge_peak_mm = traverse.fork_mm[traverse.window(*EDGE_WINDOW_M)].max()
    peak_x_m = traverse.x_m[traverse.fork_mm.argmax()]

    assert runup_peak_mm < 90.0
    assert edge_peak_mm > runup_peak_mm + 50.0
    assert EDGE_WINDOW_M[0] <= peak_x_m <= EDGE_WINDOW_M[1]


def test_mean_runup_speed_tracks_the_target(traverse: Traverse):
    """Mean speed over the flat part of the run-up is within 5 % of the 25 km/h target."""
    mean_mps = float(traverse.speed_mps[traverse.window(*RUNUP_WINDOW_M)].mean())
    assert mean_mps == pytest.approx(TARGET_SPEED_MPS, rel=0.05)


def test_pitch_stabilizer_stays_inside_its_ceiling_over_the_traverse(traverse: Traverse):
    """No moment applied anywhere on the track exceeds the +/-80 N.m damage limit."""
    assert np.abs(traverse.rider_moment_nm).max() <= PITCH_MOMENT_CEILING_NM


def test_pitch_stabilizer_is_silent_whenever_a_wheel_is_in_contact(traverse: Traverse):
    """
    Across the whole traverse the moment is exactly zero on every grounded step.

    On `single_edge` that is every step: the 90 mm edge lifts each wheel in turn for about
    110 ms but never both at once, so the stabilizer's flight behaviour is exercised only by
    the unit tests below. The full track's kicker is what puts it in the air.
    """
    grounded = traverse.front_in_contact | traverse.rear_in_contact
    assert np.all(traverse.rider_moment_nm[grounded] == 0.0)


def test_contact_query_bridges_single_step_collision_dropouts(traverse: Traverse):
    """
    Both wheels report contact on every step of the flat run-up.

    MuJoCo's sphere-heightfield collision reports no contact at all for a loaded wheel on
    13 % of steps, and for both wheels at once on 1.7 % of them, always for a single step.
    Unbridged, that gates the drive torque off and makes the bike briefly airborne on flat
    ground -- which fires the virtual rider on a road with no features at all.
    """
    flat = traverse.window(*RUNUP_WINDOW_M)
    assert np.all(traverse.front_in_contact[flat])
    assert np.all(traverse.rear_in_contact[flat])
    assert traverse.front_load_n[flat].min() > 0.0
    assert traverse.rear_load_n[flat].min() > 0.0


def test_raw_support_load_sums_to_system_weight_on_flat_ground(
    sim: RideSimulation, traverse: Traverse
):
    """
    The unbridged support channel adds up to the bike's weight on level road, the gate does not.

    This is the whole reason the two channels exist. Bridging is right for the gate, where a
    false airborne fires the rider and cuts the drive torque, and wrong for the load fed to
    `Crr . N . r`, where holding a load across the 13 % of steps with no contact row adds a
    load that was never there. Measured over this window: raw 100.2 % of weight, bridged
    111.7 %. Rolling resistance is a sink docs/RIDE.md section 8 reports on its own line, so
    that 11 % would be a systematic error in a spec-reported quantity on every run.
    """
    flat = traverse.window(*RUNUP_WINDOW_M)
    weight_n = sim.resistance.system_weight_n
    raw_mean_n = float((traverse.front_support_n[flat] + traverse.rear_support_n[flat]).mean())
    bridged_mean_n = float((traverse.front_load_n[flat] + traverse.rear_load_n[flat]).mean())

    assert raw_mean_n == pytest.approx(weight_n, rel=0.01)
    assert bridged_mean_n > 1.10 * weight_n


def test_traverse_rolling_resistance_stays_within_its_load_ceiling(
    sim: RideSimulation, traverse: Traverse
):
    """
    No step of the traverse charges rolling resistance above `Crr` times the capped load.

    Uncapped, the square edge drives this channel to 66 N.m at the front and 87 N.m at the
    rear off single-timestep solver impact spikes of 11.6 and 16.0 times system weight -- a third
    of the *brake* ceiling, under the label "rolling resistance", in a channel task 5 plots
    and uses in an energy-balance assertion.
    """
    front_ceiling_nm = CRR * sim.resistance.load_ceiling_n * sim.resistance.front_wheel.radius_m
    rear_ceiling_nm = CRR * sim.resistance.load_ceiling_n * sim.resistance.rear_wheel.radius_m

    assert np.abs(traverse.front_rolling_nm).max() <= front_ceiling_nm
    assert np.abs(traverse.rear_rolling_nm).max() <= rear_ceiling_nm
    # And the cap is low enough that the channel cannot be mistaken for a brake event.
    assert max(front_ceiling_nm, rear_ceiling_nm) < 0.1 * BRAKE_TORQUE_CEILING_NM


# --------------------------------------------------------------------------------------
# Cruise control
# --------------------------------------------------------------------------------------


def test_cruise_closes_on_chassis_speed_not_wheel_speed(model: mujoco.MjModel):
    """
    A spun-up rear wheel produces no torque demand while the chassis is on target.

    Regulating wheel speed instead would see a wheel far above target, command full negative
    torque, and hand the loop a state that has nothing to do with how fast the bike is going.
    """
    data = mujoco.MjData(model)
    cruise = CruiseController(model, target_speed_kmh=TARGET_SPEED_KMH)
    data.qvel[_dofadr(model, "root_x")] = cruise.target_speed_mps
    data.qvel[_dofadr(model, "rear_wheel_spin")] = 200.0

    assert cruise.compute(model, data, _contacts(400.0, 500.0)) == 0.0


def test_cruise_is_gated_off_while_the_rear_wheel_is_airborne(model: mujoco.MjModel):
    """With no load on the rear wheel the torque is exactly zero and nothing accumulates."""
    data = mujoco.MjData(model)
    cruise = CruiseController(model, target_speed_kmh=TARGET_SPEED_KMH)

    for _ in range(200):
        assert cruise.compute(model, data, _contacts(0.0, 0.0)) == 0.0
    assert cruise.integral_mps_s == 0.0
    assert cruise.engaged is False


def test_cruise_torque_is_clamped_to_the_drive_ceiling(model: mujoco.MjModel):
    """A standing start demands far more than the ceiling and gets exactly the ceiling."""
    data = mujoco.MjData(model)
    cruise = CruiseController(model, target_speed_kmh=TARGET_SPEED_KMH)

    assert cruise.compute(model, data, _contacts(400.0, 500.0)) == DRIVE_TORQUE_CEILING_NM


def test_cruise_integrator_does_not_wind_up_while_saturated(model: mujoco.MjModel):
    """
    The integrator holds while the output is saturated, and resumes once it is not.

    An integrator that kept accumulating through the 1.5 s of saturated acceleration would
    hand the loop a torque it then has to unwind, which reads as a speed overshoot the track
    is too short to settle.
    """
    data = mujoco.MjData(model)
    cruise = CruiseController(model, target_speed_kmh=TARGET_SPEED_KMH)
    contacts = _contacts(400.0, 500.0)

    for _ in range(500):
        cruise.compute(model, data, contacts)
    assert cruise.integral_mps_s == 0.0

    error_mps = 0.1
    data.qvel[_dofadr(model, "root_x")] = cruise.target_speed_mps - error_mps
    for _ in range(10):
        cruise.compute(model, data, contacts)
    assert cruise.integral_mps_s == pytest.approx(10 * error_mps * model.opt.timestep)


def test_cruise_suspends_integration_when_pneumatic_rear_tyre_slides_toward_error(
    model: mujoco.MjModel,
):
    """A tyre-limited drive request holds the PI integral even below the torque ceiling."""
    data = mujoco.MjData(model)
    cruise = CruiseController(model, target_speed_kmh=TARGET_SPEED_KMH)
    error_mps = 0.1
    data.qvel[_dofadr(model, "root_x")] = cruise.target_speed_mps - error_mps
    contacts = _contacts(400.0, 500.0)

    torque = cruise.compute(model, data, contacts, traction_limited=True)
    assert torque == pytest.approx(cruise.kp_nm_per_mps * error_mps)
    assert cruise.integral_mps_s == 0.0

    cruise.compute(model, data, contacts, traction_limited=False)
    assert cruise.integral_mps_s == pytest.approx(error_mps * model.opt.timestep)


@pytest.mark.parametrize("target_kmh", [MIN_TARGET_SPEED_KMH - 1.0, MAX_TARGET_SPEED_KMH + 1.0])
def test_cruise_rejects_a_target_outside_the_adjustable_band(model: mujoco.MjModel, target_kmh: float):
    """A target outside 15-45 km/h is a mis-specified experiment, not a value to clamp."""
    with pytest.raises(ValueError, match="outside the adjustable band"):
        CruiseController(model, target_speed_kmh=target_kmh)


# --------------------------------------------------------------------------------------
# Brakes
# --------------------------------------------------------------------------------------


@pytest.mark.parametrize("omega_radps,expected_sign", [(5.0, -1.0), (-5.0, 1.0)])
def test_brake_torque_opposes_wheel_rotation(
    model: mujoco.MjModel, omega_radps: float, expected_sign: float
):
    """Full demand gives the full ceiling against the direction the wheel is turning."""
    data = mujoco.MjData(model)
    brakes = BrakeController(model)
    data.qvel[_dofadr(model, "front_wheel_spin")] = omega_radps
    data.qvel[_dofadr(model, "rear_wheel_spin")] = omega_radps

    front_nm, rear_nm = brakes.compute(data, front_demand=1.0, rear_demand=1.0)
    assert front_nm == pytest.approx(expected_sign * BRAKE_TORQUE_CEILING_NM)
    assert rear_nm == pytest.approx(expected_sign * BRAKE_TORQUE_CEILING_NM)


def test_brake_torque_is_zero_at_rest(model: mujoco.MjModel):
    """A held brake on a standing bike does nothing, rather than driving it backwards."""
    data = mujoco.MjData(model)
    brakes = BrakeController(model)

    assert brakes.compute(data, front_demand=1.0, rear_demand=1.0) == (0.0, 0.0)


def test_brake_torque_tapers_through_the_low_speed_band(model: mujoco.MjModel):
    """Inside the taper band the torque scales linearly with wheel speed."""
    data = mujoco.MjData(model)
    brakes = BrakeController(model)
    fraction = 0.25
    data.qvel[_dofadr(model, "rear_wheel_spin")] = fraction * BRAKE_TAPER_RADPS

    _, rear_nm = brakes.compute(data, front_demand=0.0, rear_demand=1.0)
    assert rear_nm == pytest.approx(-fraction * BRAKE_TORQUE_CEILING_NM)


@pytest.mark.parametrize("demand,expected_nm", [(0.4, -80.0), (2.0, -200.0), (-1.0, 0.0)])
def test_brake_demand_is_clamped_into_the_unit_interval(
    model: mujoco.MjModel, demand: float, expected_nm: float
):
    """Demand scales the ceiling and is clamped, so the interactive control cannot overrun it."""
    data = mujoco.MjData(model)
    brakes = BrakeController(model)
    data.qvel[_dofadr(model, "front_wheel_spin")] = 5.0

    front_nm, _ = brakes.compute(data, front_demand=demand, rear_demand=0.0)
    assert front_nm == pytest.approx(expected_nm)


# --------------------------------------------------------------------------------------
# Rolling resistance
# --------------------------------------------------------------------------------------


def test_rolling_resistance_scales_with_the_normal_load(model: mujoco.MjModel):
    """Doubling the normal load doubles the resistive torque: Crr . N . r, exactly."""
    data = mujoco.MjData(model)
    resistance = RollingResistance(model)
    data.qvel[_dofadr(model, "rear_wheel_spin")] = 20.0

    resistance.apply(model, data, _contacts(front_n=0.0, rear_n=500.0))
    single = resistance.rear_torque_nm
    resistance.apply(model, data, _contacts(front_n=0.0, rear_n=1000.0))
    doubled = resistance.rear_torque_nm

    assert single == pytest.approx(-CRR * 500.0 * resistance.rear_wheel.radius_m)
    assert doubled == pytest.approx(2.0 * single)


@pytest.mark.parametrize("omega_radps,expected_sign", [(20.0, -1.0), (-20.0, 1.0)])
def test_rolling_resistance_opposes_rotation(
    model: mujoco.MjModel, omega_radps: float, expected_sign: float
):
    """The resistive torque follows the sign of the wheel's rotation, not of its load."""
    data = mujoco.MjData(model)
    resistance = RollingResistance(model)
    data.qvel[_dofadr(model, "front_wheel_spin")] = omega_radps

    resistance.apply(model, data, _contacts(front_n=500.0, rear_n=0.0))
    assert np.sign(resistance.front_torque_nm) == expected_sign


def test_rolling_resistance_vanishes_on_an_unloaded_wheel(model: mujoco.MjModel):
    """
    An airborne wheel carries no load, so it carries no rolling resistance.

    The write happens on every step regardless: `qfrc_applied` is never cleared by MuJoCo, so
    a writer that skipped its zero steps would leave the last loaded torque latched in and
    keep braking a wheel that is no longer touching anything.
    """
    data = mujoco.MjData(model)
    resistance = RollingResistance(model)
    front_dof = _dofadr(model, "front_wheel_spin")
    data.qvel[front_dof] = 20.0

    resistance.apply(model, data, _contacts(front_n=500.0, rear_n=500.0))
    assert data.qfrc_applied[front_dof] < 0.0

    resistance.apply(model, data, _contacts(front_n=0.0, rear_n=0.0))
    assert resistance.front_torque_nm == 0.0
    assert data.qfrc_applied[front_dof] == 0.0


def test_rolling_resistance_reads_the_raw_support_load_not_the_bridged_gate(model: mujoco.MjModel):
    """
    A wheel whose gate is still bridged but whose real support has gone gets no resistance.

    That is exactly the first 5 ms of every genuine take-off. Consuming the bridged channel
    there keeps charging `Crr . N_last . r` to a wheel that is already in the air.
    """
    data = mujoco.MjData(model)
    resistance = RollingResistance(model)
    data.qvel[_dofadr(model, "rear_wheel_spin")] = 20.0

    resistance.apply(model, data, _contacts(front_n=0.0, rear_n=600.0, rear_support_n=0.0))
    assert resistance.rear_torque_nm == 0.0


def test_rolling_resistance_caps_the_load_at_a_multiple_of_system_weight(model: mujoco.MjModel):
    """A solver impact spike is charged at the stated ceiling, not at the spike."""
    data = mujoco.MjData(model)
    resistance = RollingResistance(model)
    data.qvel[_dofadr(model, "rear_wheel_spin")] = 20.0
    spike_n = 20.0 * resistance.system_weight_n

    resistance.apply(model, data, _contacts(front_n=0.0, rear_n=spike_n))
    assert resistance.load_ceiling_n == pytest.approx(
        LOAD_CEILING_WEIGHTS * resistance.system_weight_n
    )
    assert resistance.rear_torque_nm == pytest.approx(
        -CRR * resistance.load_ceiling_n * resistance.rear_wheel.radius_m
    )


def test_rolling_resistance_tapers_through_the_low_speed_band(model: mujoco.MjModel):
    """A standing wheel gets no rolling resistance, so it is not spun up by its own loss."""
    data = mujoco.MjData(model)
    resistance = RollingResistance(model)

    resistance.apply(model, data, _contacts(front_n=500.0, rear_n=500.0))
    assert resistance.front_torque_nm == 0.0
    assert resistance.rear_torque_nm == 0.0

    data.qvel[_dofadr(model, "rear_wheel_spin")] = 0.5 * ROLLING_TAPER_RADPS
    resistance.apply(model, data, _contacts(front_n=0.0, rear_n=500.0))
    assert resistance.rear_torque_nm == pytest.approx(
        -0.5 * CRR * 500.0 * resistance.rear_wheel.radius_m
    )


# --------------------------------------------------------------------------------------
# Virtual rider
# --------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "front_n,rear_n",
    [(500.0, 500.0), (500.0, 0.0), (0.0, 500.0)],
)
def test_pitch_stabilizer_is_inactive_while_either_wheel_is_in_contact(
    model: mujoco.MjModel, front_n: float, rear_n: float
):
    """On the ground the rider steers attitude through the tyres, so the moment is zero."""
    data = mujoco.MjData(model)
    stabilizer = PitchStabilizer(model)
    data.qpos[_qposadr(model, "root_pitch")] = 0.5
    data.qvel[stabilizer.pitch_dofadr] = 2.0

    assert stabilizer.apply(model, data, _contacts(front_n, rear_n)) == 0.0
    assert data.qfrc_applied[stabilizer.pitch_dofadr] == 0.0
    assert stabilizer.active is False


def test_pitch_stabilizer_does_not_latch_a_moment_after_touchdown(model: mujoco.MjModel):
    """
    The pitch DOF is written on every step, so a moment cannot survive the landing.

    `qfrc_applied` is never cleared by MuJoCo. A stabilizer that wrote only while airborne
    would leave its last flight moment applied for the rest of the run -- a plausible-looking
    simulation with a permanent invisible torque on the chassis.
    """
    data = mujoco.MjData(model)
    stabilizer = PitchStabilizer(model)
    data.qpos[_qposadr(model, "root_pitch")] = 0.5

    airborne_moment = stabilizer.apply(model, data, _contacts(0.0, 0.0))
    assert airborne_moment != 0.0
    assert data.qfrc_applied[stabilizer.pitch_dofadr] == airborne_moment

    stabilizer.apply(model, data, _contacts(0.0, 500.0))
    assert data.qfrc_applied[stabilizer.pitch_dofadr] == 0.0


@pytest.mark.parametrize("pitch_rad", [0.5, -0.5])
def test_pitch_stabilizer_clamps_to_its_ceiling_and_opposes_the_pitch(
    model: mujoco.MjModel, pitch_rad: float
):
    """A large attitude error saturates at exactly 80 N.m, against the direction of pitch."""
    data = mujoco.MjData(model)
    stabilizer = PitchStabilizer(model)
    data.qpos[_qposadr(model, "root_pitch")] = pitch_rad

    moment_nm = stabilizer.apply(model, data, _contacts(0.0, 0.0))
    assert abs(moment_nm) == pytest.approx(PITCH_MOMENT_CEILING_NM)
    assert np.sign(moment_nm) == -np.sign(pitch_rad)


def test_pitch_stabilizer_pd_is_linear_below_the_ceiling(model: mujoco.MjModel):
    """Inside the ceiling the moment is the plain PD expression on angle and rate."""
    data = mujoco.MjData(model)
    stabilizer = PitchStabilizer(model)
    pitch_rad, rate_radps = 0.05, 0.1
    data.qpos[_qposadr(model, "root_pitch")] = pitch_rad
    data.qvel[stabilizer.pitch_dofadr] = rate_radps

    expected_nm = -stabilizer.kp_nm_per_rad * pitch_rad - stabilizer.kd_nms_per_rad * rate_radps
    assert stabilizer.apply(model, data, _contacts(0.0, 0.0)) == pytest.approx(expected_nm)
    assert abs(expected_nm) < PITCH_MOMENT_CEILING_NM


def test_pitch_stabilizer_accounts_the_momentum_and_work_it_injects(model: mujoco.MjModel):
    """
    The moment has no reaction body, so its angular impulse and work are accumulated.

    docs/RIDE.md section 8 reports the virtual rider as a source on its own line rather than
    folding it into the physics, which requires both channels to be carried here.
    """
    data = mujoco.MjData(model)
    stabilizer = PitchStabilizer(model)
    data.qpos[_qposadr(model, "root_pitch")] = 0.5
    data.qvel[stabilizer.pitch_dofadr] = 2.0
    dt = float(model.opt.timestep)

    steps = 20
    for _ in range(steps):
        stabilizer.apply(model, data, _contacts(0.0, 0.0))

    assert stabilizer.moment_nm == pytest.approx(-PITCH_MOMENT_CEILING_NM)
    assert stabilizer.angular_impulse_nms == pytest.approx(steps * -PITCH_MOMENT_CEILING_NM * dt)
    assert stabilizer.work_j == pytest.approx(steps * -PITCH_MOMENT_CEILING_NM * 2.0 * dt)

    stabilizer.reset()
    assert stabilizer.angular_impulse_nms == 0.0
    assert stabilizer.work_j == 0.0


# --------------------------------------------------------------------------------------
# Crash detection
# --------------------------------------------------------------------------------------


def test_crash_detector_trips_on_excessive_pitch(model: mujoco.MjModel):
    """Pitch beyond 60 degrees is a crash, reported with its cause and track position."""
    data = mujoco.MjData(model)
    detector = CrashDetector(model)
    data.qpos[_qposadr(model, "root_x")] = 31.5
    data.qpos[_qposadr(model, "root_pitch")] = np.radians(CRASH_PITCH_LIMIT_DEG + 1.0)

    event = detector.check(data, _contacts(0.0, 0.0))
    assert event is not None
    assert detector.tripped
    assert event.cause == CAUSE_PITCH_OVER
    assert event.position_m == pytest.approx(31.5)
    assert f"{CAUSE_PITCH_OVER} at x = 31.50 m" in event.describe()


def test_crash_detector_trips_on_handlebar_ground_contact(model: mujoco.MjModel):
    """A handlebar in the dirt is a crash even at a pitch angle well inside the limit."""
    data = mujoco.MjData(model)
    detector = CrashDetector(model)

    event = detector.check(data, _contacts(0.0, 0.0, handlebar_n=250.0))
    assert event is not None
    assert event.cause == CAUSE_HANDLEBAR_CONTACT


def test_crash_detector_stays_silent_on_an_upright_bike(model: mujoco.MjModel):
    """Neither condition met is not a crash, and nothing is latched."""
    data = mujoco.MjData(model)
    detector = CrashDetector(model)
    data.qpos[_qposadr(model, "root_pitch")] = np.radians(CRASH_PITCH_LIMIT_DEG - 1.0)

    assert detector.check(data, _contacts(500.0, 500.0)) is None
    assert not detector.tripped


def test_crash_detector_latches_the_first_cause(model: mujoco.MjModel):
    """A run reports where it went wrong, not the last condition it happened to meet."""
    data = mujoco.MjData(model)
    detector = CrashDetector(model)
    data.qpos[_qposadr(model, "root_x")] = 12.0
    data.qpos[_qposadr(model, "root_pitch")] = np.radians(CRASH_PITCH_LIMIT_DEG + 1.0)
    first = detector.check(data, _contacts(0.0, 0.0))

    data.qpos[_qposadr(model, "root_x")] = 18.0
    again = detector.check(data, _contacts(0.0, 0.0, handlebar_n=250.0))

    assert again == first
    assert again.position_m == pytest.approx(12.0)

    detector.reset()
    assert not detector.tripped
