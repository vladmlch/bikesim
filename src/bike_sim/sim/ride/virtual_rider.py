"""
Ride-Mode Virtual Rider.

Two independent pieces, deliberately not merged: a pitch stabilizer that manages attitude
in flight, and a crash detector that judges the run. A stabilized run is not the same thing
as a successful one, so the detector never consults the stabilizer (docs/RIDE.md section 7).

**The stabilizer's moment has no reaction body.** The rider is lumped rigidly into `frame`,
so a moment written onto `root_pitch` is applied against the world: it injects angular
momentum, which is the single most significant non-physicality in the model. It is therefore
instrumented rather than hidden -- the accumulated angular impulse and the accumulated work
are carried here so that section 8 can report the rider as an explicit source on its own
line, never folded into "the physics".

`qfrc_applied` is never cleared by MuJoCo, so the pitch DOF is assigned on **every** step,
including the steps where the moment is zero. A stabilizer that wrote only while airborne
would leave its last moment latched in after touchdown and keep applying it for the rest of
the run.
"""

from dataclasses import dataclass
from math import degrees, radians
from typing import Optional, Tuple

import mujoco

from bike_sim.sim.ride.contacts import TerrainContacts

# Moment ceiling from docs/RIDE.md section 7: the order of magnitude an 80 kg rider can
# generate by rotating their torso. This is a damage limit, not a tuning knob.
PITCH_MOMENT_CEILING_NM = 80.0

# PD gains, in N.m per rad and N.m.s per rad. The pitch coordinate's own inertia, measured
# from the compiled model's mass matrix, is 38.6 kg.m^2, so KP = 500 puts the undamped mode
# at 3.60 rad/s (0.57 Hz) and KD = 150 damps it at 0.54 of critical. Both terms saturate the
# ceiling well inside the attitude errors a real flight produces -- 0.160 rad of angle, or
# 0.533 rad/s of rate, on their own -- so the gains mostly set where the moment reverses and
# the ceiling sets its authority.
DEFAULT_KP_NM_PER_RAD = 500.0
DEFAULT_KD_NMS_PER_RAD = 150.0

# Crash threshold from docs/RIDE.md section 7.
CRASH_PITCH_LIMIT_DEG = 60.0

CAUSE_PITCH_OVER = "pitch_over"
CAUSE_HANDLEBAR_CONTACT = "handlebar_contact"


class PitchStabilizer:
    """
    PD regulator driving `root_pitch` toward level while both wheels are off the ground.

    Accumulates the angular impulse and the mechanical work it injects, because neither is
    balanced by a reaction elsewhere in the model.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        kp_nm_per_rad: float = DEFAULT_KP_NM_PER_RAD,
        kd_nms_per_rad: float = DEFAULT_KD_NMS_PER_RAD,
        moment_ceiling_nm: float = PITCH_MOMENT_CEILING_NM,
    ) -> None:
        """
        Args:
            model: Compiled ride-mode model, used to resolve the `root_pitch` coordinate.
            kp_nm_per_rad: Proportional gain on pitch angle.
            kd_nms_per_rad: Derivative gain on pitch rate.
            moment_ceiling_nm: Symmetric clamp on the applied moment, in N.m.

        Raises:
            ValueError: If a gain is negative, if the ceiling is not positive, or if the
                model has no `root_pitch` joint.
        """
        if kp_nm_per_rad < 0.0 or kd_nms_per_rad < 0.0:
            raise ValueError(
                f"pitch gains must be non-negative, got kp={kp_nm_per_rad}, kd={kd_nms_per_rad}"
            )
        if moment_ceiling_nm <= 0.0:
            raise ValueError(f"moment_ceiling_nm must be positive, got {moment_ceiling_nm}")

        self.pitch_qposadr, self.pitch_dofadr = _resolve_pitch(model)
        self.kp_nm_per_rad = float(kp_nm_per_rad)
        self.kd_nms_per_rad = float(kd_nms_per_rad)
        self.moment_ceiling_nm = float(moment_ceiling_nm)

        self.moment_nm = 0.0
        self.angular_impulse_nms = 0.0
        self.work_j = 0.0
        self.active = False

    def apply(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        contacts: TerrainContacts,
    ) -> float:
        """
        Computes the stabilizing moment and writes it into `qfrc_applied`.

        Args:
            model: Compiled ride-mode model, read for the integration timestep.
            data: Simulation state, written at the `root_pitch` DOF.
            contacts: This step's contact snapshot. The moment is applied only while both
                wheels are out of contact; on the ground the rider steers attitude through
                the tyres, which the contact model already provides.

        Returns:
            The applied moment in N.m, zero whenever either wheel is in contact.
        """
        self.active = contacts.airborne
        pitch_rate_radps = float(data.qvel[self.pitch_dofadr])

        if self.active:
            demand = (
                -self.kp_nm_per_rad * float(data.qpos[self.pitch_qposadr])
                - self.kd_nms_per_rad * pitch_rate_radps
            )
            self.moment_nm = max(-self.moment_ceiling_nm, min(self.moment_ceiling_nm, demand))
        else:
            self.moment_nm = 0.0

        # Assigned every step, zero included: see the module docstring.
        data.qfrc_applied[self.pitch_dofadr] = self.moment_nm

        dt = float(model.opt.timestep)
        self.angular_impulse_nms += self.moment_nm * dt
        self.work_j += self.moment_nm * pitch_rate_radps * dt
        return self.moment_nm

    def reset(self) -> None:
        """Clears the moment and both accumulators for a fresh run."""
        self.moment_nm = 0.0
        self.angular_impulse_nms = 0.0
        self.work_j = 0.0
        self.active = False


@dataclass(frozen=True)
class CrashEvent:
    """
    The first crash condition a run met.

    Attributes:
        cause: Either `pitch_over` or `handlebar_contact`.
        time_s: Simulation time at which the condition was first seen.
        position_m: Chassis `root_x` at that moment, in metres along the track.
        pitch_rad: Chassis pitch at that moment, in radians.
    """

    cause: str
    time_s: float
    position_m: float
    pitch_rad: float

    def describe(self) -> str:
        """Returns a one-line human-readable summary of the crash."""
        return (
            f"{self.cause} at x = {self.position_m:.2f} m, t = {self.time_s:.3f} s, "
            f"pitch = {degrees(self.pitch_rad):+.1f} deg"
        )


class CrashDetector:
    """
    Trips on excessive pitch or on the handlebar touching the ground, and latches.

    The first condition met is kept; later steps do not overwrite it, so a run reports where
    it went wrong rather than where it ended up.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        pitch_limit_deg: float = CRASH_PITCH_LIMIT_DEG,
    ) -> None:
        """
        Args:
            model: Compiled ride-mode model, used to resolve the root coordinates.
            pitch_limit_deg: Absolute pitch beyond which the run is a crash.

        Raises:
            ValueError: If the limit is not positive, or a root joint is missing.
        """
        if pitch_limit_deg <= 0.0:
            raise ValueError(f"pitch_limit_deg must be positive, got {pitch_limit_deg}")

        self.pitch_qposadr, _ = _resolve_pitch(model)
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_x")
        if jid < 0:
            raise ValueError("model has no joint 'root_x'; the crash detector reports track position")
        self.root_x_qposadr = int(model.jnt_qposadr[jid])
        self.pitch_limit_rad = radians(float(pitch_limit_deg))

        self.event: Optional[CrashEvent] = None

    @property
    def tripped(self) -> bool:
        """Whether a crash condition has been seen at any point in the run."""
        return self.event is not None

    def check(self, data: mujoco.MjData, contacts: TerrainContacts) -> Optional[CrashEvent]:
        """
        Tests both crash conditions against the current state.

        Args:
            data: Simulation state, read for pitch and track position.
            contacts: This step's contact snapshot, supplying handlebar-to-ground contact.

        Returns:
            The latched crash event, or None while the run is still upright.
        """
        if self.event is not None:
            return self.event

        pitch_rad = float(data.qpos[self.pitch_qposadr])
        if abs(pitch_rad) > self.pitch_limit_rad:
            cause = CAUSE_PITCH_OVER
        elif contacts.handlebar_in_contact:
            cause = CAUSE_HANDLEBAR_CONTACT
        else:
            return None

        self.event = CrashEvent(
            cause=cause,
            time_s=float(data.time),
            position_m=float(data.qpos[self.root_x_qposadr]),
            pitch_rad=pitch_rad,
        )
        return self.event

    def reset(self) -> None:
        """Clears the latched event for a fresh run."""
        self.event = None


def _resolve_pitch(model: mujoco.MjModel) -> Tuple[int, int]:
    """
    Resolves the `root_pitch` joint's qpos and dof addresses.

    Args:
        model: Compiled MuJoCo model.

    Returns:
        Tuple of (qpos address, dof address).

    Raises:
        ValueError: If the model has no `root_pitch` joint.
    """
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_pitch")
    if jid < 0:
        raise ValueError("model has no joint 'root_pitch'; the virtual rider acts on chassis pitch")
    return int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])


__all__ = [
    "PitchStabilizer",
    "CrashDetector",
    "CrashEvent",
    "PITCH_MOMENT_CEILING_NM",
    "CRASH_PITCH_LIMIT_DEG",
    "CAUSE_PITCH_OVER",
    "CAUSE_HANDLEBAR_CONTACT",
]
