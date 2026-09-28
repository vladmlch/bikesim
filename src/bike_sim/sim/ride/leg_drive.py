"""
Ride-Mode Articulated Leg Drive.

Drives the seated rider's hip/knee/ankle chains: each step it solves the leg IK for the
crank's current phase, maps this leg's share of the rider's crank torque onto a tangential
pedal force through the leg Jacobian, and adds a joint-space impedance toward the IK pose.
Torques are written to `qfrc_applied` (the `rider_forces.py` convention): the six leg DOFs
are assigned every step, zero included, because `qfrc_applied` is never cleared by MuJoCo.

**The crank phase is shared, the crank arms are not.** `solve_leg_qpos(chain, phase)` is
called with the one crank phase for both legs -- the antiphase lives in the *signed*
`LegChain.crank_len_m` (positive front, negative rear), so the IK itself places each foot
on its own pedal. The *force* bookkeeping does need the per-side phase though:
`crank_torque_share` and the tangential direction `t_hat` are functions of where the leg's
own pedal is in its stroke, i.e. ``phase + pi`` for the rear leg.

**The welds do the driving.** Feet are welded to the pedal bodies (Task 2's
`weld_foot_*`), so the pedal force the Jacobian maps into joint torques reacts through the
weld onto the crank: rider torque reaches the wheel through legs, pedals and chain, which
is why `PedalDrivetrain` drops the rider's share from the crank actuator when this writer
is active (`legs_drive`). The impedance term then holds the feet on the pedals against the
force the weld transmits -- the clipless equivalent of a leg that is both strong and
tracking a trajectory.

`initialize` exists separately from `apply`: the weld datum is the compiled `qpos0`, so at
reset the legs, pedal hinges and crank are all written to the IK-consistent pose for the
crank's *actual* starting phase (`--crank-phase` included) before the first forward pass,
instead of spending the first second slamming the constraint shut.
"""

from math import cos, sin
from typing import Dict, List, Optional, Tuple

import mujoco
import numpy as np

from bike_sim.physics.drivetrain import ripple_shape
from bike_sim.physics.rider import (
    LegChain,
    SeatedPose,
    crank_torque_share,
    leg_jacobian,
    solve_leg_qpos,
)


def _hinge_addrs(model: mujoco.MjModel, name: str) -> Tuple[int, int]:
    """(qposadr, dofadr) of a joint; same shape as drivetrain.py's helper."""
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if jid < 0:
        raise ValueError(f"model has no joint '{name}'; articulated legs cannot be driven")
    return int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])


def _solve_or_current(
    chain: LegChain, phase_rad: float, pelvis_z_m: float, q_actual: np.ndarray
) -> np.ndarray:
    """IK target for one leg, falling back to the actual pose at the reach boundary.

    `solve_leg_qpos` raises once the hip-to-ankle distance exceeds thigh + shank, which a
    sagged pelvis makes marginally true around dead centre. The foot is welded to the
    pedal, so in that window the achievable pose is wherever the weld puts it: taking the
    measured angles as the reference keeps the impedance from fighting the constraint
    until the crank brings the pedal back inside reach.
    """
    try:
        return solve_leg_qpos(chain, phase_rad, pelvis_z_m)
    except ValueError:
        return np.asarray(q_actual, dtype=float).copy()


class LegDrive:
    """Drives the articulated legs: pedal force + impedance toward the IK pose.

    Constructed with ``pose=None`` or a pose without ``leg_chains`` it is inert --
    `active` is False and every method no-ops -- so callers never branch on the rider.

    Attributes:
        pose: The seated pose the leg chains came from.
        crank_len_m: Crank arm length (unsigned), for torque -> tangential force.
        ripple_depth: Depth of the per-leg torque-share pulse, from `DrivetrainSpecs`.
        pedal_force_front_n / pedal_force_rear_n: Last applied pedal force per leg, N;
            telemetry for the recorder.
    """

    IMPEDANCE_KP = (50.0, 50.0, 20.0)   # N.m/rad, hip/knee/ankle
    IMPEDANCE_KD = (4.0, 4.0, 1.5)      # N.m.s/rad

    def __init__(
        self,
        model: mujoco.MjModel,
        pose: Optional[SeatedPose],
        crank_len_m: float,
        ripple_depth: float,
    ) -> None:
        """
        Args:
            model: Compiled ride-mode model carrying the `rider_{hip,knee,ankle}_*` hinges,
                `rider_pelvis_z`, `crank_spin` and the `pedal_spin_*` hinges.
            pose: The seated pose, or None / a rigid-legs pose for an inert driver.
            crank_len_m: Crank arm length in metres.
            ripple_depth: Torque-share pulse depth in [0, 1].

        Raises:
            ValueError: If the pose carries leg chains but the model lacks a joint they
                need -- e.g. articulated legs without the crank joint.
        """
        self.pose = pose
        self.crank_len_m = float(crank_len_m)
        self.ripple_depth = float(ripple_depth)
        self._legs: List[Tuple[LegChain, List[int], List[int]]] = []
        self._prev_q_ref: Dict[str, np.ndarray] = {}
        self._pelvis_qposadr = 0
        self._crank_qposadr = 0
        self._pedal_adrs: Dict[str, Tuple[int, int]] = {}
        self.pedal_force_front_n = 0.0
        self.pedal_force_rear_n = 0.0
        if pose is None or not pose.leg_chains:
            return
        for chain in pose.leg_chains:
            adrs = [
                _hinge_addrs(model, f"rider_{j}_{chain.side}")
                for j in ("hip", "knee", "ankle")
            ]
            qposadrs, dofadrs = zip(*adrs)
            self._legs.append((chain, list(qposadrs), list(dofadrs)))
        self._pelvis_qposadr = _hinge_addrs(model, "rider_pelvis_z")[0]
        self._crank_qposadr = _hinge_addrs(model, "crank_spin")[0]
        self._pedal_adrs = {
            c.side: _hinge_addrs(model, f"pedal_spin_{c.side}") for c in pose.leg_chains
        }

    @property
    def active(self) -> bool:
        """Whether articulated legs are present to drive."""
        return bool(self._legs)

    @property
    def crank_qposadr(self) -> int:
        """qpos address of `crank_spin`, so reset can read the crank's actual phase."""
        return self._crank_qposadr

    def initialize(self, model: mujoco.MjModel, data: mujoco.MjData, phase_rad: float) -> None:
        """
        Puts legs and pedal platforms at the IK pose for the crank's starting phase.

        The weld datum is `qpos0`, so this writes the *actual* equilibrium pelvis height
        into the IK rather than the compiled zero -- a sprung pelvis sits millimetres below
        its design position, and ignoring that is an immediate weld residual.

        Args:
            model: Compiled model. Unused; kept for the writer signature.
            data: Simulation state, written at the leg and pedal-hinge coordinates.
            phase_rad: Crank phase to pose the legs for (the value the drivetrain's reset
                just wrote, or the compiled angle in motor mode).
        """
        pelvis_z = float(data.qpos[self._pelvis_qposadr]) if self._legs else 0.0
        for chain, qposadrs, dofadrs in self._legs:
            q_ref = _solve_or_current(chain, phase_rad, pelvis_z, data.qpos[qposadrs])
            data.qpos[qposadrs] = q_ref
            data.qvel[dofadrs] = 0.0
            pq, pd = self._pedal_adrs[chain.side]
            data.qpos[pq] = -phase_rad   # keeps the platform level in frame coords
            data.qvel[pd] = 0.0
            self._prev_q_ref[chain.side] = q_ref

    def apply(self, model: mujoco.MjModel, data: mujoco.MjData, rider_torque_nm: float) -> None:
        """
        Computes the six leg torques for this step and writes them to `qfrc_applied`.

        Args:
            model: Compiled model, read for the timestep.
            data: Simulation state, read for crank phase, pelvis height and leg state and
                written at the leg DOFs of `qfrc_applied`.
            rider_torque_nm: The rider's instantaneous crank torque (ripple included),
                split between the legs by `crank_torque_share`. Zero in motor mode with
                visual pedalling: the chain equality drags the crank and the legs are
                pure impedance followers.
        """
        if not self._legs:
            return
        dt = float(model.opt.timestep)
        phase = float(data.qpos[self._crank_qposadr])
        pelvis_z = float(data.qpos[self._pelvis_qposadr])
        # `rider_torque_nm` is instantaneous -- mean·ripple_shape(phase) -- while
        # `crank_torque_share` is a share of the *mean* whose two legs sum to
        # ripple_shape(phase). Dividing the shape back out keeps the delivered crank
        # torque equal to the command: f_t·L summed over legs = share_sum·mean·L/L
        # = rider_torque_nm, not mean·shape². The floor only matters at depth 1,
        # where shape reaches 0 at exactly the phases both shares are also 0.
        mean_nm = rider_torque_nm / max(ripple_shape(phase, self.ripple_depth), 1e-6)
        for chain, qposadrs, dofadrs in self._legs:
            # The chain's own crank position for the force bookkeeping: the rear arm is
            # 180 degrees away even though `solve_leg_qpos` absorbs that via the signed
            # crank length.
            side_phase = phase + (0.0 if chain.side == "front" else np.pi)
            q = np.asarray(data.qpos[qposadrs])
            qd = np.asarray(data.qvel[dofadrs])
            q_ref = _solve_or_current(chain, phase, pelvis_z, q)
            qd_ref = (q_ref - self._prev_q_ref.get(chain.side, q_ref)) / dt
            self._prev_q_ref[chain.side] = q_ref
            # Tangential pedal force: this leg's share of the *mean* torque over the
            # crank arm. t_hat is d(pedal)/d(phase) normalized: (-sin, -cos) in (x, z)
            # -- at phase 0 (front arm at 3 o'clock) it points straight down.
            share = crank_torque_share(side_phase, self.ripple_depth)
            f_t = share * mean_nm / self.crank_len_m
            F = f_t * np.array([-sin(side_phase), -cos(side_phase)])
            tau = leg_jacobian(chain, q, pelvis_z) @ F  # (3,2) @ (2,)
            tau += np.multiply(self.IMPEDANCE_KP, q_ref - q) \
                 + np.multiply(self.IMPEDANCE_KD, qd_ref - qd)
            # Gravity/Coriolis feedforward of the leg subtree. Without it the PD alone
            # fights the legs' weight: the steady-state sag lands on the welds, which is
            # both a tracking error and a parasitic crank torque no static pose can
            # balance (the wheel simply rolls away). `qfrc_bias` is the RNE bias term --
            # at these DOFs exactly the torque that holds the segments where they are.
            tau += data.qfrc_bias[dofadrs]
            data.qfrc_applied[dofadrs] = tau
            setattr(self, f"pedal_force_{chain.side}_n", float(np.linalg.norm(F)))


__all__ = ["LegDrive"]
