"""
Ride-Mode Seated Rider Force Path.

Writes the spring-damper forces that carry the seated rider's lumped masses into
`qfrc_applied`, one generalized force per rider slide joint, the same way `forces.py`
carries the fork and the shock (docs/RIDE.md section 7).

**Why not MJCF joint springs.** Two of the four interfaces are one-sided: a saddle and a
flat pedal can push on the rider but cannot pull them back down, so a rider who is thrown
upward leaves the saddle and comes back under gravity alone. MuJoCo's joint `stiffness` is
bilateral. Computing the forces here also gives telemetry the saddle load, the pedal loads,
the bar load and the saddle gap directly, without inferring them from joint states.

**Reactions are internal.** A generalized force on a slide joint acts between the joint's
child and parent bodies, so every newton that lifts the pelvis presses the frame down through
the saddle. Unlike the pitch stabilizer's moment (section 12, item 1), nothing here has a
missing reaction.

**Sign convention.** Every rider slide joint runs along the frame's +z, so a positive joint
value is the body rising relative to the bike, and a positive generalized force pushes it up.
Each spring is preloaded to carry its static load at zero joint travel, which is where the
pose solver placed the body: `preload_deflection_m` of travel *upward* unloads it.

`qfrc_applied` is never cleared by MuJoCo, so each rider DOF is assigned on every step, zero
included, for the same reason `virtual_rider.py` gives.
"""

from typing import Dict, List, Optional, Tuple

import mujoco

from bike_sim.physics.rider import RiderBody, SeatedPose


class _JointPath:
    """One rider body's spring-damper, resolved onto its joint addresses."""

    __slots__ = ("body", "qposadr", "dofadr", "force_n", "gap_m")

    def __init__(self, body: RiderBody, qposadr: int, dofadr: int) -> None:
        self.body = body
        self.qposadr = qposadr
        self.dofadr = dofadr
        self.force_n = 0.0
        self.gap_m = 0.0

    def compute(self, q: float, qd: float) -> float:
        """
        Spring-damper force for one joint state.

        Args:
            q: Joint travel, m, positive upward relative to the parent.
            qd: Joint velocity, m/s.

        Returns:
            The generalized force to apply, N. Records the contact gap for one-sided paths.
        """
        b = self.body
        deflection = b.preload_deflection_m - q
        if b.unilateral:
            if deflection <= 0.0:
                self.gap_m = -deflection
                self.force_n = 0.0
                return 0.0
            self.gap_m = 0.0
            # The damper cannot pull the body back onto the saddle or the pedal either.
            self.force_n = max(0.0, b.stiffness_n_m * deflection - b.damping_ns_m * qd)
            return self.force_n
        self.gap_m = 0.0
        self.force_n = b.stiffness_n_m * deflection - b.damping_ns_m * qd
        return self.force_n


class RiderForceApplier:
    """
    Applies the seated rider's interface and body springs to their slide coordinates.

    Constructed with ``pose=None`` -- the bike alone, or the lumped rider -- it is inert:
    `apply` does nothing and every load reads zero, so callers do not branch on the rider.
    """

    def __init__(self, model: mujoco.MjModel, pose: Optional[SeatedPose]) -> None:
        """
        Args:
            model: Compiled ride-mode model.
            pose: The seated pose the model was built from, or None for no seated rider.

        Raises:
            ValueError: If the model lacks a joint the pose names, or the joint is not an
                unlimited slide.
        """
        self.pose = pose
        self._paths: List[_JointPath] = []
        self._by_name: Dict[str, _JointPath] = {}
        if pose is None:
            return
        for body in pose.bodies:
            qposadr, dofadr = self._resolve_slide(model, body.joint)
            path = _JointPath(body, qposadr, dofadr)
            self._paths.append(path)
            self._by_name[body.name] = path

    @staticmethod
    def _resolve_slide(model: mujoco.MjModel, joint_name: str) -> Tuple[int, int]:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if jid < 0:
            raise ValueError(f"model has no joint '{joint_name}'; the seated rider's forces cannot be applied")
        if model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_SLIDE or bool(model.jnt_limited[jid]):
            raise ValueError(f"joint '{joint_name}' must be an unlimited slide for the rider force path")
        return int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid])

    @property
    def active(self) -> bool:
        """Whether a seated rider is present."""
        return bool(self._paths)

    def apply(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """
        Computes every rider spring-damper force from the current state and applies it.

        Args:
            model: Compiled model. Unused; kept for the shared writer signature.
            data: Simulation state, written at the rider DOFs of `qfrc_applied`.
        """
        for path in self._paths:
            q = float(data.qpos[path.qposadr])
            qd = float(data.qvel[path.dofadr])
            data.qfrc_applied[path.dofadr] = path.compute(q, qd)

    # --- telemetry -------------------------------------------------------------------

    def _force(self, name: str) -> float:
        path = self._by_name.get(name)
        return path.force_n if path is not None else 0.0

    @property
    def saddle_load_n(self) -> float:
        """Force the saddle exerts on the pelvis, N; zero when the rider has left the saddle."""
        return self._force("rider_pelvis")

    @property
    def saddle_gap_m(self) -> float:
        """Gap between the pelvis and the saddle, m; zero while seated."""
        path = self._by_name.get("rider_pelvis")
        return path.gap_m if path is not None else 0.0

    @property
    def torso_spring_n(self) -> float:
        """Force the pelvis exerts on the torso through the spine spring, N."""
        return self._force("rider_torso")

    @property
    def bar_load_n(self) -> float:
        """Force the handlebar exerts on the arms, N; negative when the rider pulls up."""
        return self._force("rider_arms")

    @property
    def pedal_load_front_n(self) -> float:
        """Force the front pedal exerts on the front leg, N."""
        return self._force("rider_leg_front")

    @property
    def pedal_load_rear_n(self) -> float:
        """Force the rear pedal exerts on the rear leg, N."""
        return self._force("rider_leg_rear")

    def interface_loads_n(self) -> Dict[str, float]:
        """Current load on each interface, N: ``saddle``, ``pedals``, ``bar``."""
        return {
            "saddle": self.saddle_load_n,
            "pedals": self.pedal_load_front_n + self.pedal_load_rear_n,
            "bar": self.bar_load_n,
        }


__all__ = ["RiderForceApplier"]
