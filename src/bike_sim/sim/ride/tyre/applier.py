"""MuJoCo boundary for the pneumatic front and rear tyres.

This is the sole writer of the two wheel rows in ``data.xfrc_applied``.  The tyre kernels
remain pure; this module resolves body ids, reads world-frame wheel kinematics, and applies
the patch forces at their road centroids.
"""

from dataclasses import dataclass
from typing import Dict

import mujoco
import numpy as np

from bike_sim.physics.tyre import TyreConfig
from bike_sim.sim.ride.tyre.geometry import RoadProfile
from bike_sim.sim.ride.tyre.model import PneumaticTyre, WheelOutputs
from bike_sim.terrain.surface import SurfaceMap


@dataclass(frozen=True)
class _WheelBinding:
    """Compiled MuJoCo handles for one tyre."""

    name: str
    body_id: int
    spin_dofadr: int
    contact_geom_id: int


class TyreForceApplier:
    """Evaluates both pneumatic tyres and assigns their wheel-body external wrenches."""

    def __init__(
        self,
        model: mujoco.MjModel,
        config: TyreConfig,
        profile: RoadProfile,
        surface_map: SurfaceMap,
    ) -> None:
        """Resolves both wheels and creates their pure tyre models.

        Args:
            model: Compiled ride-mode model.
            config: Pneumatic model, fidelity tier, pressures and optional surface override.
            profile: Road profile in world metres, identical to the heightfield source.
            surface_map: Track surface lookup, overridden by ``config.surface`` when set.

        Raises:
            ValueError: If the config selects ``sphere`` or model geometry does not match the
                configured tyre radii.
        """
        if not config.pneumatic:
            raise ValueError("TyreForceApplier requires tyre_model='pneumatic'")
        self.config = config
        self.profile = profile
        self.surface_map = (
            SurfaceMap.uniform(config.surface) if config.surface is not None else surface_map
        )
        self.tier = config.tier_spec
        if self.tier.name == "detailed":
            model.opt.timestep = self.tier.timestep_s

        self.front_wheel = _resolve_wheel(model, "front_wheel", "front_wheel_spin",
                                          "geom_front_contact", config.front.outer_radius_mm / 1000.0)
        self.rear_wheel = _resolve_wheel(model, "rear_wheel", "rear_wheel_spin",
                                         "geom_rear_contact", config.rear.outer_radius_mm / 1000.0)
        self.front_tyre = PneumaticTyre(config.front, self.tier)
        self.rear_tyre = PneumaticTyre(config.rear, self.tier)
        self._velocity6 = np.zeros(6, dtype=float)
        self.front_outputs = WheelOutputs.empty(config.front)
        self.rear_outputs = WheelOutputs.empty(config.rear)

    @property
    def outputs(self) -> Dict[str, WheelOutputs]:
        """Most recent per-wheel outputs, keyed by ``front`` and ``rear``."""
        return {"front": self.front_outputs, "rear": self.rear_outputs}

    def apply(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Evaluates both tyres and assigns both wheel-body `xfrc_applied` rows.

        `mj_objectVelocity` is queried in world axes. Its angular component includes the
        carrier pitch; with the builder's +Y axle axis, `−ω_y` is the forward-positive spin
        used by the tyre equations. The joint's `qvel` is intentionally not used here.

        Args:
            model: The same compiled model used at construction.
            data: Current MuJoCo state; the two wheel wrench rows are overwritten every call.
        """
        self.front_outputs = self._apply_wheel(
            model, data, self.front_wheel, self.front_tyre
        )
        self.rear_outputs = self._apply_wheel(
            model, data, self.rear_wheel, self.rear_tyre
        )

    def _apply_wheel(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        binding: _WheelBinding,
        tyre: PneumaticTyre,
    ) -> WheelOutputs:
        self._velocity6.fill(0.0)
        mujoco.mj_objectVelocity(
            model,
            data,
            mujoco.mjtObj.mjOBJ_BODY,
            binding.body_id,
            self._velocity6,
            0,
        )
        hub_position = data.xpos[binding.body_id].copy()
        hub_velocity = self._velocity6[3:6].copy()
        omega_forward = -float(self._velocity6[1])
        outputs = tyre.evaluate(
            hub_position_world_m=hub_position,
            hub_velocity_world_mps=hub_velocity,
            omega_forward_radps=omega_forward,
            road=self.profile,
            surface_map=self.surface_map,
            dt_s=float(model.opt.timestep),
        )

        force_world_n = outputs.force_world_n
        torque_world_nm = np.zeros(3, dtype=float)
        body_com_world = data.xipos[binding.body_id]
        for patch in outputs.patches:
            torque_world_nm += np.cross(
                patch.centroid_world_m - body_com_world,
                patch.force_world_n,
            )
        # MuJoCo does not clear this array. Assign zero rows too, so an airborne wheel cannot
        # retain the last contact wrench.
        data.xfrc_applied[binding.body_id, :] = np.concatenate((force_world_n, torque_world_nm))
        return outputs

    def reset(self, data: mujoco.MjData | None = None) -> None:
        """Clears both tyre histories and, when supplied, both applied wrench rows."""
        self.front_tyre.reset()
        self.rear_tyre.reset()
        self.front_outputs = WheelOutputs.empty(self.front_tyre.tyre)
        self.rear_outputs = WheelOutputs.empty(self.rear_tyre.tyre)
        if data is not None:
            data.xfrc_applied[self.front_wheel.body_id, :] = 0.0
            data.xfrc_applied[self.rear_wheel.body_id, :] = 0.0


def _resolve_wheel(
    model: mujoco.MjModel,
    body_name: str,
    spin_joint_name: str,
    contact_geom_name: str,
    expected_radius_m: float,
) -> _WheelBinding:
    """Resolves and cross-checks the wheel body, spin DOF and radius sphere."""
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, spin_joint_name)
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, contact_geom_name)
    for object_id, object_name in (
        (body_id, body_name),
        (joint_id, spin_joint_name),
        (geom_id, contact_geom_name),
    ):
        if object_id < 0:
            raise ValueError(f"model has no {object_name!r}; pneumatic tyre force path cannot run")
    if model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_SPHERE:
        raise ValueError(f"contact geom {contact_geom_name!r} must remain a sphere")
    actual_radius_m = float(model.geom_size[geom_id][0])
    if not np.isclose(actual_radius_m, expected_radius_m, rtol=0.0, atol=1e-6):
        raise ValueError(
            f"contact geom {contact_geom_name!r} radius {actual_radius_m:.6f} m does not "
            f"match tyre radius {expected_radius_m:.6f} m"
        )
    return _WheelBinding(
        name=body_name,
        body_id=int(body_id),
        spin_dofadr=int(model.jnt_dofadr[joint_id]),
        contact_geom_id=int(geom_id),
    )


__all__ = ["TyreForceApplier"]
