"""Absolute, world-frame velocity at a point on a rotating wheel body."""

import mujoco
import numpy as np


def point_velocity(v_center: np.ndarray, omega_world: np.ndarray, offset: np.ndarray) -> np.ndarray:
    """Transport a world-frame linear velocity by ``omega x offset``."""
    vectors = tuple(np.asarray(value, dtype=float) for value in (v_center, omega_world, offset))
    if any(value.shape != (3,) or not np.isfinite(value).all() for value in vectors):
        raise ValueError("expected finite 3D vectors")
    return vectors[0] + np.cross(vectors[1], vectors[2])


def wheel_point_velocity(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_id: int,
    point_m: np.ndarray,
) -> np.ndarray:
    """Read the wheel body's absolute twist and transport it from its body origin.

    ``mj_objectVelocity(..., flg_local=0)`` gives world-axis angular and linear
    velocity at ``data.xpos[body_id]``. The wheel hinge ``qvel`` is only relative
    to its parent, so it cannot replace this twist for ground slip.
    """
    point = np.asarray(point_m, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError("expected a finite world contact point")
    velocity6 = np.zeros(6, dtype=float)
    mujoco.mj_objectVelocity(
        model, data, mujoco.mjtObj.mjOBJ_BODY, body_id, velocity6, 0
    )
    return point_velocity(velocity6[3:6], velocity6[:3], point - data.xpos[body_id])


__all__ = ["point_velocity", "wheel_point_velocity"]
