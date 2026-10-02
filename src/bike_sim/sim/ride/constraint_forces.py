"""Post-step generalized forces produced by MuJoCo joint limits."""

from dataclasses import dataclass
from math import isfinite
from types import MappingProxyType
from typing import Mapping

import mujoco
import numpy as np

from bike_sim.sim.ride.telemetry_v2 import _frozen_copy


def shock_joint_limit_qfrc(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """Map only solved ``shock_stroke`` joint-limit rows through the EFC Jacobian."""
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "shock_stroke")
    if joint_id < 0:
        raise ValueError("model has no shock_stroke joint")
    qfrc = np.zeros(model.nv)
    nefc = data.nefc
    if nefc == 0:
        return qfrc
    selected = (
        (data.efc_type[:nefc] == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)
        & (data.efc_id[:nefc] == joint_id)
    )
    if np.any(selected):
        multipliers = np.zeros(nefc)
        multipliers[selected] = data.efc_force[:nefc][selected]
        mujoco.mj_mulJacTVec(model, data, qfrc, multipliers)
    return qfrc


@dataclass(frozen=True)
class ConstraintForceSnapshot:
    """Solved forces captured after a step for its just-finished time interval."""

    interval_start_s: float
    interval_end_s: float
    qvel_start: np.ndarray
    components: Mapping[str, np.ndarray]

    def __post_init__(self) -> None:
        if (
            not isfinite(self.interval_start_s)
            or not isfinite(self.interval_end_s)
            or self.interval_end_s <= self.interval_start_s
        ):
            raise ValueError("invalid constraint interval")
        qvel = np.array(self.qvel_start, dtype=float, copy=True)
        if qvel.ndim != 1 or not np.isfinite(qvel).all():
            raise ValueError("invalid constraint velocity")
        object.__setattr__(self, "qvel_start", _frozen_copy(qvel))
        copied: dict[str, np.ndarray] = {}
        for name, force in self.components.items():
            value = np.array(force, dtype=float, copy=True)
            if value.shape != qvel.shape or not np.isfinite(value).all():
                raise ValueError("invalid constraint force")
            copied[name] = _frozen_copy(value)
        object.__setattr__(self, "components", MappingProxyType(copied))

    def powers_w(self) -> dict[str, float]:
        """Estimate interval power from solved force and incoming velocity."""
        return {
            name: float(force @ self.qvel_start)
            for name, force in self.components.items()
        }


__all__ = ["ConstraintForceSnapshot", "shock_joint_limit_qfrc"]
