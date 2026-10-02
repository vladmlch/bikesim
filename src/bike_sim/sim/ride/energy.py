"""Mechanical accounting and whole-system momentum for the compiled MuJoCo model."""

from math import isfinite

import mujoco
import numpy as np


class EnergyLedger:
    """Residual of stored mechanical energy and independently assigned work channels.

    Active and signed external work increase stored energy; positive loss reduces it.
    Callers must supply elastic potential and all work/loss channels before interpreting a
    residual as a closed balance. Battery storage belongs to an electrical ledger.
    """

    def __init__(self, initial_energy_j: float) -> None:
        if not isfinite(initial_energy_j):
            raise ValueError("invalid initial mechanical energy")
        self.initial = initial_energy_j

    def residual(
        self,
        energy_j: float,
        active_work_j: float,
        external_work_j: float,
        loss_j: float,
    ) -> float:
        if not all(isfinite(x) for x in (energy_j, active_work_j, external_work_j, loss_j)):
            raise ValueError("non-finite energy ledger")
        if loss_j < 0:
            raise ValueError("dissipation cannot be negative")
        return energy_j - self.initial - active_work_j - external_work_j + loss_j


def mechanical_energy_terms(
    model: mujoco.MjModel, data: mujoco.MjData, *, elastic_energy_j: float
) -> tuple[float, float, float]:
    """Kinetic, gravitational and explicitly modelled elastic energy in joules.

    External springs are not included in MuJoCo's potential energy. The caller must
    provide their full stored energy rather than infer it from applied force power.
    """
    if not isfinite(elastic_energy_j) or elastic_energy_j < 0:
        raise ValueError("invalid elastic energy")
    mujoco.mj_forward(model, data)
    mass_matrix = np.empty((model.nv, model.nv))
    mujoco.mj_fullM(model, data, mass_matrix)
    kinetic = float(0.5 * data.qvel @ mass_matrix @ data.qvel)
    gravitational = float(
        -np.sum(model.body_mass[:, None] * data.xipos * model.opt.gravity)
    )
    return kinetic, gravitational, float(elastic_energy_j)


def system_momentum(
    model: mujoco.MjModel, data: mujoco.MjData
) -> tuple[np.ndarray, np.ndarray]:
    """Linear momentum and angular momentum about the moving compiled system CoM.

    Includes every physical body, its orbital contribution and its inertial-frame spin.
    Refreshes derived kinematics because `mj_step` can leave them at the input state.
    """
    masses = np.asarray(model.body_mass)
    total = float(masses.sum())
    if total <= 0:
        raise ValueError("no physical system mass")
    mujoco.mj_forward(model, data)
    com = (masses[:, None] * data.xipos).sum(axis=0) / total
    linear = np.zeros(3)
    angular = np.zeros(3)
    jp = np.zeros((3, model.nv))
    jr = np.zeros((3, model.nv))
    for body, mass in enumerate(masses):
        if mass == 0:
            continue
        mujoco.mj_jacBodyCom(model, data, jp, jr, body)
        velocity, omega = jp @ data.qvel, jr @ data.qvel
        rotation = data.ximat[body].reshape(3, 3)
        inertia = rotation @ np.diag(model.body_inertia[body]) @ rotation.T
        momentum = mass * velocity
        linear += momentum
        angular += inertia @ omega + np.cross(data.xipos[body] - com, momentum)
    return linear, angular


__all__ = ["EnergyLedger", "mechanical_energy_terms", "system_momentum"]
