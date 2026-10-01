"""Solved weld reactions for rider bodies rigidly attached to the bike.

With articulated.pedal_attachment = 'weld' the MJCF builder emits one `weld`
equality per foot (body1 = rider_foot_*, body2 = pedal_*); with
articulated.saddle_attachment = 'weld' it emits `weld_saddle`
(body1 = rider_pelvis, body2 = frame); with
articulated.grip_attachment = 'weld' it emits `connect_grip`
(body1 = rider_forearm_pair, body2 = steer). This module reads the solved
constraint multipliers and reports them in the conventions
RiderContactApplier uses for pad supports.

Conventions (pinned by tests): a weld's six EFC rows are [x,y,z translation,
3 rotational]; the translational multipliers are the world-frame force applied
BY the weld ON body1 (the rider body); the equal-and-opposite force acts on
body2 (the bike body). efc_pos rows 0-2 are the world-frame translation
residual.
"""
import numpy as np
import mujoco

from bike_sim.sim.ride.physical_mapping import resolve_id

SIDES = ('front', 'rear')
_EQUALITY = mujoco.mjtConstraint.mjCNSTR_EQUALITY


class PedalWelds:
    """Per-side weld reaction reader; reads the last solved constraint state."""

    def __init__(self, model):
        self.eq_ids = {side: resolve_id(model, mujoco.mjtObj.mjOBJ_EQUALITY,
                                        f'weld_foot_{side}') for side in SIDES}
        self.crank_dof = int(model.joint('crank_spin').dofadr[0])

    @staticmethod
    def _mask(data, eq_id):
        n = data.nefc
        return ((data.efc_type[:n] == _EQUALITY)
                & (data.efc_id[:n] == eq_id))

    def force_on_rider_n(self, model, data, side):
        """World force the weld applies to the foot, in Newtons.

        +z is the pedal pushing the foot up (compressive). A negative z is the
        weld retaining the foot — the clipless pull the pad model cannot make.
        """
        mask = self._mask(data, self.eq_ids[side])
        if not np.any(mask):
            return np.zeros(3)
        lam = data.efc_force[:data.nefc][mask]
        return np.asarray(lam[:3], dtype=float)

    def translation_residual_m(self, model, data, side):
        """Norm of the weld's positional residual — actual foot/pedal mismatch."""
        mask = self._mask(data, self.eq_ids[side])
        if not np.any(mask):
            return 0.
        pos = data.efc_pos[:data.nefc][mask]
        return float(np.linalg.norm(pos[:3]))

    def delivered_crank_torque_nm(self, model, data):
        """Total weld torque about crank_spin; + spins the cranks forward."""
        if data.nefc == 0:
            return 0.
        multipliers = np.zeros(data.nefc)
        nefc = data.nefc
        for eq in self.eq_ids.values():
            mask = self._mask(data, eq)
            multipliers[mask] = data.efc_force[:nefc][mask]
        qfrc = np.zeros(model.nv)
        mujoco.mj_mulJacTVec(model, data, qfrc, multipliers)
        return float(qfrc[self.crank_dof])


class SaddleWeld:
    """Solved weld reaction for a pelvis rigidly attached at the saddle.

    The weld can pull down, which the unilateral saddle pad cannot: the
    reported force is the signed constraint reaction on the pelvis.
    """

    def __init__(self, model):
        self.eq_id = resolve_id(model, mujoco.mjtObj.mjOBJ_EQUALITY,
                                'weld_saddle')

    def force_on_rider_n(self, model, data):
        """World force the weld applies to the pelvis, in Newtons."""
        mask = PedalWelds._mask(data, self.eq_id)
        if not np.any(mask):
            return np.zeros(3)
        lam = data.efc_force[:data.nefc][mask]
        return np.asarray(lam[:3], dtype=float)

    def translation_residual_m(self, model, data):
        """Norm of the weld's positional residual — actual pelvis/frame mismatch."""
        mask = PedalWelds._mask(data, self.eq_id)
        if not np.any(mask):
            return 0.
        pos = data.efc_pos[:data.nefc][mask]
        return float(np.linalg.norm(pos[:3]))


class GripConnect:
    """Solved connect reaction for a hand pinned to the handlebar.

    A `connect` equality contributes three translational rows: the reported
    force is the signed constraint reaction on the forearm, which the
    releasable spring grip cannot produce once disabled.
    """

    def __init__(self, model):
        self.eq_id = resolve_id(model, mujoco.mjtObj.mjOBJ_EQUALITY,
                                'connect_grip')

    def force_on_rider_n(self, model, data):
        """World force the pin applies to the hand, in Newtons."""
        mask = PedalWelds._mask(data, self.eq_id)
        if not np.any(mask):
            return np.zeros(3)
        lam = data.efc_force[:data.nefc][mask]
        return np.asarray(lam[:3], dtype=float)

    def translation_residual_m(self, model, data):
        """Norm of the pin's positional residual — actual hand/bar mismatch."""
        mask = PedalWelds._mask(data, self.eq_id)
        if not np.any(mask):
            return 0.
        pos = data.efc_pos[:data.nefc][mask]
        return float(np.linalg.norm(pos[:3]))
