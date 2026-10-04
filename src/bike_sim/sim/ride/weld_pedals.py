"""Solved weld reactions for rider bodies rigidly attached to the bike.

With articulated.pedal_attachment = 'weld' the MJCF builder emits one `weld`
equality per foot (body1 = rider_foot_*, body2 = pedal_*); with
articulated.saddle_attachment = 'weld' it emits `weld_saddle`
(body1 = rider_pelvis, body2 = frame), while the reference 'pin' emits
`connect_saddle` between the same bodies; with
articulated.grip_attachment = 'connect' it emits one `connect_grip_<side>`
per hand (body1 = rider_forearm_<side>, body2 = steer). This module reads the solved
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


def equality_rows(data):
    """Map equality id -> efc row indices from a single pass over the arena.

    One flatnonzero over efc_type replaces the per-reader mask scans. Rows are
    grouped by efc_id and returned in ascending order per group, exactly as a
    boolean mask would select them. The layout is re-derived on every call:
    efc membership changes each step as contacts and limits activate, so a map
    must never be reused across calls or shared between applier copies.
    """
    n = data.nefc
    rows = np.flatnonzero(data.efc_type[:n] == _EQUALITY)
    if rows.size == 0:
        return {}
    ids = data.efc_id[:n][rows]
    # A stable sort keeps each group's rows in mask order even if MuJoCo ever
    # emitted an equality's rows in more than one contiguous block.
    order = np.argsort(ids, kind='stable')
    ids, rows = ids[order], rows[order]
    bounds = np.flatnonzero(np.diff(ids)) + 1
    groups = np.split(rows, bounds)
    keys = ids[np.r_[0, bounds]]
    return {int(eq_id): group for eq_id, group in zip(keys, groups)}


def _eq_rows(data, eq_id, rows):
    """Row indices of one equality: a map lookup, or a fresh mask scan."""
    if rows is not None:
        return rows.get(eq_id)
    return np.flatnonzero(PedalWelds._mask(data, eq_id))


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

    def force_on_rider_n(self, model, data, side, rows=None):
        """World force the weld applies to the foot, in Newtons.

        +z is the pedal pushing the foot up (compressive). A negative z is the
        weld retaining the foot — the clipless pull the pad model cannot make.

        `rows` is an optional equality_rows(data) map for this same mjData;
        when omitted the efc arena is scanned for this equality alone.
        """
        idx = _eq_rows(data, self.eq_ids[side], rows)
        if idx is None or idx.size == 0:
            return np.zeros(3)
        lam = data.efc_force[idx]
        return np.asarray(lam[:3], dtype=float)

    def translation_residual_m(self, model, data, side, rows=None):
        """Norm of the weld's positional residual — actual foot/pedal mismatch."""
        idx = _eq_rows(data, self.eq_ids[side], rows)
        if idx is None or idx.size == 0:
            return 0.
        pos = data.efc_pos[idx]
        return float(np.linalg.norm(pos[:3]))

    def delivered_crank_torque_nm(self, model, data, rows=None):
        """Total weld torque about crank_spin; + spins the cranks forward."""
        if data.nefc == 0:
            return 0.
        multipliers = np.zeros(data.nefc)
        for eq in self.eq_ids.values():
            idx = _eq_rows(data, eq, rows)
            if idx is not None:
                multipliers[idx] = data.efc_force[idx]
        qfrc = np.zeros(model.nv)
        mujoco.mj_mulJacTVec(model, data, qfrc, multipliers)
        return float(qfrc[self.crank_dof])


class SpindlePins(PedalWelds):
    """Three-row foot connects at the spindle; no free couple is transmitted.

    The native connect residual is body1 anchor minus body2 anchor in world
    coordinates. Its translational multipliers therefore apply to the rider
    with the same sign as the native body1 point Jacobian.
    """

    def __init__(self, model, sides=SIDES):
        self.eq_ids = {side: resolve_id(model, mujoco.mjtObj.mjOBJ_EQUALITY,
                                       f'connect_foot_{side}') for side in sides}
        self.crank_dof = int(model.joint('crank_spin').dofadr[0])


class SaddleWeld:
    """Solved weld reaction for a pelvis rigidly attached at the saddle.

    The weld can pull down, which the unilateral saddle pad cannot: the
    reported force is the signed constraint reaction on the pelvis.
    """

    def __init__(self, model):
        self.eq_id = resolve_id(model, mujoco.mjtObj.mjOBJ_EQUALITY,
                                'weld_saddle')

    def force_on_rider_n(self, model, data, rows=None):
        """World force the weld applies to the pelvis, in Newtons."""
        idx = _eq_rows(data, self.eq_id, rows)
        if idx is None or idx.size == 0:
            return np.zeros(3)
        lam = data.efc_force[idx]
        return np.asarray(lam[:3], dtype=float)

    def translation_residual_m(self, model, data, rows=None):
        """Norm of the weld's positional residual — actual pelvis/frame mismatch."""
        idx = _eq_rows(data, self.eq_id, rows)
        if idx is None or idx.size == 0:
            return 0.
        pos = data.efc_pos[idx]
        return float(np.linalg.norm(pos[:3]))


class SaddlePin:
    """Solved connect reaction for a pelvis pinned at the saddle.

    Unlike the legacy weld the pin contributes three translational rows only:
    it cannot carry a saddle couple, so the pelvis keeps a free pitch and the
    reported force is the signed point reaction on the pelvis.
    """

    def __init__(self, model):
        self.eq_id = resolve_id(model, mujoco.mjtObj.mjOBJ_EQUALITY,
                                'connect_saddle')

    def force_on_rider_n(self, model, data, rows=None):
        """World force the pin applies to the pelvis, in Newtons."""
        idx = _eq_rows(data, self.eq_id, rows)
        if idx is None or idx.size == 0:
            return np.zeros(3)
        lam = data.efc_force[idx]
        return np.asarray(lam[:3], dtype=float)

    def translation_residual_m(self, model, data, rows=None):
        """Norm of the pin's positional residual — actual pelvis/frame mismatch."""
        idx = _eq_rows(data, self.eq_id, rows)
        if idx is None or idx.size == 0:
            return 0.
        pos = data.efc_pos[idx]
        return float(np.linalg.norm(pos[:3]))


class GripConnect:
    """Solved connect reaction for a hand pinned to the handlebar.

    A `connect` equality contributes three translational rows: the reported
    force is the signed constraint reaction on the forearm, which the
    releasable spring grip cannot produce once disabled.
    """

    def __init__(self, model, side):
        self.side = side
        self.eq_id = resolve_id(model, mujoco.mjtObj.mjOBJ_EQUALITY,
                                f'connect_grip_{side}')

    def force_on_rider_n(self, model, data, rows=None):
        """World force the pin applies to the hand, in Newtons."""
        idx = _eq_rows(data, self.eq_id, rows)
        if idx is None or idx.size == 0:
            return np.zeros(3)
        lam = data.efc_force[idx]
        return np.asarray(lam[:3], dtype=float)

    def translation_residual_m(self, model, data, rows=None):
        """Norm of the pin's positional residual — actual hand/bar mismatch."""
        idx = _eq_rows(data, self.eq_id, rows)
        if idx is None or idx.size == 0:
            return 0.
        pos = data.efc_pos[idx]
        return float(np.linalg.norm(pos[:3]))
