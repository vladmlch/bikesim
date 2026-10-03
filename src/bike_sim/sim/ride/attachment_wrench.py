"""Recover the planar wrench of one solved attachment equality.

Reads the constraint multipliers that belong to a single equality, builds the
relative Jacobian of the two attached bodies at one world point, and solves
for the wrench [Fx, Fz, My] (weld) or [Fx, Fz] (pin) that explains the
equality's generalized force. A rank-deficient or unexplained solve is a
measurement failure, never a zero-force reading.
"""
import mujoco
import numpy as np


def equality_qfrc(model, data, eq_id: int) -> np.ndarray:
    n = data.nefc
    selected = ((data.efc_type[:n] == int(mujoco.mjtConstraint.mjCNSTR_EQUALITY))
                & (data.efc_id[:n] == eq_id))
    multipliers = np.where(selected, data.efc_force[:n], 0.0)
    qfrc = np.zeros(model.nv)
    mujoco.mj_mulJacTVec(model, data, qfrc, multipliers)
    return qfrc


def recover_wrench(relative_jacobian: np.ndarray,
                   qfrc: np.ndarray) -> np.ndarray:
    matrix = np.asarray(relative_jacobian, dtype=float).T
    target = np.asarray(qfrc, dtype=float)
    if not np.isfinite(matrix).all() or not np.isfinite(target).all():
        raise ValueError('nonfinite wrench input')
    wrench, _, rank, _ = np.linalg.lstsq(matrix, target, rcond=1e-12)
    if rank != matrix.shape[1]:
        raise ValueError('rank-deficient attachment Jacobian')
    if not np.allclose(matrix @ wrench, target, rtol=1e-8, atol=1e-8):
        raise ValueError('attachment wrench does not explain generalized force')
    return wrench


def relative_planar_jacobian(model, data, body_a, body_b, point, *,
                             rotational: bool) -> np.ndarray:
    """Rows of (J_a - J_b) at one world point, planar coordinates.

    Rows [x, z] of the translational difference always; with ``rotational``
    the world-y rotation difference is appended, so the recovered third wrench
    component is the moment about y through ``point`` -- the couple plus the
    moment arm of the recovered force at the patch center.
    """
    point = np.asarray(point, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError('invalid world point')
    if not (0 < body_a < model.nbody and 0 < body_b < model.nbody) or body_a == body_b:
        raise ValueError('relative Jacobian requires distinct physical bodies')
    jp_a, jr_a = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    jp_b, jr_b = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jp_a, jr_a, point, body_a)
    mujoco.mj_jac(model, data, jp_b, jr_b, point, body_b)
    translational = (jp_a - jp_b)[[0, 2], :]
    if not rotational:
        return translational
    return np.vstack((translational, (jr_a - jr_b)[[1], :]))


def attachment_wrench(model, data, eq_id, body_a, body_b, point, *,
                      rotational: bool) -> np.ndarray:
    """Solved planar wrench applied BY body_b ON body_a at ``point``."""
    jac = relative_planar_jacobian(model, data, body_a, body_b, point,
                                   rotational=rotational)
    return recover_wrench(jac, equality_qfrc(model, data, eq_id))


def attachment_sample(model, data, eq_id, body_rider, body_bike, point,
                      normal, kind, *, rotational, half_patch_m=0.,
                      pull_direction=None, rows=None):
    """Reduce one attachment's solved wrench to its physical budget sample.

    ``rows`` is an optional {eq_id: indices} map from a single efc pass; the
    geometric gap is the equality's translational residual norm. Forces are
    the support force on the rider, projected on the support's own normal.
    """
    from bike_sim.physics.attachment_budget import AttachmentSample
    jac = relative_planar_jacobian(model, data, body_rider, body_bike, point,
                                   rotational=rotational)
    wrench = recover_wrench(jac, equality_qfrc(model, data, eq_id))
    n = np.asarray(normal, dtype=float)
    if n.shape != (3,) or not np.isfinite(n).all():
        raise ValueError('attachment sample needs a finite 3D normal')
    normal_xz = n[[0, 2]]
    length = np.linalg.norm(normal_xz)
    if length < 1e-12:
        raise ValueError('attachment normal has no planar component')
    normal_xz = normal_xz/length
    tangent_xz = np.array([normal_xz[1], -normal_xz[0]])
    normal_n = float(wrench[0]*normal_xz[0] + wrench[1]*normal_xz[1])
    tangent_n = float(wrench[0]*tangent_xz[0] + wrench[1]*tangent_xz[1])
    moment_nm = float(wrench[2]) if rotational else 0.
    if rows is None:
        rows = equality_rows(data)
    idx = rows.get(eq_id)
    residual = data.efc_pos if idx is None else data.efc_pos[idx]
    gap_m = 0. if idx is None or idx.size == 0 else float(
        np.linalg.norm(residual[:3]))
    pull_n = 0.
    if pull_direction is not None:
        direction = np.asarray(pull_direction, dtype=float)
        if direction.shape != (3,) or not np.isfinite(direction).all():
            raise ValueError('pull direction must be a finite 3D vector')
        direction_xz = direction[[0, 2]]
        length = np.linalg.norm(direction_xz)
        if length < 1e-12:
            raise ValueError('pull direction has no planar component')
        direction_xz = direction_xz/length
        force_on_bike = -wrench[:2]
        if float(force_on_bike @ direction_xz) > 0.:
            pull_n = float(np.linalg.norm(wrench[:2]))
    return AttachmentSample(kind, normal_n, tangent_n, moment_nm, gap_m,
                            pull_n=pull_n, half_patch_m=half_patch_m)
