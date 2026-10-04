"""Recover the planar wrench of one solved attachment equality.

Reads the constraint multipliers that belong to a single equality, builds the
relative Jacobian of the two attached bodies at one world point, and solves
for the wrench [Fx, Fz, My] (weld) or [Fx, Fz] (pin) that explains the
equality's generalized force. A rank-deficient or unexplained solve is a
measurement failure, never a zero-force reading.
"""
import mujoco
import numpy as np
from dataclasses import dataclass


@dataclass(frozen=True)
class AttachmentGeometry:
    """Attachment geometry captured at the incoming state before integration."""
    eq_id: int
    rider_jac: np.ndarray
    rider_columns: np.ndarray
    bike_jac: np.ndarray
    bike_columns: np.ndarray
    observable: bool
    normal: np.ndarray
    kind: str
    rotational: bool
    half_patch_m: float
    pull_direction: np.ndarray | None


@dataclass(frozen=True)
class AttachmentRaw:
    rider_jac: np.ndarray
    rider_qfrc: np.ndarray
    bike_jac: np.ndarray
    bike_qfrc: np.ndarray
    observable: bool
    normal: np.ndarray
    kind: str
    rotational: bool
    half_patch_m: float
    gap_m: float
    pull_direction: np.ndarray | None


def _body_geometry(model, data, body, point):
    jp = np.zeros((3, model.nv)); jr = np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jp, jr, np.asarray(point, dtype=float), body)
    jac = np.vstack((jp, jr))
    cols = np.flatnonzero(np.abs(jac).max(axis=0) > 0)
    if not cols.size:
        raise ValueError('attachment body carries no degrees of freedom')
    observable = bool(np.abs(jac[0, cols]).max() != 0 and np.abs(jac[2, cols]).max() != 0)
    return jac[:, cols].copy(), cols, observable


def _body_input(model, data, body, point, qfrc):
    jac, cols, observable = _body_geometry(model, data, body, point)
    return jac, qfrc[cols].copy(), cols, observable


def _attachment_points(model, data, eq_id, rider, bike, point, rotational):
    if not rotational and model.eq_type[eq_id] == mujoco.mjtEq.mjEQ_CONNECT:
        # A soft connect's force acts at each body's own anchor. Evaluating
        # both at the rider point invents a bike couple F x gap and falsely
        # rejects observable forces whenever the permitted residual is nonzero.
        return tuple(data.xpos[body]+data.xmat[body].reshape(3,3)@offset
            for body,offset in ((rider,model.eq_data[eq_id,:3]),
                                (bike,model.eq_data[eq_id,3:6])))
    return point,point


def prepare_attachment_geometry(model, data, eq_id, body_rider, body_bike, point, normal, kind,
                                 *, rotational, half_patch_m=0., pull_direction=None):
    """Capture all pose-dependent raw inputs before a physics step.

    The solved equality force is deliberately not read here: MuJoCo only has
    the interval's final multiplier after ``mj_step``. Capturing Jacobians and
    support geometry at the interval-start pose lets the runtime pair them with
    that multiplier later without rewriting ``qpos/qvel`` or running
    ``mj_kinematics``/``mj_comPos`` twice. The gap remains the post-step
    ``efc_pos`` value, matching the existing scalar sampler exactly.
    """
    rider_point,bike_point = _attachment_points(model,data,eq_id,body_rider,body_bike,point,rotational)
    jr, cr, obs_r = _body_geometry(model, data, body_rider, rider_point)
    jb, cb, obs_b = _body_geometry(model, data, body_bike, bike_point)
    if np.intersect1d(cr, cb).size:
        raise ValueError('attachment bodies share kinematic support')
    return AttachmentGeometry(int(eq_id), jr, cr.copy(), jb, cb.copy(), obs_r and obs_b,
        np.array(normal, copy=True), kind, rotational, float(half_patch_m),
        None if pull_direction is None else np.array(pull_direction, copy=True))


def attachment_raw_from_geometry(model, data, geometry, *, rows=None):
    """Combine pre-step geometry with the just-solved equality multiplier.

    Keep the validation contract of :func:`attachment_raw`: the fast path is
    allowed to change *when* the Jacobians are computed, not what constitutes
    an observable attachment measurement.
    """
    qfrc = equality_qfrc(model, data, geometry.eq_id)
    rider_qfrc = qfrc[geometry.rider_columns]
    bike_qfrc = qfrc[geometry.bike_columns]
    # Reconstruct spatial wrenches and retain the exact Newton-third-law and
    # planar-model checks used by the scalar sampler.
    rider_wrench, *_ = np.linalg.lstsq(geometry.rider_jac.T, rider_qfrc, rcond=1e-12)
    bike_wrench, *_ = np.linalg.lstsq(geometry.bike_jac.T, bike_qfrc, rcond=1e-12)
    if not np.allclose(geometry.rider_jac.T @ rider_wrench, rider_qfrc,
                       rtol=1e-8, atol=1e-8):
        raise ValueError('attachment wrench does not explain generalized force')
    if not np.allclose(geometry.bike_jac.T @ bike_wrench, bike_qfrc,
                       rtol=1e-8, atol=1e-8):
        raise ValueError('attachment wrench does not explain generalized force')
    if not geometry.observable:
        raise ValueError('in-plane attachment force is not observable')
    scale = max(1., float(np.linalg.norm(rider_wrench[:3])))
    if not np.allclose(bike_wrench, -rider_wrench, rtol=1e-4, atol=1e-4*scale):
        raise ValueError('attachment wrenches fail Newton third law')
    out_of_plane = np.array([rider_wrench[1], rider_wrench[3], rider_wrench[5]])
    if not np.all(np.abs(out_of_plane) <= 1e-6*scale):
        raise ValueError('attachment wrench leaves the planar model')
    if rows is None:
        from bike_sim.sim.ride.weld_pedals import equality_rows
        rows = equality_rows(data)
    idx = rows.get(geometry.eq_id)
    gap_m = 0. if idx is None or idx.size == 0 else float(np.linalg.norm(data.efc_pos[idx][:3]))
    return AttachmentRaw(geometry.rider_jac, rider_qfrc,
        geometry.bike_jac, bike_qfrc, geometry.observable,
        geometry.normal, geometry.kind, geometry.rotational, geometry.half_patch_m,
        gap_m, geometry.pull_direction)


def attachment_raw(model, data, eq_id, body_rider, body_bike, point, normal, kind,
                   *, rotational, half_patch_m=0., pull_direction=None, rows=None):
    """Copy all solved measurement inputs before endpoint forward overwrites EFC."""
    qfrc = equality_qfrc(model, data, eq_id)
    rider_point,bike_point = _attachment_points(model,data,eq_id,body_rider,body_bike,point,rotational)
    jr, qr, cr, obs_r = _body_input(model, data, body_rider, rider_point, qfrc)
    jb, qb, cb, obs_b = _body_input(model, data, body_bike, bike_point, qfrc)
    if np.intersect1d(cr, cb).size:
        raise ValueError('attachment bodies share kinematic support')
    if rows is None:
        from bike_sim.sim.ride.weld_pedals import equality_rows
        rows = equality_rows(data)
    idx = rows.get(eq_id)
    gap = 0. if idx is None or idx.size == 0 else float(np.linalg.norm(data.efc_pos[idx][:3]))
    return AttachmentRaw(jr, qr, jb, qb, obs_r and obs_b, np.array(normal, copy=True),
        kind, rotational, float(half_patch_m), gap,
        None if pull_direction is None else np.array(pull_direction, copy=True))


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


def body_wrench(model, data, qfrc_eq, body, point):
    """Spatial wrench [Fx, Fy, Fz, Mx, My, Mz] the equality applies to ``body``.

    Recovers the wrench at ``point`` from the equality's generalized force
    restricted to the body's own DOF support. Unlike a raw residual-space
    inversion this makes no assumption about the constraint's internal
    parametrization (the weld's sin(t/2) rotational residual and quaternion
    correction already live inside qfrc = J^T . lambda), and it is exact at
    the pose MuJoCo built efc_J -- the interval's start state.

    Returns (wrench, dof_columns). Raises ValueError when the generalized
    force cannot be explained on the body's DOF support.
    """
    jp = np.zeros((3, model.nv)); jr = np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jp, jr, np.asarray(point, dtype=float), body)
    jac = np.vstack((jp, jr))
    columns = np.flatnonzero(np.abs(jac).max(axis=0) > 0)
    if columns.size == 0:
        raise ValueError('attachment body carries no degrees of freedom')
    wrench, _, _, _ = np.linalg.lstsq(jac[:, columns].T, qfrc_eq[columns],
                                    rcond=1e-12)
    if not np.allclose(jac[:, columns].T @ wrench, qfrc_eq[columns],
                       rtol=1e-8, atol=1e-8):
        raise ValueError('attachment wrench does not explain generalized force')
    if np.abs(jac[0, columns]).max() == 0 or np.abs(jac[2, columns]).max() == 0:
        raise ValueError('in-plane attachment force is not observable')
    return wrench, columns


def attachment_sample(model, data, eq_id, body_rider, body_bike, point,
                      normal, kind, *, rotational, half_patch_m=0.,
                      pull_direction=None, rows=None):
    """Reduce one attachment's solved wrench to its physical budget sample.

    ``rows`` is an optional {eq_id: indices} map from a single efc pass; the
    geometric gap is the equality's translational residual norm. Forces are
    the support force on the rider at ``point`` (the patch center), projected
    on the support's own normal; ``moment_nm`` is the moment about y through
    that point. The rider and bike DOF supports must be disjoint, the two
    recovered wrenches must cancel (Newton's third law), and the out-of-plane
    components must vanish -- any failure is a measurement error, not a sample.
    """
    from bike_sim.physics.attachment_budget import AttachmentSample
    qfrc = equality_qfrc(model, data, eq_id)
    rider_point,bike_point = _attachment_points(model,data,eq_id,body_rider,body_bike,point,rotational)
    wrench, cols_rider = body_wrench(model, data, qfrc, body_rider, rider_point)
    wrench_bike, cols_bike = body_wrench(model, data, qfrc, body_bike, bike_point)
    if np.intersect1d(cols_rider, cols_bike).size:
        raise ValueError('attachment bodies share kinematic support')
    scale = max(1., float(np.linalg.norm(wrench[:3])))
    if not np.allclose(wrench_bike, -wrench, rtol=1e-4, atol=1e-4*scale):
        raise ValueError('attachment wrenches fail Newton third law')
    out_of_plane = np.array([wrench[1], wrench[3], wrench[5]])
    if not np.all(np.abs(out_of_plane) <= 1e-6*scale):
        raise ValueError('attachment wrench leaves the planar model')
    if rows is None:
        from bike_sim.sim.ride.weld_pedals import equality_rows
        rows = equality_rows(data)
    idx = rows.get(eq_id)
    residual = data.efc_pos if idx is None else data.efc_pos[idx]
    gap_m = 0. if idx is None or idx.size == 0 else float(
        np.linalg.norm(residual[:3]))
    return decompose_wrench(wrench, normal, kind, rotational=rotational,
        half_patch_m=half_patch_m, gap_m=gap_m, pull_direction=pull_direction)


def decompose_wrench(wrench, normal, kind, *, rotational, half_patch_m=0.,
                     gap_m=0., pull_direction=None):
    """Pure support-frame projection shared by scalar and batched recovery."""
    from bike_sim.physics.attachment_budget import AttachmentSample
    n = np.asarray(normal, dtype=float)
    if n.shape != (3,) or not np.isfinite(n).all():
        raise ValueError('attachment sample needs a finite 3D normal')
    normal_xz = n[[0, 2]]
    length = np.linalg.norm(normal_xz)
    if length < 1e-12:
        raise ValueError('attachment normal has no planar component')
    normal_xz = normal_xz/length
    tangent_xz = np.array([normal_xz[1], -normal_xz[0]])
    normal_n = float(wrench[0]*normal_xz[0] + wrench[2]*normal_xz[1])
    tangent_n = float(wrench[0]*tangent_xz[0] + wrench[2]*tangent_xz[1])
    moment_nm = float(wrench[4]) if rotational else 0.
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
        force_on_bike = -wrench[[0, 2]]
        if float(force_on_bike @ direction_xz) > 0.:
            pull_n = float(np.linalg.norm(force_on_bike))
    return AttachmentSample(kind, normal_n, tangent_n, moment_nm, gap_m,
                            pull_n=pull_n, half_patch_m=half_patch_m)
