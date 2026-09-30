"""World-frame force helpers with explicit virtual-work conventions."""
import numpy as np
from bike_sim.physics.checks import array


def resolve_id(model, kind, name):
    import mujoco
    result = mujoco.mj_name2id(model, kind, name)
    if result < 0:
        raise ValueError(f'model has no {name!r}')
    return int(result)


def point_jacobian(model, data, body_id, point):
    import mujoco
    point = array(point, 'world point', (3,))
    if not 0 < body_id < model.nbody:
        raise ValueError('point Jacobian requires a physical body')
    jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jp, jr, point, body_id)
    return jp, jr


def point_jacobian_into(model, data, body_id, point, jp):
    """Fill a caller-provided (3, nv) translational Jacobian at a world point.

    The scratch-buffer form of `point_jacobian` for per-step call sites: the
    rotational half is skipped, and the caller owns the buffer so repeated
    evaluations do not allocate.
    """
    import mujoco
    point = np.asarray(point, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError('invalid world point')
    if not 0 < body_id < model.nbody:
        raise ValueError('point Jacobian requires a physical body')
    mujoco.mj_jac(model, data, jp, None, point, body_id)


def relative_point_jacobian(model, data, body_a, body_b, point, jac_a, jac_b):
    """Jacobian difference of one world point on two bodies.

    Fills ``jac_a`` with ``J_a - J_b`` and returns it; ``jac_b`` is scratch.
    ``(J_a - J_b) @ qvel`` is the relative point velocity, and
    ``(J_a - J_b).T @ f`` is the generalized force of applying ``f`` to
    ``body_a`` with its Newton-pair reaction ``-f`` on ``body_b`` at the same
    point -- exactly what two `mj_applyFT` calls compute, without rebuilding
    either Jacobian.
    """
    import mujoco
    point = np.asarray(point, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError('invalid world point')
    if not (0 < body_a < model.nbody and 0 < body_b < model.nbody) or body_a == body_b:
        raise ValueError('relative Jacobian requires distinct physical bodies')
    mujoco.mj_jac(model, data, jac_a, None, point, body_a)
    mujoco.mj_jac(model, data, jac_b, None, point, body_b)
    jac_a -= jac_b
    return jac_a


def point_velocity(model, data, body_id, point):
    jp, _ = point_jacobian(model, data, body_id, point)
    return jp @ data.qvel


def body_angular_velocity(model, data, body_id):
    _, jr = point_jacobian(model, data, body_id, data.xpos[body_id])
    return jr @ data.qvel


def map_wrench(model, data, body_id, point, force, torque=None):
    import mujoco
    p = array(point, 'world point', (3,))
    f = array(force, 'world force', (3,))
    t = np.zeros(3) if torque is None else array(torque, 'world torque', (3,))
    if not 0 < body_id < model.nbody:
        raise ValueError('wrench must act on a physical body')
    qfrc = np.zeros(model.nv)
    mujoco.mj_applyFT(model, data, f, t, p, body_id, qfrc)
    return qfrc
