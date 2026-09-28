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
