"""Mass properties read from the compiled model at the current pose."""

import mujoco
import numpy as np


def compiled_center_of_mass(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """Return the current world-space center of mass after kinematics are updated."""
    masses = np.asarray(model.body_mass)
    total = float(masses.sum())
    if not np.isfinite(total) or total <= 0:
        raise ValueError("model has no positive physical mass")
    return np.sum(masses[:, None] * data.xipos, axis=0) / total


def static_contact_loads(
    model: mujoco.MjModel, data: mujoco.MjData, front_geom_id: int, rear_geom_id: int
) -> tuple[float, float]:
    """Compute static normal loads from the current contact locations after sag."""
    contact_x: dict[int, list[float]] = {front_geom_id: [], rear_geom_id: []}
    for contact in data.contact:
        for geom_id in (contact.geom1, contact.geom2):
            if geom_id in contact_x:
                contact_x[geom_id].append(float(contact.pos[0]))
    if any(not values for values in contact_x.values()):
        raise ValueError("both wheel contact points are required for static load")
    front_x = float(np.mean(contact_x[front_geom_id]))
    rear_x = float(np.mean(contact_x[rear_geom_id]))
    wheelbase = front_x - rear_x
    if wheelbase <= 0:
        raise ValueError("front contact must be ahead of rear contact")
    com_x = float(compiled_center_of_mass(model, data)[0])
    total_weight = float(model.body_mass.sum()) * abs(float(model.opt.gravity[2]))
    front_load = total_weight * (com_x - rear_x) / wheelbase
    return front_load, total_weight - front_load
