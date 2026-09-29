"""Crash classification from solved engine contacts, separate from tire grounding."""
import mujoco
import numpy as np


def physical_contact_crash(model, data, load_threshold_n=1.0):
    """Return an emergency cause without treating the catch plane as road.

    Call after the solve and before refreshing endpoint kinematics. Only crash
    geoms selected in the physical builder can produce a rider-ground contact.
    """
    if not np.isfinite(load_threshold_n) or load_threshold_n < 0:
        raise ValueError('contact threshold must be finite and nonnegative')
    wrench=np.zeros(6)
    for i in range(data.ncon):
        contact=data.contact[i]
        names=(model.geom(int(contact.geom1)).name,model.geom(int(contact.geom2)).name)
        if 'catch_plane' not in names and not ('terrain' in names and any(n.startswith('geom_rider_') for n in names)):
            continue
        mujoco.mj_contactForce(model,data,i,wrench)
        if wrench[0] <= load_threshold_n:
            continue
        if 'catch_plane' in names:
            return 'catch_plane_contact'
        return 'rider_ground_contact'
    return None
