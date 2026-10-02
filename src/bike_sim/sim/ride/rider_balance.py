"""Frame-relative balance intent realized only through internal limb actuators."""
import numpy as np
from bike_sim.sim.ride.physical_mapping import point_velocity


def balance_force_request(controller, model, data, posture):
    """Bounded planar support-wrench target, not an applied generalized force.

    A neutral rider also needs a fore/aft balance reference: foot-following IK
    alone follows a falling pelvis and cannot recover it. The reference is
    attached to the moving bicycle, so translating the whole system leaves
    the request unchanged. Loss of supports remains the contact allocator's
    infeasibility, never a reason to push the floating body from the world.
    """
    cfg = controller.config
    offset = (0., 0.) if posture.pelvis_offset_m is None else posture.pelvis_offset_m
    wanted = data.xpos[controller.frame] + data.xmat[controller.frame].reshape(3, 3) @ (
        controller.pose.hip + np.array([offset[0], 0., offset[1]]))
    actual = data.xpos[controller.pelvis]
    speed = (point_velocity(model, data, controller.frame, wanted) -
             point_velocity(model, data, controller.pelvis, actual))
    force = cfg.posture_translation_k_n_m*(wanted-actual) + cfg.posture_translation_d_ns_m*speed
    force[1] = 0.
    norm = float(np.linalg.norm(force))
    if norm > cfg.posture_translation_limit_n:
        force *= cfg.posture_translation_limit_n/norm
    return force
