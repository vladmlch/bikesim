"""External road/air interactions, distinct from relative bearing losses."""
import numpy as np
from bike_sim.physics.external_resistance import _rolling_moment, _drag_force
from bike_sim.sim.ride.physical_mapping import (
    resolve_id, point_jacobian_into,
)


class ExternalResistanceApplier:
    def __init__(self, model, config):
        import mujoco
        self.config = config
        self.frame = resolve_id(model, mujoco.mjtObj.mjOBJ_BODY, 'frame')
        self.wheels = {s:resolve_id(model, mujoco.mjtObj.mjOBJ_BODY, s+'_wheel')
                       for s in ('front', 'rear')}
        # Per-step Jacobian scratch shared by the rolling and drag maps.
        self._jr = np.empty((3, model.nv))
        self._jp = np.empty((3, model.nv))

    def compute_components(self, model, data, snapshots):
        import mujoco
        cfg = self.config
        rolling = np.zeros(model.nv)
        for side, body in self.wheels.items():
            snapshot = snapshots[side]
            Fn = sum(p.normal_load_n for p in snapshot.patches if p.working_surface)
            axis = data.xmat[body].reshape(3, 3)[:, 1]
            # The rotational Jacobian maps a scalar torque about the wheel axis
            # to generalized force, and the same row yields omega about the axis.
            mujoco.mj_jac(model, data, None, self._jr, data.xpos[body], body)
            row = self._jr.T @ axis
            omega = float(row @ data.qvel)
            radius = snapshot.effective_radius_m
            if Fn > 0 and radius > 0:
                moment = _rolling_moment(cfg.crr, Fn, radius, omega, cfg.rolling_taper_rad_s)
                rolling += moment*row
        point = data.xpos[self.frame] + data.xmat[self.frame].reshape(3, 3) @ cfg.point_body_m
        point_jacobian_into(model, data, self.frame, point, self._jp)
        velocity = self._jp @ data.qvel
        relative = velocity-np.asarray(cfg.wind_world_mps)
        force = _drag_force(relative, cfg.rho_kg_m3, cfg.cda_m2)
        return {'road_rolling':rolling, 'aerodynamic':self._jp.T @ force}
