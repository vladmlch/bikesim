"""External road/air interactions, distinct from relative bearing losses."""
import numpy as np
from bike_sim.physics.external_resistance import rolling_moment, drag_force
from bike_sim.sim.ride.physical_mapping import (
    resolve_id, body_angular_velocity, point_velocity, map_wrench,
)


class ExternalResistanceApplier:
    def __init__(self, model, config):
        import mujoco
        self.config = config
        self.frame = resolve_id(model, mujoco.mjtObj.mjOBJ_BODY, 'frame')
        self.wheels = {s:resolve_id(model, mujoco.mjtObj.mjOBJ_BODY, s+'_wheel')
                       for s in ('front', 'rear')}

    def compute_components(self, model, data, snapshots):
        cfg = self.config
        rolling = np.zeros(model.nv)
        for side, body in self.wheels.items():
            snapshot = snapshots[side]
            Fn = sum(p.normal_load_n for p in snapshot.patches if p.working_surface)
            axis = data.xmat[body].reshape(3, 3)[:, 1]
            omega = float(body_angular_velocity(model, data, body) @ axis)
            radius = snapshot.effective_radius_m
            if Fn > 0 and radius > 0:
                moment = rolling_moment(cfg.crr, Fn, radius, omega, cfg.rolling_taper_rad_s)
                rolling += map_wrench(model, data, body, data.xpos[body], np.zeros(3), moment*axis)
        point = data.xpos[self.frame] + data.xmat[self.frame].reshape(3, 3) @ cfg.point_body_m
        velocity = point_velocity(model, data, self.frame, point)
        relative = velocity-np.asarray(cfg.wind_world_mps)
        force = drag_force(relative, cfg.rho_kg_m3, cfg.cda_m2)
        return {'road_rolling':rolling,
                'aerodynamic':map_wrench(model, data, self.frame, point, force)}
