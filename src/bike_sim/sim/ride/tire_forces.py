"""A single, explicitly selected compliant wheel/road force path.

The geometry comes from the compiled heightfield raster, not an independent
smooth track function. Calls require current MuJoCo kinematics. Advancing the
brush state twice at the same timestamp is an error; telemetry is read-only.
"""
from dataclasses import dataclass
from math import isfinite
import copy
import numpy as np
from bike_sim.physics.checks import array, derived, derived_array, scalar
from bike_sim.physics.tire import TireSpec, _brush_step, _normal_contact
from bike_sim.terrain.contact_profile import ProfileQuery
from bike_sim.sim.ride.physical_mapping import (
    resolve_id, point_jacobian_into,
)


def compiled_profile_vertices(model, data, terrain_name='terrain'):
    """Extract the exact longitudinal cross-section of the compiled raster.

    Only a rigid X-Z profile extruded across Y is supported. Refuse a laterally
    varying heightfield rather than quietly comparing different road geometries.
    """
    import mujoco
    geom = resolve_id(model, mujoco.mjtObj.mjOBJ_GEOM, terrain_name)
    if int(model.geom_type[geom]) != int(mujoco.mjtGeom.mjGEOM_HFIELD):
        raise ValueError('compliant_2d requires the compiled terrain heightfield')
    hfield = int(model.geom_dataid[geom])
    rows, cols = int(model.hfield_nrow[hfield]), int(model.hfield_ncol[hfield])
    start = int(model.hfield_adr[hfield])
    raster = np.asarray(model.hfield_data[start:start+rows*cols]).reshape(rows, cols)
    if cols < 2 or rows < 2 or not np.isfinite(raster).all():
        raise ValueError('invalid compiled heightfield raster')
    if not np.allclose(raster, raster[0], rtol=0, atol=1e-10):
        raise ValueError('compliant_2d does not support lateral terrain variation')
    sx, _, sz, _ = model.hfield_size[hfield]
    R = data.geom_xmat[geom].reshape(3, 3)
    if not np.allclose(R[:, 1], [0., 1., 0.], rtol=0, atol=1e-10):
        raise ValueError('terrain transform must preserve the planar Y axis')
    local = np.column_stack((np.linspace(-sx, sx, cols), np.zeros(cols), raster[0]*sz))
    world = local @ R.T + data.geom_xpos[geom]
    if not np.allclose(world[:, 1], 0., rtol=0, atol=1e-9):
        raise ValueError('working profile must lie in world Y=0')
    return array(world[:, [0, 2]], 'compiled road vertices', readonly=True)


@dataclass
class _BrushState:
    xi: float = 0.
    tangent: np.ndarray | None = None
    point: np.ndarray | None = None
    segment: int | None = None
    center: np.ndarray | None = None


_INT64_MAX = (1 << 63) - 1
# First binary64 value outside the int64 domain: float(1 << 63) == 0x1p63.
_INT64_OVERFLOW_BOUND = float(1 << 63)


def interval_id(time: float, dt: float) -> int:
    """Simulation-time -> snapshot interval id (S4 interval domain).

    Binary64 ``time / dt`` rounded half-even — bit-identical to the native
    ``interval_clock::interval_id`` twin and, like CPython ``round()``,
    independent of the process floating-point rounding mode. Finite
    nonnegative time and a finite positive interval are required; a finite
    input whose interval leaves the int64 range is an OverflowError, while
    domain violations stay ValueError — matching the native error classes.
    """
    time = scalar(time, 'tire.interval_id.time', minimum=0.)
    dt = scalar(dt, 'tire.interval_id.dt', positive=True)
    quotient = time / dt
    if not isfinite(quotient) or quotient >= _INT64_OVERFLOW_BOUND:
        raise OverflowError('tire.interval_id')
    result = round(quotient)
    if result > _INT64_MAX:
        raise OverflowError('tire.interval_id')
    return result


def effective_friction(tire, surface_map, contact_x_m, slip_mps):
    """Road friction cannot exceed the explicitly configured tire ceiling."""
    if surface_map is None:
        return float(tire.mu), 'configured'
    surface = surface_map.at(contact_x_m)
    return min(float(tire.mu), surface.mu(slip_mps)), surface.name


class TireForceApplier:
    def __init__(self, model, vertices_xz, config, surface_map=None):
        import mujoco
        config.__post_init__()
        self.config = config
        self.surface_map = surface_map if config.surface_mode == 'track' else None
        if config.surface_mode == 'track' and self.surface_map is None:
            raise ValueError('track material mode requires an explicit SurfaceMap')
        self.profile = ProfileQuery(
            vertices_xz, significant_delta_m=config.significant_delta_m,
            significance_fraction=config.significance_fraction,
            normal_angle_deg=config.distinct_normal_deg,
        )
        self.geoms = {s:resolve_id(model, mujoco.mjtObj.mjOBJ_GEOM, f'geom_{s}_contact')
                      for s in ('front', 'rear')}
        self.bodies = {s:int(model.geom_bodyid[g]) for s, g in self.geoms.items()}
        self.radii = {s:float(model.geom_size[g, 0]) for s, g in self.geoms.items()}
        # Per-step Jacobian scratch: the contact-point Jacobian serves both the
        # velocity read and the wrench map, so it is computed once per wheel.
        self._jac_contact = np.empty((3, model.nv))
        self._jac_center = np.empty((3, model.nv))
        if config.backend == 'compliant_2d':
            for body in self.bodies.values():
                ids = np.flatnonzero(model.geom_bodyid == body)
                if np.any(model.geom_contype[ids]) or np.any(model.geom_conaffinity[ids]):
                    raise ValueError('native wheel collisions must be disabled for compliant_2d')
        self.reset()

    def reset(self):
        self.states = {s:_BrushState() for s in self.geoms}
        self.snapshots = {}
        self.diagnostics = {}
        self.elastic_energy_j = 0.
        self.brush_loss_step_j = 0.
        self.radial_dissipation_power_w = 0.
        self.last_time_s = None

    def restart_clock(self):
        """End initialization without discarding static contact shear."""
        self.last_time_s = None

    def stored_energy(self, model, data):
        """Energy of current geometry and existing shear, without advancing it."""
        self.config.__post_init__()
        if self.config.backend != 'compliant_2d':
            return 0.
        energy = 0.
        for side, geom in self.geoms.items():
            cfg, state = getattr(self.config, side), self.states[side]
            contact = self.profile.contact(data.geom_xpos[geom][[0, 2]], self.radii[side], state.segment)
            energy += cfg.material.elastic_response(contact.delta)[1]
            energy += .5*cfg.tangent_k_n_m*state.xi**2
        return derived(energy, 'TireForceApplier.stored_energy')

    def compute_qfrc(self, model, data, dt, *, advance=True):
        self.config.__post_init__()
        if self.surface_map is not None:
            self.surface_map.validate()
        if not advance:
            probe = copy.copy(self)
            probe.states = copy.deepcopy(self.states)
            probe.last_time_s = None
            force=probe.compute_qfrc(model, data, dt)
            self.probe_snapshots=probe.snapshots
            return force
        from bike_sim.sim.ride.contact_state import ContactPatch, WheelContactSnapshot
        dt = scalar(dt, 'tire interval', positive=True)
        time = float(data.time)
        if self.last_time_s is not None and time <= self.last_time_s:
            raise ValueError('tire state can advance only once per increasing timestamp')
        if self.config.backend != 'compliant_2d':
            self.last_time_s = time
            return np.zeros(model.nv)
        qfrc = np.zeros(model.nv)
        energy = loss = radial_loss_power = 0.
        snapshots, diagnostics, new_states = {}, {}, {}
        for side, geom in self.geoms.items():
            cfg, state = getattr(self.config, side), self.states[side]
            center = np.array(data.geom_xpos[geom], copy=True)
            contact = self.profile.contact(center[[0, 2]], self.radii[side], state.segment)
            p = np.array([contact.point[0], 0., contact.point[1]])
            n = np.array([contact.normal[0], 0., contact.normal[1]])
            tangent = np.array([n[2], 0., -n[0]])
            point_jacobian_into(model, data, self.bodies[side], p, self._jac_contact)
            point_jacobian_into(model, data, self.bodies[side], center, self._jac_center)
            velocity = self._jac_contact @ data.qvel
            center_velocity = self._jac_center @ data.qvel
            delta_dot, slip = -float(velocity @ n), float(velocity @ tangent)
            if isinstance(cfg.material, TireSpec):
                normal, radial_energy = _normal_contact(
                    contact.delta, delta_dot, cfg.material.radial_k_n_m, cfg.material.radial_c_ns_m)
                elastic_force = cfg.material.radial_k_n_m * max(contact.delta, 0.)
            else:
                normal, radial_energy = cfg.material.normal_contact(contact.delta, delta_dot)
                elastic_force = cfg.material.elastic_response(contact.delta)[0]
            xi = state.xi
            release_loss = 0.
            if state.tangent is not None:
                # Mesh indices are not physical distances: at small terrain
                # spacing an ordinary step can cross several segments. Transport
                # across such continuous motion, but release at a separated branch.
                adjacent = abs(contact.segment_id-state.segment) <= 1
                coincident = np.linalg.norm(p-state.point) < 1e-8
                moved = np.linalg.norm(center-state.center) if state.center is not None else 0.
                similar = float(state.tangent @ tangent) >= np.cos(np.radians(self.config.distinct_normal_deg))
                continuous = similar and np.linalg.norm(p-state.point) <= (
                    2.*moved+self.radii[side]*np.linalg.norm(tangent-state.tangent)+1e-8)
                if not adjacent and not coincident and not continuous:
                    release_loss = .5*cfg.tangent_k_n_m*xi*xi
                    xi = 0.
                else:
                    transported = xi*float(state.tangent @ tangent)
                    release_loss = .5*cfg.tangent_k_n_m*(xi*xi-transported*transported)
                    xi = transported
            mu, material = effective_friction(cfg, self.surface_map, p[0], slip)
            xi_new, force, brush_loss = _brush_step(
                xi, slip, float(center_velocity @ tangent), normal,
                cfg.tangent_k_n_m, mu, cfg.relaxation_length_m, dt,
            )
            force_world = normal*n + force*tangent
            qfrc += self._jac_contact.T @ force_world
            patches = (ContactPatch(p, n, normal, force, slip),) if contact.delta > 0 else ()
            snapshots[side] = WheelContactSnapshot(
                time_s=time, patches=patches, geometric_contact=contact.delta > 0,
                interval_id=interval_id(time, dt), backend='compliant_2d', wheel_axis_m=center,
            )
            # Continuous radial passivity, separated from numerical integration error.
            radial_loss = (normal-elastic_force)*delta_dot
            radial_loss_power += max(radial_loss, 0.) if contact.delta > 0 else 0.
            energy += radial_energy + .5*cfg.tangent_k_n_m*xi_new*xi_new
            loss += max(release_loss, 0.) + brush_loss
            diagnostics[side] = {
                'multi_support':contact.multi_support, 'penetration_m':contact.delta,
                'normal_speed_mps':-delta_dot, 'slip_mps':slip,
                'normal_load_n':normal, 'tangent_force_n':force,
                'friction_coefficient':mu, 'surface':material,
                'branch_release_loss_j':max(release_loss, 0.),
                'brush_loss_j':brush_loss, 'radial_energy_j':radial_energy,
                'shear_energy_j':.5*cfg.tangent_k_n_m*xi_new*xi_new,
                'outside_material_load_range':not cfg.material.is_load_in_valid_range(normal),
            }
            new_states[side] = _BrushState(xi_new, tangent.copy(), p.copy(), contact.segment_id, center.copy())
        # Commit persistent state only after both wheels have evaluated successfully.
        derived_array(qfrc, 'TireForceApplier.qfrc')
        for value in (energy, loss, radial_loss_power):
            derived(value, 'TireForceApplier.energy_or_power')
        self.states, self.snapshots, self.diagnostics = new_states, snapshots, diagnostics
        self.elastic_energy_j, self.brush_loss_step_j = energy, loss
        self.radial_dissipation_power_w = radial_loss_power
        self.last_time_s = time
        return qfrc
