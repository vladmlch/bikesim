"""Explicit, energy-auditable material-station tire/road force path.

The per-station loop order in ``compute_qfrc`` is the native contract: the C++
port reproduces it bit-for-bit, so scalar accumulations stay sequential and
numpy reductions stay exactly where spec section 4.3 of
docs/superpowers/specs/2026-10-11-native-distributed-tire-design.md marks them.

The material key is (wheel side, station number), never the road mesh index.
Tangential forces and velocities use the SAME material-point Jacobian; a
snapshot retains each patch and the physical resultant wrench.
"""
import copy
from dataclasses import dataclass
import numpy as np
import mujoco
from bike_sim.physics.checks import scalar
from bike_sim.physics.distributed_tire import station_angles, station_density, station_response
from bike_sim.physics.tire import _brush_step
from bike_sim.terrain.profile_distance import SignedProfile
from bike_sim.sim.ride.tire_forces import effective_friction
from bike_sim.sim.ride.physical_mapping import resolve_id, point_jacobian
from bike_sim.sim.ride.contact_state import ContactPatch, WheelContactSnapshot


@dataclass
class _StationState:
    """Per-station brush state: only what the next interval reads (spec section 4.1)."""
    xi: float = 0.0
    tangent: np.ndarray | None = None


_STATION_LEAVES = ('xi', 'tangent.0', 'tangent.1', 'tangent.2')


class DistributedTireForceApplier:
    def __init__(self, model, vertices_xz, config, surface_map=None):
        if config.backend != 'distributed_2d_reference':
            raise ValueError('distributed force applier requires its explicit backend')
        self.config = config
        self.profile = SignedProfile(vertices_xz)
        self.surface_map = surface_map if config.surface_mode == 'track' else None
        if config.surface_mode == 'track' and surface_map is None:
            raise ValueError('track material mode needs the actual track surface map')
        self.angles, self.weights = station_angles(config.distributed.station_count)
        self.materials = {side: config.distributed.for_wheel(side)
                          for side in ('front', 'rear')}
        self.geoms = {s:resolve_id(model, mujoco.mjtObj.mjOBJ_GEOM, 'geom_'+s+'_contact') for s in ('front','rear')}
        self.bodies = {s:int(model.geom_bodyid[g]) for s,g in self.geoms.items()}
        self.radii = {s:float(model.geom_size[g,0]) for s,g in self.geoms.items()}
        self.local = {s:np.column_stack((r*np.cos(self.angles), np.zeros(len(self.angles)), r*np.sin(self.angles)))
                      for s,r in self.radii.items()}
        for body in self.bodies.values():
            ids = np.flatnonzero(model.geom_bodyid == body)
            if np.any(model.geom_contype[ids]) or np.any(model.geom_conaffinity[ids]):
                raise ValueError('native wheel collisions must be disabled for the distributed backend')
        self.reset()

    def reset(self):
        self.states = {}
        self.snapshots = {}; self.diagnostics = {}
        self.elastic_energy_j = self.brush_loss_step_j = self.radial_dissipation_power_w = 0.
        self.last_time_s = None

    def restart_clock(self):
        self.last_time_s = None

    def state_names(self) -> list[str]:
        """Canonical wire columns: side-major, station-major, then xi and the tangent components."""
        return [f'{side}.s{index:03d}.{leaf}' for side in ('front', 'rear')
                for index in range(len(self.angles)) for leaf in _STATION_LEAVES]

    def state_dict(self) -> dict:
        """{'names': [...], 'row': ndarray}: NaN for an inactive station or an unset tangent."""
        row = np.full(2 * len(self.angles) * 4, np.nan)
        for (side, index), state in self.states.items():
            base = (0 if side == 'front' else 1) * len(self.angles) * 4 + index * 4
            row[base] = float(state.xi)
            if state.tangent is not None:
                row[base + 1:base + 4] = np.asarray(state.tangent, float)
        return {'names': self.state_names(), 'row': row}

    def set_state(self, names, row) -> None:
        """Inverse of state_dict; every canonical column is required and unknown
        columns are rejected — the wire schema is the native writer's contract."""
        index = {str(n): i for i, n in enumerate(names)}
        row = np.asarray(row, float)
        canonical = set(self.state_names())
        for name in index:
            if name not in canonical:
                raise ValueError(f'tire state has unknown column {name}')
        if row.shape != (len(index),):
            raise ValueError(f'tire state row has {row.shape} cells for {len(index)} columns')
        states = {}
        for side in ('front', 'rear'):
            for station in range(len(self.angles)):
                keys = [f'{side}.s{station:03d}.{leaf}' for leaf in _STATION_LEAVES]
                for key in keys:
                    if key not in index:
                        raise ValueError(f'tire state is missing column {key}')
                values = np.array([row[index[k]] for k in keys])
                if np.isnan(values[0]):
                    continue
                tangent = None if np.isnan(values[1:]).all() else values[1:].copy()
                states[(side, station)] = _StationState(float(values[0]), tangent)
        self.states = states
        self.last_time_s = None

    def _stations(self, data, side):
        """Centre, rotation, every station's world point, and the ascending ids of active stations."""
        center = data.geom_xpos[self.geoms[side]].copy()
        rotation = data.xmat[self.bodies[side]].reshape(3, 3)
        points = center + self.local[side] @ rotation.T
        heights = self.profile.height(points[:, 0])
        ids = [int(i) for i in np.flatnonzero(points[:, 2] < heights)]
        return center, points, ids

    def stored_energy(self, model, data):
        energy = 0.
        for side in self.geoms:
            _, points, ids = self._stations(data, side)
            material = self.materials[side].density
            for sid in ids:
                geometry = self.profile.query(points[sid][None, [0, 2]])
                energy += self.weights[sid] * station_density(-float(geometry.distance[0]), material)[1]
        # A detached station keeps its old energy until the next force interval
        # releases it as loss. Endpoint accounting must not discard it twice.
        for (side, sid), state in sorted(self.states.items()):
            energy += .5 * getattr(self.config, side).tangent_k_n_m * self.weights[sid] * state.xi ** 2
        return energy

    def compute_qfrc(self, model, data, dt, *, advance=True):
        if not advance:
            probe = copy.copy(self)
            probe.states = copy.deepcopy(self.states)
            probe.last_time_s = None
            force = probe.compute_qfrc(model, data, dt)
            self.probe_snapshots = probe.snapshots
            return force
        dt = scalar(dt, 'distributed tire interval', positive=True)
        time = float(data.time)
        if self.last_time_s is not None and time <= self.last_time_s:
            raise ValueError('tire state can advance only once per increasing timestamp')
        result = np.zeros(model.nv)
        snapshots = {}; diagnostics = {}; new_states = {}
        energy = total_loss = radial_loss_power = 0.
        cos_distinct = np.cos(np.radians(self.config.distinct_normal_deg))
        for side, body in self.bodies.items():
            cfg = getattr(self.config, side)
            domain = self.materials[side]; material = domain.density
            center, points, ids = self._stations(data, side)
            jp_center, jr = point_jacobian(model, data, body, center)
            center_velocity = jp_center @ data.qvel
            loads = np.zeros(len(ids)); energies = np.zeros(len(ids)); depths = np.zeros(len(ids))
            normals = []; patches = []
            losses = 0.; shear_energy = 0.; radial_power = 0.; nonsmooth = 0
            for j, sid in enumerate(ids):
                geometry = self.profile.query(points[sid][None, [0, 2]])
                n2 = geometry.normal[0]
                normal = np.array([n2[0], 0., n2[1]])
                tangent = np.array([normal[2], 0., -normal[0]])
                arm = points[sid] - center
                # J(p) = J(c) + omega x (p - c), component by component (the unrolled np.cross).
                jac = np.empty_like(jp_center)
                for k in range(3):
                    jac[k] = jp_center[k] + (jr[(k + 1) % 3] * arm[(k + 2) % 3] - jr[(k + 2) % 3] * arm[(k + 1) % 3])
                velocity = jac @ data.qvel
                rate = -((velocity[0] * normal[0] + velocity[1] * normal[1]) + velocity[2] * normal[2])
                penetration = -float(geometry.distance[0])
                load, elastic_energy = station_response(penetration, rate, float(self.weights[sid]), material)
                elastic_density = station_density(penetration, material)[0]
                radial_power += max(0., (load - self.weights[sid] * elastic_density) * rate)
                old = self.states.get((side, sid), _StationState())
                xi = old.xi; k_station = cfg.tangent_k_n_m * self.weights[sid]
                if old.tangent is not None:
                    transported = xi * float(old.tangent @ tangent)
                    losses += max(0., .5 * k_station * (xi * xi - transported * transported))
                    xi = transported
                slip = float(velocity @ tangent)
                mu, _ = effective_friction(cfg, self.surface_map, float(geometry.point[0, 0]), slip)
                xi_new, tangent_force, brush_loss = _brush_step(
                    xi, slip, float(center_velocity @ tangent), load, k_station, mu, cfg.relaxation_length_m, dt)
                losses += brush_loss; shear_energy += .5 * k_station * xi_new ** 2
                force = load * normal + tangent_force * tangent
                result += jac.T @ force
                loads[j] = load; energies[j] = elastic_energy; depths[j] = penetration
                nonsmooth += int(geometry.nonsmooth[0])
                normals.append(normal)
                patches.append(ContactPatch(points[sid], normal, load, tangent_force, slip))
                new_states[(side, sid)] = _StationState(xi_new, tangent)
            for (old_side, sid), old in sorted(self.states.items()):
                if old_side == side and (side, sid) not in new_states:
                    losses += .5 * cfg.tangent_k_n_m * self.weights[sid] * old.xi ** 2
            normal_total = float(np.sum(loads)); radial_energy = float(np.sum(energies))
            max_depth = float(depths.max()) if ids else 0.
            center_clearance = float(self.profile.query(center[None, [0, 2]]).distance[0]) - self.radii[side]
            signed_overlap = max_depth if ids else -max(0., center_clearance)
            distinct = any(float(a @ b) < cos_distinct for a in normals for b in normals) if len(ids) > 1 else False
            # Separate material patches across a missing tread station are also
            # simultaneous supports, even when road normals happen to agree.
            separate = (len(ids) > 1 and any(b - a > 1 for a, b in zip(ids, ids[1:]))
                        and not (ids[0] == 0 and ids[-1] == len(self.angles) - 1))
            snapshots[side] = WheelContactSnapshot(time, tuple(patches), bool(ids), round(time / dt),
                                                   'distributed_2d_reference', center)
            diagnostics[side] = {
                'penetration_m': signed_overlap, 'normal_load_n': normal_total,
                'multi_support': bool(distinct or separate), 'supports_multiple_contacts': True,
                'capability_status': 'experimental_reference_not_release_qualified',
                'calibration_status': domain.calibration_status,
                'station_count': len(self.angles), 'active_station_count': len(ids),
                'nonsmooth_station_count': nonsmooth, 'max_patch_count': len(patches),
                'outside_material_load_range': not (domain.valid_load_range_n[0] <= normal_total <= domain.valid_load_range_n[1]),
                'outside_material_deflection_range': not (domain.valid_deflection_range_m[0] <= max_depth <= domain.valid_deflection_range_m[1]),
                'radial_energy_j': radial_energy, 'shear_energy_j': shear_energy, 'brush_loss_j': losses}
            energy += radial_energy + shear_energy; total_loss += losses; radial_loss_power += radial_power
        self.states, self.snapshots, self.diagnostics = new_states, snapshots, diagnostics
        self.elastic_energy_j, self.brush_loss_step_j = energy, total_loss
        self.radial_dissipation_power_w = radial_loss_power; self.last_time_s = time
        return result
