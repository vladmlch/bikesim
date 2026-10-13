"""Pneumatic radial-carcass tyre on the physical force-path protocol.

The per-side evaluation order in ``compute_qfrc`` is the native contract: the
C++ writer reproduces it bit-for-bit, so wheel iteration stays
``('front', 'rear')``, patch accumulation stays sequential, and each patch
force maps through ``mj_jac`` evaluated at the patch centroid -- never the
``J(center) + omega x arm`` shortcut, which rounds differently.

Unlike the legacy ``sim/ride/tyre/applier.py`` xfrc writer, this applier never
mutates ``model.opt.timestep`` for the detailed tier: the physical runtime owns
the integration step and hands it in as ``dt`` on every call.

The mutable state IS the two ``PneumaticTyre`` objects: ``evaluate`` advances
their carcass and brush memory in place, so a failed two-wheel step restores a
pre-loop deepcopy. The canonical state wire is

    {side}.ray.{i:03d}.{delta_prev_m|elastic_prev_n|maxwell_n|bristle_m}
    {side}.carcass.initialized          (1.0 / 0.0)
    {side}.rim.{active|x_m|speed_mps|peak_load_n|peak_force_n|energy_j}
    {side}.patch.count
    {side}.patch.{k:02d}.{start|stop|kappa_prime}

per side in ``('front', 'rear')`` order: all ``n_rays`` ray cells first, then
the carcass flag, the rim-event record, the patch count and ``max_patches =
(n_rays + 1) // 2`` patch slots in ray order. NaN marks an unset bristle cell,
an inactive rim-event field, and an unused patch slot; the carcass arrays are
always finite. ``bristle_m`` is the per-ray bristle deflection of the detailed
brush -- patch bristle ``j`` maps to ray ``patch.start + j`` -- and stays NaN
everywhere on the lumped tier.
"""
import copy

import mujoco
import numpy as np

from bike_sim.physics.checks import derived, derived_array, scalar
from bike_sim.physics.physical_config import PneumaticTireConfig
from bike_sim.physics.tyre import TIERS
from bike_sim.sim.ride.contact_state import ContactPatch, WheelContactSnapshot
from bike_sim.sim.ride.physical_mapping import point_jacobian_into
from bike_sim.sim.ride.tire_forces import interval_id
from bike_sim.sim.ride.tyre.applier import _resolve_wheel
from bike_sim.sim.ride.tyre.brush import DiscretisedBrush
from bike_sim.sim.ride.tyre.geometry import RoadProfile, intersect
from bike_sim.sim.ride.tyre.model import PneumaticTyre, _PatchBrushState
from bike_sim.terrain.surface import SurfaceMap

_SIDES = ('front', 'rear')
_RAY_LEAVES = ('delta_prev_m', 'elastic_prev_n', 'maxwell_n', 'bristle_m')
_RIM_LEAVES = ('active', 'x_m', 'speed_mps', 'peak_load_n', 'peak_force_n', 'energy_j')
_PATCH_LEAVES = ('start', 'stop', 'kappa_prime')

_MM_TO_M = 0.001
_MM2_TO_M2 = 1_000_000.0


def _radial_energy_j(tyre: PneumaticTyre, hits) -> float:
    """Stored carcass energy of one wheel: ``sum 0.5*(k_c*delta^2 + k_rim*overlap^2)*ds``."""
    spec = tyre.tyre
    ds_m = spec.outer_radius_mm * _MM_TO_M * tyre.ring.dtheta_rad
    k_c = spec.carcass_stiffness_n_mm2 * _MM2_TO_M2
    k_rim = spec.rim_stiffness_n_mm2 * _MM2_TO_M2
    delta = hits.delta_m
    overlap = np.maximum(delta - spec.rim_strike_deflection_mm * _MM_TO_M, 0.0)
    return float(np.sum(0.5 * k_c * delta * delta * ds_m
                        + 0.5 * k_rim * overlap * overlap * ds_m))


class PneumaticTireForceApplier:
    """Qfrc-surface applier wrapping the two ``PneumaticTyre`` wheels.

    The constructor resolves the physical wheel bindings, so a model missing
    the named bodies/joints/geoms -- or still colliding the wheel sphere --
    fails here, never mid-step.
    """

    def __init__(self, model, road_profile, config, surface_map=None):
        if config.backend != 'pneumatic':
            raise ValueError('pneumatic force applier requires the pneumatic backend')
        pneumatic = getattr(config, 'pneumatic', None)
        if not isinstance(pneumatic, PneumaticTireConfig):
            raise ValueError('pneumatic tires need an explicit pneumatic config')
        if not isinstance(road_profile, RoadProfile):
            raise ValueError('pneumatic tires need a sampled RoadProfile')
        self.config = config
        self.profile = road_profile
        self.surface_map = (SurfaceMap.uniform(pneumatic.surface)
                            if pneumatic.surface is not None else surface_map)
        if not isinstance(self.surface_map, SurfaceMap):
            raise ValueError('pneumatic tires need a track surface map or a configured surface')
        self.tier = TIERS[pneumatic.tier]
        self.wheels = {
            'front': _resolve_wheel(model, 'front_wheel', 'front_wheel_spin',
                                    'geom_front_contact',
                                    pneumatic.front.outer_radius_mm / 1000.0),
            'rear': _resolve_wheel(model, 'rear_wheel', 'rear_wheel_spin',
                                   'geom_rear_contact',
                                   pneumatic.rear.outer_radius_mm / 1000.0),
        }
        for binding in self.wheels.values():
            if model.geom_contype[binding.contact_geom_id] != 0:
                raise ValueError('native wheel collisions must be disabled for the pneumatic backend')
        self.bodies = {side: binding.body_id for side, binding in self.wheels.items()}
        self.geoms = {side: binding.contact_geom_id for side, binding in self.wheels.items()}
        self.radii = {side: float(model.geom_size[binding.contact_geom_id][0])
                      for side, binding in self.wheels.items()}
        self._n_rays = self.tier.n_rays
        # Disjoint contiguous runs over n rays: at most ceil(n/2) patches.
        self._max_patches = (self._n_rays + 1) // 2
        self._velocity6 = np.zeros(6, dtype=float)
        self._jacp = np.empty((3, model.nv))
        self.states = {side: PneumaticTyre(getattr(pneumatic, side), self.tier)
                       for side in _SIDES}
        self.reset()

    def reset(self):
        for tyre in self.states.values():
            tyre.reset()
        self.snapshots = {}
        self.diagnostics = {}
        self.elastic_energy_j = self.brush_loss_step_j = self.radial_dissipation_power_w = 0.
        self.last_time_s = None

    def restart_clock(self):
        self.last_time_s = None

    def state_names(self) -> list[str]:
        """Canonical wire columns: side-major, ray-major, then carcass, rim, patches."""
        names = []
        for side in _SIDES:
            names += [f'{side}.ray.{i:03d}.{leaf}'
                      for i in range(self._n_rays) for leaf in _RAY_LEAVES]
            names.append(f'{side}.carcass.initialized')
            names += [f'{side}.rim.{leaf}' for leaf in _RIM_LEAVES]
            names.append(f'{side}.patch.count')
            names += [f'{side}.patch.{k:02d}.{leaf}'
                      for k in range(self._max_patches) for leaf in _PATCH_LEAVES]
        return names

    def state_dict(self) -> dict:
        """{'names': [...], 'row': ndarray}: NaN for unset bristle/rim/patch cells."""
        n, width = self._n_rays, self._max_patches
        row = np.full(2 * (4 * n + 8 + 3 * width), np.nan)
        for side_index, side in enumerate(_SIDES):
            tyre = self.states[side]
            base = side_index * (4 * n + 8 + 3 * width)
            bristle = np.full(n, np.nan)
            for (start, stop), state in tyre._patch_states.items():
                if state.detailed is not None:
                    bristle[start:stop] = state.detailed.bristle_deflection_m
            carcass = tyre.carcass_state
            for leaf_index, values in enumerate((carcass.delta_prev_m, carcass.elastic_prev_n,
                                                 carcass.maxwell_force_n, bristle)):
                for i in range(n):
                    row[base + 4 * i + leaf_index] = values[i]
            row[base + 4 * n] = 1.0 if carcass.initialized else 0.0
            rim = base + 4 * n + 1
            row[rim] = 1.0 if carcass.rim_strike_active else 0.0
            if carcass.rim_strike_active:
                row[rim + 1:rim + 6] = (carcass.rim_event_x_m, carcass.rim_event_speed_mps,
                                        carcass.rim_event_peak_load_n,
                                        carcass.rim_event_peak_force_n,
                                        carcass.rim_event_energy_j)
            patches = list(tyre._patch_states.values())
            row[rim + 6] = float(len(patches))
            for k, state in enumerate(patches):
                row[rim + 7 + 3 * k:rim + 10 + 3 * k] = (
                    float(state.start), float(state.stop), state.kappa_prime)
        return {'names': self.state_names(), 'row': row}

    def set_state(self, names, row) -> None:
        """Inverse of state_dict; every canonical column is required and unknown
        columns are rejected -- the wire schema is the native writer's contract."""
        index = {str(name): i for i, name in enumerate(names)}
        row = np.asarray(row, float)
        canonical = set(self.state_names())
        for name in index:
            if name not in canonical:
                raise ValueError(f'tire state has unknown column {name}')
        if row.shape != (len(index),):
            raise ValueError(f'tire state row has {row.shape} cells for {len(index)} columns')
        for name in self.state_names():
            if name not in index:
                raise ValueError(f'tire state is missing column {name}')
        n, width = self._n_rays, self._max_patches

        def cell(name):
            return float(row[index[name]])

        decoded = {}
        for side in _SIDES:
            delta = np.empty(n); elastic = np.empty(n); maxwell = np.empty(n)
            bristle = np.empty(n)
            for i in range(n):
                prefix = f'{side}.ray.{i:03d}.'
                delta[i] = cell(prefix + 'delta_prev_m')
                elastic[i] = cell(prefix + 'elastic_prev_n')
                maxwell[i] = cell(prefix + 'maxwell_n')
                bristle[i] = cell(prefix + 'bristle_m')
            for name, values in (('delta_prev_m', delta), ('elastic_prev_n', elastic),
                                 ('maxwell_n', maxwell)):
                if not np.isfinite(values).all():
                    raise ValueError(f'tire state ray {name} cells must be finite')
            initialized = cell(f'{side}.carcass.initialized')
            if initialized not in (0.0, 1.0):
                raise ValueError(f'tire state column {side}.carcass.initialized must be 0 or 1')
            rim_active = cell(f'{side}.rim.active')
            if rim_active not in (0.0, 1.0):
                raise ValueError(f'tire state column {side}.rim.active must be 0 or 1')
            rim = {leaf: cell(f'{side}.rim.{leaf}') for leaf in _RIM_LEAVES[1:]}
            finite_rim = np.isfinite(list(rim.values()))
            if rim_active and not finite_rim.all():
                raise ValueError(f'tire state {side}.rim fields must be finite while active')
            if not rim_active and finite_rim.any():
                raise ValueError(f'tire state {side}.rim fields must be NaN while inactive')
            count_value = cell(f'{side}.patch.count')
            if not np.isfinite(count_value) or count_value < 0. or count_value != int(count_value):
                raise ValueError(f'tire state column {side}.patch.count must be a nonnegative integer')
            count = int(count_value)
            if count > width:
                raise ValueError(f'tire state column {side}.patch.count exceeds {width}')
            slots = []
            previous_stop = -1
            for k in range(count):
                start = cell(f'{side}.patch.{k:02d}.start')
                stop = cell(f'{side}.patch.{k:02d}.stop')
                kappa_prime = cell(f'{side}.patch.{k:02d}.kappa_prime')
                for name, value in (('start', start), ('stop', stop), ('kappa_prime', kappa_prime)):
                    if not np.isfinite(value):
                        raise ValueError(f'tire state {side}.patch.{k:02d}.{name} must be finite')
                if start != int(start) or stop != int(stop):
                    raise ValueError(f'tire state {side}.patch.{k:02d} bounds must be integers')
                start, stop = int(start), int(stop)
                if not (0 <= start < stop <= n):
                    raise ValueError(f'tire state {side}.patch.{k:02d} bounds outside the ray ring')
                if start < previous_stop:
                    raise ValueError(f'tire state {side} patches must be disjoint in ray order')
                previous_stop = stop
                slots.append((start, stop, kappa_prime))
            # An unset cell must be NaN: a finite value would round-trip to NaN
            # on the next state_dict, silently diverging from the native wire.
            covered = np.zeros(n, dtype=bool)
            for start, stop, _ in slots:
                covered[start:stop] = True
            finite_bristle = np.isfinite(bristle)
            for k in range(count, width):
                for leaf in _PATCH_LEAVES:
                    if np.isfinite(cell(f'{side}.patch.{k:02d}.{leaf}')):
                        raise ValueError(f'tire state {side}.patch.{k:02d} is set past patch.count')
            if self.tier.discretised_brush:
                if not np.isfinite(bristle[covered]).all():
                    raise ValueError(f'tire state {side} patch rays need finite bristle cells')
                if finite_bristle[~covered].any():
                    raise ValueError(f'tire state {side} bristle cells are set outside every patch')
            elif finite_bristle.any():
                raise ValueError(f'tire state {side} bristle cells must be NaN on the lumped tier')
            decoded[side] = (delta, elastic, maxwell, bristle, bool(initialized),
                             rim_active == 1.0, rim, slots)
        for side in _SIDES:
            delta, elastic, maxwell, bristle, initialized, rim_active, rim, slots = decoded[side]
            tyre = self.states[side]
            tyre.reset()
            carcass = tyre.carcass_state
            carcass.delta_prev_m[:] = delta
            carcass.elastic_prev_n[:] = elastic
            carcass.maxwell_force_n[:] = maxwell
            carcass.initialized = initialized
            if rim_active:
                carcass.rim_strike_active = True
                carcass.rim_event_x_m = rim['x_m']
                carcass.rim_event_speed_mps = rim['speed_mps']
                carcass.rim_event_peak_load_n = rim['peak_load_n']
                carcass.rim_event_peak_force_n = rim['peak_force_n']
                carcass.rim_event_energy_j = rim['energy_j']
            patch_states = {}
            for start, stop, kappa_prime in slots:
                state = _PatchBrushState(start, stop, kappa_prime)
                if self.tier.discretised_brush:
                    brush = DiscretisedBrush(stop - start)
                    brush.kappa_prime = kappa_prime
                    brush.bristle_deflection_m[:] = bristle[start:stop]
                    state.detailed = brush
                patch_states[(start, stop)] = state
            tyre._patch_states = patch_states
        self.last_time_s = None

    def _intersect(self, tyre, hub_x_m, hub_z_m):
        """The wheel's ray ring against the profile at the given hub position."""
        return intersect(tyre.ring, hub_x_m, hub_z_m,
                         tyre.tyre.outer_radius_mm * _MM_TO_M, self.profile)

    def _shear_energy_j(self, tyre):
        """Bristle shear of committed patches: ``0.5*k_px*dx*b^2`` summed (detailed tier).

        ``k_px`` and ``dx`` reuse the last committed patch output's load and
        contact length -- the same patch-load convention the brush stepped with;
        a patch range without a committed output carries no shear term.
        """
        if not self.tier.discretised_brush:
            return 0.
        outputs = {(patch.ray_start, patch.ray_stop): patch
                   for patch in tyre.last_outputs.patches}
        energy = 0.
        for (start, stop), state in tyre._patch_states.items():
            brush = state.detailed
            patch = outputs.get((start, stop))
            if brush is None or patch is None:
                continue
            half_length_m = 0.5 * patch.contact_length_m
            if not half_length_m > 0.0:
                continue
            surface = self.surface_map.at(float(patch.centroid_world_m[0]))
            k_px = surface.slip_stiffness_per_load * patch.normal_load_n / (2.0 * half_length_m ** 2)
            dx_m = 2.0 * half_length_m / brush.n_elements
            energy += 0.5 * k_px * dx_m * float(np.sum(
                brush.bristle_deflection_m * brush.bristle_deflection_m))
        return energy

    def stored_energy(self, model, data):
        """Energy of current geometry and committed state, without advancing it.

        Radial terms re-intersect the ray ring at the current pose;
        ``radial_energy_j`` in the diagnostics is the identical expression, so
        ``elastic_energy_j`` after a commit equals this value bitwise.
        """
        energy = 0.
        for side in _SIDES:
            tyre = self.states[side]
            hub = data.xpos[self.bodies[side]]
            hits = self._intersect(tyre, float(hub[0]), float(hub[2]))
            energy += _radial_energy_j(tyre, hits)
            energy += self._shear_energy_j(tyre)
        return energy

    def compute_qfrc(self, model, data, dt, *, advance=True):
        if not advance:
            probe = copy.copy(self)
            probe.states = copy.deepcopy(self.states)
            probe.last_time_s = None
            force = probe.compute_qfrc(model, data, dt)
            self.probe_snapshots = probe.snapshots
            return force
        dt = scalar(dt, 'pneumatic tire interval', positive=True)
        time = float(data.time)
        if self.last_time_s is not None and time <= self.last_time_s:
            raise ValueError('tire state can advance only once per increasing timestamp')
        result = np.zeros(model.nv)
        snapshots = {}
        diagnostics = {}
        energy = brush_loss = radial_power = 0.
        # evaluate() mutates each PneumaticTyre in place; a failed wheel leaves
        # the other already advanced, so restore the pre-loop copy on failure.
        backup = copy.deepcopy(self.states)
        try:
            for side in _SIDES:
                binding = self.wheels[side]
                tyre = self.states[side]
                self._velocity6.fill(0.0)
                mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY,
                                         binding.body_id, self._velocity6, 0)
                hub = np.array(data.xpos[binding.body_id], copy=True)
                outputs = tyre.evaluate(
                    hub_position_world_m=hub,
                    hub_velocity_world_mps=self._velocity6[3:6],
                    omega_forward_radps=float(self._velocity6[1]),
                    road=self.profile,
                    surface_map=self.surface_map,
                    dt_s=dt,
                )
                for patch in outputs.patches:
                    point_jacobian_into(model, data, binding.body_id,
                                        patch.centroid_world_m, self._jacp)
                    result += self._jacp.T @ patch.force_world_n
                contact_patches = tuple(
                    ContactPatch(
                        point_m=patch.centroid_world_m,
                        normal=patch.normal_force_world_n / patch.normal_load_n,
                        normal_load_n=patch.normal_load_n,
                        tangent_force_n=patch.tangential_force_n,
                        slip_mps=patch.slip_speed_mps,
                        couple_world_nm=None,
                        native_world_force_n=None,
                    )
                    for patch in outputs.patches
                )
                snapshots[side] = WheelContactSnapshot(
                    time_s=time, patches=contact_patches,
                    geometric_contact=not outputs.airborne,
                    interval_id=interval_id(time, dt), backend='pneumatic',
                    wheel_axis_m=hub)
                hits = self._intersect(tyre, float(hub[0]), float(hub[2]))
                radial_energy = _radial_energy_j(tyre, hits)
                shear_energy = self._shear_energy_j(tyre)
                patch_count = len(outputs.patches)
                brush_power_w = float(sum(patch.dissipated_power_w
                                          for patch in outputs.patches))
                load = float(outputs.normal_load_n)
                if load > 0.0:
                    slip_mps = float(sum(patch.slip_speed_mps * patch.normal_load_n
                                         for patch in outputs.patches)) / load
                    mu = float(sum(patch.friction_coefficient * patch.normal_load_n
                                   for patch in outputs.patches)) / load
                else:
                    slip_mps = 0.0
                    mu = float(self.surface_map.at(float(hub[0])).mu(0.0))
                rim_event = outputs.rim_event
                diagnostics[side] = {
                    'normal_load_n': load,
                    'support_n': float(outputs.support_n),
                    'tangent_force_n': float(sum(patch.tangential_force_n
                                                 for patch in outputs.patches)),
                    'slip_mps': slip_mps,
                    'slip_ratio': float(outputs.slip_ratio),
                    'transient_slip_ratio': float(outputs.transient_slip_ratio),
                    'friction_coefficient': mu,
                    'penetration_m': float(np.max(hits.delta_m)),
                    'contact_length_m': float(outputs.contact_length_m),
                    'patch_count': patch_count,
                    'multi_support': bool(patch_count > 1),
                    'supports_multiple_contacts': True,
                    'fully_sliding': bool(outputs.fully_sliding),
                    'airborne': bool(outputs.airborne),
                    'coverage_event': bool(outputs.coverage_event),
                    'rim_strike_active': bool(outputs.rim_strike_active),
                    'rim_event_x_m': 0.0 if rim_event is None else float(rim_event.x_m),
                    'rim_event_speed_mps': 0.0 if rim_event is None else float(rim_event.speed_mps),
                    'rim_event_peak_load_n': 0.0 if rim_event is None else float(rim_event.peak_load_n),
                    'rim_event_peak_force_n': 0.0 if rim_event is None else float(rim_event.peak_rim_force_n),
                    'rim_event_energy_j': 0.0 if rim_event is None else float(rim_event.absorbed_energy_j),
                    'pressure_bar': float(outputs.pressure_bar),
                    'dissipated_power_w': float(outputs.dissipated_power_w),
                    'brush_loss_j': brush_power_w * dt,
                    'radial_energy_j': radial_energy,
                    'shear_energy_j': shear_energy,
                    'outside_material_load_range': False,
                    'surface': (outputs.patches[0].surface_name if patch_count
                                else self.config.pneumatic.surface),
                }
                energy += radial_energy + shear_energy
                brush_loss += brush_power_w * dt
                radial_power += float(outputs.dissipated_power_w) - brush_power_w
            derived_array(result, 'PneumaticTireForceApplier.qfrc')
            energy = derived(energy, 'PneumaticTireForceApplier.energy')
            brush_loss = derived(brush_loss, 'PneumaticTireForceApplier.brush_loss')
            radial_power = derived(radial_power, 'PneumaticTireForceApplier.radial_power')
        except Exception:
            self.states = backup
            raise
        self.snapshots, self.diagnostics = snapshots, diagnostics
        self.elastic_energy_j, self.brush_loss_step_j = energy, brush_loss
        self.radial_dissipation_power_w = radial_power
        self.last_time_s = time
        return result


__all__ = ["PneumaticTireForceApplier"]
