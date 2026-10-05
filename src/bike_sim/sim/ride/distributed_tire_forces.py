"""Explicit, energy-auditable material-station tire/road force path.

The material key is (wheel side, station number), never the road mesh index.
Tangential forces and velocities use the SAME material-point Jacobian; a
snapshot retains each patch and the physical resultant wrench.
"""
import copy
import numpy as np
import mujoco
from bike_sim.physics.checks import scalar
from bike_sim.physics.distributed_tire import station_angles, normal_station_response, density_response
from bike_sim.physics.tire import _brush_step
from bike_sim.terrain.profile_distance import SignedProfile
from bike_sim.sim.ride.tire_forces import _BrushState, effective_friction
from bike_sim.sim.ride.physical_mapping import resolve_id, point_jacobian
from bike_sim.sim.ride.contact_state import ContactPatch, WheelContactSnapshot


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

    def _geometry(self, data, side):
        center = data.geom_xpos[self.geoms[side]].copy()
        # The body frame, not the cylinder geom frame, defines a material angle.
        rotation = data.xmat[self.bodies[side]].reshape(3,3)
        points = center+self.local[side]@rotation.T
        h = self.profile.height(points[:,0])
        ids = np.flatnonzero(points[:,2] < h)
        distance = self.profile.query(points[ids][:,[0,2]])
        return center, points, ids, distance

    def stored_energy(self, model, data):
        energy = 0.
        for side in self.geoms:
            _, _, ids, geometry = self._geometry(data, side)
            energy += float(np.dot(self.weights[ids], density_response(-geometry.distance, self.materials[side].density)[1]))
        # A detached station keeps its old energy until the next force interval
        # releases it as loss. Endpoint accounting must not discard it twice.
        for (side, sid), state in self.states.items():
            energy += .5*getattr(self.config,side).tangent_k_n_m*self.weights[sid]*state.xi**2
        return energy

    def compute_qfrc(self, model, data, dt, *, advance=True):
        if not advance:
            probe = copy.copy(self)
            probe.states = copy.deepcopy(self.states)
            probe.last_time_s = None
            force=probe.compute_qfrc(model,data,dt)
            self.probe_snapshots=probe.snapshots
            return force
        dt = scalar(dt, 'distributed tire interval', positive=True)
        time = float(data.time)
        if self.last_time_s is not None and time <= self.last_time_s:
            raise ValueError('tire state can advance only once per increasing timestamp')
        result = np.zeros(model.nv)
        snapshots = {}; diagnostics = {}; new_states = {}
        energy = total_loss = radial_loss_power = 0.
        for side, body in self.bodies.items():
            cfg = getattr(self.config,side)
            center, points, ids, geometry = self._geometry(data, side)
            jp_center, jr = point_jacobian(model,data,body,center)
            center_velocity = jp_center@data.qvel
            normals = np.column_stack((geometry.normal[:,0], np.zeros(len(ids)), geometry.normal[:,1]))
            tangents = np.column_stack((normals[:,2], np.zeros(len(ids)), -normals[:,0]))
            # Rigid body identity J(p)=J(c)+omega cross (p-c), vectorized over
            # the small active subset rather than rebuilding engine kinematics.
            arms = points[ids]-center
            jacobians = jp_center[None,:,:]+np.cross(jr.T[None,:,:], arms[:,None,:]).transpose(0,2,1)
            velocities = jacobians@data.qvel
            rate = -np.sum(velocities*normals,axis=1)
            penetration = -geometry.distance
            domain = self.materials[side]
            material = domain.density
            normal_loads, elastic_energies = normal_station_response(penetration,rate,self.weights[ids],material)
            elastic_density,_ = density_response(penetration,material)
            radial_power = float(np.sum(np.maximum(0.,(normal_loads-self.weights[ids]*elastic_density)*rate)))
            losses = 0.; shear_energy = 0.; patches = []
            for j, sid_value in enumerate(ids):
                sid = int(sid_value); key=(side,sid)
                old=self.states.get(key,_BrushState())
                tangent=tangents[j]; xi=old.xi; k=cfg.tangent_k_n_m*self.weights[sid]
                if old.tangent is not None:
                    transported=xi*float(old.tangent@tangent)
                    losses+=max(0.,.5*k*(xi*xi-transported*transported))
                    xi=transported
                slip=float(velocities[j]@tangent)
                mu, _ = effective_friction(cfg,self.surface_map,float(geometry.point[j,0]),slip)
                xi_new, tangent_force, brush_loss = _brush_step(xi,slip,float(center_velocity@tangent),
                    float(normal_loads[j]),k,mu,cfg.relaxation_length_m,dt)
                losses+=brush_loss; shear_energy+=.5*k*xi_new**2
                force=normal_loads[j]*normals[j]+tangent_force*tangent
                result+=jacobians[j].T@force
                # Station force location is explicit. Normal translation to the
                # road projection is equivalent; tangential lever arm is not.
                patches.append(ContactPatch(points[sid],normals[j],float(normal_loads[j]),tangent_force,slip))
                new_states[key]=_BrushState(xi_new,tangent.copy(),points[sid].copy(),int(geometry.segment[j]),center.copy())
            for key, old in self.states.items():
                if key[0]==side and key not in new_states:
                    losses+=.5*cfg.tangent_k_n_m*self.weights[key[1]]*old.xi**2
            normal=float(normal_loads.sum()); radial_energy=float(elastic_energies.sum())
            max_depth=float(penetration.max()) if len(ids) else 0.
            center_clearance=float(self.profile.query(center[[0,2]][None,:]).distance[0])-self.radii[side]
            signed_overlap=max_depth if len(ids) else -max(0.,center_clearance)
            distinct=any(float(a@b) < np.cos(np.radians(self.config.distinct_normal_deg))
                         for a in normals for b in normals) if len(ids)>1 else False
            # Separate material patches across a missing tread station are also
            # simultaneous supports, even when road normals happen to agree.
            separate=len(ids)>1 and np.any(np.diff(ids)>1) and not (ids[0]==0 and ids[-1]==len(self.angles)-1)
            snapshots[side]=WheelContactSnapshot(time,tuple(patches),bool(len(ids)),round(time/dt),
                                                  'distributed_2d_reference',center)
            diagnostics[side]={'penetration_m':signed_overlap,'normal_load_n':normal,
                'multi_support':bool(distinct or separate),'supports_multiple_contacts':True,
                'capability_status':'experimental_reference_not_release_qualified',
                'calibration_status':domain.calibration_status,'station_count':len(self.angles),
                'active_station_ids':[int(sid) for sid in ids],
                'road_segment_ids':[int(sid) for sid in geometry.segment],
                'road_projection_m':geometry.point.tolist(),
                'nonsmooth_station_count':int(geometry.nonsmooth.sum()),
                'outside_material_load_range':not(domain.valid_load_range_n[0] <= normal <= domain.valid_load_range_n[1]),
                'outside_material_deflection_range':not(domain.valid_deflection_range_m[0] <= max_depth <= domain.valid_deflection_range_m[1]),
                'radial_energy_j':radial_energy,'shear_energy_j':shear_energy,'brush_loss_j':losses}
            energy+=radial_energy+shear_energy; total_loss+=losses; radial_loss_power+=radial_power
        self.states,self.snapshots,self.diagnostics=new_states,snapshots,diagnostics
        self.elastic_energy_j,self.brush_loss_step_j=energy,total_loss
        self.radial_dissipation_power_w=radial_loss_power;self.last_time_s=time
        return result
