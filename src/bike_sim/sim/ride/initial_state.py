"""Explicit, auditable t=0 state transfer for orthogonal numerical experiments.

Not an equilibrium cache: each target retains its own source/configuration hash
and reports the acceleration residual in that target model. No running state can
be captured or restored. The compiled road must coincide in the initial support
neighbourhood; road elsewhere and timestep may differ.
"""
import copy
from dataclasses import asdict,dataclass
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
from bike_sim.sim.ride.equilibrium_cache import _state_arrays,restore_state
from bike_sim.sim.ride.physical_samples import plain


def _hash(payload):
    return hashlib.sha256(json.dumps(plain(payload),sort_keys=True,separators=(',',':'),
                                     allow_nan=False).encode()).hexdigest()


def _contract(sim,*,station_count=None):
    cfg=asdict(sim.physics_config)
    if station_count is not None:cfg['tires']['distributed']['station_count']=station_count
    for name in ('timestep_s','equilibrium_cache_enabled','equilibrium_refine_after_s','equilibrium_refine_period_s'):
        cfg.pop(name,None)
    m=sim.model
    return _hash({'physics':cfg,'specs':asdict(sim.specs),'mass':asdict(sim.mass_specs),
        'rider':asdict(sim.rider),'start_x_m':sim.start_x_m,
        'joints':[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_JOINT,j) for j in range(m.njnt)],
        'compiled':{n:getattr(m,n).tolist() for n in ('qpos0','body_mass','body_inertia','body_ipos',
                    'body_pos','body_quat','jnt_axis','jnt_pos','jnt_range','jnt_limited',
                    'dof_armature','actuator_gear','actuator_ctrlrange','eq_solref','eq_solimp')},
        'envelope_sha256': None if cfg['articulated']['joint_envelope_path'] is None else
          hashlib.sha256(Path(cfg['articulated']['joint_envelope_path']).read_bytes()).hexdigest()})


def _guard(runtime):
    if runtime.sim.steps!=0 or float(runtime.sim.data.time)!=0.:
        raise ValueError('physical initial states may only be transferred before the first step')


@dataclass(frozen=True)
class PhysicalInitialState:
    """JSON payload is immutable; each restoration receives fresh material arrays."""
    payload_json: str

    @property
    def payload(self):return json.loads(self.payload_json)

    @property
    def sha256(self):return hashlib.sha256(self.payload_json.encode()).hexdigest()

    @classmethod
    def capture(cls,sim):
        r=sim.physical
        if r is None:raise ValueError('physical runtime required')
        _guard(r)
        m,d=sim.model,sim.data
        vertices=r.vertices
        centers=[d.geom_xpos[g][0] for g in (sim.contact_query.front_id,sim.contact_query.rear_id)]
        road_x=np.linspace(min(centers)-.5,max(centers)+.5,513)
        state={k:{'dtype':str(v.dtype),'shape':list(v.shape),'values':v.tolist()} for k,v in _state_arrays(r).items()}
        drive=r.drive
        payload={'schema_version':1,'contract_sha256':_contract(sim),
            'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'qacc_warmstart':d.qacc_warmstart.tolist(),
            'road_x':road_x.tolist(),'road_z':np.interp(road_x,vertices[:,0],vertices[:,1]).tolist(),
            'material':state,'equilibrium':copy.deepcopy(sim.equilibrium),
            'drive':{name:copy.deepcopy(getattr(drive,name)) for name in ('reference','angles','psi')},
            'ideal_boundary':None if drive.ideal_hub is None else drive.ideal_hub.boundary,
            'clutch_boundary':None if drive.clutch is None else drive.clutch.boundary,
            'hub':None if drive.hub is None else {k:getattr(drive.hub,k) for k in ('boundary','energy_j','torque_nm')},
            'grip_anchor_local':None if r.rider_contacts is None else r.rider_contacts.grip_anchor_local,
            'active_state':None if r.rider_control is None else r.rider_control.active_state,
            'source_timestep_s':float(m.opt.timestep)}
        from bike_sim.validation.environment import source_fingerprint
        payload['source_sha256']=source_fingerprint(Path(__file__).resolve().parents[2])
        return cls(json.dumps(plain(payload),sort_keys=True,separators=(',',':'),allow_nan=False))

    @classmethod
    def project_stations(cls,sim,station_count: int):
        """Represent the same t=0 pose and angular shear field at another N.

        This is explicit quadrature projection, not a fresh equilibrium or an
        exact discrete-state cache hit. Generalized coordinates remain identical;
        the target's changed force residual and energy are measured on restore.
        """
        from bike_sim.physics.distributed_tire import station_angles
        source=cls.capture(sim);p=source.payload;tire=sim.physical.tire
        if sim.physics_config.tires.backend!='distributed_2d_reference':
            raise ValueError('station projection requires the distributed backend')
        angles,_=station_angles(station_count)
        old_count=sim.physics_config.tires.distributed.station_count
        old_angles=np.arange(old_count)*2.*np.pi/old_count
        # station_angles includes a fixed quadrature phase; use the actual
        # angular coordinates for both interpolation domains.
        old_angles=tire.angles
        arrays={name:[] for name in ('sides','xi','tangent','tangent_valid','point','point_valid',
                                     'segment','center','center_valid')}
        for side in ('front','rear'):
            xi=np.zeros(old_count);tangents=np.zeros((old_count,3))
            for (wheel,index),state in tire.states.items():
                if wheel!=side:continue
                xi[index]=state.xi
                if state.tangent is not None:tangents[index]=state.tangent
            target_xi=np.interp(angles,old_angles,xi,period=2.*np.pi)
            target_tangent=np.column_stack([np.interp(angles,old_angles,tangents[:,j],period=2.*np.pi) for j in range(3)])
            center=sim.data.geom_xpos[tire.geoms[side]].copy()
            rotation=sim.data.xmat[tire.bodies[side]].reshape(3,3)
            radius=tire.radii[side]
            local=np.c_[radius*np.cos(angles),np.zeros(len(angles)),radius*np.sin(angles)]
            points=center+local@rotation.T
            active=(points[:,2]<tire.profile.height(points[:,0]))|(np.abs(target_xi)>0.)
            ids=np.flatnonzero(active)
            geometry=tire.profile.query(points[ids][:,[0,2]])
            for j,index in enumerate(ids):
                tangent=target_tangent[index];norm=float(np.linalg.norm(tangent))
                if norm>1e-12:tangent=tangent/norm
                arrays['sides'].append(json.dumps([side,int(index)]));arrays['xi'].append(target_xi[index])
                arrays['tangent'].append(tangent);arrays['tangent_valid'].append(norm>1e-12)
                arrays['point'].append(points[index]);arrays['point_valid'].append(True)
                arrays['segment'].append(int(geometry.segment[j]));arrays['center'].append(center)
                arrays['center_valid'].append(True)
        for name,values in arrays.items():
            dtype=(bool if name.endswith('_valid') else int if name=='segment' else str if name=='sides' else float)
            array=np.asarray(values,dtype=dtype)
            if name in ('tangent','point','center'):array=array.reshape(-1,3)
            p['material']['state_tire_'+name]={'dtype':str(array.dtype),'shape':list(array.shape),'values':array.tolist()}
        p['contract_sha256']=_contract(sim,station_count=station_count)
        p['station_projection']={'method':'periodic piecewise-linear angular shear field; identical qpos/qvel',
            'source_count':old_count,'target_count':station_count,'source_initial_state_sha256':source.sha256}
        return cls(json.dumps(plain(p),sort_keys=True,separators=(',',':'),allow_nan=False))

    def _validated(self,runtime):
        _guard(runtime);p=self.payload;sim=runtime.sim;m=sim.model
        if p.get('schema_version')!=1 or p.get('contract_sha256')!=_contract(sim):
            raise ValueError('initial-state physical model contract differs (only timestep/road mesh may vary)')
        for name,size in (('qpos',m.nq),('qvel',m.nv),('qacc_warmstart',m.nv)):
            a=np.asarray(p[name],float)
            if a.shape!=(size,) or not np.isfinite(a).all():
                raise ValueError('invalid initial-state coordinate array')
        v=runtime.vertices
        actual=np.interp(p['road_x'],v[:,0],v[:,1])
        if not np.allclose(actual,p['road_z'],rtol=0.,atol=2e-7):
            raise ValueError('initial support road differs; solve an independent equilibrium')
        return p

    def prepare(self,runtime):
        if not getattr(runtime,'initializing',False):
            raise ValueError('initial state preparation belongs to constructor/reset only')
        p=self._validated(runtime);sim=runtime.sim;m,d=sim.model,sim.data
        mujoco.mj_resetData(m,d);d.qpos[:]=p['qpos'];mujoco.mj_forward(m,d)
        if runtime.rider_control is not None:
            runtime.rider_control.initialize(m,d)
            d.qpos[:]=p['qpos'];mujoco.mj_forward(m,d)
            runtime.rider_contacts.reset(m,d)
        runtime.drive.reset(m,d)
        if runtime.tire is not None:runtime.tire.reset()
        self.restore(runtime)
        result=copy.deepcopy(p['equilibrium'])
        result.update(cache_hit=False,initial_state_source='explicit_t0_transfer',
            initial_state_sha256=self.sha256,initial_state_source_timestep_s=p['source_timestep_s'],
            source_equilibrium_residual_qacc=p['equilibrium'].get('residual_qacc'),
            initial_state_source_sha256=p['source_sha256'])
        return result

    def restore(self,runtime):
        p=self._validated(runtime);sim=runtime.sim;m,d=sim.model,sim.data
        d.qpos[:]=p['qpos'];d.qvel[:]=p['qvel'];d.qacc_warmstart[:]=p['qacc_warmstart']
        state={k:np.asarray(v['values'],dtype=v['dtype']).reshape(v['shape']) for k,v in p['material'].items()}
        if not restore_state(runtime,state):raise ValueError('incompatible initial material state')
        drive=runtime.drive
        for name,value in p['drive'].items():setattr(drive,name,tuple(value) if name=='angles' else value)
        if drive.hub is not None:
            for key,value in p['hub'].items():setattr(drive.hub,key,value)
        mujoco.mj_forward(m,d)
        if drive.ideal_hub is not None:
            drive.ideal_hub.boundary=p['ideal_boundary'];drive.ideal_hub.prepare(m,d)
        if drive.clutch is not None:
            drive.clutch.boundary=p['clutch_boundary'];drive.clutch.prepare(m,d)
        if runtime.rider_contacts is not None:
            runtime.rider_contacts.grip_anchor_local=np.asarray(p['grip_anchor_local'],float).copy()
            runtime.rider_contacts.restart_clock()
        if runtime.rider_control is not None:
            runtime.rider_control.active_state=np.asarray(p['active_state'],float).copy()
            runtime.rider_control.activation_time_s=None
        if runtime.tire is not None:runtime.tire.restart_clock()
        drive.restart_clock()
        runtime.apply_forces(active=False,advance=False,front=runtime.cfg.initial_front_brake,
                             rear=runtime.cfg.initial_rear_brake)
        mujoco.mj_forward(m,d)
        sim.equilibrium['initial_state_target_residual_qacc']=float(np.max(np.abs(d.qacc)))
        sim.contacts,runtime.snapshots=runtime._contacts(final=False)
        # The ordinary zero-time probes must not replace captured warm-start data.
        d.qacc_warmstart[:]=p['qacc_warmstart']
