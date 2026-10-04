"""Unilateral rider supports and a releasable grip, all with paired reactions."""
import copy
from dataclasses import dataclass
from math import hypot
import numpy as np
from bike_sim.physics.checks import array, scalar
from bike_sim.physics.tire import _brush_step, _normal_contact
from bike_sim.sim.ride.physical_mapping import resolve_id, relative_point_jacobian
from bike_sim.sim.ride.support_geometry import _box_pad_contact, validate_planar_support_model
from bike_sim.sim.ride.attachment_wrench import (
    attachment_sample, attachment_raw, prepare_attachment_geometry,
    attachment_raw_from_geometry)
from bike_sim.sim.ride.weld_pedals import PedalWelds, SaddleWeld, SaddlePin, GripConnect, equality_rows


def apply_internal_force(model,data,body_a,body_b,point,force,qfrc):
    import mujoco
    p,f=array(point,'interface point',(3,)),array(force,'interface force',(3,))
    if not 0<body_a<model.nbody or not 0<body_b<model.nbody or body_a==body_b:
        raise ValueError('internal reaction needs distinct physical bodies')
    if np.asarray(qfrc).shape!=(model.nv,):
        raise ValueError('invalid internal-force destination')
    # Applying both forces at ONE world point cancels both resultant force and
    # world moment. Two separate anchors would add an unaccounted couple.
    mujoco.mj_applyFT(model,data,f,np.zeros(3),p,body_a,qfrc)
    mujoco.mj_applyFT(model,data,-f,np.zeros(3),p,body_b,qfrc)


def grip_step(xi,relative_velocity,k,c,dt):
    """Backward-force quadrature: work + stored-energy change + loss = 0."""
    xi=array(xi,'grip state',(3,)); u=array(relative_velocity,'grip velocity',(3,))
    k,c,dt=scalar(k,'grip stiffness',positive=True),scalar(c,'grip damping',minimum=0),scalar(dt,'grip dt',positive=True)
    return _grip_step(xi,u,k,c,dt)


def _grip_step(xi,u,k,c,dt):
    """Core of `grip_step` for callers that already validated inputs."""
    new=xi+dt*u
    force=-k*new-c*u
    energy=.5*k*float(new@new)
    loss=.5*k*dt*dt*float(u@u)+c*dt*float(u@u)
    if not np.isfinite(np.r_[new,force,energy,loss]).all():
        raise ValueError('grip update overflow')
    return new,force,energy,loss


@dataclass
class _SupportState:
    xi: float=0.
    tangent: np.ndarray | None=None


class RiderContactApplier:
    CONTACTS=('saddle','front_pedal','rear_pedal','grip')

    def __init__(self,model,pose,config):
        import mujoco
        self.pose,self.config=pose,config
        obj=mujoco.mjtObj
        self.frame=resolve_id(model,obj.mjOBJ_BODY,'frame')
        self.steer=resolve_id(model,obj.mjOBJ_BODY,'steer')
        self.pelvis=resolve_id(model,obj.mjOBJ_BODY,'rider_pelvis')
        self.forearms={side:resolve_id(model,obj.mjOBJ_BODY,f'rider_forearm_{side}')
                       for side in ('left','right')}
        self.shoulders={side:resolve_id(model,obj.mjOBJ_BODY,f'rider_upper_arm_{side}')
                        for side in ('left','right')}
        self.grip_sites={side:resolve_id(model,obj.mjOBJ_SITE,f'site_rider_grip_{side}')
                         for side in ('left','right')}
        self.arm_reach=float(np.linalg.norm(pose.shoulder-pose.elbow)+np.linalg.norm(pose.elbow-pose.grip))
        self.supports={
            'saddle':(self.pelvis,resolve_id(model,obj.mjOBJ_SITE,'site_rider_saddle'),
                      self.frame,resolve_id(model,obj.mjOBJ_GEOM,'geom_saddle')),
        }
        for side in ('front','rear'):
            foot=resolve_id(model,obj.mjOBJ_BODY,'rider_foot_'+side)
            sole=resolve_id(model,obj.mjOBJ_SITE,'site_rider_sole_'+side)
            pedal=resolve_id(model,obj.mjOBJ_BODY,'pedal_'+side)
            geom=resolve_id(model,obj.mjOBJ_GEOM,'geom_pedal_'+side)
            self.supports[side+'_pedal']=(foot,sole,pedal,geom)
        crank=resolve_id(model,obj.mjOBJ_JOINT,'crank_spin')
        self.crank_dof=int(model.jnt_dofadr[crank])
        # Per-step Jacobian scratch for the paired-support relative Jacobian.
        self._jac_a=np.empty((3,model.nv)); self._jac_b=np.empty((3,model.nv))
        self.welded_pedals = config.pedal_attachment == 'weld'
        self._welds = PedalWelds(model) if self.welded_pedals else None
        self.welded_saddle = config.saddle_attachment == 'weld'
        self.pinned_saddle = config.saddle_attachment == 'pin'
        self.linked_saddle = self.welded_saddle or self.pinned_saddle
        self._saddle_link = (SaddleWeld(model) if self.welded_saddle
                             else SaddlePin(model) if self.pinned_saddle else None)
        self.welded_grip = config.grip_attachment == 'weld'
        self._grip_connect = ({side:GripConnect(model,side) for side in ('left','right')}
                              if self.welded_grip else None)
        self.reset(model,None)

    def reset(self,model,data):
        self._model,self._data=model,data
        self.enabled={name:True for name in self.CONTACTS}
        self.states={f"{name}:{i}":_SupportState() for name in self.supports for i in range(2)}
        self.grip_xi_local={side:np.zeros(3) for side in ('left','right')}
        self.grip_anchor_local={side:None for side in ('left','right')}
        if data is not None:
            validate_planar_support_model(model,data,[entry[3] for entry in self.supports.values()])
            steer_R=data.xmat[self.steer].reshape(3,3)
            for side in ('left','right'):
                if self.welded_grip:
                    self.grip_anchor_local[side]=model.eq_data[self._grip_connect[side].eq_id,3:6].copy()
                else:
                    self.grip_anchor_local[side]=steer_R.T@(
                        data.site_xpos[self.grip_sites[side]]-data.xpos[self.steer])
        self.elastic_energy_j=0.
        self.loss_step_j=0.
        self.radial_dissipation_power_w=0.
        self.delivered_crank_torque_nm=0.
        self.diagnostics={}
        self.last_time_s=None
        self.pending_release_loss_j=0.
        # (per-support telemetry, crank torque) of the last solve; see settle_welds.
        self._settled_welds=None
        self.last_attachment_samples={}
        self.last_attachment_errors=()

    def set_enabled(self,name,enabled):
        if name not in self.CONTACTS or not isinstance(enabled,bool):
            raise ValueError('invalid rider contact enable request')
        if not enabled and ((self.welded_pedals and name.endswith('_pedal'))
                            or (self.linked_saddle and name == 'saddle')
                            or (self.welded_grip and name == 'grip')):
            return True   # a weld or pin cannot be released
        if not enabled and self.enabled[name]:
            if name=='grip':
                for side in ('left','right'):
                    xi=self.grip_xi_local[side]
                    self.pending_release_loss_j+=.5*self.config.grip_k_n_m*float(xi@xi)
                    self.grip_xi_local[side]=np.zeros(3)
            else:
                for i in range(2):
                    key=f"{name}:{i}"
                    self.pending_release_loss_j+=.25*self.config.support_tangent_k_n_m*self.states[key].xi**2
                    self.states[key]=_SupportState()
                if self._data is not None:
                    import mujoco
                    # Release can be requested after integration, whereas the
                    # previous diagnostic row belongs to the incoming pose.
                    # Refresh positions only; do not run a new force solve.
                    mujoco.mj_kinematics(self._model,self._data)
                    for _,_,_,_,gap,inside in self._pads(self._model,self._data,name,self.supports[name]):
                        if inside:
                            self.pending_release_loss_j+=.25*self.config.support_k_n_m*max(-gap,0.)**2
        if name=='grip' and enabled and not self.enabled[name]:
            if self._data is None or any(self.grip_anchor_local[s] is None for s in ('left','right')):
                return False
            import mujoco
            model,data=self._model,self._data
            mujoco.mj_kinematics(model,data); mujoco.mj_comPos(model,data)
            R=data.xmat[self.steer].reshape(3,3)
            for side in ('left','right'):
                grip=data.xpos[self.steer]+R@self.grip_anchor_local[side]
                gap=float(np.linalg.norm(data.site_xpos[self.grip_sites[side]]-grip))
                jac=relative_point_jacobian(model,data,self.forearms[side],self.steer,grip,self._jac_a,self._jac_b)
                speed=float(np.linalg.norm(jac@data.qvel))
                if gap>self.config.grip_capture_distance_m or speed>self.config.grip_capture_speed_mps:
                    return False
            for side in ('left','right'):
                self.grip_xi_local[side][:]=0.
        self.enabled[name]=enabled
        return bool(self.enabled[name])

    def release_all(self):
        for name in self.CONTACTS:
            self.set_enabled(name,False)

    def restart_clock(self):
        self.last_time_s=None

    def initialize_settled_state(self, model, data):
        """Commit the final initial material state before starting the clock.

        The equilibrium solver may refine both pose and shear after its last
        advancing contact evaluation. Release-energy data must correspond to
        that settled pose, not to an earlier relaxation step.
        """
        if self.last_time_s is not None or float(data.time) != 0.:
            raise ValueError('settled contact initialization requires a fresh clock')
        self.compute_qfrc(model,data,float(model.opt.timestep))
        self.loss_step_j=0.
        self.radial_dissipation_power_w=0.
        self.restart_clock()

    def _pads(self, model, data, name, entry):
        body,site,bike,geom=entry
        R=data.geom_xmat[geom].reshape(3,3)
        half=(self.config.saddle_patch_half_length_m if name=='saddle'
              else self.config.pedal_patch_half_length_m)
        radius=self.config.support_pad_radius_m
        foot_rotation=data.xmat[body].reshape(3,3)
        for i,sign in enumerate((-1.,1.)):
            # Place the circular sample above the declared sole surface, so a
            # horizontal pad retains exactly the previous gap and stiffness.
            center=data.site_xpos[site]+foot_rotation@np.array([sign*half,0.,radius])
            contact=_box_pad_contact(center,radius,data.geom_xpos[geom],R,model.geom_size[geom])
            yield (f"{name}:{i}",contact.point_m,contact.normal,contact.tangent,
                   contact.gap_m,contact.within_footprint)

    def stored_energy(self, model, data):
        cfg=self.config
        energy=sum(.5*cfg.grip_k_n_m*float(self.grip_xi_local[s]@self.grip_xi_local[s])
                   for s in ('left','right'))
        for name,entry in self.supports.items():
            if (self.welded_pedals and name.endswith('_pedal')
                    or self.linked_saddle and name == 'saddle'):
                continue
            for key,point,n,tangent,gap,inside in self._pads(model,data,name,entry):
                energy+=.25*cfg.support_tangent_k_n_m*self.states[key].xi**2
                if self.enabled[name] and inside:
                    energy+=.25*cfg.support_k_n_m*max(-gap,0.)**2
        return energy

    def prepare_attachment_raw(self, model, data):
        """Capture attachment geometry at the incoming state for a raw period step.

        Only pose-dependent quantities are computed here. Solved equality forces
        are intentionally deferred until after ``mj_step``; this keeps the raw
        telemetry bit-for-bit tied to the same start pose without the old
        post-step qpos/qvel swap and two kinematics passes.
        """
        prepared={}; errors=[]
        for name,entry in self.supports.items():
            body,site,bike,_=entry
            if name=='saddle' and self.linked_saddle:
                eq_id,rotational=self._saddle_link.eq_id,self.welded_saddle
                half=self.config.saddle_patch_half_length_m
            elif name.endswith('_pedal') and self.welded_pedals:
                eq_id=self._welds.eq_ids[name.split('_')[0]]
                rotational,half=True,float(model.geom_size[entry[3],0])
            else:
                continue
            normal=np.mean([n for _,_,n,_,_,_ in self._pads(model,data,name,entry)],axis=0)
            out='foot_'+name.split('_')[0] if name.endswith('_pedal') else 'saddle'
            try:
                prepared[out]=prepare_attachment_geometry(model,data,eq_id,body,bike,
                    np.array(data.site_xpos[site]),normal,out.rsplit('_',1)[0],
                    rotational=rotational,half_patch_m=half)
            except ValueError:
                errors.append(out+':unobservable_attachment_wrench')
        if self.welded_grip:
            for side in ('left','right'):
                anchor=self.grip_anchor_local[side]
                if anchor is None:
                    continue
                grip=data.xpos[self.steer]+data.xmat[self.steer].reshape(3,3)@anchor
                pull=data.xpos[self.pelvis]-grip
                try:
                    prepared['grip_'+side]=prepare_attachment_geometry(model,data,
                        self._grip_connect[side].eq_id,self.forearms[side],self.steer,grip,
                        pull,'grip',rotational=False,pull_direction=pull)
                except ValueError:
                    errors.append('grip_'+side+':unobservable_attachment_wrench')
        return prepared,tuple(errors)

    def settle_welds(self,model,data,interval_state=None, *, raw=False, prepared=None):
        """Latch the weld/connect reactions of the solve that just ran.

        ``interval_state`` is the optional (qpos, qvel) the solved interval
        started from — the pose at which MuJoCo built efc_J. Attachment
        wrenches are then measured on that pose, not on the post-step one.

        Call right after a solve with the full applied forces and controls
        (mj_step, or apply_forces followed by mj_forward), while efc_force and
        the body poses still belong to that solve. The next compute_qfrc
        reports and senses these reactions: a weld's force exists only after
        the solve it takes part in, so the causal reading for the coming step
        is the last solved one. Reading efc_force inside compute_qfrc instead
        would see the forward pass at the top of apply_forces, which runs with
        applied forces and controls zeroed and carries almost none of the
        rider's weight.

        Returns the per-support telemetry of this solve.
        ``normal_n`` is along the support-surface normal the unilateral pad
        model uses: positive is the surface pressing on the rider; negative is
        the weld holding the rider on, i.e. a real saddle or flat pedal would
        separate (``would_separate``). ``would_slip`` flags shear the pad's
        Coulomb limit (support_mu x compression) could not hold: on a steep
        climb the weld keeps the pelvis from sliding back off the saddle even
        while it still presses on it. The grip is a hand that may pull, so it
        reports only its force.
        """
        rows=equality_rows(data)
        result={}
        for name,entry in self.supports.items():
            if name=='saddle' and self.linked_saddle:
                force=self._saddle_link.force_on_rider_n(model,data,rows=rows)
            elif name.endswith('_pedal') and self.welded_pedals:
                force=self._welds.force_on_rider_n(model,data,name.split('_')[0],rows=rows)
            else:
                continue
            normals=[];tangents=[]
            for _,_,n,tangent,_,_ in self._pads(model,data,name,entry):
                normals.append(n);tangents.append(tangent)
            n=np.mean(normals,axis=0);n/=max(np.linalg.norm(n),1e-12)
            tangent=np.mean(tangents,axis=0);tangent/=max(np.linalg.norm(tangent),1e-12)
            normal=float(force@n);shear=float(force@tangent)
            result[name]={'force_on_rider_n':force.tolist(),'normal_n':normal,'tangent_n':shear,
                          'would_separate':normal<0.,
                          'would_slip':abs(shear)>self.config.support_mu*max(normal,0.)}
        if self.welded_grip:
            for side in ('left','right'):
                force=self._grip_connect[side].force_on_rider_n(model,data,rows=rows)
                result['grip_'+side]={'force_on_rider_n':force.tolist()}
        crank=(self._welds.delivered_crank_torque_nm(model,data,rows=rows)
               if self.welded_pedals else 0.)
        self._settled_welds=(copy.deepcopy(result),crank)
        if self.welded_pedals:
            # The torque sensor reads this on the next step.
            result['crank_torque_nm']=crank
        if raw and prepared is not None:
            prepared_raw, prepared_errors = prepared
            raw_samples={}
            errors=list(prepared_errors)
            for name,geometry in prepared_raw.items():
                try:
                    raw_samples[name]=attachment_raw_from_geometry(model,data,geometry,rows=rows)
                except ValueError:
                    errors.append(name+':unobservable_attachment_wrench')
            self.last_attachment_samples,self.last_attachment_errors=raw_samples,tuple(errors)
        else:
            self.last_attachment_samples,self.last_attachment_errors=(
                self.attachment_samples(model,data,interval_state,raw=raw))
        return (result, self.last_attachment_samples, self.last_attachment_errors) if raw else result

    def attachment_samples(self,model,data,interval_state=None, *, raw=False):
        if interval_state is None:
            return self._attachment_samples(model,data,raw=raw)
        import mujoco
        qpos,qvel=interval_state
        saved_qpos,saved_qvel=data.qpos.copy(),data.qvel.copy()
        data.qpos[:]=qpos; data.qvel[:]=qvel
        mujoco.mj_kinematics(model,data); mujoco.mj_comPos(model,data)
        try:
            return self._attachment_samples(model,data,raw=raw)
        finally:
            data.qpos[:]=saved_qpos; data.qvel[:]=saved_qvel
            mujoco.mj_kinematics(model,data); mujoco.mj_comPos(model,data)

    def _attachment_samples(self,model,data, *, raw=False):
        """Physical budget sample per solved attachment of this interval.

        Each weld/connect reaction is recovered at the support's patch centre
        from the solved efc multipliers of the solve that just ran, so it must
        be called while that efc state is still current (settle_welds timing).
        An attachment whose solved force cannot be attributed to its rows —
        rank-deficient or partly out-of-plane — reports an
        'unobservable_attachment_wrench' error instead of a fake sample.
        """
        samples={};errors=[]
        measure=attachment_raw if raw else attachment_sample
        rows=equality_rows(data)
        for name,entry in self.supports.items():
            body,site,bike,_=entry
            if name=='saddle' and self.linked_saddle:
                eq_id,rotational=self._saddle_link.eq_id,self.welded_saddle
                half=self.config.saddle_patch_half_length_m
            elif name.endswith('_pedal') and self.welded_pedals:
                eq_id=self._welds.eq_ids[name.split('_')[0]]
                rotational,half=True,float(model.geom_size[entry[3],0])
            else:
                continue
            normal=np.mean([n for _,_,n,_,_,_ in self._pads(model,data,name,entry)],axis=0)
            out='foot_'+name.split('_')[0] if name.endswith('_pedal') else 'saddle'
            try:
                samples[out]=measure(model,data,eq_id,body,bike,
                    np.array(data.site_xpos[site]),normal,out.rsplit('_',1)[0],
                    rotational=rotational,half_patch_m=half,rows=rows)
            except ValueError:
                errors.append(out+':unobservable_attachment_wrench')
        if self.welded_grip:
            for side in ('left','right'):
                anchor=self.grip_anchor_local[side]
                if anchor is None:
                    continue
                grip=data.xpos[self.steer]+data.xmat[self.steer].reshape(3,3)@anchor
                pull=data.xpos[self.pelvis]-grip
                try:
                    samples['grip_'+side]=measure(model,data,
                        self._grip_connect[side].eq_id,self.forearms[side],self.steer,
                        grip,pull,'grip',rotational=False,
                        pull_direction=pull,rows=rows)
                except ValueError:
                    errors.append('grip_'+side+':unobservable_attachment_wrench')
        return samples,tuple(errors)

    def _settled(self,name):
        """Last settled (force on rider, normal load) of a welded support."""
        if self._settled_welds is None or name not in self._settled_welds[0]:
            return np.zeros(3),0.
        entry=self._settled_welds[0][name]
        return np.array(entry['force_on_rider_n']),max(entry.get('normal_n',0.),0.)

    def _settled_crank_torque_nm(self):
        """Last settled pedal-weld torque about the crank (the torque sensor)."""
        return 0. if self._settled_welds is None else self._settled_welds[1]

    def compute_qfrc(self,model,data,dt,*,advance=True,detailed=True):
        dt=scalar(dt,'rider contact dt',positive=True)
        if not advance:
            probe=copy.copy(self)
            probe.states=copy.deepcopy(self.states)
            probe.grip_xi_local=self.grip_xi_local.copy()
            probe.enabled=self.enabled.copy()
            probe.last_time_s=None
            force=probe.compute_qfrc(model,data,dt,detailed=detailed)
            self.probe_diagnostics=probe.diagnostics
            self.probe_enabled=probe.enabled
            self.probe_delivered_crank_torque_nm=probe.delivered_crank_torque_nm
            return force
        self._model,self._data=model,data
        time=float(data.time)
        if self.last_time_s is not None and time<=self.last_time_s:
            raise ValueError('rider contact state advances only once per timestamp')
        if any(self.grip_anchor_local[s] is None for s in ('left','right')):
            raise RuntimeError('initialize rider contact anchors before evaluation')
        cfg=self.config
        # One efc scan feeds every welded-attachment residual in this call.
        # Residuals (efc_pos) depend on pose only, so this zero-input pass
        # measures them correctly; weld forces come from settle_welds.
        # The map stays a local: the probe copy above gets its own inside its
        # recursive call, and the efc layout is rebuilt every step anyway.
        eq_rows=(equality_rows(data)
                 if self.welded_pedals or self.linked_saddle or self.welded_grip
                 else None)
        qfrc=np.zeros(model.nv)
        energy=0.; loss=self.pending_release_loss_j; radial_power=0.; delivered=0.
        new_states={}; diagnostics={}
        for name,entry in self.supports.items():
            body,site,bike,geom=entry
            if self.linked_saddle and name == 'saddle':
                force_on_rider,normal_load=self._settled(name)
                settled=(self._settled_welds[0].get(name)
                         if self._settled_welds is not None else None) or {}
                diagnostics[name]={'enabled':True,'in_platform':True,
                    'normal_load_n':normal_load,
                    'would_separate':bool(settled.get('would_separate')),
                    'would_slip':bool(settled.get('would_slip')),
                    'tangent_n':float(settled.get('tangent_n',0.)),
                    'gap_m':self._saddle_link.translation_residual_m(model,data,rows=eq_rows),
                    'vertical_force_on_rider_n':float(force_on_rider[2])}
                if detailed:
                    diagnostics[name].update({'tangent_force_n':np.zeros(3),
                        'patches':[],
                        'force_on_rider_n':force_on_rider.tolist(),
                        'force_on_bike_n':(-force_on_rider).tolist(),
                        'moment_about_rider_origin_nm':np.zeros(3).tolist(),
                        'radial_energy_j':0.,'shear_energy_j':0.,
                        'relative_power_w':0.})
                new_states[f"{name}:0"]=_SupportState()
                new_states[f"{name}:1"]=_SupportState()
                continue
            if self.welded_pedals and name.endswith('_pedal'):
                side=name.split('_')[0]
                force_on_rider,normal_load=self._settled(name)
                settled=(self._settled_welds[0].get(name)
                         if self._settled_welds is not None else None) or {}
                diagnostics[name]={'enabled':True,'in_platform':True,
                    'normal_load_n':normal_load,
                    'would_separate':bool(settled.get('would_separate')),
                    'would_slip':bool(settled.get('would_slip')),
                    'tangent_n':float(settled.get('tangent_n',0.)),
                    'gap_m':self._welds.translation_residual_m(model,data,side,rows=eq_rows),
                    'vertical_force_on_rider_n':float(force_on_rider[2])}
                if detailed:
                    diagnostics[name].update({'tangent_force_n':np.zeros(3),
                        'patches':[],
                        'force_on_rider_n':force_on_rider.tolist(),
                        'force_on_bike_n':(-force_on_rider).tolist(),
                        'moment_about_rider_origin_nm':np.zeros(3).tolist(),
                        'radial_energy_j':0.,'shear_energy_j':0.,
                        'relative_power_w':0.})
                new_states[f"{name}:0"]=_SupportState()
                new_states[f"{name}:1"]=_SupportState()
                continue
            patches=[];group_force=np.zeros(3);group_moment=np.zeros(3)
            group_normal=group_tangent=group_radial=group_shear=group_power=group_vertical=0.
            in_platform=False
            minimum_gap=float('inf')
            for key,point,n,tangent,gap,inside in self._pads(model,data,name,entry):
                if abs(n[1])>1e-9 or abs(tangent[1])>1e-9:
                    raise ValueError('rider support surface is outside the planar model')
                # One Jacobian pair per pad serves the relative-velocity read
                # and the paired force application alike.
                jrel=relative_point_jacobian(model,data,body,bike,point,self._jac_a,self._jac_b)
                u=jrel@data.qvel
                penetration=-gap
                # Two finite-area pressure samples split, rather than duplicate,
                # the specified stiffness/damping and carry a physical moment.
                damping = cfg.support_c_ns_m if name == 'saddle' else cfg.pedal_c_ns_m
                k,c,kx=cfg.support_k_n_m/2,damping/2,cfg.support_tangent_k_n_m/2
                normal,radial_energy=_normal_contact(penetration,-float(u@n),k,c)
                state=self.states[key];xi=state.xi;transport_loss=0.
                if state.tangent is not None:
                    alignment=float(state.tangent@tangent)
                    # An opposite face is a new material contact, not a shear
                    # spring that can be carried through the pedal's solid core.
                    transported=xi*alignment if alignment>=0. else 0.
                    transport_loss=.5*kx*(xi*xi-transported*transported);xi=transported
                if not self.enabled[name] or not inside:
                    normal=0.;radial_energy=0.
                new_xi,friction,brush_loss=_brush_step(xi,float(u@tangent),0.,normal,kx,cfg.support_mu,cfg.support_length_m,dt)
                f=normal*n+friction*tangent
                qfrc+=jrel.T@f
                if name.endswith('_pedal'): delivered+=float(f@jrel[:,self.crank_dof])
                shear=.5*kx*new_xi**2
                energy+=radial_energy+shear;loss+=max(transport_loss,0.)+brush_loss
                radial_loss=(normal-k*max(penetration,0.))*(-float(u@n))
                if self.enabled[name] and inside and penetration>0:radial_power+=max(radial_loss,0.)
                group_normal+=normal
                group_vertical+=float(f[2])
                in_platform=in_platform or inside
                minimum_gap=min(minimum_gap,gap)
                if detailed:
                    group_tangent+=friction;group_force+=f
                    group_moment+=np.cross(point-data.xpos[body],f)
                    group_radial+=radial_energy;group_shear+=shear;group_power+=float(f@u)
                    patches.append({'in_platform':inside,'normal_load_n':normal,'gap_m':gap,
                        'point_m':point.tolist(),'force_on_rider_n':f.tolist(),
                        'radial_energy_j':radial_energy,'shear_energy_j':shear})
                new_states[key]=_SupportState(new_xi,tangent.copy())
            diagnostics[name]={'enabled':self.enabled[name],
                'in_platform':in_platform,'normal_load_n':group_normal,'gap_m':minimum_gap,
                'vertical_force_on_rider_n':group_vertical}
            if detailed:
                diagnostics[name].update({'tangent_force_n':group_tangent,'patches':patches,
                'force_on_rider_n':group_force.tolist(),'force_on_bike_n':(-group_force).tolist(),
                'moment_about_rider_origin_nm':group_moment.tolist(),
                'radial_energy_j':group_radial,'shear_energy_j':group_shear,'relative_power_w':group_power})
        if self.welded_grip:
            # The connect equalities hold each hand on its own bar point:
            # report each reaction instead of the disabled springs, and never
            # release. 'grip' stays the aggregate for consumers that read the
            # pair; 'grip_left'/'grip_right' carry the per-hand detail.
            R=data.xmat[self.steer].reshape(3,3)
            total_force=np.zeros(3)
            for side in ('left','right'):
                connect=self._grip_connect[side]
                anchor=grip=data.xpos[self.steer]+R@self.grip_anchor_local[side]
                diagnostics['grip_'+side]={'enabled':True,'reachable':True,
                    'hand_gap_m':connect.translation_residual_m(model,data,rows=eq_rows)}
                if detailed:
                    force,_=self._settled('grip_'+side)
                    total_force+=force
                    diagnostics['grip_'+side].update({
                        'overloaded':False,'trial_pair_force_n':float(np.linalg.norm(force)),
                        'pair_force_limit_n':cfg.grip_pair_force_limit_n,
                        'release_loss_j':0.,
                        'shoulder_distance_m':hypot(*(anchor-data.xpos[self.shoulders[side]])),
                        'arm_reach_m':self.arm_reach,
                        'point_m':grip.tolist(),
                        'force_on_rider_n':force.tolist(),
                        'force_on_bike_n':(-force).tolist(),'elastic_energy_j':0.})
            left,right=(diagnostics['grip_'+s] for s in ('left','right'))
            diagnostics['grip']={'enabled':True,'reachable':True,
                'hand_gap_m':max(left['hand_gap_m'],right['hand_gap_m'])}
            if detailed:
                diagnostics['grip'].update({
                    'overloaded':False,'trial_pair_force_n':float(np.linalg.norm(total_force)),
                    'pair_force_limit_n':cfg.grip_pair_force_limit_n,
                    'release_loss_j':0.,
                    'shoulder_distance_m':max(left['shoulder_distance_m'],right['shoulder_distance_m']),
                    'arm_reach_m':self.arm_reach,
                    'force_on_rider_n':total_force.tolist(),
                    'force_on_bike_n':(-total_force).tolist(),'elastic_energy_j':0.})
            self.states,self.grip_xi_local,self.diagnostics=new_states,self.grip_xi_local,diagnostics
            self.elastic_energy_j,self.loss_step_j=energy,loss
            self.radial_dissipation_power_w=radial_power
            if self.welded_pedals:
                delivered=self._settled_crank_torque_nm()
            self.delivered_crank_torque_nm=delivered
            self.pending_release_loss_j=0.; self.last_time_s=time
            return qfrc
        # Spring grip: two hands, one 'grip' enable flag. Each hand grasps
        # its own bar point with its own spring xi; the pair force limit is
        # split evenly so total hand strength is preserved.
        R=data.xmat[self.steer].reshape(3,3)
        total_force=np.zeros(3); pair_energy=0.; pair_loss=0.; overloaded=False
        per_side={}
        for side in ('left','right'):
            grip=data.xpos[self.steer]+R@self.grip_anchor_local[side]
            hand=data.site_xpos[self.grip_sites[side]]
            shoulder_gap=grip-data.xpos[self.shoulders[side]]
            hand_gap=hand-grip
            reachable=(hypot(*shoulder_gap)<=self.arm_reach+1e-6
                       and hypot(*hand_gap)<=cfg.grip_release_distance_m)
            active=self.enabled['grip'] and reachable
            old=self.grip_xi_local[side]
            force=np.zeros(3); grip_energy=0.; trial_norm=0.
            if active:
                jrel=relative_point_jacobian(model,data,self.forearms[side],self.steer,
                                             grip,self._jac_a,self._jac_b)
                relative=jrel@data.qvel
                new,force_local,grip_energy,grip_loss=_grip_step(
                    old,R.T@relative,cfg.grip_k_n_m,cfg.grip_c_ns_m,dt)
                force=R@force_local
                trial_norm=float(np.linalg.norm(force))
                if cfg.grip_pair_force_limit_n is not None:
                    from bike_sim.physics.grip_release import release_if_overloaded
                    force,release_loss,side_overloaded=release_if_overloaded(force,
                        .5*cfg.grip_k_n_m*float(old@old),cfg.grip_pair_force_limit_n/2.)
                    if side_overloaded:
                        overloaded=True
                        new=np.zeros(3); grip_energy=0.; force=np.zeros(3)
                        grip_loss=release_loss
                qfrc+=jrel.T@force
            else:
                new=np.zeros(3); grip_loss=.5*cfg.grip_k_n_m*float(old@old)
            if not reachable:
                self.enabled['grip']=False
            self.grip_xi_local[side]=new
            pair_energy+=grip_energy; pair_loss+=grip_loss; total_force+=force
            per_side[side]=dict(reachable=reachable,active=active,old=old,
                                grip=grip,hand_gap=hand_gap,
                                shoulder_gap=shoulder_gap,trial_norm=trial_norm,
                                grip_energy=grip_energy,grip_loss=grip_loss,
                                force=force)
        if overloaded:
            # A grip that lets go does so with both hands at once.
            self.enabled['grip']=False
        energy+=pair_energy; loss+=pair_loss
        for side in ('left','right'):
            s=per_side[side]
            diagnostics['grip_'+side]={'enabled':bool(s['active']),'reachable':bool(s['reachable']),
                'overloaded':overloaded,'trial_pair_force_n':s['trial_norm'],
                'pair_force_limit_n':cfg.grip_pair_force_limit_n,
                'release_loss_j':s['grip_loss'] if not s['active'] else 0.,
                'shoulder_distance_m':hypot(*s['shoulder_gap']),
                'arm_reach_m':self.arm_reach,
                'hand_gap_m':hypot(*s['hand_gap']),
                'point_m':s['grip'].tolist(),'force_on_rider_n':s['force'].tolist(),
                'force_on_bike_n':(-s['force']).tolist(),'elastic_energy_j':s['grip_energy']}
        left,right=(per_side[s] for s in ('left','right'))
        diagnostics['grip']={'enabled':any(s['active'] for s in per_side.values()),
                             'reachable':all(s['reachable'] for s in per_side.values()),
                             'overloaded':overloaded,
                             'trial_pair_force_n':float(np.linalg.norm(total_force)),
                             'pair_force_limit_n':cfg.grip_pair_force_limit_n,
                             'release_loss_j':(pair_loss if not all(s['active']
                                               for s in per_side.values()) else 0.),
                             'shoulder_distance_m':max(hypot(*left['shoulder_gap']),
                                                      hypot(*right['shoulder_gap'])),
                             'arm_reach_m':self.arm_reach,
                             'hand_gap_m':max(hypot(*left['hand_gap']),hypot(*right['hand_gap'])),
                             'force_on_rider_n':total_force.tolist(),
                             'force_on_bike_n':(-total_force).tolist(),
                             'elastic_energy_j':pair_energy}
        self.states,self.diagnostics=new_states,diagnostics
        self.elastic_energy_j,self.loss_step_j=energy,loss
        self.radial_dissipation_power_w=radial_power
        if self.welded_pedals:
            delivered=self._settled_crank_torque_nm()
        self.delivered_crank_torque_nm=delivered
        self.pending_release_loss_j=0.; self.last_time_s=time
        return qfrc
