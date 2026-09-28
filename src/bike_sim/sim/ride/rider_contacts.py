"""Unilateral rider supports and a releasable grip, all with paired reactions."""
import copy
from dataclasses import dataclass
import numpy as np
from bike_sim.physics.checks import array, scalar
from bike_sim.physics.tire import normal_contact, brush_step
from bike_sim.sim.ride.physical_mapping import resolve_id, point_velocity


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
        self.forearm=resolve_id(model,obj.mjOBJ_BODY,'rider_forearm_pair')
        self.shoulder=resolve_id(model,obj.mjOBJ_BODY,'rider_upper_arm_pair')
        self.grip_site=resolve_id(model,obj.mjOBJ_SITE,'site_rider_grip')
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
        self.reset(model,None)

    def reset(self,model,data):
        self.enabled={name:True for name in self.CONTACTS}
        self.states={name:_SupportState() for name in self.supports}
        self.grip_xi_local=np.zeros(3)
        self.grip_anchor_local=None
        if data is not None:
            frame_R=data.xmat[self.frame].reshape(3,3)
            world_anchor=data.xpos[self.frame]+frame_R@self.pose.grip
            self.grip_anchor_local=data.xmat[self.steer].reshape(3,3).T@(world_anchor-data.xpos[self.steer])
        self.elastic_energy_j=0.
        self.loss_step_j=0.
        self.radial_dissipation_power_w=0.
        self.delivered_crank_torque_nm=0.
        self.diagnostics={}
        self.last_time_s=None
        self.pending_release_loss_j=0.

    def set_enabled(self,name,enabled):
        if name not in self.CONTACTS or not isinstance(enabled,bool):
            raise ValueError('invalid rider contact enable request')
        if not enabled and self.enabled[name]:
            if name=='grip':
                self.pending_release_loss_j+=.5*self.config.grip_k_n_m*float(self.grip_xi_local@self.grip_xi_local)
                self.grip_xi_local[:]=0.
            else:
                state=self.states[name]
                self.pending_release_loss_j+=.5*self.config.support_tangent_k_n_m*state.xi**2
                self.states[name]=_SupportState()
                # Radial support energy is removed by an explicit release too.
                self.pending_release_loss_j+=self.diagnostics.get(name,{}).get('radial_energy_j',0.)
        self.enabled[name]=enabled

    def release_all(self):
        for name in self.CONTACTS:
            self.set_enabled(name,False)

    def restart_clock(self):
        self.last_time_s=None

    def stored_energy(self, model, data):
        """Current support geometry plus persistent shear, without a force update."""
        cfg = self.config
        energy = .5*cfg.grip_k_n_m*float(self.grip_xi_local @ self.grip_xi_local)
        for name, (_, site, _, geom) in self.supports.items():
            state = self.states[name]
            energy += .5*cfg.support_tangent_k_n_m*state.xi**2
            if not self.enabled[name]:
                continue
            R = data.geom_xmat[geom].reshape(3, 3)
            origin = data.geom_xpos[geom]+R[:, 2]*model.geom_size[geom, 2]
            local = R.T @ (data.site_xpos[site]-origin)
            if abs(local[0]) <= model.geom_size[geom, 0] and abs(local[1]) <= model.geom_size[geom, 1]+1e-8:
                energy += .5*cfg.support_k_n_m*max(-float(local[2]), 0.)**2
        return energy

    def compute_qfrc(self,model,data,dt,*,advance=True):
        dt=scalar(dt,'rider contact dt',positive=True)
        if not advance:
            probe=copy.copy(self)
            probe.states=copy.deepcopy(self.states)
            probe.grip_xi_local=self.grip_xi_local.copy()
            probe.enabled=self.enabled.copy()
            probe.last_time_s=None
            return probe.compute_qfrc(model,data,dt)
        time=float(data.time)
        if self.last_time_s is not None and time<=self.last_time_s:
            raise ValueError('rider contact state advances only once per timestamp')
        if self.grip_anchor_local is None:
            raise RuntimeError('initialize rider contact anchors before evaluation')
        cfg=self.config
        qfrc=np.zeros(model.nv)
        energy=0.; loss=self.pending_release_loss_j; radial_power=0.; delivered=0.
        new_states={}; diagnostics={}
        for name,(body,site,bike,geom) in self.supports.items():
            R=data.geom_xmat[geom].reshape(3,3)
            n,tangent=R[:,2],R[:,0]
            if abs(n[1])>1e-9 or abs(tangent[1])>1e-9:
                raise ValueError('rider support surface is outside the planar model')
            anchor=np.array(data.site_xpos[site],copy=True)
            origin=data.geom_xpos[geom]+n*model.geom_size[geom,2]
            local=R.T@(anchor-origin)
            in_platform=(abs(local[0])<=model.geom_size[geom,0]
                         and abs(local[1])<=model.geom_size[geom,1]+1e-8)
            point=anchor-local[2]*n
            u=(point_velocity(model,data,body,point)-point_velocity(model,data,bike,point))
            penetration=-float(local[2])
            normal,radial_energy=normal_contact(penetration,-float(u@n),cfg.support_k_n_m,cfg.support_c_ns_m)
            state=self.states[name]; xi=state.xi
            transport_loss=0.
            if state.tangent is not None:
                transported=xi*float(state.tangent@tangent)
                transport_loss=.5*cfg.support_tangent_k_n_m*(xi*xi-transported*transported)
                xi=transported
            if self.enabled[name] and not in_platform and self.diagnostics.get(name,{}).get('in_platform',False):
                loss+=self.diagnostics[name].get('radial_energy_j',0.)
            if not self.enabled[name] or not in_platform:
                normal=0.; radial_energy=0.
            new_xi,friction,brush_loss=brush_step(xi,float(u@tangent),0.,normal,
                cfg.support_tangent_k_n_m,cfg.support_mu,cfg.support_length_m,dt)
            f=normal*n+friction*tangent
            contribution=np.zeros(model.nv)
            apply_internal_force(model,data,body,bike,point,f,contribution)
            qfrc+=contribution
            if name.endswith('_pedal'):
                delivered+=float(contribution[self.crank_dof])
            energy+=radial_energy+.5*cfg.support_tangent_k_n_m*new_xi**2
            loss+=max(transport_loss,0.)+brush_loss
            radial_loss=(normal-cfg.support_k_n_m*max(penetration,0.))*(-float(u@n))
            if self.enabled[name] and in_platform and penetration>0:
                radial_power+=max(radial_loss,0.)
            diagnostics[name]={
                'enabled':self.enabled[name], 'in_platform':bool(in_platform),
                'normal_load_n':normal, 'tangent_force_n':friction, 'gap_m':float(local[2]),
                'point_m':point.tolist(), 'force_on_rider_n':f.tolist(), 'force_on_bike_n':(-f).tolist(),
                'radial_energy_j':radial_energy, 'shear_energy_j':.5*cfg.support_tangent_k_n_m*new_xi**2,
                'relative_power_w':float(f@u),
            }
            new_states[name]=_SupportState(new_xi,tangent.copy())
        R=data.xmat[self.steer].reshape(3,3)
        grip=data.xpos[self.steer]+R@self.grip_anchor_local
        hand=data.site_xpos[self.grip_site]
        reachable=(np.linalg.norm(grip-data.xpos[self.shoulder])<=self.arm_reach+1e-6
                   and np.linalg.norm(hand-grip)<=cfg.grip_release_distance_m)
        grip_active=self.enabled['grip'] and reachable
        old=self.grip_xi_local
        if grip_active:
            relative=point_velocity(model,data,self.forearm,grip)-point_velocity(model,data,self.steer,grip)
            new,force_local,grip_energy,grip_loss=grip_step(old,R.T@relative,cfg.grip_k_n_m,cfg.grip_c_ns_m,dt)
            force=R@force_local
            apply_internal_force(model,data,self.forearm,self.steer,grip,force,qfrc)
        else:
            new=np.zeros(3); force=np.zeros(3); grip_energy=0.
            grip_loss=.5*cfg.grip_k_n_m*float(old@old)
            if not reachable:
                self.enabled['grip']=False
        energy+=grip_energy; loss+=grip_loss
        diagnostics['grip']={'enabled':bool(grip_active),'reachable':bool(reachable),
                             'point_m':grip.tolist(),'force_on_rider_n':force.tolist(),
                             'force_on_bike_n':(-force).tolist(),'elastic_energy_j':grip_energy}
        self.states,self.grip_xi_local,self.diagnostics=new_states,new,diagnostics
        self.elastic_energy_j,self.loss_step_j=energy,loss
        self.radial_dissipation_power_w=radial_power
        self.delivered_crank_torque_nm=delivered
        self.pending_release_loss_j=0.; self.last_time_s=time
        return qfrc
