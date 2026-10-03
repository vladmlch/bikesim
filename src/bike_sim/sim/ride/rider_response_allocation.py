"""Allocate muscles against the full current-state forward mechanical response."""
import numpy as np
import mujoco
from scipy.optimize import LinearConstraint,linprog

from bike_sim.physics.rider_allocation import allocate_effort, grip_constraints, Allocation, _residual
from bike_sim.sim.ride.rider_dynamics import (ConstraintDynamics, attachment_force_map, search_constraint_modes)
from bike_sim.sim.ride.support_geometry import _upper_box_face

# Strict measured budgets have no numerical slack. Keep planned forces/moments
# inside them by more than the unchanged 1e-7 optimization residual tolerance.
SUPPORT_FORCE_GUARD_N=1e-5
SUPPORT_MOMENT_GUARD_NM=1e-6
GRIP_RADIUS_GUARD_FRACTION=1e-6


def linear_region_infeasible(matrix,bound,lower,upper):
    """Reject only a proven-empty linear relaxation, at the existing tolerance."""
    result=linprog(np.zeros(len(lower)),A_ub=matrix,b_ub=bound+1e-7,
        bounds=np.column_stack((lower,upper)),method='highs',
        options={'primal_feasibility_tolerance':1e-9,'dual_feasibility_tolerance':1e-9})
    return result.status==2


def allocation_support_point(model,data,controller,name,meta):
    if name.startswith('grip'):
        body=meta['other_body']
        return data.xpos[body]+data.xmat[body].reshape(3,3)@model.eq_data[meta['eq'],3:6]
    site=model.site('site_rider_saddle').id if name=='saddle' else controller.soles[name]
    return np.asarray(data.site_xpos[site])


def allocation_grip_constraints(force_map,direction,*,pulling,limit_n):
    constraints=grip_constraints(force_map,direction,pulling=pulling,
        limit_n=limit_n*(1.-GRIP_RADIUS_GUARD_FRACTION) if pulling else limit_n)
    if not pulling:
        constraints[0].lb[:]=SUPPORT_FORCE_GUARD_N
    return constraints


def support_rows(force_map, normal, *, kind, config, weight_n, half_length_m=0.):
    """Unilateral force and COP inequalities on a physical affine wrench map."""
    normal=np.asarray(normal)
    tangent=np.array([-normal[1],normal[0]])
    fn=normal@force_map[:2];ft=tangent@force_map[:2]
    mu=config.saddle_mu if kind=='saddle' else config.foot_mu
    floor=(config.saddle_reserve_weight_fraction*weight_n if kind=='saddle'
           else config.pedal_min_normal_n)
    rows=[ft-mu*fn,-ft-mu*fn,-fn]
    bounds=[-SUPPORT_FORCE_GUARD_N,-SUPPORT_FORCE_GUARD_N,-floor-SUPPORT_FORCE_GUARD_N]
    if half_length_m > 0.:
        rows.extend((force_map[2]-half_length_m*fn,-force_map[2]-half_length_m*fn))
        bounds.extend((-SUPPORT_MOMENT_GUARD_NM,-SUPPORT_MOMENT_GUARD_NM))
    return np.asarray(rows),np.asarray(bounds),floor


def allocate_response(c, model, data, torque_intent, *, effort_scale=1.,dt_s=None,steady_state=False):
    """Optimize tau, positive-power epigraph and one fixed affine coordinate.

    All body accelerations and support wrenches are consequences of tau under
    the current soft constraint law. They are not independent force wishes.
    """
    cfg=c.config;names=tuple(c.joints);nt=len(names)
    activation_enabled=cfg.activation_tau_s>0. and not steady_state
    constant=2*nt;base_size=constant+1;nz=base_size+(nt if activation_enabled else 0)
    excitation_indices=np.arange(base_size,nz);torque_indices=np.arange(nt)
    dofs=np.array([c.joints[name][1] for name in names],dtype=int)
    speeds=np.asarray(data.qvel[dofs])
    scale=50.;power_scale=450.
    lower=np.full(nz,-np.inf);upper=np.full(nz,np.inf)
    for i,name in enumerate(names):
        qa,_,aid=c.joints[name]
        positive=negative=cfg.joint_limit_nm
        if c.strength is not None:
            angle=c.anatomical_joint_angle(name,data.qpos[qa])
            positive=min(positive,c.strength_capacity(name,angle,speeds[i],1.))
            negative=min(negative,c.strength_capacity(name,angle,speeds[i],-1.))
        if speeds[i]>0.:
            positive=min(positive,cfg.joint_power_limit_w/speeds[i])
            if speeds[i]>=cfg.joint_speed_limit_rad_s:positive=0.
        elif speeds[i]<0.:
            negative=min(negative,-cfg.joint_power_limit_w/speeds[i])
            if -speeds[i]>=cfg.joint_speed_limit_rad_s:negative=0.
        lower[i],upper[i]=-negative/scale,positive/scale
        if model.actuator_forcelimited[aid]:
            lower[i]=max(lower[i],model.actuator_forcerange[aid,0]/scale)
            upper[i]=min(upper[i],model.actuator_forcerange[aid,1]/scale)
        if model.actuator_ctrllimited[aid]:
            lower[i]=max(lower[i],model.actuator_ctrlrange[aid,0]/scale)
            upper[i]=min(upper[i],model.actuator_ctrlrange[aid,1]/scale)
    lower[nt:2*nt]=0.
    upper[nt:2*nt]=(cfg.active_positive_power_limit_w/power_scale
                    if cfg.active_positive_power_limit_w is not None else np.inf)
    lower[constant]=upper[constant]=1.
    target=np.zeros(nz);target[:nt]=np.asarray(torque_intent)/scale;target[constant]=1.
    activation=None
    activation_scales=np.ones(nz);activation_scales[:nt]=scale
    if activation_enabled:
        from bike_sim.sim.ride.rider_activation_allocation import ActivationLaw
        lower[excitation_indices]=lower[:nt];upper[excitation_indices]=upper[:nt]
        activation_scales[excitation_indices]=scale
        activation=ActivationLaw(c.active_state,speeds,lower[:nt]*scale,upper[:nt]*scale,
            dt_s=float(model.opt.timestep) if dt_s is None else dt_s,
            tau_s=cfg.activation_tau_s,power_limit=cfg.active_positive_power_limit_w)
        excitation=np.clip((np.asarray(torque_intent)-(1.-activation.gain)*c.active_state)/activation.gain,
                           lower[:nt]*scale,upper[:nt]*scale)
        target[excitation_indices]=excitation/scale
        target[:nt]=activation.delivered(excitation)/scale
    target=np.clip(target,lower,upper)
    kernel=ConstraintDynamics(model,data,dofs)
    from bike_sim.physics.rider_activation import limit_positive_power
    fallback=target.copy()
    if activation is None and cfg.active_positive_power_limit_w is not None:
        fallback[:nt]=limit_positive_power(fallback[:nt]*scale,speeds,cfg.active_positive_power_limit_w)/scale
    fallback[nt:2*nt]=np.maximum(fallback[:nt]*scale*speeds,0.)/power_scale
    prefilter_rejections=0
    weight=c.rider_mass*float(np.linalg.norm(model.opt.gravity))
    projected={};normals={};half_lengths={};points={}
    for name,meta in c._alloc_attachments.items():
        point=allocation_support_point(model,data,c,name,meta)
        points[name]=point
        projected[name]=attachment_force_map(model,data,kernel,meta['eq'],meta['rider_body'],point)
        if not name.startswith('grip'):
            geom=model.geom('geom_saddle').id if name=='saddle' else c.pedal_geoms[name]
            normal=_upper_box_face(data.geom_xpos[geom],data.geom_xmat[geom].reshape(3,3),model.geom_size[geom])[1]
            normals[name]=normal[[0,2]]
            half_lengths[name]=(cfg.saddle_patch_half_length_m if name=='saddle'
                                else float(model.geom_size[geom,0]))
    def solve_mode(response):
        nonlocal prefilter_rejections
        force_maps={}
        for name,mapping in projected.items():
            force=np.zeros((3,nz))
            force[:,:nt]=mapping@response.force_matrix*scale
            force[:,constant]=mapping@response.force_offset
            force_maps[name]=force
        rows=[];bounds=[]
        for row,bound in zip(response.domain_matrix,response.domain_bound):
            full=np.zeros(nz);full[:nt]=row*scale
            rows.append(full);bounds.append(bound)
        for name,force in force_maps.items():
            if name.startswith('grip'):continue
            gr,hr,_=support_rows(force,normals[name],kind='saddle' if name=='saddle' else 'foot',
                config=cfg,weight_n=weight,half_length_m=half_lengths[name])
            rows.extend(gr);bounds.extend(hr)
        for i in range(nt):
            row=np.zeros(nz);row[i]=speeds[i]*scale;row[nt+i]=-power_scale
            rows.append(row);bounds.append(0.)
        if cfg.active_positive_power_limit_w is not None:
            row=np.zeros(nz);row[nt:2*nt]=power_scale
            rows.append(row);bounds.append(cfg.active_positive_power_limit_w)
        g=np.asarray(rows).reshape(-1,nz);h=np.asarray(bounds)
        # Initialization can visit geometries with no legal seated command.
        # Do not spend thousands of nonlinear iterations in an empty region.
        # All physical inequalities remain in the subsequent certification.
        fallback_error=_residual(fallback,np.zeros((0,nz)),np.zeros(0),g,h,(),lower,upper)
        base_empty=fallback_error>1e-7 and linear_region_infeasible(g,h,lower,upper)
        if base_empty:
            prefilter_rejections+=1
        def solve(branch,x0):
            nonlocal prefilter_rejections
            extras=[]
            if activation is not None:
                extras.append(activation.constraint(torque_indices,excitation_indices,activation_scales))
            for side,pulling in zip(('left','right'),branch):
                name='grip_'+side
                if name not in force_maps:continue
                direction=(data.xpos[c.pelvis]-points[name])[[0,2]]
                direction=direction/np.linalg.norm(direction)
                extras.extend(allocation_grip_constraints(force_maps[name][:2],direction,pulling=pulling,
                                                limit_n=cfg.grip_pull_per_hand_n))
            if base_empty:
                error=_residual(fallback,np.zeros((0,nz)),np.zeros(0),g,h,extras,lower,upper)
                return Allocation(fallback.copy(),False,error)
            linear=[constraint for constraint in extras if isinstance(constraint,LinearConstraint)]
            if linear:
                branch_g=np.vstack([g,*[-np.atleast_2d(constraint.A) for constraint in linear]])
                branch_h=np.r_[h,*[-np.atleast_1d(constraint.lb) for constraint in linear]]
                branch_error=_residual(fallback,np.zeros((0,nz)),np.zeros(0),branch_g,branch_h,(),lower,upper)
                if branch_error>1e-7 and linear_region_infeasible(branch_g,branch_h,lower,upper):
                    prefilter_rejections+=1
                    error=_residual(fallback,np.zeros((0,nz)),np.zeros(0),g,h,extras,lower,upper)
                    return Allocation(fallback.copy(),False,error)
            result=allocate_effort(target,np.zeros((0,nz)),np.zeros(0),g,h,lower,upper,
                                   extra_constraints=extras,x0=x0)
            if activation is not None:
                projected=activation.project(result.solution,torque_indices,excitation_indices,activation_scales)
                error=_residual(projected,np.zeros((0,nz)),np.zeros(0),g,h,extras,lower,upper)
                result=Allocation(projected,error<=1e-7,error)
            return result
        result,branch=c._select_allocation_branch(target,solve)
        return result,(branch,force_maps)
    result,payload,response,mode_count,complete=search_constraint_modes(
        kernel,target[:nt]*scale,solve_mode,torque_of=lambda result:result.solution[:nt]*scale)
    branch,force_maps=payload
    torque=result.solution[:nt]*scale
    acceleration=response.acceleration_offset+response.acceleration_matrix@torque
    wrenches={name:matrix@result.solution for name,matrix in force_maps.items()}
    excitation=(torque.copy() if activation is None else result.solution[excitation_indices]*scale)
    activation_state=(torque.copy() if activation is None else activation.latent(excitation))
    return torque,dict(feasible=result.feasible,violation=result.violation,grip_branch=branch,
        effort_scale=effort_scale,saddle_normal_lower_bound_n=cfg.saddle_reserve_weight_fraction*weight,
        solution_tau=torque.copy(),solution_qddot=acceleration[c._rider_dofs],
        solution_base_qacc=acceleration[c._bike_dofs],solution_power=result.solution[nt:2*nt].copy()*power_scale,
        solution_wrenches=np.concatenate([wrenches[name][:2] for name in c._alloc_attachments]),
        solution_moments=np.array([wrenches[name][2] for name in c._alloc_attachments]),
        solution_constraint_forces=response.force_offset+response.force_matrix@torque,
        solution_excitation_nm=excitation,solution_activation_state_nm=activation_state,
        solution_active_lower_nm=lower[:nt]*scale,solution_active_upper_nm=upper[:nt]*scale,
        dynamics_model='current_soft_constraint_response',constraint_modes_tried=mode_count,
        constraint_mode_search_complete=complete,
        linear_mode_rejections=prefilter_rejections,
        grip_radius_guard_fraction=GRIP_RADIUS_GUARD_FRACTION,
        allocation_force_guard_n=SUPPORT_FORCE_GUARD_N,allocation_moment_guard_nm=SUPPORT_MOMENT_GUARD_NM)
