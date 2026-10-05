"""Active/passive actuator bookkeeping shared by compute and solved samples."""
import numpy as np
from bike_sim.physics.rider_activation import activation_step,limit_positive_power


def activation_target(controller, requested, *, dt_s, steady_state):
    """Filter the desired active force before constrained muscle allocation."""
    tau=controller.config.activation_tau_s
    gain=1. if steady_state or tau==0. else float(-np.expm1(-dt_s/tau))
    target=(np.asarray(requested).copy() if steady_state else
            activation_step(controller.active_state,requested,dt_s,tau))
    return target,gain


def finalize_effort(controller, data, torques, *, advance, dt_s, steady_state):
    from bike_sim.sim.ride.rider_control import bounded_effort
    c=controller; cfg=c.config; names=tuple(c.joints)
    speeds=np.array([data.qvel[c.joints[n][1]] for n in names])
    passive=-cfg.passive_damping_nms_rad*speeds
    # Passive damping is a DOF property solved by the engine: the requested,
    # activated, bounded, and commanded torque are all purely active muscle.
    target=np.array([torques[n] for n in names])
    enabled=(cfg.activation_tau_s>0 or cfg.active_positive_power_limit_w is not None
             or c.strength is not None)
    if enabled:
        if (advance and not steady_state and cfg.activation_tau_s>0
                and c.activation_time_s is not None and data.time<=c.activation_time_s):
            raise ValueError('rider activation advances once per timestamp')
        certificate=getattr(c,'allocation_diagnostics',{})
        excitation=certificate.get('solution_excitation_nm')
        if excitation is None:
            # Standalone callers must still obey bounded excitation; the
            # production allocators provide this same solved physical input.
            gain=1. if steady_state or cfg.activation_tau_s==0. else -np.expm1(-dt_s/cfg.activation_tau_s)
            excitation=(target-(1.-gain)*c.active_state)/gain
            excitation=bounded_effort(speeds,excitation,cfg.joint_limit_nm,
                cfg.joint_speed_limit_rad_s,cfg.joint_power_limit_w)
            excitation,_=c.strength_limited(excitation,data.qpos,data.qvel)
        excitation=np.asarray(excitation)
        activated=(excitation.copy() if steady_state or
                   (getattr(c,'spindle',False) and not c.command_enabled) else
                   activation_step(c.active_state,excitation,dt_s,cfg.activation_tau_s))
        if getattr(c,'spindle',False):
            active=np.array(list(c.limit_torques(dict(zip(names,activated)),data).values()))
            _,strength_limited=c.strength_limited(activated,data.qpos,data.qvel)
        elif 'solution_active_lower_nm' in certificate:
            active=np.clip(activated,certificate['solution_active_lower_nm'],certificate['solution_active_upper_nm'])
            _,strength_limited=c.strength_limited(activated,data.qpos,data.qvel)
        else:
            active=bounded_effort(speeds,activated,cfg.joint_limit_nm,cfg.joint_speed_limit_rad_s,cfg.joint_power_limit_w)
            active,strength_limited=c.strength_limited(active,data.qpos,data.qvel)
        if cfg.active_positive_power_limit_w is not None:
            active=limit_positive_power(active,speeds,cfg.active_positive_power_limit_w)
        if not np.allclose(active,target,rtol=1e-9,atol=1e-7):
            raise ArithmeticError('allocated rider torque is outside final actuator limits')
        if advance:
            c.active_state=(active.copy() if getattr(c,'spindle',False) else activated.copy())
            # Without an activation time constant there is no filter state to
            # protect from a repeated call at the same timestamp.
            c.activation_time_s=(None if steady_state or cfg.activation_tau_s==0
                                 else float(data.time))
        torques=dict(zip(names,map(float,active)))
    else:
        active=np.array([torques[n] for n in names])
        excitation=target.copy()
        strength_limited=()
    c.effort_diagnostics={
        'rider_active_request_nm':dict(zip(names,map(float,excitation))),
        'rider_active_delivered_nm':dict(zip(names,map(float,active))),
        'rider_positive_power_w':float(np.maximum(active*speeds,0.).sum()),
        'rider_passive_power_w':float(passive@speeds),
        'rider_activation_saturated':bool(not np.allclose(active,excitation,rtol=1e-10,atol=1e-10)),
        'rider_strength_limited':strength_limited,
        'rider_effort_budget_exceeded':False,
        'rider_active_positive_power_limit_w':cfg.active_positive_power_limit_w,
        'rider_effort_observation':'incoming_request',
    }
    for i,name in enumerate(names):
        c.last_terms[name].update(active_request_nm=float(excitation[i]),active_delivered_nm=float(active[i]),
                                  passive_damping_nm=float(passive[i]),command_nm=float(torques[name]))
    return torques


def joint_effort_limits(active, speeds, config):
    """Solved per-joint positive power and absolute speed, without clipping truth."""
    positive = {name:max(float(torque)*float(speeds[name]),0.) for name,torque in active.items()}
    return dict(rider_joint_positive_power_w=positive,
        rider_joint_power_violations=tuple(name for name,power in positive.items()
            if power > config.joint_power_limit_w+1e-9),
        rider_joint_speed_violations=tuple(name for name in active
            if abs(float(speeds[name])) > config.joint_speed_limit_rad_s+1e-9))


def solved_effort(controller, data, incoming_state, dt_s):
    """Use solved actuator forces, not ctrl, and the same interval velocity.

    Clipping/implicit evaluation may violate an incoming power projection. Such
    overruns remain explicit diagnostics, never an endpoint velocity correction.
    """
    incoming_qpos,incoming_velocity=incoming_state
    c=controller; active={}; passive_power=0.; positive=0.
    for name,(_,dof,aid) in c.joints.items():
        # actuator_force is pure muscle torque now; passive tissue damping is
        # the DOF damping row of qfrc_passive at the solved state.
        damping=float(data.qfrc_passive[dof])
        delivered=float(data.actuator_force[aid])
        active[name]=delivered
        positive+=max(delivered*float(incoming_velocity[dof]),0.)
        passive_power+=damping*float(incoming_velocity[dof])
        c.last_terms[name].update(solved_force_nm=delivered,
                                  solved_active_nm=delivered,solved_passive_nm=damping)
    strength_violations=c.strength_violations(active,incoming_qpos,incoming_velocity)
    limit=c.config.active_positive_power_limit_w
    c.effort_diagnostics.update(joint_effort_limits(active,
        {name:incoming_velocity[dof] for name,(_,dof,_) in c.joints.items()},c.config))
    c.effort_diagnostics.update(rider_active_delivered_nm=active,rider_positive_power_w=positive,
        rider_passive_power_w=passive_power,rider_positive_work_step_j=positive*dt_s,
        rider_passive_work_step_j=passive_power*dt_s,
        rider_strength_violations=strength_violations,
        rider_effort_budget_exceeded=bool(limit is not None and positive>limit+1e-9),
        rider_effort_observation='solved_actuator_force_at_incoming_interval')
    return dict(c.effort_diagnostics)
