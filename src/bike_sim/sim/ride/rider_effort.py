"""Active/passive actuator bookkeeping shared by compute and solved samples."""
import numpy as np
from bike_sim.physics.rider_activation import activation_step,limit_positive_power


def finalize_effort(controller, data, torques, *, advance, dt_s, steady_state):
    from bike_sim.sim.ride.rider_control import bounded_effort
    c=controller; cfg=c.config; names=tuple(c.joints)
    speeds=np.array([data.qvel[c.joints[n][1]] for n in names])
    passive=-cfg.joint_kd_nms_rad*speeds
    target=np.array([c.last_terms[n]['requested_nm'] for n in names])-passive
    enabled=cfg.activation_tau_s>0 or cfg.active_positive_power_limit_w is not None
    if enabled:
        if advance and not steady_state and c.activation_time_s is not None and data.time<=c.activation_time_s:
            raise ValueError('rider activation advances once per timestamp')
        activated=(target.copy() if steady_state else
                   activation_step(c.active_state,target,dt_s,cfg.activation_tau_s))
        active=bounded_effort(speeds,activated,cfg.joint_limit_nm,cfg.joint_speed_limit_rad_s,cfg.joint_power_limit_w)
        if cfg.active_positive_power_limit_w is not None:
            active=limit_positive_power(active,speeds,cfg.active_positive_power_limit_w)
        # Passive damping enters the engine's implicit solve separately. The
        # final force range still belongs to the physical joint actuator.
        commands=active+passive
        if advance:
            c.active_state=activated.copy()
            c.activation_time_s=None if steady_state else float(data.time)
        torques=dict(zip(names,map(float,commands)))
    else:
        active=np.array([torques[n] for n in names])-passive
    c.effort_diagnostics={
        'rider_active_request_nm':dict(zip(names,map(float,target))),
        'rider_active_delivered_nm':dict(zip(names,map(float,active))),
        'rider_positive_power_w':float(np.maximum(active*speeds,0.).sum()),
        'rider_passive_power_w':float(passive@speeds),
        'rider_activation_saturated':bool(not np.allclose(active,target,rtol=1e-10,atol=1e-10)),
        'rider_effort_budget_exceeded':False,
        'rider_active_positive_power_limit_w':cfg.active_positive_power_limit_w,
        'rider_effort_observation':'incoming_request',
    }
    for i,name in enumerate(names):
        c.last_terms[name].update(active_request_nm=float(target[i]),active_delivered_nm=float(active[i]),
                                  passive_damping_nm=float(passive[i]),command_nm=float(torques[name]))
    return torques


def solved_effort(controller, data, incoming_velocity, dt_s):
    """Use solved actuator forces, not ctrl, and the same interval velocity.

    Clipping/implicit evaluation may violate an incoming power projection. Such
    overruns remain explicit diagnostics, never an endpoint velocity correction.
    """
    c=controller; active={}; passive_power=0.; positive=0.
    for name,(_,dof,aid) in c.joints.items():
        damping=(-c.config.joint_kd_nms_rad*float(data.actuator_velocity[aid])
                 if c.command_enabled else 0.)
        delivered=float(data.actuator_force[aid])-damping
        active[name]=delivered
        positive+=max(delivered*float(incoming_velocity[dof]),0.)
        passive_power+=damping*float(incoming_velocity[dof])
        c.last_terms[name].update(solved_force_nm=float(data.actuator_force[aid]),
                                  solved_active_nm=delivered,solved_passive_nm=damping)
    limit=c.config.active_positive_power_limit_w
    c.effort_diagnostics.update(rider_active_delivered_nm=active,rider_positive_power_w=positive,
        rider_passive_power_w=passive_power,rider_positive_work_step_j=positive*dt_s,
        rider_passive_work_step_j=passive_power*dt_s,
        rider_effort_budget_exceeded=bool(limit is not None and positive>limit+1e-9),
        rider_effort_observation='solved_actuator_force_at_incoming_interval')
    return dict(c.effort_diagnostics)
