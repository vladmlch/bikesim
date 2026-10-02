"""Initial-condition relaxation for the fully coupled physical model.

Velocity damping/reset is confined to initialization. No running-step correction
or root stabilizer is used. Failure returns a diagnostic, never a success label.
"""
import mujoco
import numpy as np


def solve_physical_equilibrium(runtime, *, max_steps=None, tolerance=.05):
    sim = runtime.sim
    model, data = sim.model, sim.data
    cached = None
    if sim.physics_config.equilibrium_cache_enabled:
        from bike_sim.sim.ride.equilibrium_cache import load
        cached = load(runtime)
    runtime.probe_query.reset()
    sim.contact_query.reset()
    mujoco.mj_resetData(model, data)
    data.qpos[sim.root_x_qposadr] = sim.start_x_m
    root_z = runtime.address('root_z')[0]
    # Initial pose follows the compiled road, not an unrelated smooth function.
    vertices=runtime.vertices
    i=int(np.clip(np.searchsorted(vertices[:,0],sim.start_x_m)-1,0,len(vertices)-2))
    slope=(vertices[i+1,1]-vertices[i,1])/(vertices[i+1,0]-vertices[i,0])
    from bike_sim.geometry.hardpoints import compute_ground_z
    data.qpos[root_z]=float(np.interp(sim.start_x_m,vertices[:,0],vertices[:,1]))-compute_ground_z(sim.specs)/1000.+.005
    data.qpos[sim.root_pitch_qposadr]=-np.arctan(slope)
    mujoco.mj_forward(model,data)
    clearance=0.
    for geom in (sim.contact_query.front_id,sim.contact_query.rear_id):
        center=data.geom_xpos[geom]
        road=float(np.interp(center[0],vertices[:,0],vertices[:,1]))
        clearance=max(clearance,road+model.geom_size[geom,0]*np.sqrt(1+slope*slope)-center[2]+.005)
    data.qpos[root_z]+=clearance
    phase = sim.physics_config.drive.crank_phase_rad
    data.qpos[runtime.address('crank_spin')[0]] = phase
    for side in ('front', 'rear'):
        data.qpos[runtime.address(f'pedal_{side}_spin')[0]] = -phase
    if runtime.rider_control is not None:
        mujoco.mj_forward(model,data)
        frame=runtime.drive.ids['frame']
        hip=runtime.rider_control.pose.hip
        wanted=data.xpos[frame]+data.xmat[frame].reshape(3,3)@hip
        data.qpos[runtime.address('rider_root_x')[0]]=wanted[0]-hip[0]
        data.qpos[runtime.address('rider_root_z')[0]]=wanted[2]-hip[2]
        data.qpos[runtime.address('rider_root_pitch')[0]]=data.qpos[sim.root_pitch_qposadr]
    mujoco.mj_forward(model, data)
    if runtime.rider_control is not None:
        runtime.rider_control.enabled = True
        runtime.rider_control.initialize(model, data)
        runtime.rider_contacts.reset(model, data)
    runtime.drive.reset(model, data)
    if runtime.tire is not None:
        runtime.tire.reset()
    cache_hit = False
    steps = 0
    residual = float('inf')
    if cached is not None:
        from bike_sim.sim.ride.equilibrium_cache import restore_state
        _, qpos, cached_steps, cached_residual, state = cached
        if restore_state(runtime, state):
            data.qpos[:] = qpos
            data.qvel.fill(0.)
            mujoco.mj_forward(model, data)
            steps, residual, cache_hit = cached_steps, cached_residual, True
    dt = float(model.opt.timestep)
    # A smaller integration step must not silently shorten the physical settling
    # budget. The historical 40,000-step limit represented 20 seconds at 0.5 ms.
    if max_steps is None:
        max_steps = max(40000, round(20.0/dt))
    if isinstance(max_steps, bool) or not isinstance(max_steps, int) or max_steps <= 0:
        raise ValueError('equilibrium max_steps must be a positive integer')
    cycle = max(1, round(.02/dt))
    first_refine = cycle*max(1, round(sim.physics_config.equilibrium_refine_after_s/(cycle*dt)))
    refine_period = cycle*max(1, round(sim.physics_config.equilibrium_refine_period_s/(cycle*dt)))
    while not cache_hit and steps < max_steps:
        for _ in range(min(cycle, max_steps-steps)):
            runtime.apply_forces(active=False, advance=True, front=sim.physics_config.initial_front_brake, rear=sim.physics_config.initial_rear_brake)
            mujoco.mj_step(model, data)
            steps += 1
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                raise RuntimeError('non-finite physical equilibrium state')
        data.qvel.fill(0.)
        runtime.apply_forces(active=False, advance=False, front=sim.physics_config.initial_front_brake, rear=sim.physics_config.initial_rear_brake)
        mujoco.mj_forward(model, data)
        residual = float(np.max(np.abs(data.qacc)))
        if residual > tolerance and (steps == first_refine or steps % refine_period == 0):
            from bike_sim.sim.ride.equilibrium_refine import refine_equilibrium
            # Re-center the trust region while the actual equilibrium residual
            # improves. A coupled rider may need more than three position boxes;
            # stagnation returns to damped physical relaxation instead of burning
            # a fixed number of identical optimizer calls.
            for _ in range(8):
                previous = residual
                residual = refine_equilibrium(runtime,acceleration_tolerance=tolerance)
                if residual <= tolerance or residual >= previous*(1.-1e-3):
                    break
        if residual <= tolerance:
            break
    if not cache_hit and steps >= max_steps and residual > tolerance:
        names = []
        for dof in np.argsort(np.abs(data.qacc))[-5:][::-1]:
            jid = int(model.dof_jntid[dof])
            names.append((mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,jid),float(data.qacc[dof])))
        raise RuntimeError(f'physical equilibrium did not converge after {steps} steps: residual={residual:.6g}; {names}')
    stroke = sim.shock_stroke_mm
    result = {'fork_travel_mm': sim.fork_travel_mm, 'shock_stroke_mm': stroke,
            'rear_travel_mm': float(sim.solver.solve_state_from_shock_stroke(stroke)['wheel_travel']),
            'root_z_m': float(data.qpos[root_z]), 'pitch_rad': sim.pitch_rad,
            'steps': steps, 'residual_qacc': residual,
            'initial_crank_phase_rad': phase,
            'settled_crank_phase_rad': float(data.qpos[runtime.address('crank_spin')[0]]),
            'cache_hit': cache_hit}
    if sim.physics_config.equilibrium_cache_enabled and not cache_hit:
        from bike_sim.sim.ride.equilibrium_cache import save
        save(runtime, data.qpos, steps, residual)
    return result
