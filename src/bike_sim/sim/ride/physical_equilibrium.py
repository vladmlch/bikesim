"""Initial-condition relaxation for the fully coupled physical model.

Velocity damping/reset is confined to initialization. No running-step correction
or root stabilizer is used. Failure returns a diagnostic, never a success label.
"""
import copy

import mujoco
import numpy as np


def solve_physical_equilibrium(runtime, *, max_steps=None, tolerance=.05):
    sim = runtime.sim
    model, data = sim.model, sim.data
    if not np.isfinite(tolerance) or tolerance <= 0.:
        raise ValueError('equilibrium tolerance must be finite and positive')
    cached = None
    state_file = sim.physics_config.equilibrium_state_file
    if state_file is not None:
        # An explicit pose file is read-only user-managed state: the embedded
        # fingerprint, the material state and the re-measured residual are all
        # verified below, and every failure names the file. It outranks the
        # derived-name auto-cache; saving via the cache flag stays as-is.
        from bike_sim.sim.ride.equilibrium_cache import load_state_file
        cached = load_state_file(runtime, state_file)
    elif sim.physics_config.equilibrium_cache_enabled:
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
        source, qpos, cached_steps, cached_residual, state = cached
        if state_file is not None:
            if not restore_state(runtime, state):
                raise ValueError(
                    f'equilibrium state file {source}: state payload does not fit this runtime')
            data.qpos[:] = qpos
            data.qvel.fill(0.)
            runtime.apply_forces(active=False, advance=False,
                                 front=sim.physics_config.initial_front_brake,
                                 rear=sim.physics_config.initial_rear_brake)
            mujoco.mj_forward(model, data)
            residual = float(np.max(np.abs(data.qacc)))
            if residual > tolerance:
                raise ValueError(
                    f'equilibrium state file {source}: restored residual '
                    f'{residual:.6g} exceeds tolerance {tolerance}')
            steps, cache_hit = cached_steps, True
        elif restore_state(runtime, state):
            data.qpos[:] = qpos
            data.qvel.fill(0.)
            mujoco.mj_forward(model, data)
            steps, residual, cache_hit = cached_steps, cached_residual, True
    dt = float(model.opt.timestep)
    # A smaller integration step must not silently shorten the physical settling
    # budget, and a larger one must not stretch it: the budget is settling TIME,
    # not steps. Standstill tangential shear self-tunes by tread creep through
    # the regularized brake constraint; the slowest measured case (pneumatic
    # detailed brush on the savage climb's compression branch) needs ~59 s of
    # simulated time, so 80 s leaves margin without changing the step physics.
    if max_steps is None:
        max_steps = max(40000, round(80.0/dt))
    if isinstance(max_steps, bool) or not isinstance(max_steps, int) or max_steps <= 0:
        raise ValueError('equilibrium max_steps must be a positive integer')
    cycle = max(1, round(.02/dt))
    first_refine = cycle*max(1, round(sim.physics_config.equilibrium_refine_after_s/(cycle*dt)))
    refine_period = cycle*max(1, round(sim.physics_config.equilibrium_refine_period_s/(cycle*dt)))
    # The latched elastic bite is a ride-time latch, not an equilibrium
    # force: its engage/release hysteresis ratchets the wheels while the
    # damped cycles run and the standstill residual never lands. Solve the
    # pose on the friction bound alone (static_braking.py::hold_suspended).
    with runtime.brake.hold_suspended():
        while not cache_hit and steps < max_steps:
            for _ in range(min(cycle, max_steps-steps)):
                runtime.apply_forces(active=False, advance=True, front=sim.physics_config.initial_front_brake, rear=sim.physics_config.initial_rear_brake)
                interval_state = (data.qpos.copy(), data.qvel.copy())
                mujoco.mj_step(model, data)
                if runtime.rider_contacts is not None:
                    runtime.rider_contacts.settle_welds(model, data, interval_state)
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
        # Release consistency. The braked relaxation converges with the wheel
        # brakes holding whatever tangential load the pose left behind -- the
        # regularized friction bound plateaus and the angle latch pins the rest.
        # The episode starts unbraked, so that store is released across the first
        # intervals as constraint work the energy ledger rightfully rejects.
        # Re-settle the pose under the released demand: the wheel brushes are
        # adjusted to wheel statics first, then damped cycles let the chassis
        # and weld deflections follow. When no released rest exists (a grade the
        # standstill needs the brakes for), the held result is kept unchanged.
        if (not cache_hit and residual <= tolerance and steps < max_steps
                and (sim.physics_config.initial_front_brake > 0.
                     or sim.physics_config.initial_rear_brake > 0.)):
            from bike_sim.sim.ride.equilibrium_cache import _state_arrays, restore_state
            held_pose = data.qpos.copy()
            held_state = copy.deepcopy(_state_arrays(runtime))
            held_steps = steps
            released, used = _relax_released(
                runtime, max_steps=min(max_steps-steps, round(15./dt)),
                cycle=cycle, tolerance=min(tolerance, 0.01))
            steps += used
            if released is not None and released <= min(tolerance, 0.01):
                residual = released
            else:
                # No released-brake rest exists under this track/config: keep the
                # brake-assisted equilibrium rather than the drifted state.
                restore_state(runtime, held_state)
                data.qpos[:] = held_pose
                data.qvel.fill(0.)
                runtime.brake.reset()
                runtime.apply_forces(active=False, advance=False,
                                     front=sim.physics_config.initial_front_brake,
                                     rear=sim.physics_config.initial_rear_brake)
                mujoco.mj_forward(model, data)
                residual = float(np.max(np.abs(data.qacc)))
                steps = held_steps
    stroke = sim.shock_stroke_mm
    result = {'fork_travel_mm': sim.fork_travel_mm, 'shock_stroke_mm': stroke,
            'rear_travel_mm': float(sim.solver.solve_state_from_shock_stroke(stroke)['wheel_travel']),
            'root_z_m': float(data.qpos[root_z]), 'pitch_rad': sim.pitch_rad,
            'steps': steps, 'residual_qacc': residual,
            'initial_crank_phase_rad': phase,
            'settled_crank_phase_rad': float(data.qpos[runtime.address('crank_spin')[0]]),
            'cache_hit': cache_hit}
    if state_file is not None:
        result['state_file'] = str(state_file)
    if sim.physics_config.equilibrium_cache_enabled and not cache_hit:
        from bike_sim.sim.ride.equilibrium_cache import save
        save(runtime, data.qpos, steps, residual)
    return result


def _released_probe(runtime, wheel_dofs):
    """Acceleration a wheel takes from rest under the released-brake field.

    Same probe contract as the residual check above -- non-advancing, zeroed
    velocity -- but with both brake demands at the episode's standstill-zero
    and ``active=True`` so the rider path evaluates the same field the first
    live interval applies (the steady-state settle field skips the crank
    phase tracking and pedal-recovery terms the live request carries).
    """
    model, data = runtime.sim.model, runtime.sim.data
    data.qvel.fill(0.)
    runtime.apply_forces(active=True, advance=False, front=0., rear=0.)
    mujoco.mj_forward(model, data)
    return np.array([float(data.qacc[dof]) for dof in wheel_dofs])


def _solve_brush_shear(runtime, wheel_dofs, probe, *, rounds=4, atol=5e-3):
    """Newton-adjust the tangential brush deflection to released wheel statics.

    The pinned-wheel relaxation leaves each brush holding the shear the brake
    was absorbing; freed at episode start it would spin the wheel and rock the
    chassis. The wheel free body is essentially decoupled in the two spin
    DOFs, so a small secant solve on the released accelerations reaches the
    same state an infinitely patient tread creep would.
    """
    tire = runtime.tire
    if (tire is None or tire.config.backend != 'compliant_2d'
            or 'front' not in tire.states or 'rear' not in tire.states):
        return
    states = tire.states

    def set_xi(front, rear):
        for side, xi in (('front', front), ('rear', rear)):
            state = states[side]
            states[side] = type(state)(float(xi), state.tangent, state.point,
                                       state.segment, state.center)

    xi = np.array([states['front'].xi, states['rear'].xi], dtype=float)
    residual = probe()
    for _ in range(rounds):
        if np.abs(residual).max() <= atol:
            break
        jacobian = np.zeros((2, 2))
        eps = 2e-5
        for j in range(2):
            trial = xi.copy(); trial[j] += eps
            set_xi(*trial)
            jacobian[:, j] = (probe()-residual)/eps
        set_xi(*xi)
        if not np.isfinite(jacobian).all() or abs(np.linalg.det(jacobian)) < 1e-9:
            break
        xi = xi + np.linalg.solve(jacobian, -residual)
        if not np.isfinite(xi).all():
            return
        set_xi(*xi)
        residual = probe()


def _relax_released(runtime, *, max_steps, cycle, tolerance):
    """Damped relaxation under the released-brake demand.

    Returns ``(residual, steps_used)``; ``residual`` is ``None`` only when the
    wheel-free-body solve left the state unusable, and non-convergence simply
    reports the last residual so the caller can restore the held pose.
    """
    sim = runtime.sim
    model, data = sim.model, sim.data
    wheel_dofs = [runtime.address('front_wheel_spin')[1],
                  runtime.address('rear_wheel_spin')[1]]

    def probe():
        _released_probe(runtime, wheel_dofs)
        return float(np.max(np.abs(data.qacc)))

    steps = 0
    residual = float('inf')
    # Alternating shear solve and damped cycles: the solve places the wheels
    # on their released statics, the cycles let the chassis, weld deflections
    # and the coast servo follow. A couple of rounds are enough for the flat
    # stand; the budget bound keeps a grade (no released rest) cheap.
    for _ in range(3):
        if steps >= max_steps:
            break
        _solve_brush_shear(runtime, wheel_dofs,
                           lambda: _released_probe(runtime, wheel_dofs))
        while steps < max_steps:
            for _ in range(min(cycle, max_steps-steps)):
                runtime.apply_forces(active=True, advance=True, front=0., rear=0.)
                interval_state = (data.qpos.copy(), data.qvel.copy())
                mujoco.mj_step(model, data)
                if runtime.rider_contacts is not None:
                    runtime.rider_contacts.settle_welds(model, data, interval_state)
                # The live interval closes with settle_actuation: the
                # bookkeeping reads the just-solved tendon/actuator state
                # and clears the pending interval the next advancing apply
                # validates against.
                runtime.drive.settle_actuation(model, data)
                steps += 1
                if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                    raise RuntimeError('non-finite physical equilibrium state')
            data.qvel.fill(0.)
            residual = probe()
            if residual <= tolerance:
                break
        if residual <= tolerance:
            break
    return residual, steps
