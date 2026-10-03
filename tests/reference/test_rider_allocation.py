"""Constrained seated-effort allocation (R4).

The 0.75*rider_weight support heuristic is replaced by an explicit model:
rider inverse dynamics, attachment closures and budgets, directional joint
strength, a circular grip limit, and the summed positive-power cap. Intent is
only the objective; violating a budget is an invalid controller state, never
a legitimate stall.
"""
import math
from pathlib import Path

import numpy as np
import pytest
from scipy.optimize import LinearConstraint, NonlinearConstraint

from bike_sim.physics.rider_allocation import (
    Allocation, allocate_effort, grip_constraints, inverse_dynamics_rows,
)

ROOT = Path(__file__).resolve().parents[2]
WELDED = ROOT / 'examples' / 'research' / 'viewer_physics_welded.toml'


# --- plan step-1 checks -----------------------------------------------------

def test_intent_is_relaxed_but_physical_limit_is_not():
    result = allocate_effort(np.array([100.]), np.zeros((0, 1)), np.zeros(0),
        np.array([[1.]]), np.array([80.]), np.array([0.]), np.array([200.]))
    assert result.feasible
    assert abs(result.solution[0] - 80.) < 1e-6


def test_infeasible_support_is_not_reported_as_valid_stall():
    result = allocate_effort(np.array([0.]), np.array([[1.]]), np.array([-1.]),
        np.zeros((0, 1)), np.zeros(0), np.array([20.]), np.array([100.]))
    assert not result.feasible


def test_scaled_grip_uses_a_circle_not_a_componentwise_square():
    # Optimization coordinates are dimensionless; force_map restores newtons.
    scale = 300.
    constraints = grip_constraints(scale*np.eye(2), np.ones(2)/np.sqrt(2),
                                   pulling=True)
    result = allocate_effort(np.array([-250., -250.])/scale,
        np.zeros((0, 2)), np.zeros(0), np.zeros((0, 2)), np.zeros(0),
        np.full(2, -1000.)/scale, np.full(2, 1000.)/scale,
        extra_constraints=constraints)
    assert result.feasible
    force = scale*result.solution
    assert np.linalg.norm(force) <= 300.
    assert np.linalg.norm(force-np.array([-250., -250.])) > 50.


# --- kernel rigour ----------------------------------------------------------

def test_diagonal_pull_beyond_the_circle_is_infeasible():
    # (250, 250) N has norm ~353 N: allowed by a 300 N per-component square,
    # rejected by the circle.
    scale = 300.
    constraints = grip_constraints(scale*np.eye(2), np.ones(2)/np.sqrt(2),
                                   pulling=True)
    result = allocate_effort(np.zeros(2),
        np.eye(2), np.array([-250., -250.])/scale,
        np.zeros((0, 2)), np.zeros(0),
        np.full(2, -1000.)/scale, np.full(2, 1000.)/scale,
        extra_constraints=constraints)
    assert not result.feasible


def test_grip_constraint_validation():
    with pytest.raises(ValueError, match='unit direction'):
        grip_constraints(np.eye(2), np.array([2., 0.]), pulling=True)
    with pytest.raises(ValueError, match='unit direction'):
        grip_constraints(np.eye(2), np.ones(2)/np.sqrt(2),
                         pulling=True, limit_n=0.)
    with pytest.raises(ValueError, match='planar grip map'):
        grip_constraints(np.ones((3, 3)), np.ones(2)/np.sqrt(2), pulling=True)


def test_pressing_branch_bounds_only_the_sign():
    # A pressing hand has no magnitude limit inside grip_constraints: the
    # projection sign is enforced, the size is left to joint budgets.
    scale = 300.
    constraints = grip_constraints(scale*np.eye(2), np.ones(2)/np.sqrt(2),
                                   pulling=False)
    result = allocate_effort(np.array([400., 400.])/scale,
        np.zeros((0, 2)), np.zeros(0), np.zeros((0, 2)), np.zeros(0),
        np.full(2, -1000.)/scale, np.full(2, 1000.)/scale,
        extra_constraints=constraints)
    assert result.feasible
    np.testing.assert_allclose(result.solution, np.array([400., 400.])/scale,
                               atol=1e-6)


def test_inverse_dynamics_rows_layout_and_sign():
    mass = np.array([[2., .1], [.1, 3.]])
    actuation = np.array([[1.], [0.]])
    support = np.array([[.5, 0.], [0., .5]])
    bias = np.array([9., -4.])
    known = np.array([1., 2.])
    aeq, beq = inverse_dynamics_rows(mass, actuation, support, bias, known)
    assert aeq.shape == (2, 5)
    np.testing.assert_allclose(aeq[:, :2], mass)
    np.testing.assert_allclose(aeq[:, 2], -actuation[:, 0])
    np.testing.assert_allclose(aeq[:, 3:], -support.T)
    np.testing.assert_allclose(beq, known - bias)


def test_allocation_rejects_bad_problem_data():
    with pytest.raises(ValueError, match='finite'):
        allocate_effort(np.array([np.nan]), np.zeros((0, 1)), np.zeros(0),
            np.zeros((0, 1)), np.zeros(0), np.array([0.]), np.array([1.]))
    with pytest.raises(ValueError, match='reversed'):
        allocate_effort(np.array([0.]), np.zeros((0, 1)), np.zeros(0),
            np.zeros((0, 1)), np.zeros(0), np.array([2.]), np.array([1.]))


def test_nonfinite_constraint_eval_reports_infinite_violation():
    bad = NonlinearConstraint(lambda x: np.array([np.nan]), -np.inf, 1.)
    result = allocate_effort(np.array([0.5]), np.zeros((0, 1)), np.zeros(0),
        np.zeros((0, 1)), np.zeros(0), np.array([0.]), np.array([1.]),
        extra_constraints=[bad])
    assert not result.feasible
    assert result.violation == float('inf')
    assert np.isfinite(result.solution).all()


def test_positive_power_cap_is_a_sum_not_a_net():
    # z = [tau(3), p(3)]; p_j >= tau_j*v_j, p_j >= 0, sum(p) <= 450.
    # Intent requests powers (400, -350, 100) W: the honest bound is 500 W of
    # positive work, so the cap must bite even though the net is only 150 W.
    speeds = np.array([2., -1., 1.])
    n = 6
    i_t, i_p = slice(0, 3), slice(3, 6)
    g_rows = []
    for j in range(3):
        row = np.zeros(n); row[j] = speeds[j]; row[3+j] = -1.
        g_rows.append(row)
    row = np.zeros(n); row[i_p] = 1.
    g_rows.append(row)
    g = np.vstack(g_rows)
    h = np.array([0., 0., 0., 450.])
    target = np.zeros(n)
    target[i_t] = np.array([400., -350., 100.])/speeds
    lo = np.full(n, -np.inf); hi = np.full(n, np.inf); lo[i_p] = 0.
    result = allocate_effort(target, np.zeros((0, n)), np.zeros(0),
        g, h, lo, hi)
    assert result.feasible
    tau = result.solution[i_t]
    positive = float(np.maximum(tau*speeds, 0.).sum())
    assert positive <= 450. + 1e-6
    assert positive < 500. - 1e-6


def test_isometric_joint_does_not_divide_by_zero_speed():
    # v == 0 must not produce a 0/0 power cap: a bounded isometric torque is
    # perfectly feasible.
    speeds = np.array([0., 1.])
    n = 4
    g = np.zeros((3, n))
    g[0, 0] = speeds[0]; g[0, 2] = -1.
    g[1, 1] = speeds[1]; g[1, 3] = -1.
    g[2, 2] = 1.; g[2, 3] = 1.
    h = np.array([0., 0., 450.])
    target = np.array([30., 5., 0., 0.])
    lo = np.array([-50., -50., 0., 0.]); hi = np.array([50., 50., 450., 450.])
    result = allocate_effort(target, np.zeros((0, n)), np.zeros(0),
        g, h, lo, hi)
    assert result.feasible
    assert result.solution[0] == pytest.approx(30., abs=1e-6)
    assert np.isfinite(result.solution).all()


def test_solution_is_deterministic_for_identical_inputs():
    args = (np.array([3., -2.]), np.array([[1., 1.]]), np.array([1.]),
            np.array([[1., 0.]]), np.array([2.]),
            np.array([-5., -5.]), np.array([5., 5.]))
    first = allocate_effort(*args)
    second = allocate_effort(*args)
    assert first.feasible and second.feasible
    np.testing.assert_array_equal(first.solution, second.solution)


def test_candidate_is_preserved_when_infeasible():
    result = allocate_effort(np.array([0.]), np.array([[1.]]), np.array([-1.]),
        np.zeros((0, 1)), np.zeros(0), np.array([20.]), np.array([100.]))
    assert not result.feasible
    assert result.solution.shape == (1,)
    assert np.isfinite(result.solution).all()


# --- adapter: whole-rider balance on the compiled plant ----------------------

def _environment(tmp_path, config=WELDED):
    from bike_sim.cli import research as research_cli
    track = tmp_path / 'track.toml'
    track.write_text('name = "probe"\nlength_m = 30.0\nsurface = "hardpack"\n'
                     'grade_profile = { knots = [[0.0, 0.0], [30.0, 0.0]] }\n')
    args = research_cli.parser().parse_args([
        '--physics-config', str(config), '--track-file', str(track),
        '--duration', '1.', '--energy-tolerance', '1e9',
        '--diagnostic-model-limits', '--initial-speed', '0',
        '--out', str(tmp_path / 'out')])
    return research_cli.make_environment(args)


def _controller(env):
    return env.sim.physical.rider_control


@pytest.mark.slow
def test_healthy_step_allocates_a_feasible_command(tmp_path):
    from bike_sim.sim.ride.control import RideControl
    env = _environment(tmp_path)
    for _ in range(40):
        env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.),
                 front_brake_demand=1., rear_brake_demand=1.)
    physical = env.sim.physical
    diagnostics = _controller(env).allocation_diagnostics
    assert diagnostics['feasible']
    assert not diagnostics['invalid_controller']
    assert 'rider_controller.infeasible' not in physical.step_violations
    channel = physical.sample.channels['rider_allocation']
    assert channel['feasible'] is True
    assert math.isfinite(channel['violation'])


@pytest.mark.slow
def test_allocated_wrenches_close_the_rider_dynamics(tmp_path):
    """The solved z must satisfy M*qddot + bias = B*tau + J.T*f + known.

    Force and moment balance about the saddle are rows of this same equality,
    so its residual is the honest whole-body check rather than a second
    statics add-on. The recorded z lives at the interval start: rewind the
    model there before rebuilding the rows.
    """
    from bike_sim.sim.ride.control import RideControl
    env = _environment(tmp_path)
    for _ in range(30):
        env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.),
                 front_brake_demand=1., rear_brake_demand=1.)
    physical = env.sim.physical
    controller = _controller(env)
    diagnostics = controller.allocation_diagnostics
    assert diagnostics['feasible']
    model, data = env.sim.model, env.sim.data
    sample = physical.sample
    # Rewind to the pre-step interval state where the QP built its rows.
    data.qpos[:] = sample.qpos
    data.qvel[:] = sample.qvel
    import mujoco
    mujoco.mj_forward(model, data)
    qddot = np.asarray(diagnostics['solution_qddot'])
    wrenches = np.asarray(diagnostics['solution_wrenches'])
    qacc_b = np.asarray(diagnostics['solution_base_qacc'])
    rd, bd = controller._rider_dofs, controller._bike_dofs
    attach = tuple(controller._alloc_attachments)
    full = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, data, full)
    bias = np.asarray(data.qfrc_bias[rd]) + full[np.ix_(rd, bd)] @ qacc_b
    wrench_rows, _ = controller._attachment_frames(model, data)
    jac = np.vstack([wrench_rows[n][:, rd] for n in attach])
    tau = np.asarray(diagnostics['solution_tau'])
    known = np.zeros(model.nv)
    envelope = controller.envelope_forces(model, data)[0]
    known[controller._envelope_dof_adrs] += envelope[controller._envelope_dof_adrs]
    hinge = np.asarray([controller.joints[n][1] for n in controller.joints])
    known[hinge] -= controller.config.joint_kd_nms_rad*data.qvel[hinge]
    b_torque = np.zeros(model.nv)
    for index, name in enumerate(controller.joints):
        b_torque[controller.joints[name][1]] += tau[index]
    residual = (full[np.ix_(rd, rd)] @ qddot + bias
                - b_torque[rd] - jac.T @ wrenches - known[rd])
    np.testing.assert_allclose(residual, np.zeros(len(rd)), atol=1e-4)


@pytest.mark.slow
def test_command_does_not_read_solved_reactions(tmp_path):
    """Same q/v, perturbed efc_force: the command must not change.

    Solved lambdas are diagnostics; the allocator must derive wrenches from
    its own model, never from the solver's answer.
    """
    from bike_sim.sim.ride.control import RideControl
    from bike_sim.sim.ride.rider_control import RiderCommand
    env = _environment(tmp_path)
    for _ in range(20):
        env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.),
                 front_brake_demand=1., rear_brake_demand=1.)
    physical = env.sim.physical
    controller = _controller(env)
    model, data = env.sim.model, env.sim.data
    command = RiderCommand(0., enabled=True,
                           posture=physical.applied_control.posture,
                           crank_target_phase_rad=None)
    state = physical.rider_state
    first = controller.compute(model, data, command, kinematic_state=state,
                               support_available=None, advance=False)
    perturbed = np.random.default_rng(7).normal(size=data.nefc)
    data.efc_force[:data.nefc] += perturbed
    second = controller.compute(model, data, command, kinematic_state=state,
                                support_available=None, advance=False)
    for name in first:
        assert first[name] == pytest.approx(second[name], abs=1e-9)


@pytest.mark.slow
def test_infeasible_intent_keeps_the_physical_bounds(tmp_path):
    """An absurd torque intent is relaxed, never exceeds strength bounds."""
    from bike_sim.sim.ride.control import RideControl
    env = _environment(tmp_path)
    for _ in range(20):
        env.step(RideControl(motor_torque_nm=0., human_torque_nm=0.),
                 front_brake_demand=1., rear_brake_demand=1.)
    controller = _controller(env)
    model, data = env.sim.model, env.sim.data
    n_t = len(controller.joints)
    names = tuple(controller.joints)
    speed = np.asarray([data.qvel[controller.joints[n][1]] for n in names])
    angle = np.asarray([controller.anatomical_joint_angle(
        n, data.qpos[controller.joints[n][0]]) for n in names])
    tau, diagnostics = controller.allocate(
        model, data, np.full(n_t, 1e4),
        np.zeros(2*len(controller._alloc_attachments)), {},
        estimates=None)
    assert diagnostics['feasible']
    for index, name in enumerate(names):
        cap = (controller.config.joint_limit_nm if controller.strength is None
               else max(
                   controller.strength_capacity(name, angle[index], speed[index], 1.),
                   controller.strength_capacity(name, angle[index], speed[index], -1.)))
        assert abs(tau[index]) <= cap + 1e-6
