"""Bounded mechanical-mode search, with actual MuJoCo response oracles."""
import mujoco
import numpy as np
import pytest

from bike_sim.physics.rider_allocation import Allocation, allocate_effort
from bike_sim.sim.ride.rider_dynamics import ConstraintDynamics, search_constraint_modes


def _stand(kind):
    friction = 'frictionloss="2"' if kind == 'friction' else ''
    tendon = ('' if kind == 'friction' else
              '<tendon><fixed limited="true" range="-10 0">'
              '<joint joint="a" coef="1"/><joint joint="b" coef="-1"/>'
              '</fixed></tendon>')
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><option gravity="0 0 0" timestep=".0005" tolerance="1e-12"/>'
        '<worldbody>'
        f'<body><joint name="a" type="slide" axis="1 0 0" {friction}/>'
        '<inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/></body>'
        '<body><joint name="b" type="slide" axis="1 0 0"/>'
        '<inertial mass="2" pos="0 0 0" diaginertia="1 1 1"/></body>'
        '</worldbody>' + tendon + '</mujoco>')
    data = mujoco.MjData(model)
    if kind == 'tendon':
        data.qpos[0] = 1e-5
    mujoco.mj_forward(model, data)
    return model, data, ConstraintDynamics(model, data, [0])


@pytest.mark.parametrize('kind,wish,boundary', [
    ('tendon', 20., -20.),  # Loaded clutch must release.
    ('tendon', -20., 20.),  # Released clutch must engage.
    ('friction', 20., 1.),  # Sliding in either direction must be able to stick.
    ('friction', -20., -1.),
    ('friction', 20., -20.),  # The feasible region may be across two boundaries.
    ('friction', -20., 20.),
])
def test_command_crosses_mode_boundaries_and_matches_current_forward(kind, wish, boundary):
    model, data, dynamics = _stand(kind)
    target = np.array([wish])
    direction = 1. if wish > boundary else -1.

    def solve(response):
        matrix = np.vstack((response.domain_matrix, [[direction]]))
        bound = np.r_[response.domain_bound, direction * boundary]
        result = allocate_effort(target, np.zeros((0, 1)), np.zeros(0),
                                 matrix, bound, np.array([-50.]), np.array([50.]))
        return result, response.mode

    assert not solve(dynamics.linearize(target))[0].feasible
    result, payload, response, count, complete = search_constraint_modes(
        dynamics, target, solve)
    assert result.feasible and complete and count > 1
    assert payload == response.mode
    assert result.solution[0] == pytest.approx(boundary, abs=1e-7)
    assert np.all(response.domain_matrix @ result.solution <= response.domain_bound + 1e-7)
    data.qfrc_applied[0] = result.solution[0]
    mujoco.mj_forward(model, data)
    np.testing.assert_allclose(
        response.acceleration_offset + response.acceleration_matrix @ result.solution,
        data.qacc, rtol=1e-10, atol=1e-9)
    np.testing.assert_allclose(
        response.force_offset + response.force_matrix @ result.solution,
        data.efc_force[:data.nefc], rtol=1e-10, atol=1e-9)


@pytest.mark.parametrize('kind,mode_count', [('tendon', 2), ('friction', 3)])
@pytest.mark.parametrize('truncate', [False, True])
def test_exact_mode_budget_distinguishes_exhaustion_from_truncation(kind, mode_count, truncate):
    _, _, dynamics = _stand(kind)
    wish = np.array([20.])
    budget = mode_count - 1 if truncate else mode_count
    calls = []
    results = []

    def solve(response):
        calls.append(response.mode)
        # Exercise the real failed-iterate transition: the zero/negative command
        # points to a neighbor that will also be returned by neighboring_modes.
        command = np.array([-20. if kind == 'tendon' else 0.])
        violation = (2., .5, 1.)[len(calls) - 1]
        result = Allocation(command, False, violation)
        results.append(result)
        return result, {'mode': response.mode, 'call': len(calls)}

    result, payload, response, count, complete = search_constraint_modes(
        dynamics, wish, solve, max_candidates=budget)
    assert count == len(calls) == len(set(calls)) == budget
    assert complete is (not truncate)
    best = min(range(len(results)), key=lambda i: results[i].violation)
    assert result is results[best] and not result.feasible
    assert payload == {'mode': calls[best], 'call': best + 1}
    assert response.mode == calls[best]
    np.testing.assert_array_equal(wish, [20.])


def test_feasible_first_mode_stops_without_exploring_neighbors(monkeypatch):
    _, _, dynamics = _stand('tendon')
    wish = np.array([20.])
    calls = []

    def no_neighbors(mode):
        pytest.fail('a feasible command must stop the mode search immediately')

    def solve(response):
        calls.append(response.mode)
        return Allocation(wish.copy(), True, 0.), 'feasible'

    monkeypatch.setattr(dynamics, 'neighboring_modes', no_neighbors)
    result, payload, response, count, complete = search_constraint_modes(
        dynamics, wish, solve, max_candidates=1)
    assert result.feasible and payload == 'feasible'
    assert calls == [response.mode] and count == 1 and complete


@pytest.mark.parametrize('budget', [0, -1, 1.5, True, None])
def test_invalid_mode_budget_fails_before_evaluating_dynamics(budget):
    with pytest.raises(ValueError, match='positive integer'):
        search_constraint_modes(None, np.array([0.]), None, max_candidates=budget)


@pytest.mark.parametrize('pulling', [False, True])
def test_guarded_grip_branch_preserves_sign_and_circle_budgets(pulling):
    from bike_sim.sim.ride.rider_response_allocation import (
        allocation_grip_constraints, GRIP_RADIUS_GUARD_FRACTION, SUPPORT_FORCE_GUARD_N)

    constraints = allocation_grip_constraints(np.eye(2), np.array([1., 0.]),
                                              pulling=pulling, limit_n=300.)
    sign = constraints[0]
    np.testing.assert_array_equal(sign.A, [[-1. if pulling else 1., 0.]])
    np.testing.assert_array_equal(sign.lb, [0. if pulling else SUPPORT_FORCE_GUARD_N])
    np.testing.assert_array_equal(sign.ub, [np.inf])
    if pulling:
        assert len(constraints) == 2
        radius = 300. * (1. - GRIP_RADIUS_GUARD_FRACTION)
        assert constraints[1].fun(np.array([-radius, 0.])) == pytest.approx(1.)
        assert constraints[1].fun(np.array([-300., 0.])) > 1.
    else:
        assert len(constraints) == 1  # Press magnitude is still a joint budget.
        assert np.all(sign.A @ np.array([400., 400.]) >= sign.lb)
        assert np.any(sign.A @ np.array([-1., 0.]) < sign.lb)


def test_guarded_grip_search_falls_back_from_remembered_pull_to_press():
    from bike_sim.sim.ride.rider_control import ArticulatedRiderController
    from bike_sim.sim.ride.rider_response_allocation import allocation_grip_constraints

    controller = ArticulatedRiderController.__new__(ArticulatedRiderController)
    controller._last_branch = (True, True)
    controller._last_solution = np.array([-100., -100.])
    target = np.array([100., 100.])
    calls = []

    def solve(branch, x0):
        calls.append(branch)
        constraints = []
        for index, pulling in enumerate(branch):
            force_map = np.zeros((2, 2))
            force_map[0, index] = 1.
            constraints.extend(allocation_grip_constraints(
                force_map, np.array([1., 0.]), pulling=pulling, limit_n=300.))
        return allocate_effort(
            target, np.eye(2), target, np.zeros((0, 2)), np.zeros(0),
            np.full(2, -1000.), np.full(2, 1000.), extra_constraints=constraints, x0=x0)

    result, branch = controller._select_allocation_branch(target, solve)
    assert result.feasible and branch == (False, False)
    assert calls[0] == (True, True) and len(calls) == len(set(calls)) == 4
    np.testing.assert_allclose(result.solution, target, atol=1e-7)
